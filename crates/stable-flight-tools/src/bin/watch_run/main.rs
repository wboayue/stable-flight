//! Watchdog for a simulator run: tails the example's recorder CSV (`--record`) and SIGTERMs it
//! when a limit trips, printing why.
//!
//! Heights and distances are from the first recorded row. The limits, each off unless given:
//!
//! - `--floor-m`: plane flight (`mode` 1) below this height; never without a `mode` column
//!   (`CopterColumns`).
//! - `--radius-m`: horizontally further than this from the start.
//! - `--timeout-s`: this long since `watch_run` started.
//!
//! Disarming (an armed row, then a disarmed one: the landing) always stops the run, after
//! `--disarm-grace-s` for the example to log its summary. SIGTERM, never a kill, lets the
//! example hand control back to the simulator. If the example exits first, `watch_run` reports
//! how long it ran and the highest point.
//!
//! ```text
//! RUST_LOG=info ./target/release/examples/<example> ... --record run.csv > run.log 2>&1 &
//! cargo run --release --bin watch_run -- --pid $! --csv run.csv --floor-m 100 --radius-m 9000 --timeout-s 1800 > run.watch &
//! ```

use std::fs::File;
use std::io::{self, Read};
use std::path::PathBuf;
use std::process::{self, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use thiserror::Error;

use stable_flight_tools::recording::{Header, PLANE_MODE, RecordingError};

#[cfg(test)]
mod tests;

/// How often the example and its CSV are checked.
const POLL: Duration = Duration::from_millis(500);

/// Stop a simulator run on a floor, radius, timeout, or disarm.
#[derive(Parser)]
struct Args {
    /// The example's process id (`$!` after launching it in the background).
    #[arg(long)]
    pid: u32,
    /// The example's recorder CSV (its `--record` path); it may not exist yet.
    #[arg(long)]
    csv: PathBuf,
    #[command(flatten)]
    limits: Limits,
    /// Wait after disarming before stopping, in s, for the example's exit summary.
    #[arg(long, default_value_t = 2.0)]
    disarm_grace_s: f32,
}

/// The limits that stop a run, each off when `None`.
#[derive(Debug, Clone, Copy, Default, clap::Args)]
struct Limits {
    /// Lowest height in plane flight, in m above the first row.
    #[arg(long)]
    floor_m: Option<f32>,
    /// Furthest horizontal distance from the first row, in m.
    #[arg(long)]
    radius_m: Option<f32>,
    /// Longest run, in s from `watch_run`'s start.
    #[arg(long)]
    timeout_s: Option<f32>,
}

/// Why a run was stopped.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Stop {
    Floor { height_m: f32, floor_m: f32 },
    Radius { distance_m: f32, radius_m: f32 },
    Timeout { timeout_s: f32 },
    Disarmed { distance_m: f32 },
}

impl std::fmt::Display for Stop {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Floor { height_m, floor_m } => {
                write!(f, "plane at {height_m:.1} m, under the {floor_m} m floor")
            }
            Self::Radius {
                distance_m,
                radius_m,
            } => write!(f, "{distance_m:.0} m from the start, past {radius_m} m"),
            Self::Timeout { timeout_s } => write!(f, "timeout {timeout_s} s"),
            Self::Disarmed { distance_m } => {
                write!(f, "disarmed {distance_m:.2} m from the start")
            }
        }
    }
}

/// How a watched run ended.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Outcome {
    /// Stopped with SIGTERM.
    Stopped(Stop),
    /// The example exited on its own.
    Exited { after_s: f32, max_height_m: f32 },
}

/// Why a run couldn't be watched.
#[derive(Debug, Error)]
enum WatchError {
    #[error("{0}")]
    Io(#[from] io::Error),
    #[error(transparent)]
    Recording(#[from] RecordingError),
    #[error("the example exited before recording")]
    ExitedBeforeRecording,
    #[error("`kill` failed to signal pid {0}")]
    Signal(u32),
}

fn main() {
    let args = Args::parse();
    match watch(&args, POLL) {
        Ok(Outcome::Stopped(stop)) => println!("watch_run: {stop}: SIGTERM"),
        Ok(Outcome::Exited {
            after_s,
            max_height_m,
        }) => println!(
            "watch_run: example exited after {after_s:.0} s (max height {max_height_m:.1} m)"
        ),
        Err(error) => {
            eprintln!("watch_run: {error}");
            process::exit(1);
        }
    }
}

/// Watches `args.pid`'s CSV every `poll` until a limit trips (then SIGTERMs it) or it exits.
fn watch(args: &Args, poll: Duration) -> Result<Outcome, WatchError> {
    let start = Instant::now();
    let elapsed_s = || start.elapsed().as_secs_f32();
    while !args.csv.exists() {
        if !alive(args.pid) {
            return Err(WatchError::ExitedBeforeRecording);
        }
        if let Some(stop) = args.limits.timed_out(elapsed_s()) {
            return stop_run(args, stop);
        }
        thread::sleep(poll);
    }
    let mut file = File::open(&args.csv)?;
    let mut lines = PartialLines::default();
    let mut reader: Option<(Header, Watchdog)> = None;
    let mut chunk = Vec::new();
    loop {
        if !alive(args.pid) {
            return Ok(Outcome::Exited {
                after_s: elapsed_s(),
                max_height_m: reader.map_or(0.0, |(_, watchdog)| watchdog.max_height_m),
            });
        }
        if let Some(stop) = args.limits.timed_out(elapsed_s()) {
            return stop_run(args, stop);
        }
        chunk.clear();
        file.read_to_end(&mut chunk)?;
        for line in lines.feed(&chunk) {
            let Some((header, watchdog)) = &mut reader else {
                let header = Header::parse(&line)?;
                let watchdog = Watchdog::new(args.limits, &header)?;
                reader = Some((header, watchdog));
                continue;
            };
            // A short row is the last of a stopped recorder: nothing to check.
            let Some(row) = header.row(&line) else {
                continue;
            };
            if let Some(stop) = watchdog.observe(&row) {
                return stop_run(args, stop);
            }
        }
        thread::sleep(poll);
    }
}

/// SIGTERMs the example for `stop`, after the disarm grace when it landed.
fn stop_run(args: &Args, stop: Stop) -> Result<Outcome, WatchError> {
    if let Stop::Disarmed { .. } = stop {
        thread::sleep(Duration::from_secs_f32(args.disarm_grace_s.max(0.0)));
    }
    if !kill(&["-TERM", &args.pid.to_string()]) {
        return Err(WatchError::Signal(args.pid));
    }
    Ok(Outcome::Stopped(stop))
}

/// Whether `pid` is a running process (`kill -0`).
fn alive(pid: u32) -> bool {
    kill(&["-0", &pid.to_string()])
}

/// Runs `kill` with `args`; whether it succeeded. The shell's `kill` needs no `unsafe` and no
/// platform crate, and every simulator run is on Unix.
fn kill(args: &[&str]) -> bool {
    Command::new("kill")
        .args(args)
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|status| status.success())
}

impl Limits {
    /// [`Stop::Timeout`] once `elapsed_s` passes the timeout.
    fn timed_out(&self, elapsed_s: f32) -> Option<Stop> {
        self.timeout_s
            .filter(|&timeout_s| elapsed_s > timeout_s)
            .map(|timeout_s| Stop::Timeout { timeout_s })
    }
}

/// Splits a growing file's chunks into whole lines, holding a partial last line (the recorder
/// flushes mid-row) until the rest arrives.
#[derive(Debug, Default)]
struct PartialLines {
    pending: Vec<u8>,
}

impl PartialLines {
    /// The lines `chunk` completes, without their line ends.
    fn feed(&mut self, chunk: &[u8]) -> Vec<String> {
        self.pending.extend_from_slice(chunk);
        let Some(last_end) = self.pending.iter().rposition(|&b| b == b'\n') else {
            return Vec::new();
        };
        let rest = self.pending.split_off(last_end + 1);
        let complete = std::mem::replace(&mut self.pending, rest);
        String::from_utf8_lossy(&complete)
            .lines()
            .map(str::to_owned)
            .collect()
    }
}

/// Checks each recorded row against the row limits: floor, radius, and disarm.
#[derive(Debug, Clone)]
struct Watchdog {
    limits: Limits,
    columns: ColumnIndices,
    /// The first row's north, east, and altitude, in m.
    origin: Option<(f32, f32, f32)>,
    was_armed: bool,
    max_height_m: f32,
}

/// The column indices a [`Watchdog`] reads.
#[derive(Debug, Clone, Copy)]
struct ColumnIndices {
    armed: usize,
    mode: Option<usize>,
    altitude: usize,
    north: usize,
    east: usize,
}

impl Watchdog {
    fn new(limits: Limits, header: &Header) -> Result<Self, WatchError> {
        let column = |name| header.require(name);
        let columns = ColumnIndices {
            armed: column("armed")?,
            mode: header.column("mode"),
            altitude: column("altitude")?,
            north: column("north_m")?,
            east: column("east_m")?,
        };
        Ok(Self {
            limits,
            columns,
            origin: None,
            was_armed: false,
            max_height_m: 0.0,
        })
    }

    /// The limit `row` (a data row under the header given to [`new`](Self::new)) trips, if any.
    fn observe(&mut self, row: &[f32]) -> Option<Stop> {
        let c = self.columns;
        let (north_m, east_m, altitude_m) = (row[c.north], row[c.east], row[c.altitude]);
        let (origin_north_m, origin_east_m, origin_altitude_m) =
            *self.origin.get_or_insert((north_m, east_m, altitude_m));
        let height_m = altitude_m - origin_altitude_m;
        let distance_m = (north_m - origin_north_m).hypot(east_m - origin_east_m);
        self.max_height_m = self.max_height_m.max(height_m);

        let plane = c.mode.is_some_and(|mode| row[mode] == PLANE_MODE);
        if let Some(floor_m) = self.limits.floor_m.filter(|&f| plane && height_m < f) {
            return Some(Stop::Floor { height_m, floor_m });
        }
        if let Some(radius_m) = self.limits.radius_m.filter(|&r| distance_m > r) {
            return Some(Stop::Radius {
                distance_m,
                radius_m,
            });
        }
        let armed = row[c.armed] == 1.0;
        if self.was_armed && !armed {
            return Some(Stop::Disarmed { distance_m });
        }
        self.was_armed |= armed;
        None
    }
}
