//! Plane-flight metrics of waypoint laps, from recorder CSVs (`--record` with
//! `TailsitterColumns` or `QuadPlaneColumns`), as a markdown table to paste on an issue.
//!
//! Per CSV, over the rows in plane mode (`mode` 1):
//!
//! - **Plane time:** first plane row to last, in s.
//! - **Peak sink:** the fastest mean vertical speed (`climb_mps`) over a trailing 0.5 s window,
//!   in m/s, down positive (compare against guidance's `PGUID_SINK_MAX`, which limits the same
//!   speed). Altitude differenced over the recorder's wall clock reads high when the simulator's
//!   clock stalls and catches up (X-Plane, #700). A non-finite `climb_mps` row (a blank cell)
//!   only lowers the windows over it. TODO(#701): on X-Plane, `climb_mps` and the altitude
//!   rate disagree by up to 0.7 m/s over seconds, so this can read under the altitude's sink.
//! - **Handoff dip:** how far the airframe sags after the handoff into plane, in m: the handoff
//!   altitude minus the lowest point in the next 30 s, when altitude climbs back to the
//!   handoff's after it. None ("—") when it doesn't, as when the first plane leg descends on
//!   purpose.
//! - **Roll / pitch RMS:** tracking error of the attitude setpoint, in degrees, over rows with
//!   one.
//! - **Peak bank:** the largest |roll|, in degrees.
//! - **Touchdown:** horizontal distance from the first row to the last armed row, in m.
//!
//! ```text
//! cargo run --release --bin lap_stats -- lap-branch.csv lap-main.csv
//! ```

use std::path::{Path, PathBuf};
use std::process;

use clap::Parser;
use thiserror::Error;

use stable_flight_tools::recording::{PLANE_MODE, Recording, RecordingError};

#[cfg(test)]
mod tests;

/// Peak sink's trailing window, in s.
const SINK_WINDOW_S: f32 = 0.5;
/// Shortest span a sink is measured over, in s: shorter ones (a gap in the plane rows, the
/// window's first rows) aren't a 0.5 s mean.
const MIN_SINK_SPAN_S: f32 = 0.4;
/// How long after the handoff the airframe has to climb back for its dip to count, in s.
const DIP_WINDOW_S: f32 = 30.0;

/// Plane-flight metrics of waypoint laps, from recorder CSVs.
#[derive(Parser)]
struct Args {
    /// Recorder CSVs, one table row each.
    #[arg(required = true)]
    csvs: Vec<PathBuf>,
}

/// Why a CSV has no lap metrics.
#[derive(Debug, Error)]
enum LapError {
    #[error("{0}")]
    Read(#[from] std::io::Error),
    #[error(transparent)]
    Recording(#[from] RecordingError),
    #[error("no plane flight")]
    NoPlaneFlight,
    #[error("never armed")]
    NeverArmed,
}

/// One lap's metrics; see the module docs.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LapStats {
    plane_s: f32,
    peak_sink_mps: f32,
    handoff_dip_m: Option<f32>,
    roll_rms_deg: f32,
    pitch_rms_deg: f32,
    peak_bank_deg: f32,
    touchdown_m: f32,
}

fn main() {
    let args = Args::parse();
    println!("{TABLE_HEADER}");
    let mut failed = false;
    for path in &args.csvs {
        match read(path) {
            Ok(stats) => println!("{}", table_row(&lap_name(path), &stats)),
            Err(error) => {
                eprintln!("{}: {error}", path.display());
                failed = true;
            }
        }
    }
    if failed {
        process::exit(1);
    }
}

fn read(path: &Path) -> Result<LapStats, LapError> {
    let recording = Recording::parse(&std::fs::read_to_string(path)?)?;
    LapStats::from_recording(&recording)
}

/// The file's stem, or its path when it has none.
fn lap_name(path: &Path) -> String {
    path.file_stem().map_or_else(
        || path.display().to_string(),
        |stem| stem.to_string_lossy().into_owned(),
    )
}

/// The markdown table's header and separator, one column per [`table_row`] cell.
const TABLE_HEADER: &str =
    "| Lap | Plane | Peak sink in plane | Handoff dip | Roll / pitch RMS | Peak bank | Touchdown |
|---|---|---|---|---|---|---|";

/// One markdown table row, in [`TABLE_HEADER`]'s order.
fn table_row(name: &str, stats: &LapStats) -> String {
    let dip = stats
        .handoff_dip_m
        .map_or_else(|| "—".to_owned(), |dip_m| format!("{dip_m:.1} m"));
    format!(
        "| {name} | {:.0} s | {:.2} m/s | {dip} | {:.2}° / {:.2}° | {:.1}° | {:.2} m |",
        stats.plane_s,
        stats.peak_sink_mps,
        stats.roll_rms_deg,
        stats.pitch_rms_deg,
        stats.peak_bank_deg,
        stats.touchdown_m
    )
}

/// The columns the metrics read, one row's worth.
#[derive(Debug, Clone, Copy)]
struct Sample {
    time_s: f32,
    armed: bool,
    plane: bool,
    altitude_m: f32,
    /// Vertical speed, up positive, in m/s.
    climb_mps: f32,
    roll_rad: f32,
    pitch_rad: f32,
    sp_roll_rad: f32,
    sp_pitch_rad: f32,
    north_m: f32,
    east_m: f32,
}

impl LapStats {
    fn from_recording(recording: &Recording) -> Result<Self, LapError> {
        let samples = samples(recording)?;
        let plane: Vec<Sample> = samples.iter().copied().filter(|s| s.plane).collect();
        let (Some(handoff), Some(last_plane)) = (plane.first(), plane.last()) else {
            return Err(LapError::NoPlaneFlight);
        };
        let start = samples.first().ok_or(LapError::NoPlaneFlight)?;
        let touchdown = samples
            .iter()
            .rev()
            .find(|s| s.armed)
            .ok_or(LapError::NeverArmed)?;
        let tracked: Vec<&Sample> = plane
            .iter()
            .filter(|s| s.sp_roll_rad.is_finite() && s.sp_pitch_rad.is_finite())
            .collect();
        Ok(Self {
            plane_s: last_plane.time_s - handoff.time_s,
            peak_sink_mps: peak_sink_mps(&plane),
            handoff_dip_m: handoff_dip_m(&plane),
            roll_rms_deg: rms_deg(tracked.iter().map(|s| s.sp_roll_rad - s.roll_rad)),
            pitch_rms_deg: rms_deg(tracked.iter().map(|s| s.sp_pitch_rad - s.pitch_rad)),
            peak_bank_deg: plane
                .iter()
                .map(|s| s.roll_rad.abs())
                .fold(0.0, f32::max)
                .to_degrees(),
            touchdown_m: (touchdown.north_m - start.north_m).hypot(touchdown.east_m - start.east_m),
        })
    }
}

/// Each row's metric columns, in time order.
fn samples(recording: &Recording) -> Result<Vec<Sample>, LapError> {
    let column = |name| recording.require(name);
    let armed = column("armed")?;
    let mode = column("mode")?;
    let altitude = column("altitude")?;
    let climb = column("climb_mps")?;
    let (roll, pitch) = (column("roll")?, column("pitch")?);
    let (sp_roll, sp_pitch) = (column("sp_roll")?, column("sp_pitch")?);
    let (north, east) = (column("north_m")?, column("east_m")?);
    Ok(recording
        .rows()
        .iter()
        .map(|row| Sample {
            time_s: row[0],
            armed: row[armed] == 1.0,
            plane: row[mode] == PLANE_MODE,
            altitude_m: row[altitude],
            climb_mps: row[climb],
            roll_rad: row[roll],
            pitch_rad: row[pitch],
            sp_roll_rad: row[sp_roll],
            sp_pitch_rad: row[sp_pitch],
            north_m: row[north],
            east_m: row[east],
        })
        .collect())
}

/// The fastest mean sink over a trailing [`SINK_WINDOW_S`], down positive, in m/s; 0 when it
/// never sinks. The mean is the vertical speed integrated over the window's rows (trapezoids),
/// divided by its span. A segment with a non-finite vertical speed adds nothing, so a window
/// over it reads low, never high, and the rest of the lap still counts.
fn peak_sink_mps(plane: &[Sample]) -> f32 {
    // Climb integrated from the first row to each row, in m, up positive.
    let mut climbed_m = vec![0.0_f32; plane.len()];
    for (i, pair) in plane.windows(2).enumerate() {
        let mean_climb_mps = 0.5 * (pair[0].climb_mps + pair[1].climb_mps);
        let segment_m = mean_climb_mps * (pair[1].time_s - pair[0].time_s);
        climbed_m[i + 1] = climbed_m[i]
            + if segment_m.is_finite() {
                segment_m
            } else {
                0.0
            };
    }
    let mut peak_mps = 0.0_f32;
    let mut from = 0;
    for (i, now) in plane.iter().enumerate() {
        while plane[from].time_s < now.time_s - SINK_WINDOW_S {
            from += 1;
        }
        let span_s = now.time_s - plane[from].time_s;
        if span_s > MIN_SINK_SPAN_S {
            peak_mps = peak_mps.max((climbed_m[from] - climbed_m[i]) / span_s);
        }
    }
    peak_mps
}

/// The handoff altitude minus the lowest point within [`DIP_WINDOW_S`] of the handoff, in m,
/// when altitude climbs back to the handoff's after that point within the window; `None` when
/// it doesn't (a descent, not a dip).
fn handoff_dip_m(plane: &[Sample]) -> Option<f32> {
    let handoff = plane.first()?;
    let window: Vec<&Sample> = plane
        .iter()
        .take_while(|s| s.time_s <= handoff.time_s + DIP_WINDOW_S)
        .collect();
    let (lowest_at, lowest) = window
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.altitude_m.total_cmp(&b.altitude_m))?;
    window[lowest_at..]
        .iter()
        .any(|s| s.altitude_m >= handoff.altitude_m)
        .then_some(handoff.altitude_m - lowest.altitude_m)
}

/// Root mean square of angle errors in rad, in degrees; 0 without any.
fn rms_deg(errors_rad: impl Iterator<Item = f32>) -> f32 {
    let (sum_sq, count) = errors_rad.fold((0.0_f64, 0_u32), |(sum, n), error| {
        (sum + f64::from(error) * f64::from(error), n + 1)
    });
    if count == 0 {
        return 0.0;
    }
    // f64 sums keep a half-hour lap's 10⁵ rows from rounding away the small errors.
    ((sum_sq / f64::from(count)).sqrt() as f32).to_degrees()
}
