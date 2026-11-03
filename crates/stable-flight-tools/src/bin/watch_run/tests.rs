use std::os::unix::process::ExitStatusExt;
use std::path::Path;

use assert2::check;

use super::*;

const HEADER: &str = "wall_s,armed,mode,altitude,north_m,east_m";

/// Armed on the ground at 10 m altitude, then a climb to 200 m above it in copter mode.
const CLIMB: [&str; 2] = ["0.0,1,0,10,0,0", "1.0,1,0,210,0,0"];

const ALL_LIMITS: Limits = Limits {
    floor_m: Some(100.0),
    radius_m: Some(499.0),
    timeout_s: None,
};

/// A row-limit case: name, limits, rows after the climb, expected stop.
type RowCase = (&'static str, Limits, &'static [&'static str], Option<Stop>);

/// The first stop `rows` trip after [`CLIMB`], read under [`HEADER`] with `limits`.
fn first_stop(limits: Limits, rows: &[&str]) -> Option<Stop> {
    let header = Header::parse(HEADER).expect("a header");
    let mut watchdog = Watchdog::new(limits, &header).expect("a watchdog");
    CLIMB
        .iter()
        .chain(rows)
        .find_map(|line| watchdog.observe(&header.row(line).expect("a whole row")))
}

/// The floor holds only in plane flight; the radius from the
/// start; disarming only after an armed row; a limit not given never trips.
#[test]
fn stops_on_each_row_limit() {
    let rows: [RowCase; 7] = [
        (
            "plane under the floor",
            ALL_LIMITS,
            &["2.0,1,1,100,0,0"],
            Some(Stop::Floor {
                height_m: 90.0,
                floor_m: 100.0,
            }),
        ),
        (
            "copter under the floor",
            ALL_LIMITS,
            &["2.0,1,0,100,0,0"],
            None,
        ),
        (
            "transition under the floor",
            ALL_LIMITS,
            &["2.0,1,2,100,0,0"],
            None,
        ),
        (
            "no floor given",
            Limits::default(),
            &["2.0,1,1,100,0,0"],
            None,
        ),
        (
            "past the radius",
            ALL_LIMITS,
            &["2.0,1,0,210,300,400"],
            Some(Stop::Radius {
                distance_m: 500.0,
                radius_m: 499.0,
            }),
        ),
        (
            "no radius given",
            Limits::default(),
            &["2.0,1,0,210,300,400"],
            None,
        ),
        (
            "disarmed after landing",
            Limits::default(),
            &["2.0,1,0,10,3,4", "3.0,0,0,10,3,4"],
            Some(Stop::Disarmed { distance_m: 5.0 }),
        ),
    ];
    for (name, limits, rows, expected) in rows {
        check!(first_stop(limits, rows) == expected, "{name}");
    }
}

/// Without a mode column (`CopterColumns`) there's no plane flight to hold above the floor.
#[test]
fn skips_the_floor_without_a_mode_column() {
    let header = Header::parse("wall_s,armed,altitude,north_m,east_m").expect("a header");
    let mut watchdog = Watchdog::new(ALL_LIMITS, &header).expect("a watchdog");
    for line in ["0.0,1,10,0,0", "1.0,1,210,0,0", "2.0,1,100,0,0"] {
        check!(
            watchdog
                .observe(&header.row(line).expect("a whole row"))
                .is_none()
        );
    }
}

/// A recording that starts disarmed (before arming) never trips the disarm stop.
#[test]
fn waits_for_arming_before_watching_for_a_disarm() {
    let header = Header::parse(HEADER).expect("a header");
    let mut watchdog = Watchdog::new(Limits::default(), &header).expect("a watchdog");
    for line in ["0.0,0,0,10,0,0", "0.1,0,0,10,0,0"] {
        check!(
            watchdog
                .observe(&header.row(line).expect("a whole row"))
                .is_none()
        );
    }
}

/// A recording without the columns the limits read isn't watchable.
#[test]
fn needs_position_and_arming_columns() {
    let header = Header::parse("wall_s,armed,altitude").expect("a header");
    check!(let Err(WatchError::Recording(RecordingError::MissingColumn("north_m"))) = Watchdog::new(ALL_LIMITS, &header));
}

/// The timeout trips once the elapsed time passes it, and never without one.
#[test]
fn times_out() {
    let limits = Limits {
        timeout_s: Some(10.0),
        ..Limits::default()
    };
    check!(limits.timed_out(9.9).is_none());
    check!(limits.timed_out(10.1) == Some(Stop::Timeout { timeout_s: 10.0 }));
    check!(Limits::default().timed_out(1e6).is_none());
}

/// A row split across reads comes out once, whole, when its line end arrives.
#[test]
fn joins_rows_split_across_reads() {
    let mut lines = PartialLines::default();
    check!(lines.feed(b"wall_s,armed\n0.0,") == ["wall_s,armed"]);
    check!(lines.feed(b"1").is_empty());
    check!(lines.feed(b"\n0.1,1\n0.2") == ["0.0,1", "0.1,1"]);
    check!(lines.feed(b",0\n") == ["0.2,0"]);
}

/// A stop prints what tripped and where.
#[test]
fn says_why_it_stopped() {
    check!(
        Stop::Floor {
            height_m: 87.25,
            floor_m: 100.0
        }
        .to_string()
            == "plane at 87.2 m, under the 100 m floor"
    );
    check!(Stop::Disarmed { distance_m: 0.164 }.to_string() == "disarmed 0.16 m from the start");
}

/// A temporary directory per test, removed after.
fn temp_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("watch_run_{name}_{}", process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

fn args(pid: u32, csv: &Path, limits: Limits) -> Args {
    Args {
        pid,
        csv: csv.to_path_buf(),
        limits,
        disarm_grace_s: 0.0,
    }
}

/// Each trigger SIGTERMs a running process: a canned recording per row limit, and a timeout
/// on a recording that trips nothing.
#[test]
fn signals_the_run_on_each_trigger() {
    let dir = temp_dir("triggers");
    let timeout = Limits {
        timeout_s: Some(0.05),
        ..Limits::default()
    };
    // (name, rows after the climb, limits, expected stop)
    let rows: [(&str, &str, Limits, Stop); 4] = [
        (
            "floor",
            "2.0,1,1,100,0,0\n",
            ALL_LIMITS,
            Stop::Floor {
                height_m: 90.0,
                floor_m: 100.0,
            },
        ),
        (
            "radius",
            "2.0,1,0,210,300,400\n",
            ALL_LIMITS,
            Stop::Radius {
                distance_m: 500.0,
                radius_m: 499.0,
            },
        ),
        (
            "disarm",
            "2.0,1,0,10,0,0\n3.0,0,0,10,0,0\n",
            ALL_LIMITS,
            Stop::Disarmed { distance_m: 0.0 },
        ),
        (
            "timeout",
            "2.0,1,0,210,0,0\n",
            timeout,
            Stop::Timeout { timeout_s: 0.05 },
        ),
    ];
    for (name, rows, limits, expected) in rows {
        let csv = dir.join(format!("{name}.csv"));
        std::fs::write(
            &csv,
            format!("{HEADER}\n{}\n{}\n{rows}", CLIMB[0], CLIMB[1]),
        )
        .expect("written");
        let mut run = Command::new("sleep").arg("30").spawn().expect("sleep runs");
        let outcome = watch(&args(run.id(), &csv, limits), Duration::from_millis(10));
        let status = run.wait().expect("sleep reaped");
        check!(outcome.ok() == Some(Outcome::Stopped(expected)), "{name}");
        check!(status.signal() == Some(15), "{name}");
    }
    std::fs::remove_dir_all(&dir).expect("cleaned up");
}

/// A timeout trips while the example hasn't written its CSV yet.
#[test]
fn times_out_before_the_recording_appears() {
    let dir = temp_dir("no_csv");
    let csv = dir.join("never.csv");
    let limits = Limits {
        timeout_s: Some(0.05),
        ..Limits::default()
    };
    let mut run = Command::new("sleep").arg("30").spawn().expect("sleep runs");
    let outcome = watch(&args(run.id(), &csv, limits), Duration::from_millis(10));
    let status = run.wait().expect("sleep reaped");
    check!(outcome.ok() == Some(Outcome::Stopped(Stop::Timeout { timeout_s: 0.05 })));
    check!(status.signal() == Some(15));
    std::fs::remove_dir_all(&dir).expect("cleaned up");
}

/// An example that exits by itself during the disarm grace still reports the landing.
#[test]
fn reports_a_landing_whose_example_exits_in_the_grace() {
    let dir = temp_dir("grace");
    let csv = dir.join("run.csv");
    std::fs::write(
        &csv,
        format!("{HEADER}\n{}\n{}\n3.0,0,0,10,0,0\n", CLIMB[0], CLIMB[1]),
    )
    .expect("written");
    let mut run = Command::new("sleep")
        .arg("0.2")
        .spawn()
        .expect("sleep runs");
    let pid = run.id();
    // Reaped as it exits, so `kill` finds no process when the grace ends.
    let reaper = thread::spawn(move || run.wait());
    let args = Args {
        disarm_grace_s: 0.6,
        ..args(pid, &csv, Limits::default())
    };
    let outcome = watch(&args, Duration::from_millis(10));
    check!(outcome.ok() == Some(Outcome::Stopped(Stop::Disarmed { distance_m: 0.0 })));
    let status = reaper.join().expect("reaper joined").expect("sleep reaped");
    check!(status.success());
    std::fs::remove_dir_all(&dir).expect("cleaned up");
}

/// A run that ends by itself is reported, and one that ends before its CSV appears is an
/// error.
#[test]
fn reports_a_run_that_exits_first() {
    let dir = temp_dir("exits");
    let csv = dir.join("run.csv");
    let mut ended = Command::new("true").spawn().expect("true runs");
    ended.wait().expect("true reaped");
    let poll = Duration::from_millis(10);

    let missing = watch(&args(ended.id(), &csv, ALL_LIMITS), poll);
    check!(let Err(WatchError::ExitedBeforeRecording) = missing);

    std::fs::write(&csv, format!("{HEADER}\n{}\n{}\n", CLIMB[0], CLIMB[1])).expect("written");
    let exited = watch(&args(ended.id(), &csv, ALL_LIMITS), poll);
    check!(let Ok(Outcome::Exited { .. }) = exited);
    std::fs::remove_dir_all(&dir).expect("cleaned up");
}
