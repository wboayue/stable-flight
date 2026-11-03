use assert2::check;

use super::*;

/// A quadplane-like recording: a text column, a NaN gap, and a truncated last row.
const CSV: &str = "wall_s,mode,altitude,sp_kind,lift_share,north_m,east_m
0.0,0,1.0,attitude,NaN,0.0,0.0
0.5,2,2.0,blend,1.0,1.0,0.0
1.0,2,3.0,blend,0.5,2.0,1.0
1.5,1,4.0,plane,NaN,3.0,2.0
2.0,3,5.0,blend,0.8,4.0,3.0
2.5,3,6.0";

/// A series' runs of (time, value) points.
type Runs = Vec<Vec<(f32, f32)>>;

/// A time window, in seconds.
type Window = (f32, f32);

fn recording() -> Recording {
    Recording::parse(CSV).expect("parsed")
}

/// Text cells read as NaN, and a short row is dropped.
#[test]
fn parses_numbers_and_drops_short_rows() {
    let recording = recording();
    check!(recording.rows.len() == 5);
    check!(recording.rows[1][3].is_nan());
    check!(recording.rows[1][2] == 2.0);
}

/// A file without `wall_s` first, or without a header, isn't a recording.
#[test]
fn rejects_other_csvs() {
    check!(Recording::parse("x,y,altitude,cruise_speed\n0,0,5,2").is_err());
    check!(Recording::parse("").is_err());
}

/// A header line finds columns by name; a data line reads only at the header's width, its
/// line end ignored.
#[test]
fn reads_a_line_at_a_time() {
    let header = Header::parse("wall_s,mode,sp_kind\n").expect("a header");
    check!(header.column("sp_kind") == Some(2));
    check!(header.column("altitude").is_none());
    check!(header.require("mode") == Ok(1));
    check!(header.require("altitude") == Err(RecordingError::MissingColumn("altitude")));
    let row = header.row("0.5,1,plane\n").expect("a whole row");
    check!(row[..2] == [0.5, 1.0]);
    check!(row[2].is_nan());
    check!(header.row("0.5,1").is_none());
    check!(header.row("0.5,1,plane,2").is_none());
    check!(Header::parse("mode,wall_s") == Err(RecordingError::NoTime));
    check!(Recording::parse("") == Err(RecordingError::Empty));
}

/// A series breaks at NaN cells, keeps to the window, and scales; a missing column is `None`.
#[test]
fn splits_series_at_gaps_within_the_window() {
    let recording = recording();
    let all = (f32::NEG_INFINITY, f32::INFINITY);
    // (column, scale, window, expected runs)
    let rows: [(&str, f32, Window, Option<Runs>); 4] = [
        (
            "lift_share",
            1.0,
            all,
            Some(vec![vec![(0.5, 1.0), (1.0, 0.5)], vec![(2.0, 0.8)]]),
        ),
        (
            "altitude",
            2.0,
            (0.5, 1.0),
            Some(vec![vec![(0.5, 4.0), (1.0, 6.0)]]),
        ),
        ("lift_share", 1.0, (1.4, 1.6), Some(vec![])),
        ("airspeed_mps", 1.0, all, None),
    ];
    for (column, scale, window, expected) in rows {
        check!(
            recording.series(column, scale, window) == expected,
            "{column} {window:?}"
        );
    }
}
