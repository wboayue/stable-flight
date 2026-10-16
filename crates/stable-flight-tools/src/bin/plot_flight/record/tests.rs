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

/// Only panels with a column in the recording are drawn, in order.
#[test]
fn draws_only_panels_it_has_columns_for() {
    let titles: Vec<_> = recording().panels().iter().map(|p| p.title).collect();
    check!(titles == ["altitude (m)", "lift and thrust", "flight mode"]);
}

/// A recording renders to SVG; an empty window is an error.
#[test]
fn renders_a_recording() {
    let dir = std::env::temp_dir().join(format!("plot_flight_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let input = dir.join("lap.csv");
    std::fs::write(&input, CSV).expect("written");
    let input = input.to_str().expect("utf-8");
    let output = dir.join("lap.svg");
    let output = output.to_str().expect("utf-8");

    let all = (f32::NEG_INFINITY, f32::INFINITY);
    check!(render(input, output, all).is_ok());
    let svg = std::fs::read_to_string(output).expect("svg written");
    check!(svg.contains("ground track"));
    check!(render(input, output, (10.0, 20.0)).is_err());
    check!(render(input, &dir.join("lap.txt").to_string_lossy(), all).is_err());
    std::fs::remove_dir_all(&dir).expect("cleaned up");
}
