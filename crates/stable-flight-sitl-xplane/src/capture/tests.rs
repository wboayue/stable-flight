use assert2::{assert, check};

use super::*;

/// A sample with a distinct value in every column.
fn sample() -> XPlaneSample {
    let mut sample = XPlaneSample::default();
    for (i, column) in COLUMNS.iter().enumerate() {
        (column.set)(&mut sample, i as f32 + 0.5);
    }
    sample.on_ground = true;
    sample.paused = false;
    sample
}

/// A capture written row by row reads back field for field, booleans as written.
#[test]
fn captures_round_trip() {
    let mut csv = Vec::new();
    write_header(&mut csv).expect("header");
    write_row(&mut csv, &sample()).expect("row");
    write_row(&mut csv, &XPlaneSample::default()).expect("row");

    let rows = read_capture(csv.as_slice()).expect("valid capture");

    assert!(rows == [sample(), XPlaneSample::default()]);
}

/// Columns are found by name, in any order and among extras.
#[test]
fn finds_columns_by_name() {
    let names: Vec<_> = COLUMNS.iter().rev().map(|c| c.name).collect();
    let values: Vec<_> = COLUMNS
        .iter()
        .rev()
        .map(|c| (c.get)(&sample()).to_string())
        .collect();
    let csv = format!("extra,{}\n1,{}\n", names.join(","), values.join(","));

    let rows = read_capture(csv.as_bytes()).expect("valid capture");

    assert!(rows == [sample()]);
}

/// A missing column, a bad value, and an empty file are errors; a short last row (a capture
/// cut off) is dropped, and blank lines skipped.
#[test]
fn reports_bad_captures() {
    let mut header = Vec::new();
    write_header(&mut header).expect("header");
    let header = String::from_utf8(header).expect("utf-8");
    let row = |sample: &XPlaneSample| {
        let mut out = Vec::new();
        write_row(&mut out, sample).expect("row");
        String::from_utf8(out).expect("utf-8")
    };

    check!(matches!(
        read_capture(&b""[..]),
        Err(CaptureError::NoHeader)
    ));
    check!(matches!(
        read_capture(&b"sim_time_s\n"[..]),
        Err(CaptureError::MissingColumn(name)) if name == "q_w"
    ));
    let bad = format!("{header}{}", row(&sample()).replacen("0.5", "x", 1));
    check!(matches!(
        read_capture(bad.as_bytes()),
        Err(CaptureError::BadValue { line: 2, column, .. }) if column == "sim_time_s"
    ));
    let cut = format!("{header}{}\n{}", row(&sample()), "1,2,3");
    check!(read_capture(cut.as_bytes()).expect("valid capture").len() == 1);
}
