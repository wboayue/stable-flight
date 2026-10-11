use std::cell::Cell;

use assert2::{assert, check};

use super::*;

/// A state with a distinct value in every captured field.
fn sample_state() -> SimulatorState {
    let mut state = SimulatorState::default();
    for (i, column) in COLUMNS.iter().enumerate() {
        (column.set)(&mut state, i as f32 + 0.5);
    }
    state.is_touching_ground = true;
    state.is_locked = false;
    state
}

fn sample_channels() -> ControlInputs {
    let mut control = ControlInputs::default();
    for (i, channel) in control.channels.iter_mut().enumerate() {
        *channel = i as f32 / 16.0;
    }
    control
}

/// A capture written row by row reads back field for field, booleans as written.
#[test]
fn captures_round_trip() {
    let state = sample_state();
    let control = sample_channels();
    let mut csv = Vec::new();
    write_header(&mut csv).expect("header");
    write_row(&mut csv, &state, &control).expect("row");
    write_row(
        &mut csv,
        &SimulatorState::default(),
        &ControlInputs::default(),
    )
    .expect("row");

    let rows = read_capture(csv.as_slice()).expect("valid capture");

    assert!(rows.len() == 2);
    check!(rows[0].channels == control.channels);
    for column in COLUMNS {
        check!(
            (column.get)(&rows[0].state) == (column.get)(&state),
            "{}",
            column.name
        );
    }
    check!(rows[0].state.is_touching_ground);
    check!(!rows[0].state.is_locked);
    check!(
        rows[1]
            == CapturedExchange {
                state: SimulatorState::default(),
                channels: [0.0; CHANNEL_COUNT],
            }
    );
}

/// Columns are found by name, so reordered and extra columns read the same.
#[test]
fn reads_columns_by_name() {
    let csv = "extra,ch11,roll_rate_dps\n7,0.25,3.5\n";
    let error = read_capture(csv.as_bytes()).expect_err("missing columns");
    check!(matches!(error, CaptureError::MissingColumn(_)));

    let mut names: Vec<String> = COLUMNS.iter().map(|c| c.name.to_string()).collect();
    names.extend((0..CHANNEL_COUNT).map(channel_column));
    names.reverse();
    names.insert(0, "extra".to_string());
    let mut values = vec!["0"; names.len()];
    let roll_rate = names
        .iter()
        .position(|n| n == "roll_rate_dps")
        .expect("column");
    values[roll_rate] = "3.5";
    let csv = format!("{}\n{}\n", names.join(","), values.join(","));

    let rows = read_capture(csv.as_bytes()).expect("valid capture");
    check!(rows[0].state.roll_rate_dps == 3.5);
}

/// A capture cut off mid-row (the process killed) reads up to the last whole row; a short row
/// elsewhere is an error.
#[test]
fn drops_a_cut_off_last_row() {
    let mut csv = Vec::new();
    write_header(&mut csv).expect("header");
    write_row(&mut csv, &sample_state(), &sample_channels()).expect("row");
    let whole = csv.len();
    write_row(&mut csv, &sample_state(), &sample_channels()).expect("row");
    let cut = &csv[..whole + 20];

    check!(read_capture(cut).expect("valid capture").len() == 1);

    let mut middle = cut.to_vec();
    middle.push(b'\n');
    middle.extend_from_slice(&csv[whole..]);
    check!(matches!(
        read_capture(middle.as_slice()),
        Err(CaptureError::BadValue { line: 3, .. })
    ));
}

/// An empty capture, or a value that isn't a number, is an error naming where.
#[test]
fn rejects_bad_captures() {
    check!(matches!(
        read_capture("".as_bytes()),
        Err(CaptureError::NoHeader)
    ));

    let mut csv = Vec::new();
    write_header(&mut csv).expect("header");
    write_row(
        &mut csv,
        &SimulatorState::default(),
        &ControlInputs::default(),
    )
    .expect("row");
    let text = String::from_utf8(csv).expect("utf-8");
    let bad = text.replacen("\n0,", "\nfast,", 1);

    let Err(CaptureError::BadValue {
        line,
        column,
        value,
    }) = read_capture(bad.as_bytes())
    else {
        panic!("expected a bad value");
    };
    check!(line == 2);
    check!(column == "physics_time_s");
    check!(value == "fast");
}

/// Hands back a fixed state and counts the calls.
#[derive(Default)]
struct FixedBridge {
    exchanges: Cell<usize>,
    rc: Cell<usize>,
}

impl RealFlightBridge for FixedBridge {
    fn exchange_data(&self, _control: &ControlInputs) -> Result<SimulatorState, BridgeError> {
        self.exchanges.set(self.exchanges.get() + 1);
        Ok(sample_state())
    }

    fn enable_rc(&self) -> Result<(), BridgeError> {
        self.rc.set(self.rc.get() + 1);
        Ok(())
    }

    fn disable_rc(&self) -> Result<(), BridgeError> {
        self.rc.set(self.rc.get() + 1);
        Ok(())
    }

    fn reset_aircraft(&self) -> Result<(), BridgeError> {
        self.rc.set(self.rc.get() + 1);
        Ok(())
    }
}

/// The recording bridge passes every call through and writes one row per exchange, with the
/// channels sent.
#[test]
fn recording_bridge_passes_through_and_records() {
    let mut csv = Vec::new();
    {
        let bridge = RecordingBridge::new(FixedBridge::default(), &mut csv).expect("header");
        let control = sample_channels();
        check!(bridge.exchange_data(&control).expect("exchange") == sample_state());
        bridge.exchange_data(&control).expect("exchange");
        bridge.disable_rc().expect("rc");
        bridge.reset_aircraft().expect("reset");
        bridge.enable_rc().expect("rc");
        check!(bridge.inner.exchanges.get() == 2);
        check!(bridge.inner.rc.get() == 3);
    }

    let rows = read_capture(csv.as_slice()).expect("valid capture");
    check!(rows.len() == 2);
    check!(rows[1].channels == sample_channels().channels);
}
