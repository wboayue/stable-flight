use std::io::{self, Write};
use std::time::Duration;

use assert2::check;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::testing::RecordedTick;
use stable_flight::{
    ActuatorCommand, AttitudeCommand, AttitudeSetpoint, BlendSetpoint, BodyRateSetpoint, Command,
    ControlInput, ControlOutput, CruiseOutput, FlightMode, FlightModeReport, ModeRefusal,
    PlaneSetpoint, RateCommand, RateSetpoint, Saturation, Setpoint, TelemetryPublisher, Vec3,
    VehicleState, WaypointCommand,
};

use super::{Columns, CopterColumns, CsvRecorder, QuadPlaneColumns, TailsitterColumns};

/// A level, armed tick with `setpoint` and `input`.
fn tick(setpoint: Setpoint, input: ControlInput) -> RecordedTick {
    RecordedTick {
        dt: Duration::from_millis(4),
        command: Command::new(true, input),
        setpoint,
        state: VehicleState {
            angular_velocity_radps: Vec3::new(0.1, -0.2, 0.3),
            position_m: Vec3::new(0.0, 0.0, 5.0),
            ..Default::default()
        },
        control: ControlOutput::new(0.5, Vec3::new(0.01, 0.02, 0.03)),
        actuators: ActuatorCommand::default(),
        saturation: Saturation::default(),
        failsafe: None,
        flight_mode: FlightModeReport::default(),
    }
}

fn attitude_setpoint() -> Setpoint {
    Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.25,
        pitch_rad: -0.125,
        heading_rad: 1.5,
        thrust: 0.5,
        transition: false,
        idle: false,
    })
}

fn attitude_sticks(yaw: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle: 0.6,
        yaw,
        ..Default::default()
    })
}

/// The CSV `columns` writes for `ticks`, as lines of fields.
fn recorded<C: Columns>(columns: C, ticks: &[RecordedTick]) -> Vec<Vec<String>> {
    let mut recorder = CsvRecorder::new(Some(Vec::new()), columns).expect("header written");
    for tick in ticks {
        recorder.publish(&tick.as_record());
    }
    let csv = String::from_utf8(recorder.into_writer().expect("a writer")).expect("UTF-8");
    csv.lines()
        .map(|line| line.split(',').map(str::to_owned).collect())
        .collect()
}

/// `row`'s field named `name` in `header`.
fn field<'a>(header: &[String], row: &'a [String], name: &str) -> &'a str {
    let index = header
        .iter()
        .position(|h| h == name)
        .unwrap_or_else(|| panic!("no column {name}"));
    &row[index]
}

fn setpoints() -> [Setpoint; 5] {
    [
        attitude_setpoint(),
        Setpoint::Rate(RateSetpoint::default()),
        Setpoint::BodyRate(BodyRateSetpoint::default()),
        Setpoint::Plane(PlaneSetpoint::default()),
        Setpoint::PlaneRate(BodyRateSetpoint::default()),
    ]
}

fn inputs() -> [ControlInput; 3] {
    [
        attitude_sticks(0.0),
        ControlInput::Rate(RateCommand::default()),
        ControlInput::Waypoint(WaypointCommand::default()),
    ]
}

/// Every setpoint and input under `columns`: a header starting `wall_s`, then one row per tick
/// with a field per name.
fn check_shape<C: Columns>(columns: C) {
    let ticks: Vec<_> = setpoints()
        .into_iter()
        .flat_map(|s| inputs().map(|i| tick(s, i)))
        .collect();
    let lines = recorded(columns, &ticks);
    check!(lines[0].join(",") == format!("wall_s,{}", C::HEADER));
    check!(lines.len() == ticks.len() + 1);
    for row in &lines[1..] {
        check!(row.len() == lines[0].len());
        check!(row[0].parse::<f32>().is_ok_and(|s| s >= 0.0));
    }
}

#[test]
fn copter_rows_match_the_header() {
    check_shape(CopterColumns);
}

#[test]
fn tailsitter_rows_match_the_header() {
    check_shape(TailsitterColumns {
        view: UnitQuaternion::identity(),
    });
}

#[test]
fn quadplane_rows_match_the_header() {
    check_shape(QuadPlaneColumns);
}

#[test]
fn copter_columns_carry_the_attitude_setpoint_and_stick_yaw() {
    // (scenario, setpoint, input, expected (sp_roll, sp_pitch, sp_heading, stick_yaw))
    let cases = [
        (
            "attitude sticks",
            attitude_setpoint(),
            attitude_sticks(-0.5),
            ["0.2500", "-0.1250", "1.5000", "-0.5000"],
        ),
        (
            "rate sticks",
            Setpoint::Rate(RateSetpoint::default()),
            ControlInput::Rate(RateCommand {
                yaw_rate: 0.75,
                ..Default::default()
            }),
            ["NaN", "NaN", "NaN", "0.7500"],
        ),
        (
            "a waypoint",
            attitude_setpoint(),
            ControlInput::Waypoint(WaypointCommand::default()),
            ["0.2500", "-0.1250", "1.5000", "NaN"],
        ),
    ];
    for (scenario, setpoint, input, expected) in cases {
        let lines = recorded(CopterColumns, &[tick(setpoint, input)]);
        let (header, row) = (&lines[0], &lines[1]);
        let names = ["sp_roll", "sp_pitch", "sp_heading", "stick_yaw"];
        let got = names.map(|name| field(header, row, name));
        check!(got == expected, "{scenario}");
        check!(field(header, row, "armed") == "1", "{scenario}");
        check!(field(header, row, "yaw_rate") == "0.3000", "{scenario}");
        check!(field(header, row, "moment_z") == "0.0300", "{scenario}");
        check!(field(header, row, "altitude") == "5.000", "{scenario}");
    }
}

#[test]
fn tailsitter_columns_name_the_setpoint_and_mode() {
    let body_rate = BodyRateSetpoint {
        roll_radps: 1.0,
        pitch_radps: 2.0,
        yaw_radps: 3.0,
        thrust: 0.5,
        idle: false,
    };
    // (setpoint, flight mode, expected (sp_kind, sp_roll, sp_p, mode))
    let cases = [
        (
            attitude_setpoint(),
            FlightMode::Copter,
            ["attitude", "0.2500", "NaN", "0"],
        ),
        (
            Setpoint::Plane(PlaneSetpoint {
                roll_rad: 0.5,
                ..Default::default()
            }),
            FlightMode::Plane,
            ["plane", "0.5000", "NaN", "1"],
        ),
        (
            Setpoint::BodyRate(body_rate),
            FlightMode::ToPlane,
            ["body_rate", "NaN", "1.0000", "2"],
        ),
        (
            Setpoint::PlaneRate(body_rate),
            FlightMode::ToCopter,
            ["plane_rate", "NaN", "1.0000", "3"],
        ),
        (
            Setpoint::Rate(RateSetpoint::default()),
            FlightMode::Copter,
            ["rate", "NaN", "NaN", "0"],
        ),
        (
            Setpoint::Blend(BlendSetpoint {
                hover: AttitudeSetpoint {
                    roll_rad: 0.25,
                    ..Default::default()
                },
                ..Default::default()
            }),
            FlightMode::ToPlane,
            ["blend", "0.2500", "NaN", "2"],
        ),
    ];
    let columns = TailsitterColumns {
        view: UnitQuaternion::identity(),
    };
    for (setpoint, mode, expected) in cases {
        let mut tick = tick(setpoint, attitude_sticks(0.0));
        tick.flight_mode.mode = mode;
        let lines = recorded(columns, &[tick]);
        let (header, row) = (&lines[0], &lines[1]);
        let got = ["sp_kind", "sp_roll", "sp_p", "mode"].map(|name| field(header, row, name));
        check!(got == expected, "{setpoint:?}");
    }
}

#[test]
fn tailsitter_columns_flag_a_refused_regime() {
    let columns = TailsitterColumns {
        view: UnitQuaternion::identity(),
    };
    // (refusal, expected `refused`)
    let cases = [(None, "0"), (Some(ModeRefusal::OnGround), "1")];
    for (refusal, expected) in cases {
        let mut tick = tick(attitude_setpoint(), attitude_sticks(0.0));
        tick.flight_mode.refusal = refusal;
        let lines = recorded(columns, &[tick]);
        check!(
            field(&lines[0], &lines[1], "refused") == expected,
            "{refusal:?}"
        );
    }
}

/// `north_m`, `east_m`, `vel_north_mps`, and `vel_east_mps` of one tick recorded with `columns`.
fn horizontal<C: Columns>(columns: C) -> [String; 4] {
    let mut tick = tick(attitude_setpoint(), attitude_sticks(0.0));
    tick.state.position_m = Vec3::new(12.5, -3.25, 5.0);
    tick.state.velocity_mps = Vec3::new(1.5, -0.75, 0.5);
    let lines = recorded(columns, &[tick]);
    ["north_m", "east_m", "vel_north_mps", "vel_east_mps"]
        .map(|name| field(&lines[0], &lines[1], name).to_owned())
}

#[test]
fn every_column_set_records_horizontal_position_and_velocity() {
    let expected = ["12.500", "-3.250", "1.500", "-0.750"].map(str::to_owned);
    check!(horizontal(CopterColumns) == expected);
    let tailsitter = TailsitterColumns {
        view: UnitQuaternion::identity(),
    };
    check!(horizontal(tailsitter) == expected);
    check!(horizontal(QuadPlaneColumns) == expected);
}

/// The quadplane set records both sides: the lift rotors, the pusher, and the surfaces, the
/// cruise output (NaN without one), a blend's lift share (NaN for other setpoints), and the
/// setpoint and mode as the tailsitter set does.
#[test]
fn quadplane_columns_carry_lift_and_cruise() {
    let blend = Setpoint::Blend(BlendSetpoint {
        hover: AttitudeSetpoint {
            roll_rad: 0.25,
            ..Default::default()
        },
        lift_share: 0.75,
        ..Default::default()
    });
    let mut actuators = ActuatorCommand::default();
    actuators.motors[..5].copy_from_slice(&[0.5, 0.25, 0.125, 0.75, 0.375]);
    actuators.servos[..3].copy_from_slice(&[0.5, -0.25, 1.0]);
    let names = [
        "mode",
        "sp_kind",
        "sp_roll",
        "lift_share",
        "cruise_thrust",
        "cruise_moment_y",
        "motor_0",
        "motor_3",
        "pusher",
        "servo_0",
        "servo_2",
    ];
    // (scenario, setpoint, mode, cruise, expected fields in `names` order)
    let cases = [
        (
            "transition: blend with cruise",
            blend,
            FlightMode::ToPlane,
            Some(CruiseOutput {
                thrust: 0.625,
                moments: Vec3::new(0.0, -0.5, 0.0),
            }),
            [
                "2", "blend", "0.2500", "0.7500", "0.6250", "-0.5000", "0.5000", "0.7500",
                "0.3750", "0.5000", "1.0000",
            ],
        ),
        (
            "copter: no cruise output",
            attitude_setpoint(),
            FlightMode::Copter,
            None,
            [
                "0", "attitude", "0.2500", "NaN", "NaN", "NaN", "0.5000", "0.7500", "0.3750",
                "0.5000", "1.0000",
            ],
        ),
    ];
    for (scenario, setpoint, mode, cruise, expected) in cases {
        let mut tick = tick(setpoint, attitude_sticks(0.0));
        tick.flight_mode.mode = mode;
        tick.control.cruise = cruise;
        tick.actuators = actuators;
        let lines = recorded(QuadPlaneColumns, &[tick]);
        let got = names.map(|name| field(&lines[0], &lines[1], name));
        check!(got == expected, "{scenario}");
    }
}

#[test]
fn records_nothing_without_a_writer() {
    let mut recorder =
        CsvRecorder::<_, Vec<u8>>::new(None, CopterColumns).expect("no header to write");
    recorder.publish(&tick(attitude_setpoint(), attitude_sticks(0.0)).as_record());
    check!(recorder.into_writer().is_none());
}

#[test]
fn records_nothing_without_a_path() {
    let recorder = CsvRecorder::create(None::<&str>, CopterColumns).expect("no file to create");
    check!(recorder.into_writer().is_none());
}

#[test]
fn creates_the_file_with_its_header() {
    let path = std::env::temp_dir().join(format!("csv-recorder-{}.csv", std::process::id()));
    let mut recorder = CsvRecorder::create(Some(&path), CopterColumns).expect("file created");
    recorder.publish(&tick(attitude_setpoint(), attitude_sticks(0.0)).as_record());
    drop(recorder);
    let csv = std::fs::read_to_string(&path).expect("file readable");
    std::fs::remove_file(&path).expect("file removed");
    check!(csv.lines().count() == 2);
    check!(csv.starts_with("wall_s,armed,"));
}

#[test]
fn fails_to_create_in_a_missing_directory() {
    let path = std::env::temp_dir().join("no-such-dir-csv-recorder/flight.csv");
    check!(CsvRecorder::create(Some(&path), CopterColumns).is_err());
}

/// A writer that accepts `budget` bytes, then fails every write.
struct FailingWriter {
    budget: usize,
    written: Vec<u8>,
}

impl Write for FailingWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        if self.budget == 0 {
            return Err(io::Error::other("disk full"));
        }
        let n = buf.len().min(self.budget);
        self.budget -= n;
        self.written.extend_from_slice(&buf[..n]);
        Ok(n)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[test]
fn a_failed_header_write_is_an_error() {
    let writer = FailingWriter {
        budget: 0,
        written: Vec::new(),
    };
    check!(CsvRecorder::new(Some(writer), CopterColumns).is_err());
}

#[test]
fn a_failed_row_write_is_ignored() {
    let header_len = "wall_s,".len() + CopterColumns::HEADER.len() + 1;
    let writer = FailingWriter {
        budget: header_len,
        written: Vec::new(),
    };
    let mut recorder = CsvRecorder::new(Some(writer), CopterColumns).expect("header fits");
    let tick = tick(attitude_setpoint(), attitude_sticks(0.0));
    // Each publish fails to write; neither panics.
    recorder.publish(&tick.as_record());
    recorder.publish(&tick.as_record());
    let writer = recorder.into_writer().expect("a writer");
    check!(writer.written.len() == header_len);
}
