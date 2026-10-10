use core::time::Duration;

use assert2::check;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{
    AbsoluteAttitudeCommand, ActuatorCommand, AttitudeCommand, AttitudeSetpoint, Command,
    ControlInput, ControlMode, ControlOutput, FailsafeState, LandCommand, RateCommand,
    RateSetpoint, Saturation, Setpoint, TelemetryRecord, Vec2, Vec3, VehicleState, WaypointCommand,
};

use super::*;
use crate::{MAX_FRAME_LEN, encode_vec};

fn name(s: &str) -> ParamName {
    ParamName::try_from(s).expect("name fits")
}

fn round_trip<M: Message + Clone + PartialEq + core::fmt::Debug>(msg: &M) -> usize {
    let mut frame = encode_vec(msg);
    let len = frame.len();
    check!(M::decode(&mut frame) == Ok(msg.clone()));
    len
}

/// Largest possible `Record`: a waypoint command with every option set, and the longest
/// varint `Duration`.
fn worst_case_record() -> Record {
    let mut saturation = Saturation::default();
    saturation.tilt = true;
    Record {
        dt: Duration::new(u64::MAX, 999_999_999),
        command: Command::armed(ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(1.0, 2.0),
            velocity_mps: Some(Vec2::new(3.0, 4.0)),
            altitude_m: 5.0,
            cruise_speed_mps: 6.0,
            roll_rad: Some(0.1),
        })),
        setpoint: Setpoint::Attitude(AttitudeSetpoint {
            roll_rad: 0.1,
            pitch_rad: 0.2,
            heading_rad: 0.3,
            thrust: 0.4,
        }),
        state: sample_state(),
        control: ControlOutput {
            thrust: 0.5,
            moments: Vec3::new(0.1, 0.2, 0.3),
        },
        actuators: ActuatorCommand {
            motors: [0.5; 8],
            rudder: 0.1,
            elevator: 0.2,
            aileron: 0.3,
            throttle: 0.4,
            elevon_left: 0.5,
            elevon_right: 0.6,
        },
        saturation,
        failsafe: Some(FailsafeState::Fallback),
    }
}

fn sample_state() -> VehicleState {
    VehicleState {
        position_m: Vec3::new(1.0, 2.0, 3.0),
        velocity_mps: Vec3::new(0.1, 0.2, 0.3),
        acceleration_mps2: Vec3::new(0.0, 0.0, -9.8),
        attitude: UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        angular_velocity_radps: Vec3::new(0.01, 0.02, 0.03),
        roll_rad: 0.1,
        pitch_rad: 0.2,
        heading_rad: 0.3,
    }
}

#[test]
fn every_uplink_variant_round_trips() {
    let cases = [
        Uplink::Control(Control::Heartbeat),
        Uplink::Control(Control::Command(Command::armed(ControlInput::Attitude(
            AttitudeCommand {
                throttle: 0.5,
                roll: -0.1,
                pitch: 0.2,
                yaw: 0.3,
                mode: 1,
            },
        )))),
        Uplink::Control(Control::Command(Command::new(
            false,
            ControlInput::Rate(RateCommand {
                throttle: 0.4,
                roll_rate: 0.1,
                ..Default::default()
            }),
        ))),
        Uplink::Control(Control::Command(Command::new(
            false,
            ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                roll_rad: 0.1,
                pitch_rad: -0.2,
                heading_rad: 3.0,
                thrust: 0.45,
            }),
        ))),
        Uplink::Control(Control::Command(Command::armed(ControlInput::Land(
            LandCommand {
                position_m: Vec2::new(4.0, -2.0),
            },
        )))),
        Uplink::Control(Control::Command(worst_case_record().command)),
        Uplink::Control(Control::Arm(true)),
        Uplink::Control(Control::Arm(false)),
        Uplink::Param(ParamRequest::Set {
            name: name("STAB_RATE_P"),
            value: 0.15,
        }),
        Uplink::Param(ParamRequest::List),
    ];
    for msg in &cases {
        round_trip(msg);
    }
}

#[test]
fn every_downlink_variant_round_trips() {
    let mut record = worst_case_record();
    record.setpoint = Setpoint::Rate(RateSetpoint {
        roll_rate: 0.1,
        pitch_rate: 0.2,
        yaw_rate: 0.3,
        thrust: 0.4,
    });
    let cases = [
        Downlink::Heartbeat {
            armed: true,
            mode: ControlMode::Waypoint,
            link_ok: false,
        },
        Downlink::State(sample_state()),
        Downlink::Heartbeat {
            armed: true,
            mode: ControlMode::Land,
            link_ok: true,
        },
        Downlink::Record(worst_case_record()),
        Downlink::Record(record),
        Downlink::Param(ParamValue {
            name: name("GUID_TILT_MAX"),
            value: 30.0,
            index: 3,
            count: 40,
        }),
        Downlink::Status(StatusText::try_from("armed").expect("fits")),
    ];
    for msg in &cases {
        round_trip(msg);
    }
}

#[test]
fn largest_messages_fit_max_frame_len() {
    let longest_name = name(&"N".repeat(params::MAX_NAME_LEN));
    let longest_status = StatusText::try_from("S".repeat(MAX_STATUS_LEN).as_str()).expect("fits");
    let downlinks = [
        Downlink::Record(worst_case_record()),
        Downlink::Param(ParamValue {
            name: longest_name.clone(),
            value: f32::MAX,
            index: u16::MAX,
            count: u16::MAX,
        }),
        Downlink::Status(longest_status),
    ];
    for msg in &downlinks {
        check!(round_trip(msg) <= MAX_FRAME_LEN);
    }
    let uplink = Uplink::Param(ParamRequest::Set {
        name: longest_name,
        value: f32::MAX,
    });
    check!(round_trip(&uplink) <= MAX_FRAME_LEN);
}

#[test]
fn record_mirrors_telemetry_record() {
    let record = worst_case_record();
    let telemetry: TelemetryRecord<'_> = record.telemetry();
    check!(Record::from(&telemetry) == record);
}
