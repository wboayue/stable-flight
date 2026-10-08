use std::time::Duration;

use assert2::check;

use super::*;

fn record_with_thrust(thrust: f32) -> RecordedTick {
    let command = Command::Attitude(AttitudeCommand::default());
    let setpoint = Setpoint::Attitude(Default::default());
    let state = VehicleState::default();
    let control = ControlOutput {
        thrust,
        ..Default::default()
    };
    let actuators = ActuatorCommand::default();
    RecordedTick::from(&TelemetryRecord {
        dt: Duration::from_millis(4),
        command: &command,
        setpoint: &setpoint,
        state: &state,
        control: &control,
        actuators: &actuators,
    })
}

fn publish_thrusts<const N: usize>(thrusts: &[f32]) -> RecordingTelemetry<N> {
    let mut telemetry = RecordingTelemetry::<N>::new();
    for &thrust in thrusts {
        telemetry.publish(&record_with_thrust(thrust).as_record());
    }
    telemetry
}

fn kept<const N: usize>(telemetry: &RecordingTelemetry<N>) -> Vec<f32> {
    telemetry.iter().map(|tick| tick.control.thrust).collect()
}

#[test]
fn recording_telemetry_keeps_the_last_n_ticks_oldest_first() {
    // (published thrusts, expected kept thrusts) for a capacity-3 recorder
    let cases: [(&[f32], &[f32]); 4] = [
        (&[], &[]),
        (&[0.1], &[0.1]),
        (&[0.1, 0.2, 0.3], &[0.1, 0.2, 0.3]),
        (&[0.1, 0.2, 0.3, 0.4, 0.5], &[0.3, 0.4, 0.5]),
    ];
    for (published, expected) in cases {
        let telemetry = publish_thrusts::<3>(published);
        check!(kept(&telemetry) == expected, "published {published:?}");
        check!(telemetry.total() == published.len() as u64);
        check!(telemetry.last().map(|tick| tick.control.thrust) == expected.last().copied());
    }
}

#[test]
fn zero_capacity_recorder_only_counts() {
    let telemetry = publish_thrusts::<0>(&[0.1, 0.2]);
    check!(telemetry.total() == 2);
    check!(telemetry.last().is_none());
    check!(telemetry.iter().next().is_none());
}

#[test]
fn null_vehicle_counts_calls_and_keeps_last_command() {
    let mut vehicle = NullVehicle::with_state(VehicleState {
        heading_rad: 1.5,
        ..Default::default()
    });
    check!(vehicle.last_actuation().is_none());

    check!(vehicle.estimate_state(Duration::ZERO).heading_rad == 1.5);
    vehicle.estimate_state(Duration::from_millis(4));
    vehicle.actuate(&ActuatorCommand {
        motors: [0.25; 8],
        ..Default::default()
    });

    check!(vehicle.estimates() == 2);
    check!(vehicle.actuations() == 1);
    assert2::assert!(let Some(last) = vehicle.last_actuation());
    check!(last.motors[0] == 0.25);
}

#[test]
fn constant_commander_repeats_its_command() {
    let mut commander = ConstantCommander::default();
    for _ in 0..3 {
        let command = commander.next_command(0.004, &VehicleState::default());
        assert2::assert!(let Command::Attitude(AttitudeCommand { throttle, arm: false, .. }) = command);
        check!(throttle == 0.0);
    }
}
