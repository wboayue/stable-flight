use std::time::Duration;

use assert2::check;

use super::*;
use crate::testing::{NullVehicle, RecordingTelemetry};
use crate::{Command, ControlOutput, Saturation, Setpoint, Vehicle};

/// Estimator that reports how many times it was asked, as the heading.
#[derive(Default)]
struct CountingEstimator {
    calls: u32,
}
impl StateEstimator for CountingEstimator {
    fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
        self.calls += 1;
        VehicleState {
            heading_rad: self.calls as f32,
            ..Default::default()
        }
    }
}

fn assert_is_vehicle<V: Vehicle>(_: &V) {}

#[test]
fn composed_vehicle_routes_estimates_and_actuation_to_their_halves() {
    let mut vehicle = ComposedVehicle::new(CountingEstimator::default(), NullVehicle::new());
    assert_is_vehicle(&vehicle);

    check!(vehicle.estimate_state(Duration::ZERO).heading_rad == 1.0);
    check!(vehicle.estimate_state(Duration::from_millis(4)).heading_rad == 2.0);
    vehicle.actuate(&ActuatorCommand {
        motors: [0.75; 8],
        ..Default::default()
    });

    // The actuator half saw only actuation; the estimator half saw only estimates.
    check!(vehicle.estimator().calls == 2);
    check!(vehicle.actuator().estimates() == 0);
    check!(vehicle.actuator().actuations() == 1);

    let (estimator, actuator) = vehicle.into_parts();
    check!(estimator.calls == 2);
    check!(actuator.last_actuation().map(|c| c.motors[0]) == Some(0.75));
}

#[test]
fn tuple_publisher_fans_out_to_every_sink() {
    let mut sinks = (
        RecordingTelemetry::<4>::new(),
        (
            RecordingTelemetry::<4>::new(),
            RecordingTelemetry::<4>::new(),
        ),
    );
    let command = Command::default();
    let setpoint = Setpoint::Attitude(Default::default());
    let state = VehicleState::default();
    let actuators = ActuatorCommand::default();

    for thrust in [0.1, 0.2] {
        sinks.publish(&TelemetryRecord {
            dt: Duration::from_millis(4),
            command: &command,
            setpoint: &setpoint,
            state: &state,
            control: &ControlOutput {
                thrust,
                ..Default::default()
            },
            actuators: &actuators,
            saturation: Saturation::default(),
        });
    }

    let (a, (b, c)) = &sinks;
    for sink in [a, b, c] {
        check!(sink.total() == 2);
        check!(sink.last().map(|tick| tick.control.thrust) == Some(0.2));
    }
}
