use std::time::Duration;

use assert2::check;

use super::*;
use crate::testing::{NullVehicle, RecordingTelemetry};
use crate::{Command, ControlOutput, Saturation, Setpoint, Vehicle};

/// State source that reports how many times it was asked, as the heading.
#[derive(Default)]
struct CountingSource {
    calls: u32,
}
impl StateSource for CountingSource {
    fn state(&mut self, _dt: Duration) -> VehicleState {
        self.calls += 1;
        VehicleState {
            heading_rad: self.calls as f32,
            ..Default::default()
        }
    }
}

fn assert_is_vehicle<V: Vehicle>(_: &V) {}

#[test]
fn composed_vehicle_routes_state_reads_and_actuation_to_their_halves() {
    let mut vehicle = ComposedVehicle::new(CountingSource::default(), NullVehicle::new());
    assert_is_vehicle(&vehicle);

    check!(vehicle.state(Duration::ZERO).heading_rad == 1.0);
    check!(vehicle.state(Duration::from_millis(4)).heading_rad == 2.0);
    vehicle.actuate(&ActuatorCommand {
        motors: [0.75; 8],
        ..Default::default()
    });

    // The actuator half saw only actuation; the source half saw only state reads.
    check!(vehicle.source().calls == 2);
    check!(vehicle.actuator().state_reads() == 0);
    check!(vehicle.actuator().actuations() == 1);

    let (source, actuator) = vehicle.into_parts();
    check!(source.calls == 2);
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
            failsafe: None,
            flight_mode: Default::default(),
            wind: None,
        });
    }

    let (a, (b, c)) = &sinks;
    for sink in [a, b, c] {
        check!(sink.total() == 2);
        check!(sink.last().map(|tick| tick.control.thrust) == Some(0.2));
    }
}
