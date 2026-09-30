use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use assert2::check;
use robomath::vec3;

use super::*;
use crate::{Actuator, ActuatorCommand, AttitudeCommand, Command, ControlOutput, StateEstimator};

/// Shared, ordered log of pipeline calls. Test-only; the loop itself never allocates.
type Log = Rc<RefCell<Vec<String>>>;

/// A state whose `heading` identifies which estimate produced it.
fn state_with_heading(heading: f32) -> VehicleState {
    VehicleState {
        heading,
        ..Default::default()
    }
}

struct RecordingCommander {
    log: Log,
}
impl Commander for RecordingCommander {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.log
            .borrow_mut()
            .push(format!("command dt={dt} seen_heading={}", state.heading));
        Command::Attitude(AttitudeCommand {
            throttle: 0.25,
            ..Default::default()
        })
    }
}

/// Returns heading 0 for the priming estimate, then 1, 2, … per tick.
struct RecordingVehicle {
    log: Log,
    estimates: u32,
}
impl StateEstimator for RecordingVehicle {
    fn estimate_state(&mut self, dt: Duration) -> VehicleState {
        self.log
            .borrow_mut()
            .push(format!("estimate dt={}", dt.as_secs_f32()));
        let state = state_with_heading(self.estimates as f32);
        self.estimates += 1;
        state
    }
}
impl Actuator for RecordingVehicle {
    fn actuate(&mut self, command: &ActuatorCommand) {
        self.log
            .borrow_mut()
            .push(format!("actuate motor0={}", command.motors[0]));
    }
}

struct RecordingStabilizer {
    log: Log,
}
impl Stabilizer for RecordingStabilizer {
    fn stabilize(&mut self, dt: f32, command: &Command, state: &VehicleState) -> ControlOutput {
        let Command::Attitude(attitude) = command else {
            unreachable!("RecordingCommander only emits attitude commands");
        };
        self.log.borrow_mut().push(format!(
            "stabilize dt={dt} throttle={} heading={}",
            attitude.throttle, state.heading
        ));
        ControlOutput::new(attitude.throttle, vec3(0.0, 0.0, 0.0))
    }

    fn prime(&mut self, state: &VehicleState) {
        self.log
            .borrow_mut()
            .push(format!("prime heading={}", state.heading));
    }
}

/// Puts thrust on motor 0 so the actuator log shows the mixer output was forwarded.
struct RecordingMixer {
    log: Log,
}
impl Mixer for RecordingMixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand {
        self.log
            .borrow_mut()
            .push(format!("mix thrust={}", controls.thrust));
        let mut command = ActuatorCommand::default();
        command.motors[0] = controls.thrust;
        command
    }
}

struct LogTelemetry {
    log: Log,
}
impl TelemetryPublisher for LogTelemetry {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.log.borrow_mut().push(format!(
            "publish dt={} heading={} thrust={} motor0={}",
            record.dt.as_secs_f32(),
            record.state.heading,
            record.control.thrust,
            record.actuators.motors[0]
        ));
    }
}

fn recording_loop(
    log: &Log,
) -> FlightLoop<
    RecordingCommander,
    RecordingVehicle,
    RecordingStabilizer,
    RecordingMixer,
    LogTelemetry,
> {
    FlightLoop::new(
        RecordingCommander { log: log.clone() },
        RecordingVehicle {
            log: log.clone(),
            estimates: 0,
        },
        RecordingStabilizer { log: log.clone() },
        RecordingMixer { log: log.clone() },
        LogTelemetry { log: log.clone() },
    )
}

#[test]
fn prime_estimates_with_zero_dt_and_primes_stabilizer_from_it() {
    let log = Log::default();
    let mut flight_loop = recording_loop(&log);

    flight_loop.prime();

    check!(*log.borrow() == ["estimate dt=0", "prime heading=0"]);
    check!(flight_loop.state().heading == 0.0);
}

#[test]
fn new_does_not_touch_the_vehicle() {
    let log = Log::default();
    let _flight_loop = recording_loop(&log);

    check!(log.borrow().is_empty());
}

#[test]
fn step_runs_the_pipeline_in_order_and_publishes_the_tick() {
    let log = Log::default();
    let mut flight_loop = recording_loop(&log);
    flight_loop.prime();
    log.borrow_mut().clear();

    flight_loop.step(Duration::from_millis(4));

    check!(
        *log.borrow()
            == [
                // Commander sees the primed state (heading 0), not this tick's estimate.
                "command dt=0.004 seen_heading=0",
                "estimate dt=0.004",
                // Stabilizer sees the fresh estimate (heading 1).
                "stabilize dt=0.004 throttle=0.25 heading=1",
                "mix thrust=0.25",
                "actuate motor0=0.25",
                "publish dt=0.004 heading=1 thrust=0.25 motor0=0.25",
            ]
    );
    check!(flight_loop.state().heading == 1.0);
}

#[test]
fn commander_sees_previous_tick_state_across_steps() {
    let log = Log::default();
    let mut flight_loop = recording_loop(&log);
    flight_loop.prime();

    for _ in 0..3 {
        flight_loop.step(Duration::from_millis(4));
    }

    let seen: Vec<String> = log
        .borrow()
        .iter()
        .filter(|line| line.starts_with("command"))
        .cloned()
        .collect();
    check!(
        seen == [
            "command dt=0.004 seen_heading=0",
            "command dt=0.004 seen_heading=1",
            "command dt=0.004 seen_heading=2",
        ]
    );
}

#[test]
fn into_parts_returns_the_components_with_their_state() {
    use crate::mixers::QuadCopterMixer;
    use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    use crate::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};

    let mut flight_loop = FlightLoop::new(
        ConstantCommander::default(),
        NullVehicle::new(),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        RecordingTelemetry::<8>::new(),
    );
    flight_loop.prime();
    flight_loop.step(Duration::from_millis(4));
    flight_loop.step(Duration::from_millis(4));

    let (_commander, vehicle, _stabilizer, _mixer, telemetry) = flight_loop.into_parts();
    check!(vehicle.estimates() == 3); // prime + 2 ticks
    check!(vehicle.actuations() == 2);
    check!(telemetry.total() == 2);
}
