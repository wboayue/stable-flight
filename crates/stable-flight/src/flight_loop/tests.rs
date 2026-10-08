use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use crate::Vec3;
use assert2::check;

use super::*;
use crate::{
    Actuator, ActuatorCommand, AttitudeCommand, AttitudeSetpoint, Command, ControlOutput, Setpoint,
    StateEstimator,
};

/// Shared, ordered log of pipeline calls. Test-only; the loop itself never allocates.
type Log = Rc<RefCell<Vec<String>>>;

/// A state whose `heading` identifies which estimate produced it.
fn state_with_heading(heading: f32) -> VehicleState {
    VehicleState {
        heading_rad: heading,
        ..Default::default()
    }
}

struct RecordingCommander {
    log: Log,
}
impl Commander for RecordingCommander {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.log.borrow_mut().push(format!(
            "command dt={dt} seen_heading={}",
            state.heading_rad
        ));
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

/// Doubles attitude-command throttle into setpoint thrust, so the logs tell the command, the
/// setpoint, and the control output apart.
struct RecordingGuidance {
    log: Log,
}
impl Guidance for RecordingGuidance {
    fn guide(&mut self, dt: f32, command: &Command, state: &VehicleState) -> Setpoint {
        let Command::Attitude(attitude) = command else {
            unreachable!("RecordingCommander only emits attitude commands");
        };
        self.log.borrow_mut().push(format!(
            "guide dt={dt} throttle={} heading={}",
            attitude.throttle, state.heading_rad
        ));
        Setpoint::Attitude(AttitudeSetpoint {
            thrust: 2.0 * attitude.throttle,
            ..Default::default()
        })
    }

    fn prime(&mut self, state: &VehicleState) {
        self.log
            .borrow_mut()
            .push(format!("prime guidance heading={}", state.heading_rad));
    }
}

struct RecordingStabilizer {
    log: Log,
}
impl Stabilizer for RecordingStabilizer {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        let Setpoint::Attitude(attitude) = setpoint else {
            unreachable!("RecordingGuidance only emits attitude setpoints");
        };
        self.log.borrow_mut().push(format!(
            "stabilize dt={dt} thrust={} heading={}",
            attitude.thrust, state.heading_rad
        ));
        ControlOutput::new(attitude.thrust + 0.25, Vec3::zeros())
    }

    fn prime(&mut self, state: &VehicleState) {
        self.log
            .borrow_mut()
            .push(format!("prime stabilizer heading={}", state.heading_rad));
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
            "publish dt={} throttle={} setpoint_thrust={} heading={} thrust={} motor0={}",
            record.dt.as_secs_f32(),
            match record.command {
                Command::Attitude(attitude) => attitude.throttle,
                _ => f32::NAN,
            },
            match record.setpoint {
                Setpoint::Attitude(attitude) => attitude.thrust,
                Setpoint::Rate(rate) => rate.thrust,
            },
            record.state.heading_rad,
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
    RecordingGuidance,
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
        RecordingGuidance { log: log.clone() },
        RecordingStabilizer { log: log.clone() },
        RecordingMixer { log: log.clone() },
        LogTelemetry { log: log.clone() },
    )
}

#[test]
fn prime_estimates_with_zero_dt_and_primes_guidance_then_stabilizer_from_it() {
    let log = Log::default();
    let mut flight_loop = recording_loop(&log);

    flight_loop.prime();

    check!(
        *log.borrow()
            == [
                "estimate dt=0",
                "prime guidance heading=0",
                "prime stabilizer heading=0",
            ]
    );
    check!(flight_loop.state().heading_rad == 0.0);
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
                // Guidance and stabilizer see the fresh estimate (heading 1).
                "guide dt=0.004 throttle=0.25 heading=1",
                "stabilize dt=0.004 thrust=0.5 heading=1",
                "mix thrust=0.75",
                "actuate motor0=0.75",
                "publish dt=0.004 throttle=0.25 setpoint_thrust=0.5 heading=1 thrust=0.75 motor0=0.75",
            ]
    );
    check!(flight_loop.state().heading_rad == 1.0);
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
    use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    use crate::mixers::QuadCopterMixer;
    use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    use crate::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};

    let mut flight_loop = FlightLoop::new(
        ConstantCommander::default(),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        RecordingTelemetry::<8>::new(),
    );
    flight_loop.prime();
    flight_loop.step(Duration::from_millis(4));
    flight_loop.step(Duration::from_millis(4));

    let (_commander, vehicle, _guidance, _stabilizer, _mixer, telemetry) = flight_loop.into_parts();
    check!(vehicle.estimates() == 3); // prime + 2 ticks
    check!(vehicle.actuations() == 2);
    check!(telemetry.total() == 2);
}

/// The config setters reach the composites in a running loop: a valid config applies, an
/// invalid one is rejected and the current one kept.
#[test]
fn config_setters_retune_the_composites_between_ticks() {
    use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    use crate::mixers::QuadCopterMixer;
    use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    use crate::testing::{ConstantCommander, NullVehicle};

    let mut flight_loop = FlightLoop::new(
        ConstantCommander::default(),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        (),
    );
    flight_loop.prime();
    flight_loop.step(Duration::from_millis(4));

    let mut guidance = *flight_loop.guidance().config();
    guidance.max_tilt = 0.4;
    check!(flight_loop.set_guidance_config(guidance).is_ok());
    let mut stabilizer = *flight_loop.stabilizer().config();
    stabilizer.body_rate.gains.p = 0.2;
    check!(flight_loop.set_stabilizer_config(stabilizer).is_ok());

    let mut bad_guidance = guidance;
    bad_guidance.max_tilt = 0.0;
    check!(flight_loop.set_guidance_config(bad_guidance).is_err());
    let mut bad_stabilizer = stabilizer;
    bad_stabilizer.thrust_curve_exponent = -1.0;
    check!(flight_loop.set_stabilizer_config(bad_stabilizer).is_err());

    check!(*flight_loop.guidance().config() == guidance);
    check!(*flight_loop.stabilizer().config() == stabilizer);
    flight_loop.step(Duration::from_millis(4));
}
