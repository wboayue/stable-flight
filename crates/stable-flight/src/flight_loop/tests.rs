use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use crate::Vec3;
use assert2::check;

use super::*;
use crate::{
    Actuator, ActuatorCommand, AttitudeCommand, AttitudeSetpoint, Command, ControlInput,
    ControlOutput, FailsafeState, Setpoint, StateSource,
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

/// Commands throttle 0.25, armed per `arms` (one entry per tick, the last one repeating).
struct RecordingCommander {
    log: Log,
    arms: Vec<bool>,
    ticks: usize,
}
impl Commander for RecordingCommander {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        let arm = self.arms[self.ticks.min(self.arms.len() - 1)];
        self.ticks += 1;
        self.log.borrow_mut().push(format!(
            "command dt={dt} seen_heading={} arm={arm}",
            state.heading_rad
        ));
        Command::new(
            arm,
            ControlInput::Attitude(AttitudeCommand {
                throttle: 0.25,
                ..Default::default()
            }),
        )
    }
}

/// Returns heading 0 for the priming read, then 1, 2, … per tick.
struct RecordingVehicle {
    log: Log,
    state_reads: u32,
}
impl StateSource for RecordingVehicle {
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.log
            .borrow_mut()
            .push(format!("state dt={}", dt.as_secs_f32()));
        let state = state_with_heading(self.state_reads as f32);
        self.state_reads += 1;
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
    fn guide(&mut self, dt: f32, input: &ControlInput, state: &VehicleState) -> Setpoint {
        let ControlInput::Attitude(attitude) = input else {
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
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand {
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
            "publish dt={} arm={} throttle={} setpoint_thrust={} setpoint_heading={} heading={} thrust={} motor0={}",
            record.dt.as_secs_f32(),
            record.command.arm,
            match record.command.input {
                ControlInput::Attitude(attitude) => attitude.throttle,
                _ => f32::NAN,
            },
            match record.setpoint {
                Setpoint::Attitude(attitude) => attitude.thrust,
                Setpoint::Rate(rate) => rate.thrust,
            },
            match record.setpoint {
                Setpoint::Attitude(attitude) => attitude.heading_rad,
                Setpoint::Rate(_) => f32::NAN,
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
    recording_loop_with_arms(log, &[true])
}

fn recording_loop_with_arms(
    log: &Log,
    arms: &[bool],
) -> FlightLoop<
    RecordingCommander,
    RecordingVehicle,
    RecordingGuidance,
    RecordingStabilizer,
    RecordingMixer,
    LogTelemetry,
> {
    FlightLoop::new(
        RecordingCommander {
            log: log.clone(),
            arms: arms.to_vec(),
            ticks: 0,
        },
        RecordingVehicle {
            log: log.clone(),
            state_reads: 0,
        },
        RecordingGuidance { log: log.clone() },
        RecordingStabilizer { log: log.clone() },
        RecordingMixer { log: log.clone() },
        LogTelemetry { log: log.clone() },
    )
}

#[test]
fn prime_reads_state_with_zero_dt_and_primes_guidance_then_stabilizer_from_it() {
    let log = Log::default();
    let mut flight_loop = recording_loop(&log);

    flight_loop.prime();

    check!(
        *log.borrow()
            == [
                "state dt=0",
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
                "command dt=0.004 seen_heading=0 arm=true",
                "state dt=0.004",
                // The first armed tick re-primes from the fresh estimate (heading 1)...
                "prime guidance heading=1",
                "prime stabilizer heading=1",
                // ...and guidance and stabilizer see it too.
                "guide dt=0.004 throttle=0.25 heading=1",
                "stabilize dt=0.004 thrust=0.5 heading=1",
                "mix thrust=0.75",
                "actuate motor0=0.75",
                "publish dt=0.004 arm=true throttle=0.25 setpoint_thrust=0.5 setpoint_heading=0 heading=1 thrust=0.75 motor0=0.75",
            ]
    );
    check!(flight_loop.state().heading_rad == 1.0);
}

#[test]
fn disarmed_tick_holds_the_motors_off_and_skips_guidance_stabilizer_and_mixer() {
    let log = Log::default();
    let mut flight_loop = recording_loop_with_arms(&log, &[false]);
    flight_loop.prime();
    log.borrow_mut().clear();

    flight_loop.step(Duration::from_millis(4));

    check!(
        *log.borrow()
            == [
                "command dt=0.004 seen_heading=0 arm=false",
                "state dt=0.004",
                "actuate motor0=0",
                // Neutral setpoint at the current heading, zero control.
                "publish dt=0.004 arm=false throttle=0.25 setpoint_thrust=0 setpoint_heading=1 heading=1 thrust=0 motor0=0",
            ]
    );
}

/// Guidance and the stabilizer are re-primed on each disarmed → armed edge, and only then.
#[test]
fn reprimes_on_each_arm_edge_only() {
    let cases: &[(&[bool], &[usize])] = &[
        // arms per tick, ticks (0-based) that prime
        (&[true, true, true], &[0]),
        (&[false, false, true, true], &[2]),
        (&[true, false, true], &[0, 2]),
        (&[false, false, false], &[]),
    ];
    for &(arms, primed_ticks) in cases {
        let log = Log::default();
        let mut flight_loop = recording_loop_with_arms(&log, arms);
        flight_loop.prime();

        let mut primed = Vec::new();
        for tick in 0..arms.len() {
            log.borrow_mut().clear();
            flight_loop.step(Duration::from_millis(4));
            let lines = log.borrow();
            let guidance = lines.iter().position(|l| l.starts_with("prime guidance"));
            let stabilizer = lines.iter().position(|l| l.starts_with("prime stabilizer"));
            check!(
                guidance.is_some() == stabilizer.is_some(),
                "arms {arms:?} tick {tick}"
            );
            if let (Some(g), Some(s)) = (guidance, stabilizer) {
                check!(g < s, "guidance primes before the stabilizer");
                primed.push(tick);
            }
        }
        check!(primed == primed_ticks, "arms {arms:?}");
    }
}

/// `armed()` reports the last tick's arm flag, false before the first tick.
#[test]
fn armed_follows_the_last_tick() {
    let arms = [false, true, true, false, true];
    let log = Log::default();
    let mut flight_loop = recording_loop_with_arms(&log, &arms);
    flight_loop.prime();
    check!(!flight_loop.armed());

    for (tick, &arm) in arms.iter().enumerate() {
        flight_loop.step(Duration::from_millis(4));
        check!(flight_loop.armed() == arm, "tick {tick}");
    }
}

/// The tuning calls reach a `Tunable` stabilizer between ticks: a stage started between ticks
/// runs on the next one, and stopping it returns its status.
#[test]
fn tuning_calls_reach_the_stabilizer_between_ticks() {
    use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    use crate::mixers::QuadCopterMixer;
    use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
    use crate::testing::{ConstantCommander, NullVehicle};
    use crate::tuning::{RelayConfig, TuningStatus};

    let mut flight_loop = FlightLoop::new(
        ConstantCommander::new(Command::armed(ControlInput::default())),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        (),
    );
    flight_loop.prime();
    flight_loop.step(Duration::from_millis(4));

    let relay = RelayConfig {
        initial_relay: 0.05,
        max_relay: 0.3,
        relay_step: 1.5,
        hysteresis: 0.002,
        min_cycle_amplitude: 0.05,
        max_cycle_amplitude: 2.0,
        max_half_cycle_s: 0.3,
        max_bias: 0.3,
        bias_gain: 0.2,
        cycles: 4,
        tolerance: 0.1,
        timeout_s: 20.0,
    };
    flight_loop.start_tuning(TunedLoop::YawRate, relay);
    flight_loop.step(Duration::from_millis(4));

    let running = Some((
        TunedLoop::YawRate,
        TuningStatus::Running {
            relay: 0.05,
            cycles: 0,
        },
    ));
    check!(flight_loop.tuning() == running);
    check!(flight_loop.stop_tuning() == running);
    check!(flight_loop.tuning() == None);
}

/// End to end with the copter pipeline: full throttle while disarmed leaves every motor at 0;
/// arming spins them.
#[test]
fn disarmed_copter_keeps_motors_off_at_full_throttle() {
    use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    use crate::mixers::QuadCopterMixer;
    use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    use crate::testing::{ConstantCommander, NullVehicle};

    let full_throttle = ControlInput::Attitude(AttitudeCommand {
        throttle: 1.0,
        ..Default::default()
    });
    for (arm, spins) in [(false, false), (true, true)] {
        let mut flight_loop = FlightLoop::new(
            ConstantCommander::new(Command::new(arm, full_throttle)),
            NullVehicle::new(),
            CopterGuidance::new(CopterGuidanceConfig::default()),
            CopterStabilizer::new(CopterStabilizerConfig::default()),
            QuadCopterMixer::default(),
            (),
        );
        flight_loop.prime();
        flight_loop.step(Duration::from_millis(4));

        let (_, vehicle, ..) = flight_loop.into_parts();
        assert2::assert!(let Some(actuators) = vehicle.last_actuation());
        check!(
            actuators.motors.iter().any(|&m| m > 0.0) == spins,
            "arm={arm}"
        );
    }
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
            "command dt=0.004 seen_heading=0 arm=true",
            "command dt=0.004 seen_heading=1 arm=true",
            "command dt=0.004 seen_heading=2 arm=true",
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
    check!(vehicle.state_reads() == 3); // prime + 2 ticks
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
        ConstantCommander::new(Command::armed(ControlInput::default())),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        (),
    );
    flight_loop.prime();
    flight_loop.step(Duration::from_millis(4));

    let mut guidance = *flight_loop.guidance().config();
    guidance.max_tilt_rad = 0.4;
    check!(flight_loop.set_guidance_config(guidance).is_ok());
    let mut stabilizer = *flight_loop.stabilizer().config();
    stabilizer.body_rate.roll_gains.p = 0.2;
    check!(flight_loop.set_stabilizer_config(stabilizer).is_ok());

    let mut bad_guidance = guidance;
    bad_guidance.max_tilt_rad = 0.0;
    check!(flight_loop.set_guidance_config(bad_guidance).is_err());
    let mut bad_stabilizer = stabilizer;
    bad_stabilizer.thrust_curve_exponent = -1.0;
    check!(flight_loop.set_stabilizer_config(bad_stabilizer).is_err());

    check!(*flight_loop.guidance().config() == guidance);
    check!(*flight_loop.stabilizer().config() == stabilizer);
    flight_loop.step(Duration::from_millis(4));
}

/// Reports fixed saturation flags from guidance, the stabilizer, or the mixer.
struct Flagged(Saturation);
impl Guidance for Flagged {
    fn guide(&mut self, _dt: f32, _input: &ControlInput, _state: &VehicleState) -> Setpoint {
        Setpoint::Attitude(AttitudeSetpoint::default())
    }
    fn saturation(&self) -> Saturation {
        self.0
    }
}
impl Stabilizer for Flagged {
    fn stabilize(
        &mut self,
        _dt: f32,
        _setpoint: &Setpoint,
        _state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput::default()
    }
    fn saturation(&self) -> Saturation {
        self.0
    }
}
impl Mixer for Flagged {
    fn mix(&mut self, _controls: &ControlOutput) -> ActuatorCommand {
        ActuatorCommand::default()
    }
    fn saturation(&self) -> Saturation {
        self.0
    }
}

/// The record carries the union of what guidance, the stabilizer, and the mixer report on an
/// armed tick, and no saturation on a disarmed one (none of them ran).
#[test]
fn record_merges_the_saturation_each_stage_reports() {
    use crate::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};

    let guidance = Saturation {
        tilt: true,
        ..Default::default()
    };
    let stabilizer = Saturation {
        moments: [false, true, false],
        ..Default::default()
    };
    let mixer = Saturation {
        mixer_thrust: true,
        ..Default::default()
    };
    let merged = guidance.merge(stabilizer).merge(mixer);

    for (arm, expected) in [(true, merged), (false, Saturation::default())] {
        let mut flight_loop = FlightLoop::new(
            ConstantCommander::new(Command::new(arm, ControlInput::default())),
            NullVehicle::new(),
            Flagged(guidance),
            Flagged(stabilizer),
            Flagged(mixer),
            RecordingTelemetry::<1>::new(),
        );
        flight_loop.prime();
        flight_loop.step(Duration::from_millis(4));

        let (.., telemetry) = flight_loop.into_parts();
        assert2::assert!(let Some(tick) = telemetry.iter().last());
        check!(tick.saturation == expected, "arm={arm}");
    }
    check!(merged.tilt && merged.moments == [false, true, false] && merged.mixer_thrust);
    check!(!merged.thrust && !merged.mixer_moments);
}

/// Reports a fixed failsafe state, flying a disarmed default command.
struct ReportsFailsafe(Option<FailsafeState>);

impl Commander for ReportsFailsafe {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        Command::default()
    }

    fn failsafe(&self) -> Option<FailsafeState> {
        self.0
    }
}

#[test]
fn record_carries_the_commanders_failsafe_state() {
    use crate::mixers::QuadCopterMixer;
    use crate::testing::{NullVehicle, RecordingTelemetry};

    for failsafe in [
        None,
        Some(FailsafeState::Primary),
        Some(FailsafeState::Fallback),
    ] {
        let mut flight_loop = FlightLoop::new(
            ReportsFailsafe(failsafe),
            NullVehicle::new(),
            Flagged(Saturation::default()),
            Flagged(Saturation::default()),
            QuadCopterMixer::default(),
            RecordingTelemetry::<1>::new(),
        );
        flight_loop.prime();
        flight_loop.step(Duration::from_millis(4));

        let (.., telemetry) = flight_loop.into_parts();
        assert2::assert!(let Some(tick) = telemetry.iter().last());
        check!(tick.failsafe == failsafe, "{failsafe:?}");
    }
}
