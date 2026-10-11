//! One control-loop tick, independent of how ticks are scheduled.
//!
//! [`FlightLoop`] owns the six pipeline components and runs them in order for a single tick.
//! It has no clock, threads, or signal handling: a runner decides when to call
//! [`FlightLoop::step`] and with what `dt` (a wall-clock runner in `stable-flight-sitl` today;
//! a lockstep simulator runner or a hardware timer interrupt later).

use core::time::Duration;

use crate::controllers::InvalidConfig;
use crate::params::{ParamError, Params};
use crate::tuning::{RelayConfig, Tunable, TuningStatus};
use crate::{
    ActuatorCommand, AttitudeSetpoint, Commander, Configurable, ControlOutput, Guidance, Mixer,
    Saturation, Setpoint, Stabilizer, TelemetryPublisher, TelemetryRecord, Vehicle, VehicleState,
};

#[cfg(test)]
mod tests;

/// Sequences one control tick: commander → estimate → guide → stabilize → mix → actuate →
/// telemetry.
///
/// Call [`prime`](FlightLoop::prime) once before the first [`step`](FlightLoop::step).
///
/// The [`Command`](crate::Command)'s `arm` gates the motors: a disarmed tick skips guidance,
/// the stabilizer, and the mixer and actuates [`ActuatorCommand::default`] (motors off). The
/// first armed tick after a disarmed one (including the first tick) re-primes guidance and
/// the stabilizer, so integrators start from zero.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight::mixers::QuadCopterMixer;
/// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// use stable_flight::{
///     Actuator, ActuatorCommand, AttitudeCommand, Command, Commander, ControlInput, FlightLoop,
///     StateSource, VehicleState,
/// };
///
/// struct Hover;
/// impl Commander for Hover {
///     fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
///         Command::armed(ControlInput::Attitude(AttitudeCommand {
///             throttle: 0.5,
///             ..Default::default()
///         }))
///     }
/// }
///
/// struct StillAir;
/// impl StateSource for StillAir {
///     fn state(&mut self, _dt: Duration) -> VehicleState {
///         VehicleState::default()
///     }
/// }
/// impl Actuator for StillAir {
///     fn actuate(&mut self, _command: &ActuatorCommand) {}
/// }
///
/// let mut flight_loop = FlightLoop::new(
///     Hover,
///     StillAir,
///     CopterGuidance::new(CopterGuidanceConfig::default()),
///     CopterStabilizer::new(CopterStabilizerConfig::default()),
///     QuadCopterMixer::default(),
///     (), // no telemetry
/// );
///
/// flight_loop.prime();
/// for _ in 0..3 {
///     flight_loop.step(Duration::from_millis(4)); // 250 Hz
/// }
/// ```
///
/// The struct itself is unbounded (bounds live on the `impl`), so types that hold a
/// `FlightLoop` — runners, firmware statics — needn't repeat the six trait bounds.
pub struct FlightLoop<C, V, G, S, M, T> {
    commander: C,
    vehicle: V,
    guidance: G,
    stabilizer: S,
    mixer: M,
    telemetry: T,
    /// Latest estimate; the commander sees the previous tick's state.
    state: VehicleState,
    /// Whether the previous tick was armed; a rising edge re-primes.
    armed: bool,
}

impl<C, V, G, S, M, T> FlightLoop<C, V, G, S, M, T>
where
    C: Commander,
    V: Vehicle,
    G: Guidance,
    S: Stabilizer,
    M: Mixer,
    T: TelemetryPublisher,
{
    /// Assembles a loop from its six components. Does not touch the vehicle.
    pub fn new(
        commander: C,
        vehicle: V,
        guidance: G,
        stabilizer: S,
        mixer: M,
        telemetry: T,
    ) -> Self {
        FlightLoop {
            commander,
            vehicle,
            guidance,
            stabilizer,
            mixer,
            telemetry,
            state: VehicleState::default(),
            armed: false,
        }
    }

    /// Takes the initial estimate (with `dt = Duration::ZERO`, per the [`StateSource`]
    /// convention) and primes guidance, then the stabilizer, from it. Call once before the
    /// first tick, so the commander's first call sees a real state. The first armed tick
    /// primes them again from its fresh estimate (see the arm gate on [`FlightLoop`]).
    ///
    /// [`StateSource`]: crate::StateSource
    pub fn prime(&mut self) {
        self.state = self.vehicle.state(Duration::ZERO);
        self.guidance.prime(&self.state);
        self.stabilizer.prime(&self.state);
    }

    /// Runs one tick with loop period `dt`.
    ///
    /// The commander sees the state from the previous tick; guidance and the stabilizer see
    /// the fresh estimate. Telemetry receives a [`TelemetryRecord`] after actuation, with the
    /// [`Saturation`] guidance, the stabilizer, and the mixer report, merged. A disarmed tick
    /// actuates [`ActuatorCommand::default`] and records a neutral setpoint, zero control, and
    /// no saturation; see [`FlightLoop`] for the arm gate. Allocation-free.
    ///
    /// # Panics
    ///
    /// If `dt` is zero on an armed tick: guidance and the stabilizer require a positive, finite step (a
    /// `Duration` is never negative or non-finite).
    pub fn step(&mut self, dt: Duration) {
        let dt_secs = dt.as_secs_f32();

        let command = self.commander.next_command(dt_secs, &self.state);
        self.state = self.vehicle.state(dt);

        let (setpoint, control, actuators, saturation) = if command.arm {
            if !self.armed {
                self.guidance.prime(&self.state);
                self.stabilizer.prime(&self.state);
            }
            let setpoint = self.guidance.guide(dt_secs, &command.input, &self.state);
            let control = self.stabilizer.stabilize(dt_secs, &setpoint, &self.state);
            let actuators = self.mixer.mix(&control);
            let saturation = self
                .guidance
                .saturation()
                .merge(self.stabilizer.saturation())
                .merge(self.mixer.saturation());
            (setpoint, control, actuators, saturation)
        } else {
            let setpoint = Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: self.state.heading_rad,
                ..Default::default()
            });
            (
                setpoint,
                ControlOutput::default(),
                ActuatorCommand::default(),
                Saturation::default(),
            )
        };
        self.armed = command.arm;
        self.vehicle.actuate(&actuators);

        self.telemetry.publish(&TelemetryRecord {
            dt,
            command: &command,
            setpoint: &setpoint,
            state: &self.state,
            control: &control,
            actuators: &actuators,
            saturation,
            failsafe: self.commander.failsafe(),
        });
    }

    /// The most recent state estimate.
    pub fn state(&self) -> &VehicleState {
        &self.state
    }

    /// The commander, e.g. to read its [`failsafe`](Commander::failsafe) state between ticks.
    pub fn commander(&self) -> &C {
        &self.commander
    }

    /// The guidance, e.g. to read its config.
    pub fn guidance(&self) -> &G {
        &self.guidance
    }

    /// The stabilizer, e.g. to read its config.
    pub fn stabilizer(&self) -> &S {
        &self.stabilizer
    }

    /// Whether the last tick was armed (false before the first tick). A disarm between ticks
    /// shows here after the next tick.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Retunes the guidance between ticks ([`Configurable::set_config`]): an invalid config is
    /// rejected and the current one kept.
    pub fn set_guidance_config(&mut self, config: G::Config) -> Result<(), InvalidConfig>
    where
        G: Configurable,
    {
        self.guidance.set_config(config)
    }

    /// Retunes the stabilizer between ticks ([`Configurable::set_config`]): an invalid config
    /// is rejected and the current one kept.
    ///
    /// ```
    /// # use std::time::Duration;
    /// # use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    /// # use stable_flight::mixers::QuadCopterMixer;
    /// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    /// # use stable_flight::{Actuator, ActuatorCommand, Command, Commander, StateSource, VehicleState};
    /// use stable_flight::{Configurable, FlightLoop};
    /// # struct Hover;
    /// # impl Commander for Hover {
    /// #     fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
    /// #         Command::default()
    /// #     }
    /// # }
    /// # struct StillAir;
    /// # impl StateSource for StillAir {
    /// #     fn state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
    /// # }
    /// # impl Actuator for StillAir {
    /// #     fn actuate(&mut self, _: &ActuatorCommand) {}
    /// # }
    /// # let mut flight_loop = FlightLoop::new(
    /// #     Hover,
    /// #     StillAir,
    /// #     CopterGuidance::new(CopterGuidanceConfig::default()),
    /// #     CopterStabilizer::new(CopterStabilizerConfig::default()),
    /// #     QuadCopterMixer::default(),
    /// #     (),
    /// # );
    /// # flight_loop.prime();
    /// flight_loop.step(Duration::from_millis(4));
    ///
    /// let mut retuned = *flight_loop.stabilizer().config();
    /// retuned.body_rate.roll_gains.p = 0.2;
    /// flight_loop.set_stabilizer_config(retuned).expect("valid config");
    /// flight_loop.step(Duration::from_millis(4)); // flies the new gains
    /// ```
    pub fn set_stabilizer_config(&mut self, config: S::Config) -> Result<(), InvalidConfig>
    where
        S: Configurable,
    {
        self.stabilizer.set_config(config)
    }

    /// Splits the loop back into `(commander, vehicle, guidance, stabilizer, mixer, telemetry)`,
    /// e.g. to inspect test doubles after a run or to hand the vehicle to another runner.
    ///
    /// ```
    /// # use std::time::Duration;
    /// # use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    /// # use stable_flight::mixers::QuadCopterMixer;
    /// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    /// # use stable_flight::{Actuator, ActuatorCommand, Command, Commander, StateSource, VehicleState};
    /// use stable_flight::FlightLoop;
    ///
    /// # struct Hover;
    /// # impl Commander for Hover {
    /// #     fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
    /// #         Command::default()
    /// #     }
    /// # }
    /// /// Counts actuations.
    /// #[derive(Default)]
    /// struct Bench {
    ///     actuations: u32,
    /// }
    /// # impl StateSource for Bench {
    /// #     fn state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
    /// # }
    /// impl Actuator for Bench {
    ///     fn actuate(&mut self, _: &ActuatorCommand) {
    ///         self.actuations += 1;
    ///     }
    /// }
    ///
    /// let mut flight_loop = FlightLoop::new(
    ///     Hover,
    ///     Bench::default(),
    ///     CopterGuidance::new(CopterGuidanceConfig::default()),
    ///     CopterStabilizer::new(CopterStabilizerConfig::default()),
    ///     QuadCopterMixer::default(),
    ///     (),
    /// );
    /// flight_loop.prime();
    /// flight_loop.step(Duration::from_millis(4));
    ///
    /// let (_, bench, _, _, _, _) = flight_loop.into_parts();
    /// assert_eq!(bench.actuations, 1);
    /// ```
    pub fn into_parts(self) -> (C, V, G, S, M, T) {
        (
            self.commander,
            self.vehicle,
            self.guidance,
            self.stabilizer,
            self.mixer,
            self.telemetry,
        )
    }
}

/// The guidance's params, then the stabilizer's (see [`crate::params`]): combined indices
/// run through the guidance's first, then the stabilizer's. Nothing checks that their names
/// are disjoint: on a duplicate, name lookup finds the guidance's and the stabilizer's is
/// reachable only by index, so table authors keep names unique across components (the copter
/// tables use `GUID_`/`STAB_` prefixes).
///
/// ```
/// # use std::time::Duration;
/// # use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// # use stable_flight::mixers::QuadCopterMixer;
/// # use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// # use stable_flight::{Actuator, ActuatorCommand, Command, Commander, StateSource, VehicleState};
/// use stable_flight::params::Params;
/// use stable_flight::{Configurable, FlightLoop};
/// # struct Hover;
/// # impl Commander for Hover {
/// #     fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
/// #         Command::default()
/// #     }
/// # }
/// # struct StillAir;
/// # impl StateSource for StillAir {
/// #     fn state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
/// # }
/// # impl Actuator for StillAir {
/// #     fn actuate(&mut self, _: &ActuatorCommand) {}
/// # }
/// # let mut flight_loop = FlightLoop::new(
/// #     Hover,
/// #     StillAir,
/// #     CopterGuidance::new(CopterGuidanceConfig::default()),
/// #     CopterStabilizer::new(CopterStabilizerConfig::default()),
/// #     QuadCopterMixer::default(),
/// #     (),
/// # );
///
/// flight_loop.set_param("GUID_ALT_P", 2.5).expect("valid value");
/// flight_loop.set_param("STAB_RATE_RLL_P", 0.2).expect("valid value");
/// assert_eq!(flight_loop.guidance().config().altitude.gains.p, 2.5);
/// assert_eq!(flight_loop.param("STAB_RATE_RLL_P"), Some(0.2));
/// ```
/// Autotune stages on a [`Tunable`] stabilizer, between ticks; forwarded so the loop hands out
/// no `&mut` to its components.
impl<C, V, G, S, M, T> FlightLoop<C, V, G, S, M, T>
where
    S: Tunable,
{
    /// Starts relay tuning on one of the stabilizer's loops, from the next tick
    /// ([`Tunable::start_tuning`]).
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`RelayConfig::validate`]).
    pub fn start_tuning(&mut self, target: S::Target, config: RelayConfig) {
        self.stabilizer.start_tuning(target, config);
    }

    /// The stabilizer's current stage, if any ([`Tunable::tuning`]).
    pub fn tuning(&self) -> Option<(S::Target, TuningStatus)> {
        self.stabilizer.tuning()
    }

    /// Ends the stage and hands its loop back ([`Tunable::stop_tuning`]).
    pub fn stop_tuning(&mut self) -> Option<(S::Target, TuningStatus)> {
        self.stabilizer.stop_tuning()
    }
}

impl<C, V, G, S, M, T> Params for FlightLoop<C, V, G, S, M, T>
where
    G: Params,
    S: Params,
{
    fn param_count(&self) -> usize {
        self.guidance.param_count() + self.stabilizer.param_count()
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        match self.split_param_index(index) {
            ParamOwner::Guidance(i) => self.guidance.param_name(i),
            ParamOwner::Stabilizer(i) => self.stabilizer.param_name(i),
        }
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        match self.split_param_index(index) {
            ParamOwner::Guidance(i) => self.guidance.param_at(i),
            ParamOwner::Stabilizer(i) => self.stabilizer.param_at(i),
        }
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        match self.split_param_index(index) {
            ParamOwner::Guidance(i) => self.guidance.set_param_at(i, value),
            ParamOwner::Stabilizer(i) => self.stabilizer.set_param_at(i, value),
        }
    }
}

/// The component a combined param index belongs to, with its index there.
enum ParamOwner {
    Guidance(usize),
    Stabilizer(usize),
}

impl<C, V, G: Params, S, M, T> FlightLoop<C, V, G, S, M, T> {
    fn split_param_index(&self, index: usize) -> ParamOwner {
        let guidance_count = self.guidance.param_count();
        if index < guidance_count {
            ParamOwner::Guidance(index)
        } else {
            ParamOwner::Stabilizer(index - guidance_count)
        }
    }
}
