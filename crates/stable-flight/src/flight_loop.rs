//! One control-loop tick, independent of how ticks are scheduled.
//!
//! [`FlightLoop`] owns the six pipeline components and runs them in order for a single tick.
//! It has no clock, threads, or signal handling: a runner decides when to call
//! [`FlightLoop::step`] and with what `dt` (a wall-clock runner in `stable-flight-sitl` today;
//! a lockstep simulator runner or a hardware timer interrupt later).

use core::time::Duration;

use crate::controllers::InvalidConfig;
use crate::{
    ActuatorCommand, AttitudeSetpoint, Commander, Configurable, ControlOutput, Guidance, Mixer,
    Setpoint, Stabilizer, TelemetryPublisher, TelemetryRecord, Vehicle, VehicleState,
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
///     StateEstimator, VehicleState,
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
/// impl StateEstimator for StillAir {
///     fn estimate_state(&mut self, _dt: Duration) -> VehicleState {
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

    /// Takes the initial estimate (with `dt = Duration::ZERO`, per the [`StateEstimator`]
    /// convention) and primes guidance, then the stabilizer, from it. Call once before the
    /// first tick.
    ///
    /// [`StateEstimator`]: crate::StateEstimator
    pub fn prime(&mut self) {
        self.state = self.vehicle.estimate_state(Duration::ZERO);
        self.guidance.prime(&self.state);
        self.stabilizer.prime(&self.state);
    }

    /// Runs one tick with loop period `dt`.
    ///
    /// The commander sees the state from the previous tick; guidance and the stabilizer see
    /// the fresh estimate. Telemetry receives a [`TelemetryRecord`] after actuation. A
    /// disarmed tick actuates [`ActuatorCommand::default`] and records a neutral setpoint and
    /// zero control; see [`FlightLoop`] for the arm gate. Allocation-free.
    ///
    /// # Panics
    ///
    /// If `dt` is zero on an armed tick: guidance and the stabilizer require a positive, finite step (a
    /// `Duration` is never negative or non-finite).
    pub fn step(&mut self, dt: Duration) {
        let dt_secs = dt.as_secs_f32();

        let command = self.commander.next_command(dt_secs, &self.state);
        self.state = self.vehicle.estimate_state(dt);

        let (setpoint, control, actuators) = if command.arm {
            if !self.armed {
                self.guidance.prime(&self.state);
                self.stabilizer.prime(&self.state);
            }
            let setpoint = self.guidance.guide(dt_secs, &command.input, &self.state);
            let control = self.stabilizer.stabilize(dt_secs, &setpoint, &self.state);
            let actuators = self.mixer.mix(&control);
            (setpoint, control, actuators)
        } else {
            let setpoint = Setpoint::Attitude(AttitudeSetpoint {
                heading_rad: self.state.heading_rad,
                ..Default::default()
            });
            (
                setpoint,
                ControlOutput::default(),
                ActuatorCommand::default(),
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
        });
    }

    /// The most recent state estimate.
    pub fn state(&self) -> &VehicleState {
        &self.state
    }

    /// The guidance, e.g. to read its config.
    pub fn guidance(&self) -> &G {
        &self.guidance
    }

    /// The stabilizer, e.g. to read its config.
    pub fn stabilizer(&self) -> &S {
        &self.stabilizer
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
    /// # use stable_flight::{Actuator, ActuatorCommand, Command, Commander, StateEstimator, VehicleState};
    /// use stable_flight::{Configurable, FlightLoop};
    /// # struct Hover;
    /// # impl Commander for Hover {
    /// #     fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
    /// #         Command::default()
    /// #     }
    /// # }
    /// # struct StillAir;
    /// # impl StateEstimator for StillAir {
    /// #     fn estimate_state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
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
    /// retuned.body_rate.gains.p = 0.2;
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
    /// # use stable_flight::{Actuator, ActuatorCommand, Command, Commander, StateEstimator, VehicleState};
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
    /// # impl StateEstimator for Bench {
    /// #     fn estimate_state(&mut self, _: Duration) -> VehicleState { VehicleState::default() }
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
