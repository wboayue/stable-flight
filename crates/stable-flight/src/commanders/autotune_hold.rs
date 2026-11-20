//! The position hold a copter autotune flies in when a pilot or ground station starts it:
//! [`AutotuneHold`].

use crate::controllers::{InvalidConfig, check, check_positive};
use crate::{Command, Commander, ControlInput, FailsafeState, VehicleState};

use super::{DEFAULT_HOLD_SPEED_MPS, Fallback, HoldCommander};

#[cfg(test)]
mod tests;

/// [`AutotuneHoldConfig::override_stick`]'s default: PX4's override threshold
/// (`McAutotuneAttitudeControl::updateStateMachine` aborts past 0.05 of roll or pitch stick).
pub const DEFAULT_OVERRIDE_STICK: f32 = 0.05;

/// [`AutotuneHold`] settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutotuneHoldConfig {
    /// The hold's top speed back to its hold point after drift, in m/s.
    pub hold_speed_mps: f32,
    /// How far, normalized to full stick, a roll or pitch stick moves off center before the
    /// pilot takes over. Lower: a lighter touch takes over, and stick noise can too.
    /// In `(0, 1]`.
    pub override_stick: f32,
}

impl Default for AutotuneHoldConfig {
    /// [`DEFAULT_HOLD_SPEED_MPS`] and [`DEFAULT_OVERRIDE_STICK`].
    fn default() -> Self {
        Self {
            hold_speed_mps: DEFAULT_HOLD_SPEED_MPS,
            override_stick: DEFAULT_OVERRIDE_STICK,
        }
    }
}

impl AutotuneHoldConfig {
    /// Checks the fields' ranges.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("hold_speed_mps", self.hold_speed_mps)?;
        check(
            "override_stick",
            self.override_stick > 0.0 && self.override_stick <= 1.0,
            "in (0, 1]",
        )
    }
}

/// Wraps the commander a copter normally flies (`inner`, e.g. a link's failsafe commander)
/// and, while [engaged](Self::engage), holds the vehicle where it was instead, so an autotune
/// probes a still vehicle whatever the sticks' throttle was. A ground station's virtual
/// joystick flies attitude and throttle, which holds no height; ArduPilot's AUTOTUNE mode
/// holds the same way.
///
/// `inner` keeps running every tick, so its own state (a failsafe's link timing) stays
/// current, and its [`failsafe`](Commander::failsafe) state is reported as its own. Its
/// command flies instead of the hold, and the hold lets go
/// ([`take_override`](Self::take_override) reports it), when it:
///
/// - disarms;
/// - lands (a pilot's land, or a failsafe's landing fallback);
/// - moves a roll or pitch stick past [`AutotuneHoldConfig::override_stick`] (PX4's rule),
///   measured from center.
///
/// Only those take over: an offboard input (a waypoint, an absolute attitude or rate) never
/// ends the hold, so a companion's setpoints wait until it lets go. A run that ends leaves the
/// hold engaged; [`tuning::autotune::held`](crate::tuning::autotune::held) reverts a run the
/// pilot took over mid-run, and a disarm reverts it anyway.
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::{AutotuneHold, AutotuneHoldConfig};
/// use stable_flight::{AttitudeCommand, Command, Commander, ControlInput, VehicleState};
/// # use stable_flight::testing::ConstantCommander;
///
/// let sticks = |roll| Command::armed(ControlInput::Attitude(AttitudeCommand {
///     throttle: 0.4,
///     roll,
///     ..Default::default()
/// }));
/// let mut hold = AutotuneHold::new(ConstantCommander::new(sticks(0.0)), AutotuneHoldConfig::default())?;
/// let state = VehicleState { position_m: [3.0, 4.0, 10.0].into(), ..Default::default() };
/// hold.next_command(0.004, &state);
///
/// hold.engage(&state);
/// let ControlInput::Waypoint(waypoint) = hold.next_command(0.004, &state).input else {
///     panic!("a hold")
/// };
/// assert_eq!(waypoint.altitude_m, 10.0);
/// assert!(!hold.take_override());
/// # Ok::<(), stable_flight::controllers::InvalidConfig>(())
/// ```
#[derive(Debug, Clone)]
pub struct AutotuneHold<C> {
    inner: C,
    config: AutotuneHoldConfig,
    hold: HoldCommander,
    engaged: bool,
    overridden: bool,
    /// `inner`'s command on the last tick: the arm state the hold latches.
    last: Command,
}

impl<C: Commander> AutotuneHold<C> {
    /// Flies `inner` until engaged.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] if `config` fails [`AutotuneHoldConfig::validate`].
    pub fn new(inner: C, config: AutotuneHoldConfig) -> Result<Self, InvalidConfig> {
        config.validate()?;
        Ok(Self {
            inner,
            config,
            hold: HoldCommander::new(config.hold_speed_mps)?,
            engaged: false,
            overridden: false,
            last: Command::default(),
        })
    }

    /// Holds where the vehicle is in `state`, armed as `inner` last flew, from the next tick.
    pub fn engage(&mut self, state: &VehicleState) {
        self.hold.engage(state, &self.last);
        self.engaged = true;
        self.overridden = false;
    }

    /// Hands the vehicle back to `inner` (the run was stopped).
    pub fn release(&mut self) {
        self.engaged = false;
    }

    /// Whether the hold flies.
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Whether `inner` took over since the last call (see the type), clearing the flag.
    pub fn take_override(&mut self) -> bool {
        core::mem::take(&mut self.overridden)
    }

    /// The wrapped commander.
    pub fn inner(&self) -> &C {
        &self.inner
    }

    /// Whether `command` takes the vehicle back from the hold.
    fn overrides(&self, command: &Command) -> bool {
        let past = |stick: f32| stick.abs() > self.config.override_stick;
        if !command.arm {
            return true;
        }
        match command.input {
            ControlInput::Land(_) => true,
            ControlInput::Attitude(sticks) => past(sticks.roll) || past(sticks.pitch),
            ControlInput::Rate(sticks) => past(sticks.roll_rate) || past(sticks.pitch_rate),
            _ => false,
        }
    }
}

impl<C: Commander> Commander for AutotuneHold<C> {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        let command = self.inner.next_command(dt, state);
        self.last = command;
        if !self.engaged {
            return command;
        }
        // A NaN stick compares false: neutral, per the input policy.
        if self.overrides(&command) {
            self.engaged = false;
            self.overridden = true;
            return command;
        }
        Command {
            regime: command.regime,
            ..self.hold.next_command(dt, state)
        }
    }

    fn failsafe(&self) -> Option<FailsafeState> {
        self.inner.failsafe()
    }
}
