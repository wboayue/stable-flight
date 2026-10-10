//! Falling back when a link-fed commander goes quiet: see [`FailsafeCommander`].
//!
//! A companion computer's link drops (late USB enumeration, a brownout, a crashed bridge), and
//! the flight controller, not the companion, decides what happens then. The primary commander
//! reports how long ago it last heard the link ([`LinkedCommander`]); when that exceeds a
//! timeout, a [`Fallback`] such as [`HoldCommander`] flies until the link has been back long
//! enough.

use crate::controllers::{InvalidConfig, check_limit, check_positive};
use crate::{Command, Commander, ControlInput, FailsafeState, Vec2, VehicleState, WaypointCommand};

#[cfg(test)]
mod tests;

/// A [`Commander`] fed by a link, which knows how long ago it last heard it.
pub trait LinkedCommander: Commander {
    /// Seconds since the last valid message, as of the last
    /// [`next_command`](Commander::next_command) call; `None` if none has arrived. Measured on
    /// this side from the `dt` the commander is given, so it needs no clock from the sender.
    fn since_update_s(&self) -> Option<f32>;

    /// Sets the arm state this commander reports to `armed`. A [`FailsafeCommander`] calls it
    /// when it hands control back, with the vehicle's actual arm state, so an arm request this
    /// commander latched while the fallback flew (and that the vehicle never acted on) is
    /// dropped rather than arming the vehicle at the handover.
    fn sync_arm(&mut self, armed: bool);
}

/// A [`Commander`] that flies while a [`FailsafeCommander`]'s primary is stale.
pub trait Fallback: Commander {
    /// The failsafe switched to this fallback, with the vehicle in `state` and `armed` as the
    /// last command left it. Called once per switch, before this tick's
    /// [`next_command`](Commander::next_command). The default does nothing.
    fn engage(&mut self, state: &VehicleState, armed: bool) {
        let _ = (state, armed);
    }

    /// Whether the primary may take over again once fresh. `false` keeps the fallback flying,
    /// e.g. a landing that must finish (#66). The default always releases.
    fn can_release(&self) -> bool {
        true
    }
}

/// When a [`FailsafeCommander`] falls back, and when it returns.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FailsafeConfig {
    /// The primary is stale once this many seconds pass without a message. Must be positive
    /// and finite.
    pub timeout_s: f32,
    /// The primary takes over again only after it has been fresh this long without a break,
    /// so a flapping link doesn't toggle control. Must be non-negative and finite.
    pub recover_after_s: f32,
    /// How far a stick axis (normalized) must move before the primary's sticks take back an
    /// armed vehicle from the fallback, so a forgotten joystick resending the same positions
    /// can't drop the vehicle out of a hold. Must be positive and finite.
    pub takeover_stick_delta: f32,
}

impl Default for FailsafeConfig {
    /// Fall back after 1 s without a message; return after 1 s of messages, sticks once one axis
    /// has moved 0.1.
    fn default() -> Self {
        Self {
            timeout_s: 1.0,
            recover_after_s: 1.0,
            takeover_stick_delta: 0.1,
        }
    }
}

impl FailsafeConfig {
    /// Checks the documented requirements of every field.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("timeout_s", self.timeout_s)?;
        check_limit("recover_after_s", self.recover_after_s)?;
        check_positive("takeover_stick_delta", self.takeover_stick_delta)
    }
}

/// Flies a link-fed `primary` while it is fresh, and a `fallback` otherwise.
///
/// - **Starts in the fallback**: until the primary is heard, nothing it says is trusted.
/// - **Falls back** as soon as the primary has gone [`timeout_s`](FailsafeConfig::timeout_s)
///   without a message, [engaging](Fallback::engage) the fallback with the current state and
///   arm state.
/// - **Returns** once the primary has been fresh for
///   [`recover_after_s`](FailsafeConfig::recover_after_s) without a break and the fallback
///   [can release](Fallback::can_release). To an **armed** vehicle, sticks (an attitude or rate
///   command) take over only once an axis has moved
///   [`takeover_stick_delta`](FailsafeConfig::takeover_stick_delta) from where it was when the
///   primary recovered, so a joystick left at low throttle can't drop the vehicle out of a hold;
///   a waypoint, or anything to a disarmed vehicle, takes over at once.
/// - **At the handover**, the primary's arm state is [synced](LinkedCommander::sync_arm) to the
///   vehicle's (disarmed if either the vehicle or the primary is), so an arm request made while
///   the fallback flew doesn't arm the vehicle; a new one does.
/// - **Arming:** the fallback decides the arm state while it flies (a [`HoldCommander`] keeps
///   it and never arms), except that a fresh primary's disarm is always honored, even while it
///   waits to take over.
///
/// The primary runs every tick, flying or not, so it keeps draining its link and timing its
/// freshness. [`failsafe`](Commander::failsafe) reports which commander flew.
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::{
///     FailsafeCommander, FailsafeConfig, HoldCommander, LinkedCommander,
/// };
/// use stable_flight::{Command, Commander, FailsafeState, VehicleState};
///
/// /// A link that was last heard `since_s` ago.
/// struct Link {
///     since_s: Option<f32>,
/// }
///
/// impl Commander for Link {
///     fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
///         Command::default()
///     }
/// }
///
/// impl LinkedCommander for Link {
///     fn since_update_s(&self) -> Option<f32> {
///         self.since_s
///     }
///
///     fn sync_arm(&mut self, _armed: bool) {}
/// }
///
/// let link = Link { since_s: None };
/// let mut commander =
///     FailsafeCommander::new(link, HoldCommander::default(), FailsafeConfig::default())?;
/// commander.next_command(0.004, &VehicleState::default());
/// assert_eq!(commander.failsafe(), Some(FailsafeState::Fallback)); // never heard
/// # Ok::<(), stable_flight::controllers::InvalidConfig>(())
/// ```
#[derive(Debug)]
pub struct FailsafeCommander<P, F> {
    primary: P,
    fallback: F,
    config: FailsafeConfig,
    /// Which commander flew the last tick; `None` before the first.
    state: Option<FailsafeState>,
    /// How long the primary has been fresh without a break, while the fallback flies.
    fresh_for_s: f32,
    /// The arm state of the last command, to engage the fallback with.
    armed: bool,
    /// The primary's sticks when it recovered over an armed vehicle, waiting to move before
    /// they take over.
    takeover_reference: Option<ControlInput>,
}

impl<P: LinkedCommander, F: Fallback> FailsafeCommander<P, F> {
    /// Flies `primary`, falling back to `fallback` per `config`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] if `config` fails [`FailsafeConfig::validate`].
    pub fn new(primary: P, fallback: F, config: FailsafeConfig) -> Result<Self, InvalidConfig> {
        config.validate()?;
        Ok(Self {
            primary,
            fallback,
            config,
            state: None,
            fresh_for_s: 0.0,
            armed: false,
            takeover_reference: None,
        })
    }

    /// The primary commander.
    pub fn primary(&self) -> &P {
        &self.primary
    }

    /// The fallback commander.
    pub fn fallback(&self) -> &F {
        &self.fallback
    }

    /// Switches to the fallback.
    fn fall_back(&mut self, state: &VehicleState) {
        self.state = Some(FailsafeState::Fallback);
        self.fresh_for_s = 0.0;
        self.takeover_reference = None;
        self.fallback.engage(state, self.armed);
    }

    /// Whether the recovered primary, flying `input`, may take over now: at once, unless it
    /// flies sticks over an armed vehicle, which must first move.
    fn may_take_over(&mut self, input: &ControlInput) -> bool {
        if !self.armed || !input.is_sticks() {
            return true;
        }
        let reference = *self.takeover_reference.get_or_insert(*input);
        sticks_moved(&reference, input, self.config.takeover_stick_delta)
    }

    /// Decides who flies this tick, given whether the primary is `fresh` and what it flies.
    /// Returns whether the primary took over this tick.
    fn update_state(
        &mut self,
        fresh: bool,
        dt: f32,
        input: &ControlInput,
        state: &VehicleState,
    ) -> bool {
        match self.state {
            Some(FailsafeState::Primary) if fresh => return false,
            None | Some(FailsafeState::Primary) => self.fall_back(state),
            _ => {}
        }
        // The fallback flies: is the primary back, and may it take over?
        self.fresh_for_s = if fresh { self.fresh_for_s + dt } else { 0.0 };
        let recovered =
            fresh && self.fresh_for_s >= self.config.recover_after_s && self.fallback.can_release();
        if !recovered {
            self.state = Some(FailsafeState::Fallback);
            self.takeover_reference = None;
            return false;
        }
        if !self.may_take_over(input) {
            self.state = Some(FailsafeState::AwaitingSticks);
            return false;
        }
        self.state = Some(FailsafeState::Primary);
        self.takeover_reference = None;
        true
    }
}

impl<P: LinkedCommander, F: Fallback> Commander for FailsafeCommander<P, F> {
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command {
        let primary = self.primary.next_command(dt, current_state);
        let fresh = self
            .primary
            .since_update_s()
            .is_some_and(|s| s <= self.config.timeout_s);

        let handed_over = self.update_state(fresh, dt, &primary.input, current_state);

        let command = if self.state == Some(FailsafeState::Primary) {
            if handed_over {
                // Drop an arm the primary latched while the fallback flew; keep its disarm.
                let armed = self.armed && primary.arm;
                self.primary.sync_arm(armed);
                Command::new(armed, primary.input)
            } else {
                primary
            }
        } else {
            let mut command = self.fallback.next_command(dt, current_state);
            // A disarm from any source is honored.
            command.arm &= !fresh || primary.arm;
            command
        };
        self.armed = command.arm;
        command
    }

    fn failsafe(&self) -> Option<FailsafeState> {
        self.state
    }
}

/// Whether the sticks in `now` moved more than `delta` on any axis from `reference`; a change of
/// kind (attitude ↔ rate) counts. A NaN axis never counts as moved.
fn sticks_moved(reference: &ControlInput, now: &ControlInput, delta: f32) -> bool {
    let moved = |a: f32, b: f32| (a - b).abs() > delta;
    match (reference, now) {
        (ControlInput::Attitude(a), ControlInput::Attitude(b)) => {
            moved(a.throttle, b.throttle)
                || moved(a.roll, b.roll)
                || moved(a.pitch, b.pitch)
                || moved(a.yaw, b.yaw)
        }
        (ControlInput::Rate(a), ControlInput::Rate(b)) => {
            moved(a.throttle, b.throttle)
                || moved(a.roll_rate, b.roll_rate)
                || moved(a.pitch_rate, b.pitch_rate)
                || moved(a.yaw_rate, b.yaw_rate)
        }
        _ => true,
    }
}

/// [`HoldCommander`]'s default speed back to the hold point, in m/s.
pub const DEFAULT_HOLD_SPEED_MPS: f32 = 2.0;

/// A [`Fallback`] that holds position: when engaged, it latches where the vehicle is
/// (horizontal position and altitude) and its arm state, then flies a waypoint there, at most
/// [`cruise_speed_mps`](Self::new) back to it after any drift. It never arms a disarmed
/// vehicle.
///
/// It needs a position estimate. While the position isn't finite it retries the latch each
/// tick, flying a NaN waypoint meanwhile, which guidance treats as level at hover thrust (the
/// input policy). A vehicle without position needs a landing fallback instead (#66).
///
/// # Examples
///
/// ```
/// use stable_flight::commanders::{Fallback, HoldCommander};
/// use stable_flight::{Commander, ControlInput, VehicleState};
///
/// let state = VehicleState { position_m: [3.0, 4.0, 10.0].into(), ..Default::default() };
/// let mut hold = HoldCommander::default();
/// hold.engage(&state, true);
///
/// let command = hold.next_command(0.004, &VehicleState::default());
/// assert!(command.arm);
/// let ControlInput::Waypoint(waypoint) = command.input else { panic!("a waypoint") };
/// assert_eq!((waypoint.position_m.x, waypoint.altitude_m), (3.0, 10.0));
/// ```
#[derive(Debug, Clone)]
pub struct HoldCommander {
    cruise_speed_mps: f32,
    hold: Option<WaypointCommand>,
    armed: bool,
}

impl Default for HoldCommander {
    /// Holds at [`DEFAULT_HOLD_SPEED_MPS`].
    fn default() -> Self {
        Self {
            cruise_speed_mps: DEFAULT_HOLD_SPEED_MPS,
            hold: None,
            armed: false,
        }
    }
}

impl HoldCommander {
    /// Holds, flying back to the hold point at most `cruise_speed_mps`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] unless `cruise_speed_mps` is positive and finite.
    pub fn new(cruise_speed_mps: f32) -> Result<Self, InvalidConfig> {
        check_positive("cruise_speed_mps", cruise_speed_mps)?;
        Ok(Self {
            cruise_speed_mps,
            ..Self::default()
        })
    }

    /// Latches `state`'s position as the hold point, if it is finite.
    fn latch(&mut self, state: &VehicleState) {
        let p = state.position_m;
        self.hold = p.iter().all(|c| c.is_finite()).then(|| WaypointCommand {
            position_m: p.xy(),
            altitude_m: p.z,
            cruise_speed_mps: self.cruise_speed_mps,
            ..Default::default()
        });
    }
}

impl Commander for HoldCommander {
    fn next_command(&mut self, _dt: f32, current_state: &VehicleState) -> Command {
        if self.hold.is_none() {
            self.latch(current_state);
        }
        let waypoint = self.hold.unwrap_or(WaypointCommand {
            position_m: Vec2::repeat(f32::NAN),
            altitude_m: f32::NAN,
            cruise_speed_mps: self.cruise_speed_mps,
            ..Default::default()
        });
        Command::new(self.armed, ControlInput::Waypoint(waypoint))
    }
}

impl Fallback for HoldCommander {
    fn engage(&mut self, state: &VehicleState, armed: bool) {
        self.armed = armed;
        self.latch(state);
    }
}
