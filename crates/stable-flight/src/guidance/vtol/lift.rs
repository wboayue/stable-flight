//! [`LiftTransition`]: the copter ⇄ plane transition of a lift+cruise VTOL (quadplane), which
//! stays level and flies its lift rotors and its wing at once, trading lift between them.

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::assert_dt;
use crate::{
    Configurable, FlightMode, FlightModeReport, FlightRegime, ModeRefusal, Vec3, VehicleState,
};

use super::ForwardGate;

mod params;
#[cfg(test)]
mod tests;

/// [`LiftTransition`] settings, and the pusher, handoff, and roll limit a quadplane guidance
/// flies the transitions with ([`QuadPlaneGuidance`](super::QuadPlaneGuidance)).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiftTransitionConfig {
    /// The airspeed that starts the forward lift ramp, in m/s.
    pub min_airspeed_mps: f32,
    /// How long the forward ramp takes the lift share from 1 to 0, in seconds.
    pub lift_ramp_s: f32,
    /// The longest a forward transition waits on airspeed before it falls back to copter, in
    /// seconds. A started lift ramp always runs to its end.
    pub timeout_s: f32,
    /// The height above the arming altitude a forward transition needs to start, in meters.
    pub min_height_m: f32,
    /// The height above the arming altitude plane flight falls back to copter below, in
    /// meters; a forward transition also needs it to start. Higher: falls back sooner, with
    /// more height to lose in the back transition (1.9 m at a 3.5 m/s sink on the sim
    /// quadplane). `0` turns it off.
    pub plane_min_height_m: f32,
    /// How long the back ramp takes the lift share from 0 to 1, in seconds.
    pub back_ramp_s: f32,
    /// The forward speed below which the back transition completes, in m/s.
    pub back_speed_mps: f32,
    /// The longest a back transition runs before it completes, in seconds.
    pub back_timeout_s: f32,
    /// The most linear thrust the pusher gets through the forward transition, `[0, 1]`.
    pub forward_thrust: f32,
    /// How fast the pusher's thrust may rise through the forward transition, in thrust per
    /// second. Positive.
    pub pusher_slew_per_s: f32,
    /// How long the pusher's thrust blends to the plane guidance's at the handoff into plane
    /// flight, in seconds ([`ThrustHandoff`](super::ThrustHandoff)); 0 switches at once.
    pub handoff_s: f32,
    /// Bank past which a transition is cut short, in radians, in `(0, π]`: forward aborts to
    /// copter, back completes ([`LiftTransition::cut_short`]).
    pub roll_limit_rad: f32,
}

impl LiftTransitionConfig {
    /// `Ok` when every field is in range: times, ramps, and the slew positive, airspeed, speed,
    /// height, and handoff non-negative, all finite; the thrust in `[0, 1]` and the roll limit
    /// in `(0, π]`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("min_airspeed_mps", self.min_airspeed_mps)?;
        check_positive("lift_ramp_s", self.lift_ramp_s)?;
        check_positive("timeout_s", self.timeout_s)?;
        check_limit("min_height_m", self.min_height_m)?;
        check_limit("plane_min_height_m", self.plane_min_height_m)?;
        check_positive("back_ramp_s", self.back_ramp_s)?;
        check_limit("back_speed_mps", self.back_speed_mps)?;
        check_positive("back_timeout_s", self.back_timeout_s)?;
        check(
            "forward_thrust",
            (0.0..=1.0).contains(&self.forward_thrust),
            "in [0, 1]",
        )?;
        check_positive("pusher_slew_per_s", self.pusher_slew_per_s)?;
        check_limit("handoff_s", self.handoff_s)?;
        check(
            "roll_limit_rad",
            self.roll_limit_rad > 0.0 && self.roll_limit_rad <= core::f32::consts::PI,
            "in (0, π] rad",
        )
    }
}

impl Default for LiftTransitionConfig {
    /// ArduPilot's 9 m/s airspeed gate and 5 s lift ramp, PX4's 15 s forward timeout, 3 s back
    /// ramp, 5 m/s back speed, 10 s back timeout, and pusher thrust up to full rising at
    /// 0.33/s, and 5 m of height, no plane minimum height (PX4's `VT_FW_MIN_ALT`), a 0.5 s
    /// handoff, and a 45° roll limit (as [`TransitionConfig`](super::TransitionConfig)); see
    /// [`LiftTransition`]'s [sources](LiftTransition#sources).
    fn default() -> Self {
        LiftTransitionConfig {
            min_airspeed_mps: 9.0,
            lift_ramp_s: 5.0,
            timeout_s: 15.0,
            min_height_m: 5.0,
            plane_min_height_m: 0.0,
            back_ramp_s: 3.0,
            back_speed_mps: 5.0,
            back_timeout_s: 10.0,
            forward_thrust: 1.0,
            pusher_slew_per_s: 0.33,
            handoff_s: 0.5,
            roll_limit_rad: 45.0_f32.to_radians(),
        }
    }
}

/// What [`LiftTransition::update`] decided this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiftTransitionStep {
    /// The mode to fly this tick.
    pub mode: FlightMode,
    /// The share of hover lift the lift rotors carry, in `[0, 1]`: 1 in `Copter`, 0 in `Plane`,
    /// ramping between them in the transitions (see [`LiftTransition`]).
    pub lift_share: f32,
    /// Control passed between hover and plane flight this tick (`ToPlane` → `Plane`,
    /// `ToCopter` → `Copter`): prime the guidance taking over.
    pub handoff: bool,
}

/// The copter ⇄ plane transition state machine of a lift+cruise VTOL: owns the
/// [`FlightMode`] and the lift rotors' share of the lift, by the rows under [Modes](#modes).
///
/// Call [`prime`](Self::prime) when the vehicle arms, then [`update`](Self::update) every
/// armed tick. Allocation-free and trig-free per tick.
///
/// # Modes
///
/// The request is a level, as for [`Transition`](super::Transition). First matching row wins:
///
/// | Mode | Condition | Next | Lift share | Handoff | Refusal reported |
/// |---|---|---|---|---|---|
/// | `Copter` | `Copter` requested | `Copter` | 1 | | none; clears a timeout |
/// | `Copter` | `Plane` requested after a fallback | `Copter` | 1 | | why: `TransitionTimeout`, `RollError`, `BelowPlaneMinAltitude` |
/// | `Copter` | `Plane` requested below `min_height_m` or `plane_min_height_m` (or NaN altitude) | `Copter` | 1 | | `BelowMinAltitude` |
/// | `Copter` | `Plane` requested | `ToPlane`, waiting on airspeed | 1 | | |
/// | `ToPlane` | `Copter` requested | `Copter` | 1 | | |
/// | `ToPlane` | the lift ramp has run `lift_ramp_s` | `Plane` | 0 | yes | |
/// | `ToPlane` | waiting longer than `timeout_s` | `Copter` | 1 | | `TransitionTimeout` |
/// | `ToPlane` | waiting, and a valid airspeed ≥ `min_airspeed_mps` | `ToPlane`, the lift ramp starts | 1 | | |
/// | `ToPlane` | otherwise | `ToPlane` | 1 waiting, then `1 − t / lift_ramp_s` | | |
/// | `Plane` | `Copter` requested | `ToCopter` | 0 | | |
/// | `Plane` | below `plane_min_height_m` | `ToCopter`, a fallback | 0 | | `BelowPlaneMinAltitude` |
/// | `ToCopter` | `Plane` requested, and no fallback | `Plane` | 0 | | |
/// | `ToCopter` | forward speed below `back_speed_mps`, or longer than `back_timeout_s` | `Copter` | 1 | yes | |
/// | `ToCopter` | otherwise | `ToCopter` | `t / back_ramp_s`, at most 1 | | |
///
/// The lift share is the share of hover lift the lift rotors carry: the guidance flying this
/// weights the lift rotors' thrust and moments by it. Without a valid airspeed the forward
/// transition never starts its lift ramp and times out to `Copter`: the wing's speed is unknown,
/// so handing it the weight could stall it. Once started, the ramp isn't timed out: the wing
/// has its speed, and abandoning mid-ramp would throw the weight back onto the rotors at once.
/// The forward speed is the ground velocity along the body `x` axis; when it isn't known (NaN
/// velocity or attitude), a valid airspeed stands in, and without either the back transition
/// completes on its timeout.
///
/// [`cut_short`](Self::cut_short) ends a transition early as
/// [`Transition::cut_short`](super::Transition::cut_short) does. Height and abandonment follow
/// [`Transition`](super::Transition): above the altitude at [`prime`](Self::prime),
/// refused until then, and after a fallback (a forward transition abandoned, or plane flight
/// below `plane_min_height_m`) `Plane` held refused until `Copter` is requested: `ToCopter`
/// completes as if `Copter` were requested, reporting the fallback.
///
/// # Sources
///
/// ArduPilot (`368dc0c428`) and PX4 (`c4e4ef98e9`), described, not copied:
///
/// - Forward: ArduPilot holds hover on the lift rotors until airspeed passes `AIRSPEED_MIN`
///   (9 m/s, `ArduPlane/quadplane.cpp:1522`, `config.h:124`), then ramps their throttle down
///   linearly over `Q_TRANSITION_MS` (5 s, `quadplane.cpp:31`, `:1589-1599`); it needs an
///   airspeed sensor to leave the wait. Its optional timeout (`Q_TRANS_FAIL`, off by default,
///   `quadplane.cpp:346`) covers only the wait (`:1488-1491`); the 15 s is PX4's
///   `VT_TRANS_TIMEOUT` (`vtol_type.cpp:327-341`), which also covers the ramp.
/// - Back: PX4 ramps the lift rotors' weight up over `VT_B_TRANS_RAMP` (3 s,
///   `vtol_att_control/standard.cpp:250-254`) and completes once the body-`x` ground speed
///   is below `MPC_XY_CRUISE` (5 m/s; airspeed without a ground velocity) or after
///   `VT_B_TRANS_DUR` (10 s, `standard.cpp:98-115`).
/// - Pusher: PX4 slews it to `VT_F_TRANS_THR` (1.0) at `VT_PSHER_SLEW` (0.33/s,
///   `standard.cpp:201-242`) going forward and cuts it going back (`:269`); ArduPilot flies
///   it on the plane controller's throttle going forward (`servos.cpp:569-588`).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{LiftTransition, LiftTransitionConfig};
/// use stable_flight::{FlightMode, FlightRegime, Vec3, VehicleState};
///
/// let mut transition = LiftTransition::new(LiftTransitionConfig::default());
/// transition.prime(&VehicleState::default());
///
/// // 10 m up and still slow: the lift rotors hold it while the pusher builds speed.
/// let slow = VehicleState { position_m: Vec3::new(0.0, 0.0, 10.0), airspeed_mps: Some(4.0), ..Default::default() };
/// let step = transition.update(0.01, FlightRegime::Plane, &slow);
/// assert_eq!((step.mode, step.lift_share), (FlightMode::ToPlane, 1.0));
///
/// // Past the airspeed gate the lift ramps down.
/// let fast = VehicleState { airspeed_mps: Some(10.0), ..slow };
/// transition.update(0.01, FlightRegime::Plane, &fast);
/// assert!(transition.update(0.01, FlightRegime::Plane, &fast).lift_share < 1.0);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct LiftTransition {
    config: LiftTransitionConfig,
    gate: ForwardGate,
    mode: FlightMode,
    refusal: Option<ModeRefusal>,
    /// Time in the current transition, in seconds.
    elapsed_s: f32,
    /// When the forward lift ramp started, as `elapsed_s`; `None` while waiting on airspeed.
    ramp_start_s: Option<f32>,
}

impl LiftTransition {
    /// A transition in `Copter` with `config`. Until [`prime`](Self::prime) sets the arming
    /// altitude it has no height reference, so it refuses `Plane` as `BelowMinAltitude`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`LiftTransitionConfig::validate`]).
    pub fn new(config: LiftTransitionConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid LiftTransitionConfig: {error}");
        }
        LiftTransition {
            config,
            gate: ForwardGate::new(),
            mode: FlightMode::Copter,
            refusal: None,
            elapsed_s: 0.0,
            ramp_start_s: None,
        }
    }

    /// Back to `Copter` with no refusal or timeout, and `state`'s altitude as the height
    /// reference. Call when the vehicle arms.
    pub fn prime(&mut self, state: &VehicleState) {
        self.mode = FlightMode::Copter;
        self.refusal = None;
        self.elapsed_s = 0.0;
        self.ramp_start_s = None;
        self.gate.prime(state);
    }

    /// Moves toward `requested` and returns this tick's mode and lift share.
    ///
    /// # Panics
    ///
    /// If `dt` isn't positive and finite.
    pub fn update(
        &mut self,
        dt: f32,
        requested: FlightRegime,
        state: &VehicleState,
    ) -> LiftTransitionStep {
        assert_dt(dt);
        self.refusal = None;
        let mut handoff = false;

        match self.mode {
            FlightMode::Copter => match requested {
                FlightRegime::Plane => {
                    let needed_m = self.config.min_height_m.max(self.config.plane_min_height_m);
                    self.refusal = self.gate.refusal(state, needed_m);
                    if self.refusal.is_none() {
                        self.start(FlightMode::ToPlane);
                    }
                }
                FlightRegime::Copter => self.gate.clear(),
            },
            FlightMode::ToPlane => {
                self.elapsed_s += dt;
                if requested != FlightRegime::Plane {
                    self.mode = FlightMode::Copter;
                } else if self.lift_ramp_done() {
                    self.mode = FlightMode::Plane;
                    handoff = true;
                } else if self.ramp_start_s.is_none() {
                    if self.elapsed_s > self.config.timeout_s {
                        self.abandon(ModeRefusal::TransitionTimeout);
                    } else if self.fast_enough(state) {
                        self.ramp_start_s = Some(self.elapsed_s);
                    }
                }
            }
            FlightMode::Plane => {
                if requested != FlightRegime::Plane {
                    self.start(FlightMode::ToCopter);
                } else if self.gate.below(state, self.config.plane_min_height_m) {
                    self.start(FlightMode::ToCopter);
                    self.gate.abandon(ModeRefusal::BelowPlaneMinAltitude);
                    self.refusal = Some(ModeRefusal::BelowPlaneMinAltitude);
                }
            }
            FlightMode::ToCopter => {
                self.elapsed_s += dt;
                let plane_requested = requested == FlightRegime::Plane;
                if plane_requested && !self.gate.fallen_back() {
                    self.mode = FlightMode::Plane;
                } else if self.slowed(state) || self.elapsed_s > self.config.back_timeout_s {
                    self.mode = FlightMode::Copter;
                    handoff = true;
                }
                if plane_requested {
                    self.refusal = self.gate.fallback();
                }
            }
        }

        LiftTransitionStep {
            mode: self.mode,
            lift_share: self.lift_share(),
            handoff,
        }
    }

    /// Cuts a transition short for `refusal` (e.g. [`ModeRefusal::RollError`], from a check the
    /// caller makes), returning the step to fly this tick instead of the one
    /// [`update`](Self::update) gave: a forward transition is abandoned to `Copter`, held
    /// refused until `Copter` is requested; a back transition completes to `Copter` with a
    /// handoff. Outside a transition it returns the current mode, unchanged.
    pub fn cut_short(&mut self, refusal: ModeRefusal) -> LiftTransitionStep {
        let handoff = match self.mode {
            FlightMode::ToPlane => {
                self.abandon(refusal);
                false
            }
            FlightMode::ToCopter => {
                self.mode = FlightMode::Copter;
                true
            }
            _ => false,
        };
        LiftTransitionStep {
            mode: self.mode,
            lift_share: self.lift_share(),
            handoff,
        }
    }

    /// The mode the last [`update`](Self::update) flew, and why it isn't the requested regime
    /// when it can't be: what a VTOL guidance reports as its
    /// [`flight_mode`](crate::Guidance::flight_mode).
    pub fn report(&self) -> FlightModeReport {
        FlightModeReport {
            mode: self.mode,
            refusal: self.refusal,
        }
    }

    /// Whether a fallback holds `Plane` refused until `Copter` is requested.
    pub(super) fn fallen_back(&self) -> bool {
        self.gate.fallen_back()
    }

    /// Enters a transition `mode` from its start.
    fn start(&mut self, mode: FlightMode) {
        self.mode = mode;
        self.elapsed_s = 0.0;
        self.ramp_start_s = None;
    }

    /// Gives up a forward transition: back to `Copter`, refused for `refusal` until `Copter`
    /// is requested.
    fn abandon(&mut self, refusal: ModeRefusal) {
        self.mode = FlightMode::Copter;
        self.gate.abandon(refusal);
        self.refusal = Some(refusal);
    }

    /// Whether the forward lift ramp has run its full time.
    fn lift_ramp_done(&self) -> bool {
        self.ramp_start_s
            .is_some_and(|start_s| self.elapsed_s - start_s >= self.config.lift_ramp_s)
    }

    /// Whether a valid airspeed has reached the forward gate.
    fn fast_enough(&self, state: &VehicleState) -> bool {
        state
            .valid_airspeed_mps()
            .is_some_and(|airspeed_mps| airspeed_mps >= self.config.min_airspeed_mps)
    }

    /// Whether the back transition has slowed below its speed, by forward ground speed or,
    /// without it, a valid airspeed. Unknown speed never has.
    fn slowed(&self, state: &VehicleState) -> bool {
        let speed_mps = Some(forward_speed_mps(state))
            .filter(|speed_mps| speed_mps.is_finite())
            .or_else(|| state.valid_airspeed_mps());
        speed_mps.is_some_and(|speed_mps| speed_mps < self.config.back_speed_mps)
    }

    /// The lift rotors' share of the lift this tick.
    fn lift_share(&self) -> f32 {
        match self.mode {
            FlightMode::Plane => 0.0,
            FlightMode::ToPlane => self.ramp_start_s.map_or(1.0, |start_s| {
                (1.0 - (self.elapsed_s - start_s) / self.config.lift_ramp_s).clamp(0.0, 1.0)
            }),
            FlightMode::ToCopter => (self.elapsed_s / self.config.back_ramp_s).min(1.0),
            _ => 1.0,
        }
    }
}

impl Configurable for LiftTransition {
    type Config = LiftTransitionConfig;

    fn config(&self) -> &LiftTransitionConfig {
        &self.config
    }

    /// Validates with [`LiftTransitionConfig::validate`]. A running transition keeps its mode
    /// and times, and follows the new ramps, gates, and timeouts from the next tick.
    fn set_config(&mut self, config: LiftTransitionConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        Ok(())
    }
}

/// The ground velocity along the body `x` axis, in m/s; NaN when the velocity or attitude is.
fn forward_speed_mps(state: &VehicleState) -> f32 {
    let nose = state.attitude * Vec3::x();
    let velocity_mps = state.velocity_mps;
    // `nose` is NED, the velocity `z` up: the `z` term flips.
    nose.x * velocity_mps.x + nose.y * velocity_mps.y - nose.z * velocity_mps.z
}
