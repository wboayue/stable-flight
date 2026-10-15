//! Copter ⇄ plane transitions for a VTOL: [`Transition`] decides the [`FlightMode`] each tick
//! from the requested [`FlightRegime`] and the vehicle's tilt, and ramps a tilt target for the
//! controller flying the transition; [`ThrustHandoff`] blends thrust when control passes from
//! one controller to the other.
//!
//! Both are pure building blocks for a VTOL guidance ([`TailsitterGuidance`]): they build no
//! setpoints and know no frames. The tilt is the nose's (body `x`) angle from vertical: 0 in hover,
//! π/2 in level forward flight. A tailsitter's hover controller, in its
//! [`View`](crate::View), flies view pitch `−tilt`; its plane controller flies airplane pitch
//! `π/2 − tilt`.
//!
//! A lift+cruise VTOL (quadplane) stays level and flies both sides at once, trading lift
//! between its lift rotors and its wing: [`LiftTransition`] is its state machine.
//!
//! # Modes
//!
//! The request is a level, so a repeated request finds the same row and a running ramp
//! continues. First matching row wins:
//!
//! | Mode | Condition | Next | Handoff | Refusal reported |
//! |---|---|---|---|---|
//! | `Copter` | `Copter` requested | `Copter` | | none; clears a timeout |
//! | `Copter` | `Plane` requested after a forward transition was abandoned | `Copter` | | why: `TransitionTimeout`, `RollError` |
//! | `Copter` | `Plane` requested below `min_height_m` (or NaN altitude) | `Copter` | | `BelowMinAltitude` |
//! | `Copter` | `Plane` requested | `ToPlane`, ramp from the current tilt | | |
//! | `ToPlane` | `Copter` requested | `Copter` (same hover controller) | | |
//! | `ToPlane` | tilt ≥ `forward_done_rad` and airspeed ≥ `min_airspeed_mps` (when valid) | `Plane` | yes | |
//! | `ToPlane` | longer than `timeout_s` | `Copter` | | `TransitionTimeout` |
//! | `Plane` | `Copter` requested | `ToCopter`, ramp from the current tilt | | |
//! | `ToCopter` | `Plane` requested | `Plane` (same plane controller) | | |
//! | `ToCopter` | tilt ≤ `back_done_rad`, or longer than `timeout_s` | `Copter` | yes | |
//!
//! The caller can cut a transition short ([`Transition::cut_short`], e.g. on a roll error it
//! measures): forward is abandoned like a timeout, back completes with a handoff.
//!
//! A standing `Plane` request refused for height starts the transition once the vehicle
//! climbs past `min_height_m`; one refused by an abandoned transition waits for a `Copter`
//! request first.
//! Height is above the altitude at [`prime`](Transition::prime) (where the vehicle armed);
//! before the first `prime` there is none, and `Plane` is refused.
//! A `Land` input means `Copter`: the guidance driving this passes that as the request.
//!
//! # Sources
//!
//! ArduPilot (master `e506f3a`) and PX4 (main `edeba2f`), described, not copied:
//!
//! - Both fly the forward transition on the hover controller, pitching the nose down at a
//!   fixed rate, and complete on pitch: ArduPilot at `Q_TAILSIT_ANGLE` (45°) from vertical
//!   (`ArduPlane/tailsitter.cpp:40`, `:537`), PX4 at −60° in its hover frame plus airspeed
//!   when valid (`vtol_att_control/tailsitter.h:52`, `tailsitter.cpp:379-396`).
//! - ArduPilot ramps at `Q_TAILSIT_RAT_FW`/`RAT_VT` (50°/s each, `tailsitter.cpp:128`,
//!   `:135`) and hands back to hover at `Q_TAILSIT_ANG_VT`, which defaults to the forward
//!   angle (`:47`).
//! - A forward timeout reverts to hover in PX4 (`VT_TRANS_TIMEOUT` 15 s,
//!   `vtol_att_control_params.yaml:92-98`, then quadchute), where ArduPilot completes it
//!   (`tailsitter.cpp:546`); a back timeout completes in both (`tailsitter.cpp:586`).
//! - PX4 blends thrust over 0.5 s when the back transition takes over from the plane
//!   controller (`tailsitter.h:58`).
//! - Through the back ramp ArduPilot flies a fixed throttle (`Q_TAILSIT_THR_VT`, by default
//!   the hover throttle, at least cruise; `tailsitter.cpp:137-142`, `:306-341`) with no
//!   altitude loop; PX4 flies its hover vertical-speed loop, its setpoint decaying to 0
//!   (`FlightTaskTransition.cpp:50-86`, main `e146fe4`). Here the thrust is fixed and raised
//!   only while sinking: one side of PX4's loop, since a climb costs height and a sink can
//!   end in the ground.

use core::f32::consts::FRAC_PI_2;

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::{assert_dt, nose_up_cosine, saturate_throttle};
use crate::{Configurable, FlightMode, FlightModeReport, FlightRegime, ModeRefusal, VehicleState};

mod forward;
mod lift;
mod params;
mod quadplane;
mod tailsitter;

pub use forward::{ForwardSplit, ForwardThrust, ForwardThrustConfig};
pub use lift::{LiftTransition, LiftTransitionConfig, LiftTransitionStep};
pub use quadplane::{QuadPlaneGuidance, QuadPlaneGuidanceConfig};
pub use tailsitter::{TailsitterGuidance, TailsitterGuidanceConfig};
#[cfg(test)]
mod tests;

/// Transition settings: what [`Transition`] uses (ramp rates, completion tilts, the airspeed
/// gate, the timeout, the height a forward transition needs) and what the VTOL guidance around
/// it applies (thrusts, the handoff blend, the roll limit). Angles are the nose's tilt from
/// vertical. Each field is checked on its own: a `forward_max_tilt_rad` below
/// `forward_done_rad` is accepted, and leaves the forward transition to its timeout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransitionConfig {
    /// How fast the forward (nose-down) tilt target ramps, in rad/s.
    pub forward_rate_radps: f32,
    /// The tilt from vertical that completes the forward transition, in radians, in
    /// `(0, π/2)`.
    pub forward_done_rad: f32,
    /// Where the forward ramp stops while the transition waits on airspeed, in radians, in
    /// `(0, π/2]`; at least `forward_done_rad` for the ramp to reach completion.
    pub forward_max_tilt_rad: f32,
    /// The airspeed the forward transition also needs when the state has a valid one, in m/s;
    /// `0` turns the gate off.
    pub min_airspeed_mps: f32,
    /// How fast the back (nose-up) tilt target ramps, in rad/s.
    pub back_rate_radps: f32,
    /// The tilt from vertical that completes the back transition, in radians, in `(0, π/2)`.
    pub back_done_rad: f32,
    /// The longest either transition runs, in seconds: forward falls back to copter, back
    /// completes.
    pub timeout_s: f32,
    /// The height above the arming altitude a forward transition needs to start, in meters.
    pub min_height_m: f32,
    /// Least linear thrust through the forward transition, `[0, 1]`: the hover thrust when it
    /// began, if higher, holds.
    pub forward_thrust: f32,
    /// Linear thrust through the back transition, `[0, 1]`, raised while sinking past
    /// [`back_sink_limit_mps`](Self::back_sink_limit_mps).
    pub back_thrust: f32,
    /// Sink rate past which the back transition gets more thrust than
    /// [`back_thrust`](Self::back_thrust), in m/s: [`back_sink_gain`](Self::back_sink_gain)
    /// per m/s of sink past it. Slowing nose-high with too little thrust, the wing stalls and
    /// the vehicle falls; this catches it. Non-negative; `0` turns it off.
    pub back_sink_limit_mps: f32,
    /// Thrust added per m/s of sink past [`back_sink_limit_mps`](Self::back_sink_limit_mps),
    /// in s/m. Non-negative.
    pub back_sink_gain: f32,
    /// How long thrust blends from one controller to the other at a handoff, in seconds
    /// ([`ThrustHandoff`]); 0 switches at once.
    pub handoff_s: f32,
    /// Roll error past which a transition is cut short, in radians, in `(0, π]`: forward
    /// aborts to copter, back completes ([`Transition::cut_short`]).
    pub roll_limit_rad: f32,
}

impl TransitionConfig {
    /// `Ok` when every field is in range (see each field).
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        let in_quarter_turn = |value: f32| value > 0.0 && value < FRAC_PI_2;
        check_positive("forward_rate_radps", self.forward_rate_radps)?;
        check(
            "forward_done_rad",
            in_quarter_turn(self.forward_done_rad),
            "in (0, π/2) rad",
        )?;
        check(
            "forward_max_tilt_rad",
            self.forward_max_tilt_rad > 0.0 && self.forward_max_tilt_rad <= FRAC_PI_2,
            "in (0, π/2] rad",
        )?;
        check_limit("min_airspeed_mps", self.min_airspeed_mps)?;
        check_positive("back_rate_radps", self.back_rate_radps)?;
        check(
            "back_done_rad",
            in_quarter_turn(self.back_done_rad),
            "in (0, π/2) rad",
        )?;
        check_positive("timeout_s", self.timeout_s)?;
        check_limit("min_height_m", self.min_height_m)?;
        let unit = |value: f32| (0.0..=1.0).contains(&value);
        check("forward_thrust", unit(self.forward_thrust), "in [0, 1]")?;
        check("back_thrust", unit(self.back_thrust), "in [0, 1]")?;
        check_limit("back_sink_limit_mps", self.back_sink_limit_mps)?;
        check_limit("back_sink_gain", self.back_sink_gain)?;
        check_limit("handoff_s", self.handoff_s)?;
        check(
            "roll_limit_rad",
            self.roll_limit_rad > 0.0 && self.roll_limit_rad <= core::f32::consts::PI,
            "in (0, π] rad",
        )
    }
}

impl Default for TransitionConfig {
    /// ArduPilot's tailsitter rates and angles (50°/s each way, complete at 45° from vertical
    /// both ways) and roll limit (45°), the forward ramp stopping at 80°, PX4's 15 s timeout
    /// and 0.5 s thrust blend, no airspeed gate, 5 m of height, half thrust each way (an
    /// airframe sets its own), and a back sink guard of 0.4 per m/s past 1 m/s; see the
    /// [module sources](self#sources).
    fn default() -> Self {
        TransitionConfig {
            forward_rate_radps: 50.0_f32.to_radians(),
            forward_done_rad: 45.0_f32.to_radians(),
            forward_max_tilt_rad: 80.0_f32.to_radians(),
            min_airspeed_mps: 0.0,
            back_rate_radps: 50.0_f32.to_radians(),
            back_done_rad: 45.0_f32.to_radians(),
            timeout_s: 15.0,
            min_height_m: 5.0,
            forward_thrust: 0.5,
            back_thrust: 0.5,
            // Measured on the sim tailsitter (#343): a 25°/s ramp at thrust 0.3–0.5, which
            // without it stalls nose-high and falls 30 m, hands over climbing at most 7 m.
            back_sink_limit_mps: 1.0,
            back_sink_gain: 0.4,
            handoff_s: 0.5,
            roll_limit_rad: 45.0_f32.to_radians(),
        }
    }
}

/// What [`Transition::update`] decided this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransitionStep {
    /// The mode to fly this tick.
    pub mode: FlightMode,
    /// While transitioning, the nose's tilt from vertical to fly, in radians: ramping toward
    /// plane flight in `ToPlane`, toward hover in `ToCopter`. `None` otherwise.
    pub tilt_target_rad: Option<f32>,
    /// Control passed between the hover and the plane controller this tick
    /// (`ToPlane` → `Plane`, `ToCopter` → `Copter`): prime the incoming one and start a
    /// [`ThrustHandoff`].
    pub handoff: bool,
}

/// The copter ⇄ plane transition state machine: owns the [`FlightMode`]. See the
/// [module docs](self) for the rows it follows.
///
/// Call [`prime`](Self::prime) when the vehicle arms, then [`update`](Self::update) every
/// armed tick. Allocation-free and trig-free per tick (one `acos` when a ramp starts).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{Transition, TransitionConfig};
/// use stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight::{FlightMode, FlightRegime, Vec3, VehicleState};
///
/// // Hovering nose up, 10 m above where it armed.
/// let hover = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(0.0, core::f32::consts::FRAC_PI_2, 0.0),
///     ..Default::default()
/// };
/// let mut transition = Transition::new(TransitionConfig::default());
/// transition.prime(&hover);
///
/// let climbed = VehicleState { position_m: Vec3::new(0.0, 0.0, 10.0), ..hover };
/// let step = transition.update(0.01, FlightRegime::Plane, &climbed);
/// assert_eq!(step.mode, FlightMode::ToPlane);
/// assert!(step.tilt_target_rad.unwrap() > 0.0); // nose starting down
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Transition {
    config: TransitionConfig,
    forward_done_cosine: f32,
    back_done_cosine: f32,
    mode: FlightMode,
    refusal: Option<ModeRefusal>,
    gate: ForwardGate,
    elapsed_s: f32,
    start_tilt_rad: f32,
}

impl Transition {
    /// A transition in `Copter` with `config`. Until [`prime`](Self::prime) sets the arming
    /// altitude it has no height reference, so it refuses `Plane` as `BelowMinAltitude`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`TransitionConfig::validate`]).
    pub fn new(config: TransitionConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid TransitionConfig: {error}");
        }
        let (forward_done_cosine, back_done_cosine) = done_cosines(&config);
        Transition {
            config,
            forward_done_cosine,
            back_done_cosine,
            mode: FlightMode::Copter,
            refusal: None,
            gate: ForwardGate::new(),
            elapsed_s: 0.0,
            start_tilt_rad: 0.0,
        }
    }

    /// Back to `Copter` with no refusal or timeout, and `state`'s altitude as the height
    /// reference. Call when the vehicle arms.
    pub fn prime(&mut self, state: &VehicleState) {
        self.mode = FlightMode::Copter;
        self.refusal = None;
        self.elapsed_s = 0.0;
        self.gate.prime(state);
    }

    /// Moves toward `requested` and returns this tick's mode and tilt target.
    ///
    /// # Panics
    ///
    /// If `dt` isn't positive and finite.
    pub fn update(
        &mut self,
        dt: f32,
        requested: FlightRegime,
        state: &VehicleState,
    ) -> TransitionStep {
        assert_dt(dt);
        let nose_up = nose_up_cosine(&state.attitude);
        self.refusal = None;
        let mut handoff = false;

        match self.mode {
            FlightMode::Copter => match requested {
                FlightRegime::Plane => self.try_forward(state, nose_up),
                FlightRegime::Copter => self.gate.clear(),
            },
            FlightMode::ToPlane => {
                self.elapsed_s += dt;
                if requested != FlightRegime::Plane {
                    self.mode = FlightMode::Copter;
                } else if self.forward_done(state, nose_up) {
                    self.mode = FlightMode::Plane;
                    handoff = true;
                } else if self.elapsed_s > self.config.timeout_s {
                    self.abandon(ModeRefusal::TransitionTimeout);
                }
            }
            FlightMode::Plane => {
                if requested != FlightRegime::Plane {
                    self.start(FlightMode::ToCopter, nose_up);
                }
            }
            FlightMode::ToCopter => {
                self.elapsed_s += dt;
                if requested == FlightRegime::Plane {
                    self.mode = FlightMode::Plane;
                } else if nose_up >= self.back_done_cosine || self.elapsed_s > self.config.timeout_s
                {
                    self.mode = FlightMode::Copter;
                    handoff = true;
                }
            }
        }

        TransitionStep {
            mode: self.mode,
            tilt_target_rad: self.tilt_target_rad(),
            handoff,
        }
    }

    /// Cuts a transition short for `refusal` (e.g. [`ModeRefusal::RollError`], from a check the
    /// caller makes), returning the step to fly this tick instead of the one
    /// [`update`](Self::update) gave: a forward transition is abandoned to `Copter`, held
    /// refused until `Copter` is requested (like a timeout); a back transition completes to
    /// `Copter` with a handoff. Outside a transition it returns the current mode, unchanged.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::guidance::vtol::{Transition, TransitionConfig};
    /// use stable_flight::{FlightMode, FlightRegime, ModeRefusal, Vec3, VehicleState};
    ///
    /// let mut transition = Transition::new(TransitionConfig::default());
    /// transition.prime(&VehicleState::default());
    /// let high = VehicleState { position_m: Vec3::new(0.0, 0.0, 10.0), ..Default::default() };
    /// transition.update(0.01, FlightRegime::Plane, &high);
    ///
    /// let step = transition.cut_short(ModeRefusal::RollError);
    /// assert_eq!(step.mode, FlightMode::Copter);
    /// assert_eq!(transition.report().refusal, Some(ModeRefusal::RollError));
    /// ```
    pub fn cut_short(&mut self, refusal: ModeRefusal) -> TransitionStep {
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
        TransitionStep {
            mode: self.mode,
            tilt_target_rad: self.tilt_target_rad(),
            handoff,
        }
    }

    /// Gives up a forward transition: back to `Copter`, refused for `refusal` until `Copter`
    /// is requested.
    fn abandon(&mut self, refusal: ModeRefusal) {
        self.mode = FlightMode::Copter;
        self.gate.abandon(refusal);
        self.refusal = Some(refusal);
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

    /// Starts `ToPlane` from `Copter`, or reports why it can't.
    fn try_forward(&mut self, state: &VehicleState, nose_up: f32) {
        self.refusal = self.gate.refusal(state, self.config.min_height_m);
        if self.refusal.is_none() {
            self.start(FlightMode::ToPlane, nose_up);
        }
    }

    /// Enters a transition `mode`, ramping from the vehicle's tilt (hover for a NaN attitude
    /// going forward, level flight going back).
    fn start(&mut self, mode: FlightMode, nose_up: f32) {
        let fallback = if mode == FlightMode::ToPlane {
            0.0
        } else {
            FRAC_PI_2
        };
        self.mode = mode;
        self.elapsed_s = 0.0;
        self.start_tilt_rad = if nose_up.is_finite() {
            libm::acosf(nose_up.clamp(-1.0, 1.0))
        } else {
            fallback
        };
    }

    /// Whether `ToPlane` has tilted far enough, with enough airspeed when it's measured. A
    /// NaN tilt never completes.
    fn forward_done(&self, state: &VehicleState, nose_up: f32) -> bool {
        let tilted = nose_up <= self.forward_done_cosine;
        let fast_enough = state
            .valid_airspeed_mps()
            .is_none_or(|airspeed_mps| airspeed_mps >= self.config.min_airspeed_mps);
        tilted && fast_enough
    }

    /// The ramped tilt target while transitioning.
    fn tilt_target_rad(&self) -> Option<f32> {
        match self.mode {
            FlightMode::ToPlane => Some(
                (self.start_tilt_rad + self.config.forward_rate_radps * self.elapsed_s)
                    .min(self.config.forward_max_tilt_rad.max(self.start_tilt_rad)),
            ),
            FlightMode::ToCopter => {
                Some((self.start_tilt_rad - self.config.back_rate_radps * self.elapsed_s).max(0.0))
            }
            _ => None,
        }
    }
}

impl Configurable for Transition {
    type Config = TransitionConfig;

    fn config(&self) -> &TransitionConfig {
        &self.config
    }

    /// Validates with [`TransitionConfig::validate`]. A running transition keeps its mode,
    /// start tilt, and elapsed time, and follows the new rates, angles, and timeout from the
    /// next tick.
    fn set_config(&mut self, config: TransitionConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        (self.forward_done_cosine, self.back_done_cosine) = done_cosines(&config);
        Ok(())
    }
}

/// What starts a forward transition, shared by [`Transition`] and [`LiftTransition`]: height
/// above the arming altitude, and no abandoned forward transition waiting on a `Copter`
/// request.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ForwardGate {
    /// The altitude at [`prime`](Self::prime); NaN before it, which no height clears.
    home_altitude_m: f32,
    /// Why the last forward transition was abandoned, held until `Copter` is requested.
    abandoned: Option<ModeRefusal>,
}

impl ForwardGate {
    fn new() -> Self {
        ForwardGate {
            home_altitude_m: f32::NAN,
            abandoned: None,
        }
    }

    /// `state`'s altitude as the height reference, and no abandoned transition.
    fn prime(&mut self, state: &VehicleState) {
        self.home_altitude_m = state.position_m.z;
        self.abandoned = None;
    }

    /// Why a forward transition can't start in `state`, needing `min_height_m`: the abandoned
    /// transition's refusal first, then `BelowMinAltitude`; `None` when it can.
    fn refusal(&self, state: &VehicleState, min_height_m: f32) -> Option<ModeRefusal> {
        let height_m = state.position_m.z - self.home_altitude_m;
        // A NaN height isn't high enough.
        let high_enough = height_m.is_finite() && height_m >= min_height_m;
        if self.abandoned.is_some() {
            self.abandoned
        } else if !high_enough {
            Some(ModeRefusal::BelowMinAltitude)
        } else {
            None
        }
    }

    /// Holds `refusal` against forward transitions until [`clear`](Self::clear).
    fn abandon(&mut self, refusal: ModeRefusal) {
        self.abandoned = Some(refusal);
    }

    /// `Copter` was requested: an abandoned transition no longer holds.
    fn clear(&mut self) {
        self.abandoned = None;
    }
}

/// The cosines of the forward and back completion tilts, which [`Transition`] compares the
/// nose's [`nose_up_cosine`] against instead of taking its `acos` every tick.
fn done_cosines(config: &TransitionConfig) -> (f32, f32) {
    (
        libm::cosf(config.forward_done_rad),
        libm::cosf(config.back_done_rad),
    )
}

/// Blends thrust from its value when control passed between controllers to the incoming
/// controller's demand, linearly over a fixed time, so the handoff steps moments but not
/// thrust.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::ThrustHandoff;
///
/// let mut handoff = ThrustHandoff::new(0.5);
/// handoff.start(0.8); // the outgoing controller's last thrust
/// let first = handoff.apply(0.25, 0.4);
/// assert!((first - 0.6).abs() < 1e-6); // halfway
/// assert_eq!(handoff.apply(0.25, 0.4), 0.4); // done
/// assert!(!handoff.active());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustHandoff {
    duration_s: f32,
    from: f32,
    elapsed_s: f32,
}

impl ThrustHandoff {
    /// A handoff that blends over `duration_s` seconds, inactive until
    /// [`start`](Self::start). Zero passes the demand straight through.
    ///
    /// # Panics
    ///
    /// If `duration_s` is negative or not finite.
    pub fn new(duration_s: f32) -> Self {
        if let Err(error) = check_limit("duration_s", duration_s) {
            panic!("invalid ThrustHandoff: {error}");
        }
        ThrustHandoff {
            duration_s,
            from: 0.0,
            elapsed_s: duration_s,
        }
    }

    /// Starts blending from `from` (the outgoing controller's last thrust, `[0, 1]`; NaN
    /// counts as 0).
    pub fn start(&mut self, from: f32) {
        self.from = saturate_throttle(from);
        self.elapsed_s = 0.0;
    }

    /// Whether a blend is running.
    pub fn active(&self) -> bool {
        self.elapsed_s < self.duration_s
    }

    /// This tick's thrust: `demanded` (saturated to `[0, 1]`, NaN as 0) once the blend is
    /// done, otherwise the blend from the start value toward it.
    ///
    /// # Panics
    ///
    /// If `dt` isn't positive and finite.
    pub fn apply(&mut self, dt: f32, demanded: f32) -> f32 {
        assert_dt(dt);
        let demanded = saturate_throttle(demanded);
        if !self.active() {
            return demanded;
        }
        self.elapsed_s += dt;
        let weight = (self.elapsed_s / self.duration_s).min(1.0);
        self.from + (demanded - self.from) * weight
    }
}
