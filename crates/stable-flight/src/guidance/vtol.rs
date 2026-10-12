//! Copter ⇄ plane transitions for a VTOL: [`Transition`] decides the [`FlightMode`] each tick
//! from the requested [`FlightRegime`] and the vehicle's tilt, and ramps a tilt target for the
//! controller flying the transition; [`ThrustHandoff`] blends thrust when control passes from
//! one controller to the other.
//!
//! Both are pure building blocks for a VTOL guidance (TODO(#124)): they build no setpoints and
//! know no frames. The tilt is the nose's (body `x`) angle from vertical: 0 in hover,
//! π/2 in level forward flight. A tailsitter's hover controller, in its
//! [`View`](crate::View), flies view pitch `−tilt`; its plane controller flies airplane pitch
//! `π/2 − tilt`.
//!
//! # Modes
//!
//! The request is a level, so a repeated request finds the same row and a running ramp
//! continues. First matching row wins:
//!
//! | Mode | Condition | Next | Handoff | Refusal reported |
//! |---|---|---|---|---|
//! | `Copter` | `Copter` requested | `Copter` | | none; clears a timeout |
//! | `Copter` | `Plane` requested after a forward timeout | `Copter` | | `TransitionTimeout` |
//! | `Copter` | `Plane` requested below `min_height_m` (or NaN altitude) | `Copter` | | `BelowMinAltitude` |
//! | `Copter` | `Plane` requested | `ToPlane`, ramp from the current tilt | | |
//! | `ToPlane` | `Copter` requested | `Copter` (same hover controller) | | |
//! | `ToPlane` | tilt ≥ `forward_done_rad` and airspeed ≥ `min_airspeed_mps` (when valid) | `Plane` | yes | |
//! | `ToPlane` | longer than `timeout_s` | `Copter` | | `TransitionTimeout` |
//! | `Plane` | `Copter` requested | `ToCopter`, ramp from the current tilt | | |
//! | `ToCopter` | `Plane` requested | `Plane` (same plane controller) | | |
//! | `ToCopter` | tilt ≤ `back_done_rad`, or longer than `timeout_s` | `Copter` | yes | |
//!
//! A standing `Plane` request refused for height starts the transition once the vehicle
//! climbs past `min_height_m`; one refused by a timeout waits for a `Copter` request first.
//! Height is above the altitude at [`prime`](Transition::prime) (where the vehicle armed).
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

use core::f32::consts::FRAC_PI_2;

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::{assert_dt, nose_up_cosine, saturate_throttle};
use crate::{Configurable, FlightMode, FlightModeReport, FlightRegime, ModeRefusal, VehicleState};

#[cfg(test)]
mod tests;

/// [`Transition`] settings: ramp rates, completion tilts, the airspeed gate, the timeout, and
/// the height a forward transition needs. Angles are the nose's tilt from vertical.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransitionConfig {
    /// How fast the forward (nose-down) tilt target ramps, in rad/s.
    pub forward_rate_radps: f32,
    /// The tilt from vertical that completes the forward transition, in radians, in
    /// `(0, π/2)`.
    pub forward_done_rad: f32,
    /// Where the forward ramp stops while the transition waits on airspeed, in radians, in
    /// `[forward_done_rad, π/2]`.
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
            self.forward_max_tilt_rad >= self.forward_done_rad
                && self.forward_max_tilt_rad <= FRAC_PI_2,
            "in [forward_done_rad, π/2] rad",
        )?;
        check_limit("min_airspeed_mps", self.min_airspeed_mps)?;
        check_positive("back_rate_radps", self.back_rate_radps)?;
        check(
            "back_done_rad",
            in_quarter_turn(self.back_done_rad),
            "in (0, π/2) rad",
        )?;
        check_positive("timeout_s", self.timeout_s)?;
        check_limit("min_height_m", self.min_height_m)
    }
}

impl Default for TransitionConfig {
    /// ArduPilot's tailsitter rates and angles (50°/s each way, complete at 45° from vertical
    /// both ways), the forward ramp stopping at 80°, PX4's 15 s timeout, no airspeed gate, and
    /// 5 m of height; see the [module sources](self#sources).
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
    timed_out: bool,
    elapsed_s: f32,
    start_tilt_rad: f32,
    home_altitude_m: f32,
}

impl Transition {
    /// A transition in `Copter` with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`TransitionConfig::validate`]).
    pub fn new(config: TransitionConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid TransitionConfig: {error}");
        }
        Transition {
            config,
            forward_done_cosine: libm::cosf(config.forward_done_rad),
            back_done_cosine: libm::cosf(config.back_done_rad),
            mode: FlightMode::Copter,
            refusal: None,
            timed_out: false,
            elapsed_s: 0.0,
            start_tilt_rad: 0.0,
            home_altitude_m: f32::NAN,
        }
    }

    /// Back to `Copter` with no refusal or timeout, and `state`'s altitude as the height
    /// reference. Call when the vehicle arms.
    pub fn prime(&mut self, state: &VehicleState) {
        self.mode = FlightMode::Copter;
        self.refusal = None;
        self.timed_out = false;
        self.elapsed_s = 0.0;
        self.home_altitude_m = state.position_m.z;
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
                FlightRegime::Copter => self.timed_out = false,
            },
            FlightMode::ToPlane => {
                self.elapsed_s += dt;
                if requested != FlightRegime::Plane {
                    self.mode = FlightMode::Copter;
                } else if self.forward_done(state, nose_up) {
                    self.mode = FlightMode::Plane;
                    handoff = true;
                } else if self.elapsed_s > self.config.timeout_s {
                    self.mode = FlightMode::Copter;
                    self.timed_out = true;
                    self.refusal = Some(ModeRefusal::TransitionTimeout);
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
        let height_m = state.position_m.z - self.home_altitude_m;
        // A NaN height isn't high enough.
        let high_enough = height_m.is_finite() && height_m >= self.config.min_height_m;
        self.refusal = if self.timed_out {
            Some(ModeRefusal::TransitionTimeout)
        } else if !high_enough {
            Some(ModeRefusal::BelowMinAltitude)
        } else {
            None
        };
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
        self.forward_done_cosine = libm::cosf(config.forward_done_rad);
        self.back_done_cosine = libm::cosf(config.back_done_rad);
        Ok(())
    }
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
