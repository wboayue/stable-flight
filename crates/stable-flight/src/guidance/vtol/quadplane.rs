//! [`QuadPlaneGuidance`]: hover and plane guidance under one [`LiftTransition`].

use super::{
    FallbackHold, ForwardThrust, ForwardThrustConfig, LiftTransition, LiftTransitionConfig,
    LiftTransitionStep, OffNoseSpeed, OffNoseSpeedConfig, ThrustHandoff,
};
use crate::controllers::InvalidConfig;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::guidance::plane::{PlaneGuidance, PlaneGuidanceConfig};
use crate::math::{DEG, bank_rad, saturate_symmetric, saturate_throttle};
use crate::params::{ParamError, Params, locate};
use crate::{
    AttitudeSetpoint, BlendSetpoint, Configurable, ControlInput, FlightMode, FlightModeReport,
    FlightRegime, Guidance, ModeRefusal, PlaneSetpoint, Saturation, Setpoint, VehicleState,
    WaypointCommand, WindEstimate,
};

mod deceleration;
#[cfg(test)]
mod tests;

use deceleration::BackDeceleration;

/// [`QuadPlaneGuidance`] settings: its two guidances', the transition's, forward thrust's, and
/// the off-nose speed cut's.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QuadPlaneGuidanceConfig {
    /// Hover guidance, on the lift rotors.
    pub hover: CopterGuidanceConfig,
    /// Wing-borne guidance, on the surfaces and pusher.
    pub plane: PlaneGuidanceConfig,
    /// The transition between them, its pusher, handoff blend, and roll limit.
    pub transition: LiftTransitionConfig,
    /// The pusher in copter flight; off by default.
    pub forward: ForwardThrustConfig,
    /// The waypoint speed cut in copter flight while the nose is off the waypoint; on by
    /// default.
    pub off_nose: OffNoseSpeedConfig,
}

impl QuadPlaneGuidanceConfig {
    /// `Ok` when each part validates.
    ///
    /// # Errors
    ///
    /// The first part's [`InvalidConfig`], in its section (`hover`, `plane`, `transition`,
    /// `forward`, `off_nose`).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.hover.validate().map_err(|e| e.in_section("hover"))?;
        self.plane.validate().map_err(|e| e.in_section("plane"))?;
        self.transition
            .validate()
            .map_err(|e| e.in_section("transition"))?;
        self.forward
            .validate()
            .map_err(|e| e.in_section("forward"))?;
        self.off_nose
            .validate()
            .map_err(|e| e.in_section("off_nose"))
    }
}

/// Guidance for a lift+cruise VTOL (quadplane): [`CopterGuidance`] on the lift rotors and
/// [`PlaneGuidance`] on the surfaces and pusher, with a [`LiftTransition`] choosing between
/// them and trading the lift across the transitions. The vehicle stays level throughout, so
/// there's no [`View`](crate::View).
///
/// Each tick:
///
/// 1. The requested regime is the command's, except that [`ControlInput::Land`] asks for
///    `Copter`: it lands as a copter, transitioning back first.
/// 2. The transition steps, unless the bank is past
///    [`roll_limit_rad`](LiftTransitionConfig::roll_limit_rad) in a transition: then it's cut
///    short ([`LiftTransition::cut_short`]).
/// 3. The mode picks the setpoint:
///
///    | Mode | Setpoint |
///    |---|---|
///    | `Copter` | hover guidance, on the command's input; after a fallback, until `Copter` is requested, a waypoint as a hover where copter flight took over; on a waypoint or land with [`ForwardThrust`] on, [`Setpoint::Blend`] at lift share 1: the hover's attitude with its nose-down pitch held to the limit, the plane side at that roll and pitch, the pusher on the rest |
///    | `ToPlane` | [`Setpoint::Blend`]: both sides wings level at the plane guidance's pitch for the command, within ArduPilot's transition limits; hover holding the altitude the transition began at, heading held; the pusher on the plane guidance's thrust, rising at most [`pusher_slew_per_s`](LiftTransitionConfig::pusher_slew_per_s), up to [`forward_thrust`](LiftTransitionConfig::forward_thrust); the step's lift share |
///    | `Plane` | plane guidance, on the command's input |
///    | `ToCopter` | [`Setpoint::Blend`]: both sides wings level, the nose up to slow it ([`back_decel_mps2`](LiftTransitionConfig::back_decel_mps2)); hover holding the altitude the back transition began at, heading held; the pusher stopped; the step's lift share |
///
///    The hover side holds altitude with the copter's altitude loop (ArduPilot holds it through
///    its airspeed wait, `SLT_Transition::update`). Going forward the pusher and pitch are the
///    plane's, as ArduPilot flies its throttle and pitch from the plane controller with pitch
///    held to 0 below 3 m/s of ground speed, ±3° while waiting, and ±8° on the ramp
///    (`SLT_Transition::set_FW_roll_pitch`): its airspeed loop keeps the lift ramp from
///    overspeeding, and its altitude loop trims the wing. Going back the pusher stops (PX4's
///    `Standard::update_transition_state`) and the nose comes up while it slows less than
///    `back_decel_mps2`, at most about 17° (PX4's back-transition deceleration,
///    `FlightTaskTransition::computeBackTransitionTiltSetpoint`; ArduPilot decelerates at its
///    `Q_TRANS_DECEL`): drag alone left the Alia at 31 m/s on its 28 s timeout (#496).
///
/// 4. A side that starts flying is primed: the plane guidance as the forward transition
///    begins and again entering `Plane` (through the transition its total energy control flies
///    from no airspeed, and only its thrust is used), the hover guidance as the back transition
///    begins. Entering `Plane`, the
///    pusher's thrust blends from where it was over
///    [`handoff_s`](LiftTransitionConfig::handoff_s) ([`ThrustHandoff`]).
///
///    Forward thrust flies only position-controlled input, as ArduPilot's `Q_FWD_THR_USE` 1
///    (`QuadPlane::get_vfwd_method`): sticks keep the pilot's tilt. A waypoint in
///    `Copter` reaches the hover guidance with its speed cut while the nose is off it
///    ([`OffNoseSpeed`]); land flies uncut, since it lands where the vehicle already is.
///
/// Pair it with a [`QuadPlaneStabilizer`](crate::stabilizers::vtol::QuadPlaneStabilizer).
/// Params chain the parts' (`GUID_*`, `PGUID_*`, `TRAN_*`, `FWD_*`, `OFFNOSE_*`).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{QuadPlaneGuidance, QuadPlaneGuidanceConfig};
/// use stable_flight::{
///     AttitudeCommand, ControlInput, FlightMode, FlightRegime, Guidance, Setpoint, Vec3,
///     VehicleState,
/// };
///
/// let mut guidance = QuadPlaneGuidance::new(QuadPlaneGuidanceConfig::default());
/// guidance.prime(&VehicleState::default());
///
/// let sticks = ControlInput::Attitude(AttitudeCommand { throttle: 0.6, ..Default::default() });
/// let high = VehicleState { position_m: Vec3::new(0.0, 0.0, 20.0), ..Default::default() };
/// let setpoint = guidance.guide(0.01, FlightRegime::Plane, &sticks, &high);
/// let Setpoint::Blend(blend) = setpoint else { panic!("{setpoint:?}") };
/// assert_eq!(blend.lift_share, 1.0); // the lift rotors hold it up while the pusher spins up
/// assert_eq!(guidance.flight_mode().unwrap().mode, FlightMode::ToPlane);
/// ```
pub struct QuadPlaneGuidance {
    config: QuadPlaneGuidanceConfig,
    hover: CopterGuidance,
    plane: PlaneGuidance,
    transition: LiftTransition,
    forward: ForwardThrust,
    off_nose: OffNoseSpeed,
    handoff: ThrustHandoff,
    fallback_hold: FallbackHold,
    deceleration: BackDeceleration,
    /// The mode the last tick flew.
    mode: FlightMode,
    /// The pusher's linear thrust on the last tick.
    pusher_thrust: f32,
    /// The altitude a transition holds, latched as it begins, in m.
    hold_altitude_m: f32,
}

/// Largest pitch, up or down, while the forward transition waits on airspeed: ArduPilot's
/// `Q_TRAN_PIT_MAX`.
const WAIT_PITCH_MAX_RAD: f32 = 3.0 * DEG;

/// Largest pitch, up or down, on the forward lift ramp: ArduPilot's `2 (Q_TRAN_PIT_MAX + 1°)`
/// (`SLT_Transition::set_FW_roll_pitch`).
const RAMP_PITCH_MAX_RAD: f32 = 8.0 * DEG;

/// Below this ground speed the forward transition holds pitch at 0, in m/s (ArduPilot's
/// `SLT_Transition::set_FW_roll_pitch`).
const LEVEL_BELOW_MPS: f32 = 3.0;

impl QuadPlaneGuidance {
    /// Guidance with `config`, in `Copter`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`QuadPlaneGuidanceConfig::validate`]).
    pub fn new(config: QuadPlaneGuidanceConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid QuadPlaneGuidanceConfig: {error}");
        }
        QuadPlaneGuidance {
            config,
            hover: CopterGuidance::new(config.hover),
            plane: PlaneGuidance::new(config.plane),
            transition: LiftTransition::new(config.transition),
            forward: ForwardThrust::new(config.forward),
            off_nose: OffNoseSpeed::new(config.off_nose),
            handoff: ThrustHandoff::new(config.transition.handoff_s),
            fallback_hold: FallbackHold::default(),
            deceleration: BackDeceleration::default(),
            mode: FlightMode::Copter,
            pusher_thrust: 0.0,
            hold_altitude_m: f32::NAN,
        }
    }

    /// This tick's step: the transition cut short if it has banked past the limit, else
    /// stepped.
    fn step(
        &mut self,
        dt: f32,
        requested: FlightRegime,
        state: &VehicleState,
    ) -> LiftTransitionStep {
        let transitioning = matches!(self.mode, FlightMode::ToPlane | FlightMode::ToCopter);
        // `>` is false for NaN: an unknown attitude leaves it to the timeouts.
        let rolled_over = transitioning
            && bank_rad(&state.attitude).abs() > self.config.transition.roll_limit_rad;
        if rolled_over {
            self.transition.cut_short(ModeRefusal::RollError)
        } else {
            self.transition.update(dt, requested, state)
        }
    }

    /// Readies the side that starts flying as the mode changes from the last tick's to `to`.
    fn enter(&mut self, to: FlightMode, state: &VehicleState) {
        match to {
            FlightMode::ToPlane => {
                self.plane.prime(state);
                self.hold_altitude_m = state.position_m.z;
                self.pusher_thrust = 0.0;
            }
            FlightMode::Plane => {
                // Fresh either way: out of the back transition it hasn't flown, and out of the
                // forward one its energy control flew from no airspeed, its demands and
                // integrals filled by a transition that only took its pusher thrust (the
                // X-Plane Alia dove 90 m at full nose-down on them, #454). The pusher's
                // thrust carries over in the handoff below.
                self.plane.prime(state);
                self.handoff = ThrustHandoff::new(self.config.transition.handoff_s);
                self.handoff.start(self.pusher_thrust);
            }
            FlightMode::ToCopter if self.mode == FlightMode::Plane => {
                self.hover.prime(state);
                self.hold_altitude_m = state.position_m.z;
                self.deceleration.reset();
            }
            _ => {}
        }
    }

    /// The hover side through a transition: the hover guidance holding
    /// [`hold_altitude_m`](Self::hold_altitude_m) where the vehicle is, wings level at
    /// `pitch_rad`. The waypoint moves with the vehicle at its own velocity, so the lateral loop
    /// asks for nothing, and its tilt is replaced anyway. Flying it as a waypoint keeps the
    /// altitude loop's state continuous with waypoint flight on either side of a transition.
    fn hold_level(&mut self, dt: f32, pitch_rad: f32, state: &VehicleState) -> AttitudeSetpoint {
        let hold = ControlInput::Waypoint(WaypointCommand {
            position_m: state.position_m.xy(),
            velocity_mps: Some(state.velocity_mps.xy()),
            altitude_m: self.hold_altitude_m,
            cruise_speed_mps: 0.0,
            heading_rad: None,
        });
        match self.hover.guide(dt, FlightRegime::Copter, &hold, state) {
            Setpoint::Attitude(setpoint) => AttitudeSetpoint {
                roll_rad: 0.0,
                pitch_rad,
                ..setpoint
            },
            _ => unreachable!("copter guidance flies a waypoint as an attitude"),
        }
    }

    /// `input` for the wing through the forward transition: a waypoint's altitude replaced by
    /// [`hold_altitude_m`](Self::hold_altitude_m), the one the lift rotors hold, so the wing
    /// pitches for level flight and carries the weight as the lift ramps out. Toward a lower
    /// altitude the wing would pitch nose down while the lift rotors hold, enter plane flight
    /// sinking and unloaded, and stall pulling out: the sim Alia sank at 19 m/s (#491). A climb
    /// or descent starts once the wing is flying. PX4 flies the transition at the wing's trim
    /// pitch whatever the altitude (`Standard::update_transition_state`). Other inputs pass
    /// through.
    fn at_held_altitude(&self, input: &ControlInput) -> ControlInput {
        match input {
            ControlInput::Waypoint(waypoint) => ControlInput::Waypoint(WaypointCommand {
                altitude_m: self.hold_altitude_m,
                ..*waypoint
            }),
            _ => *input,
        }
    }

    /// The parts whose params chain, in order.
    fn parts(&self) -> [&dyn Params; 5] {
        [
            &self.hover,
            &self.plane,
            &self.transition,
            &self.forward,
            &self.off_nose,
        ]
    }

    /// [`parts`](Self::parts), mutably, in the same order.
    fn parts_mut(&mut self) -> [&mut dyn Params; 5] {
        [
            &mut self.hover,
            &mut self.plane,
            &mut self.transition,
            &mut self.forward,
            &mut self.off_nose,
        ]
    }

    /// Copter flight: the hover guidance on `input`, a waypoint's speed cut while the nose is
    /// off it ([`OffNoseSpeed`]), its nose-down pitch handed to the pusher on a waypoint or
    /// land while [`ForwardThrust`] is on.
    fn copter(&mut self, dt: f32, input: &ControlInput, state: &VehicleState) -> Setpoint {
        let cut;
        let flown = match input {
            ControlInput::Waypoint(waypoint) => {
                cut = ControlInput::Waypoint(self.off_nose.cap(waypoint, state));
                &cut
            }
            _ => input,
        };
        let setpoint = self.hover.guide(dt, FlightRegime::Copter, flown, state);
        let positioned = matches!(input, ControlInput::Waypoint(_) | ControlInput::Land(_));
        let (true, Setpoint::Attitude(hover)) = (positioned && self.forward.enabled(), setpoint)
        else {
            self.pusher_thrust = 0.0;
            return setpoint;
        };
        let split = self.forward.split(hover.pitch_rad);
        self.pusher_thrust = split.thrust;
        Setpoint::Blend(BlendSetpoint {
            hover: AttitudeSetpoint {
                pitch_rad: split.pitch_rad,
                ..hover
            },
            plane: PlaneSetpoint {
                roll_rad: hover.roll_rad,
                pitch_rad: split.pitch_rad,
                thrust: split.thrust,
                ..PlaneSetpoint::default()
            },
            lift_share: 1.0,
        })
    }

    /// Copies the parts' configs back after a param write, so [`Configurable::config`] stays
    /// the one view of them.
    fn sync_config(&mut self) {
        self.config = QuadPlaneGuidanceConfig {
            hover: *self.hover.config(),
            plane: *self.plane.config(),
            transition: *self.transition.config(),
            forward: *self.forward.config(),
            off_nose: *self.off_nose.config(),
        };
    }
}

/// The forward transition's pitch limit, up or down: ArduPilot's transition limits, 0 below
/// [`LEVEL_BELOW_MPS`] of ground speed, then [`WAIT_PITCH_MAX_RAD`] while the lift rotors carry
/// it all, [`RAMP_PITCH_MAX_RAD`] on the ramp (`SLT_Transition::set_FW_roll_pitch`).
fn forward_pitch_limit_rad(lift_share: f32, state: &VehicleState) -> f32 {
    if lift_share < 1.0 {
        RAMP_PITCH_MAX_RAD
    } else if state.velocity_mps.xy().norm_squared() >= LEVEL_BELOW_MPS * LEVEL_BELOW_MPS {
        WAIT_PITCH_MAX_RAD
    } else {
        0.0
    }
}

/// The forward transition's pitch: the plane guidance's for the command (its altitude loop on
/// a waypoint, toward the altitude the transition holds; the stick in stabilized flight; 0 for
/// rates), within `limit_rad` either way.
fn forward_pitch_rad(wing: &Setpoint, limit_rad: f32) -> f32 {
    let Setpoint::Plane(plane) = wing else {
        return 0.0;
    };
    saturate_symmetric(plane.pitch_rad, limit_rad)
}

impl Guidance for QuadPlaneGuidance {
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint {
        let requested = if matches!(input, ControlInput::Land(_)) {
            FlightRegime::Copter
        } else {
            regime
        };
        let step = self.step(dt, requested, state);
        if step.mode != self.mode {
            self.enter(step.mode, state);
        }
        // The drag model needs all thrust along the lift rotors' axis: no transition, and no
        // pusher thrust on the last tick, which this tick's state still carries.
        self.hover
            .set_wind_paused(step.mode != FlightMode::Copter || self.pusher_thrust > 0.0);
        let transition = self.config.transition;

        let setpoint = match step.mode {
            FlightMode::ToPlane => {
                let wing_input = self.at_held_altitude(input);
                // The plane guidance's energy control flies within the transition's limits, so
                // its pitch integral stops where the pitch does
                // (`SLT_Transition::set_FW_roll_pitch`).
                let limit_rad = forward_pitch_limit_rad(step.lift_share, state);
                let wing = self.plane.guide_capped(
                    dt,
                    FlightRegime::Plane,
                    &wing_input,
                    state,
                    Some(limit_rad),
                );
                let rising = self.pusher_thrust + transition.pusher_slew_per_s * dt;
                self.pusher_thrust = saturate_throttle(wing.thrust())
                    .min(rising)
                    .min(transition.forward_thrust);
                let pitch_rad = forward_pitch_rad(&wing, limit_rad);
                Setpoint::Blend(BlendSetpoint {
                    hover: self.hold_level(dt, pitch_rad, state),
                    plane: PlaneSetpoint {
                        pitch_rad,
                        thrust: self.pusher_thrust,
                        ..PlaneSetpoint::default()
                    },
                    lift_share: step.lift_share,
                })
            }
            FlightMode::Plane => {
                let setpoint = self.plane.guide(dt, FlightRegime::Plane, input, state);
                let setpoint = if self.handoff.active() {
                    setpoint.with_thrust(self.handoff.apply(dt, setpoint.thrust()))
                } else {
                    setpoint
                };
                self.pusher_thrust = setpoint.thrust();
                setpoint
            }
            FlightMode::ToCopter => {
                self.pusher_thrust = 0.0;
                let pitch_rad = self.deceleration.pitch_rad(
                    dt,
                    state,
                    transition.back_decel_mps2,
                    transition.back_decel_i,
                );
                Setpoint::Blend(BlendSetpoint {
                    hover: self.hold_level(dt, pitch_rad, state),
                    plane: PlaneSetpoint {
                        pitch_rad,
                        ..PlaneSetpoint::default()
                    },
                    lift_share: step.lift_share,
                })
            }
            _ => {
                let fallen_back = self.transition.fallen_back();
                let flown = self.fallback_hold.input(fallen_back, input, state);
                self.copter(dt, &flown, state)
            }
        };
        self.mode = step.mode;
        setpoint
    }

    /// Primes both guidances and the transition (back to `Copter`), and ends any blend.
    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.transition.prime(state);
        self.handoff = ThrustHandoff::new(self.config.transition.handoff_s);
        self.fallback_hold.clear();
        self.deceleration.reset();
        self.mode = FlightMode::Copter;
        self.pusher_thrust = 0.0;
        self.hold_altitude_m = f32::NAN;
    }

    /// The side that flew the last tick: plane in `Plane`, hover otherwise (its altitude loop
    /// flies the transitions).
    fn saturation(&self) -> Saturation {
        match self.mode {
            FlightMode::Plane => self.plane.saturation(),
            _ => self.hover.saturation(),
        }
    }

    /// The transition's report.
    fn flight_mode(&self) -> Option<FlightModeReport> {
        Some(self.transition.report())
    }

    /// The hover guidance's, updated while it flies.
    fn wind(&self) -> Option<WindEstimate> {
        self.hover.wind()
    }
}

impl Configurable for QuadPlaneGuidance {
    type Config = QuadPlaneGuidanceConfig;

    fn config(&self) -> &QuadPlaneGuidanceConfig {
        &self.config
    }

    /// Validates the whole config first, then hands each part its own.
    fn set_config(&mut self, config: QuadPlaneGuidanceConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.hover.set_config(config.hover)?;
        self.plane.set_config(config.plane)?;
        self.transition.set_config(config.transition)?;
        self.forward.set_config(config.forward)?;
        self.off_nose.set_config(config.off_nose)?;
        self.config = config;
        Ok(())
    }
}

/// The parts' params, chained: hover (`GUID_*`), plane (`PGUID_*`), transition (`TRAN_*`),
/// forward thrust (`FWD_*`), off-nose speed (`OFFNOSE_*`).
impl Params for QuadPlaneGuidance {
    fn param_count(&self) -> usize {
        self.parts().iter().map(|part| part.param_count()).sum()
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        let (part, i) = locate(&self.parts(), index)?;
        self.parts()[part].param_name(i)
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        let (part, i) = locate(&self.parts(), index)?;
        self.parts()[part].param_at(i)
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        let (part, i) = locate(&self.parts(), index).ok_or(ParamError::Unknown)?;
        let result = self.parts_mut()[part].set_param_at(i, value);
        self.sync_config();
        result
    }
}
