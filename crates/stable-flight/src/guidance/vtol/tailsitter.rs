//! [`TailsitterGuidance`]: hover and plane guidance under one [`Transition`].

use core::f32::consts::FRAC_PI_2;

use super::{ThrustHandoff, Transition, TransitionConfig, TransitionStep};
use crate::controllers::InvalidConfig;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::guidance::plane::{PlaneGuidance, PlaneGuidanceConfig};
use crate::math::bank_rad;
use crate::params::{ParamError, Params, locate};
use crate::{
    AbsoluteAttitudeCommand, AttitudeSetpoint, Configurable, ControlInput, FlightMode,
    FlightModeReport, FlightRegime, Guidance, ModeRefusal, PlaneSetpoint, Saturation, Setpoint,
    VehicleState, View,
};

#[cfg(test)]
mod tests;

/// [`TailsitterGuidance`] settings: its two guidances' and the transition's.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TailsitterGuidanceConfig {
    /// Hover guidance, flown in the tailsitter [`View`].
    pub hover: CopterGuidanceConfig,
    /// Wing-borne guidance, in the airplane frame.
    pub plane: PlaneGuidanceConfig,
    /// The transition between them, its thrusts, handoff blend, and roll limit.
    pub transition: TransitionConfig,
}

impl TailsitterGuidanceConfig {
    /// `Ok` when each part validates.
    ///
    /// # Errors
    ///
    /// The first part's [`InvalidConfig`], in its section (`hover`, `plane`, `transition`).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.hover.validate().map_err(|e| e.in_section("hover"))?;
        self.plane.validate().map_err(|e| e.in_section("plane"))?;
        self.transition
            .validate()
            .map_err(|e| e.in_section("transition"))
    }
}

/// Guidance for a tailsitter: hover guidance (copter guidance in the tailsitter [`View`]) and
/// [`PlaneGuidance`], with a [`Transition`] choosing between them and flying the transitions.
///
/// Each tick:
///
/// 1. The requested regime is the command's, except that [`ControlInput::Land`] asks for
///    `Copter`: a tailsitter lands as a copter, transitioning back first.
/// 2. The transition steps, unless it has rolled past
///    [`roll_limit_rad`](TransitionConfig::roll_limit_rad): then it's cut short
///    ([`Transition::cut_short`]), checked first because a sideways lean also tilts the nose
///    from vertical, which would count as progress. The roll is the hover view's going
///    forward and the airplane's bank coming back (ArduPilot `tailsitter.cpp`,
///    `transition_fw_complete` and `transition_vtol_complete`).
/// 3. The mode picks the setpoint:
///
///    | Mode | Setpoint |
///    |---|---|
///    | `Copter` | hover guidance, on the command's input |
///    | `ToPlane` | hover guidance on an absolute attitude: wings level, view pitch −tilt, heading held, thrust the larger of [`forward_thrust`](TransitionConfig::forward_thrust) and the hover thrust when it began; a [`transition`](AttitudeSetpoint::transition) ramp |
///    | `Plane` | plane guidance, on the command's input |
///    | `ToCopter` | plane guidance on an absolute attitude: wings level, pitch π/2 − tilt, [`back_thrust`](TransitionConfig::back_thrust); a [`transition`](PlaneSetpoint::transition) ramp |
///
///    A ramp's pitch isn't held to the stabilizer's limits (hover tilt, plane pitch), which
///    would stop it short of completing.
///
/// 4. On a handoff (`ToPlane` → `Plane`, `ToCopter` → `Copter`) the incoming guidance is
///    primed, and thrust blends from its last value over
///    [`handoff_s`](TransitionConfig::handoff_s) ([`ThrustHandoff`]).
/// 5. After the handoff into `Plane`, stick throttle (attitude or rate sticks) is held at
///    least at the plane's [`cruise_thrust`](PlaneGuidanceConfig::cruise_thrust) until the
///    stick reaches it: the forward ramp flies a fixed thrust and ignores the stick, so a stick
///    left low would otherwise cut thrust as the wing takes over. Offboard thrust is flown as
///    given.
///
/// Pair it with a [`TailsitterStabilizer`](crate::stabilizers::vtol::TailsitterStabilizer),
/// which flies each setpoint on the matching side. Params chain the parts' (`GUID_*`,
/// `PGUID_*`, `TRAN_*`).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{TailsitterGuidance, TailsitterGuidanceConfig};
/// use stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight::{
///     AttitudeCommand, ControlInput, FlightMode, FlightRegime, Guidance, Setpoint, Vec3,
///     VehicleState,
/// };
///
/// let mut guidance = TailsitterGuidance::new(TailsitterGuidanceConfig::default());
/// let hover = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(0.0, core::f32::consts::FRAC_PI_2, 0.0),
///     ..Default::default()
/// };
/// guidance.prime(&hover);
///
/// let sticks = ControlInput::Attitude(AttitudeCommand { throttle: 0.6, ..Default::default() });
/// let high = VehicleState { position_m: Vec3::new(0.0, 0.0, 20.0), ..hover };
/// let setpoint = guidance.guide(0.01, FlightRegime::Plane, &sticks, &high);
/// assert!(matches!(setpoint, Setpoint::Attitude(_))); // the hover side flies the ramp
/// assert_eq!(guidance.flight_mode().unwrap().mode, FlightMode::ToPlane);
/// ```
pub struct TailsitterGuidance {
    config: TailsitterGuidanceConfig,
    hover: View<CopterGuidance>,
    plane: PlaneGuidance,
    transition: Transition,
    handoff: ThrustHandoff,
    /// The mode the last tick flew.
    mode: FlightMode,
    /// The last setpoint's thrust: where a handoff blend starts.
    last_thrust: f32,
    /// The forward ramp's thrust, set when it begins.
    forward_thrust: f32,
    /// Least stick thrust after a handoff into `Plane`, until the stick reaches it.
    throttle_floor: Option<f32>,
}

impl TailsitterGuidance {
    /// Guidance with `config`, in `Copter`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`TailsitterGuidanceConfig::validate`]).
    pub fn new(config: TailsitterGuidanceConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid TailsitterGuidanceConfig: {error}");
        }
        TailsitterGuidance {
            config,
            hover: View::tailsitter(CopterGuidance::new(config.hover)),
            plane: PlaneGuidance::new(config.plane),
            transition: Transition::new(config.transition),
            handoff: ThrustHandoff::new(config.transition.handoff_s),
            mode: FlightMode::Copter,
            last_thrust: 0.0,
            forward_thrust: 0.0,
            throttle_floor: None,
        }
    }

    /// This tick's step: the transition cut short if it has rolled past the limit, else
    /// stepped. The check comes first: a sideways lean also tilts the nose from vertical,
    /// which the transition would otherwise count as progress.
    fn step(&mut self, dt: f32, requested: FlightRegime, state: &VehicleState) -> TransitionStep {
        let attitude = match self.mode {
            // The hover view's roll: the thrust axis leaning out of the ramp's plane.
            FlightMode::ToPlane => Some(state.attitude * self.hover.rotation()),
            // The airplane's bank.
            FlightMode::ToCopter => Some(state.attitude),
            _ => None,
        };
        // `>` is false for NaN: an unknown attitude leaves it to the timeout.
        let rolled_over = attitude.is_some_and(|attitude| {
            bank_rad(&attitude).abs() > self.config.transition.roll_limit_rad
        });
        if rolled_over {
            self.transition.cut_short(ModeRefusal::RollError)
        } else {
            self.transition.update(dt, requested, state)
        }
    }

    /// Primes the guidance taking over at a handoff and starts the thrust blend.
    fn hand_off(&mut self, to: FlightMode, state: &VehicleState) {
        match to {
            FlightMode::Plane => {
                self.plane.prime(state);
                self.throttle_floor = Some(self.config.plane.cruise_thrust);
            }
            _ => self.hover.prime(state),
        }
        self.handoff = ThrustHandoff::new(self.config.transition.handoff_s);
        self.handoff.start(self.last_thrust);
    }

    /// `setpoint` with stick throttle held at the floor set by the handoff into `Plane`, until
    /// the stick reaches it; offboard thrust passes through.
    fn floor_stick_throttle(&mut self, setpoint: Setpoint, input: &ControlInput) -> Setpoint {
        let Some(floor) = self.throttle_floor else {
            return setpoint;
        };
        if !input.is_sticks() || setpoint.thrust() >= floor {
            self.throttle_floor = None;
            return setpoint;
        }
        setpoint.with_thrust(floor)
    }

    /// Copies the parts' configs back after a param write, so [`Configurable::config`] stays
    /// the one view of them.
    fn sync_config(&mut self) {
        self.config = TailsitterGuidanceConfig {
            hover: *self.hover.config(),
            plane: *self.plane.config(),
            transition: *self.transition.config(),
        };
    }
}

/// `setpoint` marked as a transition ramp, so the stabilizer flies its pitch past the side's
/// limit to the handover.
fn transition_ramp(setpoint: Setpoint) -> Setpoint {
    match setpoint {
        Setpoint::Attitude(attitude) => Setpoint::Attitude(AttitudeSetpoint {
            transition: true,
            ..attitude
        }),
        Setpoint::Plane(plane) => Setpoint::Plane(PlaneSetpoint {
            transition: true,
            ..plane
        }),
        other => other,
    }
}

impl Guidance for TailsitterGuidance {
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
        if step.handoff {
            self.hand_off(step.mode, state);
        }
        let transition = &self.config.transition;
        if step.mode == FlightMode::ToPlane && self.mode != FlightMode::ToPlane {
            self.forward_thrust = self.last_thrust.max(transition.forward_thrust);
        }
        let tilt_rad = step.tilt_target_rad.unwrap_or(0.0);

        let setpoint = match step.mode {
            FlightMode::ToPlane => {
                let ramp = ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                    roll_rad: 0.0,
                    pitch_rad: -tilt_rad,
                    heading_rad: f32::NAN,
                    thrust: self.forward_thrust,
                });
                transition_ramp(self.hover.guide(dt, FlightRegime::Copter, &ramp, state))
            }
            FlightMode::Plane => {
                let setpoint = self.plane.guide(dt, FlightRegime::Plane, input, state);
                self.floor_stick_throttle(setpoint, input)
            }
            FlightMode::ToCopter => {
                let ramp = ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                    roll_rad: 0.0,
                    pitch_rad: FRAC_PI_2 - tilt_rad,
                    heading_rad: f32::NAN,
                    thrust: transition.back_thrust,
                });
                transition_ramp(self.plane.guide(dt, FlightRegime::Plane, &ramp, state))
            }
            _ => self.hover.guide(dt, FlightRegime::Copter, input, state),
        };
        let setpoint = if self.handoff.active() {
            setpoint.with_thrust(self.handoff.apply(dt, setpoint.thrust()))
        } else {
            setpoint
        };
        if step.mode != FlightMode::Plane {
            self.throttle_floor = None;
        }
        self.mode = step.mode;
        self.last_thrust = setpoint.thrust();
        setpoint
    }

    /// Primes both guidances and the transition (back to `Copter`), and ends any blend.
    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.transition.prime(state);
        self.handoff = ThrustHandoff::new(self.config.transition.handoff_s);
        self.mode = FlightMode::Copter;
        self.last_thrust = 0.0;
        self.throttle_floor = None;
    }

    /// The side that flew the last tick: hover in `Copter` and `ToPlane`, plane otherwise.
    fn saturation(&self) -> Saturation {
        match self.mode {
            FlightMode::Plane | FlightMode::ToCopter => self.plane.saturation(),
            _ => self.hover.saturation(),
        }
    }

    /// The transition's report.
    fn flight_mode(&self) -> Option<FlightModeReport> {
        Some(self.transition.report())
    }
}

impl Configurable for TailsitterGuidance {
    type Config = TailsitterGuidanceConfig;

    fn config(&self) -> &TailsitterGuidanceConfig {
        &self.config
    }

    /// Validates the whole config first, then hands each part its own.
    fn set_config(&mut self, config: TailsitterGuidanceConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.hover.set_config(config.hover)?;
        self.plane.set_config(config.plane)?;
        self.transition.set_config(config.transition)?;
        self.config = config;
        Ok(())
    }
}

/// The parts' params, chained: hover (`GUID_*`), plane (`PGUID_*`), transition (`TRAN_*`).
impl Params for TailsitterGuidance {
    fn param_count(&self) -> usize {
        self.hover.param_count() + self.plane.param_count() + self.transition.param_count()
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        match locate(&[&self.hover, &self.plane, &self.transition], index)? {
            (0, i) => self.hover.param_name(i),
            (1, i) => self.plane.param_name(i),
            (_, i) => self.transition.param_name(i),
        }
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        match locate(&[&self.hover, &self.plane, &self.transition], index)? {
            (0, i) => self.hover.param_at(i),
            (1, i) => self.plane.param_at(i),
            (_, i) => self.transition.param_at(i),
        }
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        let parts: [&dyn Params; 3] = [&self.hover, &self.plane, &self.transition];
        let result = match locate(&parts, index).ok_or(ParamError::Unknown)? {
            (0, i) => self.hover.set_param_at(i, value),
            (1, i) => self.plane.set_param_at(i, value),
            (_, i) => self.transition.set_param_at(i, value),
        };
        self.sync_config();
        result
    }
}
