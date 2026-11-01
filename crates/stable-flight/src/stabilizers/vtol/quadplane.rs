//! [`QuadPlaneStabilizer`]: a lift+cruise VTOL's lift rotors on the copter stabilizer, its
//! surfaces and pusher on the plane stabilizer.

use crate::controllers::InvalidConfig;
use crate::params::{ParamError, Params, locate};
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use crate::stabilizers::plane::{PlaneStabilizer, PlaneStabilizerConfig};
use crate::tuning::{ProbeConfig, ProbeStatus, Tunable};
use crate::{
    Configurable, ControlOutput, CruiseOutput, Saturation, Setpoint, Stabilizer, Vec3, VehicleState,
};

#[cfg(test)]
mod tests;

/// [`QuadPlaneStabilizer`] settings: its two stabilizers'.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct QuadPlaneStabilizerConfig {
    /// Hover stabilizer, on the lift rotors.
    pub hover: CopterStabilizerConfig,
    /// Wing-borne stabilizer, on the surfaces and pusher.
    pub plane: PlaneStabilizerConfig,
}

impl QuadPlaneStabilizerConfig {
    /// `Ok` when both parts validate.
    ///
    /// # Errors
    ///
    /// The first part's [`InvalidConfig`], in its section (`hover`, `plane`).
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        self.hover.validate().map_err(|e| e.in_section("hover"))?;
        self.plane.validate().map_err(|e| e.in_section("plane"))
    }
}

/// Which stabilizers flew a tick.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Sides {
    Hover,
    Plane,
    Both,
}

impl Sides {
    /// The sides `setpoint` flies.
    fn of(setpoint: &Setpoint) -> Self {
        match setpoint {
            Setpoint::Plane(_) | Setpoint::PlaneRate(_) => Sides::Plane,
            Setpoint::Blend(_) => Sides::Both,
            _ => Sides::Hover,
        }
    }

    fn hover(self) -> bool {
        self != Sides::Plane
    }

    fn plane(self) -> bool {
        self != Sides::Hover
    }
}

/// A lift+cruise VTOL's (quadplane's) stabilizer: a [`CopterStabilizer`] on the lift rotors
/// ([`ControlOutput`]'s `thrust` and `moments`) and a [`PlaneStabilizer`] on the surfaces and
/// pusher ([`cruise`](ControlOutput::cruise)). Pair it with a mixer that splits the two, such
/// as [`TableMixer::quadplane`](crate::mixers::TableMixer::quadplane).
///
/// The setpoint picks the sides:
///
/// | Setpoint | Lift rotors | Surfaces and pusher |
/// |---|---|---|
/// | [`Attitude`](Setpoint::Attitude), [`Rate`](Setpoint::Rate), [`BodyRate`](Setpoint::BodyRate) | copter stabilizer | neutral, pusher stopped |
/// | [`Plane`](Setpoint::Plane), [`PlaneRate`](Setpoint::PlaneRate) | stopped | plane stabilizer |
/// | [`Blend`](Setpoint::Blend) | copter stabilizer on its `hover`, thrust and moments × `lift_share` | plane stabilizer on its `plane`, in full |
///
/// In a hover the surfaces are held neutral (PX4's `VT_ELEV_MC_LOCK`,
/// `vtol_att_control_params.yaml`), and in a blend the lift rotors' output is weighted and the
/// surfaces' isn't (PX4 `standard.cpp:294-368`). A side that starts flying is primed, so it
/// starts without the integrals of its last flight (PX4 resets the idle side's rate
/// integrators, `FixedwingRateControl.cpp:266-269`); a side that keeps flying from a blend
/// keeps them.
///
/// Autotune probes go to the hover side. Params chain the parts' (`STAB_*`, `PSTAB_*`).
///
/// # Examples
///
/// ```
/// use stable_flight::stabilizers::vtol::{QuadPlaneStabilizer, QuadPlaneStabilizerConfig};
/// use stable_flight::{AttitudeSetpoint, BlendSetpoint, PlaneSetpoint, Setpoint, Stabilizer};
/// use stable_flight::VehicleState;
///
/// let mut stabilizer = QuadPlaneStabilizer::new(QuadPlaneStabilizerConfig::default());
/// let blend = Setpoint::Blend(BlendSetpoint {
///     hover: AttitudeSetpoint { thrust: 0.8, ..Default::default() },
///     plane: PlaneSetpoint { thrust: 1.0, ..Default::default() },
///     lift_share: 0.5,
/// });
/// let output = stabilizer.stabilize(0.004, &blend, &VehicleState::default());
/// assert!((output.thrust - 0.32).abs() < 1e-6); // 0.8 through the thrust curve, halved
/// assert_eq!(output.cruise.unwrap().thrust, 1.0); // the pusher at full
/// ```
pub struct QuadPlaneStabilizer {
    config: QuadPlaneStabilizerConfig,
    hover: CopterStabilizer,
    plane: PlaneStabilizer,
    /// The sides the last tick flew.
    sides: Sides,
}

impl QuadPlaneStabilizer {
    /// A stabilizer with `config`, on the hover side.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`QuadPlaneStabilizerConfig::validate`]).
    pub fn new(config: QuadPlaneStabilizerConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid QuadPlaneStabilizerConfig: {error}");
        }
        QuadPlaneStabilizer {
            config,
            hover: CopterStabilizer::new(config.hover),
            plane: PlaneStabilizer::new(config.plane),
            sides: Sides::Hover,
        }
    }

    /// Primes the sides `sides` adds to the last tick's.
    fn prime_entering(&mut self, sides: Sides, state: &VehicleState) {
        if sides.hover() && !self.sides.hover() {
            self.hover.prime(state);
        }
        if sides.plane() && !self.sides.plane() {
            self.plane.prime(state);
        }
        self.sides = sides;
    }

    /// Copies the parts' configs back after a param write, so [`Configurable::config`] stays
    /// the one view of them.
    fn sync_config(&mut self) {
        self.config = QuadPlaneStabilizerConfig {
            hover: *self.hover.config(),
            plane: *self.plane.config(),
        };
    }
}

impl Stabilizer for QuadPlaneStabilizer {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        self.prime_entering(Sides::of(setpoint), state);
        match setpoint {
            Setpoint::Plane(_) | Setpoint::PlaneRate(_) => {
                let cruise = self.plane.stabilize(dt, setpoint, state);
                ControlOutput {
                    cruise: Some(CruiseOutput {
                        thrust: cruise.thrust,
                        moments: cruise.moments,
                    }),
                    ..ControlOutput::default()
                }
            }
            Setpoint::Blend(blend) => {
                // NaN: the lift rotors carry it all.
                let share = if blend.lift_share.is_nan() {
                    1.0
                } else {
                    blend.lift_share.clamp(0.0, 1.0)
                };
                let lift = self
                    .hover
                    .stabilize(dt, &Setpoint::Attitude(blend.hover), state);
                let cruise = self
                    .plane
                    .stabilize(dt, &Setpoint::Plane(blend.plane), state);
                ControlOutput {
                    thrust: lift.thrust * share,
                    moments: lift.moments * share,
                    cruise: Some(CruiseOutput {
                        thrust: cruise.thrust,
                        moments: cruise.moments,
                    }),
                }
            }
            _ => ControlOutput {
                cruise: Some(CruiseOutput {
                    thrust: 0.0,
                    moments: Vec3::zeros(),
                }),
                ..self.hover.stabilize(dt, setpoint, state)
            },
        }
    }

    /// Primes both sides; the hover side flies next.
    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.sides = Sides::Hover;
    }

    /// The sides that flew the last tick, merged in a blend.
    fn saturation(&self) -> Saturation {
        match self.sides {
            Sides::Hover => self.hover.saturation(),
            Sides::Plane => self.plane.saturation(),
            Sides::Both => self.hover.saturation().merge(self.plane.saturation()),
        }
    }
}

impl Configurable for QuadPlaneStabilizer {
    type Config = QuadPlaneStabilizerConfig;

    fn config(&self) -> &QuadPlaneStabilizerConfig {
        &self.config
    }

    /// Validates the whole config first, then hands each part its own.
    fn set_config(&mut self, config: QuadPlaneStabilizerConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.hover.set_config(config.hover)?;
        self.plane.set_config(config.plane)?;
        self.config = config;
        Ok(())
    }
}

/// The parts' params, chained: hover (`STAB_*`), plane (`PSTAB_*`).
impl Params for QuadPlaneStabilizer {
    fn param_count(&self) -> usize {
        self.hover.param_count() + self.plane.param_count()
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        match locate(&[&self.hover, &self.plane], index)? {
            (0, i) => self.hover.param_name(i),
            (_, i) => self.plane.param_name(i),
        }
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        match locate(&[&self.hover, &self.plane], index)? {
            (0, i) => self.hover.param_at(i),
            (_, i) => self.plane.param_at(i),
        }
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        let result = match locate(&[&self.hover, &self.plane], index).ok_or(ParamError::Unknown)? {
            (0, i) => self.hover.set_param_at(i, value),
            (_, i) => self.plane.set_param_at(i, value),
        };
        self.sync_config();
        result
    }
}

/// Autotune runs on the hover side.
impl Tunable<TunedLoop> for QuadPlaneStabilizer {
    fn start_probe(&mut self, target: TunedLoop, config: ProbeConfig) {
        self.hover.start_probe(target, config);
    }

    fn probe(&self) -> Option<(TunedLoop, ProbeStatus)> {
        self.hover.probe()
    }

    fn stop_probe(&mut self) -> Option<(TunedLoop, ProbeStatus)> {
        self.hover.stop_probe()
    }
}
