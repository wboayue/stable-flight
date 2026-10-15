//! VTOL stabilization: [`TailsitterStabilizer`] flies each setpoint on the side it belongs to,
//! hover or plane; [`QuadPlaneStabilizer`] flies a lift+cruise VTOL's lift rotors and surfaces,
//! both at once through a transition.

use crate::controllers::InvalidConfig;
use crate::params::{ParamError, Params, chained_at, chained_count, chained_name, chained_set};
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::stabilizers::plane::{PlaneStabilizer, PlaneStabilizerConfig};
use crate::tuning::{ProbeConfig, ProbeStatus, Tunable};
use crate::{Configurable, ControlOutput, Saturation, Setpoint, Stabilizer, VehicleState, View};

mod quadplane;
#[cfg(test)]
mod tests;

pub use quadplane::{QuadPlaneStabilizer, QuadPlaneStabilizerConfig};

/// [`TailsitterStabilizer`] settings: its two stabilizers'.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TailsitterStabilizerConfig {
    /// Hover stabilizer, flown in the tailsitter [`View`] (its gains in view axes).
    pub hover: CopterStabilizerConfig,
    /// Wing-borne stabilizer, in the airplane frame.
    pub plane: PlaneStabilizerConfig,
}

impl TailsitterStabilizerConfig {
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

/// A tailsitter's stabilizer: copter stabilization in the tailsitter [`View`] for
/// [`Setpoint::Attitude`], [`Setpoint::Rate`], and [`Setpoint::BodyRate`], and a
/// [`PlaneStabilizer`] for [`Setpoint::Plane`] and [`Setpoint::PlaneRate`]. The setpoint picks
/// the side, so the guidance's mode reaches it without a copy: one side runs per tick, and when
/// the side changes the incoming stabilizer is primed, so it starts without the integrals of
/// the flight it last flew (PX4 resets the inactive side's rate integrators,
/// `FixedwingRateControl.cpp`, `MulticopterRateControl.cpp`).
///
/// Autotune probes go to the hover side. Params chain the parts' (`STAB_*`, `PSTAB_*`).
///
/// # Examples
///
/// ```
/// use stable_flight::stabilizers::vtol::{TailsitterStabilizer, TailsitterStabilizerConfig};
/// use stable_flight::{PlaneSetpoint, Setpoint, Stabilizer, VehicleState};
///
/// let mut stabilizer = TailsitterStabilizer::new(TailsitterStabilizerConfig::default());
/// let bank = Setpoint::Plane(PlaneSetpoint { roll_rad: 0.3, thrust: 0.6, ..Default::default() });
/// let output = stabilizer.stabilize(0.004, &bank, &VehicleState::default());
/// assert!(output.moments.x > 0.0); // the plane side rolls right
/// ```
pub struct TailsitterStabilizer {
    config: TailsitterStabilizerConfig,
    hover: View<CopterStabilizer>,
    plane: PlaneStabilizer,
    /// Whether the last tick flew the plane side.
    on_plane: bool,
}

impl TailsitterStabilizer {
    /// A stabilizer with `config`, on the hover side.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`TailsitterStabilizerConfig::validate`]).
    pub fn new(config: TailsitterStabilizerConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid TailsitterStabilizerConfig: {error}");
        }
        TailsitterStabilizer {
            config,
            hover: View::tailsitter(CopterStabilizer::new(config.hover)),
            plane: PlaneStabilizer::new(config.plane),
            on_plane: false,
        }
    }

    /// Copies the parts' configs back after a param write, so [`Configurable::config`] stays
    /// the one view of them.
    fn sync_config(&mut self) {
        self.config = TailsitterStabilizerConfig {
            hover: *self.hover.config(),
            plane: *self.plane.config(),
        };
    }
}

impl Stabilizer for TailsitterStabilizer {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        let on_plane = matches!(setpoint, Setpoint::Plane(_) | Setpoint::PlaneRate(_));
        if on_plane != self.on_plane {
            if on_plane {
                self.plane.prime(state);
            } else {
                self.hover.prime(state);
            }
            self.on_plane = on_plane;
        }
        if on_plane {
            self.plane.stabilize(dt, setpoint, state)
        } else {
            self.hover.stabilize(dt, setpoint, state)
        }
    }

    /// Primes both sides; the hover side flies next.
    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.on_plane = false;
    }

    /// The side that flew the last tick.
    fn saturation(&self) -> Saturation {
        if self.on_plane {
            self.plane.saturation()
        } else {
            self.hover.saturation()
        }
    }
}

impl Configurable for TailsitterStabilizer {
    type Config = TailsitterStabilizerConfig;

    fn config(&self) -> &TailsitterStabilizerConfig {
        &self.config
    }

    /// Validates the whole config first, then hands each part its own.
    fn set_config(&mut self, config: TailsitterStabilizerConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.hover.set_config(config.hover)?;
        self.plane.set_config(config.plane)?;
        self.config = config;
        Ok(())
    }
}

/// The parts' params, chained: hover (`STAB_*`), plane (`PSTAB_*`).
impl Params for TailsitterStabilizer {
    fn param_count(&self) -> usize {
        chained_count(&[&self.hover, &self.plane])
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        chained_name(&[&self.hover, &self.plane], index)
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        chained_at(&[&self.hover, &self.plane], index)
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        let result = chained_set(&mut [&mut self.hover, &mut self.plane], index, value);
        self.sync_config();
        result
    }
}

/// Autotune runs on the hover side.
impl Tunable for TailsitterStabilizer {
    type Target = <View<CopterStabilizer> as Tunable>::Target;

    fn start_probe(&mut self, target: Self::Target, config: ProbeConfig) {
        self.hover.start_probe(target, config);
    }

    fn probe(&self) -> Option<(Self::Target, ProbeStatus)> {
        self.hover.probe()
    }

    fn stop_probe(&mut self) -> Option<(Self::Target, ProbeStatus)> {
        self.hover.stop_probe()
    }
}
