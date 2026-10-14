//! The simulated tailsitter as a flight-loop backend: see [`SimTailsitter`].

use core::time::Duration;

use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{Actuator, ActuatorCommand, StateSource, Vec3, VehicleState, View};

use crate::lockstep::Lockstep;
use crate::model::ModelError;
use crate::perturbations::Perturbations;
use crate::start::Start;
use crate::tailsitter_model::TailsitterModel;
use crate::wind::Wind;
use crate::wing::{FixedWing, Rotors};

#[cfg(test)]
mod tests;

/// A simulated tailsitter that a [`FlightLoop`](stable_flight::FlightLoop) flies, with the
/// [timing contract](crate::SimCopter#timing-contract) of [`SimCopter`](crate::SimCopter).
///
/// The physics: [`SimPlane`](crate::SimPlane)'s rigid body, aerodynamics, and surfaces, with
/// [`TailsitterModel`]'s motors along the wing, each thrusting along body `x`, and the
/// surfaces in their wash. It reports the airplane body frame, so hovering nose up is 90° of
/// pitch, which [`View::tailsitter`] reads as level. It starts resting on its tail at the
/// origin, nose up, belly north, and rests that way whenever it's back on the ground.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use stable_flight_sim::{SimTailsitter, airframes};
///
/// let airframe = airframes::tailsitter();
/// let mut tailsitter = SimTailsitter::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let resting = tailsitter.state(Duration::ZERO);
/// assert!((resting.pitch_rad - core::f32::consts::FRAC_PI_2).abs() < 1e-3);
///
/// // Both motors at full for a second climb nose first.
/// tailsitter.actuate(&ActuatorCommand { motors: [1.0; 8], ..Default::default() });
/// for _ in 0..250 {
///     tailsitter.state(Duration::from_millis(4));
/// }
/// assert!(tailsitter.state(Duration::ZERO).position_m.z > 1.0);
/// ```
#[derive(Debug, Clone)]
pub struct SimTailsitter {
    lockstep: Lockstep,
    dynamics: FixedWing,
}

/// A tailsitter's attitude resting on its tail, relative to level: [`View::tailsitter`]'s
/// rotation undone, so the hover view reads level there.
fn resting_on_tail() -> UnitQuaternion<f32> {
    View::tailsitter(()).rotation().inverse()
}

impl SimTailsitter {
    /// A tailsitter with `model`'s physics whose motors and surfaces `mixer`'s rows drive (see
    /// [`TailsitterModel`]), resting on its tail at the origin in calm air.
    ///
    /// # Errors
    ///
    /// [`TailsitterModel::validate`]'s errors.
    pub fn new<const N: usize>(
        model: TailsitterModel,
        mixer: &TableMixer<N>,
    ) -> Result<Self, ModelError> {
        model.validate()?;
        let rest = resting_on_tail();
        let mut lockstep = Lockstep::new();
        lockstep.place(&Start {
            position_m: Vec3::zeros(),
            velocity_mps: Vec3::zeros(),
            attitude: rest,
            angular_velocity_radps: Vec3::zeros(),
        });
        Ok(SimTailsitter {
            lockstep,
            dynamics: FixedWing::new(model.wing, mixer, Rotors::tailsitter(&model), rest),
        })
    }

    /// This tailsitter flying through `wind`.
    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.lockstep.set_wind(wind);
        self
    }

    /// This tailsitter under `perturbations` (all off by default); motor mismatch scales each
    /// motor.
    ///
    /// # Errors
    ///
    /// [`Perturbations::validate`]'s errors.
    pub fn with_perturbations(mut self, perturbations: Perturbations) -> Result<Self, ModelError> {
        perturbations.validate()?;
        self.lockstep.set_perturbations(&perturbations);
        self.dynamics
            .set_thrust_scale(perturbations.motor_thrust_scale);
        Ok(self)
    }

    /// Simulated time since construction, in s.
    pub fn time_s(&self) -> f64 {
        self.lockstep.time_s()
    }

    /// Whether the tailsitter rests on its tail (thrust and lift below its weight there).
    pub fn on_ground(&self) -> bool {
        self.lockstep.resting()
    }
}

impl StateSource for SimTailsitter {
    /// Steps `dt` and returns the new state; `Duration::ZERO` returns the current state
    /// unchanged (see the [timing contract](crate::SimCopter#timing-contract)).
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.lockstep.state(&mut self.dynamics, dt)
    }
}

impl Actuator for SimTailsitter {
    /// Queues the motor and servo commands for the next [`state`](StateSource::state) step,
    /// after any [`actuator_delay_s`](Perturbations::actuator_delay_s).
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.lockstep.push(actuator_command);
    }
}
