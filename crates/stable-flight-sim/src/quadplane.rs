//! The simulated lift+cruise VTOL as a flight-loop backend: see [`SimQuadPlane`].

use core::time::Duration;

use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{Actuator, ActuatorCommand, StateSource, VehicleState};

use crate::lockstep::Lockstep;
use crate::model::ModelError;
use crate::perturbations::Perturbations;
use crate::quadplane_model::QuadPlaneModel;
use crate::start::Start;
use crate::wind::Wind;
use crate::wing::{FixedWing, Rotors};

#[cfg(test)]
mod tests;

/// A simulated lift+cruise VTOL (quadplane) that a [`FlightLoop`](stable_flight::FlightLoop)
/// flies, with the [timing contract](crate::SimCopter#timing-contract) of
/// [`SimCopter`](crate::SimCopter).
///
/// The physics: [`SimPlane`](crate::SimPlane)'s rigid body, aerodynamics, and surfaces, with
/// [`QuadPlaneModel`]'s lift rotors thrusting along body `−z` (their torques from the mixer's
/// motor rows, as on [`SimCopter`](crate::SimCopter)) and its pusher along body `x`, all at
/// once. It reports the airplane body frame, which is also the hover's: it hovers level. It
/// starts where [`with_start`](Self::with_start) puts it, by default resting level on its gear
/// at the origin, nose north, and takes off vertically on the lift rotors.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use stable_flight_sim::{SimQuadPlane, airframes};
///
/// let airframe = airframes::quadplane();
/// let mut quadplane = SimQuadPlane::new(airframe.model, &airframe.mixer).expect("valid airframe");
///
/// // The lift rotors at full for a second climb; the pusher stays stopped.
/// let mut lift = ActuatorCommand::default();
/// lift.motors[..4].copy_from_slice(&[1.0; 4]);
/// quadplane.actuate(&lift);
/// for _ in 0..250 {
///     quadplane.state(Duration::from_millis(4));
/// }
/// assert!(quadplane.state(Duration::ZERO).position_m.z > 1.0);
/// ```
#[derive(Debug, Clone)]
pub struct SimQuadPlane {
    lockstep: Lockstep,
    dynamics: FixedWing,
}

impl SimQuadPlane {
    /// A quadplane with `model`'s physics whose lift rotors, pusher, and surfaces `mixer`'s
    /// rows drive (see [`QuadPlaneModel`]), resting on the ground in calm air.
    ///
    /// # Errors
    ///
    /// [`QuadPlaneModel::validate`]'s errors.
    pub fn new<const N: usize>(
        model: QuadPlaneModel,
        mixer: &TableMixer<N>,
    ) -> Result<Self, ModelError> {
        model.validate()?;
        Ok(SimQuadPlane {
            lockstep: Lockstep::new(),
            dynamics: FixedWing::new(
                model.wing,
                mixer,
                Rotors::quadplane(&model),
                UnitQuaternion::identity(),
            ),
        })
    }

    /// This quadplane starting at `start`, e.g. in flight ([`Start::level`]).
    pub fn with_start(mut self, start: Start) -> Self {
        self.lockstep.place(&start);
        self
    }

    /// This quadplane flying through `wind`.
    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.lockstep.set_wind(wind);
        self
    }

    /// This quadplane under `perturbations` (all off by default); motor mismatch scales the
    /// lift rotors and the pusher by slot.
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

    /// Whether the quadplane rests on the ground (lift rotors and wing below its weight there).
    pub fn on_ground(&self) -> bool {
        self.lockstep.resting()
    }

    /// Angle of attack over the last integration step, in rad.
    pub fn angle_of_attack_rad(&self) -> f32 {
        self.dynamics.aero().alpha_rad
    }

    /// Each lift rotor's thrust, in N, by motor slot (zero on the pusher's).
    pub fn lift_thrusts_n(&self) -> [f32; ActuatorCommand::SLOTS] {
        self.dynamics.lift_thrusts_n()
    }
}

impl StateSource for SimQuadPlane {
    /// Steps `dt` and returns the new state; `Duration::ZERO` returns the current state
    /// unchanged (see the [timing contract](crate::SimCopter#timing-contract)).
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.lockstep.state(&mut self.dynamics, dt)
    }
}

impl Actuator for SimQuadPlane {
    /// Queues the motor and servo commands for the next [`state`](StateSource::state) step,
    /// after any [`actuator_delay_s`](Perturbations::actuator_delay_s).
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.lockstep.push(actuator_command);
    }
}
