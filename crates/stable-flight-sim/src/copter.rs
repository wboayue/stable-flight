//! The simulator as a flight-loop backend: see [`SimCopter`].

use core::time::Duration;

use stable_flight::mixers::{Slot, TableMixer};
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{Actuator, ActuatorCommand, StateSource, Vec3, VehicleState};

use crate::body::{Body, Loads, MassProperties};
use crate::ground::Ground;
use crate::lag;
use crate::lockstep::{Dynamics, Lockstep};
use crate::model::{ModelError, MultirotorModel};
use crate::motors::Motors;
use crate::perturbations::Perturbations;
use crate::wind::Wind;

#[cfg(test)]
mod tests;

/// A simulated multirotor that a [`FlightLoop`](stable_flight::FlightLoop) flies: a
/// [`StateSource`] and [`Actuator`] stepped in lockstep with the loop, with no wall clock.
///
/// The physics: a rigid body with [`MultirotorModel`]'s mass and diagonal inertia, under gravity,
/// motor thrust and torque, linear drag through the [`Wind`], and rotational damping, on a
/// ground plane at altitude 0. It starts at rest there, at the origin, level, nose north.
/// [`Perturbations`] add delay, gyro noise, a disturbance moment, and motor mismatch.
///
/// # Timing contract
///
/// [`actuate`](Actuator::actuate) stores the command. [`state`](StateSource::state)`(dt)`
/// integrates the last stored command over `dt` (in steps of at most
/// [`MultirotorModel::max_step_s`]) and returns
/// the state at the end, so tick *n* sees the result of tick *n − 1*'s command, a one-tick
/// zero-order hold. [`Perturbations::actuator_delay_s`] delays each command further, to the
/// integration step. The priming call (`dt == Duration::ZERO`) returns the current state
/// without stepping. State is exact (no estimation) apart from [`Perturbations::gyro`] on the
/// body rates.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use stable_flight_sim::{SimCopter, airframes};
///
/// let airframe = airframes::quad_x();
/// let mut vehicle = SimCopter::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// assert_eq!(vehicle.state(Duration::ZERO).position_m.z, 0.0);
///
/// // Full throttle for a second climbs.
/// vehicle.actuate(&ActuatorCommand { motors: [1.0; 8], ..Default::default() });
/// for _ in 0..250 {
///     vehicle.state(Duration::from_millis(4));
/// }
/// assert!(vehicle.state(Duration::ZERO).position_m.z > 1.0);
/// ```
#[derive(Debug, Clone)]
pub struct SimCopter {
    lockstep: Lockstep,
    dynamics: Multirotor,
}

/// The multirotor's physics: motors through the lag, thrust along body −z, linear drag, and
/// rotational damping.
#[derive(Debug, Clone)]
struct Multirotor {
    model: MultirotorModel,
    motors: Motors,
    /// The motor lag's step factor for the current step length.
    alpha: f32,
    /// Total thrust over the last step, in N.
    thrust_n: f32,
}

impl SimCopter {
    /// A vehicle with `model`'s physics whose motors are laid out by `mixer`'s rows (see
    /// [`MultirotorModel`]), at rest on the ground in calm air.
    ///
    /// # Errors
    ///
    /// [`MultirotorModel::validate`]'s errors, or [`ModelError::Servo`] when `mixer` drives a servo.
    pub fn new<const N: usize>(
        model: MultirotorModel,
        mixer: &TableMixer<N>,
    ) -> Result<Self, ModelError> {
        model.validate()?;
        if let Some(index) = mixer.rows().iter().find_map(|row| match row.slot() {
            Slot::Servo(index) => Some(index),
            Slot::Motor(_) => None,
        }) {
            return Err(ModelError::Servo(index));
        }
        Ok(SimCopter {
            lockstep: Lockstep::new(),
            dynamics: Multirotor {
                motors: Motors::new(mixer.rows(), &model.motor_layout()),
                model,
                alpha: 0.0,
                thrust_n: 0.0,
            },
        })
    }

    /// This vehicle flying through `wind`.
    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.lockstep.set_wind(wind);
        self
    }

    /// This vehicle under `perturbations` (all off by default).
    ///
    /// # Errors
    ///
    /// [`Perturbations::validate`]'s errors.
    pub fn with_perturbations(mut self, perturbations: Perturbations) -> Result<Self, ModelError> {
        perturbations.validate()?;
        self.lockstep.set_perturbations(&perturbations);
        self.dynamics
            .motors
            .set_thrust_scale(perturbations.motor_thrust_scale);
        Ok(self)
    }

    /// Simulated time since construction, in s.
    pub fn time_s(&self) -> f64 {
        self.lockstep.time_s()
    }

    /// Whether the vehicle rests on the ground (thrust below its weight there).
    pub fn on_ground(&self) -> bool {
        self.lockstep.resting()
    }

    /// Each motor's thrust, in N, by motor slot.
    pub fn motor_thrusts_n(&self) -> [f32; ActuatorCommand::SLOTS] {
        self.dynamics.motors.thrusts_n()
    }
}

impl Dynamics for Multirotor {
    fn mass(&self) -> MassProperties {
        MassProperties {
            mass_kg: self.model.mass_kg,
            inertia_kgm2: self.model.inertia_kgm2,
        }
    }

    fn ground(&self) -> Ground {
        Ground {
            mass_kg: self.model.mass_kg,
            friction: self.model.ground_friction,
            rest: UnitQuaternion::identity(),
        }
    }

    fn max_step_s(&self) -> f32 {
        self.model.max_step_s
    }

    fn begin(&mut self, h: f32) {
        self.alpha = lag::factor(h, self.model.motor_time_constant_s);
    }

    fn loads(&mut self, command: &ActuatorCommand, body: &Body, wind_ned_mps: Vec3) -> Loads {
        self.motors.spin(&command.motors, self.alpha);
        let (thrust_n, torque_nm) = self.motors.wrench();
        self.thrust_n = thrust_n;
        let thrust_ned_n = body.attitude * Vec3::new(0.0, 0.0, -thrust_n);
        let drag_ned_n =
            body.air_relative_velocity_ned_mps(wind_ned_mps) * self.model.drag_n_per_mps;
        Loads {
            force_ned_n: thrust_ned_n - drag_ned_n,
            torque_nm,
            damping_nm: self
                .model
                .angular_damping_nm_per_radps
                .component_mul(&body.angular_velocity_radps),
        }
    }

    fn lift_n(&self, body: &Body) -> f32 {
        -(body.attitude * Vec3::new(0.0, 0.0, -self.thrust_n)).z
    }
}

impl StateSource for SimCopter {
    /// Steps `dt` and returns the new state; `Duration::ZERO` returns the current state
    /// unchanged (see the [timing contract](SimCopter#timing-contract)).
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.lockstep.state(&mut self.dynamics, dt)
    }
}

impl Actuator for SimCopter {
    /// Queues the motor commands for the next [`state`](StateSource::state) step, after any
    /// [`actuator_delay_s`](Perturbations::actuator_delay_s); servos are ignored.
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.lockstep.push(actuator_command);
    }
}
