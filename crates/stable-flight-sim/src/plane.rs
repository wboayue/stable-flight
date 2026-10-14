//! The simulated airplane as a flight-loop backend: see [`SimPlane`].

use core::time::Duration;

use stable_flight::mixers::TableMixer;
use stable_flight::{Actuator, ActuatorCommand, StateSource, Vec3, VehicleState};

use crate::aero::{self, Aero};
use crate::body::{Body, Loads, MassProperties};
use crate::fixed_wing::FixedWingModel;
use crate::ground::Ground;
use crate::lag;
use crate::lockstep::{Dynamics, Lockstep, Start};
use crate::model::ModelError;
use crate::motors::{MotorLayout, Motors};
use crate::perturbations::Perturbations;
use crate::surfaces::Surfaces;
use crate::wind::Wind;

#[cfg(test)]
mod tests;

/// A simulated airplane that a [`FlightLoop`](stable_flight::FlightLoop) flies, with the
/// [timing contract](crate::SimCopter#timing-contract) of [`SimCopter`](crate::SimCopter).
///
/// The physics: a rigid body with [`FixedWingModel`]'s mass and diagonal inertia, under
/// gravity, the wing's lift, drag, side force, and moments through the [`Wind`]
/// ([`AeroCoefficients`](crate::AeroCoefficients)), and a tractor prop's thrust along body
/// `x`. The mixer's servo rows say which way each servo moves the aileron, elevator, and rudder
/// axes; its motor rows drive the prop (motor mismatch scales them). It starts where
/// [`with_start`](Self::with_start) puts it, by default at rest on the ground at the origin; it
/// doesn't take off from there (no ground roll).
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use stable_flight_sim::{SimPlane, airframes};
///
/// let airframe = airframes::plane();
/// let mut plane = SimPlane::new(airframe.model, &airframe.mixer)
///     .expect("valid airframe")
///     .with_start(airframe.start);
/// // Gliding north with the surfaces centered for ten seconds.
/// plane.actuate(&ActuatorCommand::default());
/// for _ in 0..2500 {
///     plane.state(Duration::from_millis(4));
/// }
/// let state = plane.state(Duration::ZERO);
/// assert!(state.position_m.z < airframe.start.position_m.z);
/// assert!(state.position_m.x > 50.0);
/// ```
#[derive(Debug, Clone)]
pub struct SimPlane {
    lockstep: Lockstep,
    dynamics: FixedWing,
}

/// The airplane's physics: the prop and surfaces through their lags, and the aerodynamics.
#[derive(Debug, Clone)]
struct FixedWing {
    model: FixedWingModel,
    motors: Motors,
    surfaces: Surfaces,
    /// The motor and servo lags' step factors for the current step length.
    motor_alpha: f32,
    servo_alpha: f32,
    /// Prop thrust plus aerodynamic force over the last step, on the body axes, in N.
    force_n: Vec3,
    /// The aerodynamics over the last step.
    aero: Aero,
}

impl SimPlane {
    /// An airplane with `model`'s physics whose prop and surfaces `mixer`'s rows drive (see
    /// [`SimPlane`]), at rest on the ground in calm air.
    ///
    /// # Errors
    ///
    /// [`FixedWingModel::validate`]'s errors.
    pub fn new<const N: usize>(
        model: FixedWingModel,
        mixer: &TableMixer<N>,
    ) -> Result<Self, ModelError> {
        model.validate()?;
        // The prop sits on the roll axis and its drag torque isn't modeled: thrust only.
        let prop = MotorLayout {
            moment_arm_m: 0.0,
            yaw_torque_per_thrust_m: 0.0,
            max_thrust_n: model.max_thrust_n,
        };
        Ok(SimPlane {
            lockstep: Lockstep::new(),
            dynamics: FixedWing {
                motors: Motors::new(mixer.rows(), &prop),
                surfaces: Surfaces::new(mixer.rows()),
                model,
                motor_alpha: 0.0,
                servo_alpha: 0.0,
                force_n: Vec3::zeros(),
                aero: Aero::default(),
            },
        })
    }

    /// This airplane starting at `start`, e.g. in flight ([`Start::level`]).
    pub fn with_start(mut self, start: Start) -> Self {
        self.lockstep.place(&start);
        self
    }

    /// This airplane flying through `wind`.
    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.lockstep.set_wind(wind);
        self
    }

    /// This airplane under `perturbations` (all off by default); motor mismatch scales the
    /// prop.
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

    /// Whether the airplane rests on the ground (lift and thrust below its weight there).
    pub fn on_ground(&self) -> bool {
        self.lockstep.resting()
    }

    /// Angle of attack over the last integration step, in rad.
    pub fn angle_of_attack_rad(&self) -> f32 {
        self.dynamics.aero.alpha_rad
    }

    /// Sideslip over the last integration step, in rad: positive with the air coming from the
    /// right.
    pub fn sideslip_rad(&self) -> f32 {
        self.dynamics.aero.beta_rad
    }
}

impl Dynamics for FixedWing {
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
        }
    }

    fn max_step_s(&self) -> f32 {
        self.model.max_step_s
    }

    fn begin(&mut self, h: f32) {
        self.motor_alpha = lag::factor(h, self.model.motor_time_constant_s);
        self.servo_alpha = lag::factor(h, self.model.servo_time_constant_s);
    }

    fn loads(&mut self, command: &ActuatorCommand, body: &Body, wind_ned_mps: Vec3) -> Loads {
        self.motors.spin(&command.motors, self.motor_alpha);
        self.surfaces.move_to(&command.servos, self.servo_alpha);
        let air_mps = body
            .attitude
            .inverse_transform_vector(&body.air_relative_velocity_ned_mps(wind_ned_mps));
        self.aero = aero::wrench(
            &self.model,
            air_mps,
            body.angular_velocity_radps,
            self.surfaces.deflection_rad(&self.model.max_deflection_rad),
        );
        let (thrust_n, _) = self.motors.wrench();
        self.force_n = self.aero.force_n + Vec3::new(thrust_n, 0.0, 0.0);
        Loads {
            force_ned_n: body.attitude * self.force_n,
            torque_nm: self.aero.torque_nm,
            damping_nm: Vec3::zeros(),
        }
    }

    fn lift_n(&self, body: &Body) -> f32 {
        -(body.attitude * self.force_n).z
    }
}

impl StateSource for SimPlane {
    /// Steps `dt` and returns the new state; `Duration::ZERO` returns the current state
    /// unchanged (see the [timing contract](crate::SimCopter#timing-contract)).
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.lockstep.state(&mut self.dynamics, dt)
    }
}

impl Actuator for SimPlane {
    /// Queues the motor and servo commands for the next [`state`](StateSource::state) step,
    /// after any [`actuator_delay_s`](Perturbations::actuator_delay_s).
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.lockstep.push(actuator_command);
    }
}
