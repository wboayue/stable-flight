//! Fixed-wing physics shared by [`SimPlane`](crate::SimPlane) and
//! [`SimTailsitter`](crate::SimTailsitter): see [`FixedWing`].

use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{ActuatorCommand, Vec3};

use crate::aero::{self, Aero};
use crate::body::{Body, Loads, MassProperties};
use crate::fixed_wing::FixedWingModel;
use crate::ground::Ground;
use crate::lag;
use crate::lockstep::Dynamics;
use crate::motors::{MotorLayout, Motors};
use crate::surfaces::Surfaces;
use crate::tailsitter_model::PropWash;

#[cfg(test)]
mod tests;

/// Where a [`FixedWing`]'s motors sit and what their wash reaches.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Rotors {
    /// Distance from the centerline of a motor whose mixer yaw factor is 1, in m: its thrust
    /// along body `x` yaws the body by this arm. 0 for one prop on the centerline.
    pub motor_arm_m: f32,
    /// The props' wash over the surfaces; `None` for surfaces out of it.
    pub wash: Option<PropWash>,
    /// The attitude resting on the ground, relative to level ([`Ground::rest`]).
    pub rest: UnitQuaternion<f32>,
}

impl Rotors {
    /// One prop on the centerline, nothing in its wash, resting level: an airplane.
    pub(crate) fn airplane() -> Self {
        Rotors {
            motor_arm_m: 0.0,
            wash: None,
            rest: UnitQuaternion::identity(),
        }
    }
}

/// A fixed wing's physics: the motors and surfaces through their lags, thrust along body `x`,
/// and the aerodynamics, with the surfaces in the props' wash when [`Rotors::wash`] says so.
#[derive(Debug, Clone)]
pub(crate) struct FixedWing {
    model: FixedWingModel,
    rotors: Rotors,
    motors: Motors,
    surfaces: Surfaces,
    /// The motor and servo lags' step factors for the current step length.
    motor_alpha: f32,
    servo_alpha: f32,
    /// Thrust plus aerodynamic force over the last step, on the body axes, in N.
    force_n: Vec3,
    /// The aerodynamics over the last step.
    aero: Aero,
}

impl FixedWing {
    /// `model`'s physics, its motors and surfaces driven by `mixer`'s rows and placed by
    /// `rotors`. `model` must be valid.
    pub(crate) fn new<const N: usize>(
        model: FixedWingModel,
        mixer: &TableMixer<N>,
        rotors: Rotors,
    ) -> Self {
        // Each motor's thrust is along body `x`, so a roll or pitch factor turns nothing, and
        // prop drag torque isn't modeled: a yaw factor acts through the motor's arm.
        let layout = MotorLayout {
            moment_arm_m: 0.0,
            yaw_torque_per_thrust_m: rotors.motor_arm_m,
            max_thrust_n: model.max_thrust_n,
        };
        FixedWing {
            motors: Motors::new(mixer.rows(), &layout),
            surfaces: Surfaces::new(mixer.rows()),
            model,
            rotors,
            motor_alpha: 0.0,
            servo_alpha: 0.0,
            force_n: Vec3::zeros(),
            aero: Aero::default(),
        }
    }

    /// Scales each motor's thrust (mismatched motors).
    pub(crate) fn set_thrust_scale(&mut self, thrust_scale: [f32; ActuatorCommand::SLOTS]) {
        self.motors.set_thrust_scale(thrust_scale);
    }

    /// The aerodynamics over the last step.
    pub(crate) fn aero(&self) -> &Aero {
        &self.aero
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
            rest: self.rotors.rest,
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
        let (thrust_n, motor_torque_nm) = self.motors.wrench();
        let wash_pa = self
            .rotors
            .wash
            .map_or(0.0, |wash| wash.pressure_pa(thrust_n));
        let air_mps = body
            .attitude
            .inverse_transform_vector(&body.air_relative_velocity_ned_mps(wind_ned_mps));
        self.aero = aero::wrench(
            &self.model,
            air_mps,
            body.angular_velocity_radps,
            self.surfaces.deflection_rad(&self.model.max_deflection_rad),
            wash_pa,
        );
        self.force_n = self.aero.force_n + Vec3::new(thrust_n, 0.0, 0.0);
        Loads {
            force_ned_n: body.attitude * self.force_n,
            torque_nm: self.aero.torque_nm + motor_torque_nm,
            damping_nm: Vec3::zeros(),
        }
    }

    fn lift_n(&self, body: &Body) -> f32 {
        -(body.attitude * self.force_n).z
    }
}
