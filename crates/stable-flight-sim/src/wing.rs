//! Fixed-wing physics shared by [`SimPlane`](crate::SimPlane),
//! [`SimTailsitter`](crate::SimTailsitter), and [`SimQuadPlane`](crate::SimQuadPlane): see
//! [`FixedWing`].

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
use crate::quadplane_model::{LiftRotors, QuadPlaneModel};
use crate::surfaces::Surfaces;
use crate::tailsitter_model::{PropWash, TailsitterModel};

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
    /// Lift rotors along body `−z` and the one motor slot thrusting along body `x`; `None`
    /// for every motor along body `x`.
    pub lift: Option<Lift>,
}

/// A quadplane's lift rotors, and the slot of its pusher, the one motor along body `x`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Lift {
    pub rotors: LiftRotors,
    pub pusher_slot: u8,
}

impl Rotors {
    /// One prop on the centerline, nothing in its wash: an airplane.
    pub(crate) fn airplane() -> Self {
        Rotors {
            motor_arm_m: 0.0,
            wash: None,
            lift: None,
        }
    }

    /// `model`'s motors along the wing, the surfaces in their wash.
    pub(crate) fn tailsitter(model: &TailsitterModel) -> Self {
        Rotors {
            motor_arm_m: model.motor_arm_m,
            wash: Some(model.prop_wash),
            lift: None,
        }
    }

    /// `model`'s lift rotors and its pusher on the centerline, nothing in their wash.
    pub(crate) fn quadplane(model: &QuadPlaneModel) -> Self {
        Rotors {
            lift: Some(Lift {
                rotors: model.lift,
                pusher_slot: model.pusher_slot,
            }),
            ..Self::airplane()
        }
    }
}

/// Lift rotors through their lag: thrust along body `−z`, their torque, and their in-plane
/// drag ([`LiftRotors`]).
#[derive(Debug, Clone)]
struct LiftGroup {
    rotors: LiftRotors,
    motors: Motors,
    /// In-plane drag per m/s of air per newton of lift thrust, in 1/s: the rotors'
    /// `drag_n_per_mps` over their total thrust at full command; 0 without rotors.
    drag_per_thrust_n_per_mps: f32,
    /// The rotor lag's step factor for the current step length.
    alpha: f32,
}

impl LiftGroup {
    /// `mixer`'s motor rows but the pusher's, laid out by `lift`'s rotors.
    fn new<const N: usize>(mixer: &TableMixer<N>, lift: &Lift) -> Self {
        let motors = Motors::excluding(mixer.rows(), &lift.rotors.motor_layout(), lift.pusher_slot);
        let max_total_thrust_n = motors.max_total_thrust_n();
        // A mixer without lift rotors has no thrust to scale the drag by.
        let drag_per_thrust_n_per_mps = if max_total_thrust_n > 0.0 {
            lift.rotors.drag_n_per_mps / max_total_thrust_n
        } else {
            0.0
        };
        LiftGroup {
            rotors: lift.rotors,
            motors,
            drag_per_thrust_n_per_mps,
            alpha: 0.0,
        }
    }

    /// Runs the rotors one step on `commands` and returns their force and torque on the body
    /// axes, moving through the air at `air_mps` (body axes), in N and N·m.
    fn wrench(&mut self, commands: &[f32; ActuatorCommand::SLOTS], air_mps: Vec3) -> (Vec3, Vec3) {
        self.motors.spin(commands, self.alpha);
        let (thrust_n, torque_nm) = self.motors.wrench();
        let in_plane_mps = Vec3::new(air_mps.x, air_mps.y, 0.0);
        let drag_n = in_plane_mps * (self.drag_per_thrust_n_per_mps * thrust_n);
        (Vec3::new(0.0, 0.0, -thrust_n) - drag_n, torque_nm)
    }
}

/// A fixed wing's physics: the motors and surfaces through their lags, thrust along body `x`,
/// and the aerodynamics, with the surfaces in the props' wash when [`Rotors::wash`] says so
/// and lift rotors along body `−z` when [`Rotors::lift`] has them.
#[derive(Debug, Clone)]
pub(crate) struct FixedWing {
    model: FixedWingModel,
    rotors: Rotors,
    /// The attitude resting on the ground, relative to level ([`Ground::rest`]).
    rest: UnitQuaternion<f32>,
    /// The motors along body `x`.
    motors: Motors,
    lift: Option<LiftGroup>,
    surfaces: Surfaces,
    /// The motor and servo lags' step factors for the current step length.
    motor_alpha: f32,
    servo_alpha: f32,
    /// Thrust, lift rotors', and aerodynamic force over the last step, on the body axes, in N.
    force_n: Vec3,
    /// The aerodynamics over the last step.
    aero: Aero,
}

impl FixedWing {
    /// `model`'s physics, its motors and surfaces driven by `mixer`'s rows and placed by
    /// `rotors`, resting on the ground at `rest`. `model` must be valid.
    pub(crate) fn new<const N: usize>(
        model: FixedWingModel,
        mixer: &TableMixer<N>,
        rotors: Rotors,
        rest: UnitQuaternion<f32>,
    ) -> Self {
        let (motors, lift) = match rotors.lift {
            Some(lift) => (
                Motors::single(lift.pusher_slot, model.max_thrust_n),
                Some(LiftGroup::new(mixer, &lift)),
            ),
            None => {
                // Each motor's thrust is along body `x`, so a roll or pitch factor turns
                // nothing, and prop drag torque isn't modeled: a yaw factor acts through the
                // motor's arm.
                let layout = MotorLayout {
                    moment_arm_m: 0.0,
                    yaw_torque_per_thrust_m: rotors.motor_arm_m,
                    max_thrust_n: model.max_thrust_n,
                };
                (Motors::new(mixer.rows(), &layout), None)
            }
        };
        FixedWing {
            motors,
            lift,
            surfaces: Surfaces::new(mixer.rows()),
            model,
            rotors,
            rest,
            motor_alpha: 0.0,
            servo_alpha: 0.0,
            force_n: Vec3::zeros(),
            aero: Aero::default(),
        }
    }

    /// Scales each motor's thrust (mismatched motors).
    pub(crate) fn set_thrust_scale(&mut self, thrust_scale: [f32; ActuatorCommand::SLOTS]) {
        self.motors.set_thrust_scale(thrust_scale);
        if let Some(lift) = &mut self.lift {
            lift.motors.set_thrust_scale(thrust_scale);
        }
    }

    /// Each lift rotor's thrust, in N, by motor slot; all zero without lift rotors.
    pub(crate) fn lift_thrusts_n(&self) -> [f32; ActuatorCommand::SLOTS] {
        self.lift
            .as_ref()
            .map_or([0.0; ActuatorCommand::SLOTS], |lift| {
                lift.motors.thrusts_n()
            })
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
            rest: self.rest,
        }
    }

    fn max_step_s(&self) -> f32 {
        self.model.max_step_s
    }

    fn begin(&mut self, h: f32) {
        self.motor_alpha = lag::factor(h, self.model.motor_time_constant_s);
        self.servo_alpha = lag::factor(h, self.model.servo_time_constant_s);
        if let Some(lift) = &mut self.lift {
            lift.alpha = lag::factor(h, lift.rotors.motor_time_constant_s);
        }
    }

    fn loads(&mut self, command: &ActuatorCommand, body: &Body, wind_ned_mps: Vec3) -> Loads {
        self.motors.spin(&command.motors, self.motor_alpha);
        self.surfaces.move_to(&command.servos, self.servo_alpha);
        let (thrust_n, motor_torque_nm) = self.motors.wrench();
        let air_mps = body
            .attitude
            .inverse_transform_vector(&body.air_relative_velocity_ned_mps(wind_ned_mps));
        // The thrust axis is body `x`.
        let wash_pa = self
            .rotors
            .wash
            .map_or(0.0, |wash| wash.pressure_pa(thrust_n, air_mps.x));
        self.aero = aero::wrench(
            &self.model,
            air_mps,
            body.angular_velocity_radps,
            self.surfaces.deflection_rad(&self.model.max_deflection_rad),
            wash_pa,
        );
        let (lift_force_n, lift_torque_nm) = self
            .lift
            .as_mut()
            .map_or((Vec3::zeros(), Vec3::zeros()), |lift| {
                lift.wrench(&command.motors, air_mps)
            });
        self.force_n = self.aero.force_n + Vec3::new(thrust_n, 0.0, 0.0) + lift_force_n;
        Loads {
            force_ned_n: body.attitude * self.force_n,
            torque_nm: self.aero.torque_nm + motor_torque_nm + lift_torque_nm,
            damping_nm: Vec3::zeros(),
        }
    }

    fn lift_n(&self, body: &Body) -> f32 {
        -(body.attitude * self.force_n).z
    }
}
