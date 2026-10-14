//! The simulator as a flight-loop backend: see [`SimVehicle`].

use core::time::Duration;

use stable_flight::mixers::TableMixer;
use stable_flight::{Actuator, ActuatorCommand, StateSource, Vec3, VehicleState, euler_angles};

use crate::body::{Body, Loads, flip_z};
use crate::delay::DelayLine;
use crate::ground;
use crate::gyro::Gyro;
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
/// use stable_flight_sim::{SimVehicle, airframes};
///
/// let airframe = airframes::quad_x();
/// let mut vehicle = SimVehicle::new(airframe.model, &airframe.mixer).expect("valid airframe");
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
pub struct SimVehicle {
    model: MultirotorModel,
    motors: Motors,
    body: Body,
    wind: Wind,
    /// Motor commands from `actuate`, waiting out the actuator delay.
    commands: DelayLine,
    gyro: Gyro,
    disturbance_moment_nm: Vec3,
    /// Simulated time, in s; `f64` so long runs keep sub-millisecond steps exact enough.
    time_s: f64,
    resting: bool,
}

impl SimVehicle {
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
        Ok(SimVehicle {
            motors: Motors::new(mixer.rows(), &model)?,
            model,
            body: Body::default(),
            wind: Wind::default(),
            commands: DelayLine::new(0.0),
            gyro: Gyro::new(Default::default()),
            disturbance_moment_nm: Vec3::zeros(),
            time_s: 0.0,
            resting: true,
        })
    }

    /// This vehicle flying through `wind`.
    pub fn with_wind(mut self, wind: Wind) -> Self {
        self.wind = wind;
        self
    }

    /// This vehicle under `perturbations` (all off by default).
    ///
    /// # Errors
    ///
    /// [`Perturbations::validate`]'s errors.
    pub fn with_perturbations(mut self, perturbations: Perturbations) -> Result<Self, ModelError> {
        perturbations.validate()?;
        self.commands.set_delay(perturbations.actuator_delay_s);
        self.gyro = Gyro::new(perturbations.gyro);
        self.disturbance_moment_nm = perturbations.disturbance_moment_nm;
        self.motors
            .set_thrust_scale(perturbations.motor_thrust_scale);
        Ok(self)
    }

    /// Simulated time since construction, in s.
    pub fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Whether the vehicle rests on the ground (thrust below its weight there).
    pub fn on_ground(&self) -> bool {
        self.resting
    }

    /// Each motor's thrust, in N, by motor slot.
    pub fn motor_thrusts_n(&self) -> [f32; ActuatorCommand::SLOTS] {
        self.motors.thrusts_n(&self.model)
    }

    /// Steps the physics `dt_s` seconds under the stored command.
    fn advance(&mut self, dt_s: f32) {
        let steps = libm::ceilf(dt_s / self.model.max_step_s).max(1.0);
        let h = dt_s / steps;
        let alpha = 1.0 - libm::expf(-h / self.model.motor_time_constant_s);
        let start_velocity = self.body.velocity_ned_mps;
        for _ in 0..steps as u32 {
            self.motors.spin(self.commands.at(self.time_s), alpha);
            let (thrust_n, torque_nm) = self.motors.wrench(&self.model);
            let loads = Loads {
                thrust_n,
                torque_nm: torque_nm + self.disturbance_moment_nm,
                wind_ned_mps: self.wind_ned_mps(),
            };
            self.body.step(&self.model, &loads, h);
            self.resting = ground::contact(&mut self.body, &self.model, thrust_n, h);
            self.time_s += f64::from(h);
        }
        self.body.acceleration_ned_mps2 = (self.body.velocity_ned_mps - start_velocity) / dt_s;
    }

    fn wind_ned_mps(&self) -> Vec3 {
        flip_z(self.wind.velocity_mps(self.time_s as f32))
    }
}

impl StateSource for SimVehicle {
    /// Steps `dt` and returns the new state; `Duration::ZERO` returns the current state
    /// unchanged (see the [timing contract](SimVehicle#timing-contract)).
    fn state(&mut self, dt: Duration) -> VehicleState {
        let dt_s = dt.as_secs_f32();
        if dt_s > 0.0 {
            self.advance(dt_s);
        }
        let mut state = vehicle_state(&self.body, self.wind_ned_mps());
        if let Some(error_radps) = self.gyro.error_radps(self.time_s) {
            state.angular_velocity_radps += error_radps;
        }
        state
    }
}

impl Actuator for SimVehicle {
    /// Queues the motor commands for the next [`state`](StateSource::state) step, after any
    /// [`actuator_delay_s`](Perturbations::actuator_delay_s); servos are ignored.
    fn actuate(&mut self, actuator_command: &ActuatorCommand) {
        self.commands.push(self.time_s, actuator_command.motors);
    }
}

/// `body` in the [frame contract](stable_flight::VehicleState#frame-contract): world vectors
/// from NED to north-east-up, Euler angles from the attitude, and airspeed against
/// `wind_ned_mps`.
pub(crate) fn vehicle_state(body: &Body, wind_ned_mps: Vec3) -> VehicleState {
    let (roll_rad, pitch_rad, heading_rad) = euler_angles(&body.attitude);
    VehicleState {
        position_m: flip_z(body.position_ned_m),
        velocity_mps: flip_z(body.velocity_ned_mps),
        acceleration_mps2: flip_z(body.acceleration_ned_mps2),
        attitude: body.attitude,
        angular_velocity_radps: body.angular_velocity_radps,
        roll_rad,
        pitch_rad,
        heading_rad,
        airspeed_mps: Some((body.velocity_ned_mps - wind_ned_mps).norm()),
    }
}
