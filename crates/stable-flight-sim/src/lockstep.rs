//! What every simulated vehicle shares: lockstep timing, the actuator delay, wind, gyro noise,
//! and the rigid body on the ground plane. See [`Lockstep`].

use core::time::Duration;

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{ActuatorCommand, Vec3, VehicleState, euler_angles};

use crate::body::{Body, Loads, MassProperties, flip_z};
use crate::delay::DelayLine;
use crate::ground::{self, Ground};
use crate::gyro::Gyro;
use crate::perturbations::Perturbations;
use crate::wind::Wind;

#[cfg(test)]
mod tests;

/// Where and how a vehicle starts, in the [frame
/// contract](stable_flight::VehicleState#frame-contract)'s axes.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::Start;
///
/// // 100 m up, flying east at 15 m/s.
/// let start = Start::level(100.0, 15.0, core::f32::consts::FRAC_PI_2);
/// assert!((start.velocity_mps.y - 15.0).abs() < 1e-5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Start {
    /// Position (north, east, up), in m.
    pub position_m: Vec3,
    /// Velocity (north, east, up), in m/s.
    pub velocity_mps: Vec3,
    /// Attitude, as [`VehicleState::attitude`].
    pub attitude: UnitQuaternion<f32>,
    /// Body rates (FRD roll, pitch, yaw), in rad/s.
    pub angular_velocity_radps: Vec3,
}

impl Start {
    /// Wings level over the origin at `altitude_m`, nose and velocity along `heading_rad`
    /// (clockwise from north) at `speed_mps`, not turning.
    pub fn level(altitude_m: f32, speed_mps: f32, heading_rad: f32) -> Self {
        let (sin, cos) = (libm::sinf(heading_rad), libm::cosf(heading_rad));
        Start {
            position_m: Vec3::new(0.0, 0.0, altitude_m),
            velocity_mps: Vec3::new(cos, sin, 0.0) * speed_mps,
            attitude: UnitQuaternion::from_euler_angles(0.0, 0.0, heading_rad),
            angular_velocity_radps: Vec3::zeros(),
        }
    }
}

/// A vehicle's physics, as [`Lockstep`] integrates it: what the actuators and the air do to
/// the body.
pub(crate) trait Dynamics {
    /// The body's mass and inertia.
    fn mass(&self) -> MassProperties;

    /// The body's mass and friction on the ground.
    fn ground(&self) -> Ground;

    /// Longest integration step, in s.
    fn max_step_s(&self) -> f32;

    /// Prepares for integration steps of `h` seconds (actuator lag factors), once per
    /// [`Lockstep::state`] call rather than once per step.
    fn begin(&mut self, h: f32);

    /// Runs the actuators one step on `command` and returns the loads on `body` in air moving
    /// at `wind_ned_mps`.
    fn loads(&mut self, command: &ActuatorCommand, body: &Body, wind_ned_mps: Vec3) -> Loads;

    /// The upward force carrying the weight, in N, at `body`'s attitude after the step: what
    /// lifts it off the ground.
    fn lift_n(&self, body: &Body) -> f32;
}

/// Lockstep timing around a [`Dynamics`]: commands wait out the actuator delay, each
/// [`state`](Self::state) call integrates `dt` in equal steps, and the result is reported in
/// the frame contract with the gyro's error on the body rates.
#[derive(Debug, Clone)]
pub(crate) struct Lockstep {
    pub(crate) body: Body,
    wind: Wind,
    /// Commands from `actuate`, waiting out the actuator delay.
    commands: DelayLine,
    gyro: Gyro,
    disturbance_moment_nm: Vec3,
    /// Simulated time, in s; `f64` so long runs keep sub-millisecond steps exact enough.
    time_s: f64,
    resting: bool,
}

impl Lockstep {
    /// At rest on the ground at the origin, level, nose north, in calm air, unperturbed.
    pub(crate) fn new() -> Self {
        Lockstep {
            body: Body::default(),
            wind: Wind::default(),
            commands: DelayLine::new(0.0),
            gyro: Gyro::new(Default::default()),
            disturbance_moment_nm: Vec3::zeros(),
            time_s: 0.0,
            resting: true,
        }
    }

    /// Moves the body to `start`, off the ground unless `start` is on it.
    pub(crate) fn place(&mut self, start: &Start) {
        self.body = Body {
            position_ned_m: flip_z(start.position_m),
            velocity_ned_mps: flip_z(start.velocity_mps),
            acceleration_ned_mps2: Vec3::zeros(),
            attitude: start.attitude,
            angular_velocity_radps: start.angular_velocity_radps,
        };
        self.resting = start.position_m.z <= 0.0;
    }

    pub(crate) fn set_wind(&mut self, wind: Wind) {
        self.wind = wind;
    }

    /// Applies `perturbations`' delay, gyro noise, and disturbance moment; motor mismatch is
    /// the vehicle's. `perturbations` must be valid.
    pub(crate) fn set_perturbations(&mut self, perturbations: &Perturbations) {
        self.commands.set_delay(perturbations.actuator_delay_s);
        self.gyro = Gyro::new(perturbations.gyro);
        self.disturbance_moment_nm = perturbations.disturbance_moment_nm;
    }

    /// Simulated time since construction, in s.
    pub(crate) fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Whether the body rests on the ground.
    pub(crate) fn resting(&self) -> bool {
        self.resting
    }

    /// Queues `command` for the next steps, after the actuator delay.
    pub(crate) fn push(&mut self, command: &ActuatorCommand) {
        self.commands.push(self.time_s, *command);
    }

    /// Steps `dynamics` `dt` (none for `Duration::ZERO`) and returns the state, as the
    /// [`StateSource`](stable_flight::StateSource) contract asks.
    pub(crate) fn state(&mut self, dynamics: &mut impl Dynamics, dt: Duration) -> VehicleState {
        let dt_s = dt.as_secs_f32();
        if dt_s > 0.0 {
            self.advance(dynamics, dt_s);
        }
        let mut state = vehicle_state(&self.body, self.wind_ned_mps());
        if let Some(error_radps) = self.gyro.error_radps(self.time_s) {
            state.angular_velocity_radps += error_radps;
        }
        state
    }

    /// Steps the physics `dt_s` seconds under the delayed commands.
    fn advance(&mut self, dynamics: &mut impl Dynamics, dt_s: f32) {
        let steps = libm::ceilf(dt_s / dynamics.max_step_s()).max(1.0);
        let h = dt_s / steps;
        dynamics.begin(h);
        let (mass, ground) = (dynamics.mass(), dynamics.ground());
        let start_velocity = self.body.velocity_ned_mps;
        for _ in 0..steps as u32 {
            let wind_ned_mps = self.wind_ned_mps();
            let command = self.commands.due_at(self.time_s);
            let mut loads = dynamics.loads(command, &self.body, wind_ned_mps);
            loads.torque_nm += self.disturbance_moment_nm;
            self.body.step(&mass, &loads, h);
            let lift_n = dynamics.lift_n(&self.body);
            self.resting = ground::contact(&mut self.body, &ground, lift_n, h);
            self.time_s += f64::from(h);
        }
        self.body.acceleration_ned_mps2 = (self.body.velocity_ned_mps - start_velocity) / dt_s;
    }

    fn wind_ned_mps(&self) -> Vec3 {
        flip_z(self.wind.velocity_mps(self.time_s as f32))
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
        airspeed_mps: Some(body.air_relative_velocity_ned_mps(wind_ned_mps).norm()),
    }
}
