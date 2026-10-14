//! Departures from the clean plant that hardware brings: see [`Perturbations`].

use stable_flight::{ActuatorCommand, Vec3};

use crate::model::ModelError;

#[cfg(test)]
mod tests;

/// Longest [`Perturbations::actuator_delay_s`], in s.
pub const MAX_ACTUATOR_DELAY_S: f32 = 0.05;

/// Delay, gyro noise, a disturbance moment, and motor mismatch on a
/// [`SimVehicle`](crate::SimVehicle), each a value to sweep rather than a fit to one plant.
///
/// The default turns every perturbation off, which leaves the clean plant exactly as it was.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec3;
/// use stable_flight_sim::{GyroNoise, Perturbations, SimVehicle, Vibration, airframes};
///
/// let mut motor_thrust_scale = [1.0; 8];
/// motor_thrust_scale[0] = 0.95;
/// let perturbations = Perturbations {
///     actuator_delay_s: 0.01,
///     gyro: GyroNoise {
///         rms_radps: Vec3::repeat(0.02),
///         vibration: Some(Vibration { amplitude_radps: Vec3::repeat(0.05), frequency_hz: 90.0 }),
///         seed: 7,
///     },
///     disturbance_moment_nm: Vec3::new(0.01, 0.0, 0.0),
///     motor_thrust_scale,
/// };
/// let airframe = airframes::quad_x();
/// let vehicle = SimVehicle::new(airframe.model, &airframe.mixer)
///     .expect("valid airframe")
///     .with_perturbations(perturbations)
///     .expect("valid perturbations");
/// # drop(vehicle);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Perturbations {
    /// Delay from [`actuate`](stable_flight::Actuator::actuate) to the motors, in s, on top of
    /// the one-tick hold in the [timing contract](crate::SimVehicle#timing-contract) (ESC
    /// update, gyro filtering, estimator). At most [`MAX_ACTUATOR_DELAY_S`].
    pub actuator_delay_s: f32,
    /// Noise and vibration on the reported body rates.
    pub gyro: GyroNoise,
    /// A constant moment on the FRD body axes (roll, pitch, yaw), in N·m, such as a CG offset.
    pub disturbance_moment_nm: Vec3,
    /// Each motor slot's thrust as a fraction of the model's (mismatched motors or props).
    /// Positive; yaw drag torque scales with it, since it goes with thrust.
    pub motor_thrust_scale: [f32; ActuatorCommand::SLOTS],
}

impl Default for Perturbations {
    /// Every perturbation off.
    fn default() -> Self {
        Perturbations {
            actuator_delay_s: 0.0,
            gyro: GyroNoise::default(),
            disturbance_moment_nm: Vec3::zeros(),
            motor_thrust_scale: [1.0; ActuatorCommand::SLOTS],
        }
    }
}

/// Error on the reported [`angular_velocity_radps`](stable_flight::VehicleState::angular_velocity_radps):
/// Gaussian white noise plus an optional [`Vibration`] tone. It never reaches the physics, and
/// attitude stays exact.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GyroNoise {
    /// RMS (standard deviation) of the white noise per FRD axis, in rad/s.
    pub rms_radps: Vec3,
    /// A sinusoidal vibration added on top.
    pub vibration: Option<Vibration>,
    /// Seed of the noise generator: one seed gives the same noise, bit for bit.
    pub seed: u64,
}

/// A vibration tone on the gyro: `amplitude · sin(2π · frequency · t)`, in simulated time.
///
/// It is sampled at each [`state`](stable_flight::StateSource::state) call, so a tone above half
/// the loop rate aliases to a lower frequency, as on an unfiltered gyro read at the loop rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vibration {
    /// Peak per FRD axis, in rad/s.
    pub amplitude_radps: Vec3,
    /// Tone frequency, in Hz.
    pub frequency_hz: f32,
}

impl Perturbations {
    /// Checks every field's range.
    ///
    /// # Errors
    ///
    /// [`ModelError::DelayTooLong`] for a delay over [`MAX_ACTUATOR_DELAY_S`];
    /// [`ModelError::Negative`] for a negative or non-finite delay, noise RMS, or vibration
    /// amplitude; [`ModelError::NotPositive`] for a vibration frequency that isn't positive and
    /// finite; [`ModelError::NotFinite`] for a non-finite disturbance moment; and
    /// [`ModelError::ThrustScale`] for a motor scale that isn't positive and finite.
    pub fn validate(&self) -> Result<(), ModelError> {
        let vibration = self.gyro.vibration;
        let amplitude_radps = vibration.map_or(Vec3::zeros(), |v| v.amplitude_radps);
        let non_negative = [
            ("actuator_delay_s", self.actuator_delay_s),
            ("gyro.rms_radps.x", self.gyro.rms_radps.x),
            ("gyro.rms_radps.y", self.gyro.rms_radps.y),
            ("gyro.rms_radps.z", self.gyro.rms_radps.z),
            ("gyro.vibration.amplitude_radps.x", amplitude_radps.x),
            ("gyro.vibration.amplitude_radps.y", amplitude_radps.y),
            ("gyro.vibration.amplitude_radps.z", amplitude_radps.z),
        ];
        if let Some((name, _)) = non_negative
            .iter()
            .find(|(_, value)| !(value.is_finite() && *value >= 0.0))
        {
            return Err(ModelError::Negative(name));
        }
        if self.actuator_delay_s > MAX_ACTUATOR_DELAY_S {
            return Err(ModelError::DelayTooLong);
        }
        if let Some(vibration) = vibration
            && !(vibration.frequency_hz.is_finite() && vibration.frequency_hz > 0.0)
        {
            return Err(ModelError::NotPositive("gyro.vibration.frequency_hz"));
        }
        if !self.disturbance_moment_nm.iter().all(|m| m.is_finite()) {
            return Err(ModelError::NotFinite("disturbance_moment_nm"));
        }
        if let Some(slot) = self
            .motor_thrust_scale
            .iter()
            .position(|scale| !(scale.is_finite() && *scale > 0.0))
        {
            // `slot` is below `ActuatorCommand::SLOTS` (8).
            return Err(ModelError::ThrustScale(slot as u8));
        }
        Ok(())
    }
}
