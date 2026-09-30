//! Tuned setups for the RealFlight models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns the
//! stabilizer configuration and channel mapping for one model, so examples (and any other
//! wiring) share one tuned set.

use std::f32::consts::PI;

use robomath::{vec2, vec3};
use stable_flight::stabilizers::Gains;
use stable_flight::stabilizers::controllers::{
    AttitudeController, AttitudeControllerConfig, BodyRateController, BodyRateControllerConfig,
};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::ActuatorMapping;

#[cfg(test)]
mod tests;

/// Stabilizer configuration and channel mapping for the QuadCopterX model.
pub struct QuadCopterXAirframe {
    /// Hand-tuned stabilizer gains (rate and attitude loops; the rest are the core defaults).
    pub stabilizer: CopterStabilizerConfig,
    /// Motor-to-channel mapping of the RealFlight model.
    pub mapping: ActuatorMapping,
}

/// The ArduPilot SITL_Models
/// [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX)
/// in RealFlight.
///
/// The gains were hand-tuned flying `copter-manual-realflight` (attitude and rate modes).
/// `copter-auto-realflight` still flies `CopterStabilizerConfig::default()` until one set is
/// flown in both modes.
///
/// # Examples
///
/// ```
/// use stable_flight::stabilizers::copter::CopterStabilizer;
/// use stable_flight_sitl_realflight::airframes;
///
/// let airframe = airframes::quadcopter_x();
/// let stabilizer = CopterStabilizer::new(airframe.stabilizer);
/// assert_eq!(airframe.mapping.motors[0], Some(2));
/// ```
pub fn quadcopter_x() -> QuadCopterXAirframe {
    let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
        gains: Gains {
            p: 7.0,
            i: 0.00,
            d: 0.01,
        },
        max_integral: 100.0,
        max_angular_rates: vec3(3.0, 3.0, 3.0), // Max roll, pitch, yaw rates in rad/s
    });

    let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: Gains {
            p: 3.0,
            i: 0.0,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: vec2(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: vec2(3.0, 3.0),             // Max roll, pitch rates in rad/s
    });

    QuadCopterXAirframe {
        stabilizer: CopterStabilizerConfig {
            rates_controller,
            attitude_controller,
            ..Default::default()
        },
        // The model's motors 1–4 are on channels 2, 0, 1, 3, in `QuadCopterMixer` order.
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    }
}
