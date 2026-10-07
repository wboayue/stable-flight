//! Tuned setups for the RealFlight models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns an
//! [`Airframe`] with the guidance and stabilizer configurations, the mixer, and the channel
//! mapping for one model, so examples (and any other wiring) share one tuned set.

use std::f32::consts::PI;

use stable_flight::Vec2;
use stable_flight::controllers::Gains;
use stable_flight::controllers::{
    AttitudeController, AttitudeControllerConfig, BodyRateController, BodyRateControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::ActuatorMapping;

#[cfg(test)]
mod tests;

/// Everything model-specific about one RealFlight airframe.
///
/// `G` is the guidance configuration, `S` the stabilizer configuration, and `M` the mixer.
/// The mixer is part of the airframe because `mapping` is only valid for its motor order.
#[derive(Debug, Clone, PartialEq)]
pub struct Airframe<G, S, M> {
    /// Guidance (position-loop) configuration.
    pub guidance: G,
    /// Tuned stabilizer configuration.
    pub stabilizer: S,
    /// The mixer whose motor order `mapping` follows.
    pub mixer: M,
    /// RealFlight channel for each of the mixer's outputs.
    pub mapping: ActuatorMapping,
}

/// The ArduPilot SITL_Models
/// [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX)
/// in RealFlight.
///
/// The stabilizer gains were hand-tuned flying `copter-manual-realflight` (attitude and rate
/// modes); guidance uses the defaults. `copter-auto-realflight` still flies
/// `CopterStabilizerConfig::default()` until one set is flown in both modes.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::CopterGuidance;
/// use stable_flight::stabilizers::copter::CopterStabilizer;
/// use stable_flight_sitl_realflight::airframes;
///
/// let airframe = airframes::quadcopter_x();
/// let guidance = CopterGuidance::new(airframe.guidance);
/// let stabilizer = CopterStabilizer::new(airframe.stabilizer);
/// let mixer = airframe.mixer;
/// assert_eq!(airframe.mapping.motors[0], Some(2));
/// ```
pub fn quadcopter_x() -> Airframe<CopterGuidanceConfig, CopterStabilizerConfig, QuadCopterMixer> {
    let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
        gains: Gains {
            p: 0.15,
            i: 0.0,
            d: 0.003,
        },
        max_integral: 1.0,
    });

    let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: Gains {
            p: 3.0,
            i: 0.0,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: Vec2::new(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: Vec2::new(3.0, 3.0),             // Max roll, pitch rates in rad/s
    });

    Airframe {
        guidance: CopterGuidanceConfig::default(),
        stabilizer: CopterStabilizerConfig {
            rates_controller,
            attitude_controller,
            ..Default::default()
        },
        mixer: QuadCopterMixer::default(),
        // `QuadCopterMixer` motor `i` drives RealFlight channel `motors[i]`.
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    }
}
