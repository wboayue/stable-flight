//! Tuned setups for the RealFlight models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns an
//! [`Airframe`] with the guidance and stabilizer configurations, the mixer, the channel
//! mapping, and the body rotation for one model, so examples (and any other wiring) share one tuned set.

use std::f32::consts::PI;

use stable_flight::Vec2;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
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
    /// The vehicle's body frame expressed in RealFlight's
    /// ([`RealFlightConfiguration::body_rotation`](crate::RealFlightConfiguration::body_rotation)).
    pub body_rotation: UnitQuaternion<f32>,
}

/// The ArduPilot SITL_Models
/// [QuadCopterX](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX)
/// in RealFlight.
///
/// The stabilizer gains were hand-tuned flying `copter-manual-realflight` in attitude mode after
/// the frame fix (#62); guidance uses the core defaults (lateral and altitude gains and the tilt
/// cap were flight-checked on this model) with this model's measured hover thrust. Both
/// RealFlight examples fly this set.
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
pub fn quadcopter_x() -> Airframe<CopterGuidanceConfig, CopterStabilizerConfig, TableMixer<4>> {
    // Roll and pitch share one hand tune; autotune (#149) can split them.
    let rate_roll_pitch = Gains {
        p: 0.5,
        i: 0.2,
        d: 0.004,
    };
    let attitude_roll_pitch = Gains {
        p: 3.0,
        i: 0.0,
        d: 0.1,
    };
    Airframe {
        guidance: CopterGuidanceConfig {
            altitude: AltitudeControllerConfig {
                // Measured: a settled hover holds at mean motor 0.606 = 0.9 · 0.82², through
                // the squared thrust curve and the mixer's thrust factor.
                hover_thrust: 0.82,
                ..CopterGuidanceConfig::default().altitude
            },
            ..CopterGuidanceConfig::default()
        },
        stabilizer: CopterStabilizerConfig {
            body_rate: BodyRateControllerConfig {
                roll_gains: rate_roll_pitch,
                pitch_gains: rate_roll_pitch,
                // Yaw authority is differential prop drag only, so yaw needs its own, higher gains.
                // The model needs a steady ~0.40 nose-right yaw moment to hover; I · max_integral
                // = 0.6 cancels it with margin. At I 0.2 the integral pinned at 0.2 and acro
                // drifted ~19°/s CCW (#192).
                yaw_gains: Gains {
                    p: 0.6,
                    i: 0.6,
                    d: 0.0,
                },
                max_integral: 1.0,
            },
            attitude: AttitudeControllerConfig {
                roll_gains: attitude_roll_pitch,
                pitch_gains: attitude_roll_pitch,
                max_integral: 100.0,
                max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0), // 45° for roll and pitch
                max_rate_radps: Vec2::new(3.0, 3.0),           // Max roll, pitch rates in rad/s
            },
            ..Default::default()
        },
        mixer: TableMixer::quad_x(),
        // `TableMixer::quad_x` motor `i` drives RealFlight channel `motors[i]`.
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
        // RealFlight's frame for this model is already the copter's.
        body_rotation: UnitQuaternion::identity(),
    }
}

/// The [eXplora VTOL](https://github.com/robustini/eXploraVTOL/tree/main/simulation) tailsitter
/// in RealFlight: two motors on the wing and two elevons, hovering nose up.
///
/// - **Mixer:** [`TableMixer::tailsitter_2p_hover`], whose factors are in the airplane body
///   frame (x the nose).
/// - **Mapping:** motors 0/1 (left/right) on channels 6/5, elevons (servos 0/1) on channels
///   1/2 (RealFlight numbering; 5/4 and 0/1 here), the left elevon reversed. The channels are
///   from the #120 capture; which motor is which and the reversal from #121's first hops.
/// - **Body rotation:** +90° pitch. RealFlight's frame for this model is hover-level (sitting
///   on its tail reads level), and the vehicle's is the airplane's, so hover reads pitch 90°.
///
/// **Interim until the nose-up stabilizer (#122):** the copter stabilizer can't fly at the
/// 90° Euler singularity, so `tailsitter-manual-realflight` flies this airframe in
/// RealFlight's hover-level frame instead (identity rotation), turning the stabilizer's
/// moments into the airplane frame for the mixer. The stabilizer gains below are for that
/// interim wiring, in RealFlight's hover-level axes (roll: tilting sideways, yaw: turning about
/// the vertical), and are a starting point, not a tune.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::airframes;
///
/// let airframe = airframes::explora_vtol();
/// assert_eq!(airframe.mapping.motors[..2], [Some(5), Some(4)]);
/// assert_eq!(airframe.mapping.servos[..2], [Some(0), Some(1)]);
/// ```
pub fn explora_vtol() -> Airframe<CopterGuidanceConfig, CopterStabilizerConfig, TableMixer<4>> {
    let gains = |p, i, d| Gains { p, i, d };
    Airframe {
        guidance: CopterGuidanceConfig {
            altitude: AltitudeControllerConfig {
                // Hover motor 0.7 (#120) = 0.9 · 0.88², through the squared thrust curve and
                // the mixer's thrust factor.
                hover_thrust: 0.88,
                ..CopterGuidanceConfig::default().altitude
            },
            ..CopterGuidanceConfig::default()
        },
        stabilizer: CopterStabilizerConfig {
            body_rate: BodyRateControllerConfig {
                // Differential thrust (the mixer's airplane yaw, 0.25 per unit moment).
                roll_gains: gains(0.2, 0.1, 0.0),
                // Elevons (full range per unit moment).
                pitch_gains: gains(0.3, 0.1, 0.0),
                yaw_gains: gains(0.1, 0.05, 0.0),
                max_integral: 1.0,
            },
            attitude: AttitudeControllerConfig {
                roll_gains: gains(2.0, 0.0, 0.0),
                pitch_gains: gains(2.0, 0.0, 0.0),
                max_integral: 100.0,
                max_angles_rad: Vec2::new(PI / 6.0, PI / 6.0), // 30° for roll and pitch
                max_rate_radps: Vec2::new(2.0, 2.0),
            },
            ..Default::default()
        },
        mixer: TableMixer::tailsitter_2p_hover(),
        mapping: ActuatorMapping {
            // Left motor on channel 6, right on 5: the first hops (#121) rolled toward the motor
            // with more thrust with them the other way round.
            motors: [Some(5), Some(4), None, None, None, None, None, None],
            servos: [Some(0), Some(1), None, None, None, None, None, None],
            // The elevon channels run opposite ways (#121 ground check); with the right one reversed,
            // pitch and yaw diverged on the first hop, so it is the left.
            reversed_servos: [true, false, false, false, false, false, false, false],
        },
        body_rotation: UnitQuaternion::from_euler_angles(0.0, PI / 2.0, 0.0),
    }
}
