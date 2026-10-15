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
use stable_flight::guidance::copter::{CopterGuidanceConfig, ThrustLimits};
use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::guidance::vtol::{TailsitterGuidanceConfig, TransitionConfig};
use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::TailsitterStabilizerConfig;

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
/// The stabilizer gains are hand-tuned in attitude mode; guidance uses the core defaults
/// (lateral and altitude gains and the tilt cap are flight-checked on this model) with this
/// model's measured hover thrust. The QuadCopterX examples fly this set. The model is four
/// motors on channels 0–3; its other channels drive nothing (#271).
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
    // Roll and pitch share one hand tune; autotune can split them.
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
                // = 0.6 cancels it with margin; with less, acro drifts in yaw (#192).
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
/// - **Mixer:** [`TableMixer::tailsitter_2p`], whose factors are in the airplane body
///   frame (x the nose).
/// - **Mapping:** motors 0/1 (left/right) on channels 6/5, elevons (servos 0/1) on channels
///   1/2 (RealFlight numbering; 5/4 and 0/1 here), the left elevon reversed (captures: #120,
///   #121). A sweep of every other channel (3, 4, 7–12; 2, 3, 6–11 here) moved nothing: the
///   model has no other surfaces, so sending them 0.0 is harmless (#271).
/// - **Body rotation:** +90° pitch. RealFlight's frame for this model is hover-level (sitting
///   on its tail reads level), and the vehicle's is the airplane's, so hover reads pitch 90°.
/// - **Thrust limits:** the elevons hold the nose up only in the propwash, and a sink turns
///   the airflow against it (flight data: #123). A floor (0.65, below the 0.84 hover) and a
///   sink guard (past 1 m/s) keep the propwash; the tilt boost stops at 0.9, leaving motor
///   headroom for differential thrust.
///
/// Fly it with [`TailsitterGuidance`](stable_flight::guidance::vtol::TailsitterGuidance) and
/// [`TailsitterStabilizer`](stable_flight::stabilizers::vtol::TailsitterStabilizer), as the
/// tailsitter examples do: hover runs the copter controllers in the tailsitter
/// [`View`](stable_flight::View), where hover is level; forward flight runs the plane
/// controllers in the airplane frame.
///
/// - **Hover:** gains in the view's axes (roll: tilting sideways, yaw: turning about the
///   vertical), hand-tuned from stick pulses and low-throttle descents (flight data: #123,
///   #328). View pitch is capped at 85° for the forward ramp; sticks at 30°.
/// - **Plane and transition:** starting values (cruise throttle, airspeeds, thrusts) until
///   plane flights measure them (#124). The plane side uses the hover thrust curve, so a
///   handoff keeps the motor commands continuous.
///
/// How each value is tuned: the
/// [tailsitter tuning guide](https://github.com/wboayue/stable-flight/blob/main/docs/tuning/tailsitter.md).
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
pub fn explora_vtol()
-> Airframe<TailsitterGuidanceConfig, TailsitterStabilizerConfig, TableMixer<4>> {
    let gains = |p, i, d| Gains { p, i, d };
    Airframe {
        guidance: TailsitterGuidanceConfig {
            hover: CopterGuidanceConfig {
                altitude: AltitudeControllerConfig {
                    // Measured (#123): a level hover holds at mean motor 0.63 = 0.9 · 0.84²,
                    // through the squared thrust curve and the mixer's thrust factor.
                    hover_thrust: 0.84,
                    ..CopterGuidanceConfig::default().altitude
                },
                // The elevons hold the nose up only in the propwash, and a sink of a few m/s
                // turns the airflow against it and drops the nose, whatever the floor. The
                // floor keeps some propwash (0.65 is 60% of hover thrust force, which goes as
                // its square); the sink guard brings thrust back toward hover past a 1 m/s
                // sink (0.65 + 0.15 · 2 m/s = 0.95 at 3 m/s).
                thrust: ThrustLimits {
                    min_thrust: 0.65,
                    sink_limit_mps: 1.0,
                    sink_gain: 0.15,
                    // At full boost the motor pair saturates and differential thrust (view
                    // roll) can only push one way, so a roll can't be stopped (#123).
                    max_boost_thrust: 0.9,
                },
                // Full stick at the stabilizer's 30° tilt.
                max_stick_angles_rad: Vec2::new(PI / 6.0, PI / 6.0),
                ..CopterGuidanceConfig::default()
            },
            plane: PlaneGuidanceConfig {
                // Level cruise held at 14–15 m/s with the stick at 0.85–0.89 (#124).
                cruise_thrust: 0.85,
                ..Default::default()
            },
            transition: TransitionConfig {
                // Above a script's hover climb: its first leg is reached on horizontal distance,
                // before the climb ends (#124).
                min_height_m: 20.0,
                // A slower nose-up, handing over closer to vertical (PX4 hands over at 15°):
                // at 50°/s and 45° it reached hover at 11 m/s and ballooned 10 m, and the hover
                // side cut thrust to descend and tumbled for 8 s (#343).
                back_rate_radps: 25f32.to_radians(),
                back_done_rad: 20f32.to_radians(),
                // The plane side's minimum airspeed: completing at 45° and 1.5 m/s, the wing
                // sank 6 m while it gained speed (#124).
                min_airspeed_mps: 9.0,
                // More than hover (0.84): lift drops as the nose tilts over, until the wing
                // flies.
                forward_thrust: 0.9,
                // Below cruise (0.85), so the slow nose-up turns speed into a slowdown more than
                // a climb: the lowest thrust that hands over without sinking, climbing 9 m
                // (18 m at 0.84; at 0.6 the elevons lost the propwash and the nose wallowed,
                // #343).
                back_thrust: 0.75,
                ..Default::default()
            },
        },
        stabilizer: TailsitterStabilizerConfig {
            hover: CopterStabilizerConfig {
                body_rate: BodyRateControllerConfig {
                    // Differential thrust (the mixer's airplane yaw, 0.25 per unit moment):
                    // weak authority, so it needs these gains to stop a roll (#123).
                    roll_gains: gains(0.4, 0.2, 0.0),
                    // Elevons (full range per unit moment). Sinking, the airflow pushes the
                    // nose over; I · max_integral = 0.5 of moment holds it level (#123).
                    pitch_gains: gains(0.3, 0.5, 0.0),
                    yaw_gains: gains(0.1, 0.05, 0.0),
                    max_integral: 1.0,
                },
                attitude: AttitudeControllerConfig {
                    // The rate loops track well; P 4 asks ~110°/s at 27° off, so recoveries
                    // take about half a second without a lingering tail (#328).
                    roll_gains: gains(4.0, 0.0, 0.0),
                    pitch_gains: gains(4.0, 0.0, 0.0),
                    max_integral: 100.0,
                    // 30° of view roll and pitch; the forward ramp flies past it, to 80°, which
                    // the quaternion attitude error flies cleanly (#124).
                    max_angles_rad: Vec2::new(PI / 6.0, PI / 6.0),
                    max_rate_radps: Vec2::new(2.0, 2.0),
                },
                ..Default::default()
            },
            plane: PlaneStabilizerConfig {
                // The hover side's thrust curve, so a handoff keeps the motor commands
                // continuous.
                thrust_curve_exponent: 2.0,
                // Guesses until plane flights measure them (#124).
                trim_airspeed_mps: 14.0,
                min_airspeed_mps: 9.0,
                max_airspeed_mps: 25.0,
                ..Default::default()
            },
        },
        mixer: TableMixer::tailsitter_2p(),
        mapping: ActuatorMapping {
            // Left motor on channel 6, right on 5.
            motors: [Some(5), Some(4), None, None, None, None, None, None],
            servos: [Some(0), Some(1), None, None, None, None, None, None],
            // The elevon channels run opposite ways; the left one is reversed.
            reversed_servos: [true, false, false, false, false, false, false, false],
        },
        body_rotation: UnitQuaternion::from_euler_angles(0.0, PI / 2.0, 0.0),
    }
}
