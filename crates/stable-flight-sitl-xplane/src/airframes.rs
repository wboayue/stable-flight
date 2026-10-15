//! Tuned setups for the X-Plane models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns an
//! [`Airframe`] with the guidance and stabilizer configurations, the mixer, and the actuator
//! mapping for one model, so examples (and any other wiring) share one tuned set.

use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::ActuatorMapping;

#[cfg(test)]
mod tests;

/// Everything model-specific about one X-Plane airframe.
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
    /// X-Plane engine for each of the mixer's motors, and what is held idle or neutral.
    pub mapping: ActuatorMapping,
}

/// Per-engine throttle at which the Alia leaves the runway at X-Plane's default load
/// (6,299 lb), from `engine-sweep-xplane` (0.645–0.649 over three runs, #351).
const ALIA_LIFTOFF_THROTTLE: f32 = 0.648;

/// The Alia's mixer thrust factor: twice the default moment headroom (see [`alia250`]).
const ALIA_THRUST_FACTOR: f32 = 0.8;

/// The Alia's thrust curve: X-Plane's electric engine throttle sets motor power, and static
/// thrust goes with power^(2/3) (sweep steps from liftoff, #351), so the stabilizer raises its
/// linear thrust to the 1.5 to command the motors.
const ALIA_THRUST_CURVE_EXPONENT: f32 = 1.5;

/// The BETA Technologies Alia 250 (Laminar's stock X-Plane 12 model) in copter mode: its four
/// lift rotors on X-Plane's engines 0–3, the pusher (engine 4) idle, the surfaces held
/// neutral. Load: X-Plane's default, 6,299 lb (2,857 kg).
///
/// - **Mixer:** [`TableMixer::quad_x`] with yaw reversed: in `engine-sweep-xplane`, the
///   front-right and rear-left pair (engines 0 and 2) turn the aircraft nose left, on its
///   wheels and in the air, the reverse of `quad_x`'s matching pair. Thrust factor 0.8: at
///   0.9 a full moment moves each motor by 0.1 of command, about 0.8 rad/s² of roll, and the
///   rate loops can't follow a step; the hover (0.648 per motor) stays under 0.8. Yaw ratio 1:
///   at the default 2.5, a full yaw drove one diagonal to its ceiling and the other near
///   zero, the desaturation gave up collective, and the Alia sank 53 m at full throttle
///   (#351).
/// - **Mapping:** `quad_x`'s front-left, front-right, rear-left, rear-right on engines 1, 0,
///   2, 3 (the `.acf`'s engine positions); the pusher idle; the wings with ailerons (2–5),
///   elevators (8, 9), and rudders (10, 11) held at 0°.
/// - **Gains:** roll and pitch rate and angle from the sim autotune of
///   `stable-flight-sim`'s `alia250()`, which is sized from the `.acf` and the sweep; its
///   lateral loop, slowed for the aircraft's slow tilt; yaw rate (P 5, no I) and heading (P
///   0.8) from that airframe's yaw steps. The roll and pitch rate I (0.2, the QuadCopterX's)
///   trims X-Plane's nose-up pitch at level thrust (2.2 °/s², which the sim leaves out).
/// - **Hover thrust:** the liftoff throttle through the mixer's thrust factor and the thrust
///   curve.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::airframes;
///
/// let airframe = airframes::alia250();
/// assert_eq!(airframe.mapping.motors[..4], [Some(1), Some(0), Some(2), Some(3)]);
/// assert_eq!(airframe.mapping.idle_engines, &[4]);
/// ```
pub fn alia250() -> Airframe<CopterGuidanceConfig, CopterStabilizerConfig, TableMixer<4>> {
    let rate = |p| Gains { p, i: 0.2, d: 0.0 };
    let angle = Gains {
        p: 1.03,
        i: 0.0,
        d: 0.0,
    };
    let defaults = CopterStabilizerConfig::default();
    Airframe {
        guidance: CopterGuidanceConfig {
            altitude: AltitudeControllerConfig {
                hover_thrust: (ALIA_LIFTOFF_THROTTLE / ALIA_THRUST_FACTOR)
                    .powf(1.0 / ALIA_THRUST_CURVE_EXPONENT),
                ..CopterGuidanceConfig::default().altitude
            },
            lateral: LateralControllerConfig {
                position_p: 0.2,
                velocity: Gains {
                    p: 0.4,
                    i: 0.05,
                    d: 0.0,
                },
                ..LateralControllerConfig::default()
            },
            ..CopterGuidanceConfig::default()
        },
        stabilizer: CopterStabilizerConfig {
            body_rate: BodyRateControllerConfig {
                roll_gains: rate(1.52),
                pitch_gains: rate(2.57),
                yaw_gains: Gains {
                    p: 5.0,
                    i: 0.0,
                    d: 0.0,
                },
                ..defaults.body_rate
            },
            attitude: AttitudeControllerConfig {
                roll_gains: angle,
                pitch_gains: angle,
                ..defaults.attitude
            },
            yaw: YawControllerConfig {
                gains: Gains {
                    p: 0.8,
                    i: 0.0,
                    d: 0.0,
                },
                ..defaults.yaw
            },
            thrust_curve_exponent: ALIA_THRUST_CURVE_EXPONENT,
            ..defaults
        },
        mixer: TableMixer::quad_x()
            .with_reversed_yaw()
            .with_config(MixerConfig {
                thrust_factor: ALIA_THRUST_FACTOR,
                yaw_ratio: 1.0,
            })
            .unwrap_or_else(|e| unreachable!("0.8 is a valid thrust factor: {e}")),
        mapping: ActuatorMapping {
            motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
            idle_engines: &[4],
            neutral_wings: &[2, 3, 4, 5, 8, 9, 10, 11],
        },
    }
}
