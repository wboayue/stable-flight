//! Tuned setups for the X-Plane models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns an
//! [`Airframe`] with the guidance and stabilizer configurations, the mixer, and the actuator
//! mapping for one model, so examples (and any other wiring) share one tuned set.

use stable_flight::Vec2;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, PlaneRateControllerConfig, RateGains, YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use stable_flight::guidance::vtol::{LiftTransitionConfig, QuadPlaneGuidanceConfig};
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizerConfig;

use crate::{ActuatorMapping, Surface, SurfaceLink};

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

/// The Alia's lift and pusher engines, its wing surfaces, nothing idle.
const ALIA_MAPPING: ActuatorMapping = ActuatorMapping {
    motors: [
        Some(1),
        Some(0),
        Some(2),
        Some(3),
        Some(4),
        None,
        None,
        None,
    ],
    idle_engines: &[],
    surfaces: &ALIA_SURFACES,
    neutral_wings: &[],
};

/// The Alia's surfaces on [`TableMixer::quadplane`]'s servos, deflected as X-Plane's own
/// yoke deflects them at full travel (xplane-bridge `design.md`, Spike findings: Surfaces):
/// full right roll drops the left ailerons 10° and raises the right 20°, full back stick
/// raises the elevators 20°, full right rudder reads −30° on the left fin and +30° on the
/// right. The `.acf`'s left wings are 2, 4, 8, and 10 (`_is_right_mult` −1).
const ALIA_SURFACES: [SurfaceLink; 8] = [
    // Servo 0 rolls right: left ailerons down, right up, 20° up and 10° down.
    SurfaceLink::new(0, Surface::Aileron, 2, 10.0, -20.0),
    SurfaceLink::new(0, Surface::Aileron, 4, 10.0, -20.0),
    SurfaceLink::new(0, Surface::Aileron, 3, -20.0, 10.0),
    SurfaceLink::new(0, Surface::Aileron, 5, -20.0, 10.0),
    // Servo 1 raises the nose: trailing edges up (X-Plane's negative).
    SurfaceLink::new(1, Surface::Elevator, 8, -20.0, 20.0),
    SurfaceLink::new(1, Surface::Elevator, 9, -20.0, 20.0),
    // Servo 2 turns the nose right.
    SurfaceLink::new(2, Surface::Rudder, 10, -30.0, 30.0),
    SurfaceLink::new(2, Surface::Rudder, 11, 30.0, -30.0),
];

/// The BETA Technologies Alia 250 (Laminar's stock X-Plane 12 model), a lift+cruise VTOL: four
/// lift rotors on X-Plane's engines 0–3, the pusher on engine 4, the ailerons, elevators, and
/// rudders on its wings. It flies [`QuadPlaneGuidance`] and [`QuadPlaneStabilizer`]; copter
/// flight is its copter regime. Load: X-Plane's default, 6,299 lb (2,857 kg).
///
/// [`QuadPlaneGuidance`]: stable_flight::guidance::vtol::QuadPlaneGuidance
/// [`QuadPlaneStabilizer`]: stable_flight::stabilizers::vtol::QuadPlaneStabilizer
///
/// - **Mixer:** [`TableMixer::quadplane`] with the lift rotors' yaw reversed: in
///   `engine-sweep-xplane`, the front-right and rear-left pair (engines 0 and 2) turn the
///   aircraft nose left, on its wheels and in the air, the reverse of `quad_x`'s matching
///   pair. Thrust factor 0.8: at 0.9 a full moment moves each motor by 0.1 of command, about
///   0.8 rad/s² of roll, and the rate loops can't follow a step; the hover (0.648 per motor)
///   stays under 0.8. The pusher spans its full range (#448). Yaw ratio 1: at the default 2.5, a
///   full yaw drove one diagonal to its ceiling and the other near zero, the desaturation
///   gave up collective, and the Alia sank 53 m at full throttle (#351). At 1, the tail
///   weathervanes it past what yaw can hold at a few m/s sideways: the heading ran 50° off
///   for 6 s (#440).
/// - **Mapping:** `quad_x`'s front-left, front-right, rear-left, rear-right on engines 1, 0,
///   2, 3 (the `.acf`'s engine positions); the pusher on 4; the surfaces as X-Plane's yoke
///   moves them (ailerons on wings 2–5, elevators 8 and 9, rudders 10 and 11).
/// - **Hover gains:** roll rate and the roll and pitch angle P from the sim autotune of
///   `stable-flight-sim`'s `alia250_hover()`, which is sized from the `.acf` and the sweep; its
///   lateral loop, slowed for the aircraft's slow tilt; yaw rate (P 5, no I) and heading (P
///   0.8) from that airframe's yaw steps. Pitch rate P from `copter-autotune-xplane`'s step
///   search in X-Plane: 1.78 against the sim's 2.57. Roll rate and both angle loops keep the
///   sim's values: the search's lower gains for them step-tested no better (#540). The roll
///   and pitch rate I (0.2, the QuadCopterX's) trims X-Plane's nose-up pitch at level thrust
///   (2.2 °/s², which the sim leaves out). The angle I (0.25 roll, 0.15 pitch) holds a tilt
///   against the moment X-Plane's airframe builds as a held tilt drifts it sideways, which
///   the sim's multirotor model (drag only) leaves out too: angle steps end within 7% of the
///   step against 13–23% short at core's 0.001. Nose-up pitch steps still overshoot 6–10%,
///   one-sided as the pitch rate steps' overshoot is (#540, #576).
/// - **Hover thrust:** the liftoff throttle through the mixer's thrust factor and the thrust
///   curve.
/// - **Plane and transitions:** `stable-flight-sim`'s `alia250()`, tuned on a model fitted to
///   this aircraft's level flight, pulses, roll holds, and full-power climb (#392), pinned by a
///   test: trim 45 m/s and airspeeds 40–60 m/s (the band X-Plane holds level), the transition
///   gate at 40 m/s, 15° pitch up (below the stall), 7° trim pitch, the rate gains (pitch I 0.5
///   and P 1 against the wing's pitch stability, feedforward from the model, roll asking at
///   most 15°/s of the ailerons' 17°/s), and total energy control's climb, sink, and pitch
///   damping. X-Plane's own: the cruise thrust, the pusher it held level at 45 m/s (0.72,
///   `plane-probe-xplane`). Bank (20°, `FW_R_LIM`), the pusher slew (`VT_PSHER_SLEW`), the
///   back ramp (`VT_B_TRANS_RAMP`), the 20 m least height (`VT_FW_MIN_ALT`), and PX4's 45 s
///   and 28 s transition limits (`VT_F_TRANS_DUR`, `VT_B_TRANS_DUR`) are px4xplane's Alia
///   (`config/px4_params/5020_xplane_alia250`). The loiter radius is a 20° turn at trim
///   airspeed, about 850 m.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::airframes;
///
/// let airframe = airframes::alia250();
/// assert_eq!(airframe.mapping.motors[..5], [Some(1), Some(0), Some(2), Some(3), Some(4)]);
/// assert_eq!(airframe.stabilizer.plane.trim_airspeed_mps, 45.0);
/// ```
pub fn alia250() -> Airframe<QuadPlaneGuidanceConfig, QuadPlaneStabilizerConfig, TableMixer<8>> {
    let plane_rate = PlaneRateControllerConfig::default();
    Airframe {
        guidance: QuadPlaneGuidanceConfig {
            hover: alia250_hover_guidance(),
            plane: PlaneGuidanceConfig {
                max_bank_rad: 20.0f32.to_radians(),
                max_pitch_up_rad: 15.0f32.to_radians(),
                trim_pitch_rad: 7.0f32.to_radians(),
                cruise_thrust: ALIA_CRUISE_THROTTLE,
                // `NPFG_PERIOD` 40.
                l1_period_s: 40.0,
                loiter_radius_m: 850.0,
                tecs: TecsConfig {
                    max_climb_mps: 1.8,
                    min_sink_mps: 3.5,
                    pitch_damping: 0.5,
                    min_airspeed_mps: ALIA_MIN_AIRSPEED_MPS,
                    max_airspeed_mps: ALIA_MAX_AIRSPEED_MPS,
                    ..TecsConfig::default()
                },
                ..PlaneGuidanceConfig::default()
            },
            transition: LiftTransitionConfig {
                min_airspeed_mps: ALIA_MIN_AIRSPEED_MPS,
                timeout_s: 45.0,
                min_height_m: 20.0,
                // px4xplane's `VT_FW_MIN_ALT` (`config/px4_params/5020_xplane_alia250`): falling
                // back from a 4.3 m/s sink at 50 m/s, X-Plane bottoms out 3.0–3.3 m below it, then
                // slows to hover at 5 m/s and holds within 0.5 m (#493, #496).
                plane_min_height_m: 20.0,
                back_ramp_s: 10.0,
                back_timeout_s: 28.0,
                pusher_slew_per_s: 0.45,
                ..LiftTransitionConfig::default()
            },
            ..QuadPlaneGuidanceConfig::default()
        },
        stabilizer: QuadPlaneStabilizerConfig {
            hover: alia250_hover_stabilizer(),
            plane: PlaneStabilizerConfig {
                rate: PlaneRateControllerConfig {
                    roll: RateGains {
                        p: 0.47,
                        i: 0.05,
                        ff: 3.288,
                    },
                    pitch: RateGains {
                        p: 1.0,
                        i: 0.5,
                        ff: 0.9068,
                    },
                    yaw: RateGains {
                        p: 0.1,
                        i: 0.12,
                        ff: 3.570,
                    },
                    ..plane_rate
                },
                time_constant_s: Vec2::new(0.6, 0.4),
                max_attitude_rate_radps: Vec2::new(15.0f32.to_radians(), 60.0f32.to_radians()),
                max_bank_rad: 20.0f32.to_radians(),
                trim_airspeed_mps: 45.0,
                min_airspeed_mps: ALIA_MIN_AIRSPEED_MPS,
                max_airspeed_mps: ALIA_MAX_AIRSPEED_MPS,
                ..PlaneStabilizerConfig::default()
            },
        },
        mixer: TableMixer::quadplane()
            .with_reversed_yaw()
            .with_config(ALIA_MIXER)
            .unwrap_or_else(|e| unreachable!("0.8 is a valid thrust factor: {e}")),
        mapping: ALIA_MAPPING,
    }
}

/// The plane side's slowest, and where the lift ramp starts: the sim `alia250()`'s.
const ALIA_MIN_AIRSPEED_MPS: f32 = 40.0;

/// The plane side's fastest: the sim `alia250()`'s.
const ALIA_MAX_AIRSPEED_MPS: f32 = 60.0;

/// The pusher X-Plane holds level at the 45 m/s trim (`plane-probe-xplane`, #392).
const ALIA_CRUISE_THROTTLE: f32 = 0.72;

/// The Alia's mixer: see [`alia250`].
// TODO(#440): a higher yaw ratio once yaw yields to collective in the mixer.
const ALIA_MIXER: MixerConfig = MixerConfig {
    thrust_factor: ALIA_THRUST_FACTOR,
    yaw_ratio: 1.0,
};

/// The Alia's hover guidance: see [`alia250`].
fn alia250_hover_guidance() -> CopterGuidanceConfig {
    CopterGuidanceConfig {
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
    }
}

/// The Alia's hover stabilizer: see [`alia250`].
fn alia250_hover_stabilizer() -> CopterStabilizerConfig {
    let defaults = CopterStabilizerConfig::default();
    let rate = |p| Gains { p, i: 0.2, d: 0.0 };
    // No D, as the autotune ran. The I holds a tilt against the moment X-Plane's airframe
    // builds as the tilt drifts it sideways (1–2.6 m/s within a 5° step): at core's 0.001,
    // angle steps end 13–23% short. Higher: steps end closer, then overshoot: at 0.5, roll 6%
    // and pitch 25% (#576).
    let angle = |i| Gains { p: 1.03, i, d: 0.0 };
    CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: rate(1.52),
            // Higher overshoots nose-up steps: 12% at the sim tune's 2.57, 4% at 1.78, which
            // also settles in 0.8 s against 1.1 s (#540).
            pitch_gains: rate(1.78),
            yaw_gains: Gains {
                p: 5.0,
                i: 0.0,
                d: 0.0,
            },
            // The trim the I terms held when they were tuned: I 0.2 × 1 rad of error (#374).
            max_integral: 0.2,
            ..defaults.body_rate
        },
        attitude: AttitudeControllerConfig {
            roll_gains: angle(0.25),
            pitch_gains: angle(0.15),
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
    }
}
