//! The simulated tailsitter: see [`tailsitter`].

use core::f32::consts::PI;

use stable_flight::Vec2;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, RateGains,
};
use stable_flight::estimation::wind::WindConfig;
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::guidance::vtol::{TailsitterGuidanceConfig, TransitionConfig};
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::TailsitterStabilizerConfig;

use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::tailsitter_model::{PropWash, TailsitterModel};

use super::hover_thrust;
use super::plane::{cruise_thrust, plane};
use crate::motors::QUADRATIC;

/// A simulated tailsitter: its physics, the mixer that drives its motors and elevons, and the
/// guidance and stabilizer configs tuned for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TailsitterAirframe {
    /// Physical parameters.
    pub model: TailsitterModel,
    /// The mixer; [`SimTailsitter`](crate::SimTailsitter) reads the motors and surfaces from
    /// its rows.
    pub mixer: TableMixer<4>,
    /// Guidance configuration: hover, plane, and the transition between them.
    pub guidance: TailsitterGuidanceConfig,
    /// Stabilizer configuration: hover and plane.
    pub stabilizer: TailsitterStabilizerConfig,
}

/// ArduPilot SITL's tailsitter (`plane-tailsitter`: [`plane`]'s airframe with 1.5× its
/// thrust, `libraries/SITL/SIM_Plane.cpp:78-81`, Plane-4.5), its thrust split over two motors
/// on [`TableMixer::tailsitter_2p`], with the elevons in their wash.
///
/// The rest isn't in ArduPilot's model and is chosen:
/// - A flying wing's sideslip terms in place of the airplane's, whose tail weathercocks a
///   hover in a wind along the span (90° of sideslip) past what differential thrust holds.
/// - Motors 25 cm either side of the centerline, for the differential thrust that yaws it.
/// - Two 10-inch props washing the elevons wholly. In a hover that is about 190 Pa over them,
///   as from 18 m/s of airspeed; ArduPilot adds 20 m/s × throttle to the airspeed of its
///   moments instead (`SIM_Plane.cpp:154-162`).
///
/// The hover side flies the eXplora's hand tune from RealFlight (#123, #328; the same layout
/// of differential thrust and elevons) with view pitch rate D added for the gyro low-pass,
/// which holds the sim's nose-up hover within the copter's calm-air bands
/// (`tests/tailsitter_hover.rs`). The plane side is [`plane`]'s,
/// without the rudder feedforward (yaw is differential thrust here), on the hover side's
/// thrust curve so a handoff keeps the motor commands continuous, and with room to pitch up
/// for the back transition. Transitions fly core's defaults but for the airspeed gate, the
/// wing's minimum airspeed; they meet PX4's limits in `tests/tailsitter_transition.rs`.
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::vtol::TailsitterGuidance;
/// use stable_flight::stabilizers::vtol::TailsitterStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimTailsitter, airframes};
///
/// let airframe = airframes::tailsitter();
/// let vehicle = SimTailsitter::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     TailsitterGuidance::new(airframe.guidance),
///     TailsitterStabilizer::new(airframe.stabilizer),
///     airframe.mixer,
///     RecordingTelemetry::<16>::new(),
/// );
/// ```
pub fn tailsitter() -> TailsitterAirframe {
    const MOTORS: u8 = 2;
    const PROP_DIAMETER_M: f32 = 0.254;
    let airplane = plane();
    let model = TailsitterModel {
        wing: FixedWingModel {
            max_thrust_n: 1.5 * airplane.model.max_thrust_n / f32::from(MOTORS),
            aero: AeroCoefficients {
                // A flying wing's sideslip terms, the Zagi's (Beard and McLain, *Small Unmanned
                // Aircraft*, App. E): no tail to weathercock it. In a nose-up hover, wind along
                // the span is 90° of sideslip, and the airplane's tail turns 4 m/s of it into a
                // yaw moment past differential thrust (#387).
                c_side_beta: -0.07359,
                c_roll_beta: -0.02854,
                c_yaw_beta: -0.0004,
                ..airplane.model.aero
            },
            ..airplane.model
        },
        motor_arm_m: 0.25,
        // Its props, 25 cm out on the airplane's 2.9 m wing, wash under 2% of the wing's rate
        // damping (`PropWash::span_damping_share`): left out.
        prop_wash: PropWash::props(MOTORS, PROP_DIAMETER_M, 1.0, 0.0),
    };
    let mixer = TableMixer::tailsitter_2p();
    let gains = |p, i, d| Gains { p, i, d };
    let hover_stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            // View roll: differential thrust.
            roll_gains: gains(0.4, 0.2, 0.0),
            // View pitch: the elevons together. The D damps them against the gyro low-pass's
            // lag: without it, a hover requested mid-transition rings to 13.8° of tilt at the
            // default 40 Hz. D 0.001–0.04 holds the calm-air bands, and higher damps the angle
            // steps more (overshoot 0.10 at 0.0025, 0.015 at 0.01); 0.08 rings again. 0.01
            // sits 10× above the low end and 4× below the high one (#550).
            pitch_gains: gains(0.3, 0.5, 0.01),
            // View yaw: the elevons opposite ways.
            yaw_gains: gains(0.1, 0.05, 0.0),
            // The trim the I terms held when they were tuned: pitch's I × 1 rad of error, and
            // yaw's (#374).
            max_integral: 0.5,
            yaw_max_integral: 0.05,
        },
        attitude: AttitudeControllerConfig {
            roll_gains: gains(4.0, 0.0, 0.0),
            pitch_gains: gains(4.0, 0.0, 0.0),
            max_integral: 100.0,
            // 30° of view roll and pitch; the forward ramp flies past it.
            max_angles_rad: Vec2::new(PI / 6.0, PI / 6.0),
            max_rate_radps: Vec2::new(2.0, 2.0),
        },
        ..CopterStabilizerConfig::default()
    };
    let thrust_curve_exponent = hover_stabilizer.thrust_curve_exponent;
    let plane_stabilizer = PlaneStabilizerConfig {
        rate: stable_flight::controllers::PlaneRateControllerConfig {
            yaw: RateGains {
                ff: 0.0,
                ..airplane.stabilizer.rate.yaw
            },
            ..airplane.stabilizer.rate
        },
        thrust_curve_exponent,
        ..airplane.stabilizer
    };
    let mixer_config = MixerConfig::default();
    let guidance = TailsitterGuidanceConfig {
        hover: CopterGuidanceConfig {
            altitude: AltitudeControllerConfig {
                hover_thrust: hover_thrust(
                    model.wing.mass_kg,
                    model.wing.max_thrust_n,
                    &mixer,
                    &mixer_config,
                    QUADRATIC,
                    thrust_curve_exponent,
                ),
                ..CopterGuidanceConfig::default().altitude
            },
            // The eXplora's: unshaped, a hover leg along the span at 3 m/s overshoots its speed
            // past what differential thrust holds against the sideslip, and departs (#418).
            lateral: LateralControllerConfig {
                max_accel_mps2: 1.0,
                ..LateralControllerConfig::default()
            },
            // The eXplora's: legs hold the heading, since facing one flies it belly first, and
            // never yield it.
            waypoint_yaw_radps: 0.0,
            heading_yield_s: 0.0,
            // The model's own drag in a hover, fitted on the steady lean in 0.5–3 m/s winds:
            // bluff, k = ½ρ/B, 0.29/m across the belly and 0.016/m along the span. It feeds the
            // drag of a leg's speed forward, so the lateral I doesn't learn it and carry it past
            // the waypoint (#609), and drives the wind estimate.
            wind: WindConfig {
                ballistic_kgpm2: Vec2::new(2.1, 39.0),
                ..WindConfig::default()
            },
            ..CopterGuidanceConfig::default()
        },
        plane: PlaneGuidanceConfig {
            cruise_thrust: cruise_thrust(
                &model.wing,
                &mixer,
                &mixer_config,
                thrust_curve_exponent,
                plane_stabilizer.trim_airspeed_mps,
            ),
            ..airplane.guidance
        },
        transition: TransitionConfig {
            // The wing's own: handing over slower, it sinks while gaining speed (#124).
            min_airspeed_mps: plane_stabilizer.min_airspeed_mps,
            // A slow nose-up at below-cruise thrust turns the plane leg's speed into a slowdown
            // rather than a climb, handing over at about 2.5 m/s; the sink guard (default)
            // catches a ramp that stalls (#343).
            back_rate_radps: 25f32.to_radians(),
            back_done_rad: 20f32.to_radians(),
            back_thrust: 0.6,
            ..TransitionConfig::default()
        },
    };
    TailsitterAirframe {
        model,
        mixer,
        guidance,
        stabilizer: TailsitterStabilizerConfig {
            hover: hover_stabilizer,
            plane: plane_stabilizer,
        },
    }
}
