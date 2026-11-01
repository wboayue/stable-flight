//! Tuned setups for the RealFlight models this backend flies.
//!
//! Gains belong to the simulated airframe, not to an example: each function returns an
//! [`Airframe`] with the guidance and stabilizer configurations, the mixer, the channel
//! mapping, and the body rotation for one model, so examples (and any other wiring) share one tuned set.

use std::f32::consts::PI;

use stable_flight::commanders::ManeuverConfig;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, PlaneRateControllerConfig, RateGains,
};
use stable_flight::estimation::wind::WindConfig;
use stable_flight::guidance::copter::{CopterGuidanceConfig, ThrustLimits};
use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use stable_flight::guidance::vtol::{
    ForwardThrustConfig, LiftTransitionConfig, QuadPlaneGuidanceConfig, TailsitterGuidanceConfig,
    TransitionConfig,
};
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::{QuadPlaneStabilizerConfig, TailsitterStabilizerConfig};
use stable_flight::{Vec2, Vec3};

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
                // The model needs a steady ~0.40 nose-right yaw moment to hover; core's default
                // yaw_max_integral (0.5) cancels it with margin; with less, acro drifts in yaw
                // (#192). The RealFlight autotune's yaw search (I at 0.3 of P), at 250 Hz: 0.5
                // rad/s steps overshoot 0.01 and rise in 0.50–0.89 s, one of six reaching only 0.89
                // in the 1 s window (#558). P damps the heading loop: 30° heading turns overshoot
                // 7% and settle within 1° in 1.6 s; at P 0.6 they overshoot 25% and ring for 4.2 s,
                // and any kick (a stall, a stick) swings the heading for seconds (#383).
                yaw_gains: Gains {
                    p: 1.1532,
                    i: 0.3460,
                    d: 0.0,
                },
                ..BodyRateControllerConfig::default()
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

/// The eXplora's plane airspeeds, in m/s: least and most (guesses until plane flights measure
/// them, #124).
const EXPLORA_MIN_AIRSPEED_MPS: f32 = 9.0;
const EXPLORA_MAX_AIRSPEED_MPS: f32 = 25.0;

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
///   headroom for differential thrust. Waypoint descents are held to 1.5 m/s, where the nose
///   stays steady (#414).
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
    let rate_defaults = PlaneRateControllerConfig::default();
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
                // A new leg along the span asked for 20° of view roll in one tick, which
                // differential thrust overshot to 35°; shaped at 1 m/s², the request starts at
                // about 8° and the roll peaks at 11° (#418).
                lateral: LateralControllerConfig {
                    max_accel_mps2: 1.0,
                    ..LateralControllerConfig::default()
                },
                // Full stick at the stabilizer's 30° tilt.
                max_stick_angles_rad: Vec2::new(PI / 6.0, PI / 6.0),
                // Sinking tail first slows the airflow over the elevons: at 3 m/s the nose
                // swings ±20° on a 2.5 s cycle, at 2 m/s −8…+1°, at 1.5 m/s within 2° (#414).
                // ArduPilot's and PX4's default descent rate (`WP_SPD_DN`,
                // `MPC_Z_V_AUTO_DN`).
                max_descent_mps: 1.5,
                // Holds the heading on waypoint legs, and never yields it to the wind: facing
                // a leg flies it belly first, the wing's flat face into the airflow, and the
                // hover legs above are flown with the heading held (#418). Turning a wingtip
                // into the wind is `wind_heading_min_mps`'s job (#470).
                waypoint_yaw_radps: 0.0,
                heading_yield_s: 0.0,
                // The rate that turned the wing 90° in 3 mph wind without losing the hold
                // (#417).
                heading_rate_radps: 30.0_f32.to_radians(),
                // Holds and lands with a wingtip into the wind: belly into RealFlight's 2 m/s
                // wind it pinned the 27° tilt cap and drifted 47 m in a 60 -> 20 m descent;
                // turned, it drifted under 1.5 m (#470). Belly-on, the estimate's direction
                // settles within 10° of the 15° it needs by 4 s into the climb.
                wind_heading_min_mps: 1.0,
                // Fitted against RealFlight's own wind (2.1-2.3 m/s, 8 headings at 40 m, 0.2 s
                // means): the push is linear in the air's speed, a wing in its propwash,
                // at 4.76/s across the belly (R² 0.98, residual 0.45 m/s²); a v² term fits no
                // better. Along the span it is 0.22/s with a wingtip into the wind and 0.26-0.30
                // elsewhere; 0.24 read both check flights within 4° and 22% of the truth, where
                // the overall fit's 0.27 read a wingtip wind 30% slow (#473).
                wind: WindConfig {
                    momentum_per_s: Vec2::new(4.76, 0.24),
                    force_noise_mps2: 0.45,
                    ..WindConfig::default()
                },
                ..CopterGuidanceConfig::default()
            },
            plane: PlaneGuidanceConfig {
                // Level cruise held at 14–15 m/s with the stick at 0.85–0.89 (#124).
                cruise_thrust: 0.85,
                tecs: TecsConfig {
                    min_airspeed_mps: EXPLORA_MIN_AIRSPEED_MPS,
                    max_airspeed_mps: EXPLORA_MAX_AIRSPEED_MPS,
                    ..TecsConfig::default()
                },
                ..Default::default()
            },
            transition: TransitionConfig {
                // Above a script's hover climb: its first leg is reached on horizontal distance,
                // before the climb ends (#124).
                min_height_m: 20.0,
                // Half the scripts' 30 m plane legs: room for the back transition's loss
                // below it (3.9 m at a 4.6 m/s sink on the sim eXplora, #450).
                plane_min_height_m: 15.0,
                // A slower nose-up, handing over closer to vertical (PX4 hands over at 15°):
                // at 50°/s and 45° it reached hover at 11 m/s and ballooned 10 m, and the hover
                // side cut thrust to descend and tumbled for 8 s (#343).
                back_rate_radps: 25f32.to_radians(),
                back_done_rad: 20f32.to_radians(),
                // The plane side's minimum airspeed: completing at 45° and 1.5 m/s, the wing
                // sank 6 m while it gained speed (#124).
                min_airspeed_mps: EXPLORA_MIN_AIRSPEED_MPS,
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
                    // The rate and angle gains are RealFlight's autotune of the hover at the
                    // default 40 Hz gyro filter (`copter-autotune-realflight --airframe
                    // explora`, #539).
                    //
                    // Differential thrust (the mixer's airplane yaw, 0.25 per unit moment):
                    // weak and slow, 10-11 rad/s² per unit moment after 36-40 ms. A 40 m
                    // descent in RealFlight's 5 km/h wind held within ±3° of its lean, without
                    // the roll cycle the hand tune showed at D 0 (#476, #539).
                    roll_gains: gains(0.6504, 0.1301, 0.0017),
                    // Elevons (full range per unit moment): 19-20 rad/s² after 8-12 ms. Sinking,
                    // the airflow pushes the nose over; the integral holds 0.28 of moment, and
                    // P the rest: descents at the 1.5 m/s limit hold the nose (#539).
                    pitch_gains: gains(1.4217, 0.2844, 0.0),
                    // The elevons apart (airplane roll).
                    yaw_gains: gains(1.0191, 0.3057, 0.0),
                    // The trim each I term held when it was tuned: its I gain × 1 rad of
                    // error, the larger of roll's and pitch's (#374).
                    max_integral: 0.28,
                    yaw_max_integral: 0.31,
                },
                attitude: AttitudeControllerConfig {
                    // 5° steps rise in 546 ms in roll, 164 ms in pitch, without overshoot
                    // (#539).
                    roll_gains: gains(2.3554, 0.0, 0.0),
                    pitch_gains: gains(9.0, 0.0, 0.0),
                    max_integral: 100.0,
                    // 30° of view roll and pitch; the forward ramp flies past it, to 80°, which
                    // the quaternion attitude error flies cleanly (#124).
                    max_angles_rad: Vec2::new(PI / 6.0, PI / 6.0),
                    max_rate_radps: Vec2::new(2.0, 2.0),
                },
                // The default 40 Hz gyro low-pass, which the autotune tuned on (#539).
                ..Default::default()
            },
            plane: PlaneStabilizerConfig {
                // The plane design's (`StagePlan::plane_design`), each gain the median of
                // three RealFlight runs at 14 m/s and cruise thrust (#535).
                rate: PlaneRateControllerConfig {
                    // Roll is all damping (23/s, no stiffness) on a strong elevon (183 rad/s²
                    // per unit moment), so feedforward carries a roll rate and P only trims
                    // it: 1 rad/s steps rise in 55 ms, 6% over. Higher P or FF: more overshoot.
                    roll: RateGains {
                        p: 0.0229,
                        i: 0.08,
                        ff: 0.1279,
                    },
                    // Pitch is stiff (69–93/s² of weathercock) and barely damped (0–1.3/s), so
                    // P does the work and FF almost none: steps rise in 70–85 ms, 4% over.
                    // I 0.63 (one run 1.26) holds the rate against that stiffness. Higher P:
                    // less damped.
                    pitch: RateGains {
                        p: 0.29,
                        i: 0.63,
                        ff: 0.006,
                    },
                    // The nose-up moment of level flight at 14 m/s (#342, run 0); the plane
                    // design's fits read 0.089 (#535).
                    trim: Vec3::new(0.0, 0.09, 0.0),
                    ..rate_defaults
                },
                // How fast the attitude loops close an angle error, in s: 5° steps rise in
                // 440 ms in roll, 420 ms in pitch, without overshoot (#535). Lower: faster,
                // until the rate loop can't follow.
                time_constant_s: Vec2::new(0.226, 0.226),
                // The hover side's thrust curve, so a handoff keeps the motor commands
                // continuous.
                thrust_curve_exponent: 2.0,
                // A guess until plane flights measure it (#124).
                trim_airspeed_mps: 14.0,
                min_airspeed_mps: EXPLORA_MIN_AIRSPEED_MPS,
                max_airspeed_mps: EXPLORA_MAX_AIRSPEED_MPS,
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
            ..Default::default()
        },
        body_rotation: UnitQuaternion::from_euler_angles(0.0, PI / 2.0, 0.0),
    }
}

/// How the eXplora flies scripted maneuvers (stable-flight-sitl's
/// `ScriptCommander::with_maneuver_config`): rolls and loops at full thrust, doublets at its
/// plane cruise thrust, so a doublet measures the rate loop in the propwash the elevons see in
/// level flight (#342).
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::airframes;
///
/// let maneuvers = airframes::explora_maneuvers();
/// assert_eq!(
///     maneuvers.cruise_thrust,
///     airframes::explora_vtol().guidance.plane.cruise_thrust
/// );
/// ```
pub fn explora_maneuvers() -> ManeuverConfig {
    ManeuverConfig {
        cruise_thrust: explora_vtol().guidance.plane.cruise_thrust,
        ..ManeuverConfig::default()
    }
}

/// The share of the full pusher the Titan Cobra's forward thrust gain (1.2) and core's
/// transition pusher defaults are flown at: the pusher commands of its flights (#433, #448).
const COBRA_PUSHER_SHARE: f32 = 0.9;

/// The Titan Cobra's lift-rotor mixer thrust factor: twice the default moment headroom, a
/// unit moment moving each motor ±0.2. The nose-heavy hover holds a steady 0.38 nose-up
/// moment; at the default 0.9 (±0.1 per unit) that left 0.62 for a step, and nose-up clipped
/// 15× as often as nose-down in an autotune (#438). The Alia's mixer takes the same (#440).
const COBRA_THRUST_FACTOR: f32 = 0.8;

/// The Titan Cobra's mixer: its lift rotors at [`COBRA_THRUST_FACTOR`], with `MixerConfig`'s
/// default yaw ratio (2.5), which holds its heading (#433).
const COBRA_MIXER: MixerConfig = MixerConfig {
    thrust_factor: COBRA_THRUST_FACTOR,
    yaw_ratio: 2.5,
};

/// The motor command the Titan Cobra's lift rotors hover at: linear thrust 0.79 at the default
/// thrust factor, 0.9 · 0.79², from a steady 0.7 m/s descent (#433). ArduPilot's hover, 0.322
/// of full thrust (`Q_M_THST_HOVER`), agrees: √0.322 = 0.57 on RealFlight's squared props.
const COBRA_HOVER_MOTOR: f32 = 0.9 * 0.79 * 0.79;

/// The Titan Cobra's plane airspeeds, in m/s: `ARSPD_FBW_MIN` 11 and `ARSPD_FBW_MAX` 18.
const COBRA_MIN_AIRSPEED_MPS: f32 = 11.0;
const COBRA_MAX_AIRSPEED_MPS: f32 = 18.0;

/// The ArduPilot SITL_Models
/// [Titan Cobra VTOL](https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/QuadPlanes/Titan_Cobra_VTOL)
/// in RealFlight: a lift+cruise quadplane, four fixed lift rotors, a pusher, ailerons, and an
/// A-tail.
///
/// - **Mixer:** [`TableMixer::quadplane_vtail`] at thrust factor 0.8, a unit moment moving
///   each lift rotor ±0.2: twice the default's room for the nose-heavy trim (#438). Its lift
///   rotors are in [`TableMixer::quad_x`]'s order, so a copter wiring (the autotune) flies them
///   on `quad_x` at this config with this mapping.
/// - **Mapping** (channels 0-based; RealFlight numbering is one higher), from a channel sweep
///   and hops on the ground (#433):
///
///   | Slot | Channel | Actuator |
///   |---|---|---|
///   | motor 0 | 6 | front-left lift rotor, clockwise from above |
///   | motor 1 | 4 | front-right, counterclockwise |
///   | motor 2 | 5 | rear-left, counterclockwise |
///   | motor 3 | 7 | rear-right, clockwise |
///   | motor 4 | 2 | pusher |
///   | servo 0 | 0 | ailerons |
///   | servo 1 | 3 | right A-tail panel (a V-tail's left ruddervator) |
///   | servo 2 | 1 | left A-tail panel (a V-tail's right ruddervator) |
///
///   No surface is reversed: ArduPilot drives these channels unreversed through its V-tail
///   mixer, right panel (elevator + rudder)/2, left (elevator − rudder)/2
///   (`Plane::channel_function_mixer`, ArduPilot `b032bcbde3`), with positive aileron,
///   elevator, and rudder outputs rolling right, raising the nose, and yawing right
///   (SITL `Plane::getTorque` on its default `c_l_deltaa`, `c_m_deltae`, and `c_n_deltar`);
///   the rows of [`TableMixer::quadplane_vtail`] match. In flight at 12–16 m/s, a 0.3 cruise
///   moment on each axis and sign pushed the way the mixer expects (`surface-probe-realflight`,
///   #433).
///   Both tail panels are trimmed +0.2 (`SERVO2_TRIM`, `SERVO4_TRIM` 1600).
/// - **Body rotation:** identity; RealFlight's frame for this model is the airplane's.
///
/// Fly it with [`QuadPlaneGuidance`](stable_flight::guidance::vtol::QuadPlaneGuidance) and
/// [`QuadPlaneStabilizer`](stable_flight::stabilizers::vtol::QuadPlaneStabilizer).
///
/// - **Hover:** every rate and angle gain from the RealFlight design search at thrust factor
///   0.8: at 250 Hz every loop's step test rises, and every rate and pitch step settles; roll
///   angle leaves 1–2 of 6 steps past the band (#538, #569). The model is nose-heavy on its
///   lift rotors: with equal front and rear thrust a hop pitched nose down at 155°/s (#433),
///   and a level hover holds front motors at 0.61 and rear at 0.54 (#438), so the pitch rate
///   loop holds a steady nose-up moment: 0.37 of the default mixer's range, 0.18–0.19 of this
///   one's (#438, #538).
/// - **Plane and transition:** core defaults, with the upstream ArduPilot parameter file's
///   airspeeds, trim pitch, and cruise throttle (`TitanCobraVTOL.param`); the forward
///   transition hands over to the wing at its minimum airspeed, as ArduPilot does. A lap
///   (`flights/cobra_transition_lap.csv`) transitions in 8.9 s within 29.0–29.5 m, holds
///   28.9–30.7 m in plane, and transitions back in 5.5 s within 29.9–30.9 m (#538).
/// - **Pusher:** the forward thrust gain (1.08) and the transition's pusher cap (0.9) and slew
///   (0.297/s) are the pusher commands the flights below ran, 0.9 of the gain 1.2 and of
///   core's transition defaults (#433, #448).
/// - **Copter legs:** brake along a curve at 1.2 m/s² (`GUID_LAT_BRK`): uncapped 300 m legs
///   stop 3 m short to 0.1 m past. The linear position loop starts braking at 16.7 m/s only
///   34 m out and overshoots 15–20 m (#488).
/// - **Forward thrust:** on in copter flight (gain 1.08): on a 100 m copter leg ahead the
///   pusher keeps the nose within 7° of level, and the leg loses 0.5 m and climbs 1.4 m at
///   most; without it the nose goes 33° down and the leg loses 4.7 m and balloons 6.4 m
///   (#433).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::QuadPlaneGuidance;
/// use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
/// use stable_flight_sitl_realflight::airframes;
///
/// let airframe = airframes::titan_cobra();
/// let guidance = QuadPlaneGuidance::new(airframe.guidance);
/// let stabilizer = QuadPlaneStabilizer::new(airframe.stabilizer);
/// assert_eq!(airframe.mapping.motors[4], Some(2)); // the pusher
/// ```
pub fn titan_cobra() -> Airframe<QuadPlaneGuidanceConfig, QuadPlaneStabilizerConfig, TableMixer<8>>
{
    let gains = |p, i, d| Gains { p, i, d };
    let defaults = QuadPlaneGuidanceConfig::default();
    Airframe {
        guidance: QuadPlaneGuidanceConfig {
            hover: CopterGuidanceConfig {
                altitude: AltitudeControllerConfig {
                    // The linear thrust that runs the lift rotors at the hover command through
                    // the squared thrust curve and the mixer's thrust factor.
                    hover_thrust: (COBRA_HOVER_MOTOR / COBRA_THRUST_FACTOR).sqrt(),
                    ..defaults.hover.altitude
                },
                lateral: LateralControllerConfig {
                    // Uncapped 300 m legs at 16 m/s stop 3 m short to 0.1 m past their waypoints
                    // (#488). The linear loop, braking from 34 m out, overshot 15–20 m. At 1.5
                    // one stop went 5.5 m past: the velocity I holds the pusher on past the
                    // curve, and the pitch lags its setpoint by 8–10°.
                    brake_mps2: 1.2,
                    ..defaults.hover.lateral
                },
                ..defaults.hover
            },
            plane: PlaneGuidanceConfig {
                // `TRIM_PITCH_CD` 420.
                trim_pitch_rad: 4.2f32.to_radians(),
                // `TRIM_THROTTLE` 47 on a linear pusher curve.
                cruise_thrust: 0.47,
                // ArduPilot's TECS defaults, as the model's params leave them.
                tecs: TecsConfig {
                    min_airspeed_mps: COBRA_MIN_AIRSPEED_MPS,
                    max_airspeed_mps: COBRA_MAX_AIRSPEED_MPS,
                    ..TecsConfig::default()
                },
                ..defaults.plane
            },
            // Copter legs lose 0.5 m with it, 4.7 m without (rustdoc above, #433); PX4's limit
            // is 20 m (`VT_QC_T_ALT_LOSS`).
            forward: ForwardThrustConfig {
                gain: COBRA_PUSHER_SHARE * 1.2,
                ..defaults.forward
            },
            transition: LiftTransitionConfig {
                // ArduPilot starts the forward lift ramp past `ARSPD_FBW_MIN`
                // (`SLT_Transition::update`), 11 for this model; core's 9 is ArduPilot's default
                // (`AIRSPEED_FBW_MIN`).
                min_airspeed_mps: COBRA_MIN_AIRSPEED_MPS,
                forward_thrust: COBRA_PUSHER_SHARE * defaults.transition.forward_thrust,
                pusher_slew_per_s: COBRA_PUSHER_SHARE * defaults.transition.pusher_slew_per_s,
                // Half the scripts' 30 m plane legs: room for the back transition's loss
                // below it (1.9 m at a 3.5 m/s sink on the sim quadplane, #450).
                plane_min_height_m: 15.0,
                ..defaults.transition
            },
            ..defaults
        },
        stabilizer: QuadPlaneStabilizerConfig {
            hover: CopterStabilizerConfig {
                // Every gain below: the RealFlight design search (`copter-autotune-realflight
                // --airframe titan-cobra`, 250 Hz). Step tests on this set rise in 76 ms (roll
                // rate), 86 ms (pitch rate), 466 ms (yaw rate), 161 ms (roll angle, overshoot
                // 0.05) and 198 ms (pitch angle, overshoot 0.04) (#538).
                body_rate: BodyRateControllerConfig {
                    // D 0: the fit puts the motor lag under a tick, inside the 36–40 ms delay
                    // (#563); the hand tune's D 0.0123 on pitch slowed the rate loop's rise to
                    // 175 ms (#538).
                    roll_gains: gains(0.4847, 0.0969, 0.0),
                    pitch_gains: gains(0.3970, 0.0794, 0.0),
                    yaw_gains: gains(1.8091, 0.5427, 0.0),
                    // The nose-heavy hover needs a steady 0.18–0.19 nose-up moment; 0.29 holds
                    // it with about 50% margin. The trim each I term held when it was tuned:
                    // its I gain × 3 rad of error, the larger of roll's and pitch's (#374).
                    max_integral: 0.29,
                    // Its I gain × 1 rad (#374).
                    yaw_max_integral: 0.54,
                },
                attitude: AttitudeControllerConfig {
                    roll_gains: gains(7.1338, 0.0, 0.1),
                    pitch_gains: gains(6.5084, 0.0, 0.1),
                    max_integral: 100.0,
                    max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
                    max_rate_radps: Vec2::new(3.0, 3.0),
                },
                ..Default::default()
            },
            plane: PlaneStabilizerConfig {
                // `TRIM_ARSPD_CM` 1240.
                trim_airspeed_mps: 12.4,
                min_airspeed_mps: COBRA_MIN_AIRSPEED_MPS,
                max_airspeed_mps: COBRA_MAX_AIRSPEED_MPS,
                ..Default::default()
            },
        },
        mixer: TableMixer::quadplane_vtail()
            .with_config(COBRA_MIXER)
            .unwrap_or_else(|e| unreachable!("0.8 is a valid thrust factor: {e}")),
        mapping: ActuatorMapping {
            motors: [
                Some(6),
                Some(4),
                Some(5),
                Some(7),
                Some(2),
                None,
                None,
                None,
            ],
            servos: [Some(0), Some(3), Some(1), None, None, None, None, None],
            // `SERVO2_TRIM`, `SERVO4_TRIM` 1600: (1600 − 1500) / 500.
            servo_trims: [0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            ..Default::default()
        },
        body_rotation: UnitQuaternion::identity(),
    }
}
