//! Simulated airframes: physics paired with the mixer and flight-controller configs that fly
//! them.

use stable_flight::GRAVITY;
use stable_flight::Vec3;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, RATE_I_PER_P, YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::{MixerConfig, Slot, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::model::MultirotorModel;
use crate::motors::QUADRATIC;

mod alia;
mod explora;
mod plane;
mod quadplane;
mod tailsitter;
#[cfg(test)]
mod tests;

pub use alia::alia250;
pub use explora::explora;
pub use plane::{PlaneAirframe, plane};
pub use quadplane::{QuadPlaneAirframe, quadplane};
pub use tailsitter::{TailsitterAirframe, tailsitter};

/// A simulated multirotor: its physics, the mixer that lays out its motors, and the guidance
/// and stabilizer configs tuned for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Airframe {
    /// Physical parameters.
    pub model: MultirotorModel,
    /// The mixer; [`SimCopter`](crate::SimCopter) takes the motor layout from its rows.
    pub mixer: TableMixer<4>,
    /// Guidance (position-loop) configuration.
    pub guidance: CopterGuidanceConfig,
    /// Stabilizer configuration.
    pub stabilizer: CopterStabilizerConfig,
}

/// A 1 kg X quad, 250 mm motor to motor, thrust-to-weight about 2.4: roughly a 5-inch
/// freestyle quad with a camera, chosen to be plausible, not to match a real one.
///
/// Roll and pitch rate and angle gains are a sim autotune's step search (#347); yaw rate and
/// heading are hand-tuned on the sim's step responses
/// (`tests/step_response.rs`). Guidance uses the core defaults, with the hover thrust worked
/// out from the model rather than measured.
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::copter::CopterGuidance;
/// use stable_flight::stabilizers::copter::CopterStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimCopter, airframes};
///
/// let airframe = airframes::quad_x();
/// let vehicle = SimCopter::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     CopterGuidance::new(airframe.guidance),
///     CopterStabilizer::new(airframe.stabilizer),
///     airframe.mixer,
///     RecordingTelemetry::<16>::new(),
/// );
/// ```
pub fn quad_x() -> Airframe {
    let model = MultirotorModel {
        mass_kg: 1.0,
        inertia_kgm2: Vec3::new(0.01, 0.01, 0.018),
        // 250 mm motor to motor on the diagonal.
        moment_arm_m: x_moment_arm_m(0.25),
        max_thrust_n: 6.0,
        thrust_exponent: QUADRATIC,
        motor_time_constant_s: 0.03,
        yaw_torque_per_thrust_m: 0.016,
        drag_n_per_mps: 0.3,
        angular_damping_nm_per_radps: Vec3::new(0.002, 0.002, 0.004),
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    build(
        model,
        Tune {
            // Sim autotune from core's defaults (#373); from rate P × 0.3 to × 3 alike.
            roll_rate: rate_gains(2.70, 0.0567),
            pitch_rate: rate_gains(2.70, 0.0567),
            angle_p: 9.0,
            yaw_rate_p: YAW_RATE_P,
            heading_p: HEADING_P,
        },
    )
}

/// A 3 kg X quad, 350 mm motor to motor, thrust-to-weight 2: ArduPilot SITL's default
/// multicopter (`libraries/SITL/SIM_Frame.cpp:51,57`, mass and diagonal), heavier and slower
/// to turn than [`quad_x`].
///
/// Inertia follows ArduPilot's ring model: half the mass on the motor ring
/// (`SIM_Frame.cpp:821-825`). The rest isn't in ArduPilot's model and is chosen:
/// thrust-to-weight 2 at the low end of what flies well (ArduPilot's own hover throttle maps
/// through a PWM curve that isn't comparable to the sim's `command²`), 50 ms motor lag for
/// larger props, and drag, damping, and yaw torque scaled from `quad_x`. Gains come from the
/// sim autotune like `quad_x`'s.
pub fn heavy_quad() -> Airframe {
    const MASS_KG: f32 = 3.0;
    const DIAGONAL_M: f32 = 0.35;
    let model = MultirotorModel {
        mass_kg: MASS_KG,
        inertia_kgm2: ring_inertia(MASS_KG, DIAGONAL_M),
        moment_arm_m: x_moment_arm_m(DIAGONAL_M),
        max_thrust_n: max_thrust_n(MASS_KG, 2.0),
        thrust_exponent: QUADRATIC,
        motor_time_constant_s: 0.05,
        // Prop drag torque per thrust grows with prop diameter: about 7-inch props against
        // `quad_x`'s 5-inch.
        yaw_torque_per_thrust_m: 0.022,
        // Frontal area goes with the diagonal squared.
        drag_n_per_mps: 0.6,
        // `quad_x`'s scaled by mass × moment arm (about 4.2).
        angular_damping_nm_per_radps: Vec3::new(0.0084, 0.0084, 0.0168),
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    build(
        model,
        Tune {
            // Sim autotune from core's defaults (#373).
            roll_rate: rate_gains(2.47, 0.0559),
            pitch_rate: rate_gains(2.47, 0.0559),
            angle_p: 8.54,
            // `quad_x`'s: they hold heading in a hover; the autotune doesn't tune them.
            yaw_rate_p: YAW_RATE_P,
            heading_p: HEADING_P,
        },
    )
}

/// A 0.8 kg X quad, 250 mm motor to motor, thrust-to-weight 8: ArduPilot SITL's 5-inch
/// freestyle model (`Tools/autotest/models/freestyle.json`, mass and diagonal), lighter and
/// far stronger than [`quad_x`].
///
/// Inertia follows ArduPilot's ring model: half the mass on the motor ring
/// (`SIM_Frame.cpp:821-825`). The rest is chosen: thrust-to-weight 8, typical of a freestyle
/// build (the model's hover throttle maps through a PWM curve that isn't comparable to the
/// sim's `command²`), 15 ms motor lag for small high-kV motors, and drag, damping, and yaw
/// torque from `quad_x`, the same size. Gains come from the sim autotune like `quad_x`'s.
pub fn racer() -> Airframe {
    const MASS_KG: f32 = 0.8;
    const DIAGONAL_M: f32 = 0.25;
    let model = MultirotorModel {
        mass_kg: MASS_KG,
        inertia_kgm2: ring_inertia(MASS_KG, DIAGONAL_M),
        moment_arm_m: x_moment_arm_m(DIAGONAL_M),
        max_thrust_n: max_thrust_n(MASS_KG, 8.0),
        thrust_exponent: QUADRATIC,
        motor_time_constant_s: 0.015,
        yaw_torque_per_thrust_m: 0.016,
        drag_n_per_mps: 0.3,
        // `quad_x`'s scaled by mass (same moment arm).
        angular_damping_nm_per_radps: Vec3::new(0.0016, 0.0016, 0.0032),
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    build(
        model,
        Tune {
            // Sim autotune from core's defaults (#373).
            roll_rate: rate_gains(0.550, 0.00655),
            pitch_rate: rate_gains(0.550, 0.00655),
            angle_p: 9.0,
            // `quad_x`'s: they hold heading in a hover; the autotune doesn't tune them.
            yaw_rate_p: YAW_RATE_P,
            heading_p: HEADING_P,
        },
    )
}

/// The Alia's mixer: [`alia250_hover`]'s reasons.
// TODO(#440): a higher yaw ratio once yaw yields to collective in the mixer.
const ALIA_MIXER: MixerConfig = MixerConfig {
    thrust_factor: 0.8,
    yaw_ratio: 1.0,
};

/// The BETA Technologies Alia 250 eVTOL in copter mode, as X-Plane 12 flies it (Laminar's
/// stock model): four lift rotors on X-Plane's engines 0–3, the pusher idle, at X-Plane's
/// default load (6,299 lb). The model is sized from the `.acf` and an `engine-sweep-xplane`
/// run (`stable-flight-sitl-xplane`, #351) rather than from the real aircraft:
///
/// - **Thrust:** X-Plane's electric engine throttle sets motor power, and static prop thrust
///   goes with power^(2/3), so `thrust_exponent` is 2/3. It fits both sweep steps from the
///   liftoff throttle 0.648 (0.1 → 1.24 m/s², 0.2 → 1.9–2.1 m/s²; linear predicts 1.52 and
///   3.04, quadratic 3.27 and 7.03). Full thrust is the liftoff thrust scaled by
///   (1 / 0.648)^(2/3): thrust-to-weight 1.34. The stabilizer's `thrust_curve_exponent` 1.5
///   undoes the 2/3.
/// - **Motor lag:** 0.17 s, the step's time to 63% (0.16–0.20 s over two runs).
/// - **Moments:** airborne ±0.05 throttle pulses gave 0.416 rad/s² of roll and 0.249 rad/s² of
///   pitch, so with the roll arm (the rotors 2.59 m either side) the effective inertias are
///   8,570 and 14,300 kg·m²; the pitch figure folds in the longer pitch arm (3.04 m), so one
///   arm serves both axes. Yaw inertia takes the `.acf`'s yaw-to-roll ratio (4.99 / 6.84), and
///   the yaw pulse's −0.087 rad/s² (the first diagonal up) gives the yaw torque per newton:
///   0.39 m, prop drag plus the rotors' 3° toe.
/// - **Spin:** each diagonal turns the aircraft the other way from `quad_x`'s on the ground
///   and in the air, so the mixer is `quad_x` with yaw reversed.
/// - **Not measured:** drag (a guess from the airframe's size) and damping (none).
///
/// X-Plane also pitches the Alia up at 2.2 °/s² with level thrust, which this model leaves
/// out; in X-Plane the pitch rate I trims it.
///
/// The mixer runs at thrust factor 0.8, and the lateral loop is slowed for the aircraft's slow
/// tilt (both below). Roll and pitch rate and angle gains are the sim autotune's on a plan
/// scaled to the aircraft: core's copter probes ask for more than its authority gives.
pub fn alia250_hover() -> Airframe {
    const MASS_KG: f32 = 2857.0;
    /// Per-engine throttle at liftoff in the sweep.
    const LIFTOFF_THROTTLE: f32 = 0.648;
    const THRUST_EXPONENT: f32 = 2.0 / 3.0;
    let hover_motor_n = MASS_KG * GRAVITY / 4.0;
    let model = MultirotorModel {
        mass_kg: MASS_KG,
        inertia_kgm2: Vec3::new(8_570.0, 14_300.0, 8_570.0 * 4.99 / 6.84),
        moment_arm_m: 2.59,
        max_thrust_n: hover_motor_n / libm::powf(LIFTOFF_THROTTLE, THRUST_EXPONENT),
        thrust_exponent: THRUST_EXPONENT,
        motor_time_constant_s: 0.17,
        yaw_torque_per_thrust_m: 0.39,
        // About 100 N per m/s: side and top area of a 15 m wing and 12 m fuselage at a few
        // m/s.
        drag_n_per_mps: 100.0,
        angular_damping_nm_per_radps: Vec3::zeros(),
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    let mut airframe = build_with(
        model,
        TableMixer::quad_x().with_reversed_yaw(),
        // Twice the default moment headroom: at 0.9 a full moment moves each motor by 0.1,
        // about 0.8 rad/s² of roll, and the rate loop can't follow the autotune's steps. The
        // hover (0.648 per motor) stays under the 0.8 ceiling. Yaw at roll's motor factor: at
        // the default 2.5, full yaw drove one diagonal to its ceiling and the other near zero,
        // and the desaturation gave up collective (X-Plane: a sink to the ground at full
        // throttle, #351).
        ALIA_MIXER,
        1.0 / THRUST_EXPONENT,
        Tune {
            // Sim autotune on the large-aircraft plan (`tests/alia.rs`), from rate P 1 and 3
            // with angle P 1 (#351); that plan keeps rate I out (#374).
            roll_rate: Gains {
                p: 1.52,
                i: 0.0,
                d: 0.0,
            },
            pitch_rate: Gains {
                p: 2.57,
                i: 0.0,
                d: 0.0,
            },
            angle_p: 1.03,
            // Hand-tuned on the yaw steps (`tests/alia.rs`): `quad_x`'s overshot a 29° heading
            // step by 70% and limit-cycled in X-Plane (#351).
            yaw_rate_p: 5.0,
            heading_p: 0.8,
        },
    );
    // No yaw rate I: on this slow axis it overshoots the heading steps by 25–50%.
    airframe.stabilizer.body_rate.yaw_gains.i = 0.0;
    // The autotune ran on angle loops without D (core's default angle D 1.0 holds a 5° step
    // to three quarters of it after 2.5 s).
    airframe.stabilizer.attitude.roll_gains.d = 0.0;
    airframe.stabilizer.attitude.pitch_gains.d = 0.0;
    // A slower lateral loop for the slower tilt: on core's defaults the position loop sways
    // into the 27° tilt cap after a 5° attitude step; at these, a 6 m/s gust moves the
    // aircraft 0.9 m and it returns (#351).
    airframe.guidance.lateral = LateralControllerConfig {
        position_p: 0.2,
        velocity: Gains {
            p: 0.4,
            i: 0.05,
            d: 0.0,
        },
        ..LateralControllerConfig::default()
    };
    airframe
}

/// Every airframe, by its function's name: [`quad_x`], [`heavy_quad`], and [`racer`].
///
/// # Examples
///
/// ```
/// let names = stable_flight_sim::airframes::all().map(|(name, _)| name);
/// assert_eq!(names, ["quad_x", "heavy_quad", "racer"]);
/// ```
pub fn all() -> [(&'static str, Airframe); 3] {
    [
        ("quad_x", quad_x()),
        ("heavy_quad", heavy_quad()),
        ("racer", racer()),
    ]
}

/// Principal inertia (roll, pitch, yaw) of a quad with half its mass on a ring through the
/// motors, in kg·m²: ArduPilot SITL's model when a frame gives none
/// (`libraries/SITL/SIM_Frame.cpp:821-825`).
fn ring_inertia(mass_kg: f32, diagonal_m: f32) -> Vec3 {
    let radius_m = diagonal_m / 2.0;
    let roll_kgm2 = mass_kg * 0.25 * radius_m * radius_m;
    Vec3::new(roll_kgm2, roll_kgm2, 2.0 * roll_kgm2)
}

/// Distance from the roll or pitch axis to each motor of an X quad whose motors are
/// `diagonal_m` apart across the diagonal, in m: the arm (half the diagonal) at 45° to each
/// axis.
fn x_moment_arm_m(diagonal_m: f32) -> f32 {
    diagonal_m / 2.0 * core::f32::consts::FRAC_1_SQRT_2
}

/// One motor's full thrust, in N, giving a quad of `mass_kg` a thrust-to-weight of
/// `thrust_to_weight`.
fn max_thrust_n(mass_kg: f32, thrust_to_weight: f32) -> f32 {
    mass_kg * GRAVITY * thrust_to_weight / 4.0
}

/// Yaw rate P, hand-tuned on `quad_x`: prop drag gives yaw about 7 rad/s² per unit moment,
/// and P 1.5 rises in 0.16 s on a 0.5 rad/s step without clipping (#347).
const YAW_RATE_P: f32 = 1.5;

/// Heading P, hand-tuned on `quad_x`: a 10° heading step without overshoot or clipping on the
/// [`YAW_RATE_P`] loop (#347).
const HEADING_P: f32 = 3.5;

/// The gains an airframe's stabilizer differs from the core defaults by.
struct Tune {
    roll_rate: Gains,
    pitch_rate: Gains,
    /// Roll and pitch angle P.
    angle_p: f32,
    yaw_rate_p: f32,
    heading_p: f32,
}

/// Roll or pitch rate gains `p` and `d`, with I at the autotune's share of P
/// ([`RATE_I_PER_P`]).
const fn rate_gains(p: f32, d: f32) -> Gains {
    Gains {
        p,
        i: RATE_I_PER_P * p,
        d,
    }
}

/// `model` on the X-quad mixer, flown by core's configs with `tune`'s gains and the hover
/// thrust worked out from the model.
fn build(model: MultirotorModel, tune: Tune) -> Airframe {
    build_with(
        model,
        TableMixer::quad_x(),
        MixerConfig::default(),
        CopterStabilizerConfig::default().thrust_curve_exponent,
        tune,
    )
}

/// As [`build`], on `mixer` with `mixer_config`, with the stabilizer's
/// `thrust_curve_exponent`.
fn build_with(
    model: MultirotorModel,
    mixer: TableMixer<4>,
    mixer_config: MixerConfig,
    thrust_curve_exponent: f32,
    tune: Tune,
) -> Airframe {
    let mixer = mixer
        .with_config(mixer_config)
        .unwrap_or_else(|e| unreachable!("airframe mixer configs are in range: {e}"));
    let defaults = CopterStabilizerConfig {
        thrust_curve_exponent,
        ..CopterStabilizerConfig::default()
    };
    let angle = Gains {
        p: tune.angle_p,
        ..defaults.attitude.roll_gains
    };
    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: tune.roll_rate,
            pitch_gains: tune.pitch_rate,
            yaw_gains: Gains {
                p: tune.yaw_rate_p,
                ..defaults.body_rate.yaw_gains
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
                p: tune.heading_p,
                i: 0.0,
                d: 0.0,
            },
            ..defaults.yaw
        },
        ..defaults
    };
    let guidance = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            hover_thrust: hover_thrust(
                model.mass_kg,
                model.max_thrust_n,
                &mixer,
                &mixer_config,
                model.thrust_exponent,
                stabilizer.thrust_curve_exponent,
            ),
            ..CopterGuidanceConfig::default().altitude
        },
        ..CopterGuidanceConfig::default()
    };
    Airframe {
        model,
        mixer,
        guidance,
        stabilizer,
    }
}

/// The linear thrust setpoint that holds `mass_kg` level in a hover on `mixer`'s motors, each
/// `max_thrust_n` at full command: the motor command whose thrust carries the weight
/// (`max_thrust_n · command^thrust_exponent`), undone through the mixer's thrust factor
/// (`mixer_config`, which `TableMixer` doesn't expose) and the stabilizer's
/// `thrust_curve_exponent`.
fn hover_thrust<const N: usize>(
    mass_kg: f32,
    max_thrust_n: f32,
    mixer: &TableMixer<N>,
    mixer_config: &MixerConfig,
    thrust_exponent: f32,
    thrust_curve_exponent: f32,
) -> f32 {
    let motor_n = mass_kg * GRAVITY / motor_count(mixer);
    thrust_for(
        motor_n,
        max_thrust_n,
        mixer_config.thrust_factor,
        thrust_exponent,
        thrust_curve_exponent,
    )
}

/// The linear thrust setpoint that runs each motor at `motor_n` of `max_thrust_n`, undoing
/// `max_thrust_n · command^thrust_exponent`, the `thrust_factor` the mixer scales those
/// motors by (1 for a pusher), and the stabilizer's `thrust_curve_exponent`.
fn thrust_for(
    motor_n: f32,
    max_thrust_n: f32,
    thrust_factor: f32,
    thrust_exponent: f32,
    thrust_curve_exponent: f32,
) -> f32 {
    let fraction = motor_n / max_thrust_n;
    // A square root exactly when quadratic, so the quadratic airframes keep their values.
    let motor_command = if thrust_exponent == QUADRATIC {
        libm::sqrtf(fraction)
    } else {
        libm::powf(fraction, 1.0 / thrust_exponent)
    };
    libm::powf(motor_command / thrust_factor, 1.0 / thrust_curve_exponent)
}

/// How many motors `mixer` drives, not counting pushers.
fn motor_count<const N: usize>(mixer: &TableMixer<N>) -> f32 {
    mixer
        .rows()
        .iter()
        .filter(|row| matches!(row.slot(), Slot::Motor(_)) && !row.is_pusher())
        .count() as f32
}
