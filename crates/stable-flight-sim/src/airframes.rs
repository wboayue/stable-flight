//! Simulated airframes: physics paired with the mixer and flight-controller configs that fly
//! them.

use stable_flight::GRAVITY;
use stable_flight::Vec3;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::{MixerConfig, Slot, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::model::MultirotorModel;

mod plane;
mod quadplane;
mod tailsitter;
#[cfg(test)]
mod tests;

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
/// Roll and pitch rate and angle gains are the sim autotune's (`StagePlan::copter_search`,
/// `tests/autotune.rs`); yaw rate and heading are hand-tuned on the sim's step responses
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
            // Sim autotune (#347); the symmetric model's roll and pitch differ by the search's
            // step size.
            roll_rate: Gains {
                p: 1.95,
                i: 0.0,
                d: 0.0288,
            },
            pitch_rate: Gains {
                p: 1.70,
                i: 0.0,
                d: 0.0258,
            },
            // Sim autotune (#347).
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
            // Sim autotune from core's defaults (#366).
            roll_rate: Gains {
                p: 0.816,
                i: 0.0,
                d: 0.0209,
            },
            pitch_rate: Gains {
                p: 0.795,
                i: 0.0,
                d: 0.0200,
            },
            angle_p: 9.0,
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
            // Sim autotune from core's defaults (#366).
            roll_rate: Gains {
                p: 0.621,
                i: 0.0,
                d: 0.00446,
            },
            pitch_rate: Gains {
                p: 0.598,
                i: 0.0,
                d: 0.00439,
            },
            angle_p: 9.0,
            // `quad_x`'s: they hold heading in a hover; the autotune doesn't tune them.
            yaw_rate_p: YAW_RATE_P,
            heading_p: HEADING_P,
        },
    )
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

/// `model` on the X-quad mixer, flown by core's configs with `tune`'s gains and the hover
/// thrust worked out from the model.
fn build(model: MultirotorModel, tune: Tune) -> Airframe {
    let mixer = TableMixer::quad_x();
    let defaults = CopterStabilizerConfig::default();
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
                &MixerConfig::default(),
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
/// (`max_thrust_n · command²`), undone through the mixer's thrust factor (`mixer_config`,
/// which `TableMixer` doesn't expose) and the stabilizer's `thrust_curve_exponent`.
fn hover_thrust<const N: usize>(
    mass_kg: f32,
    max_thrust_n: f32,
    mixer: &TableMixer<N>,
    mixer_config: &MixerConfig,
    thrust_curve_exponent: f32,
) -> f32 {
    let motor_n = mass_kg * GRAVITY / motor_count(mixer);
    thrust_for(motor_n, max_thrust_n, mixer_config, thrust_curve_exponent)
}

/// The linear thrust setpoint that runs each motor at `motor_n` of `max_thrust_n`, undoing
/// `max_thrust_n · command²`, the mixer's thrust factor, and the stabilizer's
/// `thrust_curve_exponent`.
fn thrust_for(
    motor_n: f32,
    max_thrust_n: f32,
    mixer_config: &MixerConfig,
    thrust_curve_exponent: f32,
) -> f32 {
    let motor_command = libm::sqrtf(motor_n / max_thrust_n);
    libm::powf(
        motor_command / mixer_config.thrust_factor,
        1.0 / thrust_curve_exponent,
    )
}

/// How many motors `mixer` drives, not counting pushers.
fn motor_count<const N: usize>(mixer: &TableMixer<N>) -> f32 {
    mixer
        .rows()
        .iter()
        .filter(|row| matches!(row.slot(), Slot::Motor(_)) && !row.is_pusher())
        .count() as f32
}
