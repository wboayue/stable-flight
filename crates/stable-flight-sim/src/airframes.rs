//! Simulated airframes: physics paired with the mixer and flight-controller configs that fly
//! them.

use stable_flight::GRAVITY;
use stable_flight::Vec3;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::mixers::{MixerConfig, Slot, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;

use crate::aero::AIR_DENSITY_KGPM3;
use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::lockstep::Start;
use crate::model::MultirotorModel;

#[cfg(test)]
mod tests;

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
            hover_thrust: hover_thrust(&model, &mixer, &MixerConfig::default(), &stabilizer),
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

/// The linear thrust setpoint that holds `model` level in a hover on `mixer`'s motors: the
/// motor command whose thrust carries the weight (`max_thrust_n · command²`), undone through
/// the mixer's thrust factor (`mixer_config`, which `TableMixer` doesn't expose) and the
/// stabilizer's thrust curve.
fn hover_thrust<const N: usize>(
    model: &MultirotorModel,
    mixer: &TableMixer<N>,
    mixer_config: &MixerConfig,
    stabilizer: &CopterStabilizerConfig,
) -> f32 {
    let motors = mixer
        .rows()
        .iter()
        .filter(|row| matches!(row.slot(), Slot::Motor(_)))
        .count();
    let motor_n = model.mass_kg * GRAVITY / motors as f32;
    let motor_command = libm::sqrtf(motor_n / model.max_thrust_n);
    libm::powf(
        motor_command / mixer_config.thrust_factor,
        1.0 / stabilizer.thrust_curve_exponent,
    )
}

/// A simulated airplane: its physics, the mixer that drives its prop and surfaces, the
/// guidance and stabilizer configs tuned for it, and a start in flight.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneAirframe {
    /// Physical parameters.
    pub model: FixedWingModel,
    /// The mixer; [`SimPlane`](crate::SimPlane) reads the prop and surfaces from its rows.
    pub mixer: TableMixer<4>,
    /// Guidance configuration.
    pub guidance: PlaneGuidanceConfig,
    /// Stabilizer configuration.
    pub stabilizer: PlaneStabilizerConfig,
    /// Level at trim airspeed, 100 m up over the origin, heading north.
    pub start: Start,
}

/// ArduPilot SITL's default airplane (`libraries/SITL/SIM_Plane.h:44-88`,
/// `SIM_Plane.cpp:30`, Plane-4.5): 2 kg, 1.88 m span, 0.45 m² wing, its aerodynamic
/// coefficients and ±20° surfaces, on [`TableMixer::plane`].
///
/// The rest isn't in ArduPilot's model and is chosen:
/// - Inertia: ArduPilot applies the aero moment as the angular acceleration, so its model has
///   none; this one scales the Aerosonde's (Beard and McLain, *Small Unmanned Aircraft*,
///   app. E: 13.5 kg, 2.9 m span) by mass × span².
/// - Servo lag: ArduPilot SITL's default `SIM_SERVO_SPEED` (0.14 s), which it applies as a
///   first-order filter (`SITL.cpp:87`, `SIM_Aircraft.cpp:905-917`).
/// - Prop: static thrust-to-weight 1.2 with 50 ms lag; ArduPilot sizes its thrust from a hover
///   throttle that doesn't apply here.
///
/// The stabilizer and guidance are core's defaults at a 15 m/s trim airspeed, with the
/// cruise thrust worked out from the model's level-flight drag rather than measured.
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::plane::PlaneGuidance;
/// use stable_flight::stabilizers::plane::PlaneStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimPlane, airframes};
///
/// let airframe = airframes::plane();
/// let vehicle = SimPlane::new(airframe.model, &airframe.mixer)
///     .expect("valid airframe")
///     .with_start(airframe.start);
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     PlaneGuidance::new(airframe.guidance),
///     PlaneStabilizer::new(airframe.stabilizer),
///     airframe.mixer,
///     RecordingTelemetry::<16>::new(),
/// );
/// ```
pub fn plane() -> PlaneAirframe {
    const MASS_KG: f32 = 2.0;
    const SPAN_M: f32 = 1.88;
    const TRIM_AIRSPEED_MPS: f32 = 15.0;
    let model = FixedWingModel {
        mass_kg: MASS_KG,
        inertia_kgm2: aerosonde_inertia(MASS_KG, SPAN_M),
        wing_area_m2: 0.45,
        span_m: SPAN_M,
        chord_m: 0.24,
        aero: AeroCoefficients {
            c_lift_0: 0.56,
            c_lift_alpha: 6.9,
            alpha_stall_rad: 0.4712,
            stall_sharpness: 50.0,
            c_drag_0: 0.1,
            oswald: 0.9,
            c_side_beta: -0.98,
            c_side_rudder: -0.2,
            c_roll_beta: -0.12,
            c_roll_p: -1.0,
            c_roll_r: 0.14,
            c_roll_aileron: 0.25,
            c_roll_rudder: -0.037,
            c_pitch_0: 0.045,
            c_pitch_alpha: -0.7,
            c_pitch_q: -20.0,
            c_pitch_elevator: 1.0,
            c_yaw_beta: 0.25,
            c_yaw_p: 0.022,
            c_yaw_r: -1.0,
            c_yaw_rudder: 0.1,
        },
        max_deflection_rad: Vec3::repeat(0.3491),
        servo_time_constant_s: 0.14,
        max_thrust_n: 1.2 * MASS_KG * GRAVITY,
        motor_time_constant_s: 0.05,
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    let mixer = TableMixer::plane();
    let stabilizer = PlaneStabilizerConfig {
        trim_airspeed_mps: TRIM_AIRSPEED_MPS,
        ..PlaneStabilizerConfig::default()
    };
    let guidance = PlaneGuidanceConfig {
        cruise_thrust: cruise_thrust(&model, &MixerConfig::default(), TRIM_AIRSPEED_MPS),
        ..PlaneGuidanceConfig::default()
    };
    PlaneAirframe {
        model,
        mixer,
        guidance,
        stabilizer,
        start: Start::level(100.0, TRIM_AIRSPEED_MPS, 0.0),
    }
}

/// The Aerosonde's principal inertia (Beard and McLain, *Small Unmanned Aircraft*, app. E:
/// 0.8244, 1.135, 1.759 kg·m² at 13.5 kg and 2.8956 m span), scaled to `mass_kg` and
/// `span_m` by mass × span².
fn aerosonde_inertia(mass_kg: f32, span_m: f32) -> Vec3 {
    const MASS_KG: f32 = 13.5;
    const SPAN_M: f32 = 2.8956;
    let scale = mass_kg / MASS_KG * (span_m / SPAN_M) * (span_m / SPAN_M);
    Vec3::new(0.8244, 1.135, 1.759) * scale
}

/// The throttle that holds `model` in level flight at `airspeed_mps`: the thrust that matches
/// the drag at the angle of attack whose linear lift carries the weight, through the prop's
/// `max_thrust_n · command²` and the mixer's thrust factor (`mixer_config`). Throttle passes
/// through the stabilizer unchanged (thrust curve exponent 1).
fn cruise_thrust(model: &FixedWingModel, mixer_config: &MixerConfig, airspeed_mps: f32) -> f32 {
    let c = &model.aero;
    let pressure_area_n =
        0.5 * AIR_DENSITY_KGPM3 * airspeed_mps * airspeed_mps * model.wing_area_m2;
    let lift = model.mass_kg * GRAVITY / pressure_area_n;
    let aspect_ratio = model.span_m * model.span_m / model.wing_area_m2;
    let drag = c.c_drag_0 + lift * lift / (core::f32::consts::PI * c.oswald * aspect_ratio);
    let command = libm::sqrtf(pressure_area_n * drag / model.max_thrust_n);
    command / mixer_config.thrust_factor
}
