//! The simulated airplane: see [`plane`].

use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::{GRAVITY, Vec3};

use crate::aero;
use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::start::Start;

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
    let pressure_area_n = aero::pressure_area_n(model, airspeed_mps);
    let lift = model.mass_kg * GRAVITY / pressure_area_n;
    let drag = aero::drag_coefficient(model, lift);
    let command = libm::sqrtf(pressure_area_n * drag / model.max_thrust_n);
    command / mixer_config.thrust_factor
}
