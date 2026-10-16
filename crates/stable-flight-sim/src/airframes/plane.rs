//! The simulated airplane: see [`plane`].

use stable_flight::controllers::{PlaneRateControllerConfig, RateGains};
use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::mixers::{MixerConfig, Slot, TableMixer};
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::{GRAVITY, Vec3};

use crate::aero;
use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::start::Start;

use super::{motor_count, thrust_for};
use crate::motors::QUADRATIC;

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
/// The stabilizer and guidance are core's defaults at a 15 m/s trim airspeed, except for
/// what the model works out rather than a flight measures: each rate loop's feedforward (the
/// surface command per rad/s that balances the model's rate damping at trim, so a commanded
/// rate needs no feedback and a turn gets the rudder that coordinates it) and the cruise
/// thrust from its level-flight drag. Pitch rate I is tuned to the plane step spec in
/// `tests/` (#348).
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
    const PITCH_RATE_I: f32 = 1.5;
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
        thrust_exponent: QUADRATIC,
        thrust_falloff_n_per_mps: 0.0,
        thrust_line_below_cg_m: 0.0,
        motor_time_constant_s: 0.05,
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    let mixer = TableMixer::plane();
    let feedforward = rate_feedforward(&model, TRIM_AIRSPEED_MPS);
    let defaults = PlaneRateControllerConfig::default();
    let rate = PlaneRateControllerConfig {
        roll: RateGains {
            ff: feedforward.x,
            ..defaults.roll
        },
        // A held pitch at constant throttle trades airspeed for climb, so the elevator it
        // needs keeps moving after the step: PX4's I lets the attitude droop back past the
        // settle band; 1.5 settles a 10° step in 0.7 s (#348).
        pitch: RateGains {
            i: PITCH_RATE_I,
            ff: feedforward.y,
            ..defaults.pitch
        },
        yaw: RateGains {
            ff: feedforward.z,
            ..defaults.yaw
        },
        ..defaults
    };
    let stabilizer = PlaneStabilizerConfig {
        rate,
        trim_airspeed_mps: TRIM_AIRSPEED_MPS,
        ..PlaneStabilizerConfig::default()
    };
    let guidance = PlaneGuidanceConfig {
        cruise_thrust: cruise_thrust(
            &model,
            &mixer,
            &MixerConfig::default(),
            stabilizer.thrust_curve_exponent,
            TRIM_AIRSPEED_MPS,
        ),
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
pub(super) fn aerosonde_inertia(mass_kg: f32, span_m: f32) -> Vec3 {
    const MASS_KG: f32 = 13.5;
    const SPAN_M: f32 = 2.8956;
    let scale = mass_kg / MASS_KG * (span_m / SPAN_M) * (span_m / SPAN_M);
    Vec3::new(0.8244, 1.135, 1.759) * scale
}

/// The surface command per rad/s that holds a steady roll, pitch, and yaw rate on `model` at
/// `airspeed_mps`: each axis's rate damping (`c_roll_p`, `c_pitch_q`, `c_yaw_r`, over the
/// 2V/length that nondimensionalizes the rate) balanced by its surface at full deflection.
///
/// A unit moment command is taken as full deflection, as [`TableMixer::plane`]'s unit-weight
/// servo rows give it. The stabilizer scales feedforward by trim over measured airspeed, which
/// keeps it right off trim: the deflection a rate needs falls as 1/V too.
pub(super) fn rate_feedforward(model: &FixedWingModel, airspeed_mps: f32) -> Vec3 {
    let c = &model.aero;
    let per_rate = |damping: f32, length_m: f32, control: f32, max_deflection_rad: f32| {
        -damping * length_m / (2.0 * airspeed_mps * control * max_deflection_rad)
    };
    let max = model.max_deflection_rad;
    Vec3::new(
        per_rate(c.c_roll_p, model.span_m, c.c_roll_aileron, max.x),
        per_rate(c.c_pitch_q, model.chord_m, c.c_pitch_elevator, max.y),
        per_rate(c.c_yaw_r, model.span_m, c.c_yaw_rudder, max.z),
    )
}

/// The throttle that holds `model` in level flight at `airspeed_mps` on `mixer`'s forward
/// motors (its pushers, or with none its motors): the thrust that matches the drag at the angle
/// of attack whose linear lift carries the weight, shared by those motors, through
/// [`thrust_for`] with the stabilizer's `thrust_curve_exponent` and the mixer's thrust factor
/// (`mixer_config`) for motors; pushers span their full range.
pub(super) fn cruise_thrust<const N: usize>(
    model: &FixedWingModel,
    mixer: &TableMixer<N>,
    mixer_config: &MixerConfig,
    thrust_curve_exponent: f32,
    airspeed_mps: f32,
) -> f32 {
    let pressure_area_n = aero::pressure_area_n(model, airspeed_mps);
    let lift = model.mass_kg * GRAVITY / pressure_area_n;
    let drag_n = pressure_area_n * aero::drag_coefficient(model, lift);
    let (count, thrust_factor) = forward_motors(mixer, mixer_config);
    thrust_for(
        drag_n / count,
        model.full_thrust_n(airspeed_mps),
        thrust_factor,
        model.thrust_exponent,
        thrust_curve_exponent,
    )
}

/// How many of `mixer`'s motors push the airframe forward, and the factor the mixer scales
/// them by: its pushers at full range, or with none (a plane, a tailsitter) its motors at
/// the thrust factor.
fn forward_motors<const N: usize>(mixer: &TableMixer<N>, mixer_config: &MixerConfig) -> (f32, f32) {
    let pushers = mixer
        .rows()
        .iter()
        .filter(|row| matches!(row.slot(), Slot::Motor(_)) && row.is_pusher())
        .count();
    if pushers > 0 {
        (pushers as f32, 1.0)
    } else {
        (motor_count(mixer), mixer_config.thrust_factor)
    }
}
