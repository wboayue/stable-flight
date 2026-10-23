//! The simulated Alia 250 lift+cruise eVTOL, wing and all: see [`alia250`].

use stable_flight::controllers::{PlaneRateControllerConfig, RateGains};
use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use stable_flight::guidance::vtol::{
    ForwardThrustConfig, LiftTransitionConfig, OffNoseSpeedConfig, QuadPlaneGuidanceConfig,
};
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizerConfig;
use stable_flight::{Vec2, Vec3};

use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::quadplane_model::{LiftRotors, QuadPlaneModel};
use crate::start::Start;

use super::plane::{cruise_thrust, rate_feedforward};
use super::{QuadPlaneAirframe, alia250_hover};

#[cfg(test)]
mod tests;

/// Wing span, in m: the `.acf`'s main wing (wings 0–5, 25.8 ft a side).
pub(super) const SPAN_M: f32 = 15.7;
/// Wing area, in m²: the `.acf`'s main wing (210 ft²).
pub(super) const WING_AREA_M2: f32 = 19.5;
/// Mean aerodynamic chord, in m: area over span.
pub(super) const CHORD_M: f32 = WING_AREA_M2 / SPAN_M;

/// The airspeed the plane side's gains are set at, in m/s: the middle of the measured level
/// band (41–48 m/s).
const TRIM_AIRSPEED_MPS: f32 = 45.0;

/// The plane side's least airspeed, in m/s: about where the wing holds level flight (X-Plane's
/// slowest level leg, 41 m/s). The forward transition's gate too: handed over at 37.5 m/s it
/// lost 37 m diving for speed, and 296 m once the pitch loop tracked (TECS pulled it into a
/// stall); at 40 m/s it hands over at 43.7 m/s and loses 12 m (#392).
const MIN_AIRSPEED_MPS: f32 = 40.0;

/// Pitch up the plane side asks at most, in rad: below the stall. Full elevator trims the wing
/// at 17° α, and at 18° of pitch a sinking entry held full elevator and fell into a stall cycle
/// to the ground; at 15° it enters within 21 m even at pitch rate P 1.5 and TECS pitch damping
/// 1.0 (#392).
const MAX_PITCH_UP_RAD: f32 = 15.0 * core::f32::consts::PI / 180.0;

/// The BETA Technologies Alia 250 as X-Plane 12 flies it, hover and wing: its lift rotors as
/// [`alia250_hover`] has them, its wing and pusher fitted to X-Plane (#392). Mass and the
/// effective inertia are the hover model's (2,857 kg; 8,570, 14,300, 6,250 kg·m²).
///
/// **Measured** (X-Plane, Oceano, `plane-probe-xplane` and a full-power lap, 2,857 kg):
///
/// | Quantity | Value | From |
/// |---|---|---|
/// | Lift | `CL = 0.829 + 2.81 α` (α body, rad) | least squares over 18 cycle-averaged level windows, 41–48 m/s, α 3.5–10.2°, at this sim's air density (1.225 kg/m³) |
/// | Drag | `CD = 0.084 + CL²/(π · 0.8 · 12.6)` | the same windows, CD 0.115–0.143 |
/// | Pusher | `command · (5,100 − 21 · u)` N | X-Plane's thrust at full command, 5.1 kN near 0 to 4.1 kN at 46 m/s; 2.5 kN static at half |
/// | Pitch | `Cm = 0.068 − 1.48 α + 1.09 δe` | least squares of the trim elevator against α (servo 0.02 at 3.5°, 0.53 at 10.2°), with `Cm_δe` from pitch pulses (~45 °/s² per unit command at 45 m/s) |
/// | Roll | `Cl_δa = 0.077`/rad, `Cl_p = −0.38` | roll pulses, ~50 °/s² per unit command at 45 m/s; the steady rate under 24 half-aileron holds (`plane-probe-xplane`, 40–48 m/s), 7–10 °/s, `Cl_p` −0.29 to −0.46, mean −0.39 (right rolls −0.35, left −0.42) |
/// | Yaw | `Cn_δr = 0.014`/rad | yaw pulses, ~25 °/s² per unit command |
///
/// **Not measured:** the other damping and sideslip terms take [`plane`](super::plane)'s (the
/// Aerosonde's) coefficients, `c_pitch_q` set to a larger aircraft's (the lateral-directional
/// ones, #458). The stall is put past the elevator's reach (17°), which the probes never
/// passed. The pusher's thrust line sits on the center of gravity (X-Plane hints at a nose-up
/// moment at full power, unmeasured). The forward transition accelerates through 30–40 m/s
/// slower than X-Plane's: 42 s from a hover to the 40 m/s gate against X-Plane's 27 s, the lift
/// rotors' drag in the airflow likely too high.
///
/// **Controllers:** the hover side is [`alia250_hover`]'s. The plane side is tuned on this
/// model (#392): px4xplane's rate P and I where they hold, the rate feedforward from the model,
/// pitch rate I and P for the wing's pitch stability, TECS's climb and sink from the model,
/// and the transition gate and pitch-up limit set by the wing's speed and stall (each value
/// beside it below). `stable-flight-sitl-xplane`'s `alia250()` flies the same plane side.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::{SimQuadPlane, airframes};
///
/// let airframe = airframes::alia250();
/// assert!(SimQuadPlane::new(airframe.model, &airframe.mixer).is_ok());
/// ```
pub fn alia250() -> QuadPlaneAirframe {
    let hover = alia250_hover();
    let lift = &hover.model;
    let wing = FixedWingModel {
        mass_kg: lift.mass_kg,
        inertia_kgm2: lift.inertia_kgm2,
        wing_area_m2: WING_AREA_M2,
        span_m: SPAN_M,
        chord_m: CHORD_M,
        aero: AeroCoefficients {
            c_lift_0: 0.829,
            c_lift_alpha: 2.81,
            alpha_stall_rad: 17.0f32.to_radians(),
            stall_sharpness: 50.0,
            c_drag_0: 0.084,
            oswald: 0.8,
            c_side_beta: -0.98,
            c_side_rudder: -0.2,
            c_roll_beta: -0.12,
            c_roll_p: -0.38,
            c_roll_r: 0.14,
            c_roll_aileron: 0.077,
            c_roll_rudder: -0.004,
            c_pitch_0: 0.068,
            c_pitch_alpha: -1.48,
            c_pitch_q: -25.0,
            c_pitch_elevator: 1.09,
            c_yaw_beta: 0.1,
            c_yaw_p: 0.022,
            c_yaw_r: -0.15,
            c_yaw_rudder: 0.014,
        },
        // Ailerons 20° up and 10° down (the mean), elevators ±20°, rudders ±30° (`.acf`).
        max_deflection_rad: Vec3::new(
            15.0f32.to_radians(),
            20.0f32.to_radians(),
            30.0f32.to_radians(),
        ),
        // X-Plane's `.acf` surface rates: about 0.25 s to full deflection.
        servo_time_constant_s: 0.1,
        max_thrust_n: 5_100.0,
        thrust_exponent: 1.0,
        thrust_falloff_n_per_mps: 21.0,
        thrust_line_below_cg_m: 0.0,
        // The lift motors' measured lag (#351), for the pusher too.
        motor_time_constant_s: lift.motor_time_constant_s,
        ground_friction: lift.ground_friction,
        max_step_s: lift.max_step_s,
    };
    let model = QuadPlaneModel {
        wing,
        lift: LiftRotors {
            moment_arm_m: lift.moment_arm_m,
            max_thrust_n: lift.max_thrust_n,
            thrust_exponent: lift.thrust_exponent,
            motor_time_constant_s: lift.motor_time_constant_s,
            yaw_torque_per_thrust_m: lift.yaw_torque_per_thrust_m,
            // The hover model's drag on the whole airframe, here only the rotors' share at
            // full thrust; the wing's drag is the aero model's.
            drag_n_per_mps: lift.drag_n_per_mps,
        },
    };
    let mixer = TableMixer::quadplane()
        .with_reversed_yaw()
        .with_config(super::ALIA_MIXER)
        .unwrap_or_else(|e| unreachable!("the Alia's mixer config is in range: {e}"));
    let rate_defaults = PlaneRateControllerConfig::default();
    // The surface each rate needs at trim, from the model's damping and authority, as plane()'s.
    let feedforward = rate_feedforward(&model.wing, TRIM_AIRSPEED_MPS);
    let plane_stabilizer = PlaneStabilizerConfig {
        rate: PlaneRateControllerConfig {
            roll: RateGains {
                p: 0.47,
                i: 0.05,
                ff: feedforward.x,
            },
            // I 0.5 holds the attitude against the wing's pitch stability (Cm_α −1.48):
            // px4xplane's 0.045 held a 5° step at 27% of it after 8 s, and trim 2° short. P 1
            // rises a 5° step in 0.85 s, inside PX4's 2.2τ (0.88 s); px4xplane's 0.52 took 1.03 s.
            pitch: RateGains {
                p: 1.0,
                i: 0.5,
                ff: feedforward.y,
            },
            yaw: RateGains {
                p: 0.1,
                i: 0.12,
                ff: feedforward.z,
            },
            ..rate_defaults
        },
        // Roll: full aileron holds about 17°/s (`Cl_p` from the roll holds), so the attitude
        // loop asks at most 15°/s and the ailerons don't clip; τ 0.6 s rises a 20° bank in
        // 1.19 s, inside its 2.2τ (1.32 s). At PX4's 0.4 s and 70°/s the ailerons clipped
        // 1.2 s and it overshot 15% (#392).
        time_constant_s: Vec2::new(0.6, 0.4),
        max_attitude_rate_radps: Vec2::new(15.0f32.to_radians(), 60.0f32.to_radians()),
        max_bank_rad: 20.0f32.to_radians(),
        trim_airspeed_mps: TRIM_AIRSPEED_MPS,
        min_airspeed_mps: MIN_AIRSPEED_MPS,
        max_airspeed_mps: 60.0,
        ..PlaneStabilizerConfig::default()
    };
    let plane_guidance = PlaneGuidanceConfig {
        max_bank_rad: 20.0f32.to_radians(),
        max_pitch_up_rad: MAX_PITCH_UP_RAD,
        trim_pitch_rad: 7.0f32.to_radians(),
        cruise_thrust: cruise_thrust(
            &model.wing,
            &mixer,
            &super::ALIA_MIXER,
            plane_stabilizer.thrust_curve_exponent,
            TRIM_AIRSPEED_MPS,
        ),
        l1_period_s: 40.0,
        loiter_radius_m: 850.0,
        // The model's steady climb at full thrust and sink at least thrust, at trim airspeed
        // (1.78 and 3.56 m/s); the stabilizer's airspeed range.
        tecs: TecsConfig {
            max_climb_mps: 1.8,
            min_sink_mps: 3.5,
            min_airspeed_mps: plane_stabilizer.min_airspeed_mps,
            max_airspeed_mps: plane_stabilizer.max_airspeed_mps,
            // 2.1 m against 3.4 m past a 30 m descent at ArduPilot's 0.3 (#392).
            pitch_damping: 0.5,
            ..TecsConfig::default()
        },
        ..PlaneGuidanceConfig::default()
    };
    QuadPlaneAirframe {
        model,
        mixer,
        guidance: QuadPlaneGuidanceConfig {
            hover: hover.guidance,
            plane: plane_guidance,
            transition: LiftTransitionConfig {
                min_airspeed_mps: MIN_AIRSPEED_MPS,
                timeout_s: 45.0,
                min_height_m: 20.0,
                // Above px4xplane's `VT_FW_MIN_ALT` (20 m, `config/px4_params/5020_xplane_alia250`),
                // which assumes PX4's instant hover: falling back from a 5 m/s sink at 50 m/s
                // lost 11 m below the floor in X-Plane, while the lift rotors ramp in (#493).
                plane_min_height_m: 30.0,
                back_ramp_s: 10.0,
                back_timeout_s: 28.0,
                pusher_slew_per_s: 0.45,
                ..LiftTransitionConfig::default()
            },
            forward: ForwardThrustConfig::default(),
            off_nose: OffNoseSpeedConfig::default(),
        },
        stabilizer: QuadPlaneStabilizerConfig {
            hover: hover.stabilizer,
            plane: plane_stabilizer,
        },
        start: Start::level(300.0, TRIM_AIRSPEED_MPS, 0.0),
    }
}
