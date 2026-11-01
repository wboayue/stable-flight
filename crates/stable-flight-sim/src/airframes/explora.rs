//! The simulated eXplora: see [`explora`].

use core::f32::consts::PI;

use stable_flight::GRAVITY;
use stable_flight::controllers::{Gains, PlaneRateControllerConfig, RateGains};
use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use stable_flight::mixers::MixerConfig;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, DEFAULT_GYRO_CUTOFF_HZ};
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::TailsitterStabilizerConfig;
use stable_flight::{Vec2, Vec3};

use super::plane::{aerosonde_inertia, plane};
use super::tailsitter::{TailsitterAirframe, tailsitter};
use super::{hover_thrust, motor_count};
use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::motors::QUADRATIC;
use crate::{PropWash, TailsitterModel};

/// The RealFlight eXplora VTOL (`eXplora_Tailsiter_VTOL.RFX`) as flown in RealFlight: the
/// tailsitter's layout ([`tailsitter`]: two motors, elevons in their wash) sized from the
/// model's `.rfvehicle`, with its plane side flown on `explora_vtol()`'s settings
/// (stable-flight-sitl-realflight), so its rate gains are tuned here before RealFlight.
///
/// From the `.rfvehicle`:
/// - Wing: 1.40 m span (0.105 m center section and 0.594 m panels each side), 0.43 m², a
///   NACA 0014 section (no camber: no lift at zero angle of attack).
/// - Elevons: ±45°.
/// - Mass: its components weigh 0.217 kg, the placeholder values the BOM variant replaces,
///   plus a 3S 4000 mAh pack and two 1.83 oz motors: about 0.63 kg. The light wing behind
///   ±45° elevons is why its rates run far past what PX4's default feedforward expects (#342).
/// - Inertia: none listed, only modifiers (roll 60%); scaled from the Aerosonde's here, then
///   calibrated per axis (below).
/// - Props: two 10 × 4.5", kv1000 on 3S, about 21 m/s of pitch speed at full throttle, 0.325 m
///   out each side, so the wash covers the elevons' inboard part.
/// - Winglets: two small fins, which turn the wing into the air and damp its yaw far less than
///   the Aerosonde's tail.
///
/// Measured in RealFlight and matched here (#342, run 0): hover at thrust 0.82, cruise at 0.85
/// and 14–15 m/s, a level nose-up moment of 0.09, and default-gain doublets at 30°/s peaking
/// near 3.3× in roll and pitch.
///
/// Its hover is calibrated to RealFlight's autotune of it (#539), flown with RealFlight's 6 ms
/// exchange as [`actuator_delay_s`](crate::Perturbations::actuator_delay_s) and 0.01 rad/s of
/// gyro noise. The hand tune's and the autotuned gains' step tests end within 0.1 of their
/// finals there and rise within a factor of 2 of their times (half within 20%), and the design
/// search designs within 25% of RealFlight's gains (roll rate 0.83 against 0.94, pitch rate
/// 1.33 against 1.69). Its rates fit as RealFlight's do, in the hover view: differential thrust
/// (roll) on 9 ticks of pure delay and no lag, the elevons (pitch) on 2.
///
/// # Examples
///
/// ```
/// use stable_flight_sim::{SimTailsitter, airframes};
///
/// let airframe = airframes::explora();
/// SimTailsitter::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// ```
pub fn explora() -> TailsitterAirframe {
    const MASS_KG: f32 = 0.63;
    const SPAN_M: f32 = 1.40;
    const WING_AREA_M2: f32 = 0.43;
    /// RealFlight's roll inertia modifier, `RollIntertia` 60 (%).
    const ROLL_INERTIA_SHARE: f32 = 0.6;
    /// Motors' distance from the centerline, from the `.rfvehicle`, in m.
    const MOTOR_ARM_M: f32 = 0.325;
    const PROP_DIAMETER_M: f32 = 0.254;
    /// Hover and cruise thrust setpoints measured in RealFlight (#124, #342).
    const HOVER_THRUST: f32 = 0.82;
    const CRUISE_THRUST: f32 = 0.85;
    /// Full-throttle pitch speed of a 4.5" pitch prop at 1000 rpm/V on 11.1 V, in m/s: thrust
    /// falls to zero there.
    const PITCH_SPEED_MPS: f32 = 0.114 * 1000.0 * 11.1 / 60.0;
    const TRIM_AIRSPEED_MPS: f32 = 14.0;
    let base = tailsitter();
    let airplane = plane().model.aero;
    let mixer = base.mixer;
    let mixer_config = MixerConfig::default();
    let thrust_curve_exponent = base.stabilizer.hover.thrust_curve_exponent;
    // How much heavier than the Aerosonde's scaled inertia the eXplora answers its controls in
    // RealFlight, fitted on its hover (roll, pitch, yaw in the airplane frame): its hand tune's rate steps there
    // end at 0.24–0.36 (view yaw, the elevons apart), 0.80 (view pitch), and 0.55 (view roll,
    // differential thrust) of their command, and here at 0.30–0.39, 0.76–0.78, and 0.60
    // (#539). Scaling inertia keeps the controls' authority against the wing's own moments;
    // less wash over the elevons in its place matched the rate steps, but a 5° tilt's drift
    // then pushed the angle steps 10–20% past their target, which RealFlight's don't. The plane
    // side flies on it too; its surfaces are calibrated with it.
    let inertia_scale = Vec3::new(70.0, 11.1, 7.0);
    let mut inertia_kgm2 = aerosonde_inertia(MASS_KG, SPAN_M).component_mul(&inertia_scale);
    inertia_kgm2.x *= ROLL_INERTIA_SHARE;
    // Each motor's static thrust, so hover lands on the measured setpoint: the setpoint through
    // the stabilizer's curve and the mixer's thrust factor is the motor command, and the
    // quadratic motor gives the command squared of its full thrust.
    let hover_command =
        libm::powf(HOVER_THRUST, thrust_curve_exponent) * mixer_config.thrust_factor;
    let hover_fraction = hover_command * hover_command;
    let max_thrust_n = MASS_KG * GRAVITY / motor_count(&mixer) / hover_fraction;
    let wing = FixedWingModel {
        mass_kg: MASS_KG,
        inertia_kgm2,
        wing_area_m2: WING_AREA_M2,
        span_m: SPAN_M,
        chord_m: WING_AREA_M2 / SPAN_M,
        aero: AeroCoefficients {
            c_lift_0: 0.0,
            // Level flight at 14 m/s takes thrust 0.76, as in RealFlight (#342, run 0).
            c_drag_0: 0.05,
            // Calibrated to RealFlight (#342, run 0): a level nose-up moment of 0.09 at
            // 14 m/s.
            c_pitch_0: -0.25,
            // Calibrated to RealFlight (#342, run 0): default-gain 30°/s doublets peak at
            // 3.2× (roll) and 3.4× (pitch) in their first half, here 3.18× and 3.37×.
            c_roll_aileron: 2.4 * base.model.wing.aero.c_roll_aileron,
            c_pitch_elevator: 3.55 * base.model.wing.aero.c_pitch_elevator,
            // The winglets alone: about 0.036 m² of fin 0.15 m behind the center on a 0.43 m²,
            // 1.4 m wing, at a fin lift slope near 2/rad. The Aerosonde's tail turns the wing
            // into a 5° tilt's drift hard enough to push its hover's angle steps 15% past
            // their target, where RealFlight's settle on it (#539).
            c_yaw_beta: 0.02,
            // The airplane's side force and dihedral, where its plane side was fitted (#539):
            // the Zagi's from `tailsitter()` lose 6.1 m in a 90°/s roll, past
            // `tests/maneuver.rs`'s 4.8 m band.
            c_side_beta: airplane.c_side_beta,
            c_roll_beta: airplane.c_roll_beta,
            // The same fins' damping and the wing's profile drag: with the Aerosonde's, the
            // wash damped differential thrust's rate steps in a hover to two-thirds of
            // RealFlight's (#539).
            c_yaw_r: -0.02,
            ..base.model.wing.aero
        },
        max_deflection_rad: base.model.wing.max_deflection_rad.map(|_| PI / 4.0),
        // The elevon servos' speed is 0.01 in the `.rfvehicle`, and RealFlight's fit reads no
        // lag on them (#539).
        servo_time_constant_s: 0.004,
        // RealFlight's differential thrust fits as 9–10 ticks of pure delay and a 2 ms lag
        // (#539): with its 6 ms exchange and the one-tick hold, 28 ms of spin-up.
        motor_time_constant_s: 0.004,
        motor_delay_s: 0.028,
        max_thrust_n,
        thrust_exponent: QUADRATIC,
        thrust_falloff_n_per_mps: max_thrust_n / PITCH_SPEED_MPS,
        ..base.model.wing
    };
    let model = TailsitterModel {
        wing,
        motor_arm_m: MOTOR_ARM_M,
        // A fifth of the wake's pressure over the elevons, with the inertia above, flies the
        // elevon rate steps as RealFlight does; the wash damps the span it covers.
        prop_wash: PropWash::props(
            2,
            PROP_DIAMETER_M,
            0.2,
            PropWash::span_damping_share(MOTOR_ARM_M, PROP_DIAMETER_M, SPAN_M),
        ),
    };
    let plane_stabilizer = PlaneStabilizerConfig {
        rate: explora_rate(),
        thrust_curve_exponent,
        trim_airspeed_mps: TRIM_AIRSPEED_MPS,
        min_airspeed_mps: 9.0,
        max_airspeed_mps: 25.0,
        ..PlaneStabilizerConfig::default()
    };
    let mut guidance = base.guidance;
    // The model's own drag in a hover, fitted on the steady lean in 0.5–2 m/s winds (past
    // 2.5 m/s across the belly it leans on its tilt cap): bluff, k = ½ρ/B, 0.84/m across the
    // belly and 0.56/m along the span (#609).
    guidance.hover.wind.ballistic_kgpm2 = Vec2::new(0.73, 1.1);
    guidance.hover.altitude.hover_thrust = hover_thrust(
        MASS_KG,
        max_thrust_n,
        &mixer,
        &mixer_config,
        QUADRATIC,
        thrust_curve_exponent,
    );
    guidance.plane = PlaneGuidanceConfig {
        cruise_thrust: CRUISE_THRUST,
        tecs: TecsConfig {
            min_airspeed_mps: plane_stabilizer.min_airspeed_mps,
            max_airspeed_mps: plane_stabilizer.max_airspeed_mps,
            ..TecsConfig::default()
        },
        ..PlaneGuidanceConfig::default()
    };
    guidance.transition.min_airspeed_mps = plane_stabilizer.min_airspeed_mps;
    TailsitterAirframe {
        model,
        mixer,
        guidance,
        stabilizer: TailsitterStabilizerConfig {
            hover: explora_hover(base.stabilizer.hover),
            plane: plane_stabilizer,
        },
    }
}

/// The eXplora's hover gains, as `explora_vtol()` flies them: RealFlight's autotune of its
/// hover at the default 40 Hz gyro filter (#539).
fn explora_hover(hover: CopterStabilizerConfig) -> CopterStabilizerConfig {
    let gains = |p, i, d| Gains { p, i, d };
    let mut hover = CopterStabilizerConfig {
        gyro_cutoff_hz: DEFAULT_GYRO_CUTOFF_HZ,
        ..hover
    };
    hover.body_rate.roll_gains = gains(0.6504, 0.1301, 0.0017);
    hover.body_rate.pitch_gains = gains(1.4217, 0.2844, 0.0);
    hover.body_rate.yaw_gains = gains(1.0191, 0.3057, 0.0);
    hover.attitude.roll_gains = gains(2.3554, 0.0, 0.0);
    hover.attitude.pitch_gains = gains(9.0, 0.0, 0.0);
    hover.body_rate.max_integral = 0.28;
    hover.body_rate.yaw_max_integral = 0.31;
    hover
}

/// The eXplora's plane rate loop, as `explora_vtol()` flies it (#342): feedforward sized to
/// its elevons, which turn a unit moment into several times the rate PX4's defaults expect.
fn explora_rate() -> PlaneRateControllerConfig {
    let defaults = PlaneRateControllerConfig::default();
    PlaneRateControllerConfig {
        roll: RateGains {
            ff: 0.1,
            ..defaults.roll
        },
        pitch: RateGains {
            i: 0.8,
            ff: 0.05,
            ..defaults.pitch
        },
        trim: Vec3::new(0.0, 0.09, 0.0),
        ..defaults
    }
}
