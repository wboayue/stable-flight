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
use crate::fixed_wing::{AeroCoefficients, FixedWingModel, PitchTrimKnee, ThrustFalloff};
use crate::motors::QUADRATIC;
use crate::{PropWash, TailsitterModel};

/// The RealFlight eXplora VTOL (`eXplora_Tailsiter_VTOL.RFX`) as flown in RealFlight: the
/// tailsitter's layout ([`tailsitter`]: two motors, elevons in their wash) sized from the
/// model's `.rfvehicle`, with its hover flown on `explora_vtol()`'s gains
/// (stable-flight-sitl-realflight), its plane side on RealFlight's plane autotune (#535).
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
/// - Props: two 10 × 4.5", kv1000 on 3S, 0.325 m out each side, so the wash covers the
///   elevons' inboard part.
/// - Winglets: two small fins, which turn the wing into the air and damp its yaw far less than
///   the Aerosonde's tail.
///
/// Its thrust is RealFlight's (`docs/findings/explora-thrust.md`): it hovers at setpoint 0.88
/// and flies level at 14 m/s on 0.86, and its prop loses thrust with airspeed faster at part
/// throttle, so its back ramps climb as RealFlight's do. Also matched to RealFlight (#342,
/// run 0): a level nose-up moment of 0.09, and default-gain roll doublets at 30°/s peaking near
/// 3.2×.
///
/// Its cruise is calibrated to RealFlight's plane autotune fits of it (#587, #619), three
/// runs at 14 m/s. Its pitch is a flying wing's, stiff in angle of attack and barely damped:
/// stiffness 61/s², damping 0.8/s, control 87 rad/s², and a path turn of 18.8/s against
/// RealFlight's 69–93, 0–1.3, 80–82, and 18.3. Its roll fits damping 22/s and control
/// 194 rad/s² against RealFlight's 23–24 and 180–184. At 18 m/s (#648) its pitch control fits
/// 126 rad/s² and its path turn 23.1/s against RealFlight's 123 and 23.1, and its trim command
/// falls from 0.090 to 0.073 against RealFlight's 0.089 to 0.074: above trim airspeed its trim
/// moment grows slower than dynamic pressure, as RealFlight's does.
///
/// Its hover is calibrated to RealFlight's autotune of it (#539), flown with RealFlight's 6 ms
/// exchange as [`actuator_delay_s`](crate::Perturbations::actuator_delay_s) and 0.01 rad/s of
/// gyro noise. The hand tune's and the autotuned gains' step tests end within 0.1 of their
/// finals there and rise within a factor of 2 of their times (half within 20%), and the design
/// search designs within 25% of RealFlight's gains (roll rate 0.83 against 0.94, pitch rate
/// 2.09 against 1.69). Its rates fit as RealFlight's do, in the hover view: differential thrust
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
    /// Hover thrust setpoint measured in RealFlight, near still (`docs/findings/explora-thrust.md`).
    const HOVER_THRUST: f32 = 0.88;
    /// `explora_vtol()`'s cruise thrust (#124).
    const CRUISE_THRUST: f32 = 0.85;
    /// Airspeed at which full-command thrust reaches 0, in m/s: fitted to RealFlight's back
    /// ramps and cruise thrust (`docs/findings/explora-thrust.md`), past the 21 m/s a 4.5"
    /// pitch at 1000 rpm/V on 11.1 V screws forward unloaded. Lower: less zoom climb on the
    /// back ramp, more cruise thrust.
    const PITCH_SPEED_MPS: f32 = 35.0;
    const TRIM_AIRSPEED_MPS: f32 = 14.0;
    let base = tailsitter();
    let airplane = plane().model.aero;
    let mixer = base.mixer;
    let mixer_config = MixerConfig::default();
    let thrust_curve_exponent = base.stabilizer.hover.thrust_curve_exponent;
    // How much heavier than the Aerosonde's scaled inertia the eXplora answers its controls in
    // RealFlight (roll, pitch, yaw in the airplane frame). Roll: RealFlight's cruise roll fit,
    // damping and control both (#619). Pitch and yaw: its hover, where its hand tune's rate
    // steps end at 0.80 (view pitch) and 0.55 (view roll, differential thrust) of their
    // command, and here at 0.80–0.82 and 0.60–0.62 (#539). Yaw follows differential thrust's
    // authority, the thrust per unit of motor command, which falls as the hover's command
    // rises: 6.1 at hover 0.88 answers as 7.0 did at 0.82 (#651). Higher: the hover's roll rate
    // steps end shorter. Scaling inertia keeps the controls' authority against the wing's own
    // moments; less wash over the elevons in its place matched the rate steps, but a 5° tilt's
    // drift then pushed the angle steps 10–20% past their target, which RealFlight's don't.
    let inertia_scale = Vec3::new(29.0, 19.0, 6.1);
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
            // The lift slope behind RealFlight's fitted path turn, 18.3/s at 14 m/s (#587),
            // below lifting-line theory's 4.4/rad for its 4.6 aspect ratio. Higher: the flight
            // path follows a pitch change faster.
            c_lift_alpha: 3.0,
            // Level flight at 14 m/s takes thrust setpoint 0.86, as in RealFlight
            // (`docs/findings/explora-thrust.md`).
            c_drag_0: 0.043,
            // Calibrated to RealFlight: level flight at 14 m/s takes a nose-up moment command of
            // 0.09 (#342, run 0; #587's fits), this and the wash's own pitch below at cruise
            // thrust (worth −0.01) together. The command is the two over the elevons'
            // effectiveness, so below trim airspeed they scale together.
            c_pitch_0: -0.115,
            // Above trim airspeed RealFlight's trim moment grows as about V^0.9, so the command
            // that holds it falls as V^-0.73: 0.089 at 14 m/s, 0.074 at 18 (#648); here 0.090
            // and 0.073. Below trim it stays on V², where #661's back ramps are fitted: any
            // stronger nose-down moment there hands the 0.65 ramp over instead of stalling it.
            // Higher: the trim command holds up at speed.
            c_pitch_0_knee: Some(PitchTrimKnee {
                knee_mps: TRIM_AIRSPEED_MPS,
                exponent: 1.0,
            }),
            // Calibrated to RealFlight (#342, run 0): default-gain 30°/s roll doublets peak at
            // 3.2× in their first half, here 3.24×.
            c_roll_aileron: 2.4 * base.model.wing.aero.c_roll_aileron,
            // RealFlight's cruise pitch control, 80–82 rad/s² (#619). Higher: the cruise pitch
            // answers its elevons faster, and the hover's (with the wash below) too.
            c_pitch_elevator: 1.65 * base.model.wing.aero.c_pitch_elevator,
            // A flying wing's small static margin, about 3% of chord (this over the lift slope):
            // RealFlight's fitted stiffness, 69–93/s² at 14 m/s (#587), where the Aerosonde's
            // tail gives 600. More negative: stiffer in angle of attack.
            c_pitch_alpha: -0.10,
            // No tail: the wing's own pitch damping, near the Zagi's -1.4 (Beard and McLain,
            // *Small Unmanned Aircraft*, app. E) and RealFlight's fitted 0–1.3/s (#587). More
            // negative: the wash damps the hover's pitch, and its searched pitch rate settles
            // short.
            c_pitch_q: -5.0,
            // The winglets alone: about 0.036 m² of fin 0.15 m behind the center on a 0.43 m²,
            // 1.4 m wing, at a fin lift slope near 2/rad. The Aerosonde's tail turns the wing
            // into a 5° tilt's drift hard enough to push its hover's angle steps 15% past
            // their target, where RealFlight's settle on it (#539).
            c_yaw_beta: 0.02,
            // The airplane's side force and dihedral, where its plane side was fitted (#539):
            // the Zagi's from `tailsitter()` lose 6.35 m in a 90°/s roll, past
            // `tests/maneuver.rs`'s 5.8 m band.
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
        thrust_falloff: ThrustFalloff::PitchSpeed {
            full_mps: PITCH_SPEED_MPS,
        },
        ..base.model.wing
    };
    let model = TailsitterModel {
        wing,
        motor_arm_m: MOTOR_ARM_M,
        // The wake's pressure over the elevons that, with the inertia and elevons above, flies
        // the hover's elevon rate steps as RealFlight does (#539, #619): 0.4 of it pitching them,
        // 0.14 rolling them apart (view yaw: the hand tune's steps end at 0.24–0.36, here too).
        // The washed strip is a third of the elevons' span for roll and pitch alike (by `∫y`
        // and `∫1`), so the split is fitted, not derived. The wash damps the span it covers.
        // Sinking, the wash leaves the elevons 1.4 times as fast as momentum theory has it:
        // fitted on RealFlight's hover descents, where a soft pitch tune swings the nose from
        // 2.5 m/s of sink and the autotuned one holds at 3 m/s. Higher: from 1.6 the autotuned
        // tune swings at 3 m/s too (`docs/findings/explora-descent-swing.md`).
        // The wash pitches the wing nose down, so the hover holds about 0.1 of nose-up elevon
        // as RealFlight's does, and harder past the stall nose first, so the back ramp at 0.65
        // thrust stalls at full elevon where 0.70 holds on half
        // (`docs/findings/explora-wash-pitch.md`). More nose down in the hover: the soft
        // tune's descents stop swinging (its rate I pins at `max_integral`); past the stall:
        // 0.70 stalls too.
        prop_wash: PropWash::props(
            2,
            PROP_DIAMETER_M,
            0.4,
            PropWash::span_damping_share(MOTOR_ARM_M, PROP_DIAMETER_M, SPAN_M),
        )
        .with_roll_fraction(0.14)
        .with_sink_loss(1.4)
        .with_pitch_moment(-0.13)
        .with_stall_pitch_moment(-2.0),
    };
    let plane_stabilizer = PlaneStabilizerConfig {
        rate: explora_rate(),
        // RealFlight's plane autotune's, like the rate gains: 5° steps rise in 420–440 ms (#535).
        time_constant_s: Vec2::new(0.226, 0.226),
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
    // `explora_vtol`'s back ramp thrust: a clean ramp in RealFlight (elevons at most 0.23,
    // #399) and here. Lower: less zoom climb, nearer the stall.
    guidance.transition.back_thrust = 0.75;
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

/// The eXplora's plane rate loop, as `explora_vtol()` flies it: RealFlight's plane autotune
/// (#535). Roll is all damping on a strong elevon, so feedforward carries a roll rate; pitch is
/// stiff and barely damped, so P does the work and I holds the rate against the stiffness.
fn explora_rate() -> PlaneRateControllerConfig {
    PlaneRateControllerConfig {
        roll: RateGains {
            p: 0.0229,
            i: 0.08,
            ff: 0.1279,
        },
        pitch: RateGains {
            p: 0.29,
            i: 0.63,
            ff: 0.006,
        },
        trim: Vec3::new(0.0, 0.09, 0.0),
        ..PlaneRateControllerConfig::default()
    }
}
