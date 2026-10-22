//! The simulated eXplora: see [`explora`].

use core::f32::consts::PI;

use stable_flight::GRAVITY;
use stable_flight::Vec3;
use stable_flight::controllers::{PlaneRateControllerConfig, RateGains};
use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use stable_flight::mixers::MixerConfig;
use stable_flight::stabilizers::plane::PlaneStabilizerConfig;
use stable_flight::stabilizers::vtol::TailsitterStabilizerConfig;

use super::plane::aerosonde_inertia;
use super::tailsitter::{TailsitterAirframe, tailsitter};
use super::{hover_thrust, motor_count};
use crate::TailsitterModel;
use crate::fixed_wing::{AeroCoefficients, FixedWingModel};
use crate::motors::QUADRATIC;

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
/// - Inertia: RealFlight's own from that mass, roll at its 60% modifier.
/// - Props: two 10 × 4.5", kv1000 on 3S, about 21 m/s of pitch speed at full throttle.
///
/// Measured in RealFlight and matched here (#342, run 0): hover at thrust 0.82, cruise at 0.85
/// and 14–15 m/s, a level nose-up moment of 0.09, and default-gain doublets at 30°/s peaking
/// near 3.3× in roll and pitch.
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
    /// Hover and cruise thrust setpoints measured in RealFlight (#124, #342).
    const HOVER_THRUST: f32 = 0.82;
    const CRUISE_THRUST: f32 = 0.85;
    /// Full-throttle pitch speed of a 4.5" pitch prop at 1000 rpm/V on 11.1 V, in m/s: thrust
    /// falls to zero there.
    const PITCH_SPEED_MPS: f32 = 0.114 * 1000.0 * 11.1 / 60.0;
    const TRIM_AIRSPEED_MPS: f32 = 14.0;
    let base = tailsitter();
    let mixer = base.mixer;
    let mixer_config = MixerConfig::default();
    let thrust_curve_exponent = base.stabilizer.hover.thrust_curve_exponent;
    let mut inertia_kgm2 = aerosonde_inertia(MASS_KG, SPAN_M);
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
            // 3.2× (roll) and 3.4× (pitch) in their first half; the plane's derivatives at
            // ±45° give 1.9× and 1.7×.
            c_roll_aileron: 2.05 * base.model.wing.aero.c_roll_aileron,
            c_pitch_elevator: 3.2 * base.model.wing.aero.c_pitch_elevator,
            ..base.model.wing.aero
        },
        max_deflection_rad: base.model.wing.max_deflection_rad.map(|_| PI / 4.0),
        max_thrust_n,
        thrust_exponent: QUADRATIC,
        thrust_falloff_n_per_mps: max_thrust_n / PITCH_SPEED_MPS,
        ..base.model.wing
    };
    let model = TailsitterModel { wing, ..base.model };
    let plane_stabilizer = PlaneStabilizerConfig {
        rate: explora_rate(),
        thrust_curve_exponent,
        trim_airspeed_mps: TRIM_AIRSPEED_MPS,
        min_airspeed_mps: 9.0,
        max_airspeed_mps: 25.0,
        ..PlaneStabilizerConfig::default()
    };
    let mut guidance = base.guidance;
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
            hover: base.stabilizer.hover,
            plane: plane_stabilizer,
        },
    }
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
