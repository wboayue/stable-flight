//! Fixed-wing aerodynamic forces and moments: see [`wrench`].

use core::f32::consts::PI;

use stable_flight::Vec3;

use crate::fixed_wing::{AeroCoefficients, FixedWingModel};

#[cfg(test)]
mod tests;

/// Sea-level air density, in kg/m³ (ISA).
pub(crate) const AIR_DENSITY_KGPM3: f32 = 1.225;

/// Largest exponent the stall blend evaluates: `e^60` is far past where the blend settles and
/// inside `f32` range; a product of two overflows to infinity, which is the blend's limit (0).
const MAX_BLEND_EXPONENT: f32 = 60.0;

/// The aerodynamic force and moment on an airplane, and the flow angles they came from.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Aero {
    /// Force on the FRD body axes, in N.
    pub force_n: Vec3,
    /// Moment on the FRD body axes, in N·m.
    pub torque_nm: Vec3,
    /// Angle of attack, in rad.
    pub alpha_rad: f32,
    /// Sideslip, in rad: positive with the air coming from the right.
    pub beta_rad: f32,
}

/// The aerodynamics of `model` moving through the air at `air_mps` (its velocity relative to
/// the air, FRD body axes) while turning at `rates_radps`, with its aileron, elevator, and
/// rudder axes deflected by `deflection_rad` (`x`, `y`, `z`).
///
/// α is `atan2(w, u)` and β is `asin(v / V)`. Rate terms are written per unit of airspeed
/// rather than divided by it, so they vanish smoothly as the airspeed goes to zero; at zero
/// airspeed there is no force or moment.
pub(crate) fn wrench(
    model: &FixedWingModel,
    air_mps: Vec3,
    rates_radps: Vec3,
    deflection_rad: Vec3,
) -> Aero {
    let speed_mps = air_mps.norm();
    if speed_mps.is_nan() || speed_mps <= 0.0 {
        return Aero::default();
    }
    let c = &model.aero;
    let alpha_rad = libm::atan2f(air_mps.z, air_mps.x);
    let beta_rad = libm::asinf((air_mps.y / speed_mps).clamp(-1.0, 1.0));
    let (aileron, elevator, rudder) = (deflection_rad.x, deflection_rad.y, deflection_rad.z);
    let (span, chord) = (model.span_m, model.chord_m);

    // Dynamic pressure times wing area, in N, and its per-unit-airspeed part for rate terms.
    let pressure_area_n = 0.5 * AIR_DENSITY_KGPM3 * speed_mps * speed_mps * model.wing_area_m2;
    let rate_scale = 0.5 * AIR_DENSITY_KGPM3 * speed_mps * model.wing_area_m2;

    let linear_lift = c.c_lift_0 + c.c_lift_alpha * alpha_rad;
    let aspect_ratio = span * span / model.wing_area_m2;
    let lift_n = pressure_area_n * lift_coefficient(c, alpha_rad);
    let drag_n =
        pressure_area_n * (c.c_drag_0 + linear_lift * linear_lift / (PI * c.oswald * aspect_ratio));
    let side_n = pressure_area_n * (c.c_side_beta * beta_rad + c.c_side_rudder * rudder);
    // Lift is normal to the airflow, toward body −z; drag along it.
    let (sin_alpha, cos_alpha) = (libm::sinf(alpha_rad), libm::cosf(alpha_rad));
    let force_n = Vec3::new(
        lift_n * sin_alpha - drag_n * cos_alpha,
        side_n,
        -lift_n * cos_alpha - drag_n * sin_alpha,
    );

    let (p, q, r) = (rates_radps.x, rates_radps.y, rates_radps.z);
    let roll_nm = pressure_area_n
        * span
        * (c.c_roll_beta * beta_rad + c.c_roll_aileron * aileron + c.c_roll_rudder * rudder)
        + rate_scale * span * span / 2.0 * (c.c_roll_p * p + c.c_roll_r * r);
    let pitch_nm = pressure_area_n
        * chord
        * (c.c_pitch_0 + c.c_pitch_alpha * alpha_rad + c.c_pitch_elevator * elevator)
        + rate_scale * chord * chord / 2.0 * c.c_pitch_q * q;
    let yaw_nm = pressure_area_n * span * (c.c_yaw_beta * beta_rad + c.c_yaw_rudder * rudder)
        + rate_scale * span * span / 2.0 * (c.c_yaw_p * p + c.c_yaw_r * r);

    Aero {
        force_n,
        torque_nm: Vec3::new(roll_nm, pitch_nm, yaw_nm),
        alpha_rad,
        beta_rad,
    }
}

/// Lift coefficient at `alpha_rad`: linear below the stall, a flat plate's past it, blended by
/// a sigmoid that is 0 inside `±alpha_stall_rad` and 1 outside.
pub(crate) fn lift_coefficient(c: &AeroCoefficients, alpha_rad: f32) -> f32 {
    let exponent = |x: f32| libm::expf(x.clamp(-MAX_BLEND_EXPONENT, MAX_BLEND_EXPONENT));
    let past_positive = exponent(-c.stall_sharpness * (alpha_rad - c.alpha_stall_rad));
    let past_negative = exponent(c.stall_sharpness * (alpha_rad + c.alpha_stall_rad));
    let blend =
        (1.0 + past_positive + past_negative) / ((1.0 + past_positive) * (1.0 + past_negative));
    let linear = c.c_lift_0 + c.c_lift_alpha * alpha_rad;
    let sin_alpha = libm::sinf(alpha_rad);
    let flat_plate = 2.0 * alpha_rad.signum() * sin_alpha * sin_alpha * libm::cosf(alpha_rad);
    (1.0 - blend) * linear + blend * flat_plate
}
