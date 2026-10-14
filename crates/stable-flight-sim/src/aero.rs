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
/// rudder axes deflected by `deflection_rad` (`x`, `y`, `z`), and `wash_pa` of prop-wash
/// dynamic pressure over the surfaces on top of the free stream's (negative where the wash
/// opposes it; the sum is never below zero).
///
/// α is `atan2(w, u)` and β is `asin(v / V)`. Rate terms are written per unit of airspeed
/// rather than divided by it, so they vanish smoothly as the airspeed goes to zero; at zero
/// airspeed only the surfaces in the wash act.
pub(crate) fn wrench(
    model: &FixedWingModel,
    air_mps: Vec3,
    rates_radps: Vec3,
    deflection_rad: Vec3,
    wash_pa: f32,
) -> Aero {
    let c = &model.aero;
    let (span, chord) = (model.span_m, model.chord_m);
    let speed_mps = air_mps.norm();
    // `>` is false for NaN: unknown air is still air.
    let moving = speed_mps > 0.0;

    // Dynamic pressure times wing area; the surfaces also see the wash.
    let pressure_area_n = if moving {
        pressure_area_n(model, speed_mps)
    } else {
        0.0
    };
    let surface_area_n = (pressure_area_n + wash_pa * model.wing_area_m2).max(0.0);
    let surfaces = surface_wrench(model, surface_area_n, deflection_rad);
    if !moving {
        return surfaces;
    }

    let alpha_rad = libm::atan2f(air_mps.z, air_mps.x);
    let beta_rad = libm::asinf((air_mps.y / speed_mps).clamp(-1.0, 1.0));
    // The rate terms' per-unit-airspeed scale.
    let rate_scale = pressure_area_n / speed_mps;

    let (sin_alpha, cos_alpha) = (libm::sinf(alpha_rad), libm::cosf(alpha_rad));
    let blend = stall_blend(c, alpha_rad);
    let lift_n = pressure_area_n * lift_coefficient(c, alpha_rad, blend, sin_alpha, cos_alpha);
    let drag_n =
        pressure_area_n * (1.0 - blend) * drag_coefficient(model, linear_lift(c, alpha_rad))
            + pressure_area_n * blend * flat_plate_drag(c, sin_alpha);
    let side_n = pressure_area_n * c.c_side_beta * beta_rad;
    // Lift is normal to the airflow, toward body −z; drag along it.
    let force_n = Vec3::new(
        lift_n * sin_alpha - drag_n * cos_alpha,
        side_n,
        -lift_n * cos_alpha - drag_n * sin_alpha,
    );

    let (p, q, r) = (rates_radps.x, rates_radps.y, rates_radps.z);
    let roll_nm = pressure_area_n * span * c.c_roll_beta * beta_rad
        + rate_scale * span * span / 2.0 * (c.c_roll_p * p + c.c_roll_r * r);
    // Static stability on sin α: the slope at small α, bounded at any α, and gone with the
    // flow broadside (α ±90°: the wing's center of pressure near mid-chord) or reversed.
    let pitch_nm = pressure_area_n * chord * (c.c_pitch_0 + c.c_pitch_alpha * sin_alpha)
        + rate_scale * chord * chord / 2.0 * c.c_pitch_q * q;
    let yaw_nm = pressure_area_n * span * c.c_yaw_beta * beta_rad
        + rate_scale * span * span / 2.0 * (c.c_yaw_p * p + c.c_yaw_r * r);

    Aero {
        force_n: force_n + surfaces.force_n,
        torque_nm: Vec3::new(roll_nm, pitch_nm, yaw_nm) + surfaces.torque_nm,
        alpha_rad,
        beta_rad,
    }
}

/// The side force and moments of `model`'s surfaces deflected by `deflection_rad` (aileron,
/// elevator, rudder) under `pressure_area_n` (their dynamic pressure times wing area, in N).
fn surface_wrench(model: &FixedWingModel, pressure_area_n: f32, deflection_rad: Vec3) -> Aero {
    let c = &model.aero;
    let (aileron, elevator, rudder) = (deflection_rad.x, deflection_rad.y, deflection_rad.z);
    Aero {
        force_n: Vec3::new(0.0, pressure_area_n * c.c_side_rudder * rudder, 0.0),
        torque_nm: Vec3::new(
            model.span_m * (c.c_roll_aileron * aileron + c.c_roll_rudder * rudder),
            model.chord_m * c.c_pitch_elevator * elevator,
            model.span_m * c.c_yaw_rudder * rudder,
        ) * pressure_area_n,
        ..Aero::default()
    }
}

/// Dynamic pressure at `speed_mps` times `model`'s wing area, in N.
pub(crate) fn pressure_area_n(model: &FixedWingModel, speed_mps: f32) -> f32 {
    0.5 * AIR_DENSITY_KGPM3 * speed_mps * speed_mps * model.wing_area_m2
}

/// The linear lift coefficient at `alpha_rad`: `c_lift_0 + c_lift_alpha · α`.
pub(crate) fn linear_lift(c: &AeroCoefficients, alpha_rad: f32) -> f32 {
    c.c_lift_0 + c.c_lift_alpha * alpha_rad
}

/// Drag coefficient of the attached flow at `linear_lift` (the linear lift coefficient):
/// parasitic plus induced, `c_drag_0 + CL²/(π·e·AR)`.
pub(crate) fn drag_coefficient(model: &FixedWingModel, linear_lift: f32) -> f32 {
    let c = &model.aero;
    c.c_drag_0 + linear_lift * linear_lift / (PI * c.oswald * model.aspect_ratio())
}

/// Drag coefficient past the stall: parasitic plus a flat plate's `2·sin²α`, at most
/// `c_drag_0 + 2` broadside.
fn flat_plate_drag(c: &AeroCoefficients, sin_alpha: f32) -> f32 {
    c.c_drag_0 + 2.0 * sin_alpha * sin_alpha
}

/// How far past the stall `alpha_rad` is: a sigmoid that is 0 inside `±alpha_stall_rad` and 1
/// outside.
pub(crate) fn stall_blend(c: &AeroCoefficients, alpha_rad: f32) -> f32 {
    let exponent = |x: f32| libm::expf(x.clamp(-MAX_BLEND_EXPONENT, MAX_BLEND_EXPONENT));
    let past_positive = exponent(-c.stall_sharpness * (alpha_rad - c.alpha_stall_rad));
    let past_negative = exponent(c.stall_sharpness * (alpha_rad + c.alpha_stall_rad));
    (1.0 + past_positive + past_negative) / ((1.0 + past_positive) * (1.0 + past_negative))
}

/// Lift coefficient at `alpha_rad` (with its [`stall_blend`], sine, and cosine): linear below
/// the stall, a flat plate's past it.
pub(crate) fn lift_coefficient(
    c: &AeroCoefficients,
    alpha_rad: f32,
    blend: f32,
    sin_alpha: f32,
    cos_alpha: f32,
) -> f32 {
    let flat_plate = 2.0 * alpha_rad.signum() * sin_alpha * sin_alpha * cos_alpha;
    (1.0 - blend) * linear_lift(c, alpha_rad) + blend * flat_plate
}
