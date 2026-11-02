//! Fixed-wing waypoint navigation for [`PlaneGuidance`](super::PlaneGuidance): L1 path
//! following and loiter (lateral acceleration → bank), and the altitude (→ pitch) and airspeed
//! (→ throttle) loops.
//!
//! The lateral guidance follows ArduPilot's L1 controller (`libraries/AP_L1_Control`,
//! `AP_L1_Control.cpp`), described, not copied: a reference point L1 ahead on the path, at
//! `L1 = damping · period · V / π`; the lateral acceleration toward it `4ζ²·V²/L1 · sin η`, η
//! the angle from the ground velocity to the line to that point; and the bank that holds it in
//! a level turn, `atan(a / g)`. Working from the ground velocity, a crosswind crab is flown, not
//! fought.

use core::f32::consts::FRAC_PI_2;

use crate::math::clamp_symmetric;
use crate::{GRAVITY, Vec2};

#[cfg(test)]
mod tests;

/// Below this ground speed the path has no course to compare against: wings level.
const MIN_GROUND_SPEED_MPS: f32 = 1.0;

/// The largest capture angle onto the path: the vehicle heads back at no more than 45°. Part
/// of the L1 law, not a tuning knob: ArduPilot fixes the same cap in code
/// (`AP_L1_Control::update_waypoint`, no `NAVL1_*` param); toward 90° the capture turns into
/// heading straight at the line, which overshoots it.
const MAX_CAPTURE_RAD: f32 = core::f32::consts::FRAC_PI_4;

/// The L1 distance and gain for a ground speed: `(L1, 4ζ²)`.
fn l1(ground_speed_mps: f32, period_s: f32, damping: f32) -> (f32, f32) {
    let distance_m = damping * period_s * ground_speed_mps / core::f32::consts::PI;
    (distance_m, 4.0 * damping * damping)
}

/// 2-D cross product (`a × b`, the z part): positive when `b` points right of `a` in
/// north/east coordinates.
fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// The lateral acceleration (m/s², right positive) that follows the line through `point_m`
/// along `direction` (north/east, any length), from `position_m` moving at `velocity_mps`
/// (ground). Zero below [`MIN_GROUND_SPEED_MPS`] or for a degenerate or non-finite input.
pub(crate) fn track_accel_mps2(
    position_m: Vec2,
    velocity_mps: Vec2,
    point_m: Vec2,
    direction: Vec2,
    period_s: f32,
    damping: f32,
) -> f32 {
    let Some(direction) = direction.try_normalize(1e-6) else {
        return 0.0;
    };
    let speed_mps = velocity_mps.norm();
    // NaN fails the comparison: no course.
    if speed_mps.is_nan() || speed_mps < MIN_GROUND_SPEED_MPS {
        return 0.0;
    }
    let (distance_m, gain) = l1(speed_mps, period_s, damping);
    // Right of the line is positive.
    let cross_track_m = cross(direction, position_m - point_m);
    // Head back toward the line, at most 45°, more the farther off it is.
    let capture_rad = libm::asinf((cross_track_m / distance_m).clamp(-1.0, 1.0))
        .clamp(-MAX_CAPTURE_RAD, MAX_CAPTURE_RAD);
    // The course relative to the line, right positive.
    let course_rad = libm::atan2f(cross(direction, velocity_mps), direction.dot(&velocity_mps));
    let eta_rad = clamp_symmetric(-capture_rad - course_rad, FRAC_PI_2);
    let accel = gain * speed_mps * speed_mps / distance_m * libm::sinf(eta_rad);
    if accel.is_finite() { accel } else { 0.0 }
}

/// The lateral acceleration (m/s², right positive) that holds a clockwise circle of
/// `radius_m` around `center_m`: the centripetal `V²/r` of the circle, plus L1 toward the
/// tangent at the nearest point of it (ArduPilot's loiter, `AP_L1_Control.cpp`, combines a
/// radial correction with the centripetal term). At the center there is no nearest point:
/// the centripetal term alone starts the turn.
pub(crate) fn loiter_accel_mps2(
    position_m: Vec2,
    velocity_mps: Vec2,
    center_m: Vec2,
    radius_m: f32,
    period_s: f32,
    damping: f32,
) -> f32 {
    let speed_mps = velocity_mps.norm();
    let slow = speed_mps.is_nan() || speed_mps < MIN_GROUND_SPEED_MPS;
    if slow || radius_m.is_nan() || radius_m <= 0.0 {
        return 0.0;
    }
    let centripetal = speed_mps * speed_mps / radius_m;
    let Some(outward) = (position_m - center_m).try_normalize(1e-3) else {
        return centripetal;
    };
    // Clockwise seen from above, north/east: from north, toward east.
    let tangent = Vec2::new(-outward.y, outward.x);
    let nearest_m = center_m + outward * radius_m;
    centripetal
        + track_accel_mps2(
            position_m,
            velocity_mps,
            nearest_m,
            tangent,
            period_s,
            damping,
        )
}

/// The bank that turns at `accel_mps2` in level flight, `atan(a / g)`, right positive; NaN
/// gives level.
pub(crate) fn bank_for_accel_rad(accel_mps2: f32) -> f32 {
    let bank = libm::atanf(accel_mps2 / GRAVITY);
    if bank.is_finite() { bank } else { 0.0 }
}

/// A PI loop with a clamped output and an integral that stops growing while the output is
/// pinned against a limit in the direction the error pushes (anti-windup). Holds only its
/// integral.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct PiLoop {
    integral: f32,
}

impl PiLoop {
    /// `p · error + i · ∫error`, clamped to `[low, high]` (a `high` below `low` reads as
    /// `low`); the integral is clamped to `max_integral`. A non-finite error gives `fallback`
    /// and leaves the integral alone.
    #[allow(
        clippy::too_many_arguments,
        reason = "a PI loop's gains and limits, inline"
    )]
    pub(crate) fn step(
        &mut self,
        dt: f32,
        error: f32,
        p: f32,
        i: f32,
        max_integral: f32,
        (low, high): (f32, f32),
        fallback: f32,
    ) -> f32 {
        if !error.is_finite() {
            return fallback;
        }
        let high = high.max(low);
        let unclamped = p * error + i * self.integral;
        let pinned = (unclamped >= high && error > 0.0) || (unclamped <= low && error < 0.0);
        if !pinned {
            self.integral = clamp_symmetric(self.integral + error * dt, max_integral);
        }
        (p * error + i * self.integral).clamp(low, high)
    }
}
