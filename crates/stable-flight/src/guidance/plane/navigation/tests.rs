use approx::assert_relative_eq;
use assert2::check;

use super::*;

const PERIOD_S: f32 = 12.0;
const DAMPING: f32 = 0.75;

fn track(position: (f32, f32), velocity: (f32, f32)) -> f32 {
    // The line along north through the origin.
    track_accel_mps2(
        Vec2::new(position.0, position.1),
        Vec2::new(velocity.0, velocity.1),
        Vec2::zeros(),
        Vec2::new(1.0, 0.0),
        PERIOD_S,
        DAMPING,
    )
}

/// On the line and along it: no turn. Off to one side or angled away: a turn back toward the
/// line, matching the L1 formula. (position (n, e), velocity (n, e), expected sign)
#[test]
fn track_turns_toward_the_line() {
    let cases = [
        ((0.0, 0.0), (15.0, 0.0), 0),
        ((-50.0, 0.0), (15.0, 0.0), 0), // behind the point, on the line
        ((0.0, 10.0), (15.0, 0.0), -1), // right of the line: turn left
        ((0.0, -10.0), (15.0, 0.0), 1), // left: turn right
        ((0.0, 0.0), (15.0, 5.0), -1),  // on it, heading right of it
        ((0.0, 0.0), (15.0, -5.0), 1),
    ];
    for (position, velocity, sign) in cases {
        let accel = track(position, velocity);
        let got = if accel.abs() < 1e-4 {
            0
        } else {
            accel.signum() as i32
        };
        check!(got == sign, "{position:?} {velocity:?}: {accel}");
    }

    // The formula: aligned with the line, 10 m right of it, at 15 m/s.
    let speed = 15.0_f32;
    let l1 = DAMPING * PERIOD_S * speed / core::f32::consts::PI;
    let capture = (10.0 / l1).asin().min(core::f32::consts::FRAC_PI_4);
    let expected = 4.0 * DAMPING * DAMPING * speed * speed / l1 * (-capture).sin();
    assert_relative_eq!(track((0.0, 10.0), (speed, 0.0)), expected, epsilon = 1e-4);
}

/// Crabbing into a crosswind on the line, the ground velocity is along it: no turn, though
/// the nose points upwind.
#[test]
fn crosswind_crab_on_the_line_needs_no_turn() {
    // Ground track north at 15 m/s; the nose's heading doesn't enter.
    check!(track((0.0, 0.0), (15.0, 0.0)).abs() < 1e-5);
}

/// Far off or pointing away, the demand saturates at η = ±90° instead of growing.
#[test]
fn track_saturates_far_off() {
    let near = track((0.0, 200.0), (15.0, 0.0));
    let away = track((0.0, 0.0), (0.0, 15.0)); // flying due east, off a north line
    let speed = 15.0_f32;
    let l1 = DAMPING * PERIOD_S * speed / core::f32::consts::PI;
    let full = 4.0 * DAMPING * DAMPING * speed * speed / l1;
    check!(near < 0.0 && near.abs() <= full + 1e-4);
    assert_relative_eq!(away, -full, epsilon = 1e-3);
}

/// Too slow, a degenerate direction, or NaN: no demand.
#[test]
fn track_is_quiet_without_a_course() {
    check!(track((0.0, 10.0), (0.5, 0.0)) == 0.0);
    check!(track((f32::NAN, 0.0), (15.0, 0.0)) == 0.0);
    let no_direction = track_accel_mps2(
        Vec2::zeros(),
        Vec2::new(15.0, 0.0),
        Vec2::zeros(),
        Vec2::zeros(),
        PERIOD_S,
        DAMPING,
    );
    check!(no_direction == 0.0);
}

/// On the circle, flying its clockwise tangent: exactly the centripetal demand; outside, more
/// (turn in); inside, less.
#[test]
fn loiter_holds_a_clockwise_circle() {
    let (radius, speed) = (40.0, 15.0);
    let at = |north: f32, velocity: (f32, f32)| {
        loiter_accel_mps2(
            Vec2::new(north, 0.0),
            Vec2::new(velocity.0, velocity.1),
            Vec2::zeros(),
            radius,
            PERIOD_S,
            DAMPING,
        )
    };
    // North of the center, clockwise is due east.
    let on = at(radius, (0.0, speed));
    assert_relative_eq!(on, speed * speed / radius, epsilon = 1e-4);
    check!(at(radius + 10.0, (0.0, speed)) > on);
    check!(at(radius - 10.0, (0.0, speed)) < on);
    // At the center: the centripetal turn alone.
    assert_relative_eq!(
        at(0.0, (0.0, speed)),
        speed * speed / radius,
        epsilon = 1e-4
    );
    check!(at(radius, (0.0, 0.2)) == 0.0);
}

/// Bank for a level turn: atan(a/g), NaN level.
#[test]
fn bank_for_accel_is_atan_a_over_g() {
    assert_relative_eq!(
        bank_for_accel_rad(GRAVITY),
        core::f32::consts::FRAC_PI_4,
        epsilon = 1e-6
    );
    assert_relative_eq!(
        bank_for_accel_rad(-GRAVITY),
        -core::f32::consts::FRAC_PI_4,
        epsilon = 1e-6
    );
    check!(bank_for_accel_rad(0.0) == 0.0);
    check!(bank_for_accel_rad(f32::NAN) == 0.0);
}

/// The PI loop: P and I, clamped output, integral held while pinned in the error's
/// direction, NaN error → fallback with the integral untouched.
#[test]
fn pi_loop_clamps_and_holds_its_integral_when_pinned() {
    let mut pi = PiLoop::default();
    // P only below the limit.
    assert_relative_eq!(pi.step(0.1, 2.0, 0.1, 0.0, 10.0, (-1.0, 1.0), 0.0), 0.2);
    // I accumulates: 0.2 from P plus 0.5 · (2 · 0.1 · 2 ticks) = 0.4.
    let mut pi = PiLoop::default();
    pi.step(0.1, 2.0, 0.1, 0.5, 10.0, (-1.0, 1.0), 0.0);
    assert_relative_eq!(
        pi.step(0.1, 2.0, 0.1, 0.5, 10.0, (-1.0, 1.0), 0.0),
        0.4,
        epsilon = 1e-6
    );

    // Pinned at the top by a large error: the integral stops growing.
    let mut pi = PiLoop::default();
    for _ in 0..100 {
        check!(pi.step(0.1, 50.0, 0.1, 0.5, 10.0, (-1.0, 1.0), 0.0) == 1.0);
    }
    let held = pi.integral;
    check!(held < 1.0); // stopped near where the output pinned
    // Error reverses: unwinds at once from the held integral.
    let back = pi.step(0.1, -1.0, 0.1, 0.5, 10.0, (-1.0, 1.0), 0.0);
    check!(back < 1.0);

    let before = pi.integral;
    check!(pi.step(0.1, f32::NAN, 0.1, 0.5, 10.0, (-1.0, 1.0), 0.3) == 0.3);
    check!(pi.integral == before);
}
