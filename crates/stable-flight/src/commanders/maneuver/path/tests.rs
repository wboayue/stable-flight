use core::f32::consts::{FRAC_PI_2, PI};

use approx::assert_relative_eq;
use assert2::check;
use nalgebra::{Quaternion, UnitQuaternion};

use super::*;
use crate::commanders::maneuver::{Axis, SegmentEnd, Thrust};

/// Speed along the line, north, in m/s.
const SPEED_MPS: f32 = 15.0;

/// The PD alone, no gravity share.
const PD: PathConfig = PathConfig {
    position_gain_per_s2: 4.0,
    velocity_gain_per_s: 4.0,
    line_gravity_share: 0.0,
};

/// Flying north at [`SPEED_MPS`], level and at `roll_rad`, at `offset_m` (north, east, up) from
/// where the segment began, climbing at `climb_mps`.
fn flying(roll_rad: f32, offset_m: Vec3, climb_mps: f32) -> VehicleState {
    VehicleState {
        position_m: offset_m,
        velocity_mps: Vec3::new(SPEED_MPS, 0.0, climb_mps),
        airspeed_mps: Some(SPEED_MPS),
        attitude: UnitQuaternion::from_euler_angles(roll_rad, 0.0, 0.0),
        ..Default::default()
    }
}

/// A full turn about `axis` at `rate`, holding `path`.
fn segment(axis: Axis, rate: RateLaw, path: Path) -> Segment {
    Segment::Rotate {
        axis,
        rate,
        end: SegmentEnd::Angle { rad: 2.0 * PI },
        thrust: Thrust::Maneuver,
        path,
    }
}

/// The correction at `state` for `segment` begun level on the line.
fn correction(segment: Segment, config: &PathConfig, state: &VehicleState) -> Vec3 {
    PathHold::new(&segment, &flying(0.0, Vec3::zeros(), 0.0)).correction_radps(config, state)
}

/// Off a roll's line, it steers back with pitch and yaw in the body, whichever way up:
/// (case, roll in rad, offset north/east/up in m, climb in m/s, gravity share, expected body
/// rates in rad/s). 2 m off at 4/s² is 8 m/s², 8/15 rad/s at 15 m/s.
#[test]
fn a_line_steers_back_whichever_way_up() {
    let back = 8.0 / SPEED_MPS;
    let roll = segment(Axis::Roll, RateLaw::Constant { radps: PI }, Path::Line);
    let cases = [
        ("on the line", 0.0, Vec3::zeros(), 0.0, 0.0, Vec3::zeros()),
        (
            "along it, ahead",
            0.0,
            Vec3::new(30.0, 0.0, 0.0),
            0.0,
            0.0,
            Vec3::zeros(),
        ),
        (
            "below, upright: nose up",
            0.0,
            Vec3::new(0.0, 0.0, -2.0),
            0.0,
            0.0,
            Vec3::new(0.0, back, 0.0),
        ),
        (
            "below, inverted: push",
            PI,
            Vec3::new(0.0, 0.0, -2.0),
            0.0,
            0.0,
            Vec3::new(0.0, -back, 0.0),
        ),
        (
            "below, right wing down: nose left, toward up",
            FRAC_PI_2,
            Vec3::new(0.0, 0.0, -2.0),
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, -back),
        ),
        (
            "right of it, upright: nose left",
            0.0,
            Vec3::new(0.0, 2.0, 0.0),
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, -back),
        ),
        (
            "inverted on it, half the lost lift: push",
            PI,
            Vec3::zeros(),
            0.0,
            0.5,
            // 0.5 · 2 · g up, inverted.
            Vec3::new(0.0, -GRAVITY / SPEED_MPS, 0.0),
        ),
        (
            "upright on it, half the lost lift: none lost",
            0.0,
            Vec3::zeros(),
            0.0,
            0.5,
            Vec3::zeros(),
        ),
    ];
    for (case, roll_rad, offset_m, climb_mps, share, expected) in cases {
        let config = PathConfig {
            line_gravity_share: share,
            ..PD
        };
        let state = flying(roll_rad, offset_m, climb_mps);
        let radps = correction(roll, &config, &state);
        check!((radps - expected).norm() < 1e-4, "{case}: {radps:?}");
    }
}

/// Sinking off the line damps with the velocity gain: 1 m/s down at 4/s is 4 m/s² up, turned
/// at the speed along the path's normal.
#[test]
fn a_line_damps_a_sink() {
    let roll = segment(Axis::Roll, RateLaw::Constant { radps: PI }, Path::Line);
    let radps = correction(roll, &PD, &flying(0.0, Vec3::zeros(), -1.0));
    // The velocity tips 1 in 15 below the line, so the up acceleration turns it at cos of that.
    let along = SPEED_MPS / libm::sqrtf(SPEED_MPS * SPEED_MPS + 1.0);
    assert_relative_eq!(radps.y, 4.0 * along / SPEED_MPS, max_relative = 1e-4);
    check!(radps.x == 0.0);
}

/// Into a headwind as strong as the airspeed there is no ground speed to read the direction of
/// travel from: below the line it still pitches up, along the direction it began with.
#[test]
fn a_line_steers_back_into_a_headwind() {
    let roll = segment(Axis::Roll, RateLaw::Constant { radps: PI }, Path::Line);
    let hovering = VehicleState {
        velocity_mps: Vec3::zeros(),
        ..flying(0.0, Vec3::new(0.0, 0.0, -2.0), 0.0)
    };
    let radps = correction(roll, &PD, &hovering);
    check!(
        (radps - Vec3::new(0.0, 8.0 / SPEED_MPS, 0.0)).norm() < 1e-4,
        "{radps:?}"
    );
}

/// Off a loop's circle, it pulls harder outside and eases inside, and steers back into the
/// circle's plane with yaw: (case, segment, offset north/east/up in m, expected body rates in
/// rad/s). The circles start level heading north with a 10 m radius; 1 m off at 4/s² is 4/15
/// rad/s.
#[test]
fn a_circle_steers_back_onto_it() {
    let back = 4.0 / SPEED_MPS;
    let circle = |axis, radius_m| segment(axis, RateLaw::Radius { radius_m }, Path::Circle);
    let cases = [
        (
            "on a loop at its start",
            circle(Axis::Pitch, 10.0),
            Vec3::zeros(),
            Vec3::zeros(),
        ),
        (
            "below a loop, outside it: pull harder",
            circle(Axis::Pitch, 10.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, back, 0.0),
        ),
        (
            "above a loop's start, inside it: ease off",
            circle(Axis::Pitch, 10.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, -back, 0.0),
        ),
        (
            "below an outside loop, inside it: ease the push",
            circle(Axis::Pitch, -10.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.0, back, 0.0),
        ),
        (
            "right of a loop's plane: nose left",
            circle(Axis::Pitch, 10.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, -back),
        ),
        (
            "left of a right yaw circle, outside it: turn right harder",
            circle(Axis::Yaw, 10.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, back),
        ),
    ];
    for (case, segment, offset_m, expected) in cases {
        let radps = correction(segment, &PD, &flying(0.0, offset_m, 0.0));
        check!((radps - expected).norm() < 1e-4, "{case}: {radps:?}");
    }
}

/// Nothing to correct without a path, a speed, a finite state, or a direction back: (case,
/// segment, state).
#[test]
fn corrects_nothing_without_a_path_a_speed_or_a_finite_state() {
    let below = Vec3::new(0.0, 0.0, -2.0);
    let roll = segment(Axis::Roll, RateLaw::Constant { radps: PI }, Path::Line);
    let free = segment(Axis::Roll, RateLaw::Constant { radps: PI }, Path::Free);
    let still = VehicleState {
        position_m: below,
        ..Default::default()
    };
    let blind = VehicleState {
        attitude: UnitQuaternion::new_unchecked(Quaternion::new(
            f32::NAN,
            f32::NAN,
            f32::NAN,
            f32::NAN,
        )),
        ..flying(0.0, below, 0.0)
    };
    let lost = VehicleState {
        position_m: Vec3::new(0.0, 0.0, f32::NAN),
        ..flying(0.0, below, 0.0)
    };
    let loop_center = flying(0.0, Vec3::new(0.0, 0.0, 10.0), 0.0);
    let loop_ = segment(
        Axis::Pitch,
        RateLaw::Radius { radius_m: 10.0 },
        Path::Circle,
    );
    let pause = Segment::Pause {
        duration_s: 1.0,
        thrust: Thrust::Cruise,
    };
    let cases = [
        ("free", free, flying(0.0, below, 0.0)),
        ("a pause", pause, flying(0.0, below, 0.0)),
        ("still, no airspeed", roll, still),
        ("no attitude", roll, blind),
        ("no position", roll, lost),
        ("at a loop's center", loop_, loop_center),
    ];
    for (case, segment, state) in cases {
        check!(correction(segment, &PD, &state) == Vec3::zeros(), "{case}");
    }
}

/// Gains and the share are non-negative and finite: (case, corrupt, field).
#[test]
fn validates_config() {
    type Break = fn(&mut PathConfig);
    let cases: [(Break, &str); 3] = [
        (|c| c.position_gain_per_s2 = -1.0, "position_gain_per_s2"),
        (|c| c.velocity_gain_per_s = f32::NAN, "velocity_gain_per_s"),
        (
            |c| c.line_gravity_share = f32::INFINITY,
            "line_gravity_share",
        ),
    ];
    check!(PathConfig::default().validate().is_ok());
    check!(PD.validate().is_ok());
    for (corrupt, field) in cases {
        let mut config = PathConfig::default();
        corrupt(&mut config);
        check!(config.validate().map_err(|e| e.field) == Err(field));
    }
}
