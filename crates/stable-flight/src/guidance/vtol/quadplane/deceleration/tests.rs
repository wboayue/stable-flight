use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::Vec3;

const DT: f32 = 0.1;
const TARGET_MPS2: f32 = 2.0;
const GAIN: f32 = 0.1;

/// Flying north at `speed_mps`, accelerating `north_mps2` (negative slowing).
fn flying(speed_mps: f32, north_mps2: f32) -> VehicleState {
    VehicleState {
        velocity_mps: Vec3::new(speed_mps, 0.0, 0.0),
        acceleration_mps2: Vec3::new(north_mps2, 0.0, 0.0),
        ..Default::default()
    }
}

/// One tick from `start_rad`: (what, start, state, pitch after).
#[test]
fn integrates_the_shortfall_within_its_clamp() {
    let rows = [
        (
            "gliding: slowing 1 short of 2",
            0.0,
            flying(30.0, -1.0),
            0.01,
        ),
        ("slowing at the target holds", 0.1, flying(30.0, -2.0), 0.1),
        (
            "slowing faster brings it down",
            0.1,
            flying(30.0, -4.0),
            0.08,
        ),
        ("never below level", 0.0, flying(30.0, -10.0), 0.0),
        ("speeding up: 4 short", 0.0, flying(30.0, 2.0), 0.04),
        (
            "never past the cap",
            0.295,
            flying(30.0, 2.0),
            PITCH_MAX_RAD,
        ),
        ("standing still: 2 short", 0.0, flying(0.0, 0.0), 0.02),
        ("NaN acceleration holds", 0.1, flying(30.0, f32::NAN), 0.1),
        ("NaN velocity holds", 0.1, flying(f32::NAN, -1.0), 0.1),
    ];
    for (what, start_rad, state, expected_rad) in rows {
        let mut deceleration = BackDeceleration {
            pitch_rad: start_rad,
        };
        let pitch_rad = deceleration.pitch_rad(DT, &state, TARGET_MPS2, GAIN);
        assert_relative_eq!(pitch_rad, expected_rad, epsilon = 1e-6);
        check!(pitch_rad == deceleration.pitch_rad, "{what}");
    }
}

/// The deceleration is measured along the ground velocity, whichever way it points.
#[test]
fn measures_deceleration_along_the_ground_track() {
    // Flying east, slowing 2 m/s² eastward plus a sideways 3 m/s² that doesn't count.
    let east = VehicleState {
        velocity_mps: Vec3::new(0.0, 20.0, 5.0),
        acceleration_mps2: Vec3::new(3.0, -2.0, 1.0),
        ..Default::default()
    };
    let mut deceleration = BackDeceleration::default();
    check!(deceleration.pitch_rad(DT, &east, TARGET_MPS2, GAIN) == 0.0);
}

#[test]
fn reset_levels_the_nose() {
    let mut deceleration = BackDeceleration { pitch_rad: 0.2 };
    deceleration.reset();
    check!(deceleration == BackDeceleration::default());
}

#[test]
fn gain_zero_stays_level() {
    let mut deceleration = BackDeceleration::default();
    for _ in 0..100 {
        deceleration.pitch_rad(DT, &flying(30.0, 0.0), TARGET_MPS2, 0.0);
    }
    check!(deceleration.pitch_rad == 0.0);
}
