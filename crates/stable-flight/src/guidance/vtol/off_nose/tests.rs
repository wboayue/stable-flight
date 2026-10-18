use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::{Vec2, Vec3};

/// The default cut, 15 m/s.
fn default_cut() -> OffNoseSpeed {
    OffNoseSpeed::new(OffNoseSpeedConfig::default())
}

/// A waypoint 100 m from the origin, `bearing_deg` from north, at `cruise_mps`.
fn waypoint_at(bearing_deg: f32, cruise_mps: f32) -> WaypointCommand {
    let bearing_rad = bearing_deg * DEG;
    WaypointCommand {
        position_m: 100.0 * Vec2::new(libm::cosf(bearing_rad), libm::sinf(bearing_rad)),
        cruise_speed_mps: cruise_mps,
        ..Default::default()
    }
}

/// A vehicle over the origin heading `heading_deg`.
fn heading(heading_deg: f32) -> VehicleState {
    VehicleState {
        heading_rad: heading_deg * DEG,
        ..Default::default()
    }
}

#[test]
fn cuts_the_speed_by_how_far_the_waypoint_is_off_the_nose() {
    // (case, bearing, heading, cruise, expected cruise); degrees and m/s, a 15 m/s cut, rows
    // clear of 20° by 0.5° for f32 rounding
    let cases = [
        ("ahead, uncapped", 0.0, 0.0, 0.0, 0.0),
        ("just inside 20°", 19.5, 0.0, 0.0, 0.0),
        ("just inside 20° the other way", -19.5, 0.0, 0.0, 0.0),
        (
            "just past 20°",
            20.5,
            0.0,
            0.0,
            15.0 / (1.0 + 2.0 * 0.5 / 140.0),
        ),
        ("90° off", 90.0, 0.0, 0.0, 15.0 / (1.0 + 2.0 * 70.0 / 140.0)),
        (
            "90° the other way",
            0.0,
            90.0,
            0.0,
            15.0 / (1.0 + 2.0 * 70.0 / 140.0),
        ),
        ("160° off", 160.0, 0.0, 0.0, 5.0),
        ("behind", 180.0, 0.0, 0.0, 5.0),
        ("just inside 20° across the wrap", 170.5, -170.0, 0.0, 0.0),
        ("behind, faster cruise", 180.0, 0.0, 10.0, 5.0),
        ("behind, slower cruise", 180.0, 0.0, 3.0, 3.0),
        ("ahead, cruise kept", 0.0, 0.0, 10.0, 10.0),
    ];
    for (case, bearing_deg, heading_deg, cruise_mps, expected_mps) in cases {
        let capped =
            default_cut().cap(&waypoint_at(bearing_deg, cruise_mps), &heading(heading_deg));
        let got = capped.cruise_speed_mps;
        check!(
            (got - expected_mps).abs() < 1e-4,
            "{case}: {got}, expected {expected_mps}"
        );
    }
}

#[test]
fn leaves_the_waypoint_alone_when_off_or_unknown() {
    let off = OffNoseSpeed::new(OffNoseSpeedConfig { speed_mps: 0.0 });
    let behind = waypoint_at(180.0, 0.0);
    // (case, cut, vehicle)
    let nan_position = VehicleState {
        position_m: Vec3::new(f32::NAN, 0.0, 0.0),
        ..Default::default()
    };
    let cases = [
        ("off", off, heading(0.0)),
        ("NaN heading", default_cut(), heading(f32::NAN)),
        ("NaN position", default_cut(), nan_position),
    ];
    for (case, cut, state) in cases {
        check!(cut.cap(&behind, &state) == behind, "{case}");
    }
}

#[test]
fn keeps_the_rest_of_the_waypoint() {
    let waypoint = WaypointCommand {
        velocity_mps: Some(Vec2::new(-3.0, 1.0)),
        altitude_m: 30.0,
        ..waypoint_at(180.0, 0.0)
    };
    let capped = default_cut().cap(&waypoint, &heading(0.0));
    check!(capped.position_m == waypoint.position_m);
    check!(capped.velocity_mps == waypoint.velocity_mps);
    assert_relative_eq!(capped.altitude_m, 30.0);
}

#[test]
fn rejects_a_negative_or_non_finite_speed() {
    for speed_mps in [-1.0, f32::NAN, f32::INFINITY] {
        let mut cut = default_cut();
        check!(
            cut.set_config(OffNoseSpeedConfig { speed_mps }).is_err(),
            "{speed_mps}"
        );
        check!(cut.config() == &OffNoseSpeedConfig::default());
    }
}
