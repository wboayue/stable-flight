use assert2::check;

use super::*;
use crate::nalgebra::Matrix2;

/// A wind blowing toward `toward_deg` (from north, clockwise) at `speed_mps`, known to within
/// `sd_mps` on each axis.
fn wind(toward_deg: f32, speed_mps: f32, sd_mps: f32) -> WindEstimate {
    let toward_rad = toward_deg.to_radians();
    WindEstimate {
        velocity_mps: Vec2::new(libm::cosf(toward_rad), libm::sinf(toward_rad)) * speed_mps,
        covariance_m2ps2: Matrix2::identity() * sd_mps * sd_mps,
    }
}

fn deg(rad: Option<f32>) -> Option<f32> {
    rad.map(|r| wrap_angle(r).to_degrees())
}

/// (case, min speed m/s, wind, vehicle heading deg, expected heading deg)
type Row = (&'static str, f32, Option<WindEstimate>, f32, Option<f32>);

/// The first heading picked: 90° off the wind on the side nearer the heading, or none for an
/// off switch, a missing, slow, uncertain, or non-finite wind.
#[test]
fn picks_the_wingtip_heading_nearer_the_vehicle() {
    let cases: [Row; 9] = [
        // Blowing toward 217° (from 37°, RealFlight's): 127° or 307°.
        (
            "belly 3° off the wind",
            1.0,
            Some(wind(217.0, 2.0, 0.1)),
            40.0,
            Some(127.0),
        ),
        (
            "nearer the other wingtip",
            1.0,
            Some(wind(217.0, 2.0, 0.1)),
            330.0,
            Some(-53.0),
        ),
        ("off", 0.0, Some(wind(217.0, 2.0, 0.1)), 40.0, None),
        ("no estimate", 1.0, None, 40.0, None),
        (
            "slower than the least",
            1.0,
            Some(wind(217.0, 0.9, 0.01)),
            40.0,
            None,
        ),
        // 15° of 2 m/s is 0.52 m/s across it.
        (
            "direction just known",
            1.0,
            Some(wind(217.0, 2.0, 0.5)),
            40.0,
            Some(127.0),
        ),
        (
            "direction uncertain",
            1.0,
            Some(wind(217.0, 2.0, 0.55)),
            40.0,
            None,
        ),
        (
            "non-finite wind",
            1.0,
            Some(WindEstimate {
                velocity_mps: Vec2::new(f32::NAN, 1.0),
                ..wind(0.0, 2.0, 0.1)
            }),
            40.0,
            None,
        ),
        (
            "non-finite heading picks clockwise",
            1.0,
            Some(wind(0.0, 2.0, 0.1)),
            f32::NAN,
            Some(90.0),
        ),
    ];
    for (case, min_speed_mps, wind, heading_deg, expected) in cases {
        let mut heading = WindHeading::default();
        let target =
            deg(heading.target_rad(min_speed_mps, wind.as_ref(), heading_deg.to_radians()));
        let close = match (target, expected) {
            (Some(t), Some(e)) => (t - e).abs() < 1e-3,
            _ => target == expected,
        };
        check!(close, "{case}: {target:?} vs {expected:?}");
    }
}

/// Once picked, the side holds as the vehicle turns past the wind, the heading moves only
/// past the deadband, and a wind that stops allowing one keeps the latched heading.
#[test]
fn keeps_its_side_and_heading_through_small_changes() {
    let mut heading = WindHeading::default();
    let first = heading.target_rad(1.0, Some(&wind(0.0, 2.0, 0.1)), 0.3);
    check!(deg(first) == Some(90.0));

    // (case, wind toward deg, sd, vehicle heading deg, expected heading deg)
    let steps: [(&str, f32, f32, f32, f32); 4] = [
        ("vehicle now nearer the other side", 0.0, 0.1, -80.0, 90.0),
        ("wind turned 9°: inside the deadband", 9.0, 0.1, 90.0, 90.0),
        ("wind turned 11°: moves", 11.0, 0.1, 90.0, 101.0),
        (
            "uncertain: keeps the latched heading",
            60.0,
            5.0,
            90.0,
            101.0,
        ),
    ];
    for (case, toward_deg, sd_mps, heading_deg, expected) in steps {
        let target = deg(heading.target_rad(
            1.0,
            Some(&wind(toward_deg, 2.0, sd_mps)),
            heading_deg.to_radians(),
        ));
        check!(
            (target.unwrap_or(f32::NAN) - expected).abs() < 1e-3,
            "{case}: {target:?}"
        );
    }
}
