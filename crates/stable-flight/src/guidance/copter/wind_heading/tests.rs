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

/// The default config, with the wind heading on above `min_speed_mps`.
fn on(min_speed_mps: f32) -> CopterGuidanceConfig {
    CopterGuidanceConfig {
        wind_heading_min_mps: min_speed_mps,
        ..Default::default()
    }
}

fn deg(rad: Option<f32>) -> Option<f32> {
    rad.map(|r| wrap_angle(r).to_degrees())
}

/// (case, min speed m/s, wind, vehicle heading deg, expected heading deg)
type Row = (&'static str, f32, Option<WindEstimate>, f32, Option<f32>);

/// The first heading picked: 80° off where the wind comes from, on the side nearer the heading, or none for an
/// off switch, a missing, slow, uncertain, or non-finite wind.
#[test]
fn picks_the_wingtip_heading_nearer_the_vehicle() {
    let cases: [Row; 9] = [
        // Blowing toward 217° (from 37°, RealFlight's): 117° or 317°, 80° off 37°.
        (
            "belly 3° off the wind",
            1.0,
            Some(wind(217.0, 2.0, 0.1)),
            40.0,
            Some(117.0),
        ),
        (
            "nearer the other wingtip",
            1.0,
            Some(wind(217.0, 2.0, 0.1)),
            330.0,
            Some(-43.0),
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
            Some(117.0),
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
            Some(100.0),
        ),
    ];
    for (case, min_speed_mps, wind, heading_deg, expected) in cases {
        let mut heading = WindHeading::default();
        let target =
            deg(heading.target_rad(&on(min_speed_mps), wind.as_ref(), heading_deg.to_radians()));
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
    let first = heading.target_rad(&on(1.0), Some(&wind(0.0, 2.0, 0.1)), 0.3);
    check!(deg(first).is_some_and(|h| (h - 100.0).abs() < 1e-3));

    // (case, wind toward deg, sd, vehicle heading deg, expected heading deg)
    let steps: [(&str, f32, f32, f32, f32); 4] = [
        ("vehicle now nearer the other side", 0.0, 0.1, -80.0, 100.0),
        (
            "wind turned 9°: inside the deadband",
            9.0,
            0.1,
            100.0,
            100.0,
        ),
        ("wind turned 11°: moves", 11.0, 0.1, 100.0, 111.0),
        (
            "uncertain: keeps the latched heading",
            60.0,
            5.0,
            100.0,
            111.0,
        ),
    ];
    for (case, toward_deg, sd_mps, heading_deg, expected) in steps {
        let target = deg(heading.target_rad(
            &on(1.0),
            Some(&wind(toward_deg, 2.0, sd_mps)),
            heading_deg.to_radians(),
        ));
        check!(
            (target.unwrap_or(f32::NAN) - expected).abs() < 1e-3,
            "{case}: {target:?}"
        );
    }
}

/// The angle off the wind, the direction uncertainty, and the deadband come from the config.
#[test]
fn follows_the_configured_angle_uncertainty_and_deadband() {
    // (case, config, wind toward deg after a first pick toward 0°, sd m/s, expected deg)
    let rows: [(&str, CopterGuidanceConfig, f32, f32, Option<f32>); 4] = [
        (
            "square on at 90°",
            CopterGuidanceConfig {
                wind_heading_from_wind_rad: 90.0_f32.to_radians(),
                ..on(1.0)
            },
            0.0,
            0.1,
            Some(90.0),
        ),
        (
            "no deadband: 9° moves it",
            CopterGuidanceConfig {
                wind_heading_deadband_rad: 0.0,
                ..on(1.0)
            },
            9.0,
            0.1,
            Some(109.0),
        ),
        (
            "wider deadband: 11° doesn't",
            CopterGuidanceConfig {
                wind_heading_deadband_rad: 12.0_f32.to_radians(),
                ..on(1.0)
            },
            11.0,
            0.1,
            Some(100.0),
        ),
        (
            "looser uncertainty: a 1 m/s sd still turns it",
            CopterGuidanceConfig {
                wind_heading_max_sd_rad: 90.0_f32.to_radians(),
                ..on(1.0)
            },
            0.0,
            1.0,
            Some(100.0),
        ),
    ];
    for (case, config, toward_deg, sd_mps, expected) in rows {
        let mut heading = WindHeading::default();
        let first = deg(heading.target_rad(&config, Some(&wind(0.0, 2.0, sd_mps)), 0.3));
        let target = if toward_deg == 0.0 {
            first
        } else {
            deg(heading.target_rad(&config, Some(&wind(toward_deg, 2.0, sd_mps)), 0.3))
        };
        let close = match (target, expected) {
            (Some(t), Some(e)) => (t - e).abs() < 1e-3,
            _ => target == expected,
        };
        check!(close, "{case}: {target:?} vs {expected:?}");
    }
}
