use super::*;
use stable_flight::Vec3;
use stable_flight::controllers::{LateralControllerConfig, YawControllerConfig};

#[test]
fn quadcopter_x_maps_its_four_motors_validly() {
    let mapping = quadcopter_x().mapping;

    assert_eq!(mapping.validate(), Ok(()));
    assert!(mapping.motors[..4].iter().all(Option::is_some));
    assert!(mapping.motors[4..].iter().all(Option::is_none));
    assert!(mapping.servos.iter().all(Option::is_none));
}

/// Pins the flight-checked QuadCopterX set (#62), so a retune (or a core default change it
/// inherits) shows up in review.
#[test]
fn quadcopter_x_gains_are_the_flight_checked_set() {
    let airframe = quadcopter_x();
    let gains = |p, i, d| Gains { p, i, d };

    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: gains(0.5, 0.2, 0.004),
            pitch_gains: gains(0.5, 0.2, 0.004),
            yaw_gains: gains(0.6, 0.6, 0.0),
            max_integral: 1.0,
        },
        attitude: AttitudeControllerConfig {
            roll_gains: gains(3.0, 0.0, 0.1),
            pitch_gains: gains(3.0, 0.0, 0.1),
            max_integral: 100.0,
            max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
            max_rate_radps: Vec2::new(3.0, 3.0),
        },
        yaw: YawControllerConfig {
            gains: gains(2.5, 0.0, 0.06),
            max_integral: 100.0,
            max_rate_radps: 2.0,
        },
        full_scale_rates_radps: Vec3::new(3.0, 3.0, 3.0),
        thrust_curve_exponent: 2.0,
    };
    assert_eq!(airframe.stabilizer, stabilizer, "stabilizer");

    let guidance = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            gains: gains(1.5, 0.2, 2.0),
            max_integral: 5.0,
            hover_thrust: 0.82,
        },
        lateral: LateralControllerConfig {
            position_p: 0.5,
            velocity: gains(1.26, 0.0, 0.0),
            max_integral: 100.0,
        },
        navigation_period_s: 0.02,
        max_tilt_rad: 27.0_f32.to_radians(),
        max_stick_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
        max_descent_mps: 3.0,
        land_speed_mps: 0.7,
    };
    assert_eq!(airframe.guidance, guidance, "guidance");
}

/// Steady yaw moment (normalized, nose right) QuadCopterX needs to hover in RealFlight,
/// measured in #192: a mean motor yaw split of about 0.10 over the core `TableMixer::quad_x`'s yaw
/// scaling of 0.25. Re-measure if that scaling changes.
const MEASURED_HOVER_YAW_MOMENT: f32 = 0.40;

/// The flown rate gains hold zero yaw rate against the model's hover yaw imbalance with 25%
/// margin; at the old yaw I of 0.2, acro drifted ~19°/s CCW (#192).
#[test]
fn quadcopter_x_holds_zero_yaw_rate_against_the_hover_imbalance() {
    let body_rate = quadcopter_x().stabilizer.body_rate;
    for disturbance in [MEASURED_HOVER_YAW_MOMENT, 1.25 * MEASURED_HOVER_YAW_MOMENT] {
        let rate = stable_flight::testing::settled_yaw_rate(&body_rate, disturbance);
        assert!(
            rate.abs() < 1e-3,
            "settled at {rate} rad/s against {disturbance}"
        );
    }
}
