use super::*;
use stable_flight::Vec3;
use stable_flight::controllers::{LateralControllerConfig, YawControllerConfig};

#[test]
fn quadcopter_x_maps_each_motor_to_its_own_channel() {
    let airframe = quadcopter_x();

    let mut channels: Vec<usize> = airframe.mapping.motors.iter().flatten().copied().collect();
    channels.sort_unstable();

    assert_eq!(channels, [0, 1, 2, 3]);
}

/// Pins the flight-checked QuadCopterX set (#62), so a retune (or a core default change it
/// inherits) shows up in review.
#[test]
fn quadcopter_x_gains_are_the_flight_checked_set() {
    let airframe = quadcopter_x();
    let gains = |p, i, d| Gains { p, i, d };

    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            gains: gains(0.5, 0.2, 0.004),
            yaw_gains: gains(0.6, 0.6, 0.0),
            max_integral: 1.0,
        },
        attitude: AttitudeControllerConfig {
            gains: gains(3.0, 0.0, 0.1),
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
    };
    assert_eq!(airframe.guidance, guidance, "guidance");
}

/// Steady yaw moment (normalized) QuadCopterX needs to hover in RealFlight: the mean motor
/// yaw split of about 0.10 over the mixer's yaw scaling of 0.25, measured in #192.
const MEASURED_HOVER_YAW_MOMENT: f32 = 0.40;

/// The rate loop's yaw integral can cancel the model's hover yaw imbalance with margin; below
/// it, acro drifts at `(imbalance − I · max_integral) / P` (#192).
#[test]
fn quadcopter_x_yaw_integral_covers_the_hover_imbalance() {
    let body_rate = quadcopter_x().stabilizer.body_rate;
    let authority = body_rate.yaw_gains.i * body_rate.max_integral;
    assert!(
        authority >= 1.25 * MEASURED_HOVER_YAW_MOMENT,
        "yaw integral authority {authority} vs measured {MEASURED_HOVER_YAW_MOMENT}"
    );
}
