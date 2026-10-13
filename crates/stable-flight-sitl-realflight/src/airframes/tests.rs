use super::*;
use stable_flight::Mixer;
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
            velocity: gains(1.26, 0.4, 0.0),
            max_integral: 12.5,
        },
        navigation_period_s: 0.02,
        max_tilt_rad: 27.0_f32.to_radians(),
        max_stick_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
        max_descent_mps: 3.0,
        land_speed_mps: 0.7,
        thrust: ThrustLimits {
            min_thrust: 0.0,
            sink_limit_mps: 0.0,
            sink_gain: 0.0,
            max_boost_thrust: 1.0,
        },
    };
    assert_eq!(airframe.guidance, guidance, "guidance");
}

/// Steady yaw moment (normalized, nose right) QuadCopterX needs to hover in RealFlight,
/// measured (#192): a mean motor yaw split of about 0.10 over the core `TableMixer::quad_x`'s
/// yaw scaling of 0.25. Re-measure if that scaling changes.
const MEASURED_HOVER_YAW_MOMENT: f32 = 0.40;

/// The flown rate gains hold zero yaw rate against the model's hover yaw imbalance with 25%
/// margin; with less integral authority, acro drifts in yaw.
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

#[test]
fn explora_vtol_maps_two_motors_and_two_elevons_validly() {
    let mapping = explora_vtol().mapping;

    assert_eq!(mapping.validate(), Ok(()));
    assert_eq!(mapping.motors[..2], [Some(5), Some(4)]);
    assert!(mapping.motors[2..].iter().all(Option::is_none));
    assert_eq!(mapping.servos[..2], [Some(0), Some(1)]);
    assert!(mapping.servos[2..].iter().all(Option::is_none));
}

/// RealFlight-level on its tail is nose-up hover: 90° pitch in the vehicle frame.
#[test]
fn explora_vtol_reads_nose_up_at_realflight_level() {
    let (roll, pitch, _) = stable_flight::euler_angles(&explora_vtol().body_rotation);
    assert!(roll.abs() < 1e-3);
    assert!((pitch - PI / 2.0).abs() < 1e-3, "pitch {pitch}");
}

/// Both motors at about 0.63 hold the eXplora's level hover in RealFlight (#123).
const MEASURED_HOVER_MOTOR: f32 = 0.63;

/// `hover_thrust`, through the stabilizer's thrust curve and the mixer, lands both motors on
/// the measured hover throttle, with the elevons centered.
#[test]
fn explora_vtol_hover_thrust_reaches_the_measured_hover_motor() {
    let airframe = explora_vtol();
    let curved = airframe
        .guidance
        .hover
        .altitude
        .hover_thrust
        .powf(airframe.stabilizer.hover.thrust_curve_exponent);
    let mut mixer = airframe.mixer;
    let output = mixer.mix(&stable_flight::ControlOutput::new(curved, Vec3::zeros()));
    for motor in &output.motors[..2] {
        assert!(
            (motor - MEASURED_HOVER_MOTOR).abs() < 0.01,
            "motors {:?}",
            output.motors
        );
    }
    assert!(output.servos[..2] == [0.0, 0.0]);
}

/// Each airframe's thrust floor sits below its hover thrust, so waypoint flight can descend
/// and a land can touch down.
#[test]
fn thrust_floor_stays_below_hover() {
    for (name, guidance) in [
        ("quadcopter_x", quadcopter_x().guidance),
        ("explora_vtol", explora_vtol().guidance.hover),
    ] {
        assert!(
            guidance.thrust.min_thrust < guidance.altitude.hover_thrust,
            "{name}: {guidance:?}"
        );
    }
}

/// The eXplora's tailsitter configs validate, and fit together: the hover side can fly the
/// whole forward ramp, the plane side the whole back ramp, and both share a thrust curve so a
/// handoff keeps the motor commands continuous.
#[test]
fn explora_vtol_transition_fits_both_sides() {
    let airframe = explora_vtol();
    let (guidance, stabilizer) = (airframe.guidance, airframe.stabilizer);
    assert_eq!(guidance.validate(), Ok(()));
    assert_eq!(stabilizer.validate(), Ok(()));
    let transition = guidance.transition;
    assert!(stabilizer.hover.attitude.max_angles_rad.y >= transition.forward_max_tilt_rad);
    assert!(stabilizer.plane.max_pitch_rad >= PI / 2.0 - transition.back_done_rad);
    assert_eq!(
        stabilizer.plane.thrust_curve_exponent,
        stabilizer.hover.thrust_curve_exponent
    );
    assert!(transition.forward_max_tilt_rad >= transition.forward_done_rad);
}
