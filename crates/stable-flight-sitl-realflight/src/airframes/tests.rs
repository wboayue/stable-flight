use super::*;

#[test]
fn quadcopter_x_maps_each_motor_to_its_own_channel() {
    let airframe = quadcopter_x();

    let mut channels: Vec<usize> = airframe.mapping.motors.iter().flatten().copied().collect();
    channels.sort_unstable();

    assert_eq!(channels, [0, 1, 2, 3]);
}

/// Pins the flight-checked QuadCopterX gains (#62), so a retune shows up in review.
#[test]
fn quadcopter_x_gains_are_the_flight_checked_set() {
    let airframe = quadcopter_x();
    let gains = |p, i, d| Gains { p, i, d };
    let rates = &airframe.stabilizer.rates_controller;

    // (loop, actual, expected)
    let cases = [
        ("roll/pitch rate", rates.gains, gains(0.5, 0.2, 0.004)),
        ("yaw rate", rates.yaw_gains, gains(0.6, 0.2, 0.0)),
        (
            "heading",
            airframe.stabilizer.yaw_gains,
            gains(2.5, 0.0, 0.06),
        ),
        (
            "lateral",
            airframe.guidance.lateral_gains,
            gains(0.8, 0.0, 1.6),
        ),
        (
            "altitude",
            airframe.guidance.altitude_gains,
            gains(1.5, 0.2, 2.0),
        ),
    ];
    for (name, actual, expected) in cases {
        assert_eq!(actual, expected, "{name} gains");
    }
    assert_eq!(rates.max_integral, 1.0);
    assert_eq!(airframe.guidance.hover_thrust, 0.82);

    let attitude = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: gains(3.0, 0.0, 0.1),
        max_integral: 100.0,
        max_angles: Vec2::new(PI / 4.0, PI / 4.0),
        max_rate: Vec2::new(3.0, 3.0),
    });
    assert_eq!(
        airframe.stabilizer.attitude_controller, attitude,
        "attitude loop"
    );
}
