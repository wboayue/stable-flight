use assert2::check;

use super::*;

#[test]
fn heartbeat_reports_the_arm_state() {
    for armed in [false, true] {
        let base_mode = heartbeat(armed).base_mode;
        check!(base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED) == armed);
        check!(base_mode.contains(MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
    }
}

#[test]
fn maps_normalized_thrust_to_throttle_percent() {
    // (thrust, expected percent)
    let cases = [
        (0.0, 0),
        (0.5, 50),
        (0.734, 73),
        (1.0, 100),
        (-0.2, 0),
        (1.7, 100),
        (f32::NAN, 0),
    ];

    for (thrust, expected) in cases {
        check!(throttle_percent(thrust) == expected, "thrust {thrust}");
    }
}

fn state_with(position_m: [f32; 3], velocity_mps: [f32; 3]) -> VehicleState {
    VehicleState {
        position_m: position_m.into(),
        velocity_mps: velocity_mps.into(),
        ..Default::default()
    }
}

#[test]
fn local_position_is_ned_down_positive() {
    // (position_m, velocity_mps (north, east, up), expected (z, vz))
    let cases = [
        ([3.0, 4.0, 100.0], [1.0, 2.0, 5.0], (-100.0, -5.0)), // climbing at 100 m
        ([0.0, 0.0, -2.0], [0.0, 0.0, -1.5], (2.0, 1.5)),     // below datum, descending
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], (0.0, 0.0)),
    ];

    for (position_m, velocity_mps, (z, vz)) in cases {
        let data = local_position_ned(&state_with(position_m, velocity_mps), 7);
        check!(data.time_boot_ms == 7);
        check!(
            (data.x, data.y) == (position_m[0], position_m[1]),
            "north/east unchanged"
        );
        check!((data.vx, data.vy) == (velocity_mps[0], velocity_mps[1]));
        check!(data.z == z, "{position_m:?}");
        check!(data.vz == vz, "{velocity_mps:?}");
    }
}

#[test]
fn attitude_reports_euler_angles_with_yaw_in_minus_pi_to_pi() {
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_6, PI};

    // (roll, pitch, heading, expected yaw)
    let cases = [
        (FRAC_PI_6, 0.0, 0.0, 0.0),              // 30° right roll
        (0.0, 0.2, FRAC_PI_2, FRAC_PI_2),        // nose up, facing east
        (0.0, 0.0, 3.0 * FRAC_PI_2, -FRAC_PI_2), // west, from a [0, 2π) backend
        (0.0, 0.0, -FRAC_PI_2, -FRAC_PI_2),      // west, from a [-π, π) backend
        (0.0, 0.0, PI, -PI),                     // south wraps to -π
        (0.0, 0.0, 5.0 * FRAC_PI_2, FRAC_PI_2),  // east, after a full turn
    ];

    for (roll, pitch, heading, yaw) in cases {
        let state = VehicleState {
            roll_rad: roll,
            pitch_rad: pitch,
            heading_rad: heading,
            angular_velocity_radps: [0.1, -0.2, 0.3].into(),
            ..Default::default()
        };
        let data = attitude(&state, 0);
        check!((data.roll, data.pitch) == (roll, pitch));
        approx::assert_relative_eq!(data.yaw, yaw, epsilon = 1e-5);
        check!((data.rollspeed, data.pitchspeed, data.yawspeed) == (0.1, -0.2, 0.3));
    }
}

#[test]
fn vfr_hud_heading_is_whole_degrees_from_north() {
    use core::f32::consts::FRAC_PI_2;

    // (heading_rad, expected degrees)
    let cases = [
        (0.0, 0),
        (FRAC_PI_2, 90),
        (-FRAC_PI_2, 270),
        (359.6_f32.to_radians(), 0), // rounds up to 360, wraps
        (359.4_f32.to_radians(), 359),
        (-1e-7, 0),
        (f32::NAN, 0),
    ];

    for (heading_rad, expected) in cases {
        check!(
            heading_deg(heading_rad) == expected,
            "heading {heading_rad}"
        );
    }
}

#[test]
fn vfr_hud_reports_horizontal_speed_climb_and_altitude() {
    // (velocity_mps (north, east, up), altitude_m, expected speed)
    let cases = [
        ([3.0, 4.0, 2.0], 120.0, 5.0),
        ([-6.0, 8.0, -1.0], 10.0, 10.0),
        ([0.0, 0.0, 3.0], 0.0, 0.0), // pure climb: no ground speed
    ];

    for (velocity_mps, altitude_m, speed) in cases {
        let state = state_with([0.0, 0.0, altitude_m], velocity_mps);
        let data = vfr_hud(&ControlOutput::new(0.5, Default::default()), &state);
        check!(data.groundspeed == speed, "{velocity_mps:?}");
        check!(data.airspeed == speed);
        check!(data.climb == velocity_mps[2]);
        check!(data.alt == altitude_m);
        check!(data.throttle == 50);
    }
}
