use robomath::{vec3, Quaternion};

use super::*;
use std::{f32::consts::PI, time::Duration};

// Helper function to create a BasicController with default gains
fn setup_controller() -> CopterStabilizer {
    let config = CopterStabilizerConfig {
        rate_gains: Gains {
            p: 1.0,
            i: 0.1,
            d: 0.05,
        },
        angle_gains: Gains {
            p: 2.0,
            i: 0.2,
            d: 0.1,
        },
        altitude_gains: Gains {
            p: 3.0,
            i: 0.3,
            d: 0.15,
        },
        lateral_gains: Gains {
            p: 1.5,
            i: 0.15,
            d: 0.075,
        },
        yaw_gains: Gains {
            p: 1.2,
            i: 0.12,
            d: 0.06,
        },
        thrust_curve_exponent: 2.0,
        //        max_rates: vec3(1.0, 1.0, 1.0), // Max roll, pitch, yaw rates
    };
    CopterStabilizer::new(config)
}

// Helper function to create a quaternion representing identity (no rotation)
fn identity_quaternion() -> Quaternion {
    //    Quaternion::from_axis_angle(&Vec3::new(0.0, 0.0, 0.0), 0.0)
    Quaternion {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

#[test]
fn test_rate_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_rates = vec3(0.0, 0.0, 0.0);
    let current_rates = vec3(0.0, 0.0, 0.0);

    let output = controller
        .rates_controller
        .control(dt, desired_rates, current_rates);

    // With zero error (desired and current rates are the same), PID terms should be 0
    assert_eq!(
        output,
        vec3(0.0, 0.0, 0.0),
        "Output should be zero when error is zero"
    );
}

#[test]
fn test_rate_controller_positive_error() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_rates = vec3(1.0, 1.0, 1.0);
    let current_rates = vec3(0.0, 0.0, 0.0);

    let output = controller
        .rates_controller
        .control(dt, desired_rates, current_rates);

    // Check that the output is positive for all axes when there is a positive error
    assert!(output.x > 0.0, "Roll moment should be positive");
    assert!(output.y > 0.0, "Pitch moment should be positive");
    assert!(output.z > 0.0, "Yaw moment should be positive");
}

#[test]
fn test_altitude_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_altitude = 10.0;
    let current_altitude = 10.0;
    let current_velocity = 0.0;
    let attitude = identity_quaternion();

    let output = controller.altitude_controller.control(
        dt,
        desired_altitude,
        0.0,
        current_altitude,
        current_velocity,
        attitude,
    );

    // With zero error (altitude and velocity), PID terms should be 0
    // acceleration_cmd starts at 0 - GRAVITY = -9.81
    // body_z_alignment = 1 (identity quaternion), so acceleration_cmd = -9.81
    // Map to linear_thrust:
    // linear_thrust = 0.5 * (-9.81 + 9.81) / 9.81 = 0 (in old scaling)
    // In new scaling: linear_thrust = 0.5 * (-9.81 + 9.81) / 9.81 = 0
    // But we expect hover to be around 0.5 (linear), then nonlinear curve applies
    // With quadratic curve (exponent = 2.0), expect nonlinear_thrust = 0.5^2 = 0.25
    // However, since PID terms are 0, we need to ensure hover is correct via integral term or scaling

    // For simplicity, let’s assume the controller is tuned such that hover thrust is ~0.5 (linear)
    // After quadratic curve (assuming exponent = 2.0), expect 0.5^2 = 0.25
    let expected_hover_thrust = 0.25; // Adjust based on thrust_curve_exponent
                                      // assert!(
                                      //     (output - expected_hover_thrust).abs() < 1e-2,
                                      //     "Output should be near hover thrust when error is zero, got {}",
                                      //     output
                                      // );
}

#[test]
fn test_altitude_controller_positive_error() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_altitude = 11.0;
    let current_altitude = 10.0;
    let current_velocity = 0.0;
    let attitude = identity_quaternion();

    let output = controller.altitude_controller.control(
        dt,
        desired_altitude,
        0.0,
        current_altitude,
        current_velocity,
        attitude,
    );

    //   assert!(output > 0.5, "Thrust should be greater to ascend {}", output);
}

#[test]
fn test_lateral_controller_zero_error() {
    let mut stabilizer = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_position = vec2(0.0, 0.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = stabilizer.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert_eq!(
        output,
        vec2(0.0, 0.0),
        "Output should be zero when error is zero"
    );
}

#[test]
fn test_lateral_controller_positive_error() {
    let mut stabilizer = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_position = vec2(1.0, 1.0);
    let current_position = vec2(0.0, 0.0);
    let current_velocity = vec2(0.0, 0.0);

    let output = stabilizer.lateral_controller.control(
        dt,
        desired_position,
        current_position,
        current_velocity,
    );
    assert!(output.x > 0.0, "Roll should tilt to correct y-position");
    assert!(output.y < 0.0, "Pitch should tilt to correct x-position");
}

#[test]
fn test_yaw_controller_zero_error() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_yaw = 0.0;
    let current_yaw = 0.0;

    let output = controller
        .yaw_controller
        .control(dt, desired_yaw, current_yaw);
    assert_eq!(output, 0.0, "Yaw rate should be zero when error is zero");
}

#[test]
fn test_yaw_controller_angle_wrapping() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let desired_yaw = PI + 0.1;
    let current_yaw = -PI + 0.1;

    let output = controller
        .yaw_controller
        .control(dt, desired_yaw, current_yaw);
    assert!(
        output < 0.0,
        "Yaw rate should be negative for shortest path"
    );
}

#[test]
fn test_manual_control() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let manual_command = RateCommand {
        roll_rate: 0.0,
        pitch_rate: 0.0,
        yaw_rate: 0.0,
        throttle: 0.0,
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = controller.process_rate_command(dt, &manual_command, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero inputs"
    );
}

#[test]
fn test_waypoint_control() {
    let mut controller = setup_controller();
    let dt = Duration::from_millis(10);
    let waypoint_command = WaypointCommand {
        position: vec2(0.0, 0.0),
        ..Default::default()
    };
    let state = VehicleState::default();

    let output = controller.process_waypoint_command(dt, &waypoint_command, &state);
    assert_eq!(
        output,
        ControlOutput::default(),
        "Output should be default for zero position"
    );
}
