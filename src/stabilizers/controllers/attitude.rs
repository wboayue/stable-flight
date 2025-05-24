use std::f32::consts::FRAC_PI_4;

use robomath::{vec2, Vec2};

use super::super::Gains;

/// Configures the attitude controller for a vehicle.
/// Defines PID gains, maximum angles, and maximum angular rates for roll and pitch control.
/// Yaw is typically handled separately by a dedicated yaw controller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeControllerConfig {
    /// PID gains for roll and pitch control (Kp, Ki, Kd).
    pub gains: Gains,

    /// Maximum angles in radians for roll (x) and pitch (y) (e.g., π/4 = 45°).
    /// Limits the vehicle's tilt in each axis.
    pub max_angles: Vec2<f32>,

    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum angular rates in radians/second for roll (x) and pitch (y).
    /// Limits the output rates sent to the rate controller.
    pub max_rate: Vec2<f32>,
}

impl Default for AttitudeControllerConfig {
    /// Provides a default configuration with reasonable starting values.
    /// Note: PID gains (P=7.0, I=0.0, D=0.01) and max_rate (2 rad/s) may need tuning
    /// for specific vehicles.
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 3.0,   // Proportional gain
                i: 0.001, // Integral gain (disabled by default)
                d: 1.0,   // Derivative gain
            },
            max_angles: vec2(FRAC_PI_4, FRAC_PI_4), // 45° for roll and pitch
            max_integral: 100.0,                    // Arbitrary limit
            max_rate: vec2(2.0, 2.0),               // 2 rad/s (~114°/s) for roll and pitch
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct AttitudeController {
    gains: Gains,
    integral_term: Vec2<f32>, // Accumulated error (roll, pitch)
    max_integral: f32,
    last_error: Vec2<f32>, // Previous error for derivative calculation
    max_angles: Vec2<f32>, // Max angles in radians (roll, pitch)
    max_rate: Vec2<f32>,   // Max angular rates in rad/s (roll, pitch)
}

impl Default for AttitudeController {
    /// Provides a default attitude controller with default configuration.
    /// Note: PID gains (P=7.0, I=0.0, D=0.01) and max_rate (2 rad/s) may need tuning
    /// for specific vehicles.
    fn default() -> Self {
        Self::with_config(&AttitudeControllerConfig::default())
    }
}

impl AttitudeController {
    /// Initializes a new attitude controller with the given configuration.
    pub fn new() -> Self {
        Self::default()
    }

    /// Initializes a new attitude controller with the given configuration.
    pub fn with_config(config: &AttitudeControllerConfig) -> Self {
        assert!(
            config.max_angles.x >= 0.0 && config.max_angles.y >= 0.0,
            "Maximum angles must be non-negative"
        );
        assert!(
            config.max_rate.x >= 0.0 && config.max_rate.y >= 0.0,
            "Maximum rates must be non-negative"
        );
        Self {
            gains: config.gains,
            integral_term: Vec2::default(),
            max_integral: config.max_integral,
            last_error: Vec2::default(),
            max_angles: config.max_angles,
            max_rate: config.max_rate,
        }
    }

    /// Computes desired angular rates to achieve target roll and pitch angles.
    ///
    /// Uses a PID controller to convert angle errors into angular rates (rad/s)
    /// for the rate control loop. Input angles are scaled from [-1,1] to
    /// [-max_angles.x, max_angles.x] for roll and [-max_angles.y, max_angles.y] for pitch.
    /// Output rates are clamped to [-max_rate.x, max_rate.x] for roll and
    /// [-max_rate.y, max_rate.y] for pitch.
    ///
    /// # Arguments
    /// * `dt` - Time step since last update in seconds (must be positive)
    /// * `desired_angles` - Target roll/pitch as normalized values in [-1,1]
    /// * `estimated_angles` - Current roll/pitch angles in radians
    ///
    /// # Returns
    /// Desired angular rates (roll rate, pitch rate) in radians/second,
    /// clamped to configured max_rate limits
    ///
    /// # Panics
    /// * If `dt` is not positive
    /// * If `desired_angles` components are not in [-1,1]
    pub fn control(
        &mut self,
        dt: f32,
        desired_angles: Vec2<f32>,
        estimated_angles: Vec2<f32>,
    ) -> Vec2<f32> {
        assert!(dt > 0.0, "Time step must be positive");
        assert!(
            desired_angles.x >= -1.0 && desired_angles.x <= 1.0,
            "Desired roll angle ({}) must be in [-1, 1]",
            desired_angles.x
        );
        assert!(
            desired_angles.y >= -1.0 && desired_angles.y <= 1.0,
            "Desired pitch angle ({}) must be in [-1, 1]",
            desired_angles.y
        );

        // Scale normalized inputs to actual angle setpoints in radians
        let desired_angles = desired_angles * self.max_angles;

        // Calculate error (setpoint - actual)
        let error = desired_angles - estimated_angles;

        // Derivative term: rate of change of error
        let error_derivative = (error - self.last_error) / dt;

        // Integral term with anti-windup
        self.integral_term = (self.integral_term + dt * error)
            .clamp(-self.max_integral, self.max_integral);

        // PID calculation: Kp×error + Ki×∫error + Kd×d(error)/dt
        let Gains { p, i, d } = self.gains;
        let rates = p * error + i * self.integral_term + d * error_derivative;

        self.last_error = error;

        // Scale output to configured maximum rates
        vec2(
            rates.x.clamp(-self.max_rate.x, self.max_rate.x),
            rates.y.clamp(-self.max_rate.y, self.max_rate.y),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper function to create a default controller for tests
    fn default_controller() -> AttitudeController {
        AttitudeController::default()
    }

    #[test]
    fn test_new_with_default_config() {
        // Assemble
        let config = AttitudeControllerConfig::default();

        // Activate
        let controller = AttitudeController::with_config(&config);

        // Assert
        assert_eq!(controller.gains.p, 3.0);
        // assert_eq!(controller.gains.i, 0.0);
        // assert_eq!(controller.gains.d, 0.01);
        assert_eq!(controller.max_angles, vec2(FRAC_PI_4, FRAC_PI_4));
        assert_eq!(controller.max_rate, vec2(2.0, 2.0));
        assert_eq!(controller.max_integral, 100.0);
        assert_eq!(controller.integral_term, vec2(0.0, 0.0));
        assert_eq!(controller.last_error, vec2(0.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "Maximum angles must be non-negative")]
    fn test_new_panics_on_negative_max_angles() {
        // Assemble
        let config = AttitudeControllerConfig {
            max_angles: vec2(-1.0, FRAC_PI_4),
            ..AttitudeControllerConfig::default()
        };

        // Activate
        let _controller = AttitudeController::with_config(&config);

        // Assert (panic expected, no assertions needed)
    }

    #[test]
    #[should_panic(expected = "Maximum rates must be non-negative")]
    fn test_new_panics_on_negative_max_rate() {
        // Assemble
        let config = AttitudeControllerConfig {
            max_rate: vec2(2.0, -2.0),
            ..AttitudeControllerConfig::default()
        };

        // Activate
        let _controller = AttitudeController::with_config(&config);

        // Assert (panic expected, no assertions needed)
    }

    #[test]
    fn test_control_zero_error() {
        // Assemble
        let mut controller = default_controller();
        let dt = 0.1;
        let desired_angles = vec2(0.0, 0.0); // No tilt requested
        let estimated_angles = vec2(0.0, 0.0); // Already at target

        // Activate
        let rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert
        assert_eq!(rates, vec2(0.0, 0.0)); // No error, no rate output
        assert_eq!(controller.integral_term, vec2(0.0, 0.0)); // No integral buildup
        assert_eq!(controller.last_error, vec2(0.0, 0.0)); // Error remains zero
    }

    #[test]
    fn test_control_proportional_response() {
        // Assemble
        let mut controller = default_controller();
        controller.gains = Gains {
            p: 2.0, // Proportional gain
            i: 0.0,
            d: 0.0,
        }; // No integral or derivative initially
        let dt = 0.1;
        let desired_angles = vec2(1.0, -1.0); // Max roll right, max pitch down
        let estimated_angles = vec2(0.0, 0.0); // Starting at zero

        // Activate
        let rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert
        let expected_error = vec2(FRAC_PI_4, -FRAC_PI_4); // 45° in radians
        let expected_rates = controller.gains.p * expected_error; // P=7.0, I=0, D=0 initially

        assert!((rates.x - expected_rates.x).abs() < 1e-6);
        assert!((rates.y - expected_rates.y).abs() < 1e-6);
        assert_eq!(controller.last_error, expected_error);
        assert_eq!(controller.integral_term, dt * expected_error);
        // Small integral buildup
    }

    #[test]
    fn test_control_rate_clamping() {
        // Assemble
        let mut controller = AttitudeController::with_config(&AttitudeControllerConfig {
            gains: Gains {
                p: 100.0,
                i: 0.0,
                d: 0.0,
            }, // High P gain to exceed max_rate
            max_rate: vec2(2.0, 2.0),
            ..AttitudeControllerConfig::default()
        });
        let dt = 0.1;
        let desired_angles = vec2(1.0, 1.0); // Max roll and pitch
        let estimated_angles = vec2(0.0, 0.0);

        // Activate
        let rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert
        assert_eq!(rates, vec2(2.0, 2.0)); // Clamped to max_rate
    }

    #[test]
    fn test_control_integral_buildup() {
        // Assemble
        let mut controller = AttitudeController::with_config(&AttitudeControllerConfig {
            gains: Gains {
                p: 1.0,
                i: 1.0,
                d: 0.0,
            }, // Enable integral term
            ..AttitudeControllerConfig::default()
        });
        let dt = 0.1;
        let desired_angles = vec2(0.5, 0.5);
        let estimated_angles = vec2(0.0, 0.0);

        // Activate
        let _rates1 = controller.control(dt, desired_angles, estimated_angles);
        let rates2 = controller.control(dt, desired_angles, estimated_angles);

        // Assert
        let error = vec2(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);
        let expected_integral = dt * 2.0 * error; // Two steps
        assert_eq!(controller.integral_term, expected_integral);
        let expected_rates = 1.0 * error + 1.0 * expected_integral; // P + I terms
        assert!((rates2.x - expected_rates.x).abs() < 1e-6);
        assert!((rates2.y - expected_rates.y).abs() < 1e-6);
    }

    #[test]
    fn test_control_integral_clamping() {
        // Assemble
        let mut controller = AttitudeController::with_config(&AttitudeControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 8000.0,
                d: 0.0,
            }, // High I to hit max_integral
            max_integral: 1.0, // Low limit for testing
            ..AttitudeControllerConfig::default()
        });
        let dt = 2.0; // Large dt to quickly reach max_integral
        let desired_angles = vec2(1.0, 1.0);
        let estimated_angles = vec2(0.0, 0.0);

        // Activate
        let rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert
        assert_eq!(controller.integral_term, vec2(1.0, 1.0)); // Clamped to max_integral
        assert_eq!(rates, vec2(2.0, 2.0));
    }

    #[test]
    fn test_control_derivative_response() {
        // Assemble
        let mut controller = AttitudeController::with_config(&AttitudeControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 0.0,
                d: 0.5,
            }, // Only D term
            ..AttitudeControllerConfig::default()
        });
        let dt = 0.1;
        let desired_angles = vec2(1.0, 1.0);
        let estimated_angles1 = vec2(0.0, 0.0);
        let estimated_angles2 = vec2(0.5 * FRAC_PI_4, 0.5 * FRAC_PI_4);

        // Activate
        let _rates1 = controller.control(dt, desired_angles, estimated_angles1);
        let rates2 = controller.control(dt, desired_angles, estimated_angles2);

        // Assert
        let error1 = vec2(FRAC_PI_4, FRAC_PI_4);
        let error2 = error1 - estimated_angles2;
        let derivative = (error2 - error1) / dt;
        let expected_rates = controller.gains.d * derivative; // D gain = 1.0
        assert!((rates2.x - expected_rates.x).abs() < 1e-6);
        assert!((rates2.y - expected_rates.y).abs() < 1e-6);
    }

    #[test]
    #[should_panic(expected = "Time step must be positive")]
    fn test_control_panics_on_zero_dt() {
        // Assemble
        let mut controller = default_controller();
        let dt = 0.0;
        let desired_angles = vec2(0.0, 0.0);
        let estimated_angles = vec2(0.0, 0.0);

        // Activate
        let _rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert (panic expected)
    }

    #[test]
    #[should_panic(expected = "Desired roll angle (2) must be in [-1, 1]")]
    fn test_control_panics_on_invalid_desired_angles() {
        // Assemble
        let mut controller = default_controller();
        let dt = 0.1;
        let desired_angles = vec2(2.0, 0.0); // Out of range
        let estimated_angles = vec2(0.0, 0.0);

        // Activate
        let _rates = controller.control(dt, desired_angles, estimated_angles);

        // Assert (panic expected)
    }
}
