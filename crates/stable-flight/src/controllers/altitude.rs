use nalgebra::UnitQuaternion;

use crate::GRAVITY;
use crate::controllers::Gains;

/// Default [`AltitudeControllerConfig::hover_thrust`].
pub const DEFAULT_HOVER_THRUST: f32 = 0.5;

/// Floor on the body-z alignment (cos of the tilt) used for tilt compensation: past 60° the
/// boost stops growing, so a near-inverted estimate can't demand unbounded thrust.
const MIN_BODY_Z_ALIGNMENT: f32 = 0.5;

#[derive(Clone, Debug, PartialEq)]
pub struct AltitudeControllerConfig {
    /// PID gains from altitude error (m) to vertical acceleration (m/s², up).
    pub gains: Gains,
    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum climb rate in meters/second.
    /// This is used to limit the output thrust command.
    pub max_rate: f32,

    /// Linear thrust (before the stabilizer's thrust curve) that holds a level hover. The
    /// controller feeds it forward, so the integral only trims the residual. Airframe-specific.
    pub hover_thrust: f32,
}

impl Default for AltitudeControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 1.0,
                i: 0.1,
                d: 0.05,
            },
            // With feedforward the integral only trims hover; 5 m·s at I 0.2 is ±1 m/s².
            max_integral: 5.0,
            max_rate: 2.0, // 2 m/s for altitude
            hover_thrust: DEFAULT_HOVER_THRUST,
        }
    }
}

pub struct AltitudeController {
    config: AltitudeControllerConfig,
    integrated_error: f32,
}

impl Default for AltitudeController {
    fn default() -> Self {
        Self::new()
    }
}

impl AltitudeController {
    pub fn new() -> Self {
        AltitudeController::with_config(&AltitudeControllerConfig::default())
    }

    pub fn with_config(config: &AltitudeControllerConfig) -> Self {
        Self {
            config: config.clone(),
            integrated_error: 0.0,
        }
    }

    /// Controls the altitude of the vehicle by computing a normalized thrust command using a PID controller.
    ///
    /// The PID gives a vertical acceleration `a` (m/s², up); the thrust is
    /// `hover_thrust · (g + a) / (g · cos tilt)`, saturated to `[0, 1]`. Zero error is hover,
    /// and a tilted vehicle gets more thrust so its vertical component still balances gravity.
    ///
    /// # Arguments
    ///
    /// * `dt` - Time step since the last update (must be positive).
    /// * `desired_altitude` - Desired altitude in meters.
    /// * `desired_velocity` - Desired vertical velocity in m/s (currently unused, set to 0.0).
    /// * `estimated_altitude` - Current measured altitude in meters.
    /// * `estimated_velocity` - Current vertical velocity in m/s.
    /// * `estimated_attitude` - Current attitude as a quaternion.
    ///
    /// # Returns
    ///
    /// A normalized thrust command (0.0 to 1.0).
    pub fn control(
        &mut self,
        dt: f32,
        desired_altitude: f32,
        desired_velocity: f32,
        estimated_altitude: f32,
        estimated_velocity: f32,
        estimated_attitude: UnitQuaternion<f32>,
    ) -> f32 {
        let gains = &self.config.gains;
        let hover_thrust = self.config.hover_thrust;

        let altitude_error = desired_altitude - estimated_altitude;
        let velocity_error = desired_velocity - estimated_velocity;

        // A non-finite error (NaN/∞ altitude or velocity) must never enter the integral, where
        // it would stick (NaN survives `clamp`) and hold thrust at NaN → 0 for good. Neutral
        // for altitude is *hover*, not zero thrust, so return the hover setpoint instead.
        if !(altitude_error.is_finite() && velocity_error.is_finite()) {
            return hover_thrust;
        }

        let max_integral = self.config.max_integral;
        self.integrated_error =
            (self.integrated_error + altitude_error * dt).clamp(-max_integral, max_integral);

        let acceleration =
            gains.p * altitude_error + gains.d * velocity_error + gains.i * self.integrated_error;

        // Body z in world z: cos(roll)·cos(pitch) for a level-yaw-independent tilt.
        let body_z_alignment = estimated_attitude.to_rotation_matrix()[(2, 2)];
        let body_z_alignment = if body_z_alignment.is_finite() {
            body_z_alignment.max(MIN_BODY_Z_ALIGNMENT)
        } else {
            1.0
        };

        let linear_thrust = hover_thrust * (GRAVITY + acceleration) / (GRAVITY * body_z_alignment);
        linear_thrust.clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn level() -> UnitQuaternion<f32> {
        UnitQuaternion::identity()
    }

    /// Thrust is `hover · (g + a) / (g · cos tilt)` with the PID's acceleration `a`: zero error
    /// is hover, a bank adds thrust (yaw doesn't), the boost stops at 60°, and the result
    /// saturates to [0, 1].
    #[test]
    fn thrust_feeds_gravity_forward_and_boosts_for_tilt() {
        let deg = f32::to_radians;
        let attitude = |roll: f32, pitch: f32, yaw: f32| {
            UnitQuaternion::from_euler_angles(deg(roll), deg(pitch), deg(yaw))
        };
        let hover = 0.4;
        let cos = |d: f32| d.to_radians().cos();
        // (altitude error m, climb rate m/s, attitude, expected thrust); P 1, D 0.5, no I
        let cases = [
            (0.0, 0.0, level(), hover),
            (0.0, 0.0, attitude(0.0, 0.0, 120.0), hover),
            (0.0, 0.0, attitude(30.0, 0.0, 0.0), hover / cos(30.0)),
            (
                0.0,
                0.0,
                attitude(-20.0, 20.0, 45.0),
                hover / (cos(20.0) * cos(20.0)),
            ),
            (0.0, 0.0, attitude(75.0, 0.0, 0.0), hover / 0.5), // floored at 60°
            (0.0, 0.0, attitude(170.0, 0.0, 0.0), hover / 0.5), // inverted: same floor
            (1.0, 0.0, level(), hover * (GRAVITY + 1.0) / GRAVITY),
            (-1.0, 0.0, level(), hover * (GRAVITY - 1.0) / GRAVITY),
            (0.0, 2.0, level(), hover * (GRAVITY - 1.0) / GRAVITY), // climbing: damped
            (100.0, 0.0, level(), 1.0),                             // saturates high
            (-100.0, 0.0, level(), 0.0),                            // and low
        ];
        for (error, climb, attitude, expected) in cases {
            let mut controller = AltitudeController::with_config(&AltitudeControllerConfig {
                gains: Gains {
                    p: 1.0,
                    i: 0.0,
                    d: 0.5,
                },
                hover_thrust: hover,
                ..Default::default()
            });
            let thrust = controller.control(0.004, 10.0 + error, 0.0, 10.0, climb, attitude);
            approx::assert_relative_eq!(thrust, expected, epsilon = 1e-5);
        }
    }

    /// The integral accumulates `error · dt` and stops at `max_integral`.
    #[test]
    fn integral_is_clamped_to_max_integral() {
        let mut controller = AltitudeController::with_config(&AltitudeControllerConfig {
            gains: Gains {
                p: 0.0,
                i: 1.0,
                d: 0.0,
            },
            max_integral: 2.0,
            hover_thrust: 0.5,
            ..Default::default()
        });
        for _ in 0..1_000 {
            controller.control(0.01, 20.0, 0.0, 10.0, 0.0, level());
        }
        assert!(controller.integrated_error == 2.0);
        let thrust = controller.control(0.01, 10.0, 0.0, 10.0, 0.0, level());
        approx::assert_relative_eq!(thrust, 0.5 * (GRAVITY + 2.0) / GRAVITY, epsilon = 1e-5);
    }

    #[test]
    fn non_finite_estimate_holds_hover_and_does_not_poison_later_ticks() {
        let nan = f32::NAN;
        // (estimated altitude, estimated velocity)
        let cases = [(nan, 0.0), (10.0, nan), (f32::INFINITY, 0.0), (nan, nan)];
        for (altitude, velocity) in cases {
            let mut controller = AltitudeController::new();
            let during = controller.control(0.004, 20.0, 0.0, altitude, velocity, level());
            assert!(
                during == DEFAULT_HOVER_THRUST,
                "({altitude}, {velocity}) gave {during}"
            );

            let after = controller.control(0.004, 20.0, 0.0, 10.0, 0.0, level());
            let fresh = AltitudeController::new().control(0.004, 20.0, 0.0, 10.0, 0.0, level());
            assert!(
                after == fresh,
                "after ({altitude}, {velocity}): {after} vs fresh {fresh}"
            );
        }
    }
}
