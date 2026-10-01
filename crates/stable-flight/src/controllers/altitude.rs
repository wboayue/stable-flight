use nalgebra::UnitQuaternion;

use crate::controllers::Gains;

/// Linear thrust that holds altitude (zero net vertical acceleration); also the output for a
/// non-finite altitude or velocity error.
const HOVER_LINEAR_THRUST: f32 = 0.5;

#[derive(Clone, Debug, PartialEq)]
pub struct AltitudeControllerConfig {
    pub gains: Gains,
    /// Maximum absolute value of the integral term.
    /// Prevents integral windup by limiting error accumulation.
    pub max_integral: f32,

    /// Maximum climb rate in meters/second.
    /// This is used to limit the output thrust command.
    pub max_rate: f32,
}

impl Default for AltitudeControllerConfig {
    fn default() -> Self {
        Self {
            gains: Gains {
                p: 1.0,
                i: 0.1,
                d: 0.05,
            },
            max_integral: 100.0, // Arbitrary limit
            max_rate: 2.0,       // 2 m/s for altitude
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
    /// This outer loop maintains the desired altitude by calculating a thrust command (0.0 to 1.0), where:
    /// - 0.0: No thrust (vehicle falls).
    /// - 0.5: Nominal hover thrust (approximately balances gravity).
    /// - 1.0: Maximum thrust (maximum upward acceleration).
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
        const GRAVITY: f32 = 9.81;
        const MAX_ACCEL: f32 = 20.0;

        let r = estimated_attitude.to_rotation_matrix();
        let gains = &self.config.gains;

        let altitude_error = desired_altitude - estimated_altitude;
        let velocity_error = desired_velocity - estimated_velocity;

        // A non-finite error (NaN/∞ altitude or velocity) must never enter the integral, where
        // it would stick (NaN survives `clamp`) and hold thrust at NaN → 0 for good. Neutral
        // for altitude is *hover*, not zero thrust, so return the hover setpoint instead.
        if !(altitude_error.is_finite() && velocity_error.is_finite()) {
            return HOVER_LINEAR_THRUST;
        }

        self.integrated_error += altitude_error * dt;
        self.integrated_error = self.integrated_error.clamp(-100.0, 100.0);

        let proportional_term = gains.p * altitude_error;
        let derivative_term = gains.d * velocity_error;
        let integral_term = gains.i * self.integrated_error;
        let mut acceleration_cmd = proportional_term + derivative_term + integral_term;

        let body_z_alignment = r[(2, 2)].max(0.01);
        acceleration_cmd = (acceleration_cmd - GRAVITY) / body_z_alignment;
        acceleration_cmd = acceleration_cmd.clamp(-MAX_ACCEL, MAX_ACCEL);

        // Map acceleration_cmd such that:
        // - acceleration_cmd = -GRAVITY (free fall) -> linear_thrust = 0
        // - acceleration_cmd = 0 (hover) -> linear_thrust = HOVER_LINEAR_THRUST (0.5)
        // - acceleration_cmd = MAX_ACCEL (max upward) -> linear_thrust = 1
        let linear_thrust = if acceleration_cmd < 0.0 {
            // Map [-GRAVITY, 0] to [0, 0.5]
            0.5 * (acceleration_cmd + GRAVITY) / GRAVITY
        } else {
            // Map [0, MAX_ACCEL] to [0.5, 1]
            0.5 + 0.5 * acceleration_cmd / MAX_ACCEL
        };

        linear_thrust.clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn level() -> UnitQuaternion<f32> {
        UnitQuaternion::identity()
    }

    /// Tilt compensation reads only the body-z/world-z alignment, cos(roll)·cos(pitch) (the
    /// rotation matrix's (2, 2) element), floored at 0.01. Attitudes in a row share it, so
    /// they must command the same thrust; rows differ from each other. The climb request (15 m
    /// below target) keeps level and 40° thrust off the limits; past 90° the floor saturates it.
    #[test]
    fn tilt_compensation_depends_only_on_body_z_alignment() {
        let deg = f32::to_radians;
        let attitude = |roll: f32, pitch: f32, yaw: f32| {
            UnitQuaternion::from_euler_angles(deg(roll), deg(pitch), deg(yaw))
        };
        // Each row: attitudes with equal alignment (yaw-invariant, roll/pitch symmetric).
        let rows = [
            vec![attitude(0.0, 0.0, 0.0), attitude(0.0, 0.0, 120.0)],
            vec![
                attitude(40.0, 0.0, 0.0),
                attitude(0.0, 40.0, 0.0),
                attitude(-40.0, 0.0, 75.0),
            ],
            // Past 90°: alignment ≤ 0, floored to the same 0.01.
            vec![
                attitude(0.0, 89.99, 0.0),
                attitude(100.0, 0.0, 0.0),
                attitude(170.0, 0.0, 30.0),
            ],
        ];
        let thrust =
            |attitude| AltitudeController::new().control(0.004, 25.0, 0.0, 10.0, 0.0, attitude);

        let per_row: Vec<f32> = rows
            .iter()
            .map(|row| {
                let first = thrust(row[0]);
                for &other in &row[1..] {
                    approx::assert_relative_eq!(thrust(other), first, epsilon = 1e-5);
                }
                first
            })
            .collect();
        assert!(
            per_row[0] != per_row[1] && per_row[1] != per_row[2],
            "tilt must change thrust: {per_row:?}"
        );
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
                during == HOVER_LINEAR_THRUST,
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
