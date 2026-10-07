//! Guidance for multirotors such as the QuadCopter X: waypoint navigation and pilot-stick
//! interpretation, feeding [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer).
//!
//! | Command | Setpoint |
//! |---|---|
//! | [`Command::Rate`] | [`Setpoint::Rate`], sticks passed through |
//! | [`Command::Attitude`] | [`Setpoint::Attitude`]: roll/pitch sticks passed through; yaw stick slews a heading-hold target |
//! | [`Command::Waypoint`] | [`Setpoint::Attitude`]: lateral position loop → roll/pitch (every [`navigation_period`](CopterGuidanceConfig::navigation_period)), altitude loop → thrust (every tick), heading north |

use crate::Vec2;

use crate::controllers::Gains;
use crate::controllers::*;
use crate::{
    AttitudeCommand, AttitudeReference, AttitudeSetpoint, Command, Guidance, RateSetpoint,
    Setpoint, VehicleState, WaypointCommand,
};

use super::rate_divider::RateDivider;
use crate::math::{saturate_unit, wrap_angle};

#[cfg(test)]
mod tests;

/// Heading-hold target slew rate at full yaw stick, in rad/s (~57°/s).
const HEADING_SLEW_RATE: f32 = 1.0;

/// Default [`CopterGuidanceConfig::navigation_period`]: 50 Hz, a fifth of the usual 250 Hz
/// inner loop.
pub const DEFAULT_NAVIGATION_PERIOD: f32 = 0.02;

/// Gains for the [CopterGuidance] position loops.
///
/// # Examples
///
/// ```
/// use stable_flight::controllers::Gains;
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
///
/// let guidance = CopterGuidance::new(CopterGuidanceConfig {
///     altitude_gains: Gains { p: 2.0, i: 0.2, d: 0.1 },
///     ..Default::default()
/// });
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct CopterGuidanceConfig {
    /// Altitude loop: altitude error (m) → linear thrust.
    pub altitude_gains: Gains,
    /// Lateral position loop: horizontal position error (m) → roll/pitch.
    pub lateral_gains: Gains,
    /// Seconds between runs of the lateral position loop; its roll/pitch output is held in
    /// between. `0.0` runs it every tick. Rounded to the nearest tick (at a 4 ms loop, 20 ms
    /// is every 5th tick); each run steps over the actual elapsed time. The altitude loop runs
    /// every tick, since its tilt compensation needs the current attitude. Defaults to
    /// [`DEFAULT_NAVIGATION_PERIOD`].
    pub navigation_period: f32,
}

impl Default for CopterGuidanceConfig {
    fn default() -> Self {
        CopterGuidanceConfig {
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
            navigation_period: DEFAULT_NAVIGATION_PERIOD,
        }
    }
}

/// Copter guidance: altitude and lateral position loops for waypoints, and heading hold for
/// piloted attitude flight.
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight::{AttitudeCommand, Command, Guidance, Setpoint, VehicleState};
///
/// let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());
/// let state = VehicleState { heading: 1.0, ..Default::default() };
/// guidance.prime(&state);
///
/// let command = Command::Attitude(AttitudeCommand { throttle: 0.5, ..Default::default() });
/// let Setpoint::Attitude(setpoint) = guidance.guide(0.004, &command, &state) else {
///     unreachable!("attitude commands yield attitude setpoints");
/// };
/// assert!((setpoint.heading - 1.0).abs() < 1e-6); // holds the primed heading
/// assert_eq!(setpoint.thrust, 0.5);
/// ```
pub struct CopterGuidance {
    pub(crate) altitude_controller: AltitudeController,
    pub(crate) lateral_controller: LateralController,

    /// Heading-hold target for attitude commands.
    heading: f32,

    /// Paces the lateral loop.
    lateral_rate: RateDivider,
    /// The lateral loop's last roll/pitch, held between its runs.
    tilt: Vec2<f32>,
}

impl CopterGuidance {
    /// Creates a new [CopterGuidance] with the specified configuration.
    ///
    /// # Panics
    ///
    /// If `config.navigation_period` is negative or not finite.
    pub fn new(config: CopterGuidanceConfig) -> Self {
        assert!(
            config.navigation_period >= 0.0 && config.navigation_period.is_finite(),
            "navigation_period must be non-negative and finite, got {}",
            config.navigation_period
        );
        CopterGuidance {
            altitude_controller: AltitudeController::with_config(&AltitudeControllerConfig {
                gains: config.altitude_gains,
                ..Default::default()
            }),

            lateral_controller: LateralController::with_config(&LateralControllerConfig {
                gains: config.lateral_gains,
                max_integral: 100.0,
                max_rate: Vec2::new(2.0, 2.0), // unused by the lateral controller today (#71)
            }),

            heading: 0.0,

            lateral_rate: RateDivider::new(config.navigation_period),
            tilt: Vec2::zeros(),
        }
    }
}

impl Guidance for CopterGuidance {
    fn guide(&mut self, dt: f32, command: &Command, estimated_state: &VehicleState) -> Setpoint {
        assert!(dt > 0.0, "Time step must be positive");

        if !matches!(command, Command::Waypoint(_)) {
            // Re-entering waypoint mode runs the lateral loop at once instead of holding a
            // stale tilt.
            self.lateral_rate.reset();
        }

        match command {
            Command::Attitude(attitude_command) => {
                Setpoint::Attitude(self.attitude_setpoint(dt, attitude_command, estimated_state))
            }
            Command::Rate(rate_command) => Setpoint::Rate(RateSetpoint {
                roll_rate: rate_command.roll_rate,
                pitch_rate: rate_command.pitch_rate,
                yaw_rate: rate_command.yaw_rate,
                thrust: rate_command.throttle,
            }),
            Command::Waypoint(waypoint_command) => {
                Setpoint::Attitude(self.navigate(dt, waypoint_command, estimated_state))
            }
        }
    }

    /// Latches the current heading as the yaw-hold target, and drops any held waypoint tilt.
    fn prime(&mut self, state: &VehicleState) {
        self.heading = state.heading;
        self.lateral_rate.reset();
    }
}

impl CopterGuidance {
    /// Passes roll, pitch, and throttle through and slews the heading-hold target with the
    /// yaw stick.
    fn attitude_setpoint(
        &mut self,
        dt: f32,
        attitude_command: &AttitudeCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        // Commands from input devices can be out of range or NaN; saturate rather than panic
        // (roll, pitch, and throttle saturate in the stabilizer; see `crate::math`).
        let yaw = saturate_unit(attitude_command.yaw);

        // Wrapped so sustained yaw input can't grow the target without bound; the yaw
        // controller works on the wrapped error either way.
        // Centred stick adds nothing, even for a pathological `dt` (0 × ∞ would be NaN).
        let slew = if yaw == 0.0 {
            0.0
        } else {
            yaw * HEADING_SLEW_RATE * dt
        };
        let target = wrap_angle(self.heading + slew);
        if target.is_finite() {
            self.heading = target;
        } else if estimated_state.heading.is_finite() {
            // A NaN target (e.g. primed from a NaN estimate) would otherwise disable heading
            // hold for good; re-latch from the current estimate once it's valid.
            self.heading = wrap_angle(estimated_state.heading);
        }

        AttitudeSetpoint {
            roll: attitude_command.roll,
            pitch: attitude_command.pitch,
            heading: self.heading,
            thrust: attitude_command.throttle,
            reference: AttitudeReference::Euler,
        }
    }

    /// Flies toward the waypoint, holding heading north: the lateral position loop gives
    /// roll/pitch (once per navigation period, held in between) and the altitude loop gives
    /// thrust (every tick).
    fn navigate(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let tilt = self.tilt_toward(dt, waypoint_command, estimated_state);

        let linear_thrust = self.altitude_controller.control(
            dt,
            waypoint_command.altitude,
            0.0,
            estimated_state.position.z.value,
            estimated_state.velocity.z.value,
            estimated_state.attitude,
        );

        AttitudeSetpoint {
            roll: tilt.x,
            pitch: tilt.y,
            heading: 0.0,
            thrust: linear_thrust,
            reference: AttitudeReference::Quaternion,
        }
    }

    /// Normalized roll/pitch toward the waypoint from the lateral loop, run when
    /// [`lateral_rate`](Self::lateral_rate) is due and held otherwise.
    fn tilt_toward(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> Vec2<f32> {
        let Some(step) = self.lateral_rate.due(dt) else {
            return self.tilt;
        };

        let desired_position = Vec2::new(waypoint_command.position.x, waypoint_command.position.y);
        let current_position = Vec2::new(
            estimated_state.position.x.value,
            estimated_state.position.y.value,
        );
        let velocity = Vec2::new(
            estimated_state.velocity.x.value,
            estimated_state.velocity.y.value,
        );

        // A non-finite target or estimate gets level for this tick only (input policy). The
        // loop doesn't run, so the neutral output isn't held for a period and the next tick
        // retries over the combined time.
        let error = desired_position - current_position;
        if ![error.x, error.y, velocity.x, velocity.y]
            .iter()
            .all(|v| v.is_finite())
        {
            return Vec2::zeros();
        }

        self.tilt =
            self.lateral_controller
                .control(step, desired_position, current_position, velocity);
        self.lateral_rate.ran();
        self.tilt
    }
}
