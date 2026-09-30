//! Guidance for multirotors such as the QuadCopter X: waypoint navigation and pilot-stick
//! interpretation, feeding [`CopterStabilizer`](crate::stabilizers::copter::CopterStabilizer).
//!
//! | Command | Setpoint |
//! |---|---|
//! | [`Command::Rate`] | [`Setpoint::Rate`], sticks passed through |
//! | [`Command::Attitude`] | [`Setpoint::Attitude`]: roll/pitch sticks passed through; yaw stick slews a heading-hold target |
//! | [`Command::Waypoint`] | [`Setpoint::Attitude`]: lateral position loop → roll/pitch, altitude loop → thrust, heading north |
//!
//! The waypoint loops run at [`CopterGuidanceConfig::navigation_period`] (50 Hz by default)
//! and hold their setpoint in between; pilot commands are interpreted every tick, so stick
//! latency doesn't grow with the navigation period.

use robomath::vec2;

use crate::controllers::Gains;
use crate::controllers::*;
use crate::{
    AttitudeCommand, AttitudeReference, AttitudeSetpoint, Command, Guidance, RateSetpoint,
    Setpoint, VehicleState, WaypointCommand,
};

use crate::math::{saturate_unit, wrap_angle};

#[cfg(test)]
mod tests;

/// Heading-hold target slew rate at full yaw stick, in rad/s (~1.4°/s).
const HEADING_SLEW_RATE: f32 = 0.025;

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
    /// Seconds between runs of the waypoint (altitude and lateral) loops; the setpoint is
    /// held in between. `0.0` runs them every tick. Rounded to the nearest tick: at a 4 ms
    /// loop, 20 ms is every 5th tick. Defaults to [`DEFAULT_NAVIGATION_PERIOD`].
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

    navigation_period: f32,
    /// The waypoint loops' last run; `None` until the first waypoint tick after another mode.
    navigation: Option<Navigation>,
}

/// The waypoint loops' held output.
#[derive(Debug, Clone, Copy)]
struct Navigation {
    /// Seconds since the loops last ran.
    elapsed: f32,
    setpoint: AttitudeSetpoint,
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
                max_rate: vec2(2.0, 2.0), // unused by the lateral controller today (backlog)
            }),

            heading: 0.0,

            navigation_period: config.navigation_period,
            navigation: None,
        }
    }
}

impl Guidance for CopterGuidance {
    fn guide(&mut self, dt: f32, command: &Command, estimated_state: &VehicleState) -> Setpoint {
        assert!(dt > 0.0, "Time step must be positive");

        if !matches!(command, Command::Waypoint(_)) {
            // Re-entering waypoint mode runs the loops at once instead of holding a stale
            // setpoint.
            self.navigation = None;
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

    /// Latches the current heading as the yaw-hold target.
    fn prime(&mut self, state: &VehicleState) {
        self.heading = state.heading;
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
        let target = wrap_angle(self.heading + yaw * HEADING_SLEW_RATE * dt);
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

    /// Runs the lateral position loop (→ roll/pitch) and the altitude loop (→ thrust) toward
    /// the waypoint, holding heading north, once per navigation period; holds the last
    /// setpoint on the ticks in between.
    fn navigate(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let step = match &mut self.navigation {
            Some(navigation) => {
                navigation.elapsed += dt;
                // Half a tick of slack, so float accumulation (5 × 4 ms ≈ 19.999 ms) still
                // runs on the tick nearest the period.
                if navigation.elapsed < self.navigation_period - 0.5 * dt {
                    return navigation.setpoint;
                }
                navigation.elapsed
            }
            None => dt,
        };
        let setpoint = self.run_waypoint_loops(step, waypoint_command, estimated_state);
        self.navigation = Some(Navigation {
            elapsed: 0.0,
            setpoint,
        });
        setpoint
    }

    /// One step of the lateral and altitude loops over `dt` seconds.
    fn run_waypoint_loops(
        &mut self,
        dt: f32,
        waypoint_command: &WaypointCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let desired_position = vec2(waypoint_command.position.x, waypoint_command.position.y);
        let current_position = vec2(
            estimated_state.position.x.value,
            estimated_state.position.y.value,
        );

        let desired_angles = self.lateral_controller.control(
            dt,
            desired_position,
            current_position,
            vec2(
                estimated_state.velocity.x.value,
                estimated_state.velocity.y.value,
            ),
        );

        let linear_thrust = self.altitude_controller.control(
            dt,
            waypoint_command.altitude,
            0.0,
            estimated_state.position.z.value,
            estimated_state.velocity.z.value,
            estimated_state.attitude,
        );

        AttitudeSetpoint {
            roll: desired_angles.x,
            pitch: desired_angles.y,
            heading: 0.0,
            thrust: linear_thrust,
            reference: AttitudeReference::Quaternion,
        }
    }
}
