//! Frame-conformance kit for simulator backends (`conformance` feature).
//!
//! Every simulator reports state in its own frames and units; a backend converts to the
//! `VehicleState` contract (see its docs in `stable-flight`) at its boundary. This module
//! is a table of physical [`Scenario`]s with the state each must produce. A backend's tests
//! build the simulator's *native* report for each scenario (a mock-bridge fixture), run it
//! through the backend, and [`Scenario::check`] the result. That catches a swapped axis, a
//! flipped sign, or a unit slip before the backend flies.
//!
//! Enable it from a backend's `[dev-dependencies]`:
//!
//! ```toml
//! stable-flight-sitl = { version = "0.1.0", features = ["conformance"] }
//! ```
//!
//! # Examples
//!
//! ```
//! use stable_flight_sitl::conformance::{ROLL_RIGHT_30, SCENARIOS};
//!
//! // A backend that got the frames right produces the scenario's expected state.
//! for scenario in SCENARIOS {
//!     assert!(scenario.check(&scenario.expected_state()).is_ok());
//! }
//!
//! // Level flight is not a 30° right bank.
//! let level = stable_flight::VehicleState::default();
//! assert!(ROLL_RIGHT_30.check(&level).is_err());
//! ```

use std::f32::consts::{PI, TAU};
use std::fmt;

use robomath::{Quaternion, vec3};
use stable_flight::VehicleState;
use thiserror::Error;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::f32::{Acceleration, Length, Velocity};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

#[cfg(test)]
mod tests;

/// Largest difference accepted for an angle (rad) or body rate (rad/s).
pub const ANGLE_TOLERANCE: f32 = 1e-3;

/// Largest difference accepted for a position (m) or velocity (m/s).
pub const LINEAR_TOLERANCE: f32 = 1e-3;

/// Altitude every scenario flies at, in meters. Nonzero so a dropped or negated altitude
/// shows up.
pub const ALTITUDE: f32 = 100.0;

/// One physical situation and the `VehicleState` it must map to.
///
/// Fields describe the physical truth in the contract's conventions, in the units simulators
/// usually report (degrees), so a backend can build its native fixture from them:
///
/// - world frame: `x` north, `y` east, `z` up (altitude);
/// - roll: right wing down is positive; pitch: nose up is positive;
/// - heading: 0 is north, positive turns clockwise (east is 90°);
/// - body rates: `x` roll right, `y` pitch up, `z` yaw right (nose toward east from north).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scenario {
    /// Short name used in failure messages.
    pub name: &'static str,
    /// Roll angle in degrees.
    pub roll_deg: f32,
    /// Pitch angle in degrees.
    pub pitch_deg: f32,
    /// Heading in degrees.
    pub heading_deg: f32,
    /// Body rates (roll, pitch, yaw) in degrees per second.
    pub body_rates_deg_s: [f32; 3],
    /// World velocity (north, east, up) in m/s.
    pub velocity_neu: [f32; 3],
    /// World position (north, east, up) in meters.
    pub position_neu: [f32; 3],
}

const LEVEL_POSITION: [f32; 3] = [0.0, 0.0, ALTITUDE];

/// At rest, wings level, nose north.
pub const LEVEL: Scenario = Scenario {
    name: "level",
    roll_deg: 0.0,
    pitch_deg: 0.0,
    heading_deg: 0.0,
    body_rates_deg_s: [0.0; 3],
    velocity_neu: [0.0; 3],
    position_neu: LEVEL_POSITION,
};

/// Banked 30° right (right wing down).
pub const ROLL_RIGHT_30: Scenario = Scenario {
    name: "roll right 30°",
    roll_deg: 30.0,
    ..LEVEL
};

/// Nose 15° up.
pub const PITCH_UP_15: Scenario = Scenario {
    name: "pitch up 15°",
    pitch_deg: 15.0,
    ..LEVEL
};

/// Level, nose pointing east.
pub const HEADING_EAST: Scenario = Scenario {
    name: "heading east",
    heading_deg: 90.0,
    ..LEVEL
};

/// Level, climbing at 2 m/s.
pub const CLIMBING: Scenario = Scenario {
    name: "climbing",
    velocity_neu: [0.0, 0.0, 2.0],
    ..LEVEL
};

/// Level, yawing right (clockwise from above) at 30°/s.
pub const YAW_RIGHT_RATE: Scenario = Scenario {
    name: "yaw right rate",
    body_rates_deg_s: [0.0, 0.0, 30.0],
    ..LEVEL
};

/// Every scenario; a backend should cover each one.
pub const SCENARIOS: [Scenario; 6] = [
    LEVEL,
    ROLL_RIGHT_30,
    PITCH_UP_15,
    HEADING_EAST,
    CLIMBING,
    YAW_RIGHT_RATE,
];

impl Scenario {
    /// The `VehicleState` a conforming backend reports for this scenario.
    ///
    /// Acceleration is zero; the scenarios don't pin it.
    pub fn expected_state(&self) -> VehicleState {
        let (roll, pitch, heading) = self.angles();
        let [x, y, z] = self.position_neu;
        let [vx, vy, vz] = self.velocity_neu;
        let [p, q, r] = self.body_rates_deg_s;

        VehicleState {
            position: vec3(
                Length::new::<meter>(x),
                Length::new::<meter>(y),
                Length::new::<meter>(z),
            ),
            velocity: vec3(
                Velocity::new::<meter_per_second>(vx),
                Velocity::new::<meter_per_second>(vy),
                Velocity::new::<meter_per_second>(vz),
            ),
            acceleration: vec3(
                Acceleration::new::<meter_per_second_squared>(0.0),
                Acceleration::new::<meter_per_second_squared>(0.0),
                Acceleration::new::<meter_per_second_squared>(0.0),
            ),
            attitude: Quaternion::from_euler(heading, pitch, roll),
            angular_velocity: vec3(p.to_radians(), q.to_radians(), r.to_radians()),
            roll,
            pitch,
            heading,
        }
    }

    /// Checks `state` against this scenario: position, velocity, the Euler angles, the
    /// attitude quaternion's Euler angles, and the body rates.
    ///
    /// Angles are compared modulo a full turn, so a heading in [0, 2π) or (-π, π] both pass.
    ///
    /// # Errors
    ///
    /// [`ConformanceError`] listing every field outside [`ANGLE_TOLERANCE`] or
    /// [`LINEAR_TOLERANCE`].
    pub fn check(&self, state: &VehicleState) -> Result<(), ConformanceError> {
        let expected = self.expected_state();
        let mut mismatches = Vec::new();


        let mut linear = |field, expected: f32, actual: f32| {
            if !within(expected - actual, LINEAR_TOLERANCE) {
                mismatches.push(Mismatch {
                    field,
                    expected,
                    actual,
                });
            }
        };
        linear(
            "position.x",
            expected.position.x.value,
            state.position.x.value,
        );
        linear(
            "position.y",
            expected.position.y.value,
            state.position.y.value,
        );
        linear(
            "position.z",
            expected.position.z.value,
            state.position.z.value,
        );
        linear(
            "velocity.x",
            expected.velocity.x.value,
            state.velocity.x.value,
        );
        linear(
            "velocity.y",
            expected.velocity.y.value,
            state.velocity.y.value,
        );
        linear(
            "velocity.z",
            expected.velocity.z.value,
            state.velocity.z.value,
        );

        let mut angular = |field, expected: f32, actual: f32, wrap: bool| {
            let error = if wrap {
                angle_difference(expected, actual)
            } else {
                expected - actual
            };
            if !within(error, ANGLE_TOLERANCE) {
                mismatches.push(Mismatch {
                    field,
                    expected,
                    actual,
                });
            }
        };
        angular("roll", expected.roll, state.roll, true);
        angular("pitch", expected.pitch, state.pitch, true);
        angular("heading", expected.heading, state.heading, true);
        angular(
            "attitude.roll()",
            expected.roll,
            state.attitude.roll(),
            true,
        );
        angular(
            "attitude.pitch()",
            expected.pitch,
            state.attitude.pitch(),
            true,
        );
        angular(
            "attitude.yaw()",
            expected.heading,
            state.attitude.yaw(),
            true,
        );
        let (rates, actual) = (expected.angular_velocity, state.angular_velocity);
        angular("angular_velocity.x", rates.x, actual.x, false);
        angular("angular_velocity.y", rates.y, actual.y, false);
        angular("angular_velocity.z", rates.z, actual.z, false);

        if mismatches.is_empty() {
            Ok(())
        } else {
            Err(ConformanceError {
                scenario: self.name,
                mismatches,
            })
        }
    }

    /// (roll, pitch, heading) in radians.
    fn angles(&self) -> (f32, f32, f32) {
        (
            self.roll_deg.to_radians(),
            self.pitch_deg.to_radians(),
            self.heading_deg.to_radians(),
        )
    }
}

/// `|error| <= tolerance`; false for NaN, so a NaN field fails.
fn within(error: f32, tolerance: f32) -> bool {
    error.abs() <= tolerance
}

/// `expected - actual`, wrapped to [-π, π).
fn angle_difference(expected: f32, actual: f32) -> f32 {
    (expected - actual + PI).rem_euclid(TAU) - PI
}

/// A backend's state for a [`Scenario`] broke the frame contract.
#[derive(Debug, Clone, PartialEq, Error)]
#[error("scenario `{scenario}` off contract: {}", list(.mismatches))]
pub struct ConformanceError {
    /// The scenario's name.
    pub scenario: &'static str,
    /// Every field that was off, in check order.
    pub mismatches: Vec<Mismatch>,
}

/// One field of a [`ConformanceError`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mismatch {
    /// The `VehicleState` field, e.g. `velocity.z`.
    pub field: &'static str,
    /// The contract's value (SI: m, m/s, rad, rad/s).
    pub expected: f32,
    /// The backend's value.
    pub actual: f32,
}

impl fmt::Display for Mismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} expected {} got {}",
            self.field, self.expected, self.actual
        )
    }
}

fn list(mismatches: &[Mismatch]) -> String {
    mismatches
        .iter()
        .map(Mismatch::to_string)
        .collect::<Vec<_>>()
        .join("; ")
}
