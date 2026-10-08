//! Frame-conformance kit for state estimators and simulator backends.
//!
//! Every simulator or sensor stack reports state in its own frames and units; a
//! [`StateEstimator`](crate::StateEstimator) converts to the
//! [`VehicleState` frame contract](crate::VehicleState#frame-contract) at its boundary. This
//! module is a table of physical [`Scenario`]s with the state each must produce. A backend's
//! tests build the simulator's *native* report for each scenario (a mock-client fixture), run
//! it through the backend, and [`Scenario::check`] the result. That catches a swapped axis, a
//! flipped sign, a wrong rotation order, or a unit slip before the backend flies.
//!
//! [`conformance_tests!`](crate::conformance_tests) generates one test per scenario and a
//! test that fails when a scenario has none, so new scenarios reach every backend.
//!
//! # Examples
//!
//! ```
//! use stable_flight::testing::conformance::{LEVEL, ROLL_RIGHT_30, SCENARIOS};
//!
//! // A backend that got the frames right produces the scenario's expected state.
//! for scenario in SCENARIOS {
//!     assert!(scenario.check(&scenario.expected_state()).is_ok());
//! }
//!
//! // Level flight is not a 30° right bank.
//! let error = ROLL_RIGHT_30.check(&LEVEL.expected_state()).unwrap_err();
//! assert_eq!(error.mismatches()[0].field, "roll");
//! ```

use core::fmt;

use crate::Vec3;
use nalgebra::UnitQuaternion;

use crate::VehicleState;
use crate::math::wrap_angle;

#[cfg(test)]
mod tests;

/// Largest difference accepted for an angle (rad) or body rate (rad/s).
pub const ANGLE_TOLERANCE: f32 = 1e-3;

/// Largest difference accepted for a position (m), velocity (m/s), or acceleration (m/s²).
pub const LINEAR_TOLERANCE: f32 = 1e-3;

/// Altitude every scenario flies at, in meters. Nonzero so a dropped or negated altitude
/// shows up.
const SCENARIO_ALTITUDE: f32 = 100.0;

/// One physical situation and the [`VehicleState`] it must map to.
///
/// Fields give the physical truth in the
/// [frame contract's](crate::VehicleState#frame-contract) axes and signs, with angles in
/// degrees (as simulators usually report them), so a backend can build its native fixture
/// from them.
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
    /// World position (north, east, up) in meters.
    pub position_neu: [f32; 3],
    /// World velocity (north, east, up) in m/s.
    pub velocity_neu: [f32; 3],
    /// World acceleration (north, east, up) in m/s².
    pub acceleration_neu: [f32; 3],
}

/// At rest, wings level, nose north.
pub const LEVEL: Scenario = Scenario {
    name: "level",
    roll_deg: 0.0,
    pitch_deg: 0.0,
    heading_deg: 0.0,
    body_rates_deg_s: [0.0; 3],
    position_neu: [0.0, 0.0, SCENARIO_ALTITUDE],
    velocity_neu: [0.0; 3],
    acceleration_neu: [0.0; 3],
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

/// Nose east, 10° up, banked 30° right. Distinct angles on every axis catch a wrong rotation
/// order or swapped Euler angles.
pub const BANKED_CLIMB_EAST: Scenario = Scenario {
    name: "banked climb east",
    roll_deg: 30.0,
    pitch_deg: 10.0,
    heading_deg: 90.0,
    ..LEVEL
};

/// Level, climbing at 2 m/s and accelerating upward at 1 m/s².
pub const CLIMBING: Scenario = Scenario {
    name: "climbing",
    velocity_neu: [0.0, 0.0, 2.0],
    acceleration_neu: [0.0, 0.0, 1.0],
    ..LEVEL
};

/// Northeast of the origin, flying northeast and speeding up. Distinct north and east values
/// catch a swapped horizontal axis (e.g. ENU vs NED).
pub const MOVING_NORTHEAST: Scenario = Scenario {
    name: "moving northeast",
    position_neu: [30.0, 40.0, SCENARIO_ALTITUDE],
    velocity_neu: [3.0, 4.0, 0.0],
    acceleration_neu: [0.5, 1.5, 0.0],
    ..LEVEL
};

/// Level, yawing right (clockwise from above) at 30°/s.
pub const YAW_RIGHT_RATE: Scenario = Scenario {
    name: "yaw right rate",
    body_rates_deg_s: [0.0, 0.0, 30.0],
    ..LEVEL
};

/// Rolling right, pitching up, and yawing right at distinct rates, which catches swapped
/// rate axes.
pub const COMBINED_RATES: Scenario = Scenario {
    name: "combined rates",
    body_rates_deg_s: [10.0, 20.0, 30.0],
    ..LEVEL
};

/// Every scenario; a backend covers each one (see [`conformance_tests!`](crate::conformance_tests)).
pub const SCENARIOS: &[Scenario] = &[
    LEVEL,
    ROLL_RIGHT_30,
    PITCH_UP_15,
    HEADING_EAST,
    BANKED_CLIMB_EAST,
    CLIMBING,
    MOVING_NORTHEAST,
    YAW_RIGHT_RATE,
    COMBINED_RATES,
];

impl Scenario {
    /// The [`VehicleState`] a conforming backend reports for this scenario.
    pub fn expected_state(&self) -> VehicleState {
        let [roll, pitch, heading] =
            [self.roll_deg, self.pitch_deg, self.heading_deg].map(f32::to_radians);
        let [p, q, r] = self.body_rates_deg_s.map(f32::to_radians);
        let [x, y, z] = self.position_neu;
        let [vx, vy, vz] = self.velocity_neu;
        let [ax, ay, az] = self.acceleration_neu;

        VehicleState {
            position_m: Vec3::new(x, y, z),
            velocity_mps: Vec3::new(vx, vy, vz),
            acceleration_mps2: Vec3::new(ax, ay, az),
            attitude: UnitQuaternion::from_euler_angles(roll, pitch, heading),
            angular_velocity: Vec3::new(p, q, r),
            roll,
            pitch,
            heading,
        }
    }

    /// Checks `state` against this scenario: position, velocity, acceleration, the Euler
    /// angles, the attitude quaternion's Euler angles, and the body rates.
    ///
    /// Angles are compared modulo a full turn, so a heading in [0, 2π) or [-π, π) both pass.
    /// A NaN field always fails.
    ///
    /// # Errors
    ///
    /// [`ConformanceError`] listing every field outside [`ANGLE_TOLERANCE`] or
    /// [`LINEAR_TOLERANCE`].
    #[allow(
        clippy::result_large_err,
        reason = "mismatches are stored inline so the kit needs no allocator; test-only"
    )]
    pub fn check(&self, state: &VehicleState) -> Result<(), ConformanceError> {
        let e = self.expected_state();
        let mut error = ConformanceError::new(self.name);

        let linear = [
            ("position_m.x", e.position_m.x, state.position_m.x),
            ("position_m.y", e.position_m.y, state.position_m.y),
            ("position_m.z", e.position_m.z, state.position_m.z),
            ("velocity_mps.x", e.velocity_mps.x, state.velocity_mps.x),
            ("velocity_mps.y", e.velocity_mps.y, state.velocity_mps.y),
            ("velocity_mps.z", e.velocity_mps.z, state.velocity_mps.z),
            (
                "acceleration_mps2.x",
                e.acceleration_mps2.x,
                state.acceleration_mps2.x,
            ),
            (
                "acceleration_mps2.y",
                e.acceleration_mps2.y,
                state.acceleration_mps2.y,
            ),
            (
                "acceleration_mps2.z",
                e.acceleration_mps2.z,
                state.acceleration_mps2.z,
            ),
        ];
        let rates = [
            (
                "angular_velocity.x",
                e.angular_velocity.x,
                state.angular_velocity.x,
            ),
            (
                "angular_velocity.y",
                e.angular_velocity.y,
                state.angular_velocity.y,
            ),
            (
                "angular_velocity.z",
                e.angular_velocity.z,
                state.angular_velocity.z,
            ),
        ];
        let (q_roll, q_pitch, q_yaw) = crate::euler_angles(&state.attitude);
        let angles = [
            ("roll", e.roll, state.roll),
            ("pitch", e.pitch, state.pitch),
            ("heading", e.heading, state.heading),
            ("attitude roll", e.roll, q_roll),
            ("attitude pitch", e.pitch, q_pitch),
            ("attitude yaw", e.heading, q_yaw),
        ];

        for (field, expected, actual) in linear {
            error.check(field, expected, actual, expected - actual, LINEAR_TOLERANCE);
        }
        for (field, expected, actual) in rates {
            error.check(field, expected, actual, expected - actual, ANGLE_TOLERANCE);
        }
        for (field, expected, actual) in angles {
            let difference = wrap_angle(expected - actual);
            error.check(field, expected, actual, difference, ANGLE_TOLERANCE);
        }

        if error.mismatches().is_empty() {
            Ok(())
        } else {
            Err(error)
        }
    }
}

/// Number of fields [`Scenario::check`] compares.
const CHECKED_FIELDS: usize = 18;

/// A backend's state for a [`Scenario`] broke the frame contract.
///
/// Holds its mismatches inline (no allocation), so it works in `no_std` tests.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConformanceError {
    /// The scenario's name.
    pub scenario: &'static str,
    mismatches: [Mismatch; CHECKED_FIELDS],
    len: usize,
}

impl ConformanceError {
    fn new(scenario: &'static str) -> Self {
        Self {
            scenario,
            mismatches: [Mismatch::EMPTY; CHECKED_FIELDS],
            len: 0,
        }
    }

    /// Every field that was off, in check order.
    pub fn mismatches(&self) -> &[Mismatch] {
        &self.mismatches[..self.len]
    }

    /// Records `field` unless `|difference| <= tolerance` (false for NaN, so NaN fails).
    fn check(
        &mut self,
        field: &'static str,
        expected: f32,
        actual: f32,
        difference: f32,
        tolerance: f32,
    ) {
        if difference.abs() <= tolerance || self.len == CHECKED_FIELDS {
            return;
        }
        self.mismatches[self.len] = Mismatch {
            field,
            expected,
            actual,
        };
        self.len += 1;
    }
}

impl fmt::Display for ConformanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "scenario `{}` off contract", self.scenario)?;
        for (i, mismatch) in self.mismatches().iter().enumerate() {
            let separator = if i == 0 { ": " } else { "; " };
            write!(f, "{separator}{mismatch}")?;
        }
        Ok(())
    }
}

impl core::error::Error for ConformanceError {}

/// One field of a [`ConformanceError`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mismatch {
    /// The `VehicleState` field, e.g. `velocity.z`.
    pub field: &'static str,
    /// The contract's value (SI: m, m/s, m/s², rad, rad/s).
    pub expected: f32,
    /// The backend's value.
    pub actual: f32,
}

impl Mismatch {
    const EMPTY: Mismatch = Mismatch {
        field: "",
        expected: 0.0,
        actual: 0.0,
    };
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

/// Generates a backend's conformance tests: one `#[test]` per listed scenario, plus
/// `conformance_covers_every_scenario`, which fails if any scenario in
/// [`SCENARIOS`](crate::testing::conformance::SCENARIOS) has no test.
///
/// `$state_for` is a `fn(&Scenario) -> VehicleState` that builds the simulator's native report
/// for the scenario and runs it through the backend. Each row is `test_name: SCENARIO`,
/// optionally followed by `, ignore = "reason"` for a known failure.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight::testing::conformance::Scenario;
///
/// // A real backend builds its simulator's native report here.
/// fn state_for(scenario: &Scenario) -> VehicleState {
///     scenario.expected_state()
/// }
///
/// stable_flight::conformance_tests! {
///     state_for;
///     conforms_level: LEVEL;
///     conforms_roll_right_30: ROLL_RIGHT_30;
///     conforms_pitch_up_15: PITCH_UP_15, ignore = "pitch sign not converted yet";
/// }
/// ```
#[macro_export]
macro_rules! conformance_tests {
    ($state_for:path; $($test:ident: $scenario:ident $(, ignore = $reason:literal)?;)+) => {
        $(
            #[test]
            $(#[ignore = $reason])?
            fn $test() {
                let scenario = $crate::testing::conformance::$scenario;
                if let Err(error) = scenario.check(&$state_for(&scenario)) {
                    panic!("{error}");
                }
            }
        )+

        #[test]
        fn conformance_covers_every_scenario() {
            let covered = [$($crate::testing::conformance::$scenario),+];
            for scenario in $crate::testing::conformance::SCENARIOS {
                assert!(
                    covered.contains(scenario),
                    "no conformance test for scenario `{}`",
                    scenario.name
                );
            }
        }
    };
}
