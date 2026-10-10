//! Estimates whether the vehicle is on the ground or in the air: see [`LandDetector`].

use mavlink::common::MavLandedState;

// Both thresholds are repeated in the `LandDetector` rustdoc and this crate's README; update
// them together.

/// Armed and above this height over home: in the air.
const IN_AIR_HEIGHT_M: f32 = 0.5;
/// Armed and below this height over home: back on the ground.
const LANDED_HEIGHT_M: f32 = 0.2;

/// Estimates the landed state for `EXTENDED_SYS_STATE` from the arm state, whether the flight
/// controller is landing, and the height above home.
///
/// Telemetry only: nothing in the control loop reads it. Disarmed reports on the ground (as
/// PX4's land detector does), so a vehicle disarmed in flight reads landed while it falls.
/// Armed, it reports in the air above 0.5 m and landed again below 0.2 m; between the two the
/// previous state holds, so hovering near one doesn't flicker the state. In the air while
/// landing ([`ControlMode::Land`](stable_flight::ControlMode::Land)) reports landing. Height
/// alone can't tell landing armed above home (a rooftop) from hovering, so a manual landing
/// there reads in the air until disarmed; a commanded land disarms on touchdown.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::LandDetector;
/// use stable_flight_mavlink::mavlink::common::MavLandedState;
///
/// let mut detector = LandDetector::default();
/// assert_eq!(detector.update(true, false, 2.0), MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// assert_eq!(detector.update(true, true, 0.3), MavLandedState::MAV_LANDED_STATE_LANDING);
/// assert_eq!(detector.update(false, true, 0.3), MavLandedState::MAV_LANDED_STATE_ON_GROUND);
/// ```
#[derive(Debug, Default)]
pub struct LandDetector {
    in_air: bool,
}

impl LandDetector {
    /// Advances by one tick. `landing` is whether the flight controller flies a land.
    /// `height_m` is the height above home; NaN (no home, or no finite altitude) reports
    /// undefined while armed and keeps the previous estimate.
    pub fn update(&mut self, armed: bool, landing: bool, height_m: f32) -> MavLandedState {
        if !armed {
            self.in_air = false;
            return MavLandedState::MAV_LANDED_STATE_ON_GROUND;
        }
        if height_m.is_nan() {
            return MavLandedState::MAV_LANDED_STATE_UNDEFINED;
        }
        if height_m > IN_AIR_HEIGHT_M {
            self.in_air = true;
        } else if height_m < LANDED_HEIGHT_M {
            self.in_air = false;
        }
        if self.in_air && landing {
            MavLandedState::MAV_LANDED_STATE_LANDING
        } else if self.in_air {
            MavLandedState::MAV_LANDED_STATE_IN_AIR
        } else {
            MavLandedState::MAV_LANDED_STATE_ON_GROUND
        }
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    #[test]
    fn reports_in_air_above_the_takeoff_height_with_hysteresis_and_landing_in_a_land() {
        use MavLandedState::{
            MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_LANDING as LANDING,
            MAV_LANDED_STATE_ON_GROUND as GROUND, MAV_LANDED_STATE_UNDEFINED as UNDEFINED,
        };
        // (armed, landing, height_m, expected), applied in sequence to one detector.
        let ticks = [
            (false, false, 0.0, GROUND),
            (true, false, 0.0, GROUND),         // armed on the ground
            (true, false, 0.4, GROUND),         // between thresholds: stays landed
            (true, false, 0.6, AIR),            // took off
            (true, false, 0.3, AIR),            // between thresholds: stays in the air
            (true, false, f32::NAN, UNDEFINED), // no height
            (true, false, 0.3, AIR),            // NaN kept the previous estimate
            (true, false, 0.1, GROUND),         // landed
            (true, false, 1.0, AIR),
            (false, false, 1.0, GROUND), // disarmed reports landed at any height
            (true, false, 0.3, GROUND),  // and resets the estimate
            (true, false, f32::INFINITY, AIR),
            (true, false, f32::NEG_INFINITY, GROUND),
            (true, true, 0.1, GROUND), // a land on the ground: still on the ground
            (true, false, 2.0, AIR),
            (true, true, 2.0, LANDING), // a land in the air
            (true, true, 0.3, LANDING), // between thresholds
            (true, true, f32::NAN, UNDEFINED),
            (true, true, 0.1, GROUND), // down
            (true, true, 1.0, LANDING),
            (false, true, 1.0, GROUND), // disarmed (touchdown above home)
        ];

        let mut detector = LandDetector::default();
        for (i, (armed, landing, height_m, expected)) in ticks.into_iter().enumerate() {
            check!(
                detector.update(armed, landing, height_m) == expected,
                "tick {i}"
            );
        }
    }
}
