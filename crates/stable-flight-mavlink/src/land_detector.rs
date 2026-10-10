//! Estimates whether the vehicle is on the ground or in the air: see [`LandDetector`].

use mavlink::common::MavLandedState;
use stable_flight::ControlMode;

// Both thresholds are repeated in the `LandDetector` rustdoc and this crate's README; update
// them together.

/// Armed and above this height over home: in the air.
const IN_AIR_HEIGHT_M: f32 = 0.5;
/// Armed and below this height over home: back on the ground.
const LANDED_HEIGHT_M: f32 = 0.2;

/// Estimates the landed state for `EXTENDED_SYS_STATE` from the flight controller's arm state
/// and mode, and the height above home.
///
/// Telemetry only: nothing in the control loop reads it. Disarmed reports on the ground (as
/// PX4's land detector does), so a vehicle disarmed in flight reads landed while it falls.
/// Armed, it reports in the air above 0.5 m and landed again below 0.2 m; between the two the
/// previous state holds, so hovering near one doesn't flicker the state. In the air in
/// [`ControlMode::Land`] reports landing. Height
/// alone can't tell landing armed above home (a rooftop) from hovering, so a manual landing
/// there reads in the air until disarmed; a commanded land disarms on touchdown.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::LandDetector;
/// use stable_flight::ControlMode::{Attitude, Land};
/// use stable_flight_mavlink::mavlink::common::MavLandedState;
///
/// let mut detector = LandDetector::default();
/// assert_eq!(detector.update(true, Attitude, 2.0), MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// assert_eq!(detector.update(true, Land, 0.3), MavLandedState::MAV_LANDED_STATE_LANDING);
/// assert_eq!(detector.update(false, Land, 0.3), MavLandedState::MAV_LANDED_STATE_ON_GROUND);
/// ```
#[derive(Debug, Default)]
pub struct LandDetector {
    in_air: bool,
}

impl LandDetector {
    /// Advances by one tick with the flight controller's arm state and `mode`. `height_m` is the height above home; NaN (no home, or no finite altitude) reports
    /// undefined while armed and keeps the previous estimate.
    pub fn update(&mut self, armed: bool, mode: ControlMode, height_m: f32) -> MavLandedState {
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
        if self.in_air && mode == ControlMode::Land {
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
        use ControlMode::{Attitude as FLY, Land as LAND};
        use MavLandedState::{
            MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_LANDING as LANDING,
            MAV_LANDED_STATE_ON_GROUND as GROUND, MAV_LANDED_STATE_UNDEFINED as UNDEFINED,
        };
        // (armed, mode, height_m, expected), applied in sequence to one detector.
        let ticks = [
            (false, FLY, 0.0, GROUND),
            (true, FLY, 0.0, GROUND),         // armed on the ground
            (true, FLY, 0.4, GROUND),         // between thresholds: stays landed
            (true, FLY, 0.6, AIR),            // took off
            (true, FLY, 0.3, AIR),            // between thresholds: stays in the air
            (true, FLY, f32::NAN, UNDEFINED), // no height
            (true, FLY, 0.3, AIR),            // NaN kept the previous estimate
            (true, FLY, 0.1, GROUND),         // landed
            (true, FLY, 1.0, AIR),
            (false, FLY, 1.0, GROUND), // disarmed reports landed at any height
            (true, FLY, 0.3, GROUND),  // and resets the estimate
            (true, FLY, f32::INFINITY, AIR),
            (true, FLY, f32::NEG_INFINITY, GROUND),
            (true, LAND, 0.1, GROUND), // a land on the ground: still on the ground
            (true, FLY, 2.0, AIR),
            (true, LAND, 2.0, LANDING), // a land in the air
            (true, LAND, 0.3, LANDING), // between thresholds
            (true, LAND, f32::NAN, UNDEFINED),
            (true, LAND, 0.1, GROUND), // down
            (true, LAND, 1.0, LANDING),
            (false, LAND, 1.0, GROUND), // disarmed (touchdown above home)
        ];

        let mut detector = LandDetector::default();
        for (i, (armed, mode, height_m, expected)) in ticks.into_iter().enumerate() {
            check!(
                detector.update(armed, mode, height_m) == expected,
                "tick {i}"
            );
        }
    }
}
