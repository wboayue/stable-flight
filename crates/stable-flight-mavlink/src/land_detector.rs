//! Estimates whether the vehicle is on the ground or in the air: see [`LandDetector`].

use mavlink::common::MavLandedState;

// Both thresholds are repeated in the `LandDetector` rustdoc and this crate's README; update
// them together.

/// Armed and above this height over home: in the air.
const IN_AIR_HEIGHT_M: f32 = 0.5;
/// Armed and below this height over home: back on the ground.
const LANDED_HEIGHT_M: f32 = 0.2;

/// Estimates the landed state for `EXTENDED_SYS_STATE` from the arm state and height above home.
///
/// Telemetry only: nothing in the control loop reads it. Disarmed reports on the ground (as
/// PX4's land detector does), so a vehicle disarmed in flight reads landed while it falls.
/// Armed, it reports in the air above 0.5 m and landed again below 0.2 m; between the two the
/// previous state holds, so hovering near one doesn't flicker the state. Height alone can't
/// tell landing armed above home (a rooftop) from hovering, so that reads in the air until
/// disarmed.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::LandDetector;
/// use stable_flight_mavlink::mavlink::common::MavLandedState;
///
/// let mut detector = LandDetector::default();
/// assert_eq!(detector.update(true, 2.0), MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// assert_eq!(detector.update(true, 0.3), MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// assert_eq!(detector.update(false, 0.3), MavLandedState::MAV_LANDED_STATE_ON_GROUND);
/// ```
#[derive(Debug, Default)]
pub struct LandDetector {
    in_air: bool,
}

impl LandDetector {
    /// Advances by one tick. `height_m` is the height above home; NaN (no home, or no finite
    /// altitude) reports undefined while armed and keeps the previous estimate.
    pub fn update(&mut self, armed: bool, height_m: f32) -> MavLandedState {
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
        if self.in_air {
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
    fn reports_in_air_above_the_takeoff_height_with_hysteresis() {
        use MavLandedState::{
            MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_ON_GROUND as GROUND,
            MAV_LANDED_STATE_UNDEFINED as UNDEFINED,
        };
        // (armed, height_m, expected), applied in sequence to one detector.
        let ticks = [
            (false, 0.0, GROUND),
            (true, 0.0, GROUND),         // armed on the ground
            (true, 0.4, GROUND),         // between thresholds: stays landed
            (true, 0.6, AIR),            // took off
            (true, 0.3, AIR),            // between thresholds: stays in the air
            (true, f32::NAN, UNDEFINED), // no height
            (true, 0.3, AIR),            // NaN kept the previous estimate
            (true, 0.1, GROUND),         // landed
            (true, 1.0, AIR),
            (false, 1.0, GROUND), // disarmed reports landed at any height
            (true, 0.3, GROUND),  // and resets the estimate
            (true, f32::INFINITY, AIR),
            (true, f32::NEG_INFINITY, GROUND),
        ];

        let mut detector = LandDetector::default();
        for (i, (armed, height_m, expected)) in ticks.into_iter().enumerate() {
            check!(detector.update(armed, height_m) == expected, "tick {i}");
        }
    }
}
