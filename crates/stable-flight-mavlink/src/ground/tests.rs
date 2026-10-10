use assert2::check;

use super::*;

#[test]
fn reports_home_and_landed_state_changes_with_height_measured_from_home() {
    use MavLandedState::{
        MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_ON_GROUND as GROUND,
        MAV_LANDED_STATE_UNDEFINED as UNDEFINED,
    };
    // (armed, altitude_m, expected (home changed, landed changed), expected landed state)
    let ticks = [
        (true, f32::NAN, (false, false), UNDEFINED), // no home yet: undefined, as initialized
        (true, 10.0, (true, true), GROUND),          // home latched at 10 m: height 0
        (true, 10.4, (false, false), GROUND),        // below the takeoff height over home
        (true, 12.0, (false, true), AIR),            // took off
        (true, 11.0, (false, false), AIR),
        (true, 10.1, (false, true), GROUND), // landed
        (false, 10.1, (false, false), GROUND),
        (true, 30.0, (true, false), GROUND), // arming re-latches home: height 0 again
        (true, 30.6, (false, true), AIR),
    ];

    let mut ground = GroundTracker::default();
    for (i, (armed, altitude_m, (home, landed), state)) in ticks.into_iter().enumerate() {
        let changes = ground.update(armed, ControlMode::Attitude, [0.0, 0.0, altitude_m].into());
        check!(changes == GroundChanges { home, landed }, "tick {i}");
        check!(ground.landed_state() == state, "tick {i}");
    }
    check!(ground.home_m() == Some([0.0, 0.0, 30.0].into()));
}
