use assert2::check;

use super::*;

#[test]
fn latches_home_at_the_first_finite_position_then_at_each_arming() {
    let nan = f32::NAN;
    // (armed, position_m, expected latched, expected home_m after)
    let ticks = [
        (false, [nan, 0.0, 0.0], false, None), // no finite position: no home
        (false, [1.0, 0.0, 2.0], true, Some([1.0, 0.0, 2.0])), // first finite position
        (false, [5.0, 0.0, 2.0], false, Some([1.0, 0.0, 2.0])), // moved while disarmed
        (true, [5.0, 0.0, 3.0], true, Some([5.0, 0.0, 3.0])), // arming re-latches
        (true, [9.0, 0.0, 9.0], false, Some([5.0, 0.0, 3.0])), // flying
        (false, [9.0, 0.0, 0.0], false, Some([5.0, 0.0, 3.0])),
        (true, [0.0, 0.0, nan], false, Some([5.0, 0.0, 3.0])), // arming on a non-finite position
        (true, [7.0, 0.0, 1.0], true, Some([7.0, 0.0, 1.0])),  // the next finite one latches
        (true, [8.0, 0.0, 4.0], false, Some([7.0, 0.0, 1.0])),
        (false, [nan, nan, nan], false, Some([7.0, 0.0, 1.0])),
        (true, [nan, 0.0, 0.0], false, Some([7.0, 0.0, 1.0])),
        (false, [2.0, 0.0, 0.0], false, Some([7.0, 0.0, 1.0])), // disarmed first: latch dropped
    ];

    let mut home = HomeTracker::default();
    for (i, (armed, position_m, latched, home_m)) in ticks.into_iter().enumerate() {
        check!(home.update(armed, position_m.into()) == latched, "tick {i}");
        check!(home.home_m() == home_m.map(Vec3::from), "tick {i}");
    }
}
