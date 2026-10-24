use approx::assert_relative_eq;
use assert2::check;
use nalgebra::UnitQuaternion;

use super::*;

const DT: f32 = 0.01;
/// Height that clears the default `min_height_m`.
const CLEAR_M: f32 = 10.0;
/// Past the default airspeed gate.
const FAST_MPS: f32 = 12.0;
/// Below the default airspeed gate.
const SLOW_MPS: f32 = 4.0;

/// A level state heading north, `height_m` above the arming point (at 0), flying north at
/// `ground_mps` with `airspeed_mps`.
fn state(height_m: f32, ground_mps: f32, airspeed_mps: Option<f32>) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, height_m),
        velocity_mps: Vec3::new(ground_mps, 0.0, 0.0),
        airspeed_mps,
        ..Default::default()
    }
}

fn hover() -> VehicleState {
    state(CLEAR_M, 0.0, Some(0.0))
}

fn fast() -> VehicleState {
    state(CLEAR_M, FAST_MPS, Some(FAST_MPS))
}

fn primed() -> LiftTransition {
    primed_with(LiftTransitionConfig::default())
}

fn primed_with(config: LiftTransitionConfig) -> LiftTransition {
    let mut transition = LiftTransition::new(config);
    transition.prime(&state(0.0, 0.0, None));
    transition
}

/// Flies `transition` for `seconds` on `requested` in `state`, returning the last step.
fn fly(
    transition: &mut LiftTransition,
    seconds: f32,
    requested: FlightRegime,
    state: &VehicleState,
) -> LiftTransitionStep {
    let ticks = libm::roundf(seconds / DT) as usize;
    (0..ticks)
        .map(|_| transition.update(DT, requested, state))
        .last()
        .expect("at least one tick")
}

/// `ToPlane`, waiting on airspeed.
fn waiting() -> LiftTransition {
    let mut transition = primed();
    transition.update(DT, FlightRegime::Plane, &hover());
    transition
}

/// `ToPlane`, its lift ramp running.
fn ramping() -> LiftTransition {
    let mut transition = waiting();
    transition.update(DT, FlightRegime::Plane, &fast());
    transition
}

/// `ToPlane` with the lift ramp all but done.
fn ramp_ending() -> LiftTransition {
    let mut transition = ramping();
    let step = fly(&mut transition, 4.99, FlightRegime::Plane, &fast());
    assert!(step.mode == FlightMode::ToPlane);
    transition
}

/// `Plane`, the forward transition done.
fn in_plane() -> LiftTransition {
    let mut transition = ramping();
    let step = fly(&mut transition, 5.05, FlightRegime::Plane, &fast());
    assert!(step.mode == FlightMode::Plane);
    transition
}

/// `ToCopter`, just begun at cruise speed.
fn slowing() -> LiftTransition {
    let mut transition = in_plane();
    transition.update(DT, FlightRegime::Copter, &fast());
    transition
}

/// `ToPlane` past its timeout but for one tick.
fn about_to_time_out() -> LiftTransition {
    let mut transition = waiting();
    fly(&mut transition, 14.99, FlightRegime::Plane, &hover());
    transition
}

/// Each row of the mode table from a single tick: (name, starting transition, request, state,
/// mode, handoff, refusal).
#[test]
fn follows_the_mode_table() {
    type Make = fn() -> LiftTransition;
    type Row = (
        &'static str,
        Make,
        FlightRegime,
        VehicleState,
        FlightMode,
        bool,
        Option<ModeRefusal>,
    );
    let abandoned = || {
        let mut transition = about_to_time_out();
        transition.update(DT, FlightRegime::Plane, &hover());
        transition
    };
    let cases: [Row; 13] = [
        (
            "copter stays",
            primed,
            FlightRegime::Copter,
            hover(),
            FlightMode::Copter,
            false,
            None,
        ),
        (
            "abandoned stays refused",
            abandoned,
            FlightRegime::Plane,
            fast(),
            FlightMode::Copter,
            false,
            Some(ModeRefusal::TransitionTimeout),
        ),
        (
            "too low",
            primed,
            FlightRegime::Plane,
            state(1.0, 0.0, None),
            FlightMode::Copter,
            false,
            Some(ModeRefusal::BelowMinAltitude),
        ),
        (
            "NaN altitude",
            primed,
            FlightRegime::Plane,
            state(f32::NAN, 0.0, None),
            FlightMode::Copter,
            false,
            Some(ModeRefusal::BelowMinAltitude),
        ),
        (
            "forward starts",
            primed,
            FlightRegime::Plane,
            hover(),
            FlightMode::ToPlane,
            false,
            None,
        ),
        (
            "forward cancelled",
            ramping,
            FlightRegime::Copter,
            fast(),
            FlightMode::Copter,
            false,
            None,
        ),
        (
            "forward times out",
            about_to_time_out,
            FlightRegime::Plane,
            hover(),
            FlightMode::Copter,
            false,
            Some(ModeRefusal::TransitionTimeout),
        ),
        (
            "forward completes",
            ramp_ending,
            FlightRegime::Plane,
            fast(),
            FlightMode::Plane,
            true,
            None,
        ),
        (
            "waits on airspeed",
            waiting,
            FlightRegime::Plane,
            state(CLEAR_M, FAST_MPS, Some(SLOW_MPS)),
            FlightMode::ToPlane,
            false,
            None,
        ),
        (
            "back starts",
            in_plane,
            FlightRegime::Copter,
            fast(),
            FlightMode::ToCopter,
            false,
            None,
        ),
        (
            "back cancelled",
            slowing,
            FlightRegime::Plane,
            fast(),
            FlightMode::Plane,
            false,
            None,
        ),
        (
            "back completes slow",
            slowing,
            FlightRegime::Copter,
            state(CLEAR_M, 4.0, Some(10.0)),
            FlightMode::Copter,
            true,
            None,
        ),
        (
            "back continues fast",
            slowing,
            FlightRegime::Copter,
            fast(),
            FlightMode::ToCopter,
            false,
            None,
        ),
    ];
    for (name, make, requested, state, mode, handoff, refusal) in cases {
        let mut transition = make();
        let step = transition.update(DT, requested, &state);
        check!(step.mode == mode, "{name}");
        check!(step.handoff == handoff, "{name}");
        check!(transition.report().refusal == refusal, "{name}");
        check!(transition.report().mode == mode, "{name}");
    }
}

/// The lift share: 1 waiting on airspeed, linear to 0 over `lift_ramp_s` once it passes, 0 in
/// plane flight, and linear back to 1 over `back_ramp_s`, held there until the back
/// transition completes.
#[test]
fn ramps_the_lift_share_over_the_configured_times() {
    let mut transition = waiting();
    check!(fly(&mut transition, 3.0, FlightRegime::Plane, &hover()).lift_share == 1.0);
    // The ramp starts on the tick airspeed passes the gate.
    transition.update(DT, FlightRegime::Plane, &fast());
    // (seconds into the ramp, lift share)
    for (at_s, share) in [(1.0, 0.8), (2.5, 0.5), (4.0, 0.2)] {
        let mut ramp = transition.clone();
        let step = fly(&mut ramp, at_s, FlightRegime::Plane, &fast());
        check!(step.mode == FlightMode::ToPlane, "{at_s}");
        assert_relative_eq!(step.lift_share, share, epsilon = 1e-3);
    }
    // Airspeed dropping back below the gate doesn't stop a running ramp.
    let slowed = state(CLEAR_M, SLOW_MPS, Some(SLOW_MPS));
    let mut ramp = transition.clone();
    assert_relative_eq!(
        fly(&mut ramp, 2.5, FlightRegime::Plane, &slowed).lift_share,
        0.5,
        epsilon = 1e-3
    );

    let mut back = in_plane();
    check!(back.update(DT, FlightRegime::Plane, &fast()).lift_share == 0.0);
    for (at_s, share) in [(1.5, 0.5), (3.0, 1.0), (6.0, 1.0)] {
        let mut ramp = back.clone();
        let step = fly(&mut ramp, at_s, FlightRegime::Copter, &fast());
        check!(step.mode == FlightMode::ToCopter, "{at_s}");
        assert_relative_eq!(step.lift_share, share, epsilon = 1e-2);
    }
    let step = back.cut_short(ModeRefusal::RollError);
    check!(step.lift_share == 0.0, "not in a transition: plane");
}

/// A lift ramp started late in the wait runs to `Plane` past `timeout_s`, which bounds only the
/// wait on airspeed.
#[test]
fn a_started_lift_ramp_outlasts_the_timeout() {
    let mut transition = waiting();
    fly(&mut transition, 12.0, FlightRegime::Plane, &hover());
    let step = fly(&mut transition, 5.0, FlightRegime::Plane, &fast());
    check!(step.mode == FlightMode::ToPlane);
    check!(step.lift_share < 0.01);
    let step = fly(&mut transition, 0.05, FlightRegime::Plane, &fast());
    check!(step.mode == FlightMode::Plane);
    check!(transition.report().refusal.is_none());
}

/// The forward lift ramp starts only on a valid airspeed at the gate: without one (none, NaN,
/// negative) the transition waits and times out to copter.
#[test]
fn forward_ramp_needs_a_valid_airspeed() {
    // (airspeed, starts the ramp)
    let cases = [
        (Some(9.0), true),
        (Some(8.9), false),
        (None, false),
        (Some(f32::NAN), false),
        (Some(-12.0), false),
    ];
    for (airspeed_mps, ramps) in cases {
        let mut transition = waiting();
        let flying = state(CLEAR_M, FAST_MPS, airspeed_mps);
        let step = fly(&mut transition, 1.0, FlightRegime::Plane, &flying);
        check!((step.lift_share < 1.0) == ramps, "{airspeed_mps:?}");
        let step = fly(&mut transition, 14.5, FlightRegime::Plane, &flying);
        let mode = if ramps {
            FlightMode::Plane
        } else {
            FlightMode::Copter
        };
        check!(step.mode == mode, "{airspeed_mps:?}");
    }
}

/// The back transition completes on forward ground speed below `back_speed_mps`, reading the
/// speed along the body `x` axis (not the ground track), falling back to a valid airspeed without a
/// ground velocity, and on its timeout without either.
#[test]
fn back_completes_on_forward_speed_or_its_timeout() {
    let heading_east = UnitQuaternion::from_euler_angles(0.0, 0.0, core::f32::consts::FRAC_PI_2);
    let along = |heading: UnitQuaternion<f32>, velocity_mps: Vec3, airspeed_mps| VehicleState {
        attitude: heading,
        velocity_mps,
        ..state(CLEAR_M, 0.0, airspeed_mps)
    };
    let nan = Vec3::repeat(f32::NAN);
    // (case, state, completes on the first tick)
    let cases = [
        (
            "slow along the nose",
            along(heading_east, Vec3::new(0.0, 4.0, 0.0), None),
            true,
        ),
        (
            "fast along the nose",
            along(heading_east, Vec3::new(0.0, 12.0, 0.0), Some(2.0)),
            false,
        ),
        (
            "climbing along a raised nose",
            along(
                UnitQuaternion::from_euler_angles(0.0, 30.0_f32.to_radians(), 0.0),
                Vec3::new(4.0, 0.0, 4.0),
                None,
            ),
            false,
        ),
        (
            "fast sideways only",
            along(heading_east, Vec3::new(12.0, 0.0, 0.0), None),
            true,
        ),
        (
            "no ground velocity, slow airspeed",
            along(heading_east, nan, Some(4.0)),
            true,
        ),
        (
            "no ground velocity, fast airspeed",
            along(heading_east, nan, Some(12.0)),
            false,
        ),
        ("no speed at all", along(heading_east, nan, None), false),
        (
            "no attitude, slow airspeed",
            along(
                UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(f32::NAN, 0.0, 0.0, 0.0)),
                Vec3::zeros(),
                Some(4.0),
            ),
            true,
        ),
    ];
    for (case, state, completes) in cases {
        let mut transition = slowing();
        let step = transition.update(DT, FlightRegime::Copter, &state);
        check!((step.mode == FlightMode::Copter) == completes, "{case}");
        if completes {
            continue;
        }
        let step = fly(&mut transition, 9.9, FlightRegime::Copter, &state);
        check!(
            step.mode == FlightMode::ToCopter,
            "{case}: before the timeout"
        );
        let handed_off =
            (0..20).any(|_| transition.update(DT, FlightRegime::Copter, &state).handoff);
        check!(handed_off, "{case}: timed out");
        check!(
            transition.report().mode == FlightMode::Copter,
            "{case}: timed out"
        );
    }
}

/// A timed-out forward transition holds `Plane` refused until `Copter` is requested; a
/// standing request refused for height starts once high enough.
#[test]
fn refusals_hold_until_cleared() {
    let mut transition = about_to_time_out();
    transition.update(DT, FlightRegime::Plane, &hover());
    let step = fly(&mut transition, 1.0, FlightRegime::Plane, &fast());
    check!(step.mode == FlightMode::Copter);
    check!(transition.report().refusal == Some(ModeRefusal::TransitionTimeout));
    transition.update(DT, FlightRegime::Copter, &hover());
    check!(transition.report().refusal.is_none());
    check!(transition.update(DT, FlightRegime::Plane, &hover()).mode == FlightMode::ToPlane);

    let mut transition = primed();
    let low = state(2.0, 0.0, None);
    check!(transition.update(DT, FlightRegime::Plane, &low).mode == FlightMode::Copter);
    check!(transition.update(DT, FlightRegime::Plane, &hover()).mode == FlightMode::ToPlane);
}

/// The plane minimum height in these tests, in m above the arming point.
const FLOOR_M: f32 = 20.0;

/// With a plane minimum height, a forward transition needs it to start; plane flight below it
/// falls back to copter, the lift rotors carrying all the lift at once, completing the back
/// transition however `Plane` is still requested, and `Plane` stays refused until `Copter` is
/// requested.
#[test]
fn falls_back_below_the_plane_minimum_height() {
    use FlightRegime::{Copter, Plane};
    use ModeRefusal::{BelowMinAltitude, BelowPlaneMinAltitude};
    let mut transition = primed_with(LiftTransitionConfig {
        plane_min_height_m: FLOOR_M,
        ..Default::default()
    });
    let at = |height_m, speed_mps| state(height_m, speed_mps, Some(speed_mps));
    let step = transition.update(DT, Plane, &at(15.0, 0.0));
    check!(
        step.mode == FlightMode::Copter,
        "past TRAN_MIN_HGT, below the floor"
    );
    check!(transition.report().refusal == Some(BelowMinAltitude));
    check!(fly(&mut transition, 6.0, Plane, &at(30.0, FAST_MPS)).mode == FlightMode::Plane);

    // (what, request, height, speed, mode, refusal, fallen back)
    let ticks = [
        (
            "plane above the floor",
            Plane,
            21.0,
            FAST_MPS,
            FlightMode::Plane,
            None,
            false,
        ),
        (
            "below it",
            Plane,
            19.0,
            FAST_MPS,
            FlightMode::ToCopter,
            Some(BelowPlaneMinAltitude),
            true,
        ),
        (
            "still fast",
            Plane,
            19.0,
            FAST_MPS,
            FlightMode::ToCopter,
            Some(BelowPlaneMinAltitude),
            true,
        ),
        (
            "slowed",
            Plane,
            19.0,
            SLOW_MPS,
            FlightMode::Copter,
            Some(BelowPlaneMinAltitude),
            true,
        ),
        (
            "high again, still refused",
            Plane,
            30.0,
            0.0,
            FlightMode::Copter,
            Some(BelowPlaneMinAltitude),
            true,
        ),
        (
            "cleared",
            Copter,
            30.0,
            0.0,
            FlightMode::Copter,
            None,
            false,
        ),
        (
            "plane again",
            Plane,
            30.0,
            0.0,
            FlightMode::ToPlane,
            None,
            false,
        ),
    ];
    for (what, request, height_m, speed_mps, mode, refusal, fallen_back) in ticks {
        let step = transition.update(DT, request, &at(height_m, speed_mps));
        check!(step.mode == mode, "{what}");
        check!(transition.report().refusal == refusal, "{what}");
        check!(transition.fallen_back() == fallen_back, "{what}");
        if mode == FlightMode::ToCopter {
            check!(step.lift_share == 1.0, "{what}: no back ramp (#493)");
        }
    }

    let mut floorless = in_plane();
    let underground = at(-100.0, FAST_MPS);
    check!(floorless.update(DT, Plane, &underground).mode == FlightMode::Plane);
}

/// Cut short, a forward transition is abandoned (refused until `Copter`), a back transition
/// completes with a handoff, and any other mode is unchanged.
#[test]
fn cut_short_abandons_forward_and_completes_back() {
    let mut forward = ramping();
    let step = forward.cut_short(ModeRefusal::RollError);
    check!((step.mode, step.lift_share, step.handoff) == (FlightMode::Copter, 1.0, false));
    check!(forward.report().refusal == Some(ModeRefusal::RollError));
    check!(forward.update(DT, FlightRegime::Plane, &fast()).mode == FlightMode::Copter);

    let mut back = slowing();
    let step = back.cut_short(ModeRefusal::RollError);
    check!((step.mode, step.lift_share, step.handoff) == (FlightMode::Copter, 1.0, true));

    let mut copter = primed();
    let step = copter.cut_short(ModeRefusal::RollError);
    check!((step.mode, step.handoff) == (FlightMode::Copter, false));
    check!(copter.report().refusal.is_none());
}

/// Before `prime` there is no height reference: `Plane` is refused. `prime` returns to copter
/// and clears an abandoned transition.
#[test]
fn prime_sets_the_height_reference_and_resets() {
    let mut unprimed = LiftTransition::new(LiftTransitionConfig::default());
    check!(unprimed.update(DT, FlightRegime::Plane, &hover()).mode == FlightMode::Copter);
    check!(unprimed.report().refusal == Some(ModeRefusal::BelowMinAltitude));

    let mut transition = about_to_time_out();
    transition.update(DT, FlightRegime::Plane, &hover());
    transition.prime(&state(0.0, 0.0, None));
    check!(
        transition.report()
            == FlightModeReport {
                mode: FlightMode::Copter,
                refusal: None
            }
    );
    check!(transition.update(DT, FlightRegime::Plane, &hover()).mode == FlightMode::ToPlane);
    // A ramp from before `prime` doesn't carry over.
    check!(
        transition
            .update(DT, FlightRegime::Plane, &hover())
            .lift_share
            == 1.0
    );
}

/// A new config applies from the next tick of a running transition.
#[test]
fn set_config_applies_to_a_running_transition() {
    let mut transition = ramping();
    fly(&mut transition, 1.0, FlightRegime::Plane, &fast());
    let config = LiftTransitionConfig {
        lift_ramp_s: 2.0,
        ..LiftTransitionConfig::default()
    };
    check!(transition.set_config(config) == Ok(()));
    check!(*transition.config() == config);
    let step = fly(&mut transition, 1.05, FlightRegime::Plane, &fast());
    check!(step.mode == FlightMode::Plane);
    check!(
        transition.set_config(LiftTransitionConfig {
            back_ramp_s: 0.0,
            ..config
        }) == Err(InvalidConfig::new("back_ramp_s", "positive and finite"))
    );
    check!(*transition.config() == config);
}

/// Each field's range is checked on its own, naming the field.
#[test]
fn validates_each_config_field() {
    let good = LiftTransitionConfig::default();
    check!(good.validate() == Ok(()));
    type Edit = fn(&mut LiftTransitionConfig, f32);
    // (field, edit, rejected values, accepted edge)
    let cases: [(&str, Edit, &[f32], f32); 12] = [
        (
            "min_airspeed_mps",
            |c, v| c.min_airspeed_mps = v,
            &[-0.1, f32::NAN],
            0.0,
        ),
        (
            "lift_ramp_s",
            |c, v| c.lift_ramp_s = v,
            &[0.0, f32::INFINITY],
            0.1,
        ),
        ("timeout_s", |c, v| c.timeout_s = v, &[0.0, -1.0], 0.1),
        (
            "min_height_m",
            |c, v| c.min_height_m = v,
            &[-1.0, f32::NAN],
            0.0,
        ),
        (
            "plane_min_height_m",
            |c, v| c.plane_min_height_m = v,
            &[-1.0, f32::NAN],
            0.0,
        ),
        (
            "back_ramp_s",
            |c, v| c.back_ramp_s = v,
            &[0.0, f32::NAN],
            0.1,
        ),
        (
            "back_speed_mps",
            |c, v| c.back_speed_mps = v,
            &[-0.1, f32::INFINITY],
            0.0,
        ),
        (
            "back_timeout_s",
            |c, v| c.back_timeout_s = v,
            &[0.0, f32::NAN],
            0.1,
        ),
        (
            "forward_thrust",
            |c, v| c.forward_thrust = v,
            &[-0.1, 1.1, f32::NAN],
            0.0,
        ),
        (
            "pusher_slew_per_s",
            |c, v| c.pusher_slew_per_s = v,
            &[0.0, f32::INFINITY],
            0.01,
        ),
        ("handoff_s", |c, v| c.handoff_s = v, &[-0.1, f32::NAN], 0.0),
        (
            "roll_limit_rad",
            |c, v| c.roll_limit_rad = v,
            &[0.0, 3.2, f32::NAN],
            core::f32::consts::PI,
        ),
    ];
    for (field, edit, rejected, edge) in cases {
        for &value in rejected {
            let mut config = good;
            edit(&mut config, value);
            check!(
                config.validate().map_err(|e| e.field) == Err(field),
                "{field} = {value}"
            );
        }
        let mut config = good;
        edit(&mut config, edge);
        check!(config.validate() == Ok(()), "{field} = {edge}");
    }
}

#[test]
#[should_panic(expected = "invalid LiftTransitionConfig")]
fn new_panics_on_an_invalid_config() {
    LiftTransition::new(LiftTransitionConfig {
        lift_ramp_s: 0.0,
        ..LiftTransitionConfig::default()
    });
}
