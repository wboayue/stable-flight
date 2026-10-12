use core::f32::consts::FRAC_PI_2;

use approx::assert_relative_eq;
use assert2::check;
use nalgebra::{Quaternion, UnitQuaternion};

use super::*;
use crate::Vec3;

const DT: f32 = 0.01;
/// Height that clears the default `min_height_m`.
const CLEAR_M: f32 = 10.0;

/// A state with the nose `tilt_deg` from vertical, `height_m` above the arming point (at 0),
/// and `airspeed_mps`.
fn state(tilt_deg: f32, height_m: f32, airspeed_mps: Option<f32>) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, height_m),
        attitude: UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2 - tilt_deg.to_radians(), 0.0),
        airspeed_mps,
        ..Default::default()
    }
}

fn hover() -> VehicleState {
    state(0.0, CLEAR_M, None)
}

/// A primed transition with the default config, armed at altitude 0.
fn primed() -> Transition {
    primed_with(TransitionConfig::default())
}

fn primed_with(config: TransitionConfig) -> Transition {
    let mut transition = Transition::new(config);
    transition.prime(&state(0.0, 0.0, None));
    transition
}

/// `transition` driven into `Plane` (forward transition completed).
fn in_plane() -> Transition {
    let mut transition = primed();
    transition.update(DT, FlightRegime::Plane, &hover());
    let step = transition.update(DT, FlightRegime::Plane, &state(50.0, CLEAR_M, None));
    assert!(step.mode == FlightMode::Plane && step.handoff);
    transition
}

/// Each row of the mode table from a single tick: (name, starting transition, request,
/// state, mode, handoff, refusal).
#[test]
fn follows_the_mode_table() {
    let in_to_plane = || {
        let mut transition = primed();
        transition.update(DT, FlightRegime::Plane, &hover());
        transition
    };
    let in_to_copter = || {
        let mut transition = in_plane();
        transition.update(DT, FlightRegime::Copter, &state(80.0, CLEAR_M, None));
        transition
    };
    type Make = fn() -> Transition;
    type Row = (
        &'static str,
        Make,
        FlightRegime,
        VehicleState,
        FlightMode,
        bool,
        Option<ModeRefusal>,
    );
    let cases: [Row; 12] = [
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
            "too low",
            primed,
            FlightRegime::Plane,
            state(0.0, 4.0, None),
            FlightMode::Copter,
            false,
            Some(ModeRefusal::BelowMinAltitude),
        ),
        (
            "NaN altitude",
            primed,
            FlightRegime::Plane,
            state(0.0, f32::NAN, None),
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
            "forward aborts",
            in_to_plane,
            FlightRegime::Copter,
            hover(),
            FlightMode::Copter,
            false,
            None,
        ),
        (
            "forward ramps",
            in_to_plane,
            FlightRegime::Plane,
            state(30.0, CLEAR_M, None),
            FlightMode::ToPlane,
            false,
            None,
        ),
        (
            "forward completes",
            in_to_plane,
            FlightRegime::Plane,
            state(46.0, CLEAR_M, None),
            FlightMode::Plane,
            true,
            None,
        ),
        (
            "plane stays",
            in_plane,
            FlightRegime::Plane,
            state(85.0, CLEAR_M, None),
            FlightMode::Plane,
            false,
            None,
        ),
        (
            "back starts",
            in_plane,
            FlightRegime::Copter,
            state(85.0, CLEAR_M, None),
            FlightMode::ToCopter,
            false,
            None,
        ),
        (
            "back aborts",
            in_to_copter,
            FlightRegime::Plane,
            state(70.0, CLEAR_M, None),
            FlightMode::Plane,
            false,
            None,
        ),
        (
            "back ramps",
            in_to_copter,
            FlightRegime::Copter,
            state(60.0, CLEAR_M, None),
            FlightMode::ToCopter,
            false,
            None,
        ),
        (
            "back completes",
            in_to_copter,
            FlightRegime::Copter,
            state(44.0, CLEAR_M, None),
            FlightMode::Copter,
            true,
            None,
        ),
    ];

    for (name, make, requested, vehicle, mode, handoff, refusal) in cases {
        let mut transition = make();
        let step = transition.update(DT, requested, &vehicle);
        check!(step.mode == mode, "{name}");
        check!(step.handoff == handoff, "{name}");
        check!(
            transition.report() == FlightModeReport { mode, refusal },
            "{name}"
        );
        check!(
            step.tilt_target_rad.is_some()
                == matches!(mode, FlightMode::ToPlane | FlightMode::ToCopter),
            "{name}"
        );
    }
}

/// A standing request ramps the target on every tick without restarting, and completes with
/// exactly one handoff.
#[test]
fn repeated_requests_continue_the_ramp() {
    let mut transition = primed();
    let mut last = -1.0_f32;
    let mut handoffs = 0;
    for tick in 0..200 {
        // The vehicle follows the target a tick late.
        let tilt_deg = last.max(0.0).to_degrees();
        let step = transition.update(DT, FlightRegime::Plane, &state(tilt_deg, CLEAR_M, None));
        handoffs += usize::from(step.handoff);
        let Some(target) = step.tilt_target_rad else {
            break;
        };
        check!(target > last, "tick {tick}");
        last = target;
    }
    check!(transition.report().mode == FlightMode::Plane);
    check!(handoffs == 1);
}

/// The forward ramp starts at the vehicle's tilt, rises at the configured rate, and stops at
/// `forward_max_tilt_rad`; the back ramp falls to zero.
#[test]
fn ramps_from_the_vehicle_tilt_at_the_configured_rate() {
    let config = TransitionConfig::default();
    let mut transition = primed();
    // Already 10° over when the request arrives: the ramp starts there, not at 0.
    let first = transition.update(DT, FlightRegime::Plane, &state(10.0, CLEAR_M, None));
    assert_relative_eq!(
        first.tilt_target_rad.unwrap(),
        10.0_f32.to_radians(),
        epsilon = 1e-5
    );
    // Airspeed below the gate would hold it at max; here it never completes since the vehicle
    // stays at 10°.
    let after = transition.update(0.5, FlightRegime::Plane, &state(10.0, CLEAR_M, None));
    assert_relative_eq!(
        after.tilt_target_rad.unwrap(),
        10.0_f32.to_radians() + config.forward_rate_radps * 0.5,
        epsilon = 1e-5
    );
    let capped = transition.update(5.0, FlightRegime::Plane, &state(10.0, CLEAR_M, None));
    assert_relative_eq!(capped.tilt_target_rad.unwrap(), config.forward_max_tilt_rad);

    let mut transition = in_plane();
    let back = transition.update(DT, FlightRegime::Copter, &state(80.0, CLEAR_M, None));
    assert_relative_eq!(
        back.tilt_target_rad.unwrap(),
        80.0_f32.to_radians(),
        epsilon = 1e-4
    );
    let down = transition.update(5.0, FlightRegime::Copter, &state(80.0, CLEAR_M, None));
    check!(down.tilt_target_rad == Some(0.0));
}

/// With a gate set, the forward transition completes on tilt only once a valid airspeed reaches
/// it; without a valid airspeed (none, NaN, negative), tilt alone completes it.
#[test]
fn forward_completion_waits_on_valid_airspeed() {
    let config = TransitionConfig {
        min_airspeed_mps: 12.0,
        ..Default::default()
    };
    // (airspeed, completes)
    let cases = [
        (Some(11.0), false),
        (Some(12.0), true),
        (None, true),
        (Some(f32::NAN), true),
        (Some(-1.0), true),
    ];
    for (airspeed_mps, completes) in cases {
        let mut transition = primed_with(config);
        transition.update(DT, FlightRegime::Plane, &hover());
        let step = transition.update(DT, FlightRegime::Plane, &state(60.0, CLEAR_M, airspeed_mps));
        check!(
            (step.mode == FlightMode::Plane) == completes,
            "{airspeed_mps:?}"
        );
    }
}

/// A NaN attitude never completes either way: the ramp runs from hover (forward) or level
/// flight (back) and the timeout ends it.
#[test]
fn nan_attitude_waits_for_the_timeout() {
    let nan = VehicleState {
        attitude: UnitQuaternion::new_unchecked(Quaternion::new(f32::NAN, 0.0, 0.0, 0.0)),
        ..hover()
    };
    let mut transition = primed();
    let step = transition.update(DT, FlightRegime::Plane, &nan);
    check!(step.tilt_target_rad == Some(0.0)); // from hover
    check!(transition.update(DT, FlightRegime::Plane, &nan).mode == FlightMode::ToPlane);

    let mut transition = in_plane();
    let step = transition.update(DT, FlightRegime::Copter, &nan);
    check!(step.tilt_target_rad == Some(FRAC_PI_2)); // from level flight
    check!(transition.update(DT, FlightRegime::Copter, &nan).mode == FlightMode::ToCopter);
}

/// A forward transition that runs out of time falls back to copter and stays refused while
/// `Plane` is still requested; a `Copter` request clears it. A back transition that runs out
/// of time completes.
#[test]
fn timeouts_revert_forward_and_complete_back() {
    let config = TransitionConfig {
        timeout_s: 0.35,
        ..Default::default()
    };
    let stuck = state(10.0, CLEAR_M, None);
    let mut transition = primed_with(config);
    transition.update(DT, FlightRegime::Plane, &stuck);
    let modes: Vec<_> = (0..4)
        .map(|_| transition.update(0.1, FlightRegime::Plane, &stuck).mode)
        .collect();
    check!(
        modes
            == [
                FlightMode::ToPlane,
                FlightMode::ToPlane,
                FlightMode::ToPlane,
                FlightMode::Copter
            ]
    );
    let timed_out = FlightModeReport {
        mode: FlightMode::Copter,
        refusal: Some(ModeRefusal::TransitionTimeout),
    };
    check!(transition.report() == timed_out);
    check!(transition.update(DT, FlightRegime::Plane, &stuck).mode == FlightMode::Copter);
    check!(transition.report() == timed_out);

    transition.update(DT, FlightRegime::Copter, &stuck);
    check!(transition.report() == FlightModeReport::default());
    check!(transition.update(DT, FlightRegime::Plane, &stuck).mode == FlightMode::ToPlane);

    let mut transition = in_plane();
    transition.set_config(config).expect("valid");
    let level = state(85.0, CLEAR_M, None);
    transition.update(DT, FlightRegime::Copter, &level);
    let steps: Vec<_> = (0..4)
        .map(|_| transition.update(0.1, FlightRegime::Copter, &level))
        .map(|step| (step.mode, step.handoff))
        .collect();
    check!(
        steps
            == [
                (FlightMode::ToCopter, false),
                (FlightMode::ToCopter, false),
                (FlightMode::ToCopter, false),
                (FlightMode::Copter, true),
            ]
    );
}

/// A standing `Plane` request refused for height starts the transition once the vehicle
/// climbs past `min_height_m`, measured from where it was primed.
#[test]
fn standing_request_starts_once_high_enough() {
    let mut transition = Transition::new(TransitionConfig::default());
    transition.prime(&state(0.0, 100.0, None)); // armed at 100 m above the datum
    // (height above the datum, mode)
    let climb = [
        (100.0, FlightMode::Copter),
        (104.0, FlightMode::Copter),
        (105.0, FlightMode::ToPlane),
    ];
    for (altitude_m, mode) in climb {
        let step = transition.update(DT, FlightRegime::Plane, &state(0.0, altitude_m, None));
        check!(step.mode == mode, "{altitude_m}");
    }
}

/// `prime` returns to copter and clears a timeout, whatever the mode.
#[test]
fn prime_resets_mode_and_timeout() {
    let mut flying = in_plane();
    flying.prime(&hover());
    check!(flying.report() == FlightModeReport::default());

    let mut timed_out = primed_with(TransitionConfig {
        timeout_s: 0.05,
        ..Default::default()
    });
    let stuck = state(10.0, CLEAR_M, None);
    timed_out.update(DT, FlightRegime::Plane, &stuck);
    timed_out.update(0.1, FlightRegime::Plane, &stuck);
    check!(timed_out.report().refusal == Some(ModeRefusal::TransitionTimeout));
    timed_out.prime(&state(0.0, 0.0, None));
    check!(timed_out.update(DT, FlightRegime::Plane, &stuck).mode == FlightMode::ToPlane);
}

/// Each field out of range is named.
#[test]
fn validates_each_config_field() {
    type Break = fn(&mut TransitionConfig);
    let cases: [(Break, &str); 10] = [
        (|c| c.forward_rate_radps = 0.0, "forward_rate_radps"),
        (|c| c.forward_done_rad = 0.0, "forward_done_rad"),
        (|c| c.forward_done_rad = FRAC_PI_2, "forward_done_rad"),
        (
            |c| c.forward_max_tilt_rad = c.forward_done_rad - 0.1,
            "forward_max_tilt_rad",
        ),
        (|c| c.forward_max_tilt_rad = 1.6, "forward_max_tilt_rad"),
        (|c| c.min_airspeed_mps = -1.0, "min_airspeed_mps"),
        (|c| c.back_rate_radps = f32::NAN, "back_rate_radps"),
        (|c| c.back_done_rad = -0.1, "back_done_rad"),
        (|c| c.timeout_s = 0.0, "timeout_s"),
        (|c| c.min_height_m = f32::INFINITY, "min_height_m"),
    ];
    check!(TransitionConfig::default().validate().is_ok());
    for (corrupt, field) in cases {
        let mut config = TransitionConfig::default();
        corrupt(&mut config);
        let error = config.validate().expect_err(field);
        check!(error.field == field);

        let mut transition = primed();
        check!(transition.set_config(config).is_err(), "{field}");
        check!(*transition.config() == TransitionConfig::default());
    }
}

#[test]
#[should_panic(expected = "invalid TransitionConfig")]
fn new_panics_on_an_invalid_config() {
    Transition::new(TransitionConfig {
        timeout_s: -1.0,
        ..Default::default()
    });
}

/// The blend runs linearly from the start value to the demand over the duration, follows a
/// demand that changes mid-blend, then passes the demand through; NaN counts as 0.
#[test]
fn thrust_handoff_blends_then_passes_through() {
    let mut handoff = ThrustHandoff::new(0.4);
    check!(!handoff.active());
    check!(handoff.apply(DT, 0.7) == 0.7);

    handoff.start(0.8);
    check!(handoff.active());
    // (dt, demanded, expected)
    let cases = [
        (0.1, 0.4, 0.7), // a quarter of the way to 0.4
        (0.1, 0.0, 0.4), // half way to a new demand of 0
        (0.1, 0.0, 0.2), // three quarters
        (0.1, 0.0, 0.0), // done
        (0.1, 0.3, 0.3), // passes through
        (0.1, f32::NAN, 0.0),
        (0.1, 1.5, 1.0),
    ];
    for (dt, demanded, expected) in cases {
        assert_relative_eq!(handoff.apply(dt, demanded), expected, epsilon = 1e-5);
    }
    check!(!handoff.active());

    handoff.start(f32::NAN);
    assert_relative_eq!(handoff.apply(0.2, 0.6), 0.3, epsilon = 1e-5);

    let mut instant = ThrustHandoff::new(0.0);
    instant.start(0.9);
    check!(!instant.active());
    check!(instant.apply(DT, 0.2) == 0.2);
}

#[test]
#[should_panic(expected = "invalid ThrustHandoff")]
fn thrust_handoff_panics_on_a_negative_duration() {
    ThrustHandoff::new(-0.1);
}
