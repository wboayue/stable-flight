use assert2::check;
use stable_flight::commanders::{PlaneTuneHoldConfig, RouteEnd};
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use stable_flight::tuning::autotune::{AutotuneAbort, AutotuneEvent, Outcome};
use stable_flight::tuning::{Measurement, RateModel, StepMetrics, StepSummary, WingModel};
use stable_flight::view::tailsitter_rotation;
use stable_flight::{ControlInput, FlightRegime, Vec2, Vec3, VehicleState, WaypointCommand};

use super::*;

/// A script hovering at 5 m over its start, holding there as the autotune examples' do.
fn hover_script() -> ScriptCommander {
    let hover = WaypointCommand {
        altitude_m: 5.0,
        ..Default::default()
    };
    ScriptCommander::from_waypoints(vec![hover])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold)
}

/// At rest on the ground at the origin.
fn at_rest(altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, altitude_m),
        ..Default::default()
    }
}

/// The commander flies the script until `land` is set, then lands where it is and disarms
/// once the touchdown detector sees the vehicle down.
#[test]
fn then_land_flies_the_script_then_lands_and_disarms() {
    let (mut commander, switch) = ThenLand::new(hover_script());
    let state = at_rest(5.0);

    let scripted = commander.next_command(0.004, &state);
    check!(!matches!(scripted.input, ControlInput::Land(_)));

    switch.land();
    check!(switch.is_landing());
    let landing = commander.next_command(0.004, &state);
    check!(matches!(landing.input, ControlInput::Land(_)));
    check!(landing.arm);
    // The touchdown detector counts a stop only after the descent has begun.
    let sinking = VehicleState {
        velocity_mps: Vec3::new(0.0, 0.0, -0.5),
        ..state
    };
    check!(commander.next_command(0.004, &sinking).arm);
    let disarmed = (0..500).any(|_| !commander.next_command(0.004, &state).arm);
    check!(disarmed, "still armed 2 s after stopping");
}

/// The autotune starts once the script reaches its hover point and the vehicle is within
/// 0.5 m of its altitude: (altitude, at the hover point).
#[test]
fn the_hover_point_needs_the_last_waypoint_and_its_altitude() {
    let (mut commander, _) = ThenLand::new(hover_script());
    check!(
        !commander.at_hover_point(&at_rest(5.0)),
        "before the script ran"
    );
    for _ in 0..10 {
        commander.next_command(0.004, &at_rest(5.0));
    }
    for (altitude_m, at) in [(5.0, true), (5.4, true), (5.6, false), (4.0, false)] {
        check!(
            commander.at_hover_point(&at_rest(altitude_m)) == at,
            "altitude {altitude_m}"
        );
    }
}

/// A plane flight's script: one leg at 100 m over its start, held there.
fn plane_script() -> ScriptCommander {
    let leg = WaypointCommand {
        altitude_m: 100.0,
        ..Default::default()
    };
    ScriptCommander::from_waypoints(vec![leg])
        .expect("one waypoint")
        .with_route_end(RouteEnd::Hold)
}

/// Cruising at `altitude_m`, `north_m` north of the origin.
fn cruising(north_m: f32, altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(north_m, 0.0, altitude_m),
        airspeed_mps: Some(14.0),
        ..Default::default()
    }
}

/// Home at 30 m over the start, hovering back from within 30 m of it, turning back 150 m past
/// it.
const HOME: PlaneHome = PlaneHome {
    altitude_m: 30.0,
    radius_m: 30.0,
    overfly_m: 150.0,
};

const HOLD: fn() -> PlaneTuneHoldConfig = || PlaneTuneHoldConfig {
    recover_to: FlightRegime::Copter,
    ..PlaneTuneHoldConfig::new(0.85, 14.0)
};

/// A home that would never hover back, or would turn back before reaching the start, is
/// refused at construction: (radius, overfly).
#[test]
fn a_home_that_never_hovers_back_panics() {
    for (radius_m, overfly_m) in [(0.0, 150.0), (-5.0, 150.0), (30.0, 30.0), (30.0, -10.0)] {
        let home = PlaneHome {
            radius_m,
            overfly_m,
            ..HOME
        };
        let built = std::panic::catch_unwind(|| PlaneTuneFlight::new(plane_script(), home));
        check!(built.is_err(), "radius {radius_m} m, overfly {overfly_m} m");
    }
}

/// The flight flies its script until handed over, then the hold, whose turns and recovery the
/// switch reaches; outside the hold they do nothing.
#[test]
fn the_plane_flight_flies_the_script_then_the_hold() {
    let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
    let state = cruising(0.0, 100.0);
    switch.allow_turn(true);
    switch.recover();
    check!(!switch.turning());

    let scripted = commander.next_command(0.004, &state);
    check!(matches!(scripted.input, ControlInput::Waypoint(_)));
    check!(!switch.is_holding());

    switch.hold(HOLD());
    check!(switch.is_holding());
    let held = commander.next_command(0.004, &state);
    check!(matches!(held.input, ControlInput::AbsoluteAttitude(_)));
    check!(held.regime == FlightRegime::Plane);
    check!(!switch.turning());
    // A second handover keeps the running hold.
    switch.hold(PlaneTuneHoldConfig::new(0.2, 14.0));
    let ControlInput::AbsoluteAttitude(attitude) = commander.next_command(0.004, &state).input
    else {
        unreachable!("the hold flies attitudes");
    };
    check!(attitude.thrust == 0.85);
    check!(!switch.is_finished());
}

/// The last leg is the one toward the last waypoint, until it is reached: (position north,
/// on the last leg) after flying the first waypoint over the start.
#[test]
fn the_last_leg_ends_at_its_waypoint() {
    let leg = |north_m| WaypointCommand {
        position_m: Vec2::new(north_m, 0.0),
        altitude_m: 100.0,
        ..Default::default()
    };
    let script = ScriptCommander::from_waypoints(vec![leg(0.0), leg(1000.0)])
        .expect("two waypoints")
        .with_route_end(RouteEnd::Hold);
    let (mut commander, _) = PlaneTuneFlight::new(script, HOME);
    check!(!commander.on_last_leg(), "toward the first waypoint");
    for (north_m, last) in [(0.0, true), (500.0, true), (1000.0, false)] {
        commander.next_command(0.004, &cruising(north_m, 100.0));
        check!(commander.on_last_leg() == last, "at {north_m} m");
    }
}

/// Past the return radius, flying away, and allowed, the hold turns back, which the switch
/// reports.
#[test]
fn the_switch_reports_the_holds_turn() {
    let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
    switch.hold(HOLD());
    let flying_north = |north_m| VehicleState {
        velocity_mps: Vec3::new(14.0, 0.0, 0.0),
        ..cruising(north_m, 100.0)
    };
    commander.next_command(0.004, &flying_north(0.0));
    switch.allow_turn(true);
    commander.next_command(0.004, &flying_north(700.0));
    check!(switch.turning());
}

/// Finished from a hold that flew on, the flight flies a plane leg through the start at the
/// home altitude and trim airspeed, hovers back over the start once within reach, and lands on
/// it once within the waypoint acceptance radius: (north, altitude, phase).
#[test]
fn the_plane_flight_comes_home_in_plane_hovers_over_the_start_then_lands() {
    #[derive(Debug, PartialEq)]
    enum Home {
        Leg,
        Hover,
        Land,
    }
    for (north_m, altitude_m, expected) in [
        (500.0, 100.0, Home::Leg),
        (20.0, 100.0, Home::Leg),
        (500.0, 30.0, Home::Leg),
        (35.0, 30.0, Home::Leg),
        (25.0, 35.0, Home::Hover),
        (5.0, 30.0, Home::Hover),
        (2.0, 30.0, Home::Land),
    ] {
        let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
        switch.hold(HOLD());
        commander.next_command(0.004, &cruising(0.0, 100.0));
        switch.finish();
        check!(switch.is_finished());
        let command = commander.next_command(0.004, &cruising(north_m, altitude_m));
        let phase = match command.input {
            ControlInput::Waypoint(home) if command.regime == FlightRegime::Plane => {
                check!(home.cruise_speed_mps == 14.0);
                check!(
                    home.velocity_mps == Some(Vec2::new(-14.0, 0.0)),
                    "south, to the start"
                );
                check!(home.position_m == Vec2::zeros());
                check!(home.altitude_m == 30.0);
                Home::Leg
            }
            ControlInput::Waypoint(home) => {
                check!(command.regime == FlightRegime::Copter);
                check!(home.position_m == Vec2::zeros());
                check!(home.altitude_m == 30.0);
                Home::Hover
            }
            ControlInput::Land(land) => {
                check!(land.position_m == Vec2::zeros());
                Home::Land
            }
            input => panic!("{input:?}"),
        };
        check!(phase == expected, "{north_m} m, {altitude_m} m");
        check!(switch.is_landing() == (phase == Home::Land));
    }
}

/// The hover back lands once over the start, wherever the back transition ended: 25 m out it
/// hovers on, then lands on the start from 1 m.
#[test]
fn the_hover_back_lands_once_over_the_start() {
    let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
    switch.hold(HOLD());
    commander.next_command(0.004, &cruising(0.0, 100.0));
    switch.finish();
    commander.next_command(0.004, &cruising(25.0, 30.0));
    let hover = commander.next_command(0.004, &cruising(25.0, 20.0));
    check!(
        hover.regime == FlightRegime::Copter,
        "out of the plane altitude band"
    );
    check!(!switch.is_landing());
    let ControlInput::Land(land) = commander.next_command(0.004, &cruising(1.0, 20.0)).input else {
        panic!("a land over the start");
    };
    check!(land.position_m == Vec2::zeros());
}

/// The hover back waits on the caller's home radius: 150 m out hovers within a 200 m radius
/// and flies the leg on within 100 m.
#[test]
fn the_flight_home_hovers_back_within_its_radius() {
    for (radius_m, hovering) in [(100.0, false), (200.0, true)] {
        let home = PlaneHome {
            radius_m,
            overfly_m: 300.0,
            ..HOME
        };
        let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), home);
        switch.hold(HOLD());
        commander.next_command(0.004, &cruising(0.0, 100.0));
        switch.finish();
        let command = commander.next_command(0.004, &cruising(150.0, 30.0));
        check!(
            (command.regime == FlightRegime::Copter) == hovering,
            "radius {radius_m} m"
        );
    }
}

/// The hover back waits on the script's plane altitude acceptance: 20 m off the home altitude
/// hovers within a 25 m acceptance and flies the leg on within the default 10 m.
#[test]
fn the_flight_home_hovers_back_within_the_scripts_altitude_acceptance() {
    for (acceptance_m, hovering) in [(None, false), (Some(25.0), true)] {
        let mut script = plane_script();
        if let Some(acceptance_m) = acceptance_m {
            script = script.with_altitude_acceptance(FlightRegime::Plane, acceptance_m);
        }
        let (mut commander, switch) = PlaneTuneFlight::new(script, HOME);
        switch.hold(HOLD());
        commander.next_command(0.004, &cruising(0.0, 100.0));
        switch.finish();
        let command = commander.next_command(0.004, &cruising(0.0, 50.0));
        check!(
            (command.regime == FlightRegime::Copter) == hovering,
            "acceptance {acceptance_m:?} m"
        );
    }
}

/// The home leg flies the line through the start back and forth: north of the start it heads
/// south, keeps south past the start up to the overfly, then turns back north; over the start
/// and still, it heads north: (north positions flown, leg direction after the last).
#[test]
fn the_home_leg_turns_back_past_the_start() {
    for (norths_m, direction) in [
        (vec![400.0], Vec2::new(-1.0, 0.0)),
        (vec![400.0, -100.0], Vec2::new(-1.0, 0.0)),
        (vec![400.0, -160.0], Vec2::new(1.0, 0.0)),
        (vec![400.0, -160.0, -100.0], Vec2::new(1.0, 0.0)),
        (vec![400.0, -160.0, 160.0], Vec2::new(-1.0, 0.0)),
        (vec![0.0], Vec2::new(1.0, 0.0)),
    ] {
        let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
        switch.hold(HOLD());
        commander.next_command(0.004, &cruising(0.0, 100.0));
        switch.finish();
        let mut leg = None;
        for north_m in &norths_m {
            if let ControlInput::Waypoint(home) = commander
                .next_command(0.004, &cruising(*north_m, 100.0))
                .input
            {
                leg = home.velocity_mps;
            }
        }
        check!(leg == Some(direction * 14.0), "{norths_m:?}");
    }
}

/// Finished after a recovery, or before any hold, the flight lands where the vehicle is, and
/// disarms on touchdown; a second finish changes nothing.
#[test]
fn the_plane_flight_lands_in_place_after_a_recovery_or_without_a_hold() {
    for recovered in [true, false] {
        let (mut commander, switch) = PlaneTuneFlight::new(plane_script(), HOME);
        if recovered {
            switch.hold(HOLD());
            switch.recover();
        }
        let state = cruising(400.0, 60.0);
        commander.next_command(0.004, &state);
        switch.finish();
        check!(switch.is_landing(), "recovered {recovered}");
        let ControlInput::Land(land) = commander.next_command(0.004, &state).input else {
            unreachable!("a land");
        };
        check!(land.position_m == Vec2::new(400.0, 0.0));
        switch.finish();
        check!(switch.is_landing());
        let sinking = VehicleState {
            velocity_mps: Vec3::new(0.0, 0.0, -0.5),
            ..state.clone()
        };
        check!(commander.next_command(0.004, &sinking).arm);
        let disarmed = (0..500).any(|_| !commander.next_command(0.004, &state).arm);
        check!(disarmed, "still armed 2 s after stopping");
    }
}

/// Inverted is a tilt past 90° in the probed frame: (frame, roll and pitch in rad of that
/// frame, inverted). Nose-up hover reads level in a tailsitter's view, though its body frame
/// sits at 90°, where a little pitch either way would flip a body-frame check.
#[test]
fn inverted_is_a_tilt_past_ninety_degrees_in_the_frame() {
    let body = UnitQuaternion::identity();
    let view = tailsitter_rotation();
    for (name, frame, roll_rad, pitch_rad, inverted) in [
        ("body", body, 0.0, 0.0, false),
        ("body", body, 1.5, 0.0, false),
        ("body", body, 1.7, 0.0, true),
        ("body", body, 3.1, 0.0, true),
        ("view", view, 0.0, 0.0, false),
        ("view", view, 0.0, 0.05, false),
        ("view", view, 0.0, -0.05, false),
        ("view", view, 1.5, 0.0, false),
        ("view", view, 1.7, 0.0, true),
        ("view", view, 3.1, 0.0, true),
    ] {
        // The attitude whose `frame` is rolled and pitched by these angles.
        let state = VehicleState {
            attitude: UnitQuaternion::from_euler_angles(roll_rad, pitch_rad, 0.0) * frame.inverse(),
            ..Default::default()
        };
        check!(
            is_inverted(&state, &frame) == inverted,
            "{name} roll {roll_rad} pitch {pitch_rad}"
        );
    }
}

/// A step and a summary read in ms and fractions of the step, `-` where there's no sample.
#[test]
fn formats_steps_and_summaries() {
    let metrics = StepMetrics {
        amplitude: -0.5,
        overshoot: 0.04,
        rise_time_s: Some(0.08),
        settling_time_s: None,
        bounce_back: 0.01,
        final_value: 0.98,
        saturated_s: 0.012,
        ..Default::default()
    };
    check!(
        format_step(&metrics)
            == "-0.500: overshoot 0.04, rise 80 ms, settle -, bounce 0.01, final 0.98, clipped 12 ms"
    );
    let mut summary = StepSummary::default();
    summary.add(&metrics);
    check!(
        format_summary(&summary)
            == "n 1: overshoot 0.04 [0.04-0.04] (+ -, - 0.04), rise 80.00 [80.00-80.00] ms, \
                settle - ms, bounce 0.01 [0.01-0.01], final 0.98 [0.98-0.98]; not risen 0, \
                not settled 1"
    );
}

/// The snippet carries every tuned loop's gains, the yaw integral limit, and its source.
#[test]
fn the_snippet_carries_every_tuned_gain() {
    let mut config = CopterStabilizerConfig::default();
    TunedLoop::RollRate.gains_mut(&mut config).p = 0.1234;
    TunedLoop::Pitch.gains_mut(&mut config).p = 6.5;
    let snippet = config.airframe_snippet("copter-autotune-test");
    check!(snippet.starts_with("// copter-autotune-test\n"));
    check!(snippet.contains("roll_gains: Gains { p: 0.1234,"));
    check!(snippet.contains("pitch_gains: Gains { p: 6.5000,"));
    check!(snippet.contains(&format!(
        "yaw_max_integral: {:.1}",
        config.body_rate.yaw_max_integral
    )));
}

/// The plane snippet carries roll and pitch rate gains and both time constants, and its
/// source.
#[test]
fn the_plane_snippet_carries_every_tuned_gain() {
    let mut config = PlaneStabilizerConfig::default();
    config.rate.roll.p = 0.1234;
    config.rate.pitch.ff = 0.0567;
    config.time_constant_s = Vec2::new(0.35, 0.5);
    let snippet = config.airframe_snippet("plane-autotune-test");
    check!(snippet.starts_with("// plane-autotune-test\n"));
    check!(snippet.contains("roll: RateGains { p: 0.1234,"));
    check!(snippet.contains("ff: 0.0567 }"));
    check!(snippet.contains("time_constant_s: Vec2::new(0.350, 0.500),"));
    check!(
        PlaneStabilizerConfig::format_gains(&PlaneLoop::Pitch.gains(&config))
            .ends_with("ff: 0.0567 }, time constant 0.500 s")
    );
}

/// Every plane event logs without panicking, the wing fit and the snippet included.
#[test]
fn logs_every_plane_event() {
    let config = PlaneStabilizerConfig::default();
    let target = PlaneLoop::Pitch;
    let gains = target.gains(&config);
    let model = WingModel {
        stiffness_per_s2: 40.0,
        damping_per_s: 8.0,
        control_radps2: 120.0,
        lag_s: 0.02,
        delay_s: 0.012,
        path_turn_per_s: 1.2,
        trim_moment: 0.09,
    };
    let events = [
        AutotuneEvent::StageStarted(target),
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Wing(model),
        },
        AutotuneEvent::Tried {
            target,
            gains,
            passed: false,
            steps: StepSummary::default(),
        },
        AutotuneEvent::StageDone {
            target,
            gains,
            outcome: Outcome::Designed,
            steps: None,
            baseline: None,
        },
        AutotuneEvent::Complete,
    ];
    for event in &events {
        log_event(event, &config, "plane-autotune-test");
    }
}

/// Every event logs without panicking, including the completion's snippet.
#[test]
fn logs_every_event() {
    let config = CopterStabilizerConfig::default();
    let gains = TunedLoop::RollRate.gains(&config);
    let steps = StepSummary::default();
    let model = RateModel {
        rate_weights: [1.0, 0.0],
        moment_weights: [0.1, 0.0, 0.0],
        delay_ticks: 3,
        dt_s: 0.004,
    };
    let target = TunedLoop::RollRate;
    let events = [
        AutotuneEvent::StageStarted(target),
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Step(StepMetrics::default()),
        },
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Identify(model),
        },
        AutotuneEvent::Stalled {
            target,
            measurement: Some(Measurement::Step(StepMetrics::default())),
        },
        AutotuneEvent::Stalled {
            target,
            measurement: None,
        },
        AutotuneEvent::Tried {
            target,
            gains,
            passed: true,
            steps,
        },
        AutotuneEvent::StageDone {
            target,
            gains,
            outcome: Outcome::Tuned,
            steps: Some(steps),
            baseline: Some(steps),
        },
        AutotuneEvent::Lowered { target, gains },
        AutotuneEvent::Unsettled { target, gains },
        AutotuneEvent::StageFailed {
            target,
            reason: AutotuneAbort::NotSettled,
        },
        AutotuneEvent::Complete,
        AutotuneEvent::Reverted,
    ];
    for event in &events {
        log_event(event, &config, "copter-autotune-test");
    }
}
