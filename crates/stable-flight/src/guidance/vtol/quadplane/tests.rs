use approx::assert_relative_eq;
use assert2::check;
use nalgebra::UnitQuaternion;

use super::*;
use crate::{AttitudeCommand, LandCommand, Vec2, Vec3, WaypointCommand};

const DT: f32 = 0.01;
const HIGH_M: f32 = 20.0;
/// Past the default airspeed gate.
const FAST_MPS: f32 = 12.0;
const HEADING_RAD: f32 = 0.3;

/// Level, heading [`HEADING_RAD`], `height_m` above the arming point, flying along the nose at
/// `speed_mps` (ground and air).
fn at(height_m: f32, speed_mps: f32) -> VehicleState {
    banked(height_m, speed_mps, 0.0)
}

/// As [`at`], banked `roll_deg`.
fn banked(height_m: f32, speed_mps: f32, roll_deg: f32) -> VehicleState {
    let (sin, cos) = libm::sincosf(HEADING_RAD);
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, height_m),
        velocity_mps: Vec3::new(cos, sin, 0.0) * speed_mps,
        attitude: UnitQuaternion::from_euler_angles(roll_deg.to_radians(), 0.0, HEADING_RAD),
        heading_rad: HEADING_RAD,
        airspeed_mps: Some(speed_mps),
        ..Default::default()
    }
}

fn hovering() -> VehicleState {
    at(HIGH_M, 0.0)
}

fn fast() -> VehicleState {
    at(HIGH_M, FAST_MPS)
}

/// As [`at`] at [`FAST_MPS`], slowing along its track at the default `back_decel_mps2`, so
/// the back transition keeps the nose level.
fn slowing(height_m: f32) -> VehicleState {
    let state = at(height_m, FAST_MPS);
    let decel_mps2 = LiftTransitionConfig::default().back_decel_mps2;
    VehicleState {
        acceleration_mps2: -state.velocity_mps.normalize() * decel_mps2,
        ..state
    }
}

fn sticks(throttle: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle,
        ..Default::default()
    })
}

/// Primed on the ground below `HIGH_M`.
fn primed() -> QuadPlaneGuidance {
    let mut guidance = QuadPlaneGuidance::new(QuadPlaneGuidanceConfig::default());
    guidance.prime(&at(0.0, 0.0));
    guidance
}

fn mode(guidance: &QuadPlaneGuidance) -> FlightMode {
    guidance
        .flight_mode()
        .map_or(FlightMode::Copter, |report| report.mode)
}

/// Flies `guidance` for `seconds` on `regime` and `input` in `state`, returning the last
/// setpoint.
fn fly(
    guidance: &mut QuadPlaneGuidance,
    seconds: f32,
    regime: FlightRegime,
    input: &ControlInput,
    state: &VehicleState,
) -> Setpoint {
    let ticks = libm::roundf(seconds / DT) as usize;
    (0..ticks)
        .map(|_| guidance.guide(DT, regime, input, state))
        .last()
        .expect("at least one tick")
}

/// `setpoint`'s blend, failing the check for any other.
fn blend_of(setpoint: Setpoint) -> BlendSetpoint {
    check!(matches!(setpoint, Setpoint::Blend(_)), "{setpoint:?}");
    match setpoint {
        Setpoint::Blend(blend) => blend,
        _ => BlendSetpoint::default(),
    }
}

/// Driven through the forward transition on `input`, up to its handoff into `Plane`.
fn in_plane_with(input: &ControlInput) -> QuadPlaneGuidance {
    let mut guidance = primed();
    let entered = (0..1000).any(|_| {
        guidance.guide(DT, FlightRegime::Plane, input, &fast());
        mode(&guidance) == FlightMode::Plane
    });
    assert!(entered);
    guidance
}

/// `Copter` flies the hover guidance on the command's input; `Plane` the plane guidance, once
/// the handoff blend is done.
#[test]
fn routes_each_settled_mode_to_its_guidance() {
    let config = QuadPlaneGuidanceConfig::default();
    let input = sticks(0.9);
    let mut hover = CopterGuidance::new(config.hover);
    hover.prime(&at(0.0, 0.0));
    let ours = primed().guide(DT, FlightRegime::Copter, &input, &hovering());
    check!(ours == hover.guide(DT, FlightRegime::Copter, &input, &hovering()));

    let mut guidance = in_plane_with(&input);
    let ours = fly(
        &mut guidance,
        config.transition.handoff_s + 0.05,
        FlightRegime::Plane,
        &input,
        &fast(),
    );
    let alone = PlaneGuidance::new(config.plane).guide(DT, FlightRegime::Plane, &input, &fast());
    check!(ours == alone);
}

/// The forward transition flies a blend: wings level, heading held, the hover side's altitude
/// loop holding the height the transition began at; the pusher on the plane guidance's thrust,
/// rising at most `pusher_slew_per_s` and up to `forward_thrust`; the transition's lift share.
#[test]
fn forward_transition_flies_the_pusher_on_the_plane_thrust() {
    let config = QuadPlaneGuidanceConfig::default().transition;
    let mut guidance = primed();
    let input = sticks(0.6);
    let first = blend_of(guidance.guide(DT, FlightRegime::Plane, &input, &hovering()));
    check!(mode(&guidance) == FlightMode::ToPlane);
    assert_relative_eq!(first.plane.thrust, config.pusher_slew_per_s * DT);
    check!(first.lift_share == 1.0);

    let blend = blend_of(fly(
        &mut guidance,
        1.0,
        FlightRegime::Plane,
        &input,
        &hovering(),
    ));
    check!((blend.hover.roll_rad, blend.plane.roll_rad) == (0.0, 0.0));
    assert_relative_eq!(blend.hover.heading_rad, HEADING_RAD);
    assert_relative_eq!(
        blend.plane.thrust,
        config.pusher_slew_per_s * 1.01,
        epsilon = 1e-4
    );
    // Below the held height the altitude loop asks for more; the transition began at HIGH_M.
    let low = blend_of(guidance.guide(DT, FlightRegime::Plane, &input, &at(HIGH_M - 3.0, 0.0)));
    check!(low.hover.thrust > blend.hover.thrust);

    // The stick's throttle, once reached.
    let blend = blend_of(fly(
        &mut guidance,
        5.0,
        FlightRegime::Plane,
        &input,
        &hovering(),
    ));
    check!(blend.plane.thrust == 0.6);

    // Past the airspeed gate the lift share ramps down.
    let blend = blend_of(fly(
        &mut guidance,
        1.0,
        FlightRegime::Plane,
        &input,
        &fast(),
    ));
    assert_relative_eq!(
        blend.lift_share,
        1.0 - 1.0 / config.lift_ramp_s,
        epsilon = 0.01
    );

    // At most `forward_thrust`.
    let mut capped = QuadPlaneGuidanceConfig::default();
    capped.transition.forward_thrust = 0.4;
    let mut guidance = QuadPlaneGuidance::new(capped);
    guidance.prime(&at(0.0, 0.0));
    let blend = blend_of(fly(
        &mut guidance,
        5.0,
        FlightRegime::Plane,
        &sticks(1.0),
        &hovering(),
    ));
    check!(blend.plane.thrust == 0.4);
}

/// Both sides fly the plane guidance's pitch within ArduPilot's transition limits: 0 below
/// 3 m/s of ground speed, ±3° waiting on airspeed, ±8° on the lift ramp; 0 for rate input.
#[test]
fn forward_pitch_follows_the_plane_within_the_transition_limits() {
    let pitch_stick = |pitch: f32| {
        ControlInput::Attitude(AttitudeCommand {
            throttle: 0.6,
            pitch,
            ..Default::default()
        })
    };
    let rates = ControlInput::Rate(crate::RateCommand {
        throttle: 0.6,
        pitch_rate: 1.0,
        ..Default::default()
    });
    // (case, input, state, ramping, expected pitch in degrees)
    let cases = [
        ("still", pitch_stick(1.0), hovering(), false, 0.0),
        ("moving, up", pitch_stick(1.0), at(HIGH_M, 5.0), false, 3.0),
        (
            "moving, down",
            pitch_stick(-1.0),
            at(HIGH_M, 5.0),
            false,
            -3.0,
        ),
        ("ramping", pitch_stick(1.0), fast(), true, 8.0),
        ("rates", rates, fast(), true, 0.0),
    ];
    for (case, input, state, ramping, pitch_deg) in cases {
        let mut guidance = primed();
        guidance.guide(DT, FlightRegime::Plane, &input, &hovering());
        if ramping {
            guidance.guide(DT, FlightRegime::Plane, &input, &fast());
        }
        let blend = blend_of(guidance.guide(DT, FlightRegime::Plane, &input, &state));
        check!(mode(&guidance) == FlightMode::ToPlane, "{case}");
        check!((blend.lift_share < 1.0) == ramping, "{case}");
        assert_relative_eq!(
            blend.plane.pitch_rad.to_degrees(),
            pitch_deg,
            epsilon = 1e-4
        );
        check!(blend.hover.pitch_rad == blend.plane.pitch_rad, "{case}");
    }
}

/// On a waypoint, the forward transition flies the wing toward the altitude it holds, not the
/// waypoint's: a waypoint far below pitches and thrusts as a level one, waiting and on the
/// ramp (#491).
#[test]
fn forward_transition_flies_the_wing_level_toward_a_lower_waypoint() {
    let toward = |altitude_m: f32| {
        ControlInput::Waypoint(WaypointCommand {
            position_m: Vec2::new(1000.0, 0.0),
            velocity_mps: None,
            altitude_m,
            cruise_speed_mps: FAST_MPS,
            heading_rad: None,
        })
    };
    // (case, state the transition flies in, ramping)
    let cases = [
        ("waiting", at(HIGH_M, 5.0), false),
        ("ramping", fast(), true),
    ];
    for (case, state, ramping) in cases {
        let [level, below] = [HIGH_M, HIGH_M - 200.0].map(|altitude_m| {
            let input = toward(altitude_m);
            let mut guidance = primed();
            guidance.guide(DT, FlightRegime::Plane, &input, &hovering());
            let blend = blend_of(fly(&mut guidance, 1.0, FlightRegime::Plane, &input, &state));
            check!(mode(&guidance) == FlightMode::ToPlane, "{case}");
            check!((blend.lift_share < 1.0) == ramping, "{case}");
            blend.plane
        });
        check!(below.pitch_rad == level.pitch_rad, "{case}");
        check!(below.thrust == level.thrust, "{case}");
    }
}

/// The back transition flies a blend with the pusher stopped, both sides level while it slows
/// at `back_decel_mps2`, the hover side holding the height it began at as the lift share ramps
/// up; a `Land` input asks for it.
#[test]
fn back_transition_cuts_the_pusher_and_takes_the_lift_back() {
    let config = QuadPlaneGuidanceConfig::default().transition;
    let land = ControlInput::Land(LandCommand::default());
    let mut guidance = in_plane_with(&sticks(0.6));
    let first = blend_of(guidance.guide(DT, FlightRegime::Plane, &land, &slowing(HIGH_M)));
    check!(mode(&guidance) == FlightMode::ToCopter);
    check!(first.plane == PlaneSetpoint::default());

    let blend = blend_of(fly(
        &mut guidance,
        1.0,
        FlightRegime::Plane,
        &land,
        &slowing(HIGH_M),
    ));
    assert_relative_eq!(blend.lift_share, 1.0 / config.back_ramp_s, epsilon = 1e-3);
    check!((blend.hover.roll_rad, blend.hover.pitch_rad) == (0.0, 0.0));
    let low = blend_of(guidance.guide(DT, FlightRegime::Plane, &land, &slowing(HIGH_M - 3.0)));
    check!(low.hover.thrust > blend.hover.thrust);

    // Slowed below the back speed: copter.
    let slow = at(HIGH_M, config.back_speed_mps - 1.0);
    let setpoint = guidance.guide(DT, FlightRegime::Plane, &land, &slow);
    check!(mode(&guidance) == FlightMode::Copter);
    check!(matches!(setpoint, Setpoint::Attitude(_)));
}

/// A back transition that slows less than `back_decel_mps2` noses up on both sides by the
/// integral of the shortfall, up to 0.3 rad (#496), and the next one starts level.
#[test]
fn back_transition_noses_up_while_it_glides() {
    let config = QuadPlaneGuidanceConfig::default().transition;
    let copter = sticks(0.6);
    let mut guidance = in_plane_with(&copter);
    // Not slowing at all: 2 m/s² short, 0.1 rad/s per m/s².
    for (seconds, pitch_rad) in [(0.5, 0.1), (0.5, 0.2), (2.0, 0.3)] {
        let blend = blend_of(fly(
            &mut guidance,
            seconds,
            FlightRegime::Copter,
            &copter,
            &fast(),
        ));
        check!(mode(&guidance) == FlightMode::ToCopter);
        assert_relative_eq!(blend.plane.pitch_rad, pitch_rad, epsilon = 2e-3);
        check!(blend.hover.pitch_rad == blend.plane.pitch_rad);
        check!(blend.hover.roll_rad == 0.0);
    }

    guidance.guide(DT, FlightRegime::Copter, &copter, &at(HIGH_M, 0.0));
    check!(mode(&guidance) == FlightMode::Copter);
    let entered = (0..1000).any(|_| {
        guidance.guide(DT, FlightRegime::Plane, &copter, &fast());
        mode(&guidance) == FlightMode::Plane
    });
    check!(entered);
    let first = blend_of(guidance.guide(DT, FlightRegime::Copter, &copter, &fast()));
    let one_tick_rad = config.back_decel_i * config.back_decel_mps2 * DT;
    assert_relative_eq!(first.plane.pitch_rad, one_tick_rad, epsilon = 1e-6);
}

/// The back transition primes the hover guidance: it holds the heading the plane flew in on,
/// not the one it left the hover with.
#[test]
fn back_transition_holds_the_heading_it_arrives_on() {
    let mut guidance = in_plane_with(&sticks(0.6));
    let turned = VehicleState {
        heading_rad: 1.2,
        attitude: UnitQuaternion::from_euler_angles(0.0, 0.0, 1.2),
        ..fast()
    };
    let blend = blend_of(guidance.guide(DT, FlightRegime::Copter, &sticks(0.6), &turned));
    assert_relative_eq!(blend.hover.heading_rad, 1.2);
}

/// Taking `Plane` back from the back transition, the pusher's thrust blends from its stop to
/// the plane guidance's over `handoff_s`, then follows it.
#[test]
fn plane_handoff_blends_the_pusher_thrust() {
    let handoff_s = QuadPlaneGuidanceConfig::default().transition.handoff_s;
    let offboard = ControlInput::AbsoluteAttitude(crate::AbsoluteAttitudeCommand {
        thrust: 0.4,
        ..Default::default()
    });
    let mut guidance = in_plane_with(&offboard);
    let back = blend_of(guidance.guide(DT, FlightRegime::Copter, &offboard, &fast()));
    check!(back.plane.thrust == 0.0);
    let mid = fly(
        &mut guidance,
        handoff_s / 2.0,
        FlightRegime::Plane,
        &offboard,
        &fast(),
    );
    check!(mode(&guidance) == FlightMode::Plane);
    assert_relative_eq!(mid.thrust(), 0.2, epsilon = 0.01);
    let done = fly(
        &mut guidance,
        handoff_s,
        FlightRegime::Plane,
        &offboard,
        &fast(),
    );
    check!(done.thrust() == 0.4);
}

/// A bank past the limit cuts a transition short: forward aborts to copter and reports the
/// roll error; back completes to copter. Within the limit, or outside a transition, nothing.
#[test]
fn bank_past_the_limit_cuts_transitions_short() {
    let limit_deg = QuadPlaneGuidanceConfig::default()
        .transition
        .roll_limit_rad
        .to_degrees();
    let input = sticks(0.6);
    let over = banked(HIGH_M, FAST_MPS, limit_deg + 5.0);
    let within = banked(HIGH_M, FAST_MPS, limit_deg - 5.0);

    let mut guidance = primed();
    guidance.guide(DT, FlightRegime::Plane, &input, &hovering());
    guidance.guide(DT, FlightRegime::Plane, &input, &within);
    check!(mode(&guidance) == FlightMode::ToPlane);
    guidance.guide(DT, FlightRegime::Plane, &input, &over);
    check!(guidance.flight_mode().and_then(|r| r.refusal) == Some(ModeRefusal::RollError));
    check!(mode(&guidance) == FlightMode::Copter);

    let mut guidance = in_plane_with(&input);
    guidance.guide(DT, FlightRegime::Plane, &input, &over);
    check!(mode(&guidance) == FlightMode::Plane);
    guidance.guide(DT, FlightRegime::Copter, &input, &within);
    check!(mode(&guidance) == FlightMode::ToCopter);
    guidance.guide(DT, FlightRegime::Copter, &input, &over);
    check!(mode(&guidance) == FlightMode::Copter);
}

/// Saturation comes from the plane side in `Plane`, the hover side otherwise.
#[test]
fn saturation_from_the_side_that_flew() {
    let mut guidance = primed();
    let full = sticks(1.0);
    // A forward transition far below its held height: the altitude loop clips at full thrust.
    guidance.guide(DT, FlightRegime::Plane, &full, &hovering());
    guidance.guide(DT, FlightRegime::Plane, &full, &at(HIGH_M - 50.0, 0.0));
    check!(guidance.saturation().thrust);

    let mut guidance = in_plane_with(&full);
    guidance.guide(DT, FlightRegime::Plane, &full, &fast());
    check!(guidance.saturation() == Saturation::default());
}

/// `prime` returns to copter, its pusher stopped.
#[test]
fn prime_returns_to_copter() {
    let mut guidance = in_plane_with(&sticks(0.6));
    guidance.prime(&at(0.0, 0.0));
    check!(mode(&guidance) == FlightMode::Copter);
    let setpoint = guidance.guide(DT, FlightRegime::Plane, &sticks(0.6), &at(HIGH_M, 0.0));
    check!(blend_of(setpoint).plane.thrust < 0.01);
}

/// Forward thrust at `gain`, the default pitch limit.
fn with_forward(gain: f32) -> QuadPlaneGuidance {
    let config = QuadPlaneGuidanceConfig {
        forward: ForwardThrustConfig {
            gain,
            ..ForwardThrustConfig::default()
        },
        ..QuadPlaneGuidanceConfig::default()
    };
    let mut guidance = QuadPlaneGuidance::new(config);
    guidance.prime(&at(0.0, 0.0));
    guidance
}

/// A waypoint 100 m ahead along the heading, at [`HIGH_M`]: the hover pitches nose down.
fn far_ahead() -> ControlInput {
    let (sin, cos) = libm::sincosf(HEADING_RAD);
    ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(cos, sin) * 100.0,
        altitude_m: HIGH_M,
        ..Default::default()
    })
}

/// In `Copter` with forward thrust on, a waypoint or land flies a blend at full lift share:
/// the hover's attitude with its nose-down pitch held to the limit, the plane side at that roll
/// and pitch, the pusher on the rest. Sticks, and forward thrust off, fly the hover alone.
#[test]
fn copter_flight_hands_nose_down_pitch_to_the_pusher() {
    let land = ControlInput::Land(LandCommand {
        position_m: Vec2::new(100.0, 0.0),
    });
    let limit_rad = ForwardThrustConfig::default().pitch_limit_rad;
    // (case, gain, input, blended)
    let cases = [
        ("waypoint", 1.2, far_ahead(), true),
        ("land", 1.2, land, true),
        ("sticks", 1.2, sticks(0.6), false),
        ("off", 0.0, far_ahead(), false),
    ];
    for (case, gain, input, blended) in cases {
        let mut guidance = with_forward(gain);
        let setpoint = guidance.guide(DT, FlightRegime::Copter, &input, &hovering());
        let alone = with_forward(0.0).guide(DT, FlightRegime::Copter, &input, &hovering());
        check!(mode(&guidance) == FlightMode::Copter, "{case}");
        if !blended {
            check!(setpoint == alone, "{case}");
            continue;
        }
        let Setpoint::Attitude(hover) = alone else {
            panic!("{case}: {alone:?}");
        };
        check!(hover.pitch_rad < -limit_rad, "{case}: {hover:?}");
        let split = ForwardThrust::new(guidance.config().forward).split(hover.pitch_rad);
        let blend = blend_of(setpoint);
        check!(blend.lift_share == 1.0, "{case}");
        check!(
            blend.hover
                == AttitudeSetpoint {
                    pitch_rad: -limit_rad,
                    ..hover
                },
            "{case}"
        );
        check!(blend.plane.roll_rad == hover.roll_rad, "{case}");
        check!(blend.plane.pitch_rad == -limit_rad, "{case}");
        check!(blend.plane.thrust == split.thrust, "{case}");
        check!(blend.plane.thrust > 0.0, "{case}");
    }
}

/// Params chain hover, plane, transition, forward thrust; a write lands in its part and in `config()`, and an
/// invalid config changes nothing.
#[test]
fn params_and_config_chain_the_parts() {
    let mut guidance = primed();
    check!(guidance.param_name(0) == Some("GUID_ALT_P"));
    check!(guidance.set_param("TRAN_PSH_SLEW", 0.5).is_ok());
    check!(guidance.config().transition.pusher_slew_per_s == 0.5);
    check!(guidance.set_param("PGUID_CRUISE", 0.4).is_ok());
    check!(guidance.config().plane.cruise_thrust == 0.4);
    check!(guidance.param_at(guidance.param_count()).is_none());
    check!(guidance.set_param("TRAN_FW_THR", 2.0).is_err());
    check!(guidance.set_param("FWD_THR_GAIN", 1.2).is_ok());
    check!(guidance.config().forward.gain == 1.2);
    check!(guidance.param("FWD_PIT_LIM").is_some());
    check!(guidance.set_param("FWD_PIT_LIM", 90.0).is_err());

    let mut bad = *guidance.config();
    bad.transition.lift_ramp_s = 0.0;
    let error = guidance.set_config(bad).map_err(|e| e.section);
    check!(error == Err(Some("transition")));
    check!(guidance.config().transition.lift_ramp_s > 0.0);
    let mut bad = *guidance.config();
    bad.forward.gain = -1.0;
    check!(guidance.set_config(bad).map_err(|e| e.section) == Err(Some("forward")));
    let mut good = *guidance.config();
    good.transition.handoff_s = 1.0;
    check!(guidance.set_config(good).is_ok());
    check!(*guidance.config() == good);
}

#[test]
#[should_panic(expected = "invalid QuadPlaneGuidanceConfig")]
fn rejects_an_invalid_config() {
    let mut config = QuadPlaneGuidanceConfig::default();
    config.transition.forward_thrust = 2.0;
    QuadPlaneGuidance::new(config);
}

/// Entering `Plane` out of the forward transition, the plane guidance starts fresh: its first
/// setpoint's pitch is a newly primed plane guidance's on the same input and state, not what
/// its total energy control built flying the transition from no airspeed (#454: the X-Plane
/// Alia entered at full nose-down and dove 90 m).
#[test]
fn plane_guidance_starts_fresh_entering_plane() {
    let config = QuadPlaneGuidanceConfig::default();
    let climb = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(500.0, 0.0),
        velocity_mps: Some(Vec2::new(FAST_MPS, 0.0)),
        altitude_m: HIGH_M + 10.0,
        cruise_speed_mps: FAST_MPS + 3.0,
        heading_rad: None,
    });
    let mut guidance = primed();
    // Slow first: the plane guidance flies the transition with no airspeed.
    fly(&mut guidance, 2.0, FlightRegime::Plane, &climb, &hovering());
    check!(mode(&guidance) == FlightMode::ToPlane);
    let entry = (0..1000)
        .find_map(|_| {
            let setpoint = guidance.guide(DT, FlightRegime::Plane, &climb, &fast());
            (mode(&guidance) == FlightMode::Plane).then_some(setpoint)
        })
        .expect("enters Plane");
    let Setpoint::Plane(entry) = entry else {
        panic!("plane flight gives a plane setpoint: {entry:?}");
    };

    let mut fresh = PlaneGuidance::new(config.plane);
    fresh.prime(&fast());
    let Setpoint::Plane(expected) = fresh.guide(DT, FlightRegime::Plane, &climb, &fast()) else {
        panic!("waypoints give plane attitudes");
    };
    assert_relative_eq!(entry.pitch_rad, expected.pitch_rad, epsilon = 1e-6);
}
