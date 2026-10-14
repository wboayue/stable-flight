use core::f32::consts::FRAC_PI_2;

use approx::assert_relative_eq;
use assert2::check;
use nalgebra::UnitQuaternion;

use super::*;
use crate::{AttitudeCommand, LandCommand, PlaneSetpoint, Vec3};

const DT: f32 = 0.01;
const HIGH_M: f32 = 20.0;

/// The airframe with its nose `tilt_deg` from vertical (0: hovering nose up), `roll_deg` about
/// the nose, `HIGH_M` above the arming point.
fn at(tilt_deg: f32, roll_deg: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, HIGH_M),
        attitude: UnitQuaternion::from_euler_angles(
            roll_deg.to_radians(),
            (90.0 - tilt_deg).to_radians(),
            0.3,
        ),
        ..Default::default()
    }
}

fn sticks(throttle: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle,
        ..Default::default()
    })
}

/// Primed on the ground below `HIGH_M`, so height above arming is `HIGH_M`.
fn primed() -> TailsitterGuidance {
    let mut guidance = TailsitterGuidance::new(TailsitterGuidanceConfig::default());
    guidance.prime(&VehicleState {
        position_m: Vec3::zeros(),
        ..at(0.0, 0.0)
    });
    guidance
}

fn mode(guidance: &TailsitterGuidance) -> FlightMode {
    guidance
        .flight_mode()
        .map(|r| r.mode)
        .unwrap_or(FlightMode::Copter)
}

/// Driven through the forward transition into `Plane`.
fn in_plane() -> TailsitterGuidance {
    let mut guidance = primed();
    guidance.guide(DT, FlightRegime::Plane, &sticks(0.6), &at(0.0, 0.0));
    guidance.guide(DT, FlightRegime::Plane, &sticks(0.6), &at(50.0, 0.0));
    assert!(mode(&guidance) == FlightMode::Plane);
    guidance
}

/// Copter flies the hover guidance on the command's input; plane flies the plane guidance.
#[test]
fn routes_each_settled_mode_to_its_guidance() {
    let mut copter = primed();
    let setpoint = copter.guide(DT, FlightRegime::Copter, &sticks(0.6), &at(0.0, 0.0));
    check!(matches!(setpoint, Setpoint::Attitude(a) if !a.transition));
    check!(mode(&copter) == FlightMode::Copter);

    let mut plane = in_plane();
    for _ in 0..100 {
        // Past the handoff blend.
        plane.guide(DT, FlightRegime::Plane, &sticks(0.7), &at(85.0, 0.0));
    }
    let setpoint = plane.guide(DT, FlightRegime::Plane, &sticks(0.7), &at(85.0, 0.0));
    check!(
        setpoint
            == Setpoint::Plane(PlaneSetpoint {
                roll_rad: 0.0,
                pitch_rad: 0.0,
                thrust: 0.7,
                transition: false,
            })
    );
}

/// The forward ramp flies the hover side wings level, nose down by the tilt target in the
/// view, at the larger of the configured thrust and the hover thrust when it began, marked a
/// transition so the hover tilt limit doesn't stop it (#388).
#[test]
fn forward_ramp_tilts_the_hover_view() {
    let mut guidance = primed();
    // Hovering at 0.8 thrust, above the default forward thrust (0.5).
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.8), &at(0.0, 0.0));
    let first = guidance.guide(DT, FlightRegime::Plane, &sticks(0.0), &at(0.0, 0.0));
    let later = guidance.guide(0.2, FlightRegime::Plane, &sticks(0.0), &at(5.0, 0.0));
    let (Setpoint::Attitude(first), Setpoint::Attitude(later)) = (first, later) else {
        panic!("the hover side flies the forward ramp");
    };
    check!(mode(&guidance) == FlightMode::ToPlane);
    check!(first.roll_rad == 0.0 && later.roll_rad == 0.0);
    check!(first.transition && later.transition);
    // 50°/s for 0.2 s past the 0° start: 10° of tilt, view pitch −10°.
    assert_relative_eq!(later.pitch_rad, -10f32.to_radians(), epsilon = 1e-3);
    check!(later.pitch_rad < first.pitch_rad);
    check!(first.thrust >= 0.8 - 1e-6 && later.thrust >= 0.8 - 1e-6);
}

/// The back ramp flies the plane side wings level, nose up to π/2 − tilt, at the back thrust,
/// marked a transition so the plane pitch cap doesn't stop it (#388); a `Land` input asks for
/// it too.
#[test]
fn back_ramp_pitches_the_plane_up_and_land_asks_for_it() {
    for input in [sticks(0.6), ControlInput::Land(LandCommand::default())] {
        let mut guidance = in_plane();
        let regime = if matches!(input, ControlInput::Land(_)) {
            FlightRegime::Plane // overridden by the land
        } else {
            FlightRegime::Copter
        };
        guidance.guide(DT, regime, &input, &at(80.0, 0.0));
        let setpoint = guidance.guide(0.2, regime, &input, &at(80.0, 0.0));
        check!(mode(&guidance) == FlightMode::ToCopter, "{input:?}");
        let Setpoint::Plane(plane) = setpoint else {
            panic!("the plane side flies the back ramp");
        };
        // From 80° at 50°/s for 0.2 s: 70° of tilt, pitch 20°.
        assert_relative_eq!(
            plane.pitch_rad,
            FRAC_PI_2 - 70f32.to_radians(),
            epsilon = 1e-3
        );
        check!(plane.roll_rad == 0.0);
        check!(plane.transition, "{input:?}");
    }
}

/// At a handoff the thrust blends from the outgoing side's last value to the incoming side's
/// demand over `handoff_s`, then follows the demand.
#[test]
fn handoff_blends_thrust() {
    let mut guidance = primed();
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.8), &at(0.0, 0.0));
    guidance.guide(DT, FlightRegime::Plane, &sticks(0.7), &at(0.0, 0.0));
    let ramp_thrust = guidance.last_thrust;
    // Completes: the plane side takes over at the sticks' 0.7.
    let first = guidance.guide(0.1, FlightRegime::Plane, &sticks(0.7), &at(50.0, 0.0));
    check!(mode(&guidance) == FlightMode::Plane);
    // A fifth of the 0.5 s blend.
    assert_relative_eq!(
        first.thrust(),
        ramp_thrust + (0.7 - ramp_thrust) * 0.2,
        epsilon = 1e-4
    );
    for _ in 0..5 {
        guidance.guide(0.1, FlightRegime::Plane, &sticks(0.7), &at(80.0, 0.0));
    }
    let settled = guidance.guide(0.1, FlightRegime::Plane, &sticks(0.7), &at(80.0, 0.0));
    assert_relative_eq!(settled.thrust(), 0.7, epsilon = 1e-6);
}

/// A roll past the limit cuts a transition short: forward aborts to copter and reports the
/// roll error until copter is requested; back completes to copter. Within the limit, nothing.
#[test]
fn roll_error_cuts_transitions_short() {
    // Forward: rolled 50° in the hover view (about the airplane's yaw axis, nose up).
    let rolled_view = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2 - 0.2, 0.3)
            * UnitQuaternion::from_euler_angles(0.0, 0.0, 50f32.to_radians()),
        ..at(0.0, 0.0)
    };
    let mut guidance = primed();
    guidance.guide(DT, FlightRegime::Plane, &sticks(0.6), &at(0.0, 0.0));
    guidance.guide(DT, FlightRegime::Plane, &sticks(0.6), &rolled_view);
    check!(
        guidance.flight_mode()
            == Some(FlightModeReport {
                mode: FlightMode::Copter,
                refusal: Some(ModeRefusal::RollError),
            })
    );

    // Back: banked 60°.
    let mut guidance = in_plane();
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.6), &at(80.0, 0.0));
    check!(mode(&guidance) == FlightMode::ToCopter);
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.6), &at(80.0, 60.0));
    check!(guidance.flight_mode() == Some(FlightModeReport::default()));

    // Within the limit (30° < 45°): the ramp goes on.
    let mut guidance = in_plane();
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.6), &at(80.0, 0.0));
    guidance.guide(DT, FlightRegime::Copter, &sticks(0.6), &at(80.0, 30.0));
    check!(mode(&guidance) == FlightMode::ToCopter);
}

/// `prime` returns to copter and clears a blend.
#[test]
fn prime_returns_to_copter() {
    let mut guidance = in_plane();
    guidance.prime(&at(0.0, 0.0));
    check!(mode(&guidance) == FlightMode::Copter);
    check!(!guidance.handoff.active());
    let setpoint = guidance.guide(DT, FlightRegime::Copter, &sticks(0.5), &at(0.0, 0.0));
    check!(matches!(setpoint, Setpoint::Attitude(_)));
}

/// Params chain hover, plane, transition; a write lands in its part and in `config()`, and
/// an invalid one changes nothing.
#[test]
fn params_chain_the_parts() {
    let mut guidance = primed();
    let names: Vec<_> = (0..guidance.param_count())
        .filter_map(|i| guidance.param_name(i))
        .collect();
    check!(names.first() == Some(&"GUID_ALT_P"));
    check!(names.contains(&"PGUID_BANK_MAX") && names.contains(&"TRAN_FW_THR"));
    check!(names.len() == guidance.param_count());

    check!(guidance.set_param("PGUID_BANK_MAX", 30.0).is_ok());
    check!(guidance.set_param("TRAN_FW_THR", 0.9).is_ok());
    assert_relative_eq!(
        guidance.config().plane.max_bank_rad,
        30f32.to_radians(),
        epsilon = 1e-6
    );
    check!(guidance.config().transition.forward_thrust == 0.9);
    check!(guidance.param("TRAN_FW_THR") == Some(0.9));

    let before = *guidance.config();
    check!(guidance.set_param("TRAN_FW_THR", 1.5).is_err());
    check!(guidance.set_param("NOPE", 1.0) == Err(ParamError::Unknown));
    check!(*guidance.config() == before);
}

/// After the handoff into plane, a stick left below cruise is held at cruise until the stick
/// reaches it; from then the stick flies. Offboard thrust isn't floored.
#[test]
fn plane_handoff_holds_cruise_until_the_stick_reaches_it() {
    let cruise = PlaneGuidanceConfig::default().cruise_thrust;
    let settle = |guidance: &mut TailsitterGuidance, input: ControlInput| {
        for _ in 0..10 {
            guidance.guide(0.1, FlightRegime::Plane, &input, &at(85.0, 0.0));
        }
        guidance
            .guide(0.1, FlightRegime::Plane, &input, &at(85.0, 0.0))
            .thrust()
    };
    let mut guidance = in_plane_with(sticks(0.1));
    assert_relative_eq!(settle(&mut guidance, sticks(0.1)), cruise, epsilon = 1e-6);
    assert_relative_eq!(settle(&mut guidance, sticks(0.8)), 0.8, epsilon = 1e-6);
    // Released: the stick flies, even below cruise.
    assert_relative_eq!(settle(&mut guidance, sticks(0.1)), 0.1, epsilon = 1e-6);

    let mut offboard = in_plane_with(sticks(0.1));
    let absolute = ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
        thrust: 0.2,
        ..Default::default()
    });
    assert_relative_eq!(settle(&mut offboard, absolute), 0.2, epsilon = 1e-6);
}

/// Driven into `Plane` with `input` on the handoff tick.
fn in_plane_with(input: ControlInput) -> TailsitterGuidance {
    let mut guidance = primed();
    guidance.guide(DT, FlightRegime::Plane, &input, &at(0.0, 0.0));
    guidance.guide(DT, FlightRegime::Plane, &input, &at(50.0, 0.0));
    assert!(mode(&guidance) == FlightMode::Plane);
    guidance
}
