use core::f32::consts::FRAC_PI_2;

use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::params::Params;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use crate::tuning::autotune::COPTER_RATE_RELAY;
use crate::{AttitudeCommand, AttitudeSetpoint, CruiseOutput, Vec3, euler_angles};

const DT: f32 = 0.004;

/// The tailsitter's mount in the view: the airplane body frame expressed in the view (+90°
/// pitch), RealFlight's `body_rotation` for the eXplora.
fn mount() -> UnitQuaternion<f32> {
    UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0)
}

/// A body-frame state whose view (tailsitter) attitude is `(roll, pitch, heading)`, with view
/// body rates `rates`.
fn from_view(roll: f32, pitch: f32, heading: f32, rates: Vec3) -> VehicleState {
    let view = VehicleState {
        attitude: UnitQuaternion::from_euler_angles(roll, pitch, heading),
        angular_velocity_radps: rates,
        ..Default::default()
    };
    view.rotate_body(&mount())
}

/// Nose-up hover sits at the body frame's Euler singularity, but reads level in the view with a
/// well-defined heading, banked or not.
#[test]
fn nose_up_hover_reads_level_in_the_view() {
    // (view roll, pitch, heading), degrees
    let cases: [(f32, f32, f32); 8] = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 90.0),
        (0.0, 0.0, 180.0),
        (0.0, 0.0, -90.0),
        (5.0, 0.0, 30.0),
        (-5.0, 0.0, 30.0),
        (0.0, 5.0, -120.0),
        (3.0, -4.0, 60.0),
    ];
    let view = View::tailsitter(());
    for (roll_deg, pitch_deg, heading_deg) in cases {
        let (roll, pitch, heading) = (
            roll_deg.to_radians(),
            pitch_deg.to_radians(),
            heading_deg.to_radians(),
        );
        let body = from_view(roll, pitch, heading, Vec3::zeros());
        check!(
            body.pitch_rad > 80.0_f32.to_radians(),
            "body pitch {}",
            body.pitch_rad.to_degrees()
        );

        let seen = view.view_state(&body);
        let tolerance = 1e-4;
        check!(
            (seen.roll_rad - roll).abs() < tolerance,
            "{roll_deg} {pitch_deg} {heading_deg}: {seen:?}"
        );
        check!(
            (seen.pitch_rad - pitch).abs() < tolerance,
            "{roll_deg} {pitch_deg} {heading_deg}: {seen:?}"
        );
        check!(
            crate::wrap_angle(seen.heading_rad - heading).abs() < tolerance,
            "{roll_deg} {pitch_deg} {heading_deg}: {seen:?}"
        );
    }
}

/// Body rates land on the view's axes: airplane yaw is view roll, pitch is pitch, airplane roll
/// is view −yaw.
#[test]
fn body_rates_land_on_the_view_axes() {
    // (body rates, view rates)
    let cases = [
        (Vec3::z(), Vec3::x()),
        (Vec3::y(), Vec3::y()),
        (Vec3::x(), -Vec3::z()),
        (Vec3::new(0.3, -0.2, 0.1), Vec3::new(0.1, -0.2, -0.3)),
    ];
    let view = View::tailsitter(());
    for (body, expected) in cases {
        let state = VehicleState {
            attitude: mount(),
            angular_velocity_radps: body,
            ..Default::default()
        };
        let seen = view.view_state(&state).angular_velocity_radps;
        check!((seen - expected).norm() < 1e-6, "{body:?} → {seen:?}");
    }
}

/// A stabilizer that answers every tick with fixed moments, the same on its lift rotors and
/// its cruise surfaces, and reports fixed per-axis saturation.
struct FixedMoments(Vec3, [bool; 3]);

impl Stabilizer for FixedMoments {
    fn stabilize(
        &mut self,
        _dt: f32,
        _setpoint: &Setpoint,
        _state: &VehicleState,
    ) -> ControlOutput {
        ControlOutput {
            cruise: Some(CruiseOutput {
                thrust: 0.3,
                moments: self.0,
            }),
            ..ControlOutput::new(0.5, self.0)
        }
    }

    fn saturation(&self) -> Saturation {
        Saturation {
            moments: self.1,
            mixer_thrust: true,
            ..Saturation::default()
        }
    }
}

/// View moments go back to the body's axes for the mixer, cruise moments too: view roll is
/// airplane yaw, view yaw is airplane −roll; thrusts pass through.
#[test]
fn moments_return_to_the_body_axes() {
    // (view moments, body moments)
    let cases = [
        (Vec3::x(), Vec3::z()),
        (Vec3::y(), Vec3::y()),
        (Vec3::z(), -Vec3::x()),
        (Vec3::new(0.1, -0.2, 0.3), Vec3::new(-0.3, -0.2, 0.1)),
    ];
    for (view_moments, body_moments) in cases {
        let mut stabilizer = View::tailsitter(FixedMoments(view_moments, [false; 3]));
        let output = stabilizer.stabilize(
            DT,
            &Setpoint::Attitude(AttitudeSetpoint::default()),
            &VehicleState::default(),
        );
        check!(output.thrust == 0.5);
        check!(
            (output.moments - body_moments).norm() < 1e-6,
            "{view_moments:?} → {:?}",
            output.moments
        );
        let cruise = output.cruise.expect("cruise passes through");
        check!(cruise.thrust == 0.3);
        check!(
            (cruise.moments - body_moments).norm() < 1e-6,
            "{view_moments:?} → {cruise:?}"
        );
    }
}

/// The viewed copter stabilizer on the airplane-frame state flies exactly as the copter
/// stabilizer on RealFlight's hover-level state, its moments turned into the airplane frame.
#[test]
fn matches_the_hover_level_wiring() {
    let config = CopterStabilizerConfig::default();
    let mut viewed = View::tailsitter(CopterStabilizer::new(config));
    let mut interim = CopterStabilizer::new(config);
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.1,
        pitch_rad: -0.05,
        heading_rad: 0.4,
        thrust: 0.6,
        transition: false,
        idle: false,
    });
    // RealFlight hover-level states (roll, pitch, heading, rates), a few ticks each.
    let states = [
        (0.0, 0.0, 0.0, Vec3::zeros()),
        (0.05, -0.1, 0.3, Vec3::new(0.2, -0.1, 0.05)),
        (-0.2, 0.15, 1.2, Vec3::new(-0.4, 0.3, -0.2)),
        (0.3, 0.3, -2.5, Vec3::new(1.0, -0.5, 0.8)),
    ];
    for (roll, pitch, heading, rates) in states {
        let hover_level = VehicleState {
            attitude: UnitQuaternion::from_euler_angles(roll, pitch, heading),
            angular_velocity_radps: rates,
            ..Default::default()
        }
        .rotate_body(&UnitQuaternion::identity());
        let body = hover_level.rotate_body(&mount());
        for _ in 0..5 {
            let expected = interim.stabilize(DT, &setpoint, &hover_level);
            let output = viewed.stabilize(DT, &setpoint, &body);
            let expected_moments = mount().inverse() * expected.moments;
            check!(output.thrust == expected.thrust);
            check!(
                (output.moments - expected_moments).norm() < 1e-4,
                "{:?} vs {expected_moments:?}",
                output.moments
            );
        }
    }
}

/// An identity view is the inner stabilizer, bit for bit.
#[test]
fn identity_view_is_the_inner_stabilizer() {
    let config = CopterStabilizerConfig::default();
    let mut viewed = View::new(CopterStabilizer::new(config), UnitQuaternion::identity());
    let mut plain = CopterStabilizer::new(config);
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.2,
        thrust: 0.5,
        ..Default::default()
    });
    let attitude = UnitQuaternion::from_euler_angles(-0.1, 0.05, 0.7);
    let (roll_rad, pitch_rad, heading_rad) = euler_angles(&attitude);
    let state = VehicleState {
        attitude,
        angular_velocity_radps: Vec3::new(0.1, 0.2, -0.3),
        roll_rad,
        pitch_rad,
        heading_rad,
        ..Default::default()
    };
    for _ in 0..10 {
        check!(viewed.stabilize(DT, &setpoint, &state) == plain.stabilize(DT, &setpoint, &state));
    }
}

/// Viewed copter guidance holds the view heading of a nose-up hover, which the body frame's
/// Euler angles can't give (roll and heading collapse at 90° pitch).
#[test]
fn viewed_guidance_holds_the_hover_heading() {
    let mut guidance = View::tailsitter(CopterGuidance::new(CopterGuidanceConfig::default()));
    let body = from_view(0.0, 0.0, 1.0, Vec3::zeros());
    guidance.prime(&body);

    let input = ControlInput::Attitude(AttitudeCommand {
        throttle: 0.5,
        ..Default::default()
    });
    let Setpoint::Attitude(setpoint) = guidance.guide(DT, FlightRegime::Copter, &input, &body)
    else {
        unreachable!("attitude commands yield attitude setpoints");
    };
    assert_relative_eq!(setpoint.heading_rad, 1.0, epsilon = 1e-4);
}

/// Params and config pass through under the inner component's names.
#[test]
fn params_pass_through_the_view() {
    let mut stabilizer = View::tailsitter(CopterStabilizer::new(CopterStabilizerConfig::default()));
    let plain = CopterStabilizer::new(CopterStabilizerConfig::default());
    check!(stabilizer.param_count() == plain.param_count());

    stabilizer
        .set_param("STAB_RATE_RLL_P", 0.33)
        .expect("valid param");
    check!(stabilizer.param("STAB_RATE_RLL_P") == Some(0.33));
    check!(stabilizer.inner().config().body_rate.roll_gains.p == 0.33);
}

/// Autotune probes pass through to the inner stabilizer.
#[test]
fn probes_pass_through_the_view() {
    let mut stabilizer = View::tailsitter(CopterStabilizer::new(CopterStabilizerConfig::default()));
    check!(stabilizer.probe().is_none());

    stabilizer.start_probe(TunedLoop::YawRate, ProbeConfig::Relay(COPTER_RATE_RELAY));
    check!(stabilizer.inner().probe().map(|(target, _)| target) == Some(TunedLoop::YawRate));
    check!(stabilizer.stop_probe().map(|(target, _)| target) == Some(TunedLoop::YawRate));
    check!(stabilizer.probe().is_none());
}

/// Per-axis saturation flags follow the moments into body axes (view roll is airplane yaw, view
/// yaw is airplane roll); an identity view leaves them alone, and other flags pass through.
#[test]
fn saturation_flags_follow_the_moments_into_body_axes() {
    let tailsitter = View::tailsitter(()).rotation;
    // (view, view flags, body flags)
    let cases = [
        (tailsitter, [true, false, false], [false, false, true]),
        (tailsitter, [false, true, false], [false, true, false]),
        (tailsitter, [false, false, true], [true, false, false]),
        (tailsitter, [true, false, true], [true, false, true]),
        (
            UnitQuaternion::identity(),
            [true, false, false],
            [true, false, false],
        ),
        (
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
            [true, false, false],
            [true, true, false],
        ),
    ];
    for (rotation, view_flags, body_flags) in cases {
        let stabilizer = View::new(FixedMoments(Vec3::zeros(), view_flags), rotation);
        let saturation = stabilizer.saturation();
        check!(
            saturation.moments == body_flags,
            "{view_flags:?} → {:?}",
            saturation.moments
        );
        check!(saturation.mixer_thrust);
    }
}

/// Records the regime it's asked for and reports a fixed flight mode.
struct RegimeGuidance {
    requested: Option<FlightRegime>,
    report: Option<FlightModeReport>,
}
impl Guidance for RegimeGuidance {
    fn guide(
        &mut self,
        _dt: f32,
        regime: FlightRegime,
        _input: &ControlInput,
        _state: &VehicleState,
    ) -> Setpoint {
        self.requested = Some(regime);
        Setpoint::Attitude(AttitudeSetpoint::default())
    }
    fn flight_mode(&self) -> Option<FlightModeReport> {
        self.report
    }
}

#[test]
fn view_passes_the_regime_in_and_the_flight_mode_out() {
    let report = FlightModeReport {
        mode: crate::FlightMode::ToCopter,
        refusal: None,
    };
    for report in [None, Some(report)] {
        let mut view = View::tailsitter(RegimeGuidance {
            requested: None,
            report,
        });
        view.guide(
            DT,
            FlightRegime::Plane,
            &ControlInput::default(),
            &VehicleState::default(),
        );
        check!(view.inner().requested == Some(FlightRegime::Plane));
        check!(view.flight_mode() == report);
    }
}
