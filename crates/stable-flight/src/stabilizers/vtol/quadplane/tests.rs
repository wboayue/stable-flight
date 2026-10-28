use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::controllers::RateGains;
use crate::guidance::vtol::{QuadPlaneGuidance, QuadPlaneGuidanceConfig};
use crate::stabilizers::copter::TunedLoop;
use crate::{AttitudeSetpoint, BlendSetpoint, BodyRateSetpoint, PlaneSetpoint};

const DT: f32 = 0.004;

fn hover_attitude() -> AttitudeSetpoint {
    AttitudeSetpoint {
        roll_rad: 0.2,
        thrust: 0.5,
        ..Default::default()
    }
}

fn plane_attitude() -> PlaneSetpoint {
    PlaneSetpoint {
        roll_rad: 0.2,
        thrust: 0.7,
        ..Default::default()
    }
}

fn blend(lift_share: f32) -> Setpoint {
    Setpoint::Blend(BlendSetpoint {
        hover: hover_attitude(),
        plane: plane_attitude(),
        lift_share,
    })
}

/// A config whose plane roll rate loop is all I, so its integral shows in the roll moment.
fn plane_integral_only() -> QuadPlaneStabilizerConfig {
    let mut config = QuadPlaneStabilizerConfig::default();
    config.plane.rate.roll = RateGains {
        p: 0.0,
        i: 1.0,
        ff: 0.0,
    };
    config
}

/// The cruise output of `output`, failing the check without one.
fn cruise(output: &ControlOutput) -> CruiseOutput {
    check!(output.cruise.is_some());
    output.cruise.unwrap_or_default()
}

/// Hover setpoints fly the lift rotors as the copter stabilizer alone, surfaces neutral and
/// the pusher stopped; plane setpoints fly the surfaces and pusher as the plane stabilizer
/// alone, the lift rotors stopped.
#[test]
fn flies_each_setpoint_on_its_actuators() {
    let config = QuadPlaneStabilizerConfig::default();
    let state = VehicleState::default();
    let hover_rates = Setpoint::BodyRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    for setpoint in [Setpoint::Attitude(hover_attitude()), hover_rates] {
        let ours = QuadPlaneStabilizer::new(config).stabilize(DT, &setpoint, &state);
        let alone = CopterStabilizer::new(config.hover).stabilize(DT, &setpoint, &state);
        check!(ours.thrust == alone.thrust, "{setpoint:?}");
        check!(ours.moments == alone.moments, "{setpoint:?}");
        check!(ours.cruise == Some(CruiseOutput::default()), "{setpoint:?}");
    }
    let plane_rates = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    for setpoint in [Setpoint::Plane(plane_attitude()), plane_rates] {
        let ours = QuadPlaneStabilizer::new(config).stabilize(DT, &setpoint, &state);
        let alone = PlaneStabilizer::new(config.plane).stabilize(DT, &setpoint, &state);
        check!(ours.thrust == 0.0, "{setpoint:?}");
        check!(ours.moments == Vec3::zeros(), "{setpoint:?}");
        let cruise = cruise(&ours);
        check!(cruise.thrust == alone.thrust, "{setpoint:?}");
        check!(cruise.moments == alone.moments, "{setpoint:?}");
    }
}

/// A blend weights the lift rotors' thrust and moments by the share, saturated (NaN as 1),
/// and flies the surfaces and pusher in full.
#[test]
fn blend_weights_the_lift_rotors_by_their_share() {
    let config = QuadPlaneStabilizerConfig::default();
    let state = VehicleState::default();
    let lift = CopterStabilizer::new(config.hover).stabilize(
        DT,
        &Setpoint::Attitude(hover_attitude()),
        &state,
    );
    let wing = PlaneStabilizer::new(config.plane).stabilize(
        DT,
        &Setpoint::Plane(plane_attitude()),
        &state,
    );
    check!(lift.moments.x > 0.0);
    // (share, weight flown)
    let cases = [
        (1.0, 1.0),
        (0.5, 0.5),
        (0.0, 0.0),
        (f32::NAN, 1.0),
        (1.5, 1.0),
        (-0.5, 0.0),
    ];
    for (share, weight) in cases {
        let output = QuadPlaneStabilizer::new(config).stabilize(DT, &blend(share), &state);
        assert_relative_eq!(output.thrust, lift.thrust * weight);
        assert_relative_eq!(output.moments, lift.moments * weight);
        let cruise = cruise(&output);
        check!(cruise.thrust == wing.thrust, "{share}");
        check!(cruise.moments == wing.moments, "{share}");
    }
}

/// A side that starts flying is primed; one that keeps flying out of a blend keeps its
/// integrals.
#[test]
fn primes_a_side_as_it_starts_flying() {
    let config = plane_integral_only();
    let state = VehicleState::default();
    let plane = Setpoint::Plane(plane_attitude());
    const TICKS: usize = 50;

    // Blend then plane: the plane side's integral carries on, as if it had flown plane all
    // along.
    let mut ours = QuadPlaneStabilizer::new(config);
    let mut alone = PlaneStabilizer::new(config.plane);
    for _ in 0..TICKS {
        ours.stabilize(DT, &blend(0.5), &state);
        alone.stabilize(DT, &plane, &state);
    }
    let carried = cruise(&ours.stabilize(DT, &plane, &state));
    check!(carried.moments == alone.stabilize(DT, &plane, &state).moments);
    check!(carried.moments.x > 0.0);

    // Hover then blend: the plane side starts fresh.
    ours.stabilize(DT, &Setpoint::Attitude(hover_attitude()), &state);
    let fresh = PlaneStabilizer::new(config.plane).stabilize(DT, &plane, &state);
    check!(cruise(&ours.stabilize(DT, &blend(0.5), &state)).moments == fresh.moments);

    // Plane then blend: the hover side starts fresh.
    let mut ours = QuadPlaneStabilizer::new(config);
    ours.stabilize(DT, &blend(1.0), &state);
    ours.stabilize(DT, &plane, &state);
    let hover = Setpoint::Attitude(hover_attitude());
    let fresh = CopterStabilizer::new(config.hover).stabilize(DT, &hover, &state);
    check!(ours.stabilize(DT, &blend(1.0), &state).moments == fresh.moments);
}

/// Saturation comes from the sides that flew, merged in a blend.
#[test]
fn saturation_from_the_sides_that_flew() {
    let mut stabilizer = QuadPlaneStabilizer::new(QuadPlaneStabilizerConfig::default());
    let state = VehicleState::default();
    let steep_bank = PlaneSetpoint {
        roll_rad: 1.5,
        thrust: 0.5,
        ..Default::default()
    };
    let over_thrust = AttitudeSetpoint {
        thrust: 1.5,
        ..Default::default()
    };
    stabilizer.stabilize(DT, &Setpoint::Plane(steep_bank), &state);
    check!(stabilizer.saturation().tilt);
    check!(!stabilizer.saturation().thrust);
    stabilizer.stabilize(DT, &Setpoint::Attitude(over_thrust), &state);
    check!(!stabilizer.saturation().tilt);
    check!(stabilizer.saturation().thrust);
    let both = Setpoint::Blend(BlendSetpoint {
        hover: over_thrust,
        plane: steep_bank,
        lift_share: 0.5,
    });
    stabilizer.stabilize(DT, &both, &state);
    check!(stabilizer.saturation().tilt);
    check!(stabilizer.saturation().thrust);
}

/// `prime` returns to the hover side, clearing both sides' integrals.
#[test]
fn prime_clears_both_sides() {
    let config = plane_integral_only();
    let state = VehicleState::default();
    let mut stabilizer = QuadPlaneStabilizer::new(config);
    for _ in 0..50 {
        stabilizer.stabilize(DT, &blend(0.5), &state);
    }
    stabilizer.prime(&state);
    let fresh = QuadPlaneStabilizer::new(config).stabilize(DT, &blend(0.5), &state);
    check!(stabilizer.stabilize(DT, &blend(0.5), &state) == fresh);
}

/// Params chain hover then plane; a write lands in `config()`, and `set_config` validates the
/// whole before applying any part.
#[test]
fn params_and_config_chain_the_parts() {
    let mut stabilizer = QuadPlaneStabilizer::new(QuadPlaneStabilizerConfig::default());
    check!(stabilizer.param_name(0) == Some("STAB_RATE_RLL_P"));
    check!(stabilizer.set_param("PSTAB_ASPD_TRIM", 12.0).is_ok());
    check!(stabilizer.config().plane.trim_airspeed_mps == 12.0);
    check!(stabilizer.param_at(stabilizer.param_count()).is_none());
    check!(stabilizer.set_param("PSTAB_ASPD_TRIM", -1.0).is_err());
    check!(stabilizer.config().plane.trim_airspeed_mps == 12.0);

    let mut bad = *stabilizer.config();
    bad.hover.thrust_curve_exponent = 3.0;
    bad.plane.trim_airspeed_mps = -1.0;
    let error = stabilizer.set_config(bad).map_err(|e| e.section);
    check!(error == Err(Some("plane")));
    check!(stabilizer.config().hover.thrust_curve_exponent != 3.0);
}

/// Autotune probes run on the hover side.
#[test]
fn autotune_probes_the_hover_side() {
    let mut stabilizer = QuadPlaneStabilizer::new(QuadPlaneStabilizerConfig::default());
    check!(stabilizer.probe().is_none());
    stabilizer.start_probe(
        TunedLoop::RollRate,
        ProbeConfig::Step(crate::tuning::autotune::COPTER_RATE_STEP),
    );
    check!(stabilizer.probe().map(|(target, _)| target) == Some(TunedLoop::RollRate));
    check!(stabilizer.stop_probe().is_some());
    check!(stabilizer.probe().is_none());
}

/// Guidance and stabilizer param names are unique across the quadplane's five tables, and
/// fit MAVLink's 16 bytes.
#[test]
fn quadplane_param_names_are_unique_and_short() {
    let guidance = QuadPlaneGuidance::new(QuadPlaneGuidanceConfig::default());
    let stabilizer = QuadPlaneStabilizer::new(QuadPlaneStabilizerConfig::default());
    let mut names: Vec<&str> = (0..guidance.param_count())
        .filter_map(|i| guidance.param_name(i))
        .chain((0..stabilizer.param_count()).filter_map(|i| stabilizer.param_name(i)))
        .collect();
    check!(names.iter().all(|n| n.len() <= crate::params::MAX_NAME_LEN));
    let count = names.len();
    names.sort_unstable();
    names.dedup();
    check!(names.len() == count);
}

#[test]
#[should_panic(expected = "invalid QuadPlaneStabilizerConfig")]
fn rejects_an_invalid_config() {
    let mut config = QuadPlaneStabilizerConfig::default();
    config.plane.trim_airspeed_mps = 0.0;
    QuadPlaneStabilizer::new(config);
}
