use assert2::check;

use super::*;
use crate::guidance::vtol::{TailsitterGuidance, TailsitterGuidanceConfig};
use crate::{AttitudeSetpoint, BodyRateSetpoint, PlaneSetpoint};

const DT: f32 = 0.004;

fn hover_setpoint() -> Setpoint {
    Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.2,
        thrust: 0.5,
        ..Default::default()
    })
}

fn plane_setpoint() -> Setpoint {
    Setpoint::Plane(PlaneSetpoint {
        roll_rad: 0.2,
        thrust: 0.5,
        ..Default::default()
    })
}

/// Each setpoint flies on its side, the same as that side alone.
#[test]
fn dispatches_each_setpoint_to_its_side() {
    let config = TailsitterStabilizerConfig::default();
    let state = VehicleState::default();
    let hover_rates = Setpoint::BodyRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    for setpoint in [hover_setpoint(), hover_rates] {
        let ours = TailsitterStabilizer::new(config).stabilize(DT, &setpoint, &state);
        let alone =
            View::tailsitter(CopterStabilizer::new(config.hover)).stabilize(DT, &setpoint, &state);
        check!(ours == alone, "{setpoint:?}");
    }
    let plane_rates = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    for setpoint in [plane_setpoint(), plane_rates] {
        let ours = TailsitterStabilizer::new(config).stabilize(DT, &setpoint, &state);
        let alone = PlaneStabilizer::new(config.plane).stabilize(DT, &setpoint, &state);
        check!(ours == alone, "{setpoint:?}");
    }
}

/// Switching sides primes the incoming side: the plane side's integral from earlier plane
/// flight doesn't survive a stretch of hover.
#[test]
fn switching_sides_primes_the_incoming_one() {
    let mut config = TailsitterStabilizerConfig::default();
    config.plane.rate.roll = crate::controllers::RateGains {
        p: 0.0,
        i: 1.0,
        ff: 0.0,
    };
    let state = VehicleState::default();
    let roll = Setpoint::PlaneRate(BodyRateSetpoint {
        roll_radps: 1.0,
        thrust: 0.5,
        ..Default::default()
    });
    let hold = Setpoint::PlaneRate(BodyRateSetpoint {
        thrust: 0.5,
        ..Default::default()
    });

    let mut stabilizer = TailsitterStabilizer::new(config);
    for _ in 0..50 {
        stabilizer.stabilize(DT, &roll, &state);
    }
    // Staying on the plane side, the integral holds a roll moment.
    check!(stabilizer.stabilize(DT, &hold, &state).moments.x > 0.0);

    stabilizer.stabilize(DT, &hover_setpoint(), &state);
    check!(stabilizer.stabilize(DT, &hold, &state).moments.x == 0.0);
}

/// Saturation comes from the side that flew.
#[test]
fn saturation_from_the_active_side() {
    let mut stabilizer = TailsitterStabilizer::new(TailsitterStabilizerConfig::default());
    let state = VehicleState::default();
    let steep_bank = Setpoint::Plane(PlaneSetpoint {
        roll_rad: 1.5,
        thrust: 0.5,
        ..Default::default()
    });
    stabilizer.stabilize(DT, &steep_bank, &state);
    check!(stabilizer.saturation().tilt);
    stabilizer.stabilize(DT, &hover_setpoint(), &state);
    check!(!stabilizer.saturation().tilt);
}

/// Params chain hover then plane, and a write lands in `config()`.
#[test]
fn params_chain_the_parts() {
    let mut stabilizer = TailsitterStabilizer::new(TailsitterStabilizerConfig::default());
    check!(stabilizer.param_name(0) == Some("STAB_RATE_RLL_P"));
    check!(stabilizer.set_param("PSTAB_ASPD_TRIM", 12.0).is_ok());
    check!(stabilizer.config().plane.trim_airspeed_mps == 12.0);
    check!(stabilizer.set_param("PSTAB_ASPD_TRIM", -1.0).is_err());
    check!(stabilizer.config().plane.trim_airspeed_mps == 12.0);
}

/// Guidance and stabilizer param names are unique across the tailsitter's five tables, and
/// fit MAVLink's 16 bytes.
#[test]
fn tailsitter_param_names_are_unique_and_short() {
    let guidance = TailsitterGuidance::new(TailsitterGuidanceConfig::default());
    let stabilizer = TailsitterStabilizer::new(TailsitterStabilizerConfig::default());
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
