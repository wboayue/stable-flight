use assert2::{assert, check};

use super::*;
use crate::Vec3;

const DT: f32 = 0.004;

fn config() -> PlaneTuneHoldConfig {
    PlaneTuneHoldConfig::new(0.55, 18.0)
}

/// Level flight at `pitch_rad` and `airspeed_mps`, heading north at `position_m`.
fn flying(position_m: Vec2, pitch_rad: f32, airspeed_mps: Option<f32>) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(position_m.x, position_m.y, 120.0),
        pitch_rad,
        airspeed_mps,
        ..Default::default()
    }
}

fn attitude(command: &Command) -> AbsoluteAttitudeCommand {
    let ControlInput::AbsoluteAttitude(attitude) = command.input else {
        panic!("not an attitude: {command:?}");
    };
    attitude
}

#[test]
fn holds_the_attitude_it_starts_on() {
    let mut hold = PlaneTuneHold::new(config());
    let command = hold.next_command(DT, &flying(Vec2::zeros(), 0.07, Some(18.0)));
    check!(command.arm);
    check!(command.regime == FlightRegime::Plane);
    let attitude = attitude(&command);
    check!(attitude.roll_rad == 0.0);
    check!(attitude.pitch_rad == 0.07);
    check!(attitude.thrust == 0.55);
}

/// The trim pitches the nose down when slow and up when fast, within its limit of the start,
/// and leaves it without an airspeed.
#[test]
fn trims_pitch_toward_trim_airspeed() {
    let start_rad = 0.05;
    for (airspeed_mps, expected_rad) in [
        (Some(15.0), start_rad - 3.0 * DEG),
        (Some(21.0), start_rad + 3.0 * DEG),
        (Some(5.0), start_rad - 10.0 * DEG),
        (Some(18.0), start_rad),
        (None, start_rad),
    ] {
        let mut hold = PlaneTuneHold::new(config());
        hold.next_command(DT, &flying(Vec2::zeros(), start_rad, Some(18.0)));
        let state = flying(Vec2::zeros(), start_rad, airspeed_mps);
        let mut pitch_rad = f32::NAN;
        // Six time constants: within 0.3% of the target.
        for _ in 0..(30.0 / DT) as usize {
            pitch_rad = attitude(&hold.next_command(DT, &state)).pitch_rad;
        }
        check!(
            (pitch_rad - expected_rad).abs() < 0.001,
            "{airspeed_mps:?}: {pitch_rad}"
        );
    }
}

#[test]
fn the_trim_moves_slowly() {
    let mut hold = PlaneTuneHold::new(config());
    hold.next_command(DT, &flying(Vec2::zeros(), 0.0, Some(18.0)));
    // A probe's 4 s at 3 m/s slow moves the nose 56% of the way to its trim, 1.7° of 3°.
    let slow = flying(Vec2::zeros(), 0.0, Some(15.0));
    let mut pitch_rad = 0.0;
    for _ in 0..(4.0 / DT) as usize {
        pitch_rad = attitude(&hold.next_command(DT, &slow)).pitch_rad;
    }
    check!(
        pitch_rad < -1.6 * DEG && pitch_rad > -1.8 * DEG,
        "{pitch_rad}"
    );
}

/// Past the return radius the hold banks toward its start, but only once turns are allowed;
/// a turn under way finishes when turns aren't, and ends heading back.
#[test]
fn turns_back_between_probes_only() {
    let mut hold = PlaneTuneHold::new(config());
    hold.next_command(DT, &flying(Vec2::zeros(), 0.0, Some(18.0)));
    // 700 m north, heading north: the start is behind.
    let far = flying(Vec2::new(700.0, 0.0), 0.0, Some(18.0));
    check!(attitude(&hold.next_command(DT, &far)).roll_rad == 0.0);
    check!(!hold.turning());

    hold.allow_turn(true);
    let bank_rad = attitude(&hold.next_command(DT, &far)).roll_rad;
    check!(bank_rad.abs() == 20.0 * DEG);
    check!(hold.turning());

    hold.allow_turn(false);
    let east = VehicleState {
        heading_rad: core::f32::consts::FRAC_PI_2,
        ..far.clone()
    };
    // Heading east, the start to the south: a right turn, still under way.
    check!(attitude(&hold.next_command(DT, &east)).roll_rad == 20.0 * DEG);
    let south = VehicleState {
        heading_rad: core::f32::consts::PI - 0.05,
        ..far
    };
    check!(attitude(&hold.next_command(DT, &south)).roll_rad == 0.0);
    check!(!hold.turning());
}

#[test]
fn stays_level_inside_the_return_radius() {
    let mut hold = PlaneTuneHold::new(config());
    hold.allow_turn(true);
    hold.next_command(DT, &flying(Vec2::zeros(), 0.0, Some(18.0)));
    let near = flying(Vec2::new(500.0, 0.0), 0.0, Some(18.0));
    check!(attitude(&hold.next_command(DT, &near)).roll_rad == 0.0);
}

#[test]
fn a_vtol_recovers_hovering_where_it_is() {
    let mut hold = PlaneTuneHold::new(PlaneTuneHoldConfig {
        recover_to: FlightRegime::Copter,
        ..config()
    });
    hold.next_command(DT, &flying(Vec2::zeros(), 0.0, Some(18.0)));
    hold.recover();
    assert!(hold.is_recovering());
    let there = flying(Vec2::new(300.0, -40.0), 0.0, Some(18.0));
    let command = hold.next_command(DT, &there);
    check!(command.regime == FlightRegime::Copter);
    let ControlInput::Waypoint(waypoint) = command.input else {
        panic!("not a waypoint: {command:?}");
    };
    check!(waypoint.position_m == Vec2::new(300.0, -40.0));
    check!(waypoint.altitude_m == 120.0);
    // Fixed where it began.
    let later = flying(Vec2::new(320.0, -40.0), 0.0, Some(10.0));
    check!(hold.next_command(DT, &later) == command);
}

#[test]
fn a_plane_recovers_wings_level() {
    let mut hold = PlaneTuneHold::new(config());
    hold.allow_turn(true);
    hold.next_command(DT, &flying(Vec2::zeros(), 0.04, Some(18.0)));
    hold.next_command(DT, &flying(Vec2::new(700.0, 0.0), 0.04, Some(18.0)));
    hold.recover();
    let command = hold.next_command(DT, &flying(Vec2::new(700.0, 0.0), 0.3, Some(18.0)));
    check!(command.regime == FlightRegime::Plane);
    check!(attitude(&command).roll_rad == 0.0);
    check!(!hold.turning());
}

#[test]
fn rejects_an_invalid_config() {
    let rows: [(&str, PlaneTuneHoldConfig); 4] = [
        (
            "thrust",
            PlaneTuneHoldConfig {
                thrust: 1.2,
                ..config()
            },
        ),
        (
            "trim_airspeed_mps",
            PlaneTuneHoldConfig {
                trim_airspeed_mps: 0.0,
                ..config()
            },
        ),
        (
            "turn_bank_rad",
            PlaneTuneHoldConfig {
                turn_bank_rad: 1.2,
                ..config()
            },
        ),
        (
            "return_radius_m",
            PlaneTuneHoldConfig {
                return_radius_m: f32::NAN,
                ..config()
            },
        ),
    ];
    for (field, bad) in rows {
        check!(bad.validate().map_err(|e| e.field) == Err(field));
    }
}
