//! Coordinated turns on `airframes::plane()`: a loiter flown by guidance and a bank held in
//! absolute attitude mode, each with the ball centered.

mod common;

use assert2::check;
use stable_flight::Vec2;
use stable_flight_sim::airframes;

use common::fly;
use common::plane::{Interlude, PlanePlant, attitude_command, loiter, plane_loop};

/// Seconds to fly before a turn is judged: the loiter's entry, or the held bank's lead on the
/// line and its roll-in.
const SETTLE_S: f32 = 30.0;

/// Seconds a turn is judged over: a loiter circle at trim airspeed takes about 17 s.
const HOLD_S: f32 = 40.0;

/// Seconds on the line before the held bank rolls in.
const LEAD_S: f32 = 20.0;

/// Sideslip, either way, in rad: no external band applies. Measured 0.9° in the loiter and
/// 1.4° at a held 45° bank; 2.5° leaves about 2× margin and fails rate feedforward that
/// leaves the rudder to feedback (PX4's yaw default: 2.9° and 4.0°).
const SIDESLIP_RAD: f32 = 2.5 * core::f32::consts::PI / 180.0;

/// Altitude band, either way, in m: ArduPilot's for a loiter altitude regained
/// (Plane-4.5 autotest `AutoTestPlane.LOITER`), held here for the whole window.
const ALTITUDE_BAND_M: f32 = 1.0;

/// Distance off the loiter circle, either way, in m: no external band applies. Measured 2.8 m;
/// 5 m leaves about 2× margin.
const RADIUS_BAND_M: f32 = 5.0;

#[test]
fn loiters_coordinated_on_the_circle_and_altitude() {
    let plant = PlanePlant::clean(airframes::plane());
    let airspeed_mps = plant.airframe.stabilizer.trim_airspeed_mps;
    let (altitude_m, radius_m) = (plant.altitude_m(), plant.airframe.guidance.loiter_radius_m);
    let center_m = Vec2::new(0.0, radius_m);
    let mut flight_loop = plane_loop(&plant, loiter(center_m, altitude_m, airspeed_mps));
    fly(&mut flight_loop, SETTLE_S, |_, _| {});
    let (mut sideslip_rad, mut altitude_off_m, mut radius_off_m) = (0.0_f32, 0.0_f32, 0.0_f32);
    fly(&mut flight_loop, HOLD_S, |flight_loop, _| {
        let state = flight_loop.state();
        let position_m = state.position_m;
        sideslip_rad = sideslip_rad.max(flight_loop.vehicle().sideslip_rad().abs());
        altitude_off_m = altitude_off_m.max((position_m.z - altitude_m).abs());
        let from_center_m = Vec2::new(position_m.x, position_m.y) - center_m;
        radius_off_m = radius_off_m.max((from_center_m.norm() - radius_m).abs());
    });
    check!(sideslip_rad <= SIDESLIP_RAD, "sideslip {sideslip_rad} rad");
    check!(
        altitude_off_m <= ALTITUDE_BAND_M,
        "altitude off by {altitude_off_m} m"
    );
    check!(
        radius_off_m <= RADIUS_BAND_M,
        "{radius_off_m} m off the circle"
    );
}

#[test]
fn holds_a_bank_coordinated() {
    let plant = PlanePlant::clean(airframes::plane());
    let thrust = plant.airframe.guidance.cruise_thrust;
    // (bank °): a loiter's, and FBWA's limit (core's plane guidance `max_bank_rad`).
    for bank_deg in [30.0_f32, 45.0] {
        let bank = attitude_command(bank_deg.to_radians(), 0.0, thrust);
        let commander = Interlude::new(plant.line(), LEAD_S, f32::INFINITY, bank);
        let mut flight_loop = plane_loop(&plant, commander);
        fly(&mut flight_loop, SETTLE_S, |_, _| {});
        let mut sideslip_rad = 0.0_f32;
        fly(&mut flight_loop, HOLD_S, |flight_loop, _| {
            sideslip_rad = sideslip_rad.max(flight_loop.vehicle().sideslip_rad().abs());
        });
        check!(
            sideslip_rad <= SIDESLIP_RAD,
            "{bank_deg}° bank: sideslip {sideslip_rad} rad"
        );
    }
}
