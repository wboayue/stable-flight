//! Altitude steps on `airframes::plane()` by total energy control (#454): climbs and descents
//! along the line, holding airspeed. No external band applies (neither ArduPilot's nor PX4's
//! autotests time an altitude step), so each band is the measured value with its margin.

mod common;

use assert2::check;
use stable_flight_sim::airframes;

use common::fly;
use common::plane::{ALTITUDE_BAND_M, Interlude, PlanePlant, north_line_command, plane_loop};

/// Seconds on the line before the step: the prop spins up from rest and the loops settle.
const LEAD_S: f32 = 15.0;

/// Seconds flown after the step.
const STEP_WINDOW_S: f32 = 60.0;

/// Step size, either way, in m.
const STEP_M: f32 = 20.0;

/// Most overshoot past the new altitude, in m. Measured 0.12 m up and 0.04 m down; 0.5 m
/// leaves 4× margin. Separate altitude and airspeed loops overshot 1.6 and 1.3 m.
const OVERSHOOT_M: f32 = 0.5;

/// Seconds until the altitude stays within [`ALTITUDE_BAND_M`] of the new altitude. Measured
/// 9.5 s up and 8.3 s down (the 5 m/s climb limit alone takes 4 s; the height demand's lag
/// rounds off the approach); 12 s leaves about 1.25× margin.
const SETTLE_S: f32 = 12.0;

/// Most the altitude moves the wrong way first, in m. Measured 0.11 m (climbing trades a
/// little height for speed first); 0.5 m leaves about 4× margin.
const WRONG_WAY_M: f32 = 0.5;

/// Largest airspeed error through the step, in m/s: the energy trade the separate loops
/// couldn't make. Measured 0.83 m/s up and 1.03 m/s down; 1.5 m/s leaves about 1.45× margin.
/// Separate altitude and airspeed loops lost 2.5 and 3.0 m/s.
const AIRSPEED_ERROR_MPS: f32 = 1.5;

/// What a step did: overshoot past the new altitude (m, in the step's direction), seconds
/// until it stays within [`ALTITUDE_BAND_M`], the most it moved the wrong way (m), and the
/// largest airspeed error (m/s).
#[derive(Debug)]
struct Step {
    overshoot_m: f32,
    settled_s: f32,
    wrong_way_m: f32,
    airspeed_error_mps: f32,
}

fn step(step_m: f32) -> Step {
    let plant = PlanePlant::clean(airframes::plane());
    let (start_m, airspeed_mps) = (
        plant.altitude_m(),
        plant.airframe.stabilizer.trim_airspeed_mps,
    );
    let target_m = start_m + step_m;
    let direction = step_m.signum();
    let commander = Interlude::new(
        plant.line(),
        LEAD_S,
        f32::INFINITY,
        north_line_command(target_m, airspeed_mps),
    );
    let mut flight_loop = plane_loop(&plant, commander);
    fly(&mut flight_loop, LEAD_S, |_, _| {});
    let mut result = Step {
        overshoot_m: 0.0,
        settled_s: 0.0,
        wrong_way_m: 0.0,
        airspeed_error_mps: 0.0,
    };
    fly(&mut flight_loop, STEP_WINDOW_S, |flight_loop, t| {
        let state = flight_loop.state();
        let altitude_m = state.position_m.z;
        result.overshoot_m = result.overshoot_m.max((altitude_m - target_m) * direction);
        result.wrong_way_m = result.wrong_way_m.max((start_m - altitude_m) * direction);
        if (altitude_m - target_m).abs() > ALTITUDE_BAND_M {
            result.settled_s = t;
        }
        let airspeed = state.airspeed_mps.unwrap_or(f32::NAN);
        result.airspeed_error_mps = result
            .airspeed_error_mps
            .max((airspeed - airspeed_mps).abs());
    });
    result
}

/// A 20 m step up and down settles within [`SETTLE_S`], overshooting at most [`OVERSHOOT_M`]
/// and holding airspeed within [`AIRSPEED_ERROR_MPS`].
#[test]
fn steps_altitude_holding_airspeed() {
    for step_m in [STEP_M, -STEP_M] {
        let result = step(step_m);
        check!(result.overshoot_m <= OVERSHOOT_M, "{step_m}: {result:?}");
        check!(result.settled_s <= SETTLE_S, "{step_m}: {result:?}");
        check!(result.wrong_way_m <= WRONG_WAY_M, "{step_m}: {result:?}");
        check!(
            result.airspeed_error_mps <= AIRSPEED_ERROR_MPS,
            "{step_m}: {result:?}"
        );
    }
}
