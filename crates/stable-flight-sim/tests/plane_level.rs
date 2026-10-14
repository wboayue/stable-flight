//! Level flight on `airframes::plane()`: the line, altitude, and airspeed held in calm air and
//! in steady wind, with ArduPilot's autotest bands (Plane-4.5 `Tools/autotest/arduplane.py`).

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight_sim::{Wind, airframes};

use common::fly;
use common::plane::{PlanePlant, north_line, plane_loop};

/// Altitude flown, in m (the airframe's start).
const ALTITUDE_M: f32 = 100.0;

/// Airspeed flown, in m/s (the airframe's trim and start).
const AIRSPEED_MPS: f32 = 15.0;

/// Seconds to settle from the start (the prop spins up from rest) before the hold is judged.
const SETTLE_S: f32 = 15.0;

/// Seconds the hold is judged over.
const HOLD_S: f32 = 30.0;

/// Altitude band, either way, in m: ArduPilot's for a loiter altitude regained
/// (`arduplane.py:2727`), held here for the whole window.
const ALTITUDE_BAND_M: f32 = 1.0;

/// Airspeed band, either way, in m/s: ArduPilot's for a commanded airspeed held
/// (`arduplane.py:897`, 5 s minimum; the whole window here).
const AIRSPEED_BAND_MPS: f32 = 1.0;

/// Distance off the line, in m: no external band applies. Measured 0.05 m at most, in the
/// 5 m/s crosswind; 1 m leaves a 20× margin.
const CROSS_TRACK_M: f32 = 1.0;

#[test]
fn holds_line_altitude_and_airspeed() {
    // (case, steady wind NEU m/s)
    let cases = [
        ("calm", Vec3::zeros()),
        ("headwind", Vec3::new(-5.0, 0.0, 0.0)),
        ("crosswind", Vec3::new(0.0, 5.0, 0.0)),
    ];
    for (case, steady_mps) in cases {
        let plant = PlanePlant {
            wind: Wind {
                steady_mps,
                gust: None,
            },
            ..PlanePlant::clean(airframes::plane())
        };
        let mut flight_loop = plane_loop(&plant, north_line(ALTITUDE_M, AIRSPEED_MPS));
        fly(&mut flight_loop, SETTLE_S, |_, _| {});
        let (mut altitude_m, mut airspeed_mps, mut cross_track_m) = (0.0_f32, 0.0_f32, 0.0_f32);
        fly(&mut flight_loop, HOLD_S, |flight_loop, _| {
            let state = flight_loop.state();
            altitude_m = altitude_m.max((state.position_m.z - ALTITUDE_M).abs());
            let airspeed = state.airspeed_mps.unwrap_or(f32::NAN);
            airspeed_mps = airspeed_mps.max((airspeed - AIRSPEED_MPS).abs());
            cross_track_m = cross_track_m.max(state.position_m.y.abs());
        });
        check!(
            altitude_m <= ALTITUDE_BAND_M,
            "{case}: altitude off by {altitude_m} m"
        );
        check!(
            airspeed_mps <= AIRSPEED_BAND_MPS,
            "{case}: airspeed off by {airspeed_mps} m/s"
        );
        check!(
            cross_track_m <= CROSS_TRACK_M,
            "{case}: {cross_track_m} m off the line"
        );
    }
}
