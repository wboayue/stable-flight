//! Level flight on `airframes::plane()`: the line, altitude, and airspeed held in calm air and
//! in steady wind, with ArduPilot's autotest bands (Plane-4.5 `Tools/autotest/arduplane.py`).

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight_sim::{Wind, airframes};

use common::fly;
use common::plane::{AIRSPEED_BAND_MPS, ALTITUDE_BAND_M, LineDeviation, PlanePlant, plane_loop};

/// Seconds to settle from the start (the prop spins up from rest) before the hold is judged.
const SETTLE_S: f32 = 15.0;

/// Seconds the hold is judged over.
const HOLD_S: f32 = 30.0;

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
        let (altitude_m, airspeed_mps) = (
            plant.altitude_m(),
            plant.airframe.stabilizer.trim_airspeed_mps,
        );
        let mut flight_loop = plane_loop(&plant, plant.line());
        fly(&mut flight_loop, SETTLE_S, |_, _| {});
        let mut worst = LineDeviation::default();
        fly(&mut flight_loop, HOLD_S, |flight_loop, _| {
            worst.observe(flight_loop.state(), altitude_m, airspeed_mps);
        });
        check!(worst.altitude_m <= ALTITUDE_BAND_M, "{case}: {worst:?}");
        check!(worst.airspeed_mps <= AIRSPEED_BAND_MPS, "{case}: {worst:?}");
        check!(worst.cross_track_m <= CROSS_TRACK_M, "{case}: {worst:?}");
    }
}
