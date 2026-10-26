//! Rate-axis identification (#516) through `CopterStabilizer`'s probe host: excited on the
//! roll and pitch angle loops, each copter airframe's fitted model matches its own motor lag
//! and the actuator delay, from the low and high starts alike.

mod common;

use assert2::check;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::COPTER_IDENTIFY;
use stable_flight::tuning::{Measurement, ProbeConfig, ProbeStatus};
use stable_flight_sim::Wind;
use stable_flight_sim::airframes;

use common::Plant;
use common::robustness::{Set, Start, perturbations, starting};

/// Most the fitted lag may miss the model's motor lag, as a share of it: measured within 0.6%
/// (#516), so 5% leaves about 8× margin.
const LAG_SHARE: f32 = 0.05;

/// Simulated seconds a probe may run: the excitation's 3 s plus margin.
const TIMEOUT_S: f32 = 5.0;

/// (set, delays it may fit, in ticks): the actuator delay's ticks plus the zero-order hold's
/// one; 25 ms is 6.25 ticks, so 6 or 7.
const SETS: [(Set, &[usize]); 3] = [
    (Set::Clean, &[1]),
    (Set::Delay10Ms, &[3]),
    (Set::Delay25Ms, &[6, 7]),
];

#[test]
fn fits_each_airframes_motor_lag_and_delay() {
    let airframes = [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
        ("racer", airframes::racer()),
    ];
    for (name, airframe) in airframes {
        let lag_s = airframe.model.motor_time_constant_s;
        for (set, delays) in SETS {
            for start in [Start::Low, Start::High] {
                let plant = Plant {
                    airframe: starting(airframe, start),
                    wind: Wind::default(),
                    perturbations: perturbations(set, &airframe),
                };
                for target in [TunedLoop::Roll, TunedLoop::Pitch] {
                    let row = format!("{name} {set:?} {start:?} {target:?}");
                    let status = common::probe(
                        &plant,
                        target,
                        ProbeConfig::Identify(COPTER_IDENTIFY),
                        TIMEOUT_S,
                    );
                    let Some(ProbeStatus::Done(Measurement::Identify(model))) = status else {
                        check!(false, "{row}: {status:?}");
                        continue;
                    };
                    check!(delays.contains(&model.delay_ticks), "{row}: {model:?}");
                    let fitted_s = model.lag_s().unwrap_or(f32::NAN);
                    check!(
                        (fitted_s - lag_s).abs() <= LAG_SHARE * lag_s,
                        "{row}: lag {fitted_s} s against {lag_s} s"
                    );
                }
            }
        }
    }
}
