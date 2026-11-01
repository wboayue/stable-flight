//! Rate-axis identification (#516) through `CopterStabilizer`'s probe host, and the gains
//! loop shaping designs from it (#517): excited on the roll and pitch rate loops, each copter
//! airframe's fitted model matches its own motor lag and the actuator delay from the low and
//! high starts alike, and the designed gains meet the step spec on the same plant.

mod common;

use assert2::check;
use stable_flight::controllers::GyroFilters;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{COPTER_IDENTIFY, COPTER_LOOP_SHAPING};
use stable_flight::tuning::{
    Measurement, ProbeConfig, ProbeStatus, RateModel, design_angle_p, design_rate,
};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{Perturbations, Wind};

use common::robustness::{Set, Start, perturbations, starting};
use common::{Plant, spec};

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
                for target in [TunedLoop::RollRate, TunedLoop::PitchRate] {
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

/// `target`'s fitted model on `plant`, through the probe.
fn identified(plant: &Plant, target: TunedLoop) -> RateModel {
    let config = ProbeConfig::Identify(COPTER_IDENTIFY);
    match common::probe(plant, target, config, TIMEOUT_S) {
        Some(ProbeStatus::Done(Measurement::Identify(model))) => model,
        other => panic!("{target:?}: {other:?}"),
    }
}

/// Roll and pitch rate and angle gains designed from each axis's fit (`COPTER_LOOP_SHAPING`)
/// meet the step spec (`spec::misses`, ArduPilot's and PX4's autotune targets) on the plant they
/// were fitted on: every copter airframe, clean and with 10 and 25 ms of delay, from the low
/// start.
#[test]
fn designed_gains_meet_the_spec() {
    let airframes = [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
        ("racer", airframes::racer()),
    ];
    for (name, airframe) in airframes {
        for (set, _) in SETS {
            let row = format!("{name} {set:?}");
            let plant = Plant {
                airframe: starting(airframe, Start::Low),
                wind: Wind::default(),
                perturbations: perturbations(set, &airframe),
            };
            let mut config = plant.airframe.stabilizer;
            let loops = [
                (TunedLoop::RollRate, TunedLoop::Roll),
                (TunedLoop::PitchRate, TunedLoop::Pitch),
            ];
            for (rate_loop, angle_loop) in loops {
                let model = identified(&plant, rate_loop);
                let start = rate_loop.gains(&config);
                let gyro_hz = config.gyro_cutoff_hz;
                let rate = design_rate(
                    &model,
                    start,
                    GyroFilters::low_pass(gyro_hz),
                    &COPTER_LOOP_SHAPING,
                )
                .expect("rate gains");
                *rate_loop.gains_mut(&mut config) = rate;
                let angle = angle_loop.gains(&config);
                let p = design_angle_p(
                    &model,
                    rate,
                    angle,
                    GyroFilters::low_pass(gyro_hz),
                    &COPTER_LOOP_SHAPING,
                )
                .expect("angle P");
                angle_loop.gains_mut(&mut config).p = p;
            }
            let tuned = Plant {
                airframe: Airframe {
                    stabilizer: config,
                    ..airframe
                },
                ..plant
            };
            let misses = spec::misses(&tuned, &spec::TUNED);
            check!(misses.is_empty(), "{row}: {misses:#?}");
        }
    }
}

/// Most a fit on a slow host may lower the designed rate P, as a share of the nominal design:
/// with the physics 12% ahead of the ticks it came out 5–13% lower (#524), so 15% leaves a
/// point or two.
const SLOW_HOST_P_SHARE: f32 = 0.15;

/// Physics per tick on RealFlight at about 223 Hz under the 250 Hz loop (#524).
const SLOW_HOST_TIME_SCALE: f32 = 1.12;

/// A fit on a host ticking slower than the loop's rate (every tick still passes the nominal
/// `dt`) designs a rate P no higher than on a host that keeps time, and at most
/// [`SLOW_HOST_P_SHARE`] lower: a slow host only makes the design cautious, so autotune fits
/// through it (#524). Every copter airframe, clean and with 10 ms of delay.
#[test]
fn a_fit_on_a_slow_host_designs_a_cautious_rate_p() {
    let airframes = [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
        ("racer", airframes::racer()),
    ];
    for (name, airframe) in airframes {
        for set in [Set::Clean, Set::Delay10Ms] {
            let row = format!("{name} {set:?}");
            let plant = Plant {
                airframe: starting(airframe, Start::Low),
                wind: Wind::default(),
                perturbations: perturbations(set, &airframe),
            };
            let slow = Plant {
                perturbations: Perturbations {
                    time_scale: SLOW_HOST_TIME_SCALE,
                    ..plant.perturbations
                },
                ..plant
            };
            let start = TunedLoop::RollRate.gains(&plant.airframe.stabilizer);
            let designed_p = |plant: &Plant| {
                let model = identified(plant, TunedLoop::RollRate);
                design_rate(
                    &model,
                    start,
                    plant.airframe.stabilizer.gyro_filters_on(0),
                    &COPTER_LOOP_SHAPING,
                )
                .expect("rate gains")
                .p
            };
            let (nominal_p, slow_p) = (designed_p(&plant), designed_p(&slow));
            check!(slow_p <= nominal_p, "{row}: {slow_p} against {nominal_p}");
            check!(
                slow_p >= (1.0 - SLOW_HOST_P_SHARE) * nominal_p,
                "{row}: {slow_p} against {nominal_p}"
            );
        }
    }
}
