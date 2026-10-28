use assert2::{assert, check};

use super::*;
use crate::tuning::plant::{Plant, PlantParams};
use crate::tuning::{Injection, LoopSignals, Measurement, Probe, ProbeConfig, ProbeStatus};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// A 4° square wave for 3 s, as the copter's.
const CONFIG: IdentifyConfig = IdentifyConfig {
    amplitude: 0.07,
    duration_s: 3.0,
    washout_tc_s: 0.2,
};

/// Angle P (rate per angle) and rate P (moment per rate) of the loops the tests close around
/// the plant: stable on every row's plant.
const ANGLE_P: f32 = 4.0;
const RATE_P: f32 = 0.3;

/// Runs `probe` on `params`'s axis inside an angle loop and a rate loop, as a copter host
/// flies it: the probe's offset on the angle setpoint, the axis's rate (through `corrupt`,
/// given the tick) and last moment back. Returns the finished probe.
fn identify_with(
    params: PlantParams,
    mut probe: IdentifyProbe,
    corrupt: impl Fn(usize, f32) -> f32,
) -> IdentifyProbe {
    let mut plant = Plant::new(params, DT);
    let (mut angle_rad, mut last_moment) = (0.0f32, 0.0f32);
    let mut tick = 0;
    while matches!(probe.status(), IdentifyStatus::Running { .. }) {
        let rate_radps = plant.measure();
        let offset_rad = probe.step(DT, corrupt(tick, rate_radps), last_moment);
        let rate_setpoint_radps = ANGLE_P * (offset_rad - angle_rad);
        last_moment = (RATE_P * (rate_setpoint_radps - rate_radps)).clamp(-1.0, 1.0);
        plant.step(last_moment);
        angle_rad += plant.output() * DT;
        tick += 1;
    }
    probe
}

/// [`identify_with`] on clean signals: the probe's final status.
fn identify(params: PlantParams, probe: IdentifyProbe) -> IdentifyStatus {
    identify_with(params, probe, |_, rate_radps| rate_radps).status()
}

fn model(status: IdentifyStatus) -> RateModel {
    match status {
        IdentifyStatus::Done(model) => model,
        other => panic!("expected Done, got {other:?}"),
    }
}

/// A plant with one lag: the second sits at 1 ms, under a tick.
fn single_lag(gain: f32, lag_s: f32, delay_s: f32) -> PlantParams {
    PlantParams {
        gain,
        lag1_s: lag_s,
        lag2_s: 0.001,
        delay_s,
        ..PlantParams::rate_axis()
    }
}

/// The fit recovers each plant's lag and delay, and its gain, from 3 s of excitation in closed
/// loop: (gain rad/s² per unit moment, lag s, dead time s) → delay ticks, the dead time's ticks
/// plus the hold's one. The lag fits within 5% (the second lag adds about 1 ms), the gain
/// within 10%.
#[test]
fn recovers_lag_delay_and_gain() {
    let rows = [
        ((20.0, 0.02, 0.0), 1),
        ((40.0, 0.03, 0.0), 1),
        ((10.0, 0.05, 0.0), 1),
        ((20.0, 0.03, 0.008), 3),
        ((20.0, 0.03, 0.02), 6),
    ];
    for ((gain, lag_s, delay_s), delay_ticks) in rows {
        let row = format!("gain {gain}, lag {lag_s} s, delay {delay_s} s");
        let fitted = model(identify(
            single_lag(gain, lag_s, delay_s),
            IdentifyProbe::new(CONFIG),
        ));
        check!(fitted.delay_ticks == delay_ticks, "{row}: {fitted:?}");
        let fitted_lag_s = fitted.lag_s().unwrap_or(f32::NAN);
        check!(
            (fitted_lag_s - lag_s).abs() <= 0.05 * lag_s,
            "{row}: lag {fitted_lag_s}"
        );
        let accel = fitted.accel_radps2().unwrap_or(f32::NAN);
        check!((accel - gain).abs() <= 0.1 * gain, "{row}: accel {accel}");
        check!((fitted.dt_s - DT).abs() < 1e-6);
    }
}

/// The square wave's edges fall at PX4's half-periods (1.0, 0.9, 0.8 s, …), and each edge
/// washes out over the time constant: one time constant after the first rising edge, the
/// offset is about `amplitude·e⁻¹`.
#[test]
fn excites_with_a_washed_out_shrinking_square_wave() {
    let config = IdentifyConfig {
        duration_s: 10.0,
        ..CONFIG
    };
    let mut probe = IdentifyProbe::new(config);
    let offsets: Vec<f32> = (0..2500).map(|_| probe.step(DT, 0.0, 0.0)).collect();
    // Edges: where the offset jumps by most of the amplitude in one tick.
    let edges_s: Vec<f32> = offsets
        .windows(2)
        .enumerate()
        .filter(|(_, pair)| (pair[1] - pair[0]).abs() > 0.5 * config.amplitude)
        .map(|(tick, _)| (tick + 1) as f32 * DT)
        .collect();
    for (edge_s, expected_s) in edges_s.iter().zip([1.0, 1.9, 2.7, 3.4, 4.0]) {
        check!((edge_s - expected_s).abs() <= 1.5 * DT, "{edges_s:?}");
    }
    let one_tc = offsets[(config.washout_tc_s / DT) as usize - 1];
    let expected = config.amplitude * (-1.0f32).exp();
    check!(
        (one_tc - expected).abs() <= 0.05 * config.amplitude,
        "{one_tc}"
    );
}

/// A non-finite rate or moment drops that tick and the history refilling after it: the sums
/// hold [`HISTORY`] fewer ticks, and the fit still finishes.
#[test]
fn drops_a_non_finite_tick_and_refills_its_history() {
    let params = single_lag(20.0, 0.02, 0.0);
    let clean = identify_with(params, IdentifyProbe::new(CONFIG), |_, rate| rate);
    let dropped = identify_with(params, IdentifyProbe::new(CONFIG), |tick, rate| {
        if tick == 300 { f32::NAN } else { rate }
    });
    check!(clean.fits.rows - dropped.fits.rows == HISTORY);
    check!(
        matches!(dropped.status(), IdentifyStatus::Done(_)),
        "{:?}",
        dropped.status()
    );
}

/// The delay is the shortest whose residual is within 1% of the least, so rounding doesn't pick
/// between near-equal delays: (residuals per delay from 1, delay picked).
#[test]
fn picks_the_shortest_of_near_tied_delays() {
    let rows: [(&[Option<f32>], Option<usize>); 6] = [
        (&[Some(3.0), Some(1.0), Some(2.0)], Some(2)),
        (&[Some(1.005), Some(1.0)], Some(1)),
        (&[Some(1.02), Some(1.0)], Some(2)),
        (&[None, Some(0.0), Some(0.0)], Some(2)),
        (&[None, None], None),
        (&[], None),
    ];
    for (residuals, delay) in rows {
        check!(shortest_tied(residuals) == delay, "{residuals:?}");
    }
}

/// A delay's moment effect and residual, as `delay_to_fit` takes them.
type DelayFit = Option<(f32, f32)>;

/// The fit takes the best of the delays whose moment pushes the axis the way it was pushed, and
/// the best of all only when none does: (each delay's moment effect and residual from 1, delay
/// picked).
#[test]
fn fits_a_delay_whose_moment_pushes_the_right_way() {
    let rows: [(&str, &[DelayFit], Option<usize>); 4] = [
        (
            "all push: least residual",
            &[Some((1.0, 2.0)), Some((1.0, 1.0))],
            Some(2),
        ),
        (
            "the least pushes back: the next",
            &[Some((1.0, 1.5)), Some((-1.0, 1.0))],
            Some(1),
        ),
        (
            "none pushes: least of all",
            &[Some((-1.0, 2.0)), Some((-1.0, 1.0))],
            Some(2),
        ),
        ("none solved", &[None, None], None),
    ];
    for (row, fits, delay) in rows {
        check!(delay_to_fit(fits) == delay, "{row}");
    }
}

/// A fit on recorded flight data skips the delays whose model pushes the axis the wrong way:
/// QuadCopterX's first roll fit in RealFlight run 33 (#518) fitted best at 5 ticks with a
/// negative moment effect (−1 rad/s², #529). Which pushing delay wins isn't pinned: the best
/// is 6 ticks (11 rad/s²), and 4 (40 rad/s²) ties with it at 1% by 0.03%, under `f32`
/// summation order.
#[test]
fn a_recorded_fit_reads_a_positive_acceleration() {
    let mut probe = IdentifyProbe::new(CONFIG);
    for line in include_str!("realflight_roll_fit.csv").lines() {
        if line.starts_with('#') {
            continue;
        }
        let (rate, moment) = line.split_once(',').expect("rate,moment");
        let rate_radps = rate.parse().expect("rate");
        let last_moment = moment.parse().expect("moment");
        probe.step(DT, rate_radps, last_moment);
    }
    let model = model(probe.status());
    check!(model.moment_effect() > 0.0, "{model:?}");
    let accel_radps2 = model.accel_radps2().unwrap_or(f32::NAN);
    check!(accel_radps2 > 0.0, "{model:?}: {accel_radps2} rad/s²");
}

/// An axis that never moves can't be fitted: no delay's sums solve.
#[test]
fn a_motionless_axis_is_unexcited() {
    let mut probe = IdentifyProbe::new(CONFIG);
    while matches!(probe.status(), IdentifyStatus::Running { .. }) {
        probe.step(DT, 0.0, 0.0);
    }
    check!(probe.status() == IdentifyStatus::Aborted(IdentifyAbort::Unexcited));
}

/// Once finished, the probe offsets nothing and keeps its result.
#[test]
fn a_finished_probe_offsets_nothing() {
    let mut probe = IdentifyProbe::new(CONFIG);
    while matches!(probe.status(), IdentifyStatus::Running { .. }) {
        probe.step(DT, 0.0, 0.0);
    }
    let status = probe.status();
    check!(probe.step(DT, 1.0, 1.0) == 0.0);
    check!(probe.status() == status);
}

/// Through [`Probe`], identification adds its offset to the loop's setpoint and reports a
/// [`Measurement::Identify`] once done.
#[test]
fn probe_injects_the_offset_on_the_setpoint() {
    let mut probe = Probe::new(ProbeConfig::Identify(CONFIG), 0.0);
    let signals = LoopSignals {
        setpoint: 0.1,
        ..LoopSignals::default()
    };
    let Injection::Setpoint(setpoint) = probe.step(DT, &signals) else {
        panic!("identification replaces the setpoint");
    };
    check!(setpoint > 0.1 && setpoint <= 0.1 + CONFIG.amplitude);
    check!(probe.trim() == None);

    let fitted = identify(single_lag(20.0, 0.02, 0.0), IdentifyProbe::new(CONFIG));
    let status = ProbeStatus::from(fitted);
    assert!(matches!(
        status,
        ProbeStatus::Done(Measurement::Identify(_))
    ));
}

/// Each field's requirement: (field changed, valid).
#[test]
fn validate_checks_each_field() {
    let rows = [
        (CONFIG, true),
        (
            IdentifyConfig {
                amplitude: 0.0,
                ..CONFIG
            },
            false,
        ),
        (
            IdentifyConfig {
                amplitude: -0.07,
                ..CONFIG
            },
            true,
        ),
        (
            IdentifyConfig {
                amplitude: f32::NAN,
                ..CONFIG
            },
            false,
        ),
        (
            IdentifyConfig {
                duration_s: 0.0,
                ..CONFIG
            },
            false,
        ),
        (
            IdentifyConfig {
                duration_s: f32::INFINITY,
                ..CONFIG
            },
            false,
        ),
        (
            IdentifyConfig {
                washout_tc_s: 0.0,
                ..CONFIG
            },
            false,
        ),
    ];
    for (config, valid) in rows {
        check!(config.validate().is_ok() == valid, "{config:?}");
    }
}

/// The lag exists only for a real pole in (0, 1): (pole, lag).
#[test]
fn lag_needs_a_pole_inside_the_unit_interval() {
    let rows = [
        (0.9, Some(-DT / 0.9f32.ln())),
        (0.0, None),
        (-0.3, None),
        (1.0, None),
        (1.2, None),
    ];
    for (pole, lag_s) in rows {
        let model = RateModel {
            rate_weights: [1.0 + pole, -pole],
            moment_weights: [0.1, 0.0, 0.0],
            delay_ticks: 1,
            dt_s: DT,
        };
        check!(model.lag_s() == lag_s, "pole {pole}");
        check!(
            model.accel_radps2().is_some() == lag_s.is_some(),
            "pole {pole}"
        );
    }
}
