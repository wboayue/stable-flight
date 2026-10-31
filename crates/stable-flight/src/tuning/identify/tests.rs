use assert2::{assert, check};

use super::*;
use crate::tuning::plant::{Plant, PlantParams};
use crate::tuning::{LoopSignals, Measurement, Probe, ProbeConfig, ProbeStatus};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// A 0.07 square wave for 3 s, washed out as the copter's.
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
/// loop: (gain rad/s² per unit moment, lag s, dead time s) → delay ticks. The plant's moment
/// takes effect the dead time plus 1½ ticks on (the probe's one tick and half of the
/// zero-order hold's), and the fit trades delay against lag between near-tied fits, so it holds
/// what the design's phase reads: the fitted lag plus delay within 10% of the plant's two lags
/// plus that time (measured within 1.8%), the delay within a tick of it, and the gain within
/// 10%.
#[test]
fn recovers_lag_delay_and_gain() {
    let rows = [
        ((20.0, 0.02, 0.0), 1.5),
        ((40.0, 0.03, 0.0), 1.5),
        ((10.0, 0.05, 0.0), 1.5),
        ((5.0, 0.2, 0.0), 1.5),
        ((20.0, 0.03, 0.008), 3.5),
        ((20.0, 0.03, 0.02), 6.5),
    ];
    for ((gain, lag_s, delay_s), delay_ticks) in rows {
        let row = format!("gain {gain}, lag {lag_s} s, delay {delay_s} s");
        let fitted = model(identify(
            single_lag(gain, lag_s, delay_s),
            IdentifyProbe::new(CONFIG),
        ));
        let [now, next, _] = fitted.moment_weights.map(f32::abs);
        let fitted_ticks = fitted.delay_ticks as f32 + next / (now + next);
        check!(
            (fitted_ticks - delay_ticks).abs() <= 1.0,
            "{row}: {fitted:?}"
        );
        let fitted_s = fitted.lag_s().unwrap_or(f32::NAN) + fitted_ticks * DT;
        let plant_s = lag_s + 0.001 + delay_ticks * DT;
        check!(
            (fitted_s - plant_s).abs() <= 0.1 * plant_s,
            "{row}: lag and delay {fitted_s} s against {plant_s} s"
        );
        let accel = fitted.accel_radps2().unwrap_or(f32::NAN);
        check!((accel - gain).abs() <= 0.1 * gain, "{row}: accel {accel}");
        check!((fitted.dt_s - DT).abs() < 1e-6);
    }
}

/// Repeatable gyro noise for the tests: white, uniform in ±`amplitude` rad/s, from a fixed
/// linear congruential sequence keyed by the tick.
fn gyro_noise(tick: usize, amplitude: f32) -> f32 {
    let state = (tick as u64)
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    let unit = (state >> 40) as f32 / (1u64 << 24) as f32;
    amplitude * (2.0 * unit - 1.0)
}

/// Through white gyro noise of about the sim's (0.035 rad/s uniform, 0.02 RMS), the fit reads the
/// plant's lag plus delay within 20% (measured 11% long, 2% on clean signals), where the
/// recorded-moment fit, with the noisy rate among its terms, reads the lag as nothing (#551).
#[test]
fn reads_the_lag_through_gyro_noise() {
    let params = single_lag(20.0, 0.03, 0.0);
    let probe = identify_with(params, IdentifyProbe::new(CONFIG), |tick, rate| {
        rate + gyro_noise(tick, 0.035)
    });
    let fitted = model(probe.status());
    let [now, next, _] = fitted.moment_weights.map(f32::abs);
    let fitted_s =
        fitted.lag_s().unwrap_or(f32::NAN) + (fitted.delay_ticks as f32 + next / (now + next)) * DT;
    let plant_s = 0.03 + 0.001 + 1.5 * DT;
    check!(
        (fitted_s - plant_s).abs() <= 0.2 * plant_s,
        "lag and delay {fitted_s} s against {plant_s} s: {fitted:?}"
    );
    let recorded = fit_recorded(&probe.window).expect("recorded fit");
    check!(recorded.lag_s() < Some(0.01), "{recorded:?}");
}

/// A noisy window is fitted on the explained moments; a quiet one, or one whose moments
/// mostly sit at the rate loop's limit, on the recorded moments: (case, rate P of the closed
/// loop, gyro noise amplitude in rad/s, whether the explained fit is taken).
#[test]
fn a_noisy_unsaturated_window_takes_the_explained_fit() {
    let params = single_lag(20.0, 0.03, 0.0);
    let rows = [
        ("quiet", RATE_P, 0.0, false),
        ("noisy", RATE_P, 0.035, true),
        ("noisy and saturated", 40.0, 0.035, false),
    ];
    for (case, rate_p, noise, explained) in rows {
        let mut plant = Plant::new(params, DT);
        let (mut angle_rad, mut last_moment) = (0.0f32, 0.0f32);
        let mut probe = IdentifyProbe::new(CONFIG);
        let mut tick = 0;
        while matches!(probe.status(), IdentifyStatus::Running { .. }) {
            let rate_radps = plant.measure() + gyro_noise(tick, noise);
            tick += 1;
            let offset_rad = probe.step(DT, rate_radps, last_moment);
            let rate_setpoint_radps = ANGLE_P * (offset_rad - angle_rad);
            last_moment = (rate_p * (rate_setpoint_radps - rate_radps)).clamp(-1.0, 1.0);
            plant.step(last_moment);
            angle_rad += plant.output() * DT;
        }
        let window = &probe.window;
        let expected = if explained {
            fit_explained(window)
        } else {
            fit_recorded(window)
        };
        check!(fit_window(window) == expected, "{case}");
    }
}

/// Rate noise is the RMS of the rate's second difference: (case, rates, rad/s).
#[test]
fn rate_noise_reads_the_second_difference() {
    let rows: [(&str, &[f32], f32); 4] = [
        ("too short", &[1.0, 2.0], 0.0),
        ("a steady rate", &[0.5; 10], 0.0),
        ("a steady acceleration", &[0.0, 0.1, 0.2, 0.3, 0.4], 0.0),
        // Each second difference is ±0.4.
        ("alternating ±0.1", &[0.1, -0.1, 0.1, -0.1, 0.1], 0.4),
    ];
    for (case, rates, expected) in rows {
        check!((rate_noise_radps(rates) - expected).abs() < 1e-6, "{case}");
    }
}

/// A window longer than [`WINDOW_CAPACITY`] ticks records its first ones and still fits them.
#[test]
fn a_long_window_fits_its_first_ticks() {
    let config = IdentifyConfig {
        duration_s: 6.0,
        ..CONFIG
    };
    let probe = identify_with(
        single_lag(20.0, 0.03, 0.0),
        IdentifyProbe::new(config),
        |_, rate| rate,
    );
    check!(probe.window.len == WINDOW_CAPACITY);
    check!(
        matches!(probe.status(), IdentifyStatus::Done(_)),
        "{:?}",
        probe.status()
    );
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

/// A non-finite rate or moment is recorded as the last finite one, and the recorded-moment fit
/// skips it: the fit still finishes and reads the clean fit's delay, and its lag within 1%.
#[test]
fn holds_a_non_finite_tick_at_the_last_value() {
    let params = single_lag(20.0, 0.02, 0.0);
    let clean = model(identify(params, IdentifyProbe::new(CONFIG)));
    let dropped = model(
        identify_with(params, IdentifyProbe::new(CONFIG), |tick, rate| {
            if tick == 300 { f32::NAN } else { rate }
        })
        .status(),
    );
    check!(dropped.delay_ticks == clean.delay_ticks, "{dropped:?}");
    let [dropped_s, clean_s] = [dropped, clean].map(|model| model.lag_s().unwrap_or(f32::NAN));
    check!(
        (dropped_s - clean_s).abs() <= 0.01 * clean_s,
        "{dropped:?} against {clean:?}"
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

/// The windows of a recorded fixture: per line `window,rate,last moment`, `#` lines skipped.
fn realflight_windows<const N: usize>(fixture: &str) -> [Vec<(f32, f32)>; N] {
    let mut windows: [Vec<(f32, f32)>; N] = core::array::from_fn(|_| Vec::new());
    for line in fixture.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = line.split(',').collect();
        let window: usize = fields[0].parse().expect("window");
        let rate_radps = fields[1].parse().expect("rate");
        let last_moment = fields[2].parse().expect("moment");
        windows[window].push((rate_radps, last_moment));
    }
    windows
}

/// The probe's fit to a recorded window, and the window's rate noise.
fn fit_recorded_window(window: &[(f32, f32)]) -> (RateModel, f32) {
    let mut probe = IdentifyProbe::new(crate::tuning::autotune::COPTER_IDENTIFY);
    for &(rate_radps, last_moment) in window {
        probe.step(DT, rate_radps, last_moment);
    }
    let noise = rate_noise_radps(&probe.window.rates_radps[..probe.window.len]);
    (model(probe.status()), noise)
}

/// Rate P designed from `fitted` at the copter's loop shaping and the 40 Hz gyro filter.
fn designed_p(fitted: &RateModel, start: crate::controllers::Gains) -> Option<f32> {
    use crate::tuning::autotune::COPTER_LOOP_SHAPING;
    crate::tuning::design_rate(fitted, start, 40.0, &COPTER_LOOP_SHAPING)
        .ok()
        .map(|gains| gains.p)
}

/// RealFlight's QuadCopterX identify windows (PR #585's flight check) are quiet, so they take
/// the recorded-moment fit, and each designs rate P within the 0.5–1.0 that fit designs on
/// that airframe (#551).
#[test]
fn realflight_quadcopterx_windows_design_rate_p_in_range() {
    let start = crate::controllers::Gains {
        p: 0.5,
        i: 0.1,
        d: 0.004,
    };
    let windows = realflight_windows::<4>(include_str!("realflight_quadcopterx_windows.csv"));
    for (index, window) in windows.iter().enumerate() {
        let (fitted, noise) = fit_recorded_window(window);
        check!(noise < NOISY_RATE_RADPS, "window {index}: noise {noise}");
        let p = designed_p(&fitted, start);
        check!(
            p.is_some_and(|p| (0.5..=1.0).contains(&p)),
            "window {index}: P {p:?} from {fitted:?}"
        );
    }
}

/// RealFlight's eXplora roll and pitch rate windows (#539's run 3) are quiet, and design the
/// rate P RealFlight's autotune designed from them, within 5% (the replay reproduced it to two
/// places): the two-stage fit read the roll's 36–40 ms delay as 136 ms of lag and designed P
/// 3.92, and the pitch 3.36 (#551).
#[test]
fn realflight_explora_windows_design_realflights_rate_p() {
    let start = crate::controllers::Gains {
        p: 0.4,
        i: 0.2,
        d: 0.03,
    };
    // (axis, RealFlight's designed rate P)
    let designs = [("roll", 0.94), ("pitch", 1.69)];
    let windows = realflight_windows::<2>(include_str!("realflight_explora_windows.csv"));
    for ((axis, realflight_p), window) in designs.into_iter().zip(&windows) {
        let (fitted, noise) = fit_recorded_window(window);
        check!(noise < NOISY_RATE_RADPS, "{axis}: noise {noise}");
        let p = designed_p(&fitted, start);
        check!(
            p.is_some_and(|p| (p - realflight_p).abs() <= 0.05 * realflight_p),
            "{axis}: P {p:?} against {realflight_p} from {fitted:?}"
        );
    }
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
    let mut probe = Probe::new(ProbeConfig::Identify(CONFIG));
    let signals = LoopSignals {
        setpoint: 0.1,
        ..LoopSignals::default()
    };
    let setpoint = probe.step(DT, &signals);
    check!(setpoint > 0.1 && setpoint <= 0.1 + CONFIG.amplitude);

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

/// A pole in (0, 1) is a lag; one at or below 0 a lag under a tick, so 0 (#563); one at or
/// past 1 none: (pole, lag).
#[test]
fn lag_needs_a_pole_below_one() {
    let rows = [
        (0.9, Some(-DT / 0.9f32.ln())),
        (0.0, Some(0.0)),
        (-0.2, Some(0.0)),
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
        // 0.1 of moment per tick, over (1 − p)·dt with a pole at or below 0 taken as 0.
        let accel = lag_s.map(|_| 0.1 / ((1.0 - pole.max(0.0)) * DT));
        check!(model.accel_radps2() == accel, "pole {pole}");
    }
}
