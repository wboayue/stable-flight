use std::collections::VecDeque;

use approx::assert_relative_eq;
use assert2::{assert, check};

use super::*;
use crate::tuning::{LoopSignals, Measurement, Probe, ProbeAbort, ProbeConfig, ProbeStatus};
use nalgebra::Complex;

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// A 0.3 rad/s square wave for 6 s, washed out over 0.2 s.
const CONFIG: IdentifyConfig = IdentifyConfig {
    amplitude: 0.3,
    duration_s: 6.0,
    washout_tc_s: 0.2,
    noisy: None,
};

/// The rate loop the tests close around the wing: moment per rad/s of rate error.
const RATE_P: f32 = 0.2;

/// A wing axis as [`WingModel`] describes it, stepped at [`DT`]: `α′ = rate − path_turn·α`.
struct Axis {
    model: WingModel,
    rate_radps: f32,
    alpha_rad: f32,
    surface: f32,
    queue: VecDeque<f32>,
}

impl Axis {
    fn new(model: WingModel) -> Self {
        let delay_ticks = (model.delay_s / DT).round() as usize;
        Self {
            model,
            rate_radps: 0.0,
            alpha_rad: 0.0,
            surface: 0.0,
            queue: VecDeque::from(vec![0.0; delay_ticks]),
        }
    }

    fn step(&mut self, moment: f32) {
        let m = self.model;
        self.queue.push_back(moment);
        let felt = self.queue.pop_front().unwrap_or(moment);
        let share = DT / (DT + m.lag_s);
        self.surface += share * (felt - self.surface);
        let accel = -m.stiffness_per_s2 * self.alpha_rad - m.damping_per_s * self.rate_radps
            + m.control_radps2 * self.surface;
        self.alpha_rad += (self.rate_radps - m.path_turn_per_s * self.alpha_rad) * DT;
        self.rate_radps += accel * DT;
    }
}

/// Runs a probe on `model`'s axis inside a rate loop, `alpha` reporting the angle of attack
/// (else 0, as on a roll axis); the finished probe's status.
fn identify(model: WingModel, alpha: bool) -> WingStatus {
    let mut probe = WingIdentifyProbe::new(CONFIG);
    let mut axis = Axis::new(model);
    let mut last_moment = 0.0f32;
    while matches!(probe.status(), WingStatus::Running { .. }) {
        let alpha_rad = if alpha { axis.alpha_rad } else { 0.0 };
        let offset = probe.step(DT, axis.rate_radps, last_moment, alpha_rad);
        last_moment = (RATE_P * (offset - axis.rate_radps)).clamp(-1.0, 1.0);
        axis.step(last_moment);
    }
    probe.status()
}

fn fitted(status: WingStatus) -> WingModel {
    match status {
        WingStatus::Done(model) => model,
        other => panic!("expected Done, got {other:?}"),
    }
}

/// The sweep's fitted airplane pitch axis (#533): stiff, damped, quick surfaces.
const PITCH: WingModel = WingModel {
    stiffness_per_s2: 15.0,
    damping_per_s: 7.0,
    control_radps2: 13.0,
    lag_s: 0.0,
    delay_s: 0.0,
    path_turn_per_s: 15.0,
    trim_moment: 0.0,
};

/// A roll axis: no stiffness, no path turn.
const ROLL: WingModel = WingModel {
    stiffness_per_s2: 0.0,
    damping_per_s: 8.0,
    control_radps2: 40.0,
    lag_s: 0.0,
    delay_s: 0.0,
    path_turn_per_s: 0.0,
    trim_moment: 0.0,
};

/// The body rate per unit moment `model` answers with at `w_radps`: the moment through the
/// surfaces' lag and delay, against the damping and, through the angle of attack
/// (`α = rate / (jω + path_turn)`), the stiffness.
fn response(model: &WingModel, w_radps: f32) -> Complex<f32> {
    let jw = Complex::new(0.0, w_radps);
    let late_rad = -w_radps * model.delay_s;
    let surface = Complex::new(late_rad.cos(), late_rad.sin()) / (jw * model.lag_s + 1.0);
    let stiffness = if model.path_turn_per_s == 0.0 && model.stiffness_per_s2 == 0.0 {
        Complex::new(0.0, 0.0)
    } else {
        Complex::new(model.stiffness_per_s2, 0.0) / (jw + model.path_turn_per_s)
    };
    surface * model.control_radps2 / (jw + model.damping_per_s + stiffness)
}

#[test]
fn recovers_a_wing_axis() {
    // (row, model, angle of attack reported)
    let rows = [
        ("roll", ROLL, false),
        (
            "roll, lagged",
            WingModel {
                lag_s: 0.06,
                ..ROLL
            },
            false,
        ),
        ("pitch", PITCH, true),
        (
            "pitch, lagged and late",
            WingModel {
                lag_s: 0.04,
                delay_s: 0.016,
                ..PITCH
            },
            true,
        ),
        (
            "the Alia 250's slow pitch",
            WingModel {
                stiffness_per_s2: 4.5,
                damping_per_s: 0.32,
                control_radps2: 0.61,
                path_turn_per_s: 0.56,
                ..PITCH
            },
            true,
        ),
    ];
    // A stiff wing's angle of attack follows its rate, so stiffness and damping trade off in
    // the fit with little change in how the axis answers: the response is what's checked,
    // over the excitation's 0.2–1 s half-periods and up to the loops' crossover.
    for (row, truth, alpha) in rows {
        let model = fitted(identify(truth, alpha));
        for w_radps in [1.0, 3.0, 10.0, 20.0] {
            let (fit, true_response) = (response(&model, w_radps), response(&truth, w_radps));
            let ratio = fit / true_response;
            let gain = ratio.norm_sqr().sqrt();
            let phase_deg = ratio.im.atan2(ratio.re).to_degrees();
            check!(
                (gain - 1.0).abs() < 0.1,
                "{row} at {w_radps} rad/s: {model:?}"
            );
            check!(
                phase_deg.abs() < 10.0,
                "{row} at {w_radps} rad/s: {model:?}"
            );
        }
    }
}

/// The X-Plane Alia 250's roll windows, three plane autotunes at 45 m/s (#694): per window,
/// each tick's roll rate and the moment commanded on the tick before.
fn xplane_alia_roll_windows() -> [Vec<(f32, f32)>; 3] {
    let mut windows: [Vec<(f32, f32)>; 3] = core::array::from_fn(|_| Vec::new());
    let fixture = include_str!("xplane_alia_roll_windows.csv");
    for line in fixture.lines().filter(|line| !line.starts_with('#')) {
        let fields: Vec<&str> = line.split(',').collect();
        let window: usize = fields[0].parse().expect("window");
        let rate_radps = fields[1].parse().expect("rate");
        let last_moment = fields[2].parse().expect("moment");
        windows[window].push((rate_radps, last_moment));
    }
    windows
}

/// The model a probe fits to the recorded `ticks`.
fn fit_recorded(ticks: &[(f32, f32)]) -> WingModel {
    let mut window = Window::new();
    for &(rate_radps, last_moment) in ticks {
        window.record(DT, rate_radps, last_moment, 0.0);
    }
    fit(&window).expect("a fit")
}

/// Most a roll fit's feedforward may differ over the X-Plane Alia's three windows, as a ratio:
/// they fit 2.80 to 3.17 (1.13), and on the rate's change alone 2.09 to 3.12 (1.49) (#694).
const MAX_ROLL_FEEDFORWARD_SPREAD: f32 = 1.2;

#[test]
fn the_xplane_alias_roll_fits_one_feedforward_over_three_runs() {
    let feedforwards = xplane_alia_roll_windows().map(|ticks| fit_recorded(&ticks).feedforward());
    check!(
        feedforwards.iter().all(|ff| ff.is_finite()),
        "{feedforwards:?}"
    );
    let low = feedforwards.iter().copied().fold(f32::INFINITY, f32::min);
    let high = feedforwards.iter().copied().fold(0.0, f32::max);
    check!(
        high / low <= MAX_ROLL_FEEDFORWARD_SPREAD,
        "{feedforwards:?}"
    );
}

/// The refinement's sample, in s.
const REFINE_SAMPLE_S: f32 = DT * DECIMATION as f32;

/// The roll terms `refine_case`'s window is simulated from.
const TRUE_ROLL: RollTerms = RollTerms {
    damping_per_s: 8.0,
    control_radps2: 3.0,
    constant_radps2: 0.1,
    start_radps: 0.0,
};

/// Roll terms whose simulation runs away: damping −225/s multiplies the rate by 19 each
/// sample.
const RUNAWAY_ROLL: RollTerms = RollTerms {
    damping_per_s: -225.0,
    ..TRUE_ROLL
};

/// A full window whose rate `TRUE_ROLL` simulates exactly from a ±`amplitude` surface square
/// wave of 1 s, with no delay, and the surface.
fn refine_case(amplitude: f32) -> (Window, [f32; WINDOW_CAPACITY]) {
    let mut window = Window::new();
    let mut surface = [0.0f32; WINDOW_CAPACITY];
    let h = f64::from(REFINE_SAMPLE_S);
    let half_damping = 0.5 * h * TRUE_ROLL.damping_per_s;
    let mut rate = TRUE_ROLL.start_radps;
    for k in 0..WINDOW_CAPACITY {
        let half_period = (k as f32 * REFINE_SAMPLE_S / 0.5) as usize;
        surface[k] = if half_period.is_multiple_of(2) {
            amplitude
        } else {
            -amplitude
        };
        if k > 0 {
            let push = TRUE_ROLL.control_radps2 * f64::from(surface[k]) + TRUE_ROLL.constant_radps2;
            rate = ((1.0 - half_damping) * rate + h * push) / (1.0 + half_damping);
        }
        window.rates_radps[k] = rate as f32;
        window.moments[k] = surface[k];
    }
    window.len = WINDOW_CAPACITY;
    (window, surface)
}

/// A least squares fit holding `terms` (stiffness 0).
fn fit_holding(terms: &RollTerms) -> Fit {
    Fit {
        terms: SVector::<f64, TERMS>::new(
            0.0,
            -terms.damping_per_s,
            terms.control_radps2,
            terms.constant_radps2,
        ),
        residual: 0.0,
    }
}

#[test]
fn refines_roll_terms_or_keeps_the_start() {
    let biased = RollTerms {
        damping_per_s: 4.0,
        control_radps2: 1.5,
        ..TRUE_ROLL
    };
    // (name, surface amplitude, start, expected terms)
    let rows = [
        (
            "a biased start recovers the simulated terms",
            0.3,
            biased,
            TRUE_ROLL,
        ),
        // No surface moves the rate: no step lowers the residual.
        ("an unexcited window keeps its start", 0.0, biased, biased),
        // Every simulation runs away, so no step lowers the residual and the damping passes
        // `MAX_REFINE_DAMPING`.
        ("a runaway start is kept", 0.3, RUNAWAY_ROLL, RUNAWAY_ROLL),
    ];
    for (name, amplitude, start, expected) in rows {
        let (window, surface) = refine_case(amplitude);
        let refined =
            refine_roll_terms(&window, &surface, 0, REFINE_SAMPLE_S, &fit_holding(&start));
        for (got, want) in [
            (refined.damping_per_s, expected.damping_per_s),
            (refined.control_radps2, expected.control_radps2),
        ] {
            check!(
                (got - want).abs() <= 1e-3 * want.abs().max(1.0),
                "{name}: {refined:?}"
            );
        }
    }
}

#[test]
fn a_runaway_simulation_is_infinitely_off() {
    let (window, surface) = refine_case(0.3);
    let residual = simulate(&window, &surface, 0, REFINE_SAMPLE_S, &RUNAWAY_ROLL, None);
    check!(residual == f64::INFINITY);
    let exact = simulate(&window, &surface, 0, REFINE_SAMPLE_S, &TRUE_ROLL, None);
    check!(exact < 1e-6);
}

#[test]
fn a_roll_axis_fits_no_stiffness() {
    let model = fitted(identify(ROLL, false));
    check!(model.stiffness_per_s2 == 0.0);
    check!(model.path_turn_per_s == 0.0);
}

#[test]
fn a_motionless_axis_is_unexcited() {
    let mut probe = WingIdentifyProbe::new(CONFIG);
    while matches!(probe.status(), WingStatus::Running { .. }) {
        probe.step(DT, 0.0, 0.0, 0.0);
    }
    assert!(probe.status() == WingStatus::Aborted(IdentifyAbort::Unexcited));
}

#[test]
fn holds_a_non_finite_tick_at_the_last_value() {
    let mut window = Window::new();
    let ticks = [
        (0.1, 0.2, 0.3),
        (0.1, 0.2, 0.3),
        (f32::NAN, f32::INFINITY, f32::NAN),
        (f32::NAN, 0.6, 0.7),
    ];
    for (rate, moment, alpha) in ticks {
        window.record(DT, rate, moment, alpha);
    }
    // Every second tick: the second's rate and angle of attack, then the fourth's with its
    // NaN rate held at the last finite one; each sample's moment the mean of its two ticks'.
    check!(window.len == 2);
    check!(window.rates_radps[..2] == [0.1, 0.1]);
    check!(window.moments[..2] == [0.2, 0.4]);
    check!(window.alphas_rad[..2] == [0.3, 0.7]);
    assert_relative_eq!(window.sample_s(), 2.0 * DT);
}

#[test]
fn a_long_window_fits_its_first_samples() {
    let mut window = Window::new();
    for _ in 0..3 * WINDOW_CAPACITY {
        window.record(DT, 0.1, 0.2, 0.0);
    }
    check!(window.len == WINDOW_CAPACITY);
}

#[test]
fn a_finished_probe_offsets_nothing() {
    let config = IdentifyConfig {
        duration_s: 0.01,
        ..CONFIG
    };
    let mut probe = WingIdentifyProbe::new(config);
    for _ in 0..5 {
        probe.step(DT, 0.0, 0.0, 0.0);
    }
    check!(!matches!(probe.status(), WingStatus::Running { .. }));
    check!(probe.step(DT, 0.0, 0.0, 0.0) == 0.0);
}

#[test]
fn a_probe_identifies_a_wing_on_its_rate_setpoint() {
    let mut probe = Probe::new(ProbeConfig::IdentifyWing(CONFIG));
    let signals = LoopSignals {
        setpoint: 0.1,
        ..LoopSignals::default()
    };
    let setpoint = probe.step(DT, &signals);
    check!(setpoint > 0.1 && setpoint <= 0.1 + CONFIG.amplitude);
    check!(matches!(probe.status(), ProbeStatus::Running(_)));

    // A `Probe` (a copter's host) flies it as an identification; the plane hosts the wing fit.
    let status = ProbeStatus::from(identify(PITCH, true));
    check!(matches!(status, ProbeStatus::Done(Measurement::Wing(_))));
    let aborted = ProbeStatus::from(WingStatus::Aborted(IdentifyAbort::Unexcited));
    check!(matches!(
        aborted,
        ProbeStatus::Aborted(ProbeAbort::Identify(_))
    ));
}
