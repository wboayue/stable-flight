use approx::assert_relative_eq;
use assert2::check;
use core::f32::consts::{FRAC_1_SQRT_2, PI};

use super::*;
use crate::tuning::identify::unit_delay;

const DT: f32 = 0.004;

/// The steady amplitude of a unit sine at `frequency_hz` through the filter, read over the
/// last second of three.
fn sine_gain(cutoff_hz: f32, frequency_hz: f32) -> f32 {
    let mut filter = LowPass::default();
    let ticks = (3.0 / DT) as usize;
    (0..ticks)
        .map(|tick| {
            let input = libm::sinf(2.0 * PI * frequency_hz * tick as f32 * DT);
            (tick, filter.apply(input, DT, cutoff_hz))
        })
        .filter(|(tick, _)| *tick > ticks * 2 / 3)
        .map(|(_, output)| output.abs())
        .fold(0.0, f32::max)
}

#[test]
fn passes_slow_signals_and_cuts_fast_ones() {
    // (cutoff Hz, sine Hz, expected gain): Butterworth, 1/√2 at the cutoff, prewarped.
    let cases = [
        (40.0, 2.0, 1.0),
        (40.0, 40.0, FRAC_1_SQRT_2),
        (40.0, 90.0, 0.08),
        (20.0, 20.0, FRAC_1_SQRT_2),
        (20.0, 90.0, 0.02),
    ];
    for (cutoff_hz, frequency_hz, expected) in cases {
        let gain = sine_gain(cutoff_hz, frequency_hz);
        let response = LowPass::response(cutoff_hz, DT, unit_delay(2.0 * PI * frequency_hz, DT));
        check!(
            (gain - expected).abs() < 0.02,
            "{cutoff_hz} Hz filter at {frequency_hz} Hz: {gain}"
        );
        check!(
            (response.norm_sqr().sqrt() - gain).abs() < 0.01,
            "{cutoff_hz} Hz filter at {frequency_hz} Hz: response {}",
            response.norm_sqr().sqrt()
        );
    }
}

#[test]
fn starts_at_the_first_sample() {
    let mut filter = LowPass::default();
    for _ in 0..5 {
        assert_relative_eq!(filter.apply(0.7, DT, 40.0), 0.7, epsilon = 1e-6);
    }
}

#[test]
fn passes_through_without_a_usable_cutoff() {
    // At or past Nyquist (125 Hz at 4 ms), off, or not finite.
    for cutoff_hz in [0.0, -5.0, 125.0, 400.0, f32::NAN, f32::INFINITY] {
        let mut filter = LowPass::default();
        check!(filter.apply(1.0, DT, cutoff_hz) == 1.0, "{cutoff_hz}");
        check!(filter.apply(-3.0, DT, cutoff_hz) == -3.0, "{cutoff_hz}");
        let response = LowPass::response(cutoff_hz, DT, unit_delay(300.0, DT));
        check!(response == Complex::new(1.0, 0.0), "{cutoff_hz}");
    }
}

#[test]
fn a_non_finite_sample_passes_and_keeps_the_history() {
    let mut filter = LowPass::default();
    filter.apply(1.0, DT, 40.0);
    let mut twin = filter;
    check!(filter.apply(f32::NAN, DT, 40.0).is_nan());
    check!(filter.apply(f32::INFINITY, DT, 40.0) == f32::INFINITY);
    check!(filter.apply(0.5, DT, 40.0) == twin.apply(0.5, DT, 40.0));
}

#[test]
fn a_new_cutoff_or_tick_takes_effect() {
    let mut filter = LowPass::default();
    filter.apply(0.0, DT, 40.0);
    let at_40 = filter.apply(1.0, DT, 40.0);
    let mut filter = LowPass::default();
    filter.apply(0.0, DT, 10.0);
    filter.apply(0.0, DT, 40.0);
    check!(filter.apply(1.0, DT, 40.0) == at_40);
    // Turned off, then on again: it starts afresh from the sample.
    check!(filter.apply(5.0, DT, 0.0) == 5.0);
    assert_relative_eq!(filter.apply(5.0, DT, 40.0), 5.0, epsilon = 1e-5);
}

#[test]
fn a_new_cutoff_or_tick_keeps_a_held_signal() {
    // (cutoff Hz then, tick s then): each switches to 40 Hz at 4 ms while holding 1.
    for (cutoff_hz, dt) in [(20.0, DT), (10.0, DT), (40.0, 0.002), (40.0, 0.008)] {
        let mut filter = LowPass::default();
        for _ in 0..200 {
            filter.apply(1.0, dt, cutoff_hz);
        }
        for _ in 0..5 {
            let output = filter.apply(1.0, DT, 40.0);
            check!(
                (output - 1.0).abs() < 1e-4,
                "from {cutoff_hz} Hz at {dt} s: {output}"
            );
        }
    }
}
