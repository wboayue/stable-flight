use approx::assert_relative_eq;
use assert2::{assert, check};

use super::*;

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

fn config(amplitude: f32, duration_s: f32) -> StepConfig {
    StepConfig {
        amplitude,
        duration_s,
        settle_band: 0.1,
        filter_hz: 0.0,
    }
}

/// Feeds `measurements` one per tick of `dt` and returns the final status.
fn run(config: StepConfig, dt: f32, measurements: &[f32]) -> StepStatus {
    let mut probe = StepProbe::new(config);
    for &measurement in measurements {
        probe.step(dt, measurement, false);
    }
    probe.status()
}

fn done(status: StepStatus) -> StepMetrics {
    match status {
        StepStatus::Done(metrics) => metrics,
        other => panic!("expected Done, got {other:?}"),
    }
}

/// A unit-gain second-order loop `ωₙ² / (s² + 2ζωₙs + ωₙ²)` following the probe's setpoint,
/// integrated in fine sub-steps. Returns the metrics and, from a much finer run of the same
/// system, the reference rise and settling times.
fn second_order(zeta: f32, omega_hz: f32) -> (StepMetrics, f32, f32) {
    const SUBSTEPS: usize = 40;
    let omega = 2.0 * PI * omega_hz;
    let duration_s = 1.0;
    let mut probe = StepProbe::new(config(1.0, duration_s));
    let (mut y, mut v) = (0.0_f32, 0.0_f32);
    // Fine reference: sample every sub-step.
    let (mut t10, mut t90, mut last_out) = (None, None, 0.0_f32);
    let mut t = 0.0_f32;
    let mut setpoint;
    loop {
        setpoint = probe.step(DT, y, false);
        if !matches!(probe.status(), StepStatus::Running { .. }) {
            break;
        }
        let h = DT / SUBSTEPS as f32;
        for _ in 0..SUBSTEPS {
            v += h * (omega * omega * (setpoint - y) - 2.0 * zeta * omega * v);
            y += h * v;
            t += h;
            if t10.is_none() && y >= 0.1 {
                t10 = Some(t);
            }
            if t90.is_none() && y >= 0.9 {
                t90 = Some(t);
            }
            if (y - 1.0).abs() > 0.1 {
                last_out = t;
            }
        }
    }
    let rise = t90.unwrap_or(f32::NAN) - t10.unwrap_or(f32::NAN);
    (done(probe.status()), rise, last_out)
}

/// On second-order loops, the overshoot matches `e^(−πζ/√(1−ζ²))`, and rise and settling
/// times match a fine-grained run within the tick the probe samples at.
#[test]
fn measures_second_order_responses() {
    // (ζ, natural frequency in Hz)
    let cases = [(0.3, 4.0), (0.5, 4.0), (0.7, 4.0), (1.0, 4.0), (0.5, 8.0)];
    for (zeta, omega_hz) in cases {
        let (metrics, rise_s, settling_s) = second_order(zeta, omega_hz);

        let expected = if zeta < 1.0 {
            libm::expf(-PI * zeta / libm::sqrtf(1.0 - zeta * zeta))
        } else {
            0.0
        };
        check!(
            (metrics.overshoot - expected).abs() < 0.02,
            "ζ {zeta}: overshoot {} vs {expected}",
            metrics.overshoot
        );
        let rise = metrics.rise_time_s.unwrap_or(f32::NAN);
        check!(
            (rise - rise_s).abs() <= 2.0 * DT,
            "ζ {zeta}: rise {rise} vs {rise_s}"
        );
        let settling = metrics.settling_time_s.unwrap_or(f32::NAN);
        check!(
            (settling - settling_s).abs() <= 2.0 * DT,
            "ζ {zeta}: settling {settling} vs {settling_s}"
        );
        check!((metrics.final_value - 1.0).abs() < 0.1, "ζ {zeta}");
        check!(metrics.saturated_s == 0.0);
    }
}

/// A scripted response: each metric read off it by hand.
#[test]
fn reads_each_metric_off_the_response() {
    // t = 0, 0.1, …, 0.5 s
    let measurements = [0.0, 0.5, 1.2, 0.95, 1.05, 1.0];
    let metrics = done(run(config(1.0, 0.5), 0.1, &measurements));

    assert_relative_eq!(metrics.peak, 1.2);
    assert_relative_eq!(metrics.peak_time_s, 0.2);
    assert_relative_eq!(metrics.overshoot, 0.2, epsilon = 1e-6);
    assert_relative_eq!(metrics.rise_time_s.unwrap_or(f32::NAN), 0.1);
    assert_relative_eq!(metrics.settling_time_s.unwrap_or(f32::NAN), 0.3);
    assert_relative_eq!(metrics.bounce_back, 0.25, epsilon = 1e-6);
    assert_relative_eq!(metrics.final_value, 1.0);
    check!(metrics.amplitude == 1.0);
}

/// Normalized metrics don't depend on the step's direction, size, or starting point.
#[test]
fn metrics_are_normalized_to_the_step() {
    let shape = [0.0, 0.5, 1.2, 0.95, 1.05, 1.0];
    let reference = done(run(config(1.0, 0.5), 0.1, &shape));
    // (amplitude, starting measurement)
    for (amplitude, start) in [(-1.0, 0.0), (0.2, 0.3), (-3.0, -1.0)] {
        let measurements = shape.map(|y| start + amplitude * y);
        let metrics = done(run(config(amplitude, 0.5), 0.1, &measurements));
        check!(metrics.amplitude == amplitude);
        assert_relative_eq!(metrics.overshoot, reference.overshoot, epsilon = 1e-5);
        assert_relative_eq!(metrics.bounce_back, reference.bounce_back, epsilon = 1e-5);
        assert_relative_eq!(metrics.final_value, reference.final_value, epsilon = 1e-5);
        check!(metrics.rise_time_s.is_some() && metrics.settling_time_s.is_some());
    }
}

/// The setpoint is the starting measurement plus the step, held for the whole window and
/// after it.
#[test]
fn holds_the_setpoint_at_the_start_plus_the_step() {
    let mut probe = StepProbe::new(config(-0.5, 0.1));
    let mut setpoints = Vec::new();
    for measurement in [0.2, 0.0, -0.1, 0.4] {
        setpoints.push(probe.step(0.05, measurement, false));
    }
    check!(setpoints == [-0.3; 4]);
    check!(matches!(probe.status(), StepStatus::Done(_)));
}

/// A response that never reaches 90% has no rise time; one outside the band at the end has
/// no settling time.
#[test]
fn reports_missing_rise_and_settling() {
    let slow = done(run(config(1.0, 0.3), 0.1, &[0.0, 0.3, 0.5, 0.6]));
    check!(slow.rise_time_s == None);
    check!(slow.settling_time_s == None);

    let ringing = done(run(config(1.0, 0.3), 0.1, &[0.0, 1.0, 1.0, 1.3]));
    check!(ringing.rise_time_s.is_some());
    check!(ringing.settling_time_s == None);
}

/// The filter smooths noise: alternating ±0.3 around the step reads a lower peak filtered.
#[test]
fn the_filter_smooths_noise() {
    let noisy: Vec<f32> = (0..100)
        .map(|tick| {
            if tick == 0 {
                0.0
            } else {
                1.0 + if tick % 2 == 0 { 0.3 } else { -0.3 }
            }
        })
        .collect();
    let raw = done(run(config(1.0, 0.39), DT, &noisy));
    let filtered = done(run(
        StepConfig {
            filter_hz: 20.0,
            ..config(1.0, 0.39)
        },
        DT,
        &noisy,
    ));
    check!(raw.peak > 1.29);
    check!(filtered.peak < 1.1, "filtered peak {}", filtered.peak);
}

/// Each clipped tick adds its `dt` to the saturated time.
#[test]
fn counts_saturated_time() {
    let mut probe = StepProbe::new(config(1.0, 0.1));
    for saturated in [true, true, false, true, false, false] {
        probe.step(0.02, 0.0, saturated);
    }
    let metrics = done(probe.status());
    assert_relative_eq!(metrics.saturated_s, 0.06);
}

/// The response running the wrong way (after the first tenth of the window) or away aborts;
/// a brief wrong-way kick at the start doesn't.
#[test]
fn aborts_on_a_reversed_or_runaway_response() {
    let mut reversed = vec![0.0];
    reversed.extend([-0.6; 12]);
    // (measurements at 0.01 s ticks over a 1 s window, expected abort)
    let cases = [
        (reversed, Some(StepAbort::Reversed)),
        (vec![0.0, 0.5, 3.5], Some(StepAbort::Runaway)),
        (vec![0.0, -0.8, -0.8, 0.2], None),
    ];
    for (measurements, expected) in cases {
        let status = run(config(1.0, 1.0), 0.01, &measurements);
        let aborted = match status {
            StepStatus::Aborted(abort) => Some(abort),
            _ => None,
        };
        check!(aborted == expected, "{measurements:?}: {status:?}");
        if expected.is_none() {
            check!(matches!(status, StepStatus::Running { .. }));
        }
    }
}

/// A NaN measurement is skipped; a NaN at the start counts as 0.
#[test]
fn skips_nan_measurements() {
    let metrics = done(run(
        config(1.0, 0.4),
        0.1,
        &[f32::NAN, 0.5, f32::NAN, 1.0, 1.0],
    ));
    assert_relative_eq!(metrics.final_value, 1.0);
    assert_relative_eq!(metrics.peak, 1.0);
}

/// `validate` rejects each bad field.
#[test]
fn validate_rejects_bad_fields() {
    let good = config(1.0, 0.25);
    // (config, field)
    let cases = [
        (
            StepConfig {
                amplitude: 0.0,
                ..good
            },
            "amplitude",
        ),
        (
            StepConfig {
                amplitude: f32::NAN,
                ..good
            },
            "amplitude",
        ),
        (
            StepConfig {
                duration_s: 0.0,
                ..good
            },
            "duration_s",
        ),
        (
            StepConfig {
                settle_band: 0.0,
                ..good
            },
            "settle_band",
        ),
        (
            StepConfig {
                settle_band: 1.0,
                ..good
            },
            "settle_band",
        ),
        (
            StepConfig {
                filter_hz: -1.0,
                ..good
            },
            "filter_hz",
        ),
        (
            StepConfig {
                filter_hz: f32::INFINITY,
                ..good
            },
            "filter_hz",
        ),
    ];
    assert!(good.validate().is_ok());
    for (config, field) in cases {
        let error = config.validate().expect_err(field);
        check!(error.field == field);
    }
    check!(good.reversed().amplitude == -1.0);
}

#[test]
#[should_panic(expected = "invalid StepConfig")]
fn invalid_config_panics() {
    StepProbe::new(config(0.0, 1.0));
}

/// The summary keeps each metric's mean and range, splits overshoot by direction, and counts
/// steps that never rose or settled.
#[test]
fn summary_aggregates_steps() {
    let step =
        |amplitude: f32, overshoot: f32, rise: Option<f32>, settling: Option<f32>| StepMetrics {
            amplitude,
            overshoot,
            rise_time_s: rise,
            settling_time_s: settling,
            bounce_back: overshoot / 2.0,
            final_value: 1.0,
            ..Default::default()
        };
    let mut summary = StepSummary::default();
    check!(summary.overshoot.mean() == None);
    for metrics in [
        step(1.0, 0.3, Some(0.05), Some(0.2)),
        step(-1.0, 0.1, Some(0.07), None),
        step(1.0, 0.2, None, Some(0.3)),
    ] {
        summary.add(&metrics);
    }

    check!(summary.count() == 3);
    assert_relative_eq!(summary.overshoot.mean().unwrap_or(f32::NAN), 0.2);
    check!(summary.overshoot.min == 0.1);
    check!(summary.overshoot.max == 0.3);
    assert_relative_eq!(summary.overshoot_positive.mean().unwrap_or(f32::NAN), 0.25);
    check!(summary.overshoot_negative.mean() == Some(0.1));
    assert_relative_eq!(summary.rise_time_s.mean().unwrap_or(f32::NAN), 0.06);
    check!(summary.not_risen == 1);
    assert_relative_eq!(summary.settling_time_s.mean().unwrap_or(f32::NAN), 0.25);
    check!(summary.not_settled == 1);
    assert_relative_eq!(summary.bounce_back.mean().unwrap_or(f32::NAN), 0.1);
    check!(summary.final_value.mean() == Some(1.0));
}
