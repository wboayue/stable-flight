use assert2::check;
use core::f32::consts::PI;

use super::*;
use crate::tuning::identify::unit_delay;

const DT: f32 = 0.004;

/// The notch the sim's robustness noise rows fly: ArduPilot's form at the 90 Hz tone, half the
/// center wide, 40 dB deep.
const TONE: NotchConfig = NotchConfig {
    center_hz: 90.0,
    bandwidth_hz: 45.0,
    attenuation_db: 40.0,
};

/// The steady amplitude of a unit sine at `frequency_hz` through the notch, read over the
/// last second of three.
fn sine_gain(config: &NotchConfig, frequency_hz: f32) -> f32 {
    let mut notch = Notch::default();
    let ticks = (3.0 / DT) as usize;
    (0..ticks)
        .map(|tick| {
            let input = libm::sinf(2.0 * PI * frequency_hz * tick as f32 * DT);
            (tick, notch.apply(input, DT, config))
        })
        .filter(|(tick, _)| *tick > ticks * 2 / 3)
        .map(|(_, output)| output.abs())
        .fold(0.0, f32::max)
}

/// The notch cuts its center to its depth and leaves slow signals alone; its response agrees
/// with the filter: (sine Hz, the gain's bounds).
#[test]
fn cuts_the_center_and_passes_slow_signals() {
    let decibels = |gain: f32| 20.0 * libm::log10f(gain);
    let rows: [(f32, f32, f32); 3] = [
        (90.0, -60.0, -35.0), // 40 dB deep, read off a sampled sine
        (10.0, -0.1, 0.1),    // well within 1 dB
        (30.0, -0.5, 0.1),
    ];
    for (frequency_hz, low_db, high_db) in rows {
        let gain = sine_gain(&TONE, frequency_hz);
        let response = TONE
            .response(DT, unit_delay(2.0 * PI * frequency_hz, DT))
            .norm_sqr()
            .sqrt();
        check!(
            (low_db..=high_db).contains(&decibels(gain)),
            "{frequency_hz} Hz: {} dB",
            decibels(gain)
        );
        check!(
            (decibels(response) - decibels(gain)).abs() < 1.0 || frequency_hz == 90.0,
            "{frequency_hz} Hz: response {response} against {gain}"
        );
    }
    let center = TONE
        .response(DT, unit_delay(2.0 * PI * 90.0, DT))
        .norm_sqr()
        .sqrt();
    check!((decibels(center) + 40.0).abs() < 0.1, "{center}");
}

#[test]
fn starts_at_the_first_sample() {
    let mut notch = Notch::default();
    for _ in 0..5 {
        check!((notch.apply(0.7, DT, &TONE) - 0.7).abs() < 1e-6);
    }
}

/// Off, without width, or at or past 0.48 of the sample rate (120 Hz at 4 ms), the notch
/// passes the signal through, and its response is 1.
#[test]
fn passes_through_when_off_or_out_of_range() {
    let rows = [
        ("off", NotchConfig::default()),
        (
            "at 0.48 of the sample rate",
            NotchConfig {
                center_hz: 120.0,
                ..TONE
            },
        ),
        (
            "no width",
            NotchConfig {
                bandwidth_hz: 0.0,
                ..TONE
            },
        ),
        (
            "raised past 0.48 of the sample rate",
            NotchConfig {
                center_hz: 10.0,
                bandwidth_hz: 240.0,
                ..TONE
            },
        ),
    ];
    for (case, config) in rows {
        let mut notch = Notch::default();
        for tick in 0..50 {
            let input = libm::sinf(2.0 * PI * 90.0 * tick as f32 * DT);
            check!(notch.apply(input, DT, &config) == input, "{case}");
        }
        check!(
            config.response(DT, unit_delay(1.0, DT)) == Complex::new(1.0, 0.0),
            "{case}"
        );
    }
}

/// A center under 0.52 of the bandwidth cuts at 0.52 of it: the band's lower edge stays above
/// 0 Hz.
#[test]
fn a_center_below_half_the_width_is_raised() {
    let low = NotchConfig {
        center_hz: 10.0,
        bandwidth_hz: 100.0,
        ..TONE
    };
    let raised = NotchConfig {
        center_hz: 52.0,
        ..low
    };
    let w = |hz: f32| unit_delay(2.0 * PI * hz, DT);
    check!(low.response(DT, w(52.0)) == raised.response(DT, w(52.0)));
}

/// A non-finite sample passes through and leaves the history as it was.
#[test]
fn a_non_finite_sample_passes_through() {
    let mut notch = Notch::default();
    let mut twin = Notch::default();
    notch.apply(0.3, DT, &TONE);
    twin.apply(0.3, DT, &TONE);
    check!(notch.apply(f32::NAN, DT, &TONE).is_nan());
    check!(notch.apply(0.5, DT, &TONE) == twin.apply(0.5, DT, &TONE));
}

/// Each field's requirement: (case, config, the field rejected).
#[test]
fn validate_names_the_bad_field() {
    let rows = [
        (
            "negative center",
            NotchConfig {
                center_hz: -1.0,
                ..TONE
            },
            Some("center_hz"),
        ),
        (
            "NaN center",
            NotchConfig {
                center_hz: f32::NAN,
                ..TONE
            },
            Some("center_hz"),
        ),
        (
            "negative width",
            NotchConfig {
                bandwidth_hz: -1.0,
                ..TONE
            },
            Some("bandwidth_hz"),
        ),
        (
            "infinite width",
            NotchConfig {
                bandwidth_hz: f32::INFINITY,
                ..TONE
            },
            Some("bandwidth_hz"),
        ),
        (
            "zero depth",
            NotchConfig {
                attenuation_db: 0.0,
                ..TONE
            },
            Some("attenuation_db"),
        ),
        (
            "infinite depth",
            NotchConfig {
                attenuation_db: f32::INFINITY,
                ..TONE
            },
            Some("attenuation_db"),
        ),
        (
            "off with no width",
            NotchConfig {
                center_hz: 0.0,
                bandwidth_hz: 0.0,
                ..TONE
            },
            None,
        ),
        ("the tone's", TONE, None),
    ];
    for (case, config, field) in rows {
        check!(
            config.validate().err().map(|error| error.field) == field,
            "{case}"
        );
    }
}
