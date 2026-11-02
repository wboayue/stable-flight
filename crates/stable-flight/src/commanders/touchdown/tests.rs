use assert2::check;

use super::*;

/// `seconds` of 0.1 s ticks at `speed`.
fn ticks(speed: f32, seconds: f32) -> impl Iterator<Item = Tick> + Clone {
    let n = (seconds / 0.1).round() as usize;
    core::iter::repeat_n((0.1, speed), n)
}

/// One tick of `dt` at `speed`.
fn short(dt: f32, speed: f32) -> impl Iterator<Item = Tick> + Clone {
    core::iter::once((dt, speed))
}

/// (dt, vertical speed)
type Tick = (f32, f32);

/// One row per scenario: a sequence of (dt, vertical speed) ticks and whether the last one
/// reads touched down. Holds are 0.1 s ticks to 1.05 s or 0.95 s, clear of `f32` rounding at
/// the 1 s threshold.
#[test]
fn detects_touchdown_only_after_a_descent_then_a_held_stop() {
    // (name, ticks, touched down)
    let cases: [(_, Vec<Tick>, _); 9] = [
        ("hover before descending", ticks(0.0, 5.0).collect(), false),
        (
            "descend then stop past the hold",
            ticks(-0.7, 2.0)
                .chain(ticks(0.0, 1.0))
                .chain(short(0.05, 0.0))
                .collect(),
            true,
        ),
        (
            "descend then stop short of the hold",
            ticks(-0.7, 2.0)
                .chain(ticks(0.0, 0.9))
                .chain(short(0.05, 0.0))
                .collect(),
            false,
        ),
        ("still descending", ticks(-0.7, 5.0).collect(), false),
        (
            "slow sink stays below the stop band",
            ticks(-0.7, 1.0).chain(ticks(-0.15, 3.0)).collect(),
            false,
        ),
        (
            "a bounce restarts the hold",
            ticks(-0.7, 1.0)
                .chain(ticks(0.0, 0.8))
                .chain(ticks(0.3, 0.1))
                .chain(ticks(0.0, 0.8))
                .collect(),
            false,
        ),
        (
            "jitter inside the stop band counts as stopped",
            ticks(-0.7, 1.0)
                .chain(ticks(0.05, 0.5))
                .chain(ticks(-0.05, 0.5))
                .chain(short(0.05, 0.0))
                .collect(),
            true,
        ),
        (
            "NaN restarts the hold",
            ticks(-0.7, 1.0)
                .chain(ticks(0.0, 0.8))
                .chain(ticks(f32::NAN, 0.1))
                .chain(ticks(0.0, 0.8))
                .collect(),
            false,
        ),
        (
            "NaN keeps the descent",
            ticks(-0.7, 1.0)
                .chain(ticks(f32::NAN, 0.5))
                .chain(ticks(0.0, 1.0))
                .chain(short(0.05, 0.0))
                .collect(),
            true,
        ),
    ];
    run(&cases, TouchdownDetector::default);
}

/// Feeds each row's ticks to a fresh detector and checks the last tick's verdict.
fn run(cases: &[(&str, Vec<Tick>, bool)], detector: impl Fn() -> TouchdownDetector) {
    for (name, ticks, expected) in cases {
        let (name, expected) = (*name, *expected);
        let mut detector = detector();
        let mut down = false;
        for &(dt, speed) in ticks {
            down = detector.update(dt, speed);
        }
        check!(down == expected, "{name}");
        check!(detector.touched_down() == expected, "{name}");
    }
}

#[test]
fn touchdown_latches_until_reset() {
    let mut detector = TouchdownDetector::default();
    detector.update(0.1, -0.7);
    detector.update(1.05, 0.0);
    check!(detector.touched_down());
    check!(detector.update(0.1, -3.0), "latched through later motion");

    detector.reset();
    check!(!detector.touched_down());
    check!(!detector.update(2.0, 0.0), "reset forgets the descent too");
}

/// A 0.15 m/s band and a 2 s hold: jitter the default band rejects counts as stopped, and the
/// default hold no longer touches down.
#[test]
fn reads_touchdown_per_the_configured_band_and_hold() {
    let config = TouchdownConfig {
        stopped_mps: 0.15,
        hold_s: 2.0,
    };
    let jitter = |seconds: f32| ticks(0.12, seconds / 2.0).chain(ticks(-0.12, seconds / 2.0));
    // (name, ticks, touched down)
    let cases: [(_, Vec<Tick>, _); 3] = [
        (
            "jitter in the wider band past the hold",
            ticks(-0.7, 1.0)
                .chain(jitter(2.0))
                .chain(short(0.05, 0.0))
                .collect(),
            true,
        ),
        (
            "stopped past the default hold, short of the configured one",
            ticks(-0.7, 1.0).chain(ticks(0.0, 1.9)).collect(),
            false,
        ),
        (
            "moving past the wider band restarts the hold",
            ticks(-0.7, 1.0)
                .chain(ticks(0.0, 1.5))
                .chain(ticks(0.16, 0.1))
                .chain(ticks(0.0, 1.5))
                .collect(),
            false,
        ),
    ];
    run(&cases, || TouchdownDetector::new(config).expect("valid"));
}

#[test]
fn reset_keeps_the_config() {
    let mut detector = TouchdownDetector::new(TouchdownConfig {
        hold_s: 2.0,
        ..TouchdownConfig::default()
    })
    .expect("valid");
    detector.reset();
    detector.update(0.1, -0.7);
    check!(
        !detector.update(1.5, 0.0),
        "the configured 2 s hold, not the default 1 s"
    );
}

#[test]
fn rejects_each_field_out_of_range() {
    let with = |stopped_mps, hold_s| TouchdownConfig {
        stopped_mps,
        hold_s,
    };
    // (case, config, the field named, or None for valid)
    let cases = [
        ("default", TouchdownConfig::default(), None),
        ("band just below the descent", with(0.19, 1.0), None),
        ("zero band", with(0.0, 1.0), Some("stopped_mps")),
        (
            "band at the descent",
            with(DESCENDING_MPS, 1.0),
            Some("stopped_mps"),
        ),
        ("NaN band", with(f32::NAN, 1.0), Some("stopped_mps")),
        ("zero hold", with(0.1, 0.0), Some("hold_s")),
        ("infinite hold", with(0.1, f32::INFINITY), Some("hold_s")),
    ];
    for (case, config, field) in cases {
        let error = config.validate().err().map(|e| e.to_string());
        let named = match field {
            None => error.is_none(),
            Some(field) => error.as_deref().is_some_and(|e| e.starts_with(field)),
        };
        check!(named, "{case}: {error:?}");
        check!(
            TouchdownDetector::new(config).is_ok() == field.is_none(),
            "{case}"
        );
    }
}
