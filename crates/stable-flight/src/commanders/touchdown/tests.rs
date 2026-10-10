use assert2::check;

use super::*;

/// (dt, vertical speed)
type Tick = (f32, f32);

/// One row per scenario: a sequence of (dt, vertical speed) ticks and whether the last one
/// reads touched down. Holds are 0.1 s ticks to 1.05 s or 0.95 s, clear of `f32` rounding at
/// the 1 s threshold.
#[test]
fn detects_touchdown_only_after_a_descent_then_a_held_stop() {
    let ticks = |speed: f32, seconds: f32| {
        let n = (seconds / 0.1).round() as usize;
        core::iter::repeat_n((0.1, speed), n)
    };
    let short = |dt: f32, speed: f32| core::iter::once((dt, speed));
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
    for (name, ticks, expected) in cases {
        let mut detector = TouchdownDetector::default();
        let mut down = false;
        for (dt, speed) in ticks {
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
