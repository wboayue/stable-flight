use super::*;

/// The period of the single-period tests.
const PERIOD: f32 = 0.02;

/// (period, dts, expected `(tick, step)` runs)
type Case = (f32, Vec<f32>, &'static [(usize, f32)]);

/// Runs `dts` through a divider that always runs when due; returns `(tick, step)` per run.
fn runs(period: f32, dts: &[f32]) -> Vec<(usize, f32)> {
    let mut divider = RateDivider::default();
    let mut runs = Vec::new();
    for (tick, &dt) in dts.iter().enumerate() {
        if let Some(step) = divider.due(dt, period) {
            divider.ran();
            runs.push((tick, step));
        }
    }
    runs
}

#[test]
fn runs_on_the_tick_nearest_each_period_over_the_elapsed_time() {
    let jitter = [
        0.004, 0.005, 0.003, 0.006, 0.004, 0.004, 0.003, 0.005, 0.006,
    ];
    let cases: [Case; 9] = [
        (0.02, vec![0.004; 11], &[(0, 0.004), (5, 0.02), (10, 0.02)]),
        (0.0, vec![0.004; 3], &[(0, 0.004), (1, 0.004), (2, 0.004)]),
        // Period shorter than a tick: every tick.
        (0.002, vec![0.004; 3], &[(0, 0.004), (1, 0.004), (2, 0.004)]),
        // 5.45 ticks rounds down, 5.55 up.
        (
            0.0218,
            vec![0.004; 11],
            &[(0, 0.004), (5, 0.02), (10, 0.02)],
        ),
        (
            0.0222,
            vec![0.004; 13],
            &[(0, 0.004), (6, 0.024), (12, 0.024)],
        ),
        // 6.67 ticks rounds to 7, stepping over 21 ms.
        (
            0.02,
            vec![0.003; 15],
            &[(0, 0.003), (7, 0.021), (14, 0.021)],
        ),
        // 8 × 2.5 ms sums to 19.999998 ms in f32; the slack still runs on tick 8.
        (
            0.02,
            vec![0.0025; 17],
            &[(0, 0.0025), (8, 0.02), (16, 0.02)],
        ),
        (0.02, vec![0.005; 9], &[(0, 0.005), (4, 0.02), (8, 0.02)]),
        // Varying dt: runs once the accumulated time is within half a tick of the period.
        (0.02, jitter.to_vec(), &[(0, 0.004), (4, 0.018), (8, 0.018)]),
    ];
    for (period, dts, expected) in cases {
        let actual = runs(period, &dts);
        assert!(
            actual.len() == expected.len()
                && actual
                    .iter()
                    .zip(expected)
                    .all(|(a, e)| a.0 == e.0 && (a.1 - e.1).abs() < 1e-6),
            "period {period}, dts {dts:?}: ran {actual:?}, expected {expected:?}"
        );
    }
}

#[test]
fn a_skipped_run_stays_due_and_steps_over_the_combined_time() {
    let mut divider = RateDivider::default();
    assert_eq!(divider.due(0.004, PERIOD), Some(0.004));
    divider.ran();
    for _ in 0..4 {
        assert_eq!(divider.due(0.004, PERIOD), None);
    }
    // Due on tick 5, but the loop doesn't run (e.g. a NaN estimate).
    assert!(divider.due(0.004, PERIOD).is_some());
    let retry = divider.due(0.004, PERIOD).expect("still due");
    approx::assert_relative_eq!(retry, 0.024, epsilon = 1e-6);
}

#[test]
fn a_skipped_first_run_retries_over_one_tick() {
    let mut divider = RateDivider::default();
    assert_eq!(divider.due(0.004, PERIOD), Some(0.004));
    assert_eq!(divider.due(0.004, PERIOD), Some(0.004));
}

#[test]
fn reset_makes_the_loop_due_on_the_next_tick() {
    let mut divider = RateDivider::default();
    divider.due(0.004, PERIOD);
    divider.ran();
    divider.reset();
    assert_eq!(divider.due(0.004, PERIOD), Some(0.004));
}

/// A period change applies on the next tick and keeps the elapsed time.
#[test]
fn period_change_applies_on_the_next_tick() {
    let mut divider = RateDivider::default();
    assert_eq!(divider.due(0.004, PERIOD), Some(0.004));
    divider.ran();
    for _ in 0..2 {
        assert_eq!(divider.due(0.004, PERIOD), None);
    }
    // 12 ms elapsed: due at once under a 10 ms period.
    let step = divider
        .due(0.004, 0.01)
        .expect("due under the shorter period");
    approx::assert_relative_eq!(step, 0.012, epsilon = 1e-6);
}
