use std::time::Duration;

use assert2::check;

use super::*;

/// 250 Hz, a 0.5 s window, slow under 90% (225 Hz), keeping time again over 96% (240 Hz).
fn watch() -> TickRate {
    TickRate::new(250.0, Duration::from_millis(500), 0.9, 0.96).expect("valid")
}

/// Observes `ticks` ticks from `start_us`, `gaps_us` apart (cycled), and returns each tick's
/// verdict and the time of the last.
fn run(rate: &mut TickRate, start_us: u64, gaps_us: &[u64], ticks: usize) -> (Vec<bool>, u64) {
    let mut elapsed_us = start_us;
    let mut verdicts = Vec::with_capacity(ticks);
    for (tick, gap_us) in gaps_us.iter().cycle().take(ticks).enumerate() {
        elapsed_us += gap_us;
        let stats = RunStats {
            ticks: tick as u64,
            elapsed: Duration::from_micros(elapsed_us),
            overruns: 0,
            skipped_ticks: 0,
            host_slow: false,
        };
        verdicts.push(rate.observe(&stats));
    }
    (verdicts, elapsed_us)
}

/// Steady rates from a host keeping time: nominal and jitter around it keep time, a steady
/// 120 Hz (RealFlight's slow mode) and just under the slow threshold are slow; nothing is
/// slow until the window fills.
#[test]
fn reports_a_steady_rate_against_the_threshold() {
    // (gaps between ticks in µs, slow once full)
    let cases: [(&[u64], bool); 5] = [
        (&[4_000], false),        // 250 Hz
        (&[3_000, 5_000], false), // jitter averaging 250 Hz
        (&[4_400], false),        // 227 Hz, just over 225
        (&[4_500], true),         // 222 Hz, just under
        (&[8_333], true),         // 120 Hz
    ];
    for (gaps_us, slow) in cases {
        let mut rate = watch();
        let (verdicts, _) = run(&mut rate, 0, gaps_us, 400);
        check!(
            verdicts[..125].iter().all(|v| !v),
            "{gaps_us:?}: slow before full"
        );
        check!(
            verdicts[399] == slow,
            "{gaps_us:?}: {:?} Hz",
            rate.rate_hz()
        );
        check!(rate.is_slow() == slow);
    }
}

/// Between the thresholds the host stays as it was: at 230 Hz a host keeping time stays so,
/// and a slow one stays slow until it regains 240 Hz.
#[test]
fn holds_its_state_between_the_thresholds() {
    // (lead-in gap in µs, then the gap held, slow at the end)
    let cases = [
        (4_000, 4_348, false), // keeping time, then 230 Hz
        (8_333, 4_348, true),  // slow, then 230 Hz
        (8_333, 4_082, false), // slow, then 245 Hz
    ];
    for (lead_us, held_us, slow) in cases {
        let mut rate = watch();
        let (_, at_us) = run(&mut rate, 0, &[lead_us], 300);
        let (verdicts, _) = run(&mut rate, at_us, &[held_us], 300);
        check!(
            verdicts[299] == slow,
            "{lead_us} then {held_us} µs: {:?} Hz",
            rate.rate_hz()
        );
    }
}

/// A one-off 100 ms stall shows as slow while it is in the window, then the rate recovers;
/// recovering from a slow stretch takes a window of ticks at the nominal rate.
#[test]
fn a_stall_shows_while_in_the_window_then_recovers() {
    let mut rate = watch();
    let (_, at_us) = run(&mut rate, 0, &[4_000], 200);
    let (stalled, at_us) = run(&mut rate, at_us, &[100_000], 1);
    check!(stalled == [true], "125 ticks over 0.6 s: 208 Hz");
    let (after, _) = run(&mut rate, at_us, &[4_000], 200);
    check!(
        after[..124].iter().all(|v| *v),
        "the stall still in the window"
    );
    check!(after[125..].iter().all(|v| !v), "out of it");

    let mut rate = watch();
    let (_, at_us) = run(&mut rate, 0, &[8_333], 300);
    check!(rate.is_slow());
    let (after, _) = run(&mut rate, at_us, &[4_000], 200);
    check!(
        after[..100].iter().all(|v| *v),
        "mostly slow ticks in the window"
    );
    check!(after[125..].iter().all(|v| !v));
}

/// The rate over the window, once full.
#[test]
fn measures_the_rate_over_the_window() {
    let mut rate = watch();
    check!(rate.rate_hz().is_none());
    run(&mut rate, 0, &[8_000], 200);
    let hz = rate.rate_hz().expect("full");
    check!((hz - 125.0).abs() < 1e-6, "{hz}");
}

/// When, in µs, the first tick reporting `slow` ran, among `verdicts` from ticks `gap_us` apart
/// that ended at `end_us`.
fn flipped_at_us(verdicts: &[bool], end_us: u64, gap_us: u64, slow: bool) -> u64 {
    let flip = verdicts.iter().position(|v| *v == slow).expect("flipped");
    end_us - gap_us * (verdicts.len() - 1 - flip) as u64
}

/// Slow stretches are counted and timed from the tick that turned slow to the one keeping time
/// again; the current stretch counts up to the last tick.
#[test]
fn counts_and_times_the_slow_stretches() {
    let mut rate = watch();
    check!(rate.slow_stretches() == 0);
    check!(rate.slow_time() == Duration::ZERO);

    let (_, at_us) = run(&mut rate, 0, &[4_000], 300);
    let (slow, at_us) = run(&mut rate, at_us, &[8_333], 300);
    let turned_slow_us = flipped_at_us(&slow, at_us, 8_333, true);
    check!(rate.slow_time() == Duration::from_micros(at_us - turned_slow_us));
    let (back, at_us) = run(&mut rate, at_us, &[4_000], 300);
    let kept_time_us = flipped_at_us(&back, at_us, 4_000, false);
    let first = Duration::from_micros(kept_time_us - turned_slow_us);
    check!(rate.slow_stretches() == 1);
    check!(rate.last_slow_stretch() == first);
    check!(
        rate.slow_time() == first,
        "not counting on after keeping time"
    );

    let (slow, at_us) = run(&mut rate, at_us, &[8_333], 300);
    let turned_slow_us = flipped_at_us(&slow, at_us, 8_333, true);
    check!(rate.slow_stretches() == 2);
    check!(
        rate.last_slow_stretch() == first,
        "the current stretch hasn't ended"
    );
    check!(rate.slow_time() == first + Duration::from_micros(at_us - turned_slow_us));
}

/// `standard` is the autotune's watch (#438); the window needs two ticks, so 3 Hz is the least.
#[test]
fn standard_watches_half_a_second_at_90_and_96_percent() {
    check!(TickRate::standard(250.0) == Ok(watch()));
    check!(TickRate::standard(3.0).is_ok());
    check!(TickRate::standard(2.0).map_err(|e| e.field) == Err("window"));
}

/// Each setting out of range is named.
#[test]
fn rejects_settings_out_of_range() {
    let ms = Duration::from_millis;
    // (nominal_hz, window, slow_share, resume_share, field)
    let cases = [
        (0.0, ms(500), 0.9, 0.96, "nominal_hz"),
        (f64::NAN, ms(500), 0.9, 0.96, "nominal_hz"),
        (f64::INFINITY, ms(500), 0.9, 0.96, "nominal_hz"),
        (250.0, ms(4), 0.9, 0.96, "window"), // one tick
        (250.0, Duration::ZERO, 0.9, 0.96, "window"),
        (250.0, ms(500), 0.0, 0.96, "slow_share"),
        (250.0, ms(500), 1.5, 1.5, "slow_share"),
        (250.0, ms(500), f64::NAN, 0.96, "slow_share"),
        (250.0, ms(500), 0.9, 0.8, "resume_share"), // under slow_share
        (250.0, ms(500), 0.9, 1.1, "resume_share"),
        (250.0, ms(500), 0.9, f64::NAN, "resume_share"),
    ];
    for (nominal_hz, window, slow_share, resume_share, field) in cases {
        let error = TickRate::new(nominal_hz, window, slow_share, resume_share).expect_err(field);
        check!(error.field == field);
    }
    check!(
        TickRate::new(250.0, ms(8), 1.0, 1.0).is_ok(),
        "two ticks, the whole rate, no gap"
    );
}
