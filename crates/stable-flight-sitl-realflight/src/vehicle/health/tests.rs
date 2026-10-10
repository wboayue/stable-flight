use super::*;

/// 0.1 s ticks; thresholds are checked a tick or more away so `f32` accumulation can't tip them.
const DT_S: f32 = 0.1;

fn advance(health: &mut LinkHealth, ticks: usize) {
    for _ in 0..ticks {
        health.advance(DT_S);
    }
}

#[test]
fn starts_connected() {
    let health = LinkHealth::default();

    assert!(!health.is_down());
    assert!(!health.retry_due());
}

#[test]
fn first_retry_waits_the_initial_backoff() {
    let mut health = LinkHealth::default();
    health.connection_lost();

    assert!(health.is_down());
    advance(&mut health, 4);
    assert!(!health.retry_due(), "due after 0.4 s");
    advance(&mut health, 2);
    assert!(health.retry_due(), "not due after 0.6 s");
}

#[test]
fn failed_attempts_double_the_backoff_up_to_the_cap() {
    let mut health = LinkHealth::default();
    health.connection_lost();

    let waits: Vec<f32> = (0..5).map(|_| health.reconnect_failed()).collect();

    assert_eq!(
        waits,
        [1.0, 2.0, MAX_BACKOFF_S, MAX_BACKOFF_S, MAX_BACKOFF_S]
    );
}

#[test]
fn a_failed_attempt_restarts_the_wait() {
    let mut health = LinkHealth::default();
    health.connection_lost();
    advance(&mut health, 6);

    health.reconnect_failed();

    advance(&mut health, 9);
    assert!(!health.retry_due(), "due after 0.9 s of a 1 s backoff");
    advance(&mut health, 2);
    assert!(health.retry_due(), "not due after 1.1 s of a 1 s backoff");
}

#[test]
fn reconnecting_reports_failed_attempts_and_resets_the_backoff() {
    let mut health = LinkHealth::default();
    health.connection_lost();
    health.reconnect_failed();
    health.reconnect_failed();

    assert_eq!(health.reconnected(), 2);
    assert!(!health.is_down());

    // A later loss starts over from the initial backoff.
    health.connection_lost();
    assert_eq!(health.reconnect_failed(), 2.0 * INITIAL_BACKOFF_S);
}

#[test]
fn a_failed_attempt_while_connected_does_nothing() {
    let mut health = LinkHealth::default();

    assert_eq!(health.reconnect_failed(), 0.0);
    assert!(!health.is_down());
}

#[test]
fn reports_at_most_once_per_interval_with_the_suppressed_count() {
    let mut health = LinkHealth::default();
    // One failure per tick; collect (tick, suppressed) for each logged one.
    let mut reports = Vec::new();
    for tick in 0..25u32 {
        if let Some(suppressed) = health.report() {
            reports.push((tick, suppressed));
        }
        health.advance(DT_S);
    }

    // The first at once, then one per second carrying the 9 (or so) skipped in between;
    // accumulation may land a report on tick 10 or 11.
    assert_eq!(reports.len(), 3);
    assert_eq!(reports[0], (0, 0));
    let total: u32 = reports.iter().map(|(_, n)| n).sum::<u32>() + reports.len() as u32;
    let (last_tick, _) = reports[2];
    assert_eq!(
        total,
        last_tick + 1,
        "every failure up to the last report is counted"
    );
}

#[test]
fn reports_again_at_once_after_a_quiet_interval() {
    let mut health = LinkHealth::default();
    health.report();
    advance(&mut health, 15);

    assert_eq!(health.report(), Some(0));
}
