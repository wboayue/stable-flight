use assert2::check;

use super::*;

#[test]
fn fires_once_per_period_from_start_without_drift_or_bursts() {
    let ms = Duration::from_millis;
    // (poll at ms after start, expected firing)
    let polls = [
        (0, true),    // at start
        (50, false),  // within the period
        (103, true),  // late poll fires
        (199, false), // next is due at 200, not 203: no drift
        (200, true),
        (750, true),  // five periods late: fires once
        (800, false), // and reschedules from the late poll, without catch-up
        (850, true),
    ];

    let t0 = Instant::now();
    let mut interval = Interval::new(ms(100));
    check!(!interval.due(t0), "not started");
    interval.start(t0);
    for (at_ms, fires) in polls {
        check!(interval.due(t0 + ms(at_ms)) == fires, "{at_ms} ms");
    }
}
