use assert2::check;

use super::*;

#[test]
fn backs_off_while_idle_and_resets_on_traffic() {
    let busy = Activity {
        mavlink: 1,
        ..Activity::default()
    };
    let idle = Activity::default();
    let failed_only = Activity {
        failures: 3,
        dropped: 2,
        ..Activity::default()
    };
    let ms = Duration::from_millis;
    // (case, pass, expected sleep), applied in order to one backoff
    let steps = [
        ("traffic", busy, ms(1)),
        ("idle", idle, ms(2)),
        ("idle", idle, ms(4)),
        ("idle", idle, ms(8)),
        ("idle: capped", idle, ms(8)),
        ("failures and drops aren't traffic", failed_only, ms(8)),
        ("traffic resets", busy, ms(1)),
    ];
    let mut backoff = IdleBackoff::default();
    for (i, (case, activity, expected)) in steps.into_iter().enumerate() {
        check!(backoff.next(&activity) == expected, "step {i}: {case}");
    }
}
