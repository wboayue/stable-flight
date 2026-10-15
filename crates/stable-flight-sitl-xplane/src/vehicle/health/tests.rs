use assert2::check;

use super::*;

const DT: f32 = 0.004;

fn run(liveness: &mut Liveness, seconds: f32) {
    for _ in 0..(seconds / DT).round() as usize {
        liveness.advance(DT);
    }
}

/// A silence starts after 0.5 s without a frame, is reported once, and the next frame ends it
/// with its length.
#[test]
fn reports_a_silence_once_and_its_end() {
    let mut liveness = Liveness::default();
    run(&mut liveness, 0.45);
    check!(!liveness.is_silent());
    check!(!liveness.silence_started());

    run(&mut liveness, 0.1);
    check!(liveness.is_silent());
    check!(liveness.silence_started());
    check!(!liveness.silence_started());

    run(&mut liveness, 1.0);
    let ended = liveness.frame_arrived().expect("silence ended");
    check!((ended - 1.55).abs() < 0.01);
    check!(!liveness.is_silent());
    check!(liveness.frame_arrived() == None);
}

/// A frame before the silence threshold ends nothing.
#[test]
fn ignores_short_gaps() {
    let mut liveness = Liveness::default();
    run(&mut liveness, 0.3);
    check!(liveness.frame_arrived() == None);
}

/// Resubscribes go out only when needed, at most every 0.25 s; the first at once.
#[test]
fn spaces_resubscribes() {
    let mut liveness = Liveness::default();
    check!(!liveness.resubscribe_due(false));
    check!(liveness.resubscribe_due(true));
    run(&mut liveness, 0.2);
    check!(!liveness.resubscribe_due(true));
    run(&mut liveness, 0.1);
    check!(liveness.resubscribe_due(true));
}

/// Failures log at most once a second.
#[test]
fn rate_limits_failure_reports() {
    let mut liveness = Liveness::default();
    check!(liveness.report(format_args!("first")));
    run(&mut liveness, 0.5);
    check!(!liveness.report(format_args!("suppressed")));
    run(&mut liveness, 0.6);
    check!(liveness.report(format_args!("again")));
}
