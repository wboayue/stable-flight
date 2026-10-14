use assert2::check;

use super::*;

fn motors(command: f32) -> [f32; SLOTS] {
    [command; SLOTS]
}

#[test]
fn starts_stopped() {
    let mut line = DelayLine::new(0.01);
    check!(*line.at(1.0) == motors(0.0));
}

#[test]
fn zero_delay_applies_at_once() {
    let mut line = DelayLine::new(0.0);
    line.push(0.5, motors(0.4));
    check!(*line.at(0.5) == motors(0.4));
}

#[test]
fn applies_each_command_once_its_delay_passes() {
    let mut line = DelayLine::new(0.01);
    line.push(0.0, motors(0.2));
    line.push(0.004, motors(0.6));
    // (time s, command running)
    let cases = [
        (0.0, 0.0),
        (0.009, 0.0),
        (0.010, 0.2),
        (0.013, 0.2),
        (0.014, 0.6),
        (1.0, 0.6),
    ];
    for (time_s, command) in cases {
        check!(*line.at(time_s) == motors(command), "at {time_s} s");
    }
}

#[test]
fn a_full_line_applies_its_oldest_command_early() {
    let mut line = DelayLine::new(0.05);
    for i in 0..=CAPACITY {
        line.push(i as f64 * 1e-4, motors((i + 1) as f32));
    }
    // The first command made room for the last, though its delay hasn't passed.
    check!(*line.at(0.0) == motors(1.0));
    check!(line.len == CAPACITY);
    check!(*line.at(1.0) == motors((CAPACITY + 1) as f32));
}
