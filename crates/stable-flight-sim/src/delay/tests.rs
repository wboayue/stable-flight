use assert2::check;

use super::*;

/// Every motor at `value`, every servo at `-value`, so both travel together.
fn command(value: f32) -> ActuatorCommand {
    ActuatorCommand {
        motors: [value; ActuatorCommand::SLOTS],
        servos: [-value; ActuatorCommand::SLOTS],
    }
}

#[test]
fn starts_stopped() {
    let mut line = DelayLine::new(0.01);
    check!(*line.due_at(1.0) == command(0.0));
}

#[test]
fn zero_delay_applies_at_once() {
    let mut line = DelayLine::new(0.0);
    line.push(0.5, command(0.4));
    check!(*line.due_at(0.5) == command(0.4));
}

#[test]
fn applies_each_command_once_its_delay_passes() {
    let mut line = DelayLine::new(0.01);
    line.push(0.0, command(0.2));
    line.push(0.004, command(0.6));
    // (time s, command running)
    let cases = [
        (0.0, 0.0),
        (0.009, 0.0),
        (0.010, 0.2),
        (0.013, 0.2),
        (0.014, 0.6),
        (1.0, 0.6),
    ];
    for (time_s, value) in cases {
        check!(*line.due_at(time_s) == command(value), "at {time_s} s");
    }
}

#[test]
fn a_full_line_applies_its_oldest_command_early() {
    let mut line = DelayLine::new(0.05);
    for i in 0..=CAPACITY {
        line.push(i as f64 * 1e-4, command((i + 1) as f32));
    }
    // The first command made room for the last, though its delay hasn't passed.
    check!(*line.due_at(0.0) == command(1.0));
    check!(line.len == CAPACITY);
    check!(*line.due_at(1.0) == command((CAPACITY + 1) as f32));
}
