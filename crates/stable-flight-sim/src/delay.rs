//! A fixed-depth delay line on the actuator commands: see [`DelayLine`].

use stable_flight::ActuatorCommand;

#[cfg(test)]
mod tests;

/// Commands waiting out the delay. 64 cover [`MAX_ACTUATOR_DELAY_S`](crate::MAX_ACTUATOR_DELAY_S)
/// (50 ms) at ticks of 0.8 ms or longer.
const CAPACITY: usize = 64;

/// Slack on a command's due time, in s: substep times add `f32` steps, so a command due on a
/// step boundary can land a rounding error short of it.
const DUE_TOLERANCE_S: f64 = 1e-6;

/// Actuator commands, each applied `delay_s` after it was pushed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct DelayLine {
    delay_s: f64,
    /// The command the actuators run on now.
    current: ActuatorCommand,
    /// Commands not yet due, oldest first from `head`: (push time in s, command).
    pending: [(f64, ActuatorCommand); CAPACITY],
    head: usize,
    len: usize,
}

impl DelayLine {
    /// An empty line delaying by `delay_s`; the actuators start at zero.
    pub(crate) fn new(delay_s: f32) -> Self {
        DelayLine {
            delay_s: f64::from(delay_s),
            current: ActuatorCommand::default(),
            pending: [(0.0, ActuatorCommand::default()); CAPACITY],
            head: 0,
            len: 0,
        }
    }

    /// Sets the delay for commands not yet applied.
    pub(crate) fn set_delay(&mut self, delay_s: f32) {
        self.delay_s = f64::from(delay_s);
    }

    /// Queues `command`, pushed at `time_s`. A full line applies its oldest command early to
    /// make room (ticks shorter than the delay over [`CAPACITY`]).
    pub(crate) fn push(&mut self, time_s: f64, command: ActuatorCommand) {
        if self.len == CAPACITY {
            self.pop();
        }
        self.pending[(self.head + self.len) % CAPACITY] = (time_s, command);
        self.len += 1;
    }

    /// Applies every command due by `time_s` (pushed at least `delay_s` earlier) and returns the
    /// one the actuators now run on: the newest of them, or the last applied if none came due.
    pub(crate) fn due_at(&mut self, time_s: f64) -> &ActuatorCommand {
        let due_s = time_s - self.delay_s + DUE_TOLERANCE_S;
        while self.len > 0 && self.pending[self.head].0 <= due_s {
            self.pop();
        }
        &self.current
    }

    /// Applies the oldest pending command.
    fn pop(&mut self) {
        self.current = self.pending[self.head].1;
        self.head = (self.head + 1) % CAPACITY;
        self.len -= 1;
    }
}
