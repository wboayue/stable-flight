//! How long the bridge sleeps between passes: see [`IdleBackoff`].

use std::time::Duration;

use crate::pump::Activity;

#[cfg(test)]
mod tests;

/// Sleep after a pass that received something: traffic is served within a millisecond.
const MIN_SLEEP: Duration = Duration::from_millis(1);
/// Longest sleep while idle: bounds the delay of the first message after a quiet spell.
const MAX_SLEEP: Duration = Duration::from_millis(8);

/// Sleeps [`MIN_SLEEP`] while traffic flows and backs off, doubling up to [`MAX_SLEEP`], while
/// idle: about 125 wakeups a second with nothing to do, instead of 1000.
#[derive(Debug)]
pub(crate) struct IdleBackoff {
    sleep: Duration,
}

impl Default for IdleBackoff {
    fn default() -> Self {
        Self { sleep: MIN_SLEEP }
    }
}

impl IdleBackoff {
    /// How long to sleep after a pass that did `activity`.
    pub(crate) fn next(&mut self, activity: &Activity) -> Duration {
        self.sleep = if activity.received() {
            MIN_SLEEP
        } else {
            (self.sleep * 2).min(MAX_SLEEP)
        };
        self.sleep
    }
}
