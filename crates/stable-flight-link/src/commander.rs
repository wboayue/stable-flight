//! Flying from the link: see [`LinkCommander`].

use heapless::spsc::Consumer;
use stable_flight::commanders::LinkedCommander;
use stable_flight::{Command, Commander, ControlInput, VehicleState};

use crate::Control;

#[cfg(test)]
mod tests;

/// A [`Commander`] fed by the link: the [`Control`] messages an
/// [`UplinkRouter`](crate::router::UplinkRouter) queues for it (e.g. through
/// [`FcLink::wire_linked`](crate::fc::FcLink::wire_linked)). It never touches the transport.
///
/// Each tick it drains its queue in arrival order:
///
/// - [`Control::Command`]: what to fly. The latest one wins; its `arm` field is ignored, so a
///   stream of setpoints can't arm or disarm the vehicle as a side effect.
/// - [`Control::Arm`]: arms (`true`) or disarms (`false`); latched until the next one.
/// - [`Control::Heartbeat`]: only keeps the link fresh.
///
/// Freshness ([`since_update_s`](LinkedCommander::since_update_s), 0 on a tick that drained a
/// message that counts, growing by `dt` otherwise, `None` until the first) depends on what it
/// flies:
///
/// - a **waypoint**, or nothing yet: any message counts, heartbeats included, because flying
///   the last waypoint on a quiet but live link is safe;
/// - **sticks** (an attitude or rate command): only commands count, so a joystick stream that
///   stops (the ground station's joystick turned off, an app frozen) goes stale even while the
///   link heartbeats, rather than holding the last stick position.
///
/// Wrap it in a [`FailsafeCommander`](stable_flight::commanders::FailsafeCommander) so a stale
/// link falls back. Until a command arrives it flies a neutral [`ControlInput::default`], and
/// it starts disarmed.
///
/// # Examples
///
/// ```
/// use stable_flight_link::commander::LinkCommander;
/// use stable_flight_link::heapless::spsc::Queue;
/// use stable_flight_link::stable_flight::commanders::LinkedCommander;
/// use stable_flight_link::stable_flight::{Commander, VehicleState};
/// use stable_flight_link::Control;
///
/// let mut queue = Queue::<Control, 8>::new();
/// let (mut router_end, commander_end) = queue.split();
/// let mut commander = LinkCommander::new(commander_end);
/// assert_eq!(commander.since_update_s(), None);
///
/// router_end.enqueue(Control::Arm(true)).unwrap();
/// let command = commander.next_command(0.004, &VehicleState::default());
/// assert!(command.arm);
/// assert_eq!(commander.since_update_s(), Some(0.0));
/// ```
pub struct LinkCommander<'a> {
    control: Consumer<'a, Control>,
    input: ControlInput,
    /// The latest command was sticks (attitude or rate), so only commands keep it fresh.
    flying_sticks: bool,
    armed: bool,
    /// Since any message.
    since_message_s: Option<f32>,
    /// Since a command.
    since_command_s: Option<f32>,
}

impl<'a> LinkCommander<'a> {
    /// A commander draining `control`, disarmed and never heard.
    pub fn new(control: Consumer<'a, Control>) -> Self {
        Self {
            control,
            input: ControlInput::default(),
            flying_sticks: false,
            armed: false,
            since_message_s: None,
            since_command_s: None,
        }
    }
}

impl Commander for LinkCommander<'_> {
    fn next_command(&mut self, dt: f32, _current_state: &VehicleState) -> Command {
        let (mut message, mut command) = (false, false);
        while let Some(control) = self.control.dequeue() {
            message = true;
            match control {
                Control::Command(c) => {
                    command = true;
                    self.input = c.input;
                    self.flying_sticks =
                        matches!(c.input, ControlInput::Attitude(_) | ControlInput::Rate(_));
                }
                Control::Arm(arm) => self.armed = arm,
                Control::Heartbeat => {}
            }
        }
        self.since_message_s = since(message, self.since_message_s, dt);
        self.since_command_s = since(command, self.since_command_s, dt);
        Command::new(self.armed, self.input)
    }
}

impl LinkedCommander for LinkCommander<'_> {
    fn since_update_s(&self) -> Option<f32> {
        if self.flying_sticks {
            self.since_command_s
        } else {
            self.since_message_s
        }
    }
}

/// Time since something was last heard: 0 if it was this tick, else `last` grown by `dt`.
fn since(heard: bool, last: Option<f32>, dt: f32) -> Option<f32> {
    if heard {
        Some(0.0)
    } else {
        last.map(|s| s + dt)
    }
}
