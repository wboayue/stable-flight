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
/// Any message counts as hearing the link: [`since_update_s`](LinkedCommander::since_update_s)
/// is 0 on a tick that drained one, grows by `dt` otherwise, and is `None` until the first.
/// Wrap it in a [`FailsafeCommander`](stable_flight::commanders::FailsafeCommander) so a silent
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
    armed: bool,
    since_update_s: Option<f32>,
}

impl<'a> LinkCommander<'a> {
    /// A commander draining `control`, disarmed and never heard.
    pub fn new(control: Consumer<'a, Control>) -> Self {
        Self {
            control,
            input: ControlInput::default(),
            armed: false,
            since_update_s: None,
        }
    }
}

impl Commander for LinkCommander<'_> {
    fn next_command(&mut self, dt: f32, _current_state: &VehicleState) -> Command {
        let mut heard = false;
        while let Some(message) = self.control.dequeue() {
            heard = true;
            match message {
                Control::Command(command) => self.input = command.input,
                Control::Arm(arm) => self.armed = arm,
                Control::Heartbeat => {}
            }
        }
        self.since_update_s = match (heard, self.since_update_s) {
            (true, _) => Some(0.0),
            (false, since) => since.map(|s| s + dt),
        };
        Command::new(self.armed, self.input)
    }
}

impl LinkedCommander for LinkCommander<'_> {
    fn since_update_s(&self) -> Option<f32> {
        self.since_update_s
    }
}
