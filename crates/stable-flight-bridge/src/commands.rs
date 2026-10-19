//! Commands a ground station sends the flight controller: see [`CommandService`].

use std::time::{Duration, Instant};

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::stable_flight::{
    ControlInput, ControlMode, FlightModeReport, FlightRegime, LandCommand, Vec2,
};
use stable_flight_link::{Control, Uplink};

use crate::encode;
use crate::outbox::Outbox;
use crate::setpoints;

#[cfg(test)]
mod tests;

/// What the service needs of a live flight controller.
#[derive(Debug, Clone, Copy)]
pub(crate) struct FcView {
    /// Armed, per its latest heartbeat.
    pub(crate) armed: bool,
    /// Its mode, per its latest heartbeat.
    pub(crate) mode: ControlMode,
    /// What it flies, per its latest heartbeat.
    pub(crate) flight_mode: FlightModeReport,
    /// Its latest horizontal position (north, east), if known and finite.
    pub(crate) position_m: Option<Vec2>,
}

/// What the flight controller's heartbeat must show to confirm a request.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Confirmation {
    Armed(bool),
    Mode(ControlMode),
    /// Flying or heading to `requested`; `flown` is the regime to withdraw to if it isn't.
    Regime {
        requested: FlightRegime,
        flown: FlightRegime,
    },
}

impl Confirmation {
    /// Whether a heartbeat reporting `armed`, `mode`, and `flight_mode` shows it.
    fn shown_by(self, armed: bool, mode: ControlMode, flight_mode: FlightModeReport) -> bool {
        match self {
            Confirmation::Armed(arm) => arm == armed,
            Confirmation::Mode(expected) => expected == mode,
            Confirmation::Regime { requested, .. } => flight_mode.mode.target() == requested,
        }
    }

    /// Whether a heartbeat reporting `flight_mode` refuses it.
    fn refused_by(self, flight_mode: FlightModeReport) -> bool {
        matches!(self, Confirmation::Regime { .. }) && flight_mode.refusal.is_some()
    }

    /// What to send when it fails, so nothing is left standing: a regime request goes back to
    /// the regime flown.
    fn withdrawal(self) -> Option<Uplink> {
        match self {
            Confirmation::Regime { flown, .. } => Some(Uplink::Control(Control::Regime(flown))),
            _ => None,
        }
    }
}

/// A request the flight controller hasn't confirmed yet.
#[derive(Debug, Clone, Copy)]
struct Pending {
    command: MavCmd,
    confirmation: Confirmation,
    deadline: Instant,
    sender: MavHeader,
}

/// Translates the ground station's commands for the flight controller and acks them:
///
/// - `MAV_CMD_COMPONENT_ARM_DISARM` (param1 1 arms, 0 disarms) goes out as `Control::Arm`. The
///   force variants in param2 (QGC's "Force Arm", 2989, and "Emergency Stop", 21196) act the
///   same: the flight controller has no arming checks to skip and honors every disarm. One for
///   the state the vehicle is already in is `ACCEPTED` at once.
/// - `MAV_CMD_NAV_LAND` (from offboard apps: QGC offers no Land for a generic autopilot) goes
///   out as a land at the vehicle's latest position; the latitude and longitude in param5/6
///   are ignored. The flight controller disarms on touchdown. A land while disarmed is
///   `DENIED`, one before any position has arrived is `TEMPORARILY_REJECTED`, and one while
///   already landing is `ACCEPTED` at once.
/// - `MAV_CMD_DO_VTOL_TRANSITION` (QGC's transition button, `Vehicle.cc:2524-2533`; param1
///   `MAV_VTOL_STATE_MC` or `_FW`) goes out as `Control::Regime` (`Copter` or `Plane`), for a
///   VTOL only (`UNSUPPORTED` otherwise). Any other param1, or one while disarmed, is `DENIED`;
///   one for the regime already flown or heading there is `ACCEPTED` at once. ArduPilot takes
///   it only in AUTO (`quadplane.cpp:2029-2063`); here it flies in any mode. A transition the
///   heartbeat reports refused (a [`ModeRefusal`](stable_flight_link::stable_flight::ModeRefusal),
///   which the flight controller also announces as status text) is `FAILED` at once.
///   A failed one is withdrawn, the regime flown requested again, so it neither starts later
///   on its own (a height refusal clears once the vehicle climbs) nor blocks a retry (an
///   abandoned transition waits for a `Copter` request, which QGC never sends while it shows
///   multirotor).
///
/// A request that goes out is acked `ACCEPTED` only once the flight controller's heartbeat
/// shows it (the arm state, the land mode, or the regime), and `FAILED` if it doesn't within
/// `ack_timeout`. QGC waits 1.2 s and never retries an arm, so the timeout stays under that.
/// A request with no live flight controller, or while another is pending, is
/// `TEMPORARILY_REJECTED`.
///
/// Every other command is left to the caller ([`on_command_long`](Self::on_command_long)
/// returns `false`).
#[derive(Debug)]
pub(crate) struct CommandService {
    ack_timeout: Duration,
    /// The vehicle flies both regimes, so it takes transitions.
    vtol: bool,
    pending: Option<Pending>,
}

impl CommandService {
    /// Fails an unconfirmed request after `ack_timeout`; takes transitions for a `vtol`.
    pub(crate) fn new(ack_timeout: Duration, vtol: bool) -> Self {
        Self {
            ack_timeout,
            vtol,
            pending: None,
        }
    }

    /// Handles a `COMMAND_LONG` from `sender` received at `now`, with the flight controller if
    /// it is alive. Returns whether the command was this service's.
    pub(crate) fn on_command_long(
        &mut self,
        sender: &MavHeader,
        data: &COMMAND_LONG_DATA,
        fc: Option<FcView>,
        now: Instant,
        out: &mut Outbox,
    ) -> bool {
        let request = match data.command {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => arm_disarm(data.param1 >= 0.5, fc),
            MavCmd::MAV_CMD_NAV_LAND => land(fc),
            MavCmd::MAV_CMD_DO_VTOL_TRANSITION => transition(data.param1, self.vtol, fc),
            _ => return false,
        };
        let request = if self.pending.is_some() {
            Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED)
        } else {
            request
        };
        match request {
            Ok((uplink, confirmation)) => {
                out.to_fc.push(uplink);
                self.pending = Some(Pending {
                    command: data.command,
                    confirmation,
                    deadline: now + self.ack_timeout,
                    sender: *sender,
                });
            }
            Err(result) => ack(sender, data.command, result, out),
        }
        true
    }

    /// The flight controller's heartbeat reported `armed` in `mode`, flying `flight_mode`:
    /// confirms a matching request, or fails one it refuses.
    pub(crate) fn on_fc_heartbeat(
        &mut self,
        armed: bool,
        mode: ControlMode,
        flight_mode: FlightModeReport,
        out: &mut Outbox,
    ) {
        let Some(pending) = self.pending else {
            return;
        };
        if pending.confirmation.shown_by(armed, mode, flight_mode) {
            self.finish(pending, MavResult::MAV_RESULT_ACCEPTED, out);
        } else if pending.confirmation.refused_by(flight_mode) {
            self.finish(pending, MavResult::MAV_RESULT_FAILED, out);
        }
    }

    /// Fails a request still unconfirmed at `now`.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        if let Some(pending) = self.pending.filter(|p| now >= p.deadline) {
            self.finish(pending, MavResult::MAV_RESULT_FAILED, out);
        }
    }

    /// Acks `pending` with `result`, withdrawing it unless accepted.
    fn finish(&mut self, pending: Pending, result: MavResult, out: &mut Outbox) {
        ack(&pending.sender, pending.command, result, out);
        if result != MavResult::MAV_RESULT_ACCEPTED {
            out.to_fc.extend(pending.confirmation.withdrawal());
        }
        self.pending = None;
    }
}

/// What to send for a request, and the heartbeat that confirms it; or the ack to answer with
/// at once.
type Request = Result<(Uplink, Confirmation), MavResult>;

/// An arm (`arm`) or disarm request.
fn arm_disarm(arm: bool, fc: Option<FcView>) -> Request {
    match fc {
        None => Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED),
        Some(fc) if fc.armed == arm => Err(MavResult::MAV_RESULT_ACCEPTED),
        Some(_) => Ok((Uplink::Control(Control::Arm(arm)), Confirmation::Armed(arm))),
    }
}

/// A land at the vehicle's latest position.
fn land(fc: Option<FcView>) -> Request {
    let Some(fc) = fc else {
        return Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
    };
    if !fc.armed {
        return Err(MavResult::MAV_RESULT_DENIED);
    }
    if fc.mode == ControlMode::Land {
        return Err(MavResult::MAV_RESULT_ACCEPTED);
    }
    let Some(position_m) = fc.position_m else {
        return Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
    };
    Ok((
        setpoints::uplink(ControlInput::Land(LandCommand { position_m })),
        Confirmation::Mode(ControlMode::Land),
    ))
}

/// A VTOL transition to `MAV_VTOL_STATE` `param1`, on a `vtol`.
fn transition(param1: f32, vtol: bool, fc: Option<FcView>) -> Request {
    if !vtol {
        return Err(MavResult::MAV_RESULT_UNSUPPORTED);
    }
    let requested = match param1 {
        p if p == MavVtolState::MAV_VTOL_STATE_MC as u8 as f32 => FlightRegime::Copter,
        p if p == MavVtolState::MAV_VTOL_STATE_FW as u8 as f32 => FlightRegime::Plane,
        _ => return Err(MavResult::MAV_RESULT_DENIED),
    };
    let Some(fc) = fc else {
        return Err(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
    };
    if !fc.armed {
        return Err(MavResult::MAV_RESULT_DENIED);
    }
    let flown = fc.flight_mode.mode.target();
    if flown == requested {
        return Err(MavResult::MAV_RESULT_ACCEPTED);
    }
    Ok((
        Uplink::Control(Control::Regime(requested)),
        Confirmation::Regime { requested, flown },
    ))
}

/// Queues an ack of `command` with `result` for `sender`.
fn ack(sender: &MavHeader, command: MavCmd, result: MavResult, out: &mut Outbox) {
    let ack = encode::command_ack(sender, command, result);
    out.to_gcs.push(MavMessage::COMMAND_ACK(ack));
}
