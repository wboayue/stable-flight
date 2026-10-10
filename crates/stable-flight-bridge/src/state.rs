//! The flight controller's state as MAVLink telemetry: see [`StateService`].

use std::time::{Duration, Instant};

use mavlink::common::*;
use stable_flight_link::Record;
use stable_flight_link::stable_flight::{ControlMode, Vec2, VehicleState};
use stable_flight_mavlink::{FlatEarth, GroundTracker};

use crate::interval::Interval;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// Sends the flight controller's latest [`VehicleState`] to the ground station (the payloads,
/// home, and landed state come from `stable-flight-mavlink`):
///
/// - every `state_interval`: `ATTITUDE`, `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `VFR_HUD`;
/// - every `slow_interval`, and as soon as either changes: `HOME_POSITION`, `EXTENDED_SYS_STATE`.
///
/// The flight controller streams state faster than this (50 Hz by default); only the latest is
/// kept, but every one updates home and the landed state, with the arm state from its latest
/// heartbeat. `GLOBAL_POSITION_INT` and `HOME_POSITION` need an origin ([`FlatEarth`]) and a
/// home. `VFR_HUD`'s throttle is the commanded thrust from the latest [`Record`], 0 until one
/// arrives (the flight controller sends none by default).
///
/// Nothing goes out until the first state arrives. `time_boot_ms` counts from when the flight
/// controller first appeared: the link carries no flight-controller clock. The caller polls
/// only while the flight controller is alive, so stale state never goes out.
#[derive(Debug)]
pub(crate) struct StateService {
    earth: Option<FlatEarth>,
    /// When the flight controller first appeared: the zero of `time_boot_ms`.
    boot: Option<Instant>,
    session: Session,
    stream: Interval,
    slow: Interval,
}

/// What the flight controller has told the service since it last appeared; a restart may have
/// changed all of it, so it starts over each time.
#[derive(Debug, Default)]
struct Session {
    latest: Option<VehicleState>,
    /// Normalized commanded thrust from the latest record.
    thrust: f32,
    ground: GroundTracker,
    /// Home was latched since the last poll: send `HOME_POSITION` now.
    home_changed: bool,
    /// The landed state changed since the last poll: send `EXTENDED_SYS_STATE` now.
    landed_changed: bool,
}

impl StateService {
    /// Projects global position through `earth` (none sends no global position or home), with
    /// streams every `state_interval` and the slow ones every `slow_interval`.
    pub(crate) fn new(
        earth: Option<FlatEarth>,
        state_interval: Duration,
        slow_interval: Duration,
    ) -> Self {
        Self {
            earth,
            boot: None,
            session: Session::default(),
            stream: Interval::new(state_interval),
            slow: Interval::new(slow_interval),
        }
    }

    /// The flight controller appeared (or came back) at `now`. It may have restarted, so start
    /// over: no state, home, or thrust until it sends them, then every stream at once.
    pub(crate) fn fc_appeared(&mut self, now: Instant, _out: &mut Outbox) {
        self.boot.get_or_insert(now);
        self.session = Session::default();
        self.stream.start(now);
        self.slow.start(now);
    }

    /// A state from the flight controller, whose latest heartbeat reported `armed` in `mode`.
    pub(crate) fn on_state(&mut self, state: &VehicleState, armed: bool, mode: ControlMode) {
        let session = &mut self.session;
        let changes = session.ground.update(armed, mode, state.position_m);
        session.home_changed |= changes.home;
        session.landed_changed |= changes.landed;
        session.latest = Some(state.clone());
    }

    /// The latest state's horizontal position (north, east), if finite; none before a state
    /// since the flight controller last appeared.
    pub(crate) fn position_m(&self) -> Option<Vec2> {
        let position_m = self.session.latest.as_ref()?.position_m.xy();
        position_m
            .iter()
            .all(|c| c.is_finite())
            .then_some(position_m)
    }

    /// A record from the flight controller: its commanded thrust, for `VFR_HUD`.
    pub(crate) fn on_record(&mut self, record: &Record) {
        self.session.thrust = record.control.thrust;
    }

    /// Sends what is due at `now` from the latest state.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        let session = &mut self.session;
        let Some(state) = &session.latest else {
            return;
        };
        // Truncates after about 49 days, like any `time_boot_ms`.
        let time_boot_ms = self.boot.map_or(0, |boot| {
            now.saturating_duration_since(boot).as_millis() as u32
        });
        let home_m = session.ground.home_m();

        if self.stream.due(now) {
            out.to_gcs
                .push(MavMessage::ATTITUDE(stable_flight_mavlink::attitude(
                    state,
                    time_boot_ms,
                )));
            out.to_gcs.push(MavMessage::LOCAL_POSITION_NED(
                stable_flight_mavlink::local_position_ned(state, time_boot_ms),
            ));
            if let Some(earth) = &self.earth
                && let Some(home_m) = home_m
                && let Some(data) =
                    stable_flight_mavlink::global_position_int(earth, state, home_m.z, time_boot_ms)
            {
                out.to_gcs.push(MavMessage::GLOBAL_POSITION_INT(data));
            }
            out.to_gcs
                .push(MavMessage::VFR_HUD(stable_flight_mavlink::vfr_hud(
                    state,
                    session.thrust,
                )));
        }

        // `|`, not `||`: the timer advances on every poll.
        let slow_due = self.slow.due(now);
        if slow_due | std::mem::take(&mut session.home_changed)
            && let Some(earth) = &self.earth
            && let Some(home_m) = home_m
        {
            out.to_gcs.push(MavMessage::HOME_POSITION(
                stable_flight_mavlink::home_position(earth, home_m),
            ));
        }
        if slow_due | std::mem::take(&mut session.landed_changed) {
            out.to_gcs.push(MavMessage::EXTENDED_SYS_STATE(
                stable_flight_mavlink::extended_sys_state(session.ground.landed_state()),
            ));
        }
    }
}
