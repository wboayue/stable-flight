//! The flight controller's state as MAVLink telemetry: see [`StateService`].

use std::time::{Duration, Instant};

use mavlink::common::*;
use stable_flight_link::Record;
use stable_flight_link::stable_flight::VehicleState;
use stable_flight_mavlink::{FlatEarth, GroundTracker};

use crate::interval::Interval;
use crate::outbox::Outbox;

#[cfg(test)]
mod tests;

/// Sends the flight controller's latest [`VehicleState`] to the ground station, as sitl's
/// `MavlinkPublisher` does in process (the payloads, home, and landed state come from the same
/// `stable-flight-mavlink` code):
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
    latest: Option<VehicleState>,
    /// Normalized commanded thrust from the latest record.
    thrust: f32,
    ground: GroundTracker,
    /// Home was latched since the last poll: send `HOME_POSITION` now.
    home_changed: bool,
    /// The landed state changed since the last poll: send `EXTENDED_SYS_STATE` now.
    landed_changed: bool,
    stream: Interval,
    slow: Interval,
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
            latest: None,
            thrust: 0.0,
            ground: GroundTracker::default(),
            home_changed: false,
            landed_changed: false,
            stream: Interval::new(state_interval),
            slow: Interval::new(slow_interval),
        }
    }

    /// The flight controller appeared (or came back) at `now`. It may have restarted, so start
    /// over: no state, home, or thrust until it sends them, then every stream at once.
    pub(crate) fn fc_appeared(&mut self, now: Instant, _out: &mut Outbox) {
        self.boot.get_or_insert(now);
        self.latest = None;
        self.thrust = 0.0;
        self.ground = GroundTracker::default();
        self.home_changed = false;
        self.landed_changed = false;
        self.stream.start(now);
        self.slow.start(now);
    }

    /// A state from the flight controller, whose latest heartbeat reported `armed`.
    pub(crate) fn on_state(&mut self, state: &VehicleState, armed: bool) {
        let changes = self.ground.update(armed, state.position_m);
        self.home_changed |= changes.home;
        self.landed_changed |= changes.landed;
        self.latest = Some(state.clone());
    }

    /// A record from the flight controller: its commanded thrust, for `VFR_HUD`.
    pub(crate) fn on_record(&mut self, record: &Record) {
        self.thrust = record.control.thrust;
    }

    /// Sends what is due at `now` from the latest state.
    pub(crate) fn poll(&mut self, now: Instant, out: &mut Outbox) {
        let Some(state) = &self.latest else {
            return;
        };
        // Truncates after about 49 days, like any `time_boot_ms`.
        let time_boot_ms = self.boot.map_or(0, |boot| {
            now.saturating_duration_since(boot).as_millis() as u32
        });
        let home_m = self.ground.home_m();

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
                    self.thrust,
                )));
        }

        // `|`, not `||`: the timer advances on every poll.
        let slow_due = self.slow.due(now);
        if slow_due | std::mem::take(&mut self.home_changed)
            && let Some(earth) = &self.earth
            && let Some(home_m) = home_m
        {
            out.to_gcs.push(MavMessage::HOME_POSITION(
                stable_flight_mavlink::home_position(earth, home_m),
            ));
        }
        if slow_due | std::mem::take(&mut self.landed_changed) {
            out.to_gcs.push(MavMessage::EXTENDED_SYS_STATE(
                stable_flight_mavlink::extended_sys_state(self.ground.landed_state()),
            ));
        }
    }
}
