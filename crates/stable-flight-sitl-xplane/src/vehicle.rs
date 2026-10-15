use std::time::Duration;

use log::{info, warn};
use stable_flight::{Actuator, ActuatorCommand, StateSource, VehicleState};
use thiserror::Error;
use xplane_bridge::datarefs::{engine, overrides, wing};
use xplane_bridge::{Dataref, DatarefSink, DatarefSource, Latest, Writable, XPlaneError};

use crate::{
    ActuatorMapping, Anchor, LocalOrigin, MappingError, OriginEvent, StateDatarefs, XPlaneSample,
    vehicle_state, wait_ready,
};
use health::{Liveness, SILENCE_S};

mod health;
#[cfg(test)]
mod tests;

/// How long [`XPlaneVehicle::connect`] waits for every state dataref by default.
pub const DEFAULT_READY_TIMEOUT: Duration = Duration::from_secs(10);

/// Why [`XPlaneVehicle::connect`] (or [`wait_ready`]) failed.
#[derive(Debug, Error)]
pub enum ConnectError {
    /// The actuator mapping is invalid.
    #[error("invalid actuator mapping: {0}")]
    Mapping(#[from] MappingError),
    /// The bridge failed to send or receive.
    #[error("X-Plane bridge failed: {0}")]
    Bridge(#[from] XPlaneError),
    /// Some datarefs had no value by the deadline: X-Plane isn't running at that address, or
    /// doesn't know a dataref.
    #[error("no value from X-Plane within {timeout:?} for: {}", missing.join(", "))]
    NotReady {
        /// The datarefs that never arrived.
        missing: Vec<&'static str>,
        /// How long it waited.
        timeout: Duration,
    },
}

/// What [`XPlaneVehicle::connect`] needs: the bridge, the actuator mapping, and how long to
/// wait for X-Plane.
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl_xplane::xplane_bridge::XPlaneUdpBridge;
/// use stable_flight_sitl_xplane::{ActuatorMapping, XPlaneConfiguration};
///
/// let bridge = XPlaneUdpBridge::connect("127.0.0.1:49000").expect("Failed to open socket");
/// let config = XPlaneConfiguration::new(bridge, ActuatorMapping::default());
/// ```
#[derive(Debug)]
pub struct XPlaneConfiguration<B> {
    /// The bridge to X-Plane.
    pub bridge: B,
    /// Which engines and wings the mixer's outputs drive.
    pub mapping: ActuatorMapping,
    /// How long [`XPlaneVehicle::connect`] waits for every state dataref.
    pub ready_timeout: Duration,
}

impl<B> XPlaneConfiguration<B> {
    /// `bridge` and `mapping`, waiting [`DEFAULT_READY_TIMEOUT`] for X-Plane.
    pub fn new(bridge: B, mapping: ActuatorMapping) -> Self {
        XPlaneConfiguration {
            bridge,
            mapping,
            ready_timeout: DEFAULT_READY_TIMEOUT,
        }
    }
}

/// A vehicle flown in X-Plane over [`xplane_bridge`].
///
/// [`connect`](Self::connect) subscribes the state ([`StateDatarefs`]), waits until every
/// dataref has a value, zeroes every mapped and idle engine, and takes control:
/// `override_throttles` and, with [`ActuatorMapping::neutral_wings`], `override_control_surfaces` with those
/// wings' surfaces at 0°. Dropping the vehicle writes the overrides back to 0, handing control
/// back to X-Plane, before the bridge unsubscribes.
///
/// # Timing
///
/// X-Plane sends state at most once per rendered frame (60–120 Hz) and isn't stepped by the
/// controller, so it runs under the real-time runner, not lockstep. [`StateSource::state`]
/// polls the bridge without blocking: when a frame arrived since the last tick, it converts
/// the newest values; otherwise it returns the last state again, so a 250 Hz loop sees each
/// state for two to four ticks. State is one frame plus the UDP transit old (about 8–17 ms,
/// plus Wi-Fi's few ms to a second host). [`Actuator::actuate`] writes each mapped engine
/// every tick; X-Plane applies the last write before each frame, so actuator latency is also
/// up to one frame. See the core `Vehicle` docs for the cross-backend contract.
///
/// # Liveness
///
/// UDP has no connection, so the vehicle watches the stream:
///
/// - **Silence:** no frame for 0.5 s (of the ticks' nominal `dt`) logs a warning and
///   resubscribes every 0.25 s until frames return, which covers an X-Plane restart (it forgets
///   every subscription). When frames return, it logs the silence's length and takes control
///   again (a restarted X-Plane has also cleared the overrides), without zeroing the engines. `state` keeps returning the
///   last state meanwhile.
/// - **Lost subscriptions:** X-Plane drops `RREF`s that arrive in a burst, so while frames flow,
///   a dataref that hasn't arrived for 0.5 s also triggers a resubscribe every 0.25 s.
/// - **Pause:** X-Plane keeps streaming frozen values while paused; the vehicle logs the pause
///   and its end.
/// - **Errors:** a failed send or receive is logged at most once a second.
///
/// # Frames and origin
///
/// [`vehicle_state`] converts each sample; [`LocalOrigin`] keeps position relative to where
/// the aircraft first was ([`anchor`](Self::anchor)), continuous when X-Plane moves its local
/// origin, re-anchored when the aircraft relocates. Each anchor is logged with the bridge's
/// `--origin` for it.
///
/// # Examples
///
/// ```no_run
/// use stable_flight::{Actuator, ActuatorCommand, StateSource};
/// use stable_flight_sitl_xplane::xplane_bridge::XPlaneUdpBridge;
/// use stable_flight_sitl_xplane::{ActuatorMapping, XPlaneConfiguration, XPlaneVehicle};
/// use std::time::Duration;
///
/// let mapping = ActuatorMapping {
///     motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
///     ..Default::default()
/// };
/// let bridge = XPlaneUdpBridge::connect("127.0.0.1:49000").expect("Failed to open socket");
/// let mut vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(bridge, mapping))
///     .expect("X-Plane not ready");
///
/// let state = vehicle.state(Duration::ZERO);
/// vehicle.actuate(&ActuatorCommand::default());
/// ```
pub struct XPlaneVehicle<B: DatarefSource + DatarefSink> {
    bridge: B,
    datarefs: StateDatarefs,
    latest: Latest,
    state: VehicleState,
    origin: LocalOrigin,
    mapping: ActuatorMapping,
    /// `ENGN_thro_use`, one dataref per element, built once.
    engines: Vec<Dataref<f32, Writable>>,
    /// The surfaces held at 0°.
    neutral_surfaces: Vec<Dataref<f32, Writable>>,
    /// Whether the last sample was paused.
    paused: bool,
    /// As of the last frame, a dataref is missing or lagging the others (see
    /// [`StateDatarefs::incomplete`]): only a frame can change it.
    lagging: bool,
    liveness: Liveness,
}

impl<B: DatarefSource + DatarefSink> XPlaneVehicle<B> {
    /// Subscribes the state, waits for every dataref (resubscribing every 250 ms; blocks up to
    /// `ready_timeout`), and takes control (see [`XPlaneVehicle`]).
    ///
    /// # Errors
    ///
    /// [`ConnectError::Mapping`] before touching X-Plane; [`ConnectError::NotReady`] naming the
    /// missing datarefs after the timeout; [`ConnectError::Bridge`] when the bridge fails.
    pub fn connect(config: XPlaneConfiguration<B>) -> Result<Self, ConnectError> {
        let XPlaneConfiguration {
            mut bridge,
            mapping,
            ready_timeout,
        } = config;
        mapping.validate()?;

        let datarefs = StateDatarefs::subscribe(&mut bridge)?;
        let mut latest = Latest::default();
        let sample = wait_ready(&mut bridge, &datarefs, &mut latest, ready_timeout)?;

        let surfaces = [
            wing::AILERON1_DEG,
            wing::AILERON2_DEG,
            wing::ELEVATOR1_DEG,
            wing::ELEVATOR2_DEG,
            wing::RUDDER1_DEG,
            wing::RUDDER2_DEG,
        ];
        let neutral_surfaces = mapping
            .neutral_wings
            .iter()
            .flat_map(|&w| surfaces.iter().filter_map(move |surface| surface.at(w)))
            .collect();
        let mut vehicle = XPlaneVehicle {
            bridge,
            datarefs,
            latest,
            state: VehicleState::default(),
            origin: LocalOrigin::default(),
            mapping,
            engines: engine::THRO_USE.iter().collect(),
            neutral_surfaces,
            paused: false,
            lagging: false,
            liveness: Liveness::default(),
        };
        vehicle.take_control()?;
        vehicle.zero_engines()?;
        vehicle.update(&sample);
        Ok(vehicle)
    }

    /// Where position (0, 0) is: the bridge's `--origin` is `latitude,longitude,0`.
    pub fn anchor(&self) -> Option<Anchor> {
        self.origin.anchor()
    }

    /// The bridge, for its statistics.
    pub fn bridge(&self) -> &B {
        &self.bridge
    }

    /// Sets the overrides and holds the neutral surfaces at 0°. Leaves the engines alone: on a
    /// stall mid-flight, zeros sent here could land a frame before the tick's throttles.
    fn take_control(&mut self) -> Result<(), XPlaneError> {
        self.bridge.write(&overrides::THROTTLES, true)?;
        if !self.neutral_surfaces.is_empty() {
            self.bridge.write(&overrides::CONTROL_SURFACES, true)?;
            for surface in &self.neutral_surfaces {
                self.bridge.write(surface, 0.0)?;
            }
        }
        Ok(())
    }

    /// Zeroes every engine the mapping writes, so none keeps X-Plane's last throttle before the
    /// first tick.
    fn zero_engines(&mut self) -> Result<(), XPlaneError> {
        for engine in self.mapping.engines() {
            if let Some(dataref) = self.engines.get(engine) {
                self.bridge.write(dataref, 0.0)?;
            }
        }
        Ok(())
    }

    /// Converts a new sample, tracking the origin and the pause.
    fn update(&mut self, sample: &XPlaneSample) {
        if sample.paused != self.paused {
            self.paused = sample.paused;
            info!(
                "X-Plane {}",
                if sample.paused { "paused" } else { "resumed" }
            );
        }
        match self.origin.update(sample) {
            Some(OriginEvent::Anchored(anchor)) => {
                info!("Position anchored at {}", origin_line(&anchor));
            }
            Some(OriginEvent::Relocated(anchor)) => {
                warn!(
                    "Aircraft relocated; position re-anchored at {}",
                    origin_line(&anchor)
                );
            }
            Some(OriginEvent::Shifted { by_m }) => {
                info!(
                    "X-Plane moved its local origin; offset changed by ({:.0}, {:.0}) m",
                    by_m.x, by_m.y
                );
            }
            None => {}
        }
        self.state = vehicle_state(sample, self.origin.offset_m());
    }

    /// Polls once; on a frame, applies it and converts the sample.
    fn poll(&mut self) {
        let frame = match self.bridge.poll() {
            Ok(frame) => frame,
            Err(e) => {
                self.liveness
                    .report(format_args!("Failed to receive from X-Plane: {e}"));
                return;
            }
        };
        if let Some(frame) = frame {
            self.latest.apply(&frame);
            self.lagging = self
                .datarefs
                .incomplete(&self.latest, Duration::from_secs_f32(SILENCE_S));
            if let Some(silence_s) = self.liveness.frame_arrived() {
                info!("X-Plane streaming again after {silence_s:.1} s; taking control");
                if let Err(e) = self.take_control() {
                    self.liveness
                        .report(format_args!("Failed to take control of X-Plane: {e}"));
                }
            }
            if let Some(sample) = self.datarefs.sample(&self.latest) {
                self.update(&sample);
            }
        }
    }
}

/// The anchor and the bridge's `--origin` for it.
fn origin_line(anchor: &Anchor) -> String {
    format!(
        "{:.7}, {:.7} (bridge --origin {:.7},{:.7},0)",
        anchor.latitude_deg, anchor.longitude_deg, anchor.latitude_deg, anchor.longitude_deg
    )
}

impl<B: DatarefSource + DatarefSink> StateSource for XPlaneVehicle<B> {
    /// Polls X-Plane once without blocking and returns the newest state (see
    /// [Timing](XPlaneVehicle#timing)). `dt` advances the silence and resubscribe timers only:
    /// the simulator reports its own truth state.
    fn state(&mut self, dt: Duration) -> VehicleState {
        self.liveness.advance(dt.as_secs_f32());
        self.poll();

        if self.liveness.silence_started() {
            warn!("No state from X-Plane for {SILENCE_S} s; resubscribing until it returns");
        }
        let incomplete = self.liveness.is_silent() || self.lagging;
        if self.liveness.resubscribe_due(incomplete)
            && let Err(e) = self.bridge.resubscribe()
        {
            self.liveness
                .report(format_args!("Failed to resubscribe to X-Plane: {e}"));
        }

        self.state.clone()
    }
}

impl<B: DatarefSource + DatarefSink> Actuator for XPlaneVehicle<B> {
    /// Writes each mapped engine's throttle and zeroes each idle engine.
    fn actuate(&mut self, command: &ActuatorCommand) {
        for (engine, throttle) in self.mapping.throttles(command) {
            let Some(dataref) = self.engines.get(engine) else {
                continue;
            };
            if let Err(e) = self.bridge.write(dataref, throttle) {
                self.liveness
                    .report(format_args!("Failed to write to X-Plane: {e}"));
                return;
            }
        }
    }
}

impl<B: DatarefSource + DatarefSink> Drop for XPlaneVehicle<B> {
    /// Hands control back to X-Plane: writes the overrides it set back to 0. Logs a warning if
    /// a write fails.
    fn drop(&mut self) {
        if let Err(e) = self.bridge.write(&overrides::THROTTLES, false) {
            warn!("Failed to release X-Plane's throttles: {e}");
        }
        if !self.neutral_surfaces.is_empty()
            && let Err(e) = self.bridge.write(&overrides::CONTROL_SURFACES, false)
        {
            warn!("Failed to release X-Plane's control surfaces: {e}");
        }
    }
}
