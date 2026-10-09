//! What the bridge's services queue to send: see [`Outbox`].

use mavlink::common::MavMessage;
use stable_flight_link::Uplink;

/// What to send after handling a message: uplinks to the flight controller and MAVLink to the
/// ground station, stamped with the flight controller's ids. The caller drains both after each
/// call.
#[derive(Debug, Default)]
pub(crate) struct Outbox {
    pub(crate) to_fc: Vec<Uplink>,
    pub(crate) to_gcs: Vec<MavMessage>,
}
