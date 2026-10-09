//! Serving the flight controller's [`Params`] over the link.
//!
//! Requests arrive inside the tick (the commander decodes them) but can only be applied between
//! ticks, where something holds `&mut` to the [`FlightLoop`](stable_flight::FlightLoop). Two
//! bounded single-producer queues bridge that gap:
//!
//! ```text
//! commander ──ParamRequest──▶ ParamServer::poll (between ticks) ──ParamValue──▶ publisher
//! ```
//!
//! [`ParamServer::poll`] applies at most a configured number of writes per call, so a ground
//! station's burst of hundreds of writes spreads over ticks instead of overrunning one, and it
//! streams a listing from a cursor, only as fast as the publisher drains the reply queue.

use heapless::spsc::{Consumer, Producer};
use serde::{Deserialize, Serialize};
use stable_flight::params::Params;

use crate::ParamName;

#[cfg(test)]
mod tests;

/// A parameter request from the companion: [`Uplink::Param`](crate::Uplink::Param).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ParamRequest {
    /// Set one parameter by name, in its own units. Echoed with the value sent if accepted,
    /// the current value if rejected; an unknown name gets no reply.
    Set {
        /// Parameter name.
        name: ParamName,
        /// New value.
        value: f32,
    },
    /// List every parameter, one [`ParamValue`] each, in index order. Restarts a listing
    /// already in progress.
    List,
}

/// One parameter's value: [`Downlink::Param`](crate::Downlink::Param).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParamValue {
    /// Parameter name.
    pub name: ParamName,
    /// Value, in the parameter's own units.
    pub value: f32,
    /// Index of this parameter, `0..count`, stable across the flight controller's lifetime.
    pub index: u16,
    /// Number of parameters.
    pub count: u16,
}

/// Applies [`ParamRequest`]s to a [`Params`] and queues the [`ParamValue`] replies; call
/// [`poll`](Self::poll) between ticks.
///
/// A request is taken only when the reply queue has room for its answer, so no echo or
/// listing entry is ever dropped: a full reply queue holds requests back instead. Writes
/// confirm with the **value sent** (the [`Params`] stores-or-rejects contract makes it exact,
/// where a read-back of a degree param may differ in the last bit), rejections with the
/// current value.
///
/// # Examples
///
/// ```
/// use stable_flight_link::heapless::spsc::Queue;
/// use stable_flight_link::params::{ParamRequest, ParamServer, ParamValue};
/// use stable_flight_link::stable_flight::stabilizers::copter::{
///     CopterStabilizer, CopterStabilizerConfig,
/// };
///
/// let mut requests = Queue::<ParamRequest, 16>::new();
/// let mut replies = Queue::<ParamValue, 8>::new();
/// let (mut to_server, from_commander) = requests.split();
/// let (to_publisher, mut from_server) = replies.split();
/// let mut server = ParamServer::new(from_commander, to_publisher, 4);
/// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
///
/// // The commander, inside the tick:
/// let name = "STAB_ATT_AMAX_X".try_into().unwrap();
/// to_server.enqueue(ParamRequest::Set { name, value: 30.0 }).unwrap();
///
/// // Between ticks:
/// server.poll(&mut stabilizer);
///
/// // The publisher:
/// let echo = from_server.dequeue().unwrap();
/// assert_eq!(echo.value, 30.0); // the value sent, exactly
/// ```
pub struct ParamServer<'a> {
    requests: Consumer<'a, ParamRequest>,
    replies: Producer<'a, ParamValue>,
    max_writes_per_poll: usize,
    /// Next index of a listing in progress.
    cursor: Option<usize>,
}

impl<'a> ParamServer<'a> {
    /// A server reading `requests` and answering on `replies`, applying at most
    /// `max_writes_per_poll` writes per [`poll`](Self::poll).
    ///
    /// # Panics
    ///
    /// If `max_writes_per_poll` is zero: writes would never apply.
    pub fn new(
        requests: Consumer<'a, ParamRequest>,
        replies: Producer<'a, ParamValue>,
        max_writes_per_poll: usize,
    ) -> Self {
        assert!(max_writes_per_poll > 0, "max_writes_per_poll must be > 0");
        Self {
            requests,
            replies,
            max_writes_per_poll,
            cursor: None,
        }
    }

    /// Applies queued requests (at most `max_writes_per_poll` writes) while the reply queue
    /// has room, then continues a listing in progress until the reply queue is full.
    pub fn poll(&mut self, params: &mut impl Params) {
        let mut writes = 0;
        while writes < self.max_writes_per_poll && self.replies.ready() {
            let Some(request) = self.requests.dequeue() else {
                break;
            };
            match request {
                ParamRequest::Set { name, value } => {
                    writes += 1;
                    self.set(params, &name, value);
                }
                ParamRequest::List => self.cursor = Some(0),
            }
        }

        while let Some(index) = self.cursor {
            if !self.replies.ready() {
                break;
            }
            self.cursor = (index + 1 < params.param_count()).then_some(index + 1);
            if let Some(reply) = param_value(params, index, None) {
                self.reply(reply);
            }
        }
    }

    fn set(&mut self, params: &mut impl Params, name: &str, value: f32) {
        let Some(index) = params.param_index(name) else {
            return; // Unknown: no reply, as MAVLink ground stations expect.
        };
        let echo = params.set_param_at(index, value).ok().map(|()| value);
        if let Some(reply) = param_value(params, index, echo) {
            self.reply(reply);
        }
    }

    fn reply(&mut self, reply: ParamValue) {
        // Callers check `ready()` first, so this can't fail.
        let _ = self.replies.enqueue(reply);
    }
}

/// The reply for `index`: `value` if given, else the current value. `None` if `index` is out
/// of range or its name doesn't fit a [`ParamName`] (core tests keep every name within
/// [`MAX_NAME_LEN`](stable_flight::params::MAX_NAME_LEN)).
fn param_value(params: &impl Params, index: usize, value: Option<f32>) -> Option<ParamValue> {
    let name = ParamName::try_from(params.param_name(index)?).ok()?;
    let value = match value {
        Some(value) => value,
        None => params.param_at(index)?,
    };
    Some(ParamValue {
        name,
        value,
        index: u16::try_from(index).ok()?,
        count: u16::try_from(params.param_count()).unwrap_or(u16::MAX),
    })
}
