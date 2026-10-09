//! A MAVLink connection double for tests (`testing` feature): see [`MockConnection`].

use std::collections::VecDeque;
use std::io;
use std::sync::{Arc, Mutex, MutexGuard};

use mavlink::common::MavMessage;
use mavlink::error::{MessageReadError, MessageWriteError};
use mavlink::{MAVLinkMessageRaw, MavConnection, MavHeader, MavlinkVersion};

/// An in-memory `MavConnection`: `try_recv` returns scripted results in order, then
/// `WouldBlock`; `send` records every message, and fails after recording it when told to.
/// Clones share their state, so a test keeps a handle to a connection it boxed.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::mavlink::common::{HEARTBEAT_DATA, MavMessage};
/// use stable_flight_mavlink::mavlink::{MavConnection, MavHeader};
/// use stable_flight_mavlink::testing::MockConnection;
///
/// let mock = MockConnection::default();
/// let connection: Box<dyn MavConnection<MavMessage> + Send + Sync> = Box::new(mock.clone());
/// let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
///
/// mock.push_received(MavHeader::default(), heartbeat.clone());
/// assert!(connection.try_recv().is_ok());
/// assert!(connection.try_recv().is_err()); // WouldBlock: nothing left
///
/// connection.send(&MavHeader::default(), &heartbeat).unwrap();
/// assert_eq!(mock.sent().len(), 1);
/// ```
#[derive(Debug, Clone, Default)]
pub struct MockConnection {
    state: Arc<Mutex<State>>,
}

#[derive(Debug, Default)]
struct State {
    received: VecDeque<Result<(MavHeader, MavMessage), MessageReadError>>,
    sent: Vec<(MavHeader, MavMessage)>,
    fail_sends: bool,
}

impl MockConnection {
    /// A connection whose every send is recorded, then fails.
    pub fn failing() -> Self {
        let mock = Self::default();
        mock.state().fail_sends = true;
        mock
    }

    /// Queues `msg` from `header` for a later `try_recv`.
    pub fn push_received(&self, header: MavHeader, msg: MavMessage) {
        self.state().received.push_back(Ok((header, msg)));
    }

    /// Queues `error` for a later `try_recv`.
    pub fn push_error(&self, error: MessageReadError) {
        self.state().received.push_back(Err(error));
    }

    /// Every message sent so far, failed sends included.
    pub fn sent(&self) -> Vec<(MavHeader, MavMessage)> {
        self.state().sent.clone()
    }

    /// Scripted results not yet received.
    pub fn pending(&self) -> usize {
        self.state().received.len()
    }

    fn state(&self) -> MutexGuard<'_, State> {
        // A test that panicked while holding the lock has already failed.
        self.state
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

fn io_error(kind: io::ErrorKind) -> MessageReadError {
    MessageReadError::Io(io::Error::from(kind))
}

impl MavConnection<MavMessage> for MockConnection {
    /// The next scripted result; a test never blocks, so an empty script is `Unsupported`.
    fn recv(&self) -> Result<(MavHeader, MavMessage), MessageReadError> {
        self.state()
            .received
            .pop_front()
            .unwrap_or_else(|| Err(io_error(io::ErrorKind::Unsupported)))
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        Err(io_error(io::ErrorKind::Unsupported))
    }

    fn try_recv(&self) -> Result<(MavHeader, MavMessage), MessageReadError> {
        self.state()
            .received
            .pop_front()
            .unwrap_or_else(|| Err(io_error(io::ErrorKind::WouldBlock)))
    }

    fn send(&self, header: &MavHeader, data: &MavMessage) -> Result<usize, MessageWriteError> {
        let mut state = self.state();
        state.sent.push((*header, data.clone()));
        if state.fail_sends {
            Err(MessageWriteError::Io(io::Error::from(
                io::ErrorKind::ConnectionRefused,
            )))
        } else {
            Ok(0)
        }
    }

    fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

    fn protocol_version(&self) -> MavlinkVersion {
        MavlinkVersion::V2
    }

    fn set_allow_recv_any_version(&mut self, _allow: bool) {}

    fn allow_recv_any_version(&self) -> bool {
        false
    }
}
