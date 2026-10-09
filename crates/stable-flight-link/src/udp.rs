//! UDP transport (`std` feature): one datagram per frame, for host setups where the flight
//! loop (SITL) and the bridge run on one machine or a LAN.
//!
//! ```no_run
//! use stable_flight_link::udp::UdpLink;
//! use stable_flight_link::{MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink};
//!
//! // The SITL side: listen on 14560, send to the bridge on 14561.
//! let (mut rx, mut tx) = UdpLink::bind("127.0.0.1:14560", "127.0.0.1:14561")?.into_halves();
//! let mut buf = [0u8; MAX_FRAME_LEN];
//! while let Some(msg) = rx.recv_message::<Uplink>(&mut buf)? {
//!     // ...
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::io::{self, ErrorKind};
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};

#[cfg(doc)]
use crate::Link;
use crate::{LinkRx, LinkTx};

#[cfg(test)]
mod tests;

/// A [`Link`] over a non-blocking UDP socket bound to a local address and connected to one
/// fixed peer: it sends only to the peer and receives only from it. Each datagram carries one
/// frame.
///
/// The peer needn't be up: until it is, sends are dropped and nothing is received, with no
/// error (the ICMP port-unreachable a connected socket reports is swallowed). A datagram larger
/// than the receive buffer is lost: truncated so it fails decode, or an error on platforms that
/// report it.
#[derive(Debug)]
pub struct UdpLink {
    rx: UdpRx,
    tx: UdpTx,
}

impl UdpLink {
    /// Binds `local` and connects to `peer`, non-blocking.
    ///
    /// # Errors
    ///
    /// An address that doesn't resolve, or a socket that can't be bound or connected, or that
    /// [`try_from`](Self::try_from) rejects.
    pub fn bind(local: impl ToSocketAddrs, peer: impl ToSocketAddrs) -> io::Result<Self> {
        let socket = UdpSocket::bind(local)?;
        socket.connect(peer)?;
        Self::try_from(socket)
    }

    /// The bound local address (useful after binding port 0).
    ///
    /// # Errors
    ///
    /// The socket's address can't be read.
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.rx.socket.local_addr()
    }

    /// The receiving and sending halves, each owning a handle to the same socket.
    pub fn into_halves(self) -> (UdpRx, UdpTx) {
        (self.rx, self.tx)
    }
}

impl TryFrom<UdpSocket> for UdpLink {
    type Error = io::Error;

    /// Wraps a socket already bound and connected to its peer, making it non-blocking: for
    /// setups that create the socket themselves (e.g. both ends on ephemeral ports).
    ///
    /// # Errors
    ///
    /// The socket isn't connected, or can't be made non-blocking or cloned.
    fn try_from(socket: UdpSocket) -> io::Result<Self> {
        socket.peer_addr()?;
        socket.set_nonblocking(true)?;
        let tx = UdpTx {
            socket: socket.try_clone()?,
        };
        Ok(Self {
            rx: UdpRx { socket },
            tx,
        })
    }
}

/// Receiving half of a [`UdpLink`].
#[derive(Debug)]
pub struct UdpRx {
    socket: UdpSocket,
}

/// Sending half of a [`UdpLink`]. When the socket can't take a datagram, [`LinkTx::send`]
/// drops it.
#[derive(Debug)]
pub struct UdpTx {
    socket: UdpSocket,
}

/// The peer isn't listening (yet): not a transport failure.
fn peer_down(e: &io::Error) -> bool {
    matches!(
        e.kind(),
        ErrorKind::ConnectionRefused | ErrorKind::ConnectionReset
    )
}

/// Runs `op`, retrying once after a peer-down error. A connected socket reports an earlier
/// send's port-unreachable on the next call, which then does nothing else: without the retry
/// that call would drop a frame to a peer that has since come up, or report nothing waiting
/// while datagrams are queued. The retry sees the peer's current state.
fn retry_peer_down<T>(mut op: impl FnMut() -> io::Result<T>) -> io::Result<T> {
    match op() {
        Err(e) if peer_down(&e) => op(),
        result => result,
    }
}

impl LinkRx for UdpRx {
    type Error = io::Error;

    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        match retry_peer_down(|| self.socket.recv(buf)) {
            Ok(len) => Ok(Some(len)),
            Err(e) if e.kind() == ErrorKind::WouldBlock || peer_down(&e) => Ok(None),
            Err(e) => Err(e),
        }
    }
}

impl LinkTx for UdpTx {
    type Error = io::Error;

    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        match retry_peer_down(|| self.socket.send(frame)) {
            Ok(_) => Ok(()),
            // Full buffer or absent peer: drop, per the `LinkTx` contract.
            Err(e) if e.kind() == ErrorKind::WouldBlock || peer_down(&e) => Ok(()),
            Err(e) => Err(e),
        }
    }
}

impl LinkRx for UdpLink {
    type Error = io::Error;

    fn try_recv(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Self::Error> {
        self.rx.try_recv(buf)
    }
}

impl LinkTx for UdpLink {
    type Error = io::Error;

    fn send(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.tx.send(frame)
    }
}
