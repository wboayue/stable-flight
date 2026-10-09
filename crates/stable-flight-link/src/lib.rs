//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-link)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! Wire protocol between a Stable Flight flight controller and a companion computer (or a
//! laptop running the bridge): the [`Uplink`] / [`Downlink`] messages, the frame codec
//! ([`Message`]), and the byte-level [`Link`] trait that transports implement. `no_std` and
//! allocation-free. Frame format and design rationale: the
//! [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md#link).
//!
//! # Examples
//!
//! ```
//! use stable_flight_link::{Downlink, MAX_FRAME_LEN, Message};
//!
//! let mut frame = [0u8; MAX_FRAME_LEN];
//! let len = Downlink::Status("armed".try_into().unwrap()).encode(&mut frame).unwrap();
//!
//! let decoded = Downlink::decode(&mut frame[..len]).unwrap();
//! assert_eq!(decoded, Downlink::Status("armed".try_into().unwrap()));
//! ```

#![cfg_attr(not(test), no_std)]

#[cfg(all(not(test), feature = "std"))]
extern crate std;

mod frame;
mod io;
mod link;
#[cfg(any(test, feature = "testing"))]
pub mod loopback;
mod message;
pub mod params;
#[cfg(feature = "std")]
pub mod udp;

pub use frame::{DecodeError, EncodeError, MAX_FRAME_LEN, Message, PROTOCOL_VERSION};
pub use io::{LinkError, RecvMessage, SendMessage};
pub use link::{Link, LinkRx, LinkTx};
pub use message::{
    Downlink, MAX_STATUS_LEN, ParamName, ParamRequest, ParamValue, Record, StatusText, Uplink,
};

/// The `heapless` version [`ParamName`] and [`StatusText`] use.
pub use heapless;
/// The core crate whose types the messages carry.
pub use stable_flight;

/// Encodes `msg` into a fresh, exactly sized frame.
#[cfg(test)]
pub(crate) fn encode_vec<M: Message>(msg: &M) -> Vec<u8> {
    let mut frame = vec![0u8; MAX_FRAME_LEN];
    let len = msg
        .encode(&mut frame)
        .expect("MAX_FRAME_LEN fits every message");
    frame.truncate(len);
    frame
}
