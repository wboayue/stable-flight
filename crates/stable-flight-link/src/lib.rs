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
//! ([`Message`]), and the byte-level [`Link`] trait that transports implement.
//!
//! A frame is `[version][postcard payload][crc16]`, COBS-encoded and terminated by `0x00`;
//! see [`PROTOCOL_VERSION`] and [`MAX_FRAME_LEN`]. The crate is `no_std` and allocation-free.
//! Transports live elsewhere; the `testing` feature adds an in-memory `loopback` pair.
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

mod frame;
mod link;
#[cfg(any(test, feature = "testing"))]
pub mod loopback;
mod message;

pub use frame::{FrameError, MAX_FRAME_LEN, Message, PROTOCOL_VERSION};
pub use link::Link;
pub use message::{Downlink, MAX_STATUS_LEN, ParamName, Record, StatusText, Uplink};

/// The `heapless` version [`ParamName`] and [`StatusText`] use.
pub use heapless;
/// The core crate whose types the messages carry.
pub use stable_flight;
