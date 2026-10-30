//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-sitl)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! This library runs the [Stable Flight](https://crates.io/crates/stable-flight) flight controller
//! in simulation. Simulation allows safe refining of the flight control logic before real-world deployment.
//! The flight-control core (traits, guidance, stabilizers, mixers, [`stable_flight::FlightLoop`]) lives in the
//! `stable-flight` crate; this crate adds the simulator-independent software-in-the-loop harness
//! around it: joystick and scripted commanders, an autotune flight's commander and log
//! ([`autotune`]), and the real-time runner ([`RealTimeRunner`])
//! with graceful shutdown ([`ShutdownSignal`]), and per-tick CSV recording ([`recorder`]). Telemetry goes over the native link
//! (`stable-flight-link`'s `LinkPublisher`), and `stable-flight-bridge` speaks MAVLink for it. Each simulator backend is its own crate, e.g.
//! [`stable-flight-sitl-realflight`](https://crates.io/crates/stable-flight-sitl-realflight).
//! See the following document for an [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md).
//!
//! Checkout [README](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl/README.md) to get started.
//!
//! [![Watch the demo](https://img.youtube.com/vi/KsTX3pH_htg/0.jpg)](https://youtu.be/KsTX3pH_htg)
//!

/// The flight-control core this crate is built on, re-exported so users can reach the exact
/// version it uses (`stable_flight_sitl::stable_flight::FlightLoop`) without a second copy.
pub use stable_flight;

pub mod autotune;
pub mod cli;
pub mod commanders;
pub mod recorder;
pub mod runner;
pub mod shutdown;

#[doc(inline)]
pub use runner::RealTimeRunner;
#[doc(inline)]
pub use shutdown::ShutdownSignal;
