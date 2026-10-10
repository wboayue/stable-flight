//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-sitl-realflight)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! [RealFlight](https://www.realflight.com) simulator backend for the
//! [Stable Flight](https://crates.io/crates/stable-flight) flight controller.
//! [`RealFlightVehicle`] implements the core `StateEstimator` + `Actuator` (so it is a
//! `Vehicle`) over a [`realflight_bridge`] connection, and [`airframes`] holds the tuned setup
//! for each supported RealFlight model.
//!
//! The crate depends on the `stable-flight` core only, so it works with any runner; pair it
//! with [`stable-flight-sitl`](https://crates.io/crates/stable-flight-sitl) for the real-time
//! runner, commanders, and telemetry. See the
//! [README](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl-realflight/README.md)
//! to get started.

/// The RealFlight Link client this crate is built on, re-exported so users construct the
/// bridge (`RealFlightRemoteBridge`, `RealFlightLocalBridge`) from the exact version it uses.
pub use realflight_bridge;

pub mod airframes;
mod vehicle;

pub use vehicle::{
    ActuatorMapping, RealFlightConfiguration, RealFlightVehicle, SLOW_EXCHANGE, vehicle_state,
};
