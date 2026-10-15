//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-sitl-xplane)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! [X-Plane](https://www.x-plane.com) simulator backend for the
//! [Stable Flight](https://crates.io/crates/stable-flight) flight controller.
//! [`XPlaneVehicle`] implements the core `StateSource` + `Actuator` (so it is a `Vehicle`) over
//! an [`xplane_bridge`] connection. [`StateDatarefs`] subscribes the raw state it reads,
//! [`vehicle_state`] converts one [`XPlaneSample`] to the core frame contract, [`LocalOrigin`]
//! keeps position continuous across X-Plane's moving local origin, and [`capture`] records raw
//! samples for checking the conversion offline.
//!
//! The crate depends on the `stable-flight` core only, so it works with any runner; pair it
//! with [`stable-flight-sitl`](https://crates.io/crates/stable-flight-sitl) for the real-time
//! runner and commanders. See the
//! [README](https://github.com/wboayue/stable-flight/blob/main/crates/stable-flight-sitl-xplane/README.md)
//! to get started.

/// The X-Plane UDP client this crate is built on, re-exported so users connect
/// (`XPlaneUdpBridge::connect`) with the exact version it uses.
pub use xplane_bridge;

pub mod capture;
mod mapping;
mod origin;
mod sample;
mod state;
pub mod sweep;
mod vehicle;

pub use mapping::{ActuatorMapping, ENGINE_COUNT, MappingError, WING_COUNT};
pub use origin::{Anchor, LocalOrigin, OriginEvent};
pub use sample::{STATE_RATE, StateDatarefs, XPlaneSample, wait_ready};
pub use state::vehicle_state;
pub use vehicle::{ConnectError, DEFAULT_READY_TIMEOUT, XPlaneConfiguration, XPlaneVehicle};
