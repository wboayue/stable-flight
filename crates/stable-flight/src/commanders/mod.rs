//! Hardware-independent [`Commander`](crate::Commander) implementations.
//!
//! Commanders that need an OS, files, or input devices (joysticks, CSV scripts) live in
//! `stable-flight-sitl`; they build on the ones here.

pub mod waypoint;

#[doc(inline)]
pub use waypoint::{EmptyWaypoints, RouteEnd, WaypointFollower};
