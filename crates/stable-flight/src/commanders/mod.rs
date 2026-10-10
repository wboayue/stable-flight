//! Hardware-independent [`Commander`](crate::Commander) implementations.
//!
//! Commanders that need an OS, files, or input devices (joysticks, CSV scripts) live in
//! `stable-flight-sitl`; they build on the ones here.

pub mod failsafe;
pub mod touchdown;
pub mod waypoint;

#[doc(inline)]
pub use failsafe::{
    DEFAULT_HOLD_SPEED_MPS, FailsafeCommander, FailsafeConfig, Fallback, HoldCommander,
    LinkedCommander,
};
#[doc(inline)]
pub use touchdown::TouchdownDetector;
#[doc(inline)]
pub use waypoint::{EmptyWaypoints, RouteEnd, WaypointFollower};
