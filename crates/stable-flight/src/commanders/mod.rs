//! Hardware-independent [`Commander`](crate::Commander) implementations.
//!
//! Commanders that need an OS, files, or input devices (joysticks, CSV scripts) live in
//! `stable-flight-sitl`; they build on the ones here.

pub mod failsafe;
pub mod maneuver;
pub mod touchdown;
pub mod tune_hold;
pub mod waypoint;

#[doc(inline)]
pub use failsafe::{
    DEFAULT_HOLD_SPEED_MPS, FailsafeCommander, FailsafeConfig, Fallback, HoldCommander,
    LinkedCommander,
};
#[doc(inline)]
pub use maneuver::{Maneuver, ManeuverConfig, ManeuverRunner};
#[doc(inline)]
pub use touchdown::{
    DEFAULT_STOPPED_MPS, DEFAULT_TOUCHDOWN_HOLD_S, TouchdownConfig, TouchdownDetector,
};
#[doc(inline)]
pub use tune_hold::{PlaneTuneHold, PlaneTuneHoldConfig};
#[doc(inline)]
pub use waypoint::{EmptyWaypoints, RouteEnd, WaypointFollower};
