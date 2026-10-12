//! Outer-loop [`Guidance`](crate::Guidance) implementations, which turn commands into the
//! setpoints a [`Stabilizer`](crate::Stabilizer) tracks, and their building blocks
//! ([`vtol`]'s transition state machine).

pub mod copter;
mod rate_divider;
pub mod vtol;
