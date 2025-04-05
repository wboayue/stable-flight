//! Contains components that generate flight control directives based on mission definitions and/or pilot inputs.
//!
//! Defines components that implement the [Commander] trait and generate flight control commands.
//! The unified interface supports both manual (attitude and rate-based)
//! and autonomous (waypoint-based) flight control.
//!
//! The following commanders are implemented:
//!
//! - **[ScriptCommander]**: Supports the playback of commands from a script.
//! - **[Extreme3DCommander]**: Generates manual commands from the Extreme3D Joystick.
//! - **[SpectrumCommander]**: Generates manual commands from the Spextrum transmitter.

pub mod extreme_3d;
pub mod script;
pub mod spektrum;

#[allow(unused_imports)] // used in documentation
use super::Commander;

#[doc(inline)]
pub use extreme_3d::Extreme3DCommander;
#[doc(inline)]
pub use script::ScriptCommander;
#[doc(inline)]
pub use spektrum::SpectrumCommander;

#[cfg(test)]
mod tests;
