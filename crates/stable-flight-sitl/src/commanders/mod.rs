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
//! - **[SpektrumCommander]**: Generates manual commands from the Spektrum transmitter.

pub mod extreme_3d;
pub mod script;
pub mod spektrum;

use thiserror::Error;

/// Errors that can occur during commander initialization or operation.
#[derive(Error, Debug)]
pub enum CommanderError {
    /// Failed to initialize the input device (joystick, controller, etc.)
    #[error("Failed to initialize input device: {0}")]
    InitializationFailed(String),

    /// Input device disconnected during operation
    #[error("Input device disconnected")]
    DeviceDisconnected,

    /// Invalid configuration or parameters
    #[error("Invalid configuration: {0}")]
    InvalidConfiguration(String),
}

/// Opens the gamepad subsystem shared by the joystick commanders.
fn open_gilrs() -> Result<gilrs::Gilrs, CommanderError> {
    gilrs::Gilrs::new().map_err(|e| CommanderError::InitializationFailed(e.to_string()))
}

#[allow(unused_imports)] // used in documentation
use stable_flight::Commander;

#[doc(inline)]
pub use extreme_3d::Extreme3DCommander;
#[doc(inline)]
pub use script::ScriptCommander;
#[doc(inline)]
pub use spektrum::SpektrumCommander;
