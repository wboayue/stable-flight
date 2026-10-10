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

/// Maps `value` from `[min, max]` to `[0, 1]`, clamped: device ranges are often narrower than
/// the axis's physical travel (e.g. ±0.7 of ±1.0), so unclamped values would overshoot. NaN
/// passes through; the flight loop treats it as neutral.
fn scale_value(value: f32, min: f32, max: f32) -> f32 {
    ((value - min) / (max - min)).clamp(0.0, 1.0)
}

/// Maps a centered axis `value` from `[-half_range, half_range]` to `[-1, 1]`, clamped like
/// [`scale_value`].
fn scale_signed(value: f32, half_range: f32) -> f32 {
    (value / half_range).clamp(-1.0, 1.0)
}

#[allow(unused_imports)] // used in documentation
use stable_flight::Commander;

#[doc(inline)]
pub use extreme_3d::Extreme3DCommander;
#[doc(inline)]
pub use script::ScriptCommander;
#[doc(inline)]
pub use spektrum::SpektrumCommander;

#[cfg(test)]
mod tests {
    use super::{scale_signed, scale_value};

    #[test]
    fn scale_value_maps_to_unit_range_and_clamps_overshoot() {
        // (value, min, max, expected)
        let cases = [
            (0.0, -0.7, 0.7, 0.5),
            (0.7, -0.7, 0.7, 1.0),
            (-0.7, -0.7, 0.7, 0.0),
            (1.0, -0.7, 0.7, 1.0), // full physical travel past the configured range
            (-1.0, -0.7, 0.7, 0.0),
            (0.95, 0.1, 0.9, 1.0),
            (5.0, 0.0, 10.0, 0.5),
        ];
        for (value, min, max, expected) in cases {
            let scaled = scale_value(value, min, max);
            assert!(
                (scaled - expected).abs() < 1e-6,
                "scale_value({value}, {min}, {max}) = {scaled}, expected {expected}"
            );
        }
    }

    #[test]
    fn scale_signed_centers_on_zero_and_clamps_overshoot() {
        // (value, half_range, expected)
        let cases = [
            (0.0, 0.7, 0.0),
            (0.35, 0.7, 0.5),
            (-0.35, 0.7, -0.5),
            (0.7, 0.7, 1.0),
            (-0.7, 0.7, -1.0),
            (1.0, 0.7, 1.0), // full physical travel past the configured range
            (-1.0, 0.7, -1.0),
        ];
        for (value, half_range, expected) in cases {
            let scaled = scale_signed(value, half_range);
            assert!(
                (scaled - expected).abs() < 1e-6,
                "scale_signed({value}, {half_range}) = {scaled}, expected {expected}"
            );
        }
    }

    #[test]
    fn scaling_passes_nan_through() {
        assert!(scale_value(f32::NAN, -0.7, 0.7).is_nan());
        assert!(scale_signed(f32::NAN, 0.7).is_nan());
    }
}
