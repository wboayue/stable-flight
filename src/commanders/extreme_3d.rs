//! Logitech Extreme 3D Pro Joystick Commander
//!
//! This module provides a [`Commander`] implementation for the Logitech Extreme 3D Pro
//! joystick, mapping its axes and buttons to flight control commands.
//!
//! # Hardware Mapping
//!
//! The joystick controls are mapped as follows:
//!
//! ## Axes
//! - **X axis (left/right)**: Roll control
//! - **Y axis (forward/back)**: Pitch control
//! - **Twist (rotate)**: Yaw control
//! - **Throttle slider**: Throttle (0-100%)
//!
//! ## Buttons
//! - **Top trigger (North button)**: Arm the vehicle
//! - **Base button (code 589827)**: Disarm the vehicle
//! - **D-Pad Left**: Mode 0
//! - **D-Pad Up**: Mode 1
//! - **D-Pad Right**: Mode 2
//!
//! # Dead Zones
//!
//! To reduce noise and improve control precision, scaled dead zones are applied:
//! - Roll/Pitch: 10% (±0.1)
//! - Yaw: 15% (±0.15) - larger due to twist axis sensitivity
//!
//! The dead zone implementation uses smooth scaling to eliminate discontinuities.
//! Inputs below the dead zone threshold are zeroed, while inputs above are rescaled
//! so that the dead zone boundary maps to 0.0 and full deflection maps to ±1.0.
//! This provides smooth, gap-free control transitions.
//!
//! # Examples
//!
//! Basic usage with attitude control:
//! ```no_run
//! use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
//! use stable_flight_sitl::{Commander, VehicleState};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! // Create commander in attitude mode
//! let mut commander = Extreme3DCommander::new()?;
//!
//! // Generate commands in a control loop
//! let state = VehicleState::default();
//! let command = commander.next(0.004, &state); // 250 Hz (4ms)
//! # Ok(())
//! # }
//! ```
//!
//! Mode selection is done at runtime using the D-pad buttons:
//! - D-Pad Up: Switch to Rate mode (Mode 1) for acrobatic flight
//! - D-Pad Left/Right: Switch to Attitude mode (Mode 0 or 2) for stable flight
//!
//! ```no_run
//! use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
//! use stable_flight_sitl::{Commander, VehicleState};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! let mut commander = Extreme3DCommander::new()?;
//! let state = VehicleState::default();
//!
//! // Press D-Pad Up on the joystick to switch to Rate mode
//! // The commander will automatically switch modes based on D-pad input
//! let command = commander.next(0.004, &state);
//! // Command type (Attitude or Rate) depends on current D-pad selection
//! # Ok(())
//! # }
//! ```

use gilrs::{Axis, Button, Event, EventType, GamepadId, Gilrs};

use crate::commanders::CommanderError;
use crate::{AttitudeCommand, Command, Commander, RateCommand, VehicleState};

/// Hardware code for the throttle axis on the Extreme 3D Pro.
/// This value is provided by the Gilrs library for the throttle slider.
const THROTTLE_CODE: u32 = 65590;

/// Hardware code for the disarm button on the Extreme 3D Pro.
/// This corresponds to a specific button on the joystick base.
const DISARM_BUTTON_CODE: u32 = 589827;

/// Dead zone threshold for roll and pitch axes (10%).
/// Values within ±0.1 of center are treated as zero to reduce stick drift.
const ROLL_PITCH_DEAD_ZONE: f32 = 0.1;

/// Dead zone threshold for yaw axis (15%).
/// Larger than roll/pitch due to increased sensitivity of the twist axis.
const YAW_DEAD_ZONE: f32 = 0.15;

/// Throttle axis minimum value observed from hardware.
/// The Extreme 3D Pro throttle slider doesn't quite reach -1.0 at the bottom,
/// stopping at approximately -0.99. This constant is used for accurate scaling.
const THROTTLE_MIN: f32 = -0.99;

/// Throttle axis maximum value observed from hardware.
/// The Extreme 3D Pro throttle slider doesn't quite reach 1.0 at the top,
/// stopping at approximately 0.99. This constant is used for accurate scaling.
const THROTTLE_MAX: f32 = 0.99;

/// Control mode for the Extreme 3D commander.
///
/// Determines whether joystick inputs are interpreted as attitude
/// (orientation) commands or rate (angular velocity) commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Attitude control mode.
    ///
    /// Joystick inputs command desired roll, pitch, and yaw angles.
    /// The flight controller will stabilize to maintain these orientations.
    /// This mode is easier for beginners and provides stable, predictable flight.
    ///
    /// # Example
    /// - Moving the stick 50% right commands a specific roll angle (e.g., 25°)
    /// - Centering the stick returns the vehicle to level flight
    Attitude,

    /// Rate control mode (also known as "acro" mode).
    ///
    /// Joystick inputs command desired angular rates (rotation speeds).
    /// The flight controller maintains the commanded rotation rate but does not
    /// auto-level when the stick is centered. This mode allows for acrobatic
    /// maneuvers and precise control but requires more pilot skill.
    ///
    /// # Example
    /// - Moving the stick 50% right commands a constant roll rate (e.g., 180°/s)
    /// - Centering the stick stops rotation but maintains current orientation
    Rate,
}

/// Commander implementation for the Logitech Extreme 3D Pro joystick.
///
/// This commander reads input from the Extreme 3D Pro joystick and converts
/// it into flight control commands. It supports both attitude and rate control
/// modes, and applies dead zones to reduce stick drift and improve control precision.
///
/// See the module-level documentation for hardware mapping, control modes, and usage examples.
///
/// # Thread Safety
///
/// This struct is not thread-safe and should only be used from a single thread.
/// The Gilrs event loop internally manages joystick state.
pub struct Extreme3DCommander {
    /// Gilrs instance for reading joystick input
    gilrs: Gilrs,

    /// Currently active gamepad ID (set when first event is received)
    active_gamepad: Option<GamepadId>,

    /// Control channels storing normalized joystick values
    /// [throttle, roll, pitch, yaw, arm, mode]
    channels: [f32; 6],

    /// Control mode (Attitude or Rate)
    mode: Mode,
}

impl Extreme3DCommander {
    /// Creates a new Extreme3DCommander instance in Attitude mode.
    ///
    /// The commander starts in Attitude mode by default. Use `with_mode()` to
    /// specify a different initial mode, or use D-pad buttons during operation
    /// to switch modes.
    ///
    /// # Returns
    ///
    /// Returns `Ok(Extreme3DCommander)` if the joystick is successfully initialized,
    /// or `Err(CommanderError)` if initialization fails.
    ///
    /// # Errors
    ///
    /// Returns `CommanderError::InitializationFailed` if the Gilrs library cannot
    /// be initialized, which may occur if:
    /// - No joystick is connected
    /// - The operating system's input subsystem is unavailable
    /// - There are permission issues accessing input devices
    ///
    /// # Examples
    ///
    /// See module-level documentation for usage examples.
    pub fn new() -> Result<Self, CommanderError> {
        Self::with_mode(Mode::Attitude)
    }

    /// Creates a new Extreme3DCommander with the specified initial mode.
    ///
    /// Use this constructor when you need to start in Rate mode for headless
    /// or non-interactive use cases where D-pad input is unavailable.
    ///
    /// # Parameters
    ///
    /// - `mode`: The initial control mode (Attitude or Rate)
    ///
    /// # Returns
    ///
    /// Returns `Ok(Extreme3DCommander)` if the joystick is successfully initialized,
    /// or `Err(CommanderError)` if initialization fails.
    ///
    /// # Errors
    ///
    /// Returns `CommanderError::InitializationFailed` if the Gilrs library cannot
    /// be initialized.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use stable_flight_sitl::commanders::extreme_3d::{Extreme3DCommander, Mode};
    ///
    /// // Start in Rate mode for acrobatic flight
    /// let commander = Extreme3DCommander::with_mode(Mode::Rate)?;
    /// # Ok::<(), stable_flight_sitl::commanders::CommanderError>(())
    /// ```
    pub fn with_mode(mode: Mode) -> Result<Self, CommanderError> {
        let gilrs = Gilrs::new().map_err(|e| {
            CommanderError::InitializationFailed(format!("Gilrs initialization failed: {}", e))
        })?;

        // Initialize channels[5] to match the chosen mode so next() preserves it
        let mut channels = [0.0; 6];
        channels[5] = match mode {
            Mode::Attitude => 0.0,
            Mode::Rate => 1.0,
        };

        Ok(Self {
            gilrs,
            active_gamepad: None,
            channels,
            mode,
        })
    }

    /// Returns the current control mode.
    ///
    /// # Returns
    ///
    /// The current `Mode` (Attitude or Rate).
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Reads and updates control channels from the Extreme 3D joystick.
    ///
    /// This method polls the joystick for new events and updates the internal
    /// channel state. It processes both event-based inputs (buttons) and
    /// continuous state (axes).
    ///
    /// # Returns
    ///
    /// An array of 6 normalized control values:
    /// - `[0]`: Throttle (0.0 to 1.0)
    /// - `[1]`: Roll (-1.0 to 1.0)
    /// - `[2]`: Pitch (-1.0 to 1.0)
    /// - `[3]`: Yaw (-1.0 to 1.0)
    /// - `[4]`: Arm state (0.0 = disarmed, 1.0 = armed)
    /// - `[5]`: Mode (0.0, 1.0, or 2.0)
    ///
    /// # Performance
    ///
    /// This method is designed to be called at high frequency (e.g., 250 Hz)
    /// in the flight control loop. It processes all pending events and reads
    /// current axis values on each call.
    pub fn read_channels(&mut self) -> [f32; 6] {
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);

            match event {
                EventType::AxisChanged(_, value, code) => {
                    if code.into_u32() == THROTTLE_CODE {
                        // Throttle is inverted (1.0 at bottom, 0.0 at top) and scaled
                        self.channels[0] =
                            1.0 - scale_value(value, THROTTLE_MIN, THROTTLE_MAX).clamp(0.0, 1.0);
                    }
                }
                EventType::ButtonChanged(Button::North, 1.0, _) => {
                    self.channels[4] = 1.0;
                }
                EventType::ButtonChanged(Button::Unknown, 1.0, code) => {
                    if code.into_u32() == DISARM_BUTTON_CODE {
                        self.channels[4] = 0.0;
                    }
                }
                EventType::ButtonChanged(Button::DPadLeft, 1.0, _) => {
                    self.channels[5] = 0.0; // Attitude mode
                }
                EventType::ButtonChanged(Button::DPadUp, 1.0, _) => {
                    self.channels[5] = 1.0; // Rate mode
                }
                EventType::ButtonChanged(Button::DPadRight, 1.0, _) => {
                    self.channels[5] = 2.0; // Attitude mode
                }
                _ => {}
            }
        }

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                self.channels[1] = data.value();
            }

            // Pitch and yaw are negated to match flight control conventions
            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                self.channels[2] = -data.value();
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
                self.channels[3] = -data.value();
            }
        }

        // Advance the Gilrs internal counter to process next frame of events
        self.gilrs.inc();

        self.channels
    }
}

impl Default for Extreme3DCommander {
    /// Creates a default Extreme3DCommander in Attitude mode.
    ///
    /// # Panics
    ///
    /// Panics if joystick initialization fails. For fallible initialization,
    /// use `Extreme3DCommander::new()` instead.
    fn default() -> Self {
        Self::new().expect("Failed to initialize Extreme3DCommander with default settings")
    }
}

impl Commander for Extreme3DCommander {
    /// Generates the next control command based on current joystick input.
    ///
    /// This method reads all pending joystick events, updates internal state,
    /// and returns either an `AttitudeCommand` or `RateCommand` depending on
    /// the current mode selected via D-pad buttons:
    /// - Mode 0 or 2 (D-Pad Left/Right): Returns `Command::Attitude`
    /// - Mode 1 (D-Pad Up): Returns `Command::Rate`
    ///
    /// Dead zones are automatically applied to roll, pitch, and yaw inputs.
    fn next(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        let channels = self.read_channels();

        let mode = channels[5] as u8;
        match mode {
            1 => self.mode = Mode::Rate,
            _ => self.mode = Mode::Attitude,
        }

        match &self.mode {
            Mode::Attitude => Command::Attitude(AttitudeCommand {
                throttle: channels[0],
                roll: apply_dead_zone(channels[1], ROLL_PITCH_DEAD_ZONE),
                pitch: apply_dead_zone(channels[2], ROLL_PITCH_DEAD_ZONE),
                yaw: apply_dead_zone(channels[3], YAW_DEAD_ZONE),
                arm: channels[4] > 0.5,
                mode,
            }),
            Mode::Rate => Command::Rate(RateCommand {
                throttle: channels[0],
                roll_rate: apply_dead_zone(channels[1], ROLL_PITCH_DEAD_ZONE),
                pitch_rate: apply_dead_zone(channels[2], ROLL_PITCH_DEAD_ZONE),
                yaw_rate: apply_dead_zone(channels[3], YAW_DEAD_ZONE),
                arm: channels[4] > 0.5,
                mode,
            }),
        }
    }
}

/// Scales a value from an arbitrary range to 0.0-1.0.
///
/// # Parameters
///
/// - `value`: The input value to scale
/// - `min`: The minimum value of the input range
/// - `max`: The maximum value of the input range
///
/// # Returns
///
/// A value scaled to the range [0.0, 1.0], where `min` maps to 0.0 and `max` maps to 1.0.
///
/// # Examples
///
/// ```
/// # fn scale_value(value: f32, min: f32, max: f32) -> f32 {
/// #     (value - min) / (max - min)
/// # }
/// assert_eq!(scale_value(0.0, -1.0, 1.0), 0.5);
/// assert_eq!(scale_value(-1.0, -1.0, 1.0), 0.0);
/// assert_eq!(scale_value(1.0, -1.0, 1.0), 1.0);
/// ```
fn scale_value(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}

/// Applies a scaled dead zone to a control input value.
///
/// Values with absolute magnitude less than the dead zone threshold are
/// zeroed out to prevent stick drift. Values outside the dead zone are
/// rescaled to smoothly remap the range [dead_zone, 1.0] to [0.0, 1.0],
/// eliminating discontinuities at the dead zone boundary.
///
/// # Parameters
///
/// - `value`: The input value (typically -1.0 to 1.0)
/// - `dead_zone`: The dead zone threshold (positive value, must be < 1.0)
///
/// # Returns
///
/// - `0.0` if `|value| < dead_zone`
/// - A scaled value in the range [-1.0, 1.0] otherwise, where:
///   - `dead_zone` maps to `0.0`
///   - `1.0` maps to `1.0`
///   - `-dead_zone` maps to `0.0`
///   - `-1.0` maps to `-1.0`
///
/// # Examples
///
/// ```
/// # fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
/// #     if value.abs() < dead_zone {
/// #         0.0
/// #     } else {
/// #         let sign = value.signum();
/// #         let scaled = (value.abs() - dead_zone) / (1.0 - dead_zone);
/// #         sign * scaled
/// #     }
/// # }
/// assert_eq!(apply_dead_zone(0.05, 0.1), 0.0);   // Inside dead zone
/// assert_eq!(apply_dead_zone(-0.05, 0.1), 0.0);  // Inside dead zone
/// assert_eq!(apply_dead_zone(0.1, 0.1), 0.0);    // At dead zone boundary
/// assert_eq!(apply_dead_zone(1.0, 0.1), 1.0);    // At maximum
/// assert_eq!(apply_dead_zone(-1.0, 0.1), -1.0);  // At minimum
///
/// // Midpoint between dead zone and max scales to ~0.5
/// let mid = apply_dead_zone(0.55, 0.1);
/// assert!((mid - 0.5).abs() < 0.01);
/// ```
///
/// # Note
///
/// This implementation eliminates the discontinuity present in simple dead zone
/// implementations by rescaling the output. The control input now transitions
/// smoothly from 0.0 at the dead zone boundary to full deflection at maximum input.
fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
    // Validate that dead_zone is in valid range (0.0, 1.0)
    debug_assert!(
        dead_zone > 0.0 && dead_zone < 1.0,
        "Dead zone must be in range (0.0, 1.0), got {}",
        dead_zone
    );
    debug_assert!(
        value.is_finite(),
        "Input value must be finite, got {}",
        value
    );

    if value.abs() < dead_zone {
        0.0
    } else {
        // Preserve the sign of the input
        let sign = value.signum();
        // Scale the range [dead_zone, 1.0] to [0.0, 1.0]
        let scaled = (value.abs() - dead_zone) / (1.0 - dead_zone);
        sign * scaled
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Tests that the scaled dead zone function correctly handles inputs within the dead zone.
    #[test]
    fn test_apply_dead_zone_inside_dead_zone() {
        let dead_zone = 0.1;

        // Values inside dead zone should return 0.0
        assert_eq!(apply_dead_zone(0.0, dead_zone), 0.0);
        assert_eq!(apply_dead_zone(0.05, dead_zone), 0.0);
        assert_eq!(apply_dead_zone(-0.05, dead_zone), 0.0);
        assert_eq!(apply_dead_zone(0.09, dead_zone), 0.0);
        assert_eq!(apply_dead_zone(-0.09, dead_zone), 0.0);
    }

    /// Tests that the scaled dead zone function correctly handles the dead zone boundary.
    #[test]
    fn test_apply_dead_zone_at_boundary() {
        let dead_zone = 0.1;

        // At the exact dead zone boundary, should return 0.0
        assert_eq!(apply_dead_zone(0.1, dead_zone), 0.0);
        assert_eq!(apply_dead_zone(-0.1, dead_zone), 0.0);
    }

    /// Tests that the scaled dead zone function correctly scales values outside the dead zone.
    #[test]
    fn test_apply_dead_zone_outside_dead_zone() {
        let dead_zone = 0.1;

        // Maximum values should map to ±1.0
        assert_eq!(apply_dead_zone(1.0, dead_zone), 1.0);
        assert_eq!(apply_dead_zone(-1.0, dead_zone), -1.0);

        // Just outside dead zone should be close to 0.0 (smooth transition)
        let just_outside = apply_dead_zone(0.11, dead_zone);
        assert!(just_outside > 0.0 && just_outside < 0.02);

        let just_outside_neg = apply_dead_zone(-0.11, dead_zone);
        assert!(just_outside_neg < 0.0 && just_outside_neg > -0.02);
    }

    /// Tests that the scaled dead zone function provides smooth scaling (no discontinuity).
    #[test]
    fn test_apply_dead_zone_smooth_scaling() {
        let dead_zone = 0.1;

        // Midpoint between dead zone and max should scale to approximately 0.5
        let midpoint = (dead_zone + 1.0) / 2.0; // 0.55
        let result = apply_dead_zone(midpoint, dead_zone);
        assert!((result - 0.5).abs() < 0.01, "Expected ~0.5, got {}", result);

        // 75% point should scale to approximately 0.75
        let three_quarter = dead_zone + 0.75 * (1.0 - dead_zone); // 0.775
        let result = apply_dead_zone(three_quarter, dead_zone);
        assert!(
            (result - 0.75).abs() < 0.01,
            "Expected ~0.75, got {}",
            result
        );

        // Negative values should mirror positive values
        let result_neg = apply_dead_zone(-midpoint, dead_zone);
        assert!(
            (result_neg + 0.5).abs() < 0.01,
            "Expected ~-0.5, got {}",
            result_neg
        );
    }

    /// Tests that the scaled dead zone function maintains symmetry for positive and negative inputs.
    #[test]
    fn test_apply_dead_zone_symmetry() {
        let dead_zone = 0.1;

        // Test multiple points to ensure symmetry
        let test_values = vec![0.0, 0.2, 0.5, 0.8, 1.0];

        for value in test_values {
            let pos_result = apply_dead_zone(value, dead_zone);
            let neg_result = apply_dead_zone(-value, dead_zone);

            assert!(
                (pos_result + neg_result).abs() < 1e-6,
                "Asymmetry detected: f({}) = {}, f({}) = {}",
                value,
                pos_result,
                -value,
                neg_result
            );
        }
    }

    /// Table-driven test for scaled dead zone with various dead zone thresholds.
    #[test]
    fn test_apply_dead_zone_table_driven() {
        let test_cases = vec![
            // (input, dead_zone, expected_output)
            (0.0, 0.1, 0.0),
            (0.05, 0.1, 0.0),
            (0.1, 0.1, 0.0),
            (0.55, 0.1, 0.5),
            (1.0, 0.1, 1.0),
            (-0.55, 0.1, -0.5),
            (-1.0, 0.1, -1.0),
            // Different dead zone (15% for yaw)
            (0.0, 0.15, 0.0),
            (0.15, 0.15, 0.0),
            (0.575, 0.15, 0.5),
            (1.0, 0.15, 1.0),
            // Larger dead zone
            (0.0, 0.3, 0.0),
            (0.3, 0.3, 0.0),
            (0.65, 0.3, 0.5),
            (1.0, 0.3, 1.0),
        ];

        for (input, dead_zone, expected) in test_cases {
            let result = apply_dead_zone(input, dead_zone);
            assert!(
                (result - expected).abs() < 0.01,
                "apply_dead_zone({}, {}) expected {}, got {}",
                input,
                dead_zone,
                expected,
                result
            );
        }
    }

    /// Tests the scale_value helper function.
    #[test]
    fn test_scale_value() {
        // Test scaling from [-1, 1] to [0, 1]
        assert_eq!(scale_value(-1.0, -1.0, 1.0), 0.0);
        assert_eq!(scale_value(0.0, -1.0, 1.0), 0.5);
        assert_eq!(scale_value(1.0, -1.0, 1.0), 1.0);

        // Test scaling from [THROTTLE_MIN, THROTTLE_MAX] to [0, 1]
        let scaled_min = scale_value(THROTTLE_MIN, THROTTLE_MIN, THROTTLE_MAX);
        let scaled_max = scale_value(THROTTLE_MAX, THROTTLE_MIN, THROTTLE_MAX);
        assert!((scaled_min - 0.0).abs() < 1e-6);
        assert!((scaled_max - 1.0).abs() < 1e-6);

        // Test scaling from arbitrary range
        assert_eq!(scale_value(5.0, 0.0, 10.0), 0.5);
        assert_eq!(scale_value(0.0, 0.0, 10.0), 0.0);
        assert_eq!(scale_value(10.0, 0.0, 10.0), 1.0);
    }

    /// Tests that dead zone constants are within valid ranges.
    #[test]
    #[allow(clippy::assertions_on_constants)]
    fn test_dead_zone_constants() {
        assert!(ROLL_PITCH_DEAD_ZONE > 0.0 && ROLL_PITCH_DEAD_ZONE < 1.0);
        assert!(YAW_DEAD_ZONE > 0.0 && YAW_DEAD_ZONE < 1.0);
        assert!(
            YAW_DEAD_ZONE > ROLL_PITCH_DEAD_ZONE,
            "Yaw dead zone should be larger"
        );
    }

    /// Tests that throttle constants are valid.
    #[test]
    #[allow(clippy::assertions_on_constants)]
    fn test_throttle_constants() {
        assert!(THROTTLE_MIN > -1.0 && THROTTLE_MIN < 0.0);
        assert!(THROTTLE_MAX > 0.0 && THROTTLE_MAX < 1.0);
        assert!(THROTTLE_MAX > THROTTLE_MIN);
    }

    /// Table-driven test for apply_dead_zone input validation in debug mode.
    /// Tests that invalid inputs trigger debug assertions.
    #[test]
    #[cfg(debug_assertions)]
    fn test_apply_dead_zone_validation() {
        use std::panic;

        let test_cases = vec![
            // (value, dead_zone, description)
            (0.5, 1.0, "dead_zone = 1.0 should panic"),
            (0.5, 1.5, "dead_zone > 1.0 should panic"),
            (0.5, -0.1, "negative dead_zone should panic"),
            (0.5, 0.0, "zero dead_zone should panic"),
            (f32::INFINITY, 0.1, "infinite value should panic"),
            (
                f32::NEG_INFINITY,
                0.1,
                "negative infinite value should panic",
            ),
            (f32::NAN, 0.1, "NaN value should panic"),
        ];

        for (value, dead_zone, description) in test_cases {
            let result = panic::catch_unwind(|| {
                apply_dead_zone(value, dead_zone);
            });

            assert!(
                result.is_err(),
                "Expected panic for case: {} (value={}, dead_zone={})",
                description,
                value,
                dead_zone
            );
        }
    }

    /// Tests that apply_dead_zone accepts valid inputs without panicking.
    #[test]
    fn test_apply_dead_zone_valid_inputs() {
        let test_cases = vec![
            // (value, dead_zone, description)
            (0.5, 0.1, "normal input"),
            (0.0, 0.1, "zero value"),
            (-0.5, 0.1, "negative value"),
            (1.0, 0.1, "maximum value"),
            (-1.0, 0.1, "minimum value"),
            (0.5, 0.01, "small dead_zone"),
            (0.5, 0.99, "large dead_zone"),
        ];

        for (value, dead_zone, description) in test_cases {
            let _ = apply_dead_zone(value, dead_zone);
            assert!(
                true,
                "Successfully handled case: {} (value={}, dead_zone={})",
                description, value, dead_zone
            );
        }
    }
}
