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
//! use stable_flight::{Commander, VehicleState};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! // Create commander in attitude mode
//! let mut commander = Extreme3DCommander::new()?;
//!
//! // Generate commands in a control loop
//! let state = VehicleState::default();
//! let command = commander.next_command(0.004, &state); // 250 Hz (4ms)
//! # Ok(())
//! # }
//! ```
//!
//! Mode and regime selection are done at runtime using the D-pad buttons:
//! - D-Pad Up: Rate mode for acrobatic flight, in the current regime (acro rolls and loops in
//!   plane)
//! - D-Pad Left: Attitude mode, copter regime
//! - D-Pad Right: Attitude mode, plane regime (a VTOL transitions; a copter refuses it)
//!
//! ```no_run
//! use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
//! use stable_flight::{Commander, VehicleState};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! let mut commander = Extreme3DCommander::new()?;
//! let state = VehicleState::default();
//!
//! // Press D-Pad Up on the joystick to switch to Rate mode
//! // The commander will automatically switch modes based on D-pad input
//! let command = commander.next_command(0.004, &state);
//! // Command type (Attitude or Rate) depends on current D-pad selection
//! # Ok(())
//! # }
//! ```

use gilrs::{Axis, Button, Event, EventType, GamepadId, Gilrs};
use log::debug;

use crate::commanders::{CommanderError, open_gilrs, scale_value};
use stable_flight::{
    AttitudeCommand, Command, Commander, ControlInput, FlightRegime, RateCommand, VehicleState,
};

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

/// Threshold for arm channel to be considered armed.
/// Values above this threshold indicate the vehicle should be armed.
const ARM_THRESHOLD: f32 = 0.5;

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

impl Mode {
    /// Converts mode to channel value for storage in channels[5].
    fn to_channel_value(self) -> f32 {
        match self {
            Self::Attitude => 0.0,
            Self::Rate => 1.0,
        }
    }
}

/// One joystick input the commander reads, decoded from a gilrs event: gilrs's name for the
/// button or axis, its value, and the native code (stable across gilrs's mappings).
#[derive(Debug, Clone, Copy, PartialEq)]
enum JoystickInput {
    /// A button's value changed (`ButtonChanged`).
    Button {
        button: Button,
        value: f32,
        code: u32,
    },
    /// An axis moved (`AxisChanged`).
    Axis { axis: Axis, value: f32, code: u32 },
}

impl JoystickInput {
    /// The input in `event`, or `None` for an event the commander doesn't read.
    fn decode(event: EventType) -> Option<Self> {
        match event {
            EventType::ButtonChanged(button, value, code) => Some(Self::Button {
                button,
                value,
                code: code.into_u32(),
            }),
            EventType::AxisChanged(axis, value, code) => Some(Self::Axis {
                axis,
                value,
                code: code.into_u32(),
            }),
            _ => None,
        }
    }
}

/// The commander's state from joystick events: the channels, the stick mode, and the
/// requested regime.
#[derive(Debug, Clone, Copy, PartialEq)]
struct JoystickState {
    /// `[throttle, roll, pitch, yaw, arm, mode]`, as [`Extreme3DCommander::read_channels`].
    channels: [f32; 6],
    /// Stick mode, set by D-pad Left and Right (attitude) and Up (rate).
    mode: Mode,
    /// Requested regime, set by D-pad Left (copter) and Right (plane).
    regime: FlightRegime,
}

impl JoystickState {
    /// Disarmed, centered, throttle at zero, in `mode` and the copter regime.
    fn new(mode: Mode) -> Self {
        let mut channels = [0.0; 6];
        channels[5] = mode.to_channel_value();
        Self {
            channels,
            mode,
            regime: FlightRegime::Copter,
        }
    }

    /// Applies one event's input: the throttle axis, the arm and disarm buttons, and D-pad
    /// presses. The stick axes are read from gilrs's state instead, each poll.
    fn apply(&mut self, input: JoystickInput) {
        match input {
            JoystickInput::Axis { value, code, .. } if code == THROTTLE_CODE => {
                // Throttle is inverted (1.0 at bottom, 0.0 at top) and scaled
                self.channels[0] = 1.0 - scale_value(value, THROTTLE_MIN, THROTTLE_MAX);
            }
            JoystickInput::Button {
                button: Button::North,
                value: 1.0,
                ..
            } => {
                self.channels[4] = 1.0;
            }
            JoystickInput::Button {
                button: Button::Unknown,
                value: 1.0,
                code,
            } if code == DISARM_BUTTON_CODE => {
                self.channels[4] = 0.0;
            }
            JoystickInput::Button {
                button, value: 1.0, ..
            } if is_dpad(button) => {
                (self.mode, self.regime) = dpad(button, self.mode, self.regime);
                self.channels[5] = self.mode.to_channel_value();
                debug!("D-pad {button:?}: {:?}, {:?}", self.mode, self.regime);
            }
            _ => {}
        }
    }
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

    /// Channels, stick mode, and requested regime from the joystick's events
    state: JoystickState,
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
        Ok(Self {
            gilrs: open_gilrs()?,
            active_gamepad: None,
            state: JoystickState::new(mode),
        })
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
    /// - `[5]`: Mode (0.0 attitude, 1.0 rate)
    ///
    /// # Performance
    ///
    /// This method is designed to be called at high frequency (e.g., 250 Hz)
    /// in the flight control loop. It processes all pending events and reads
    /// current axis values on each call.
    pub fn read_channels(&mut self) -> [f32; 6] {
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);
            if let Some(input) = JoystickInput::decode(event) {
                self.state.apply(input);
            }
        }

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            let channels = &mut self.state.channels;

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                channels[1] = data.value();
            }

            // Pitch and yaw are negated to match flight control conventions
            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                channels[2] = -data.value();
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
                channels[3] = -data.value();
            }
        }

        // Advance the Gilrs internal counter to process next frame of events
        self.gilrs.inc();

        self.state.channels
    }
}

impl Commander for Extreme3DCommander {
    /// Generates the next control command based on current joystick input.
    ///
    /// This method reads all pending joystick events, updates internal state,
    /// and returns either an `AttitudeCommand` or `RateCommand` depending on
    /// the current mode selected via D-pad buttons:
    /// - D-Pad Left/Right: `ControlInput::Attitude`
    /// - D-Pad Up: `ControlInput::Rate`
    ///
    /// Dead zones are automatically applied to roll, pitch, and yaw inputs.
    fn next_command(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        let channels = self.read_channels();
        let arm = channels[4] > ARM_THRESHOLD;

        let input = match self.state.mode {
            Mode::Attitude => ControlInput::Attitude(AttitudeCommand {
                throttle: channels[0],
                roll: apply_dead_zone(channels[1], ROLL_PITCH_DEAD_ZONE),
                pitch: apply_dead_zone(channels[2], ROLL_PITCH_DEAD_ZONE),
                yaw: apply_dead_zone(channels[3], YAW_DEAD_ZONE),
            }),
            Mode::Rate => ControlInput::Rate(RateCommand {
                throttle: channels[0],
                roll_rate: apply_dead_zone(channels[1], ROLL_PITCH_DEAD_ZONE),
                pitch_rate: apply_dead_zone(channels[2], ROLL_PITCH_DEAD_ZONE),
                yaw_rate: apply_dead_zone(channels[3], YAW_DEAD_ZONE),
            }),
        };
        Command::new(arm, input).with_regime(self.state.regime)
    }
}

/// Whether `button` is one of the D-pad's mode buttons.
fn is_dpad(button: Button) -> bool {
    matches!(
        button,
        Button::DPadLeft | Button::DPadUp | Button::DPadRight
    )
}

/// The mode and regime after a D-pad press: Left is attitude in copter, Right attitude in
/// plane, Up rate in the regime already requested; anything else changes nothing.
fn dpad(button: Button, mode: Mode, regime: FlightRegime) -> (Mode, FlightRegime) {
    match button {
        Button::DPadLeft => (Mode::Attitude, FlightRegime::Copter),
        Button::DPadRight => (Mode::Attitude, FlightRegime::Plane),
        Button::DPadUp => (Mode::Rate, regime),
        _ => (mode, regime),
    }
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

    /// D-pad Left: attitude in copter; Right: attitude in plane; Up: rate in the regime
    /// already requested. (button, starting mode and regime, expected mode and regime)
    #[test]
    fn dpad_sets_mode_and_regime() {
        use FlightRegime::{Copter, Plane};
        let cases = [
            (
                Button::DPadLeft,
                (Mode::Rate, Plane),
                (Mode::Attitude, Copter),
            ),
            (
                Button::DPadRight,
                (Mode::Rate, Copter),
                (Mode::Attitude, Plane),
            ),
            (Button::DPadUp, (Mode::Attitude, Plane), (Mode::Rate, Plane)),
            (
                Button::DPadUp,
                (Mode::Attitude, Copter),
                (Mode::Rate, Copter),
            ),
            (Button::South, (Mode::Rate, Plane), (Mode::Rate, Plane)),
        ];
        for (button, (mode, regime), expected) in cases {
            assert_eq!(dpad(button, mode, regime), expected, "{button:?}");
        }
        assert!(is_dpad(Button::DPadRight) && !is_dpad(Button::DPadDown));
    }

    fn button(button: Button, value: f32, code: u32) -> JoystickInput {
        JoystickInput::Button {
            button,
            value,
            code,
        }
    }

    /// D-pad codes `axis_dpad_to_button` gives on macOS: button page, usages 16-19 (`gilrs-core` 0.6.8 `io_kit.rs:340`).
    const DPAD_LEFT: u32 = 589_842;
    const DPAD_RIGHT: u32 = 589_843;
    const DPAD_UP: u32 = 589_840;

    /// Each sequence of inputs, applied from the initial state, leaves this mode and regime.
    /// Only a press (1.0) acts: a release, or another button, keeps what was requested.
    #[test]
    fn inputs_set_mode_and_regime() {
        use FlightRegime::{Copter, Plane};
        let right = button(Button::DPadRight, 1.0, DPAD_RIGHT);
        let cases: [(&str, &[JoystickInput], (Mode, FlightRegime)); 5] = [
            ("right", &[right], (Mode::Attitude, Plane)),
            (
                "right then release",
                &[right, button(Button::DPadRight, 0.0, DPAD_RIGHT)],
                (Mode::Attitude, Plane),
            ),
            (
                "right then left",
                &[right, button(Button::DPadLeft, 1.0, DPAD_LEFT)],
                (Mode::Attitude, Copter),
            ),
            (
                "right then up",
                &[right, button(Button::DPadUp, 1.0, DPAD_UP)],
                (Mode::Rate, Plane),
            ),
            (
                "right then arm",
                &[right, button(Button::North, 1.0, 589_829)],
                (Mode::Attitude, Plane),
            ),
        ];
        for (name, inputs, expected) in cases {
            let mut state = JoystickState::new(Mode::Attitude);
            for &input in inputs {
                state.apply(input);
            }
            assert_eq!((state.mode, state.regime), expected, "{name}");
            assert_eq!(state.channels[5], state.mode.to_channel_value(), "{name}");
        }
    }

    /// The arm button arms, the disarm button (by code) disarms, and the throttle axis (by
    /// code) sets channel 0, inverted.
    #[test]
    fn inputs_set_arm_and_throttle() {
        let mut state = JoystickState::new(Mode::Attitude);
        state.apply(button(Button::North, 1.0, 589_829));
        assert_eq!(state.channels[4], 1.0);
        state.apply(button(Button::Unknown, 1.0, DISARM_BUTTON_CODE + 1));
        assert_eq!(state.channels[4], 1.0, "another unknown button");
        state.apply(button(Button::Unknown, 1.0, DISARM_BUTTON_CODE));
        assert_eq!(state.channels[4], 0.0);

        let axis = |code| JoystickInput::Axis {
            axis: Axis::Unknown,
            value: THROTTLE_MIN,
            code,
        };
        state.apply(axis(THROTTLE_CODE + 1));
        assert_eq!(state.channels[0], 0.0, "another axis");
        state.apply(axis(THROTTLE_CODE));
        assert_eq!(state.channels[0], 1.0);
    }

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
            let result = apply_dead_zone(value, dead_zone);
            assert!(
                result.is_finite(),
                "{description} (value={value}, dead_zone={dead_zone}) gave {result}"
            );
        }
    }

    /// Tests Mode::to_channel_value returns correct values.
    #[test]
    fn test_mode_to_channel_value() {
        assert_eq!(Mode::Attitude.to_channel_value(), 0.0);
        assert_eq!(Mode::Rate.to_channel_value(), 1.0);
    }
}
