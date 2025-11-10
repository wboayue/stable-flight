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
//! To reduce noise and improve control precision, dead zones are applied:
//! - Roll/Pitch: 10% (±0.1)
//! - Yaw: 15% (±0.15) - larger due to twist axis sensitivity
//!
//! # Examples
//!
//! Basic usage with attitude control:
//! ```no_run
//! use stable_flight_sitl::commanders::extreme_3d::{Extreme3DCommander, Mode};
//! use stable_flight_sitl::{Commander, VehicleState};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! // Create commander in attitude mode
//! let mut commander = Extreme3DCommander::new(Mode::Attitude)?;
//!
//! // Generate commands in a control loop
//! let state = VehicleState::default();
//! let command = commander.next(0.004, &state); // 250 Hz (4ms)
//! # Ok(())
//! # }
//! ```
//!
//! Using rate control mode for acrobatic flight:
//! ```no_run
//! use stable_flight_sitl::commanders::extreme_3d::{Extreme3DCommander, Mode};
//!
//! # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
//! let mut commander = Extreme3DCommander::new(Mode::Rate)?;
//! // Commander will now generate rate commands instead of attitude commands
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
/// # Control Channels
///
/// Internally maintains 6 control channels:
/// - `channels[0]`: Throttle (0.0 to 1.0)
/// - `channels[1]`: Roll (-1.0 to 1.0)
/// - `channels[2]`: Pitch (-1.0 to 1.0)
/// - `channels[3]`: Yaw (-1.0 to 1.0)
/// - `channels[4]`: Arm state (0.0 = disarmed, 1.0 = armed)
/// - `channels[5]`: Flight mode (0.0, 1.0, or 2.0)
///
/// # Thread Safety
///
/// This struct is not thread-safe and should only be used from a single thread.
/// The Gilrs event loop internally manages joystick state.
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl::commanders::extreme_3d::{Extreme3DCommander, Mode};
/// use stable_flight_sitl::{Commander, VehicleState};
///
/// # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
/// let mut commander = Extreme3DCommander::new(Mode::Attitude)?;
///
/// // In your control loop (e.g., at 250 Hz)
/// let state = VehicleState::default();
/// let dt = 0.004; // 4ms between updates
/// let command = commander.next(dt, &state);
///
/// match command {
///     stable_flight_sitl::Command::Attitude(cmd) => {
///         println!("Throttle: {}, Roll: {}", cmd.throttle, cmd.roll);
///     }
///     _ => {}
/// }
/// # Ok(())
/// # }
/// ```
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
    /// Creates a new Extreme3DCommander instance.
    ///
    /// # Parameters
    ///
    /// - `mode`: The control mode to use (Attitude or Rate)
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
    /// ```no_run
    /// use stable_flight_sitl::commanders::{Extreme3DCommander, extreme_3d::Mode};
    ///
    /// let commander = Extreme3DCommander::new(Mode::Attitude)?;
    /// # Ok::<(), stable_flight_sitl::commanders::CommanderError>(())
    /// ```
    pub fn new(mode: Mode) -> Result<Self, CommanderError> {
        let gilrs = Gilrs::new().map_err(|e| {
            CommanderError::InitializationFailed(format!("Gilrs initialization failed: {}", e))
        })?;

        Ok(Self {
            gilrs,
            active_gamepad: None,
            channels: [0.0; 6],
            mode,
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
                    // arm
                    self.channels[4] = 1.0;
                }
                EventType::ButtonChanged(Button::Unknown, 1.0, code) => {
                    // disarm
                    if code.into_u32() == DISARM_BUTTON_CODE {
                        self.channels[4] = 0.0;
                    }
                }
                EventType::ButtonChanged(Button::DPadLeft, 1.0, _) => {
                    // mode 0
                    self.channels[5] = 0.0;
                }
                EventType::ButtonChanged(Button::DPadUp, 1.0, _) => {
                    // mode 1
                    self.channels[5] = 1.0;
                }
                EventType::ButtonChanged(Button::DPadRight, 1.0, _) => {
                    // mode 2
                    self.channels[5] = 2.0;
                }
                _ => {}
            }
        }

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            // roll
            gamepad.axis_data(Axis::LeftStickX).map(|data| {
                self.channels[1] = data.value();
            });

            // pitch
            gamepad.axis_data(Axis::LeftStickY).map(|data| {
                self.channels[2] = -data.value();
            });

            // yaw
            gamepad.axis_data(Axis::RightStickY).map(|data| {
                self.channels[3] = -data.value();
            });
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
        Self::new(Mode::Attitude)
            .expect("Failed to initialize Extreme3DCommander with default settings")
    }
}

impl Commander for Extreme3DCommander {
    fn next(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        let channels = self.read_channels();

        let mode = channels[5] as u8;

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

/// Applies a dead zone to a control input value.
///
/// Values with absolute magnitude less than the dead zone threshold are
/// zeroed out to prevent stick drift and improve control precision.
///
/// # Parameters
///
/// - `value`: The input value (typically -1.0 to 1.0)
/// - `dead_zone`: The dead zone threshold (positive value)
///
/// # Returns
///
/// - `0.0` if `|value| < dead_zone`
/// - `value` otherwise
///
/// # Examples
///
/// ```
/// # fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
/// #     if value.abs() < dead_zone {
/// #         0.0
/// #     } else {
/// #         value
/// #     }
/// # }
/// assert_eq!(apply_dead_zone(0.05, 0.1), 0.0);  // Inside dead zone
/// assert_eq!(apply_dead_zone(-0.05, 0.1), 0.0); // Inside dead zone
/// assert_eq!(apply_dead_zone(0.5, 0.1), 0.5);   // Outside dead zone
/// assert_eq!(apply_dead_zone(-0.5, 0.1), -0.5); // Outside dead zone
/// ```
///
/// # Note
///
/// This implementation does not rescale the output. Values outside the dead zone
/// are passed through unchanged, meaning there's a discontinuity at the dead zone
/// boundary. For smoother control, consider using a scaled dead zone that
/// remaps the range [dead_zone, 1.0] to [0.0, 1.0].
fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
    if value.abs() < dead_zone {
        0.0
    } else {
        value
    }
}
