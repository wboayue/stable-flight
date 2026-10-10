//! Spektrum transmitter commander.
//!
//! Reads a Spektrum transmitter bound to the [Spektrum WS3000T] wireless simulator dongle, which
//! shows up as a generic HID gamepad. Its axes reach gilrs under gamepad names that don't match
//! the physical sticks, so the mapping below is per dongle axis:
//!
//! | Transmitter channel | Dongle axis (gilrs)     | Command                         |
//! |---------------------|-------------------------|---------------------------------|
//! | Throttle            | `RightTrigger2` button  | throttle `[0, 1]`               |
//! | Aileron             | `RightStickX`           | roll `[-1, 1]`, + right bank    |
//! | Elevator            | `LeftStickY` (negated)  | pitch `[-1, 1]`, + nose up      |
//! | Rudder              | `LeftStickX` (negated)  | yaw `[-1, 1]`, + right turn     |
//! | Gear (arm switch)   | `RightStickY`           | arm above 0.5                   |
//! | Mode switch         | `LeftTrigger2` button   | `AttitudeCommand::mode` `0..=2` |
//!
//! The `spektrum_commander` example prints the raw dongle values next to the command, to check
//! the mapping and stick travel on a given setup.
//!
//! [Spektrum WS3000T]: https://www.spektrumrc.com/product/spektrum-ws3000t-wireless-simulator-usb-dongle/SPMWS3000T.html

use gilrs::{Axis, Button, Event, GamepadId, Gilrs};

use crate::commanders::{CommanderError, open_gilrs, scale_signed, scale_value};
use stable_flight::{AttitudeCommand, Command, Commander, ControlInput, VehicleState};

/// Throttle button value at idle; the dongle doesn't reach 0.0.
const THROTTLE_MIN: f32 = 0.1;
/// Throttle button value at full; the dongle doesn't reach 1.0.
const THROTTLE_MAX: f32 = 0.9;
/// Aileron axis travel each side of center; full stick reads about this, not 1.0.
const ROLL_HALF_RANGE: f32 = 0.7;
/// Elevator and rudder axis travel each side of center.
const PITCH_YAW_HALF_RANGE: f32 = 0.65;

/// [`Commander`] driven by a Spektrum transmitter through the WS3000T USB dongle (see the
/// [module docs](self) for the channel mapping).
pub struct SpektrumCommander {
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
}

impl SpektrumCommander {
    /// Opens the gamepad subsystem; the transmitter is picked up on its first input event.
    ///
    /// # Errors
    ///
    /// Returns [`CommanderError::InitializationFailed`] if Gilrs cannot be initialized
    /// (input subsystem unavailable or not permitted).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use stable_flight_sitl::commanders::SpektrumCommander;
    ///
    /// # fn main() -> Result<(), stable_flight_sitl::commanders::CommanderError> {
    /// let commander = SpektrumCommander::new()?;
    /// # Ok(())
    /// # }
    /// ```
    pub fn new() -> Result<Self, CommanderError> {
        Ok(Self {
            gilrs: open_gilrs()?,
            active_gamepad: None,
        })
    }

    /// Reads the control channels, all `0.0` until the dongle's first event:
    /// - `[0]`: throttle, `0.0` to `1.0`
    /// - `[1]`: roll, `-1.0` (left) to `1.0` (right)
    /// - `[2]`: pitch, `-1.0` (nose down) to `1.0` (nose up)
    /// - `[3]`: yaw, `-1.0` (left) to `1.0` (right)
    /// - `[4]`: arm, `0.0` or `1.0`
    /// - `[5]`: mode switch, `0.0`, `0.5`, or `1.0`
    pub fn read_channels(&mut self) -> [f32; 6] {
        while let Some(Event { id, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);
        }

        let mut channels = [0.0; 6];

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            if let Some(data) = gamepad.button_data(Button::RightTrigger2) {
                channels[0] = throttle_channel(data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickX) {
                channels[1] = roll_channel(data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                channels[2] = pitch_channel(data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                channels[3] = yaw_channel(data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
                channels[4] = arm_channel(data.value());
            }

            if let Some(data) = gamepad.button_data(Button::LeftTrigger2) {
                channels[5] = mode_channel(data.value());
            }
        }

        self.gilrs.inc();

        channels
    }
}

impl Commander for SpektrumCommander {
    fn next_command(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
        let channels = self.read_channels();

        let mode = if channels[5] < 0.45 {
            0
        } else if channels[5] < 0.55 {
            1
        } else {
            2
        };

        Command::new(
            channels[4] > ARM_THRESHOLD,
            ControlInput::Attitude(AttitudeCommand {
                throttle: channels[0],
                roll: channels[1],
                pitch: channels[2],
                yaw: channels[3],
                mode,
            }),
        )
    }
}

/// Arm-switch axis value above which the commander arms. The axis rests at 0.0 (and gilrs
/// reports 0.0 before the first event), so a centered or unread switch stays disarmed.
const ARM_THRESHOLD: f32 = 0.5;

/// Maps the arm-switch axis to the arm channel: `1.0` (armed) only past [`ARM_THRESHOLD`].
fn arm_channel(value: f32) -> f32 {
    if value > ARM_THRESHOLD { 1.0 } else { 0.0 }
}

/// Maps the three-position mode switch to `1.0` (below 0.4), `0.0` (above 0.6), or `0.5`
/// (between, or NaN).
fn mode_channel(value: f32) -> f32 {
    if value < 0.4 {
        1.0
    } else if value > 0.6 {
        0.0
    } else {
        0.5
    }
}

/// Maps the throttle button to `[0, 1]`.
fn throttle_channel(value: f32) -> f32 {
    scale_value(value, THROTTLE_MIN, THROTTLE_MAX)
}

/// Maps the aileron axis to roll, `+` for a right bank.
fn roll_channel(value: f32) -> f32 {
    scale_signed(value, ROLL_HALF_RANGE)
}

/// Maps the elevator axis to pitch, `+` for nose up: the axis reads negative with the stick
/// pulled back.
fn pitch_channel(value: f32) -> f32 {
    -scale_signed(value, PITCH_YAW_HALF_RANGE)
}

/// Maps the rudder axis to yaw, `+` for a right turn: the axis reads negative with right rudder.
fn yaw_channel(value: f32) -> f32 {
    -scale_signed(value, PITCH_YAW_HALF_RANGE)
}

#[cfg(test)]
mod tests {
    use approx::relative_eq;

    use super::{
        arm_channel, mode_channel, pitch_channel, roll_channel, throttle_channel, yaw_channel,
    };

    /// Only a deliberate switch throw arms; centered (or never-read) stays disarmed.
    #[test]
    fn arm_channel_arms_only_past_the_threshold() {
        // (axis value, expected channel)
        let cases = [
            (-1.0, 0.0),
            (0.0, 0.0), // centered / before the first event
            (0.5, 0.0),
            (0.51, 1.0),
            (1.0, 1.0),
            (f32::NAN, 0.0),
        ];
        for (value, expected) in cases {
            assert_eq!(arm_channel(value), expected, "axis {value}");
        }
    }

    /// One axis mapping: raw dongle value to command.
    type Channel = fn(f32) -> f32;

    /// Centered sticks command level, wings-flat, no-yaw flight (#65: they read 0.5, a constant
    /// bank), and each axis reaches full deflection with the `AttitudeCommand` sign.
    #[test]
    fn sticks_map_to_signed_unit_range() {
        // (axis, raw dongle value, expected command)
        let cases: [(&str, Channel, f32, f32); 15] = [
            ("roll", roll_channel, 0.0, 0.0),
            ("roll", roll_channel, 0.7, 1.0), // right stick: right bank
            ("roll", roll_channel, -0.7, -1.0),
            ("roll", roll_channel, 1.0, 1.0), // past the configured travel
            ("pitch", pitch_channel, 0.0, 0.0),
            ("pitch", pitch_channel, -0.65, 1.0), // stick back: nose up
            ("pitch", pitch_channel, 0.65, -1.0),
            ("pitch", pitch_channel, -1.0, 1.0),
            ("yaw", yaw_channel, 0.0, 0.0),
            ("yaw", yaw_channel, -0.65, 1.0), // right rudder: right turn
            ("yaw", yaw_channel, 0.65, -1.0),
            ("yaw", yaw_channel, 1.0, -1.0),
            ("throttle", throttle_channel, 0.1, 0.0),
            ("throttle", throttle_channel, 0.5, 0.5),
            ("throttle", throttle_channel, 0.95, 1.0),
        ];
        for (axis, channel, raw, expected) in cases {
            let command = channel(raw);
            assert!(
                relative_eq!(command, expected, epsilon = 1e-6),
                "{axis}({raw}) = {command}, expected {expected}"
            );
        }
    }

    #[test]
    fn mode_channel_maps_switch_positions() {
        // (switch value, expected channel)
        let cases = [
            (0.0, 1.0),
            (0.39, 1.0),
            (0.4, 0.5),
            (0.5, 0.5),
            (0.6, 0.5),
            (0.61, 0.0),
            (1.0, 0.0),
            (f32::NAN, 0.5),
        ];
        for (value, expected) in cases {
            assert_eq!(mode_channel(value), expected, "switch {value}");
        }
    }
}
