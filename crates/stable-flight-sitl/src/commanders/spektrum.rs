//! Spektrum receiver module
//! Receives control inputs from Spektrum radio attached to computer
//! via USB dongle

use gilrs::{Axis, Button, Event, GamepadId, Gilrs};

use crate::commanders::{CommanderError, open_gilrs};
use stable_flight::{AttitudeCommand, Command, Commander, VehicleState};

/// [`Commander`] driven by a Spektrum transmitter connected over a USB dongle.
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

    /// Read control channels from Spektrum radio
    /// 0: throttle (right trigger)
    /// 1: roll (right stick x)
    /// 2: pitch (left stick y)
    /// 3: yaw (left stick x)
    /// 4: arm (right stick y)
    /// 5: mode (left trigger)  
    pub fn read_channels(&mut self) -> [f32; 6] {
        while let Some(Event { id, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);
        }

        let mut channels = [0.0; 6];

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            if let Some(data) = gamepad.button_data(Button::RightTrigger2) {
                channels[0] = scale_value(data.value(), 0.1, 0.9);
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickX) {
                channels[1] = scale_value(data.value(), -0.7, 0.7);
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                channels[2] = 1.0 - scale_value(data.value(), -0.65, 0.65);
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                channels[3] = 1.0 - scale_value(data.value(), -0.65, 0.65);
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
                channels[4] = if data.value() < 0.0 { 0.0 } else { 1.0 };
            }

            if let Some(data) = gamepad.button_data(Button::LeftTrigger2) {
                let value = data.value();
                channels[5] = if value < 0.4 {
                    1.0
                } else if value > 0.6 {
                    0.0
                } else {
                    0.5
                };
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

        Command::Attitude(AttitudeCommand {
            throttle: channels[0],
            roll: channels[1],
            pitch: channels[2],
            yaw: channels[3],
            arm: channels[4] > 0.5,
            mode,
        })
    }
}

/// Maps `value` from `[min, max]` to `[0, 1]`, clamped: the configured ranges are narrower than
/// the stick's physical travel (e.g. ±0.7 of ±1.0), so unclamped values overshoot to ~1.2.
fn scale_value(value: f32, min: f32, max: f32) -> f32 {
    ((value - min) / (max - min)).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::scale_value;

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
        ];
        for (value, min, max, expected) in cases {
            let scaled = scale_value(value, min, max);
            assert!(
                (scaled - expected).abs() < 1e-6,
                "scale_value({value}, {min}, {max}) = {scaled}, expected {expected}"
            );
        }
    }
}
