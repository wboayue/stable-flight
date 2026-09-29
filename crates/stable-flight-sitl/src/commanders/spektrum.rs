//! Spektrum receiver module
//! Receives control inputs from Spektrum radio attached to computer
//! via USB dongle

use std::cell::RefCell;

use gilrs::{Axis, Button, Event, GamepadId, Gilrs};

use crate::{AttitudeCommand, Command, Commander, VehicleState};

pub struct SpectrumCommander {
    gilrs: RefCell<Gilrs>,
    active_gamepad: RefCell<Option<GamepadId>>,
    // control channels
    channels: RefCell<[f32; 6]>,
}

impl SpectrumCommander {
    pub fn new() -> SpectrumCommander {
        SpectrumCommander {
            gilrs: RefCell::new(Gilrs::new().expect("Failed to initialize Gilrs")),
            active_gamepad: RefCell::new(None),
            channels: RefCell::new([0.0; 6]),
        }
    }

    /// Read control channels from Spektrum radio
    /// 0: throttle (right trigger)
    /// 1: roll (right stick x)
    /// 2: pitch (left stick y)
    /// 3: yaw (left stick x)
    /// 4: arm (right stick y)
    /// 5: mode (left trigger)  
    pub fn read_channels(&self) -> [f32; 6] {
        let mut gilrs = self.gilrs.borrow_mut();

        while let Some(Event { id, .. }) = gilrs.next_event() {
            self.active_gamepad.replace(Some(id));
        }

        let mut channels = [0.0; 6];

        if let Some(id) = *self.active_gamepad.borrow() {
            let gamepad = gilrs.gamepad(id);

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

        gilrs.inc();

        channels
    }
}

impl Default for SpectrumCommander {
    fn default() -> Self {
        Self::new()
    }
}

impl Commander for SpectrumCommander {
    fn next(&mut self, _dt: f32, _current_state: &VehicleState) -> Command {
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

fn scale_value(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}
