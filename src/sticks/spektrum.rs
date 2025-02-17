//! Spektrum receiver module
//! Receives control inputs from Spektrum radio attached to computer
//! via USB dongle

use std::cell::RefCell;

use gilrs::{Axis, Button, Event, GamepadId, Gilrs};

use crate::{ControlInput, Receiver};

pub struct SpectrumStick {
    gilrs: RefCell<Gilrs>,
    active_gamepad: RefCell<Option<GamepadId>>,
    // control channels
    channels: RefCell<[f64; 6]>,
}

impl SpectrumStick {
    pub fn new() -> SpectrumStick {
        SpectrumStick {
            gilrs: RefCell::new(Gilrs::new().expect("Failed to initialize Gilrs")),
            active_gamepad: RefCell::new(None),
            channels: RefCell::new([0.0; 6]),
        }
    }
}

impl Default for SpectrumStick {
    fn default() -> Self {
        Self::new()
    }
}

impl Receiver for SpectrumStick {
    fn get_controls(&self) -> ControlInput {
        let mut gilrs = self.gilrs.borrow_mut();

        while let Some(Event { id, .. }) = gilrs.next_event() {
            self.active_gamepad.replace(Some(id));
        }

        let mut channels = self.channels.borrow_mut();

        if let Some(id) = *self.active_gamepad.borrow() {
            let gamepad = gilrs.gamepad(id);

            if let Some(data) = gamepad.button_data(Button::RightTrigger2) {
                channels[0] = scale_value(data.value() as f64, 0.1, 0.9);
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickX) {
                channels[1] = scale_value(data.value() as f64, -0.7, 0.7);
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                channels[2] = 1.0 - scale_value(data.value() as f64, -0.65, 0.65);
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                channels[3] = 1.0 - scale_value(data.value() as f64, -0.65, 0.65);
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

        let mut scaled_output = [0.0; 12];

        for i in 0..6 {
            scaled_output[i] = channels[i];
        }

        ControlInput {
            channels: scaled_output,
        }
    }
}

fn scale_value(value: f64, min: f64, max: f64) -> f64 {
    (value - min) / (max - min)
}
