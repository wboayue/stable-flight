//! Spektrum receiver module
//! Receives control inputs from Spektrum radio attached to computer
//! via USB dongle

use std::cell::{Ref, RefCell};

use gilrs::{Axis, Button, Event, EventType, GamepadId, Gilrs};
use log::debug;

use crate::{ControlInput, Receiver};

pub struct SpectrumReceiver {
    gilrs: RefCell<Gilrs>,
    active_gamepad: RefCell<Option<GamepadId>>,
}

impl SpectrumReceiver {
    pub fn new() -> SpectrumReceiver {
        SpectrumReceiver {
            gilrs: RefCell::new(Gilrs::new().expect("Failed to initialize Gilrs")),
            active_gamepad: RefCell::new(None),
        }
    }
}

impl Receiver for SpectrumReceiver {
    fn get_controls(&self) -> ControlInput {
        while let Some(Event {
            id, event, time, ..
        }) = self.gilrs.borrow_mut().next_event()
        {
            match event {
                EventType::AxisChanged(Axis::LeftStickY, val, _) => {
                    println!("Throttle (3): {:?}", val);
                }
                EventType::AxisChanged(Axis::LeftStickX, val, _) => {
                    println!("Yaw (4): {:?}", val);
                }
                EventType::AxisChanged(Axis::RightStickX, val, _) => {
                    println!("Pitch (2): {:?}", val);
                }
                EventType::ButtonChanged(Button::RightTrigger2, val, _) => {
                    println!("Roll (1): {:?}", val);
                }
                EventType::AxisChanged(Axis::RightStickY, val, _) => {
                    println!("Aux (1): {:?}", val);
                }
                EventType::ButtonChanged(Button::LeftTrigger2, val, _) => {
                    println!("Aux (2): {:?}", val);
                }
                e => {
                    debug!("{:?}", e)
                }
            }
            self.active_gamepad.replace(Some(id));
        }

        if let Some(id) = *self.active_gamepad.borrow() {
            let gilrs = self.gilrs.borrow();
            let gamepad = gilrs.gamepad(id);

            if let Some(data) = gamepad.button_data(Button::RightTrigger2) {
                println!("channel 1: {:?}", data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickX) {
                println!("channel 2: {:?}", data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                println!("channel 3: {:?}", data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                println!("channel 4: {:?}", data.value());
            }

            if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
                println!("channel 5: {:?}", data.value());
            }

            if let Some(data) = gamepad.button_data(Button::LeftTrigger2) {
                println!("channel 6: {:?}", data.value());
            }
        }

        self.gilrs.borrow_mut().inc();

        ControlInput {}
    }
}
