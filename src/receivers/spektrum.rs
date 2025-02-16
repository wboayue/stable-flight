//! Spektrum receiver module
//! Receives control inputs from Spektrum radio attached to computer
//! via USB dongle

use std::cell::RefCell;

use gilrs::{Axis, Button, Event, EventType, Gilrs};
use log::debug;

use crate::{ControlInput, Receiver};

pub struct SpectrumReceiver {
    gilrs: RefCell<Gilrs>,
}

impl SpectrumReceiver {
    pub fn new() -> SpectrumReceiver {
        SpectrumReceiver {
            gilrs: RefCell::new(Gilrs::new().expect("Failed to initialize Gilrs")),
        }
    }   
}

// AxisChanged(LeftStickY

impl Receiver for SpectrumReceiver {
    fn get_controls(&self) -> ControlInput {
        while let Some(Event { id, event, time, .. }) = self.gilrs.borrow_mut().next_event() {
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
                e => { debug!("{:?}", e) }
            }
        }

        ControlInput {}
    }
}
