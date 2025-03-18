//! Logitic Extreme 3D Pro Joystick

use std::time::Duration;

use gilrs::{Axis, Button, Event, EventType, GamepadId, Gilrs};

use super::{AttitudeCommand, Command, Commander, RateCommand};
use crate::VehicleState;

const THROTTLE_CODE: u32 = 65590;

pub struct Extreme3DCommander {
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    // control channels
    channels: [f32; 6],
}

impl Extreme3DCommander {
    pub fn new() -> Self {
        Self {
            gilrs: Gilrs::new().expect("Failed to initialize Gilrs"),
            active_gamepad: None,
            channels: [0.0; 6],
        }
    }

    /// Read control channels from Spektrum radio
    /// 0: throttle (right trigger)
    /// 1: roll (right stick x)
    /// 2: pitch (left stick y)
    /// 3: yaw (left stick x)
    /// 4: arm (right stick y)
    /// 5: mode (left trigger)  
    pub fn read_channels(&mut self) -> [f32; 6] {
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);

            match event {
                EventType::AxisChanged(_, value, code) => {
                    if code.into_u32() == THROTTLE_CODE {
                        self.channels[0] = 1.0 - scale_value(value, -0.99, 0.99).clamp(0.0, 1.0);
                    }
                }
                _ => {}
            }
        }

        // FIXME: This is a hack to ensure throttle is always set to 1.0
        self.channels[0] = 0.81;

        if let Some(id) = self.active_gamepad {
            let gamepad = self.gilrs.gamepad(id);

            // roll
            if let Some(data) = gamepad.axis_data(Axis::LeftStickX) {
                self.channels[1] = data.value();
            }

            // pitch
            if let Some(data) = gamepad.axis_data(Axis::LeftStickY) {
                self.channels[2] = -data.value();
            }

            // yaw
            if let Some(data) = gamepad.button_data(Button::LeftTrigger2) {
                self.channels[3] = scale_value(data.value(), 0.0, 1.0) * 2.0 - 1.0;
            }

            //     if let Some(data) = gamepad.axis_data(Axis::RightStickY) {
            //         channels[4] = if data.value() < 0.0 { 0.0 } else { 1.0 };
            //     }

            //     if let Some(data) = gamepad.button_data(Button::LeftTrigger2) {
            //         let value = data.value();
            //         channels[5] = if value < 0.4 {
            //             1.0
            //         } else if value > 0.6 {
            //             0.0
            //         } else {
            //             0.5
            //         };
            //     }
        }

        self.gilrs.inc();

        self.channels
    }
}

impl Default for Extreme3DCommander {
    fn default() -> Self {
        Self::new()
    }
}

impl Commander for Extreme3DCommander {
    fn next(&mut self, _dt: Duration, _current_state: &VehicleState) -> Command {
        let channels = self.read_channels();

        let mode = if channels[5] < 0.45 {
            0
        } else if channels[5] < 0.55 {
            1
        } else {
            2
        };

        // Command::Attitude(AttitudeCommand {
        //     throttle: channels[0],
        //     roll: channels[1],
        //     pitch: channels[2],
        //     yaw: channels[3],
        //     arm: channels[4] > 0.5,
        //     mode,
        // })
        Command::Rate(RateCommand {
            throttle: channels[0],
            roll_rate: apply_dead_zone(channels[1], 0.1),
            pitch_rate: apply_dead_zone(channels[2], 0.1),
            yaw_rate: -apply_dead_zone(channels[3], 0.15),
            arm: channels[4] > 0.5,
            mode,
        })
    }
}

fn scale_value(value: f32, min: f32, max: f32) -> f32 {
    (value - min) / (max - min)
}

fn apply_dead_zone(value: f32, dead_zone: f32) -> f32 {
    if value.abs() < dead_zone {
        0.0
    } else {
        value
    }
}