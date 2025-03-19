use robomath::Vec2;

use crate::VehicleState;

use super::{Command, Commander, WaypointCommand};

#[derive(Clone)]
pub struct ScriptCommander {
    pub path: String,
}

impl ScriptCommander {
    pub fn new(script: &str) -> Self {
        ScriptCommander {
            path: script.to_string(),
        }
    }
}

impl Commander for ScriptCommander {
    fn next(&mut self, dt: std::time::Duration, _current_state: &VehicleState) -> super::Command {
        Command::Waypoint(WaypointCommand {
            position: Vec2 { x: 0.0, y: 0.0 },
            velocity: None,
            altitude: 10.0,
            cruise_speed: 10.0,
            roll: None,
        })
    }
}
