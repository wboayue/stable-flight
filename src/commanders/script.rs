use crate::math::Vec2;

use super::{Commander, Command, WaypointCommand};


pub struct ScriptCommander {
    pub path: String,
}

impl ScriptCommander {
    pub fn new(script: &str) -> Self {
        ScriptCommander { path: script.to_string() }
    }
}

impl Commander for ScriptCommander {
    fn next(&self, dt: std::time::Duration) -> super::Command {
        Command::Waypoint(WaypointCommand { 
            position: Vec2{ x: 0.0, y: 0.0 }, 
            velocity: None, 
            altitude: 10.0, 
            cruise_speed: 10.0, 
            roll: None 
        })
    }
}
