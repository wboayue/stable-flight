//! # Flight Control Commands Module
//!
//! This module defines the core types and traits for generating and representing flight control commands.
//! It provides a unified interface that supports both manual and autonomous (waypoint-based) flight control,
//! facilitating a flexible integration into a broader flight control system.
//!
//! ## Overview
//!
//! The module consists of the following key components:
//!
//! - **[Commander]**: A trait for generating flight control commands based on elapsed time.
//! - **[Command]**: An enumeration that encapsulates different command types (manual and waypoint).
//! - **[ManualCommand]**: A structure representing direct control inputs such as throttle, roll, pitch, and yaw.
//! - **[WaypointCommand]**: A structure representing navigational commands including target position,
//!   optional velocity, altitude, cruise speed, and an optional roll angle for coordinated maneuvers.
//!
//! ## Detailed Description
//!
//! ### Commander Trait
//!
//! The `Commander` trait is responsible for producing the next flight control command given the elapsed time (`dt`).
//! This allows for time-dependent command generation, such as ramping values or coordinating timed sequences.
//!
//! Implementations of `Commander` can include:
//! - **User Input Handlers:** Translating joystick or keyboard inputs into direct commands.
//! - **Navigation Systems:** Generating waypoint commands for autonomous flight.
//! - **Mission Controllers:** Producing commands based on a predefined flight plan.
//! - **Formation Controllers:** Maintaining relative positioning among multiple aircraft.
//!
//! ### Command Enum
//!
//! The `Command` enum encapsulates the various control paradigms available:
//!
//! - **Manual:** Represents direct, low-level control inputs, typically with normalized values (e.g., throttle from 0.0 to 1.0).
//! - **Waypoint:** Represents high-level navigational instructions, including target position, altitude, and optional velocity.
//!
//! This design provides a unified interface to handle both manual and autonomous flight modes.
//!
//! ### ManualCommand Struct
//!
//! The `ManualCommand` struct encapsulates direct control inputs that typically come from pilot or user inputs.
//!
//! **Value Ranges:**
//! - `throttle`: 0.0 (idle) to 1.0 (full power)
//! - `roll`: -1.0 (left) to 1.0 (right)
//! - `pitch`: -1.0 (nose down) to 1.0 (nose up)
//! - `yaw`: -1.0 (left) to 1.0 (right)
//!
//! ### WaypointCommand Struct
//!
//! The `WaypointCommand` struct represents an autonomous navigation command.
//!
//! It specifies:
//! - A target position using a 2D vector (`Vec2`).
//! - An optional desired velocity vector at the waypoint for smoother transitions.
//! - The target altitude in meters.
//! - The desired cruise speed during transit.
//! - An optional desired roll angle (in radians) for coordinated turns or maneuvers.
//!
//! ## Usage Example
//!
//! Creating and using a manual command:
//!
//! ```rust
//! use stable_flight_sitl::commanders::{Command, AttitudeCommand};
//!
//! let manual_cmd = Command::Attitude(AttitudeCommand {
//!     throttle: 0.75,  // 75% power
//!     roll: 0.1,       // Slight right bank
//!     pitch: -0.05,    // Slight nose down
//!     yaw: 0.0,        // No yaw input
//!     ..Default::default()
//! });
//! ```
//!
//! Creating and using a waypoint command:
//!
//! ```rust
//! use stable_flight_sitl::commanders::{Command, WaypointCommand};
//! use stable_flight_sitl::math::{Vec2, vec2};
//!
//! let waypoint_cmd = Command::Waypoint(WaypointCommand {
//!     position: vec2(1000.0, 2000.0),  // Target position
//!     velocity: Some(vec2(15.0, 0.0)),  // Optional target velocity
//!     altitude: 500.0,                       // Target altitude in meters
//!     cruise_speed: 25.0,                    // Desired speed in m/s
//!     roll: Some(0.05),                      // Optional roll angle in radians
//! });
//! ```
//!
//! ## Integration
//!
//! This module is intended to be a foundational component of a larger flight control system.
//! By separating the command generation (`Commander` trait) from command representation (`Command` enum),
//! the design supports a flexible and extensible control architecture. This allows for easy integration of
//! different control paradigms and simplifies the addition of new command types in the future.

use std::time::Duration;

use crate::math::Vec2;

pub mod script;
pub mod spektrum;
#[cfg(test)]
mod tests;

/// Represents an entity that produces flight control commands.
///
/// The `Commander` trait is responsible for generating the next command to be executed
/// by a flight control system. It takes into account the elapsed time since the last
/// command (`dt`) to potentially produce time-dependent commands.
///
/// Implementors of this trait may include:
/// * User input handlers that translate joystick/keyboard inputs into manual commands
/// * Navigation systems that generate waypoint commands for autonomous flight
/// * Mission controllers that produce commands based on predefined flight plans
/// * Formation controllers that maintain relative positioning
/// ```
pub trait Commander {
    /// Produces the next command to be executed by the flight control system.
    ///
    /// # Parameters
    ///
    /// * `dt` - The time elapsed since the last command was generated.
    ///   This allows for time-dependent command generation, such as
    ///   ramping up values gradually or handling timed sequences.
    ///
    /// # Returns
    ///
    /// A `Command` instance that can be either:
    /// * `Command::Manual` - Direct control inputs for manual flight
    /// * `Command::Waypoint` - Position and parameters for autonomous navigation
    fn next(&self, dt: Duration) -> Command;
}

/// Represents a flight control command to be executed by the flight system.
///
/// The `Command` enum encapsulates the different types of commands that can be issued
/// to control an aircraft. It supports two primary control paradigms:
///
/// * `Manual` - Low-level direct control inputs typically generated from user input devices
/// * `Waypoint` - High-level navigational commands used for autonomous flight
///
/// This separation allows the flight control system to handle both manual piloting
/// and automated navigation with a unified command interface.
///
/// # Examples
///
/// Creating a manual command:
/// ```
/// use stable_flight_sitl::commanders::{Command, AttitudeCommand};
///
/// let manual_cmd = Command::Attitude(AttitudeCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: -0.05,    // Slight nose down
///     yaw: 0.0,        // No yaw input
///     ..Default::default()
/// });
/// ```
///
/// Creating a waypoint command:
/// ```
/// use stable_flight_sitl::commanders::{Command, WaypointCommand};
/// use stable_flight_sitl::math::{Vec2, vec2};
///
/// let waypoint_cmd = Command::Waypoint(WaypointCommand {
///     position: vec2(1000.0, 2000.0),  // Target position in world coordinates
///     velocity: Some(vec2(15.0, 0.0)),  // Optional desired velocity at target
///     altitude: 500.0,                       // Target altitude in meters
///     cruise_speed: 25.0,                    // Desired speed during transit
///     roll: Some(0.05),                      // Optional roll angle in radians
/// });
/// ```
#[derive(Debug)]
pub enum Command {
    /// Direct manual control inputs for the aircraft.
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the control surfaces and power systems of the aircraft.
    Attitude(AttitudeCommand),

    /// Direct manual control inputs for the aircraft rotational rate.
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the rotational rate of the aircraft.
    Rate(RateCommand),

    /// Autonomous navigation command specifying a destination point.
    ///
    /// This command provides high-level navigational instructions that are translated
    /// into appropriate control surface and throttle settings by the flight controller.
    Waypoint(WaypointCommand),
}

/// Represents direct manual control inputs for an aircraft.
///
/// These values typically correspond to pilot inputs from control devices
/// like joysticks, yokes, or keyboard/mouse combinations.
///
/// # Value Ranges
///
/// * `throttle`: 0.0 (idle) to 1.0 (full power)
/// * `roll`: -1.0 (left bank) to 1.0 (right bank)
/// * `pitch`: -1.0 (nose down) to 1.0 (nose up)
/// * `yaw`: -1.0 (left turn) to 1.0 (right turn)
#[derive(Debug)]
pub struct AttitudeCommand {
    /// Controls the engine power output, ranging from 0.0 (idle) to 1.0 (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll/bank angle, ranging from -1.0 (left) to 1.0 (right).
    pub roll: f32,

    /// Controls the aircraft's pitch angle, ranging from -1.0 (down) to 1.0 (up).
    pub pitch: f32,

    /// Controls the aircraft's yaw rotation, ranging from -1.0 (left) to 1.0 (right).
    pub yaw: f32,

    /// Controls the arming state of the aircraft (0.0 for disarmed, 1.0 for armed).
    /// This can be used to enable/disable the motors or other safety features.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from 0 to 2.
    pub mode: u8,
}

impl Default for AttitudeCommand {
    fn default() -> Self {
        AttitudeCommand {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            arm: false,
            mode: 0,
        }
    }
}

/// Represents direct manual control inputs of rotational rate of aircraft.
///
/// These values typically correspond to pilot inputs from control devices
/// like joysticks, yokes, or keyboard/mouse combinations.
#[derive(Debug)]
pub struct RateCommand {
    /// Controls the engine power output, ranging from 0.0 (idle) to 1.0 (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll/bank angle, ranging from -1.0 (left) to 1.0 (right).
    pub roll_rate: f32,

    /// Controls the aircraft's pitch angle, ranging from -1.0 (down) to 1.0 (up).
    pub pitch_rate: f32,

    /// Controls the aircraft's yaw rotation, ranging from -1.0 (left) to 1.0 (right).
    pub yaw_rate: f32,

    /// Controls the arming state of the aircraft (0.0 for disarmed, 1.0 for armed).
    /// This can be used to enable/disable the motors or other safety features.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from 0 to 2.
    pub mode: u8,
}

impl Default for RateCommand {
    fn default() -> Self {
        RateCommand {
            throttle: 0.0,
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            arm: false,
            mode: 0,
        }
    }
}

/// Represents a navigational waypoint for autonomous flight.
///
/// A waypoint command instructs the aircraft to navigate to a specific position
/// at a given altitude, optionally with a specified velocity to be achieved at
/// the target position.
#[derive(Debug, Default)]
pub struct WaypointCommand {
    /// The target position in 2D space (X,Y coordinates).
    pub position: Vec2<f32>,

    /// Optional desired velocity vector at the waypoint.
    ///
    /// When provided, the aircraft will attempt to arrive at the position
    /// with this velocity. This is useful for creating smooth paths through
    /// multiple waypoints.
    pub velocity: Option<Vec2<f32>>,

    /// Target altitude in meters.
    pub altitude: f32,

    /// Desired cruise speed (in meters per second) while traveling to the waypoint.
    pub cruise_speed: f32,

    /// Optional desired roll angle in radians.
    ///
    /// When provided, the aircraft will attempt to maintain this roll angle
    /// while approaching the waypoint. This is useful for creating coordinated
    /// turns or acrobatic maneuvers.
    pub roll: Option<f32>,
}
