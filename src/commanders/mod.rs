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
//! use flight_control::{Command, ManualCommand};
//!
//! let manual_cmd = Command::Manual(ManualCommand {
//!     throttle: 0.75,  // 75% power
//!     roll: 0.1,       // Slight right bank
//!     pitch: -0.05,    // Slight nose down
//!     yaw: 0.0,        // No yaw input
//! });
//! ```
//!
//! Creating and using a waypoint command:
//!
//! ```rust
//! use flight_control::{Command, WaypointCommand};
//! use flight_control::math::Vec2;
//!
//! let waypoint_cmd = Command::Waypoint(WaypointCommand {
//!     position: Vec2::new(1000.0, 2000.0),  // Target position
//!     velocity: Some(Vec2::new(15.0, 0.0)),  // Optional target velocity
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
/// use flight_control::{Command, ManualCommand};
///
/// let manual_cmd = Command::Manual(ManualCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: -0.05,    // Slight nose down
///     yaw: 0.0,        // No yaw input
/// });
/// ```
///
/// Creating a waypoint command:
/// ```
/// use flight_control::{Command, WaypointCommand};
/// use flight_control::math::Vec2;
///
/// let waypoint_cmd = Command::Waypoint(WaypointCommand {
///     position: Vec2::new(1000.0, 2000.0),  // Target position in world coordinates
///     velocity: Some(Vec2::new(15.0, 0.0)),  // Optional desired velocity at target
///     altitude: 500.0,                       // Target altitude in meters
///     cruise_speed: 25.0,                    // Desired speed during transit
/// });
/// ```
#[derive(Debug)]
pub enum Command {
    /// Direct manual control inputs for the aircraft.
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the control surfaces and power systems of the aircraft.
    Manual(ManualCommand),

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
pub struct ManualCommand {
    /// Controls the engine power output, ranging from 0.0 (idle) to 1.0 (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll/bank angle, ranging from -1.0 (left) to 1.0 (right).
    pub roll: f32,

    /// Controls the aircraft's pitch angle, ranging from -1.0 (down) to 1.0 (up).
    pub pitch: f32,

    /// Controls the aircraft's yaw rotation, ranging from -1.0 (left) to 1.0 (right).
    pub yaw: f32,
}

/// Represents a navigational waypoint for autonomous flight.
///
/// A waypoint command instructs the aircraft to navigate to a specific position
/// at a given altitude, optionally with a specified velocity to be achieved at
/// the target position.
#[derive(Debug)]
pub struct WaypointCommand {
    /// The target position in 2D space (X,Y coordinates).
    pub position: Vec2,

    /// Optional desired velocity vector at the waypoint.
    ///
    /// When provided, the aircraft will attempt to arrive at the position
    /// with this velocity. This is useful for creating smooth paths through
    /// multiple waypoints.
    pub velocity: Option<Vec2>,

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
