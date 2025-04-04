//! # Flight Control Commands Module
//!
//! This module defines the core types and traits for generating and representing flight control
//! commands. It provides a unified interface that supports both manual (attitude and rate-based)
//! and autonomous (waypoint-based) flight control, facilitating flexible integration into a broader
//! flight control system.
//!
//! ## Overview
//!
//! The module consists of the following key components:
//!
//! - **[`Commander`]**: A trait for generating flight control commands based on elapsed time and vehicle state.
//! - **[`Command`]**: An enum that encapsulates different command types (`Attitude`, `Rate`, `Waypoint`).
//! - **[`AttitudeCommand`]**: A struct representing direct control inputs for the aircraft's attitude (orientation).
//! - **[`RateCommand`]**: A struct representing direct control inputs for the aircraft's rotational rates.
//! - **[`WaypointCommand`]**: A struct representing navigational commands, including target position,
//!   optional velocity, altitude, cruise speed, and an optional roll angle for coordinated maneuvers.
//!
//! ## Detailed Description
//!
//! ### Commander Trait
//!
//! The [`Commander`] trait is responsible for producing the next flight control command given the elapsed
//! time (`dt`) and the current vehicle state (`current_state`). This allows for time-dependent and
//! state-dependent command generation, such as ramping values, coordinating timed sequences, or adjusting
//! commands based on the aircraft's position or velocity.
//!
//! Implementations of [`Commander`] can include:
//! - **User Input Handlers**: Translate joystick or keyboard inputs into direct commands (e.g., [`Command::Attitude`] or [`Command::Rate`]).
//! - **Navigation Systems**: Generate waypoint commands (e.g., [`Command::Waypoint`]) for autonomous flight.
//! - **Mission Controllers**: Produce commands based on a predefined flight plan.
//! - **Formation Controllers**: Maintain relative positioning among multiple aircraft in a formation.
//!
//! ### Command Enum
//!
//! The [`Command`] enum encapsulates the various control paradigms available:
//!
//! - **[`Command::Attitude`]**: Represents direct manual control inputs for the aircraft's attitude (orientation), such as roll, pitch, and yaw angles.
//! - **[`Command::Rate`]**: Represents direct manual control inputs for the aircraft's rotational rates, such as the rate of change of roll, pitch, and yaw.
//! - **[`Command::Waypoint`]**: Represents high-level navigational instructions for autonomous flight, including target position, altitude, and optional velocity.
//!
//! This design provides a unified interface to handle both manual and autonomous flight modes.
//!
//! ### AttitudeCommand Struct
//!
//! The [`AttitudeCommand`] struct encapsulates direct control inputs for the aircraft's attitude, typically derived
//! from pilot or user inputs. It controls the aircraft's orientation (roll, pitch, yaw) and throttle.
//!
//! **Conventional Value Ranges:**
//! - `throttle`: `0.0` (idle) to `1.0` (full power)
//! - `roll`: `-1.0` (full left bank) to `1.0` (full right bank)
//! - `pitch`: `-1.0` (full nose down) to `1.0` (full nose up)
//! - `yaw`: `-1.0` (full left turn) to `1.0` (full right turn)
//! - `arm`: `false` (disarmed) or `true` (armed)
//! - `mode`: `0` to `2` (specific meaning depends on the flight system)
//!
//! ### RateCommand Struct
//!
//! The [`RateCommand`] struct encapsulates direct control inputs for the aircraft's rotational rates, typically
//! derived from pilot or user inputs. It controls the rate of rotation (roll rate, pitch rate, yaw rate) and throttle.
//!
//! **Conventional Value Ranges:**
//! - `throttle`: `0.0` (idle) to `1.0` (full power)
//! - `roll_rate`: `-1.0` (maximum left roll rate) to `1.0` (maximum right roll rate)
//! - `pitch_rate`: `-1.0` (maximum nose-down rate) to `1.0` (maximum nose-up rate)
//! - `yaw_rate`: `-1.0` (maximum left yaw rate) to `1.0` (maximum right yaw rate)
//! - `arm`: `false` (disarmed) or `true` (armed)
//! - `mode`: `0` to `2` (specific meaning depends on the flight system)
//!
//! ### WaypointCommand Struct
//!
//! The [`WaypointCommand`] struct represents an autonomous navigation command.
//!
//! It specifies:
//! - A target position using a 2D vector ([`Vec2`]).
//! - An optional desired velocity vector at the waypoint for smoother transitions.
//! - The target altitude in meters.
//! - The desired cruise speed during transit (in meters per second).
//! - An optional desired roll angle (in radians) for coordinated turns or maneuvers.
//!
//! ## Usage Example
//!
//! Creating and using an attitude command:
//!
//! ```rust
//! use stable_flight_sitl::{Command, AttitudeCommand};
//!
//! let attitude_cmd = Command::Attitude(AttitudeCommand {
//!     throttle: 0.75,  // 75% power
//!     roll: 0.1,       // Slight right bank
//!     pitch: -0.05,    // Slight nose down
//!     yaw: 0.0,        // No yaw input
//!     arm: true,       // Armed
//!     mode: 0,         // Default mode
//! });
//! ```
//!
//! Creating and using a rate command:
//!
//! ```rust
//! use stable_flight_sitl::{Command, RateCommand};
//!
//! let rate_cmd = Command::Rate(RateCommand {
//!     throttle: 0.6,    // 60% throttle
//!     roll_rate: 0.3,   // Moderate right roll rate
//!     pitch_rate: 0.1,  // Slight nose-up rate
//!     yaw_rate: 0.0,    // No yaw rate
//!     arm: true,        // Armed
//!     mode: 1,          // Example mode (e.g., stabilized)
//! });
//! ```
//!
//! Creating and using a waypoint command:
//!
//! ```rust
//! use robomath::{Vec2, vec2};
//! use stable_flight_sitl::{Command, WaypointCommand};
//!
//! let waypoint_cmd = Command::Waypoint(WaypointCommand {
//!     position: vec2(1000.0, 2000.0),  // Target position
//!     velocity: Some(vec2(15.0, 0.0)), // Optional target velocity
//!     altitude: 500.0,                 // Target altitude in meters
//!     cruise_speed: 25.0,              // Desired speed in m/s
//!     roll: Some(0.05),                // Optional roll angle in radians
//! });
//! ```
//!
//! ## Integration
//!
//! This module is intended to be a foundational component of a larger flight control system.
//! By separating command generation ([`Commander`] trait) from command representation ([`Command`] enum),
//! the design supports a flexible and extensible control architecture. This allows for easy integration
//! of different control paradigms and simplifies the addition of new command types (e.g., velocity-based
//! or formation-based commands) in the future.

pub mod extreme_3d;
pub mod script;
pub mod spektrum;

#[cfg(test)]
mod tests;
