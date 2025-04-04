pub mod commanders;
pub mod controllers;
pub mod mixers;
pub mod orchestrator;
pub mod telemetry;
pub mod vehicles;

use std::time::Duration;

use log::info;
use uom::si::f32::*;

pub use orchestrator::FlightOrchestrator;
use robomath::{Quaternion, Vec2, Vec3};

// Commanders

/// Represents an entity that produces flight control commands.
///
/// The `Commander` trait is responsible for generating the next command to be executed
/// by a flight control system. It takes into account the elapsed time since the last
/// command (`dt`) and the current state of the vehicle (`current_state`) to produce
/// commands that can be time-dependent or state-dependent.
///
/// Implementors of this trait may include:
/// - **User input handlers**: Translate joystick/keyboard inputs into manual commands (e.g., `Command::Attitude` or `Command::Rate`).
/// - **Navigation systems**: Generate waypoint commands (e.g., `Command::Waypoint`) for autonomous flight to specific locations.
/// - **Mission controllers**: Produce commands based on predefined flight plans or mission objectives.
/// - **Formation controllers**: Maintain relative positioning between multiple aircraft in a formation.
///
/// # Examples
///
/// Implementing a simple `Commander` that always returns a constant `Attitude` command:
/// ```
/// use std::time::Duration;
/// use stable_flight_sitl::VehicleState;
/// use stable_flight_sitl::{Commander, Command, AttitudeCommand};
///
/// struct ConstantAttitudeCommander;
///
/// impl Commander for ConstantAttitudeCommander {
///     fn next(&mut self, dt: Duration, current_state: &VehicleState) -> Command {
///         Command::Attitude(AttitudeCommand {
///             throttle: 0.5,  // 50% throttle
///             roll: 0.0,      // No roll
///             pitch: 0.0,     // No pitch
///             yaw: 0.0,       // No yaw
///             arm: true,      // Armed
///             mode: 0,        // Default mode
///         })
///     }
/// }
/// ```
pub trait Commander {
    /// Produces the next command to be executed by the flight control system.
    ///
    /// # Parameters
    ///
    /// - `dt`: The time elapsed since the last command was generated, as a `Duration`.
    ///   This allows for time-dependent command generation, such as ramping up values
    ///   gradually or handling timed sequences.
    /// - `current_state`: The current state of the vehicle, as a `VehicleState`.
    ///   Typically includes information like position, velocity, attitude, and other
    ///   relevant data, allowing the commander to generate state-dependent commands.
    ///
    /// # Returns
    ///
    /// A `Command` instance that can be one of the following variants:
    /// - `Command::Attitude`: Direct manual control inputs for the aircraft's attitude (orientation).
    /// - `Command::Rate`: Direct manual control inputs for the aircraft's rotational rates.
    /// - `Command::Waypoint`: High-level navigational instructions for autonomous flight to a specific position.
    fn next(&mut self, dt: Duration, current_state: &VehicleState) -> Command;
}

/// Represents a flight control command to be executed by the flight system.
///
/// The `Command` enum encapsulates different types of commands that can be issued
/// to control an aircraft. It supports three types of commands, categorized into two
/// primary control paradigms:
///
/// * `Manual` - Low-level direct control inputs typically generated from user input devices.
///   This includes:
///   - `Attitude`: Controls the aircraft's orientation (roll, pitch, yaw).
///   - `Rate`: Controls the aircraft's rotational rates (rate of change of roll, pitch, yaw).
/// * `Waypoint` - High-level navigational commands used for autonomous flight.
///
/// This separation allows the flight control system to handle both manual piloting
/// and automated navigation with a unified command interface.
///
/// # Examples
///
/// Creating a manual attitude command:
/// ```
/// use stable_flight_sitl::{Command, AttitudeCommand};
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
/// Creating a manual rate command:
/// ```
/// use stable_flight_sitl::{Command, RateCommand};
///
/// let rate_cmd = Command::Rate(RateCommand {
///     throttle: 0.5,   // 50% throttle
///     roll_rate: 0.2,  // Moderate roll rate
///     pitch_rate: 0.1, // Moderate pitch rate
///     yaw_rate: 0.0,   // No yaw rate
///     ..Default::default()
/// });
/// ```
///
/// Creating a waypoint command:
/// ```
/// use robomath::{Vec2, vec2};
/// use stable_flight_sitl::{Command, WaypointCommand};
///
/// let waypoint_cmd = Command::Waypoint(WaypointCommand {
///     position: vec2(1000.0, 2000.0),  // Target position in world coordinates
///     velocity: Some(vec2(15.0, 0.0)), // Optional desired velocity at target
///     altitude: 500.0,                 // Target altitude in meters
///     cruise_speed: 25.0,              // Desired speed during transit
///     roll: Some(0.05),                // Optional roll angle in radians
/// });
/// ```
///
/// # Notes
/// - For `WaypointCommand`, optional fields like `velocity` and `roll` can be `None`. When `None`,
///   the flight controller may use default values or ignore the parameter, depending on its implementation.
#[derive(Debug)]
pub enum Command {
    /// Direct manual control inputs for the aircraft's attitude (orientation).
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the aircraft's roll, pitch, and yaw orientation.
    /// The ranges are conventional and may not be enforced by the system.
    Attitude(AttitudeCommand),

    /// Direct manual control inputs for the aircraft's rotational rate.
    ///
    /// These values typically range from -1.0 to 1.0 (or 0.0 to 1.0 for throttle)
    /// and directly influence the rate of change of roll, pitch, and yaw.
    /// The ranges are conventional and may not be enforced by the system.
    Rate(RateCommand),

    /// Autonomous navigation command specifying a destination point.
    ///
    /// This command provides high-level navigational instructions that are translated
    /// into appropriate control surface and throttle settings by the flight controller.
    Waypoint(WaypointCommand),
}

/// Represents direct manual control of an aircraft's attitude (orientation).
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's orientation
/// (roll, pitch, yaw) and throttle. It is typically used within a `Command::Attitude`
/// variant to issue manual flight commands to the flight system.
///
/// # Value Ranges
///
/// - `throttle`: `0.0` (idle) to `1.0` (full power)
/// - `roll`: `-1.0` (full left bank) to `1.0` (full right bank)
/// - `pitch`: `-1.0` (full nose down) to `1.0` (full nose up)
/// - `yaw`: `-1.0` (full left turn) to `1.0` (full right turn)
/// - `arm`: `false` (disarmed) or `true` (armed)
/// - `mode`: `0` to `2` (specific meaning depends on the flight system)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating an `AttitudeCommand` for a slight climb with a right bank:
/// ```
/// use stable_flight_sitl::AttitudeCommand;
///
/// let cmd = AttitudeCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: 0.05,     // Slight nose up
///     yaw: 0.0,        // No yaw input
///     arm: true,       // Arm the aircraft
///     mode: 1,         // Example mode (e.g., stabilized)
/// };
/// ```
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `AttitudeCommand`, providing the following
/// default values:
/// - `throttle`: `0.0`
/// - `roll`: `0.0`
/// - `pitch`: `0.0`
/// - `yaw`: `0.0`
/// - `arm`: `false`
/// - `mode`: `0`
#[derive(Debug)]
pub struct AttitudeCommand {
    /// Controls the engine power output, ranging from `0.0` (idle) to `1.0` (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll/bank angle, ranging from `-1.0` (full left bank)
    /// to `1.0` (full right bank).
    pub roll: f32,

    /// Controls the aircraft's pitch angle, ranging from `-1.0` (full nose down)
    /// to `1.0` (full nose up).
    pub pitch: f32,

    /// Controls the aircraft's yaw rotation, ranging from `-1.0` (full left turn)
    /// to `1.0` (full right turn).
    pub yaw: f32,

    /// Controls the arming state of the aircraft (`false` for disarmed, `true` for armed).
    /// Arming typically enables motors or other safety-critical systems.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from `0` to `2`.
    /// The specific meaning of each value depends on the flight system implementation
    /// (e.g., `0` might be manual mode, `1` stabilized, `2` autonomous).
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

/// Represents direct manual control of an aircraft's rotational rates.
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's rate of rotation
/// (roll rate, pitch rate, yaw rate) and throttle. It is typically used within a
/// `Command::Rate` variant to issue manual flight commands to the flight system.
///
/// The "rate" refers to the speed of rotation around each axis, typically measured
/// in a normalized range where `-1.0` and `1.0` represent the maximum rates supported
/// by the flight system (e.g., radians per second, scaled to system limits).
///
/// # Value Ranges
///
/// - `throttle`: `0.0` (idle) to `1.0` (full power)
/// - `roll_rate`: `-1.0` (maximum left roll rate) to `1.0` (maximum right roll rate)
/// - `pitch_rate`: `-1.0` (maximum nose-down rate) to `1.0` (maximum nose-up rate)
/// - `yaw_rate`: `-1.0` (maximum left yaw rate) to `1.0` (maximum right yaw rate)
/// - `arm`: `false` (disarmed) or `true` (armed)
/// - `mode`: `0` to `2` (specific meaning depends on the flight system)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `RateCommand` for a moderate right roll and slight nose-up rate:
/// ```
/// use stable_flight_sitl::RateCommand;
///
/// let cmd = RateCommand {
///     throttle: 0.6,    // 60% throttle
///     roll_rate: 0.3,   // Moderate right roll rate
///     pitch_rate: 0.1,  // Slight nose-up rate
///     yaw_rate: 0.0,    // No yaw rate
///     arm: true,        // Arm the aircraft
///     mode: 1,          // Example mode (e.g., stabilized)
/// };
/// ```
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `RateCommand`, providing the following
/// default values:
/// - `throttle`: `0.0`
/// - `roll_rate`: `0.0`
/// - `pitch_rate`: `0.0`
/// - `yaw_rate`: `0.0`
/// - `arm`: `false`
/// - `mode`: `0`
#[derive(Debug)]
pub struct RateCommand {
    /// Controls the engine power output, ranging from `0.0` (idle) to `1.0` (maximum).
    pub throttle: f32,

    /// Controls the aircraft's roll rate, ranging from `-1.0` (maximum left roll rate)
    /// to `1.0` (maximum right roll rate).
    pub roll_rate: f32,

    /// Controls the aircraft's pitch rate, ranging from `-1.0` (maximum nose-down rate)
    /// to `1.0` (maximum nose-up rate).
    pub pitch_rate: f32,

    /// Controls the aircraft's yaw rate, ranging from `-1.0` (maximum left yaw rate)
    /// to `1.0` (maximum right yaw rate).
    pub yaw_rate: f32,

    /// Controls the arming state of the aircraft (`false` for disarmed, `true` for armed).
    /// Arming typically enables motors or other safety-critical systems.
    pub arm: bool,

    /// Controls the aircraft's flight mode, ranging from `0` to `2`.
    /// The specific meaning of each value depends on the flight system implementation
    /// (e.g., `0` might be manual mode, `1` stabilized, `2` autonomous).
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
/// A `WaypointCommand` instructs the aircraft to navigate to a specific position
/// in 2D space at a given altitude, optionally with a specified velocity to be achieved
/// at the target position and a desired roll angle during the approach. This struct is
/// typically used within a `Command::Waypoint` variant to issue autonomous navigation
/// commands to the flight system.
///
/// # Value Ranges
///
/// - `position`: X, Y coordinates in 2D space (units and coordinate system depend on the flight system, typically meters in world coordinates)
/// - `velocity`: Optional velocity vector in meters per second (no specific range; depends on system limits)
/// - `altitude`: Target altitude in meters (typically non-negative; negative values may be interpreted as ground level or cause undefined behavior)
/// - `cruise_speed`: Desired speed in meters per second (typically positive; zero or negative values may cause the aircraft to stop or behave unpredictably)
/// - `roll`: Optional roll angle in radians (typically between `-π` and `π`; depends on system limits)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `WaypointCommand` to navigate to a position with a specific altitude and speed:
/// ```
/// use robomath::{Vec2, vec2};
/// use stable_flight_sitl::WaypointCommand;
///
/// let cmd = WaypointCommand {
///     position: vec2(1000.0, 2000.0),  // Target position (X=1000m, Y=2000m)
///     velocity: Some(vec2(15.0, 0.0)), // Arrive with 15 m/s eastward velocity
///     altitude: 500.0,                 // Target altitude of 500 meters
///     cruise_speed: 25.0,              // Travel at 25 m/s
///     roll: Some(0.05),                // Slight right roll during approach
/// };
/// ```
///
/// # Behavior of Optional Fields
///
/// - If `velocity` is `None`, the flight system may choose a default velocity or ignore velocity constraints.
/// - If `roll` is `None`, the flight system typically maintains a default roll (e.g., zero or coordinated turn behavior).
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `WaypointCommand`, providing the following
/// default values:
/// - `position`: `Vec2 { x: 0.0, y: 0.0 }`
/// - `velocity`: `None`
/// - `altitude`: `0.0`
/// - `cruise_speed`: `0.0`
/// - `roll`: `None`
#[derive(Debug, Default)]
pub struct WaypointCommand {
    /// The target position in 2D space (X, Y coordinates).
    ///
    /// The coordinate system and units depend on the flight system implementation
    /// (typically meters in world coordinates).
    pub position: Vec2<f32>,

    /// Optional desired velocity vector at the waypoint, in meters per second.
    ///
    /// When provided, the aircraft will attempt to arrive at the position with this
    /// velocity. This is useful for creating smooth paths through multiple waypoints.
    /// If `None`, the flight system may choose a default velocity or ignore velocity constraints.
    pub velocity: Option<Vec2<f32>>,

    /// Target altitude in meters.
    ///
    /// Typically non-negative; negative values may be interpreted as ground level
    /// or cause undefined behavior depending on the flight system.
    pub altitude: f32,

    /// Desired cruise speed in meters per second while traveling to the waypoint.
    ///
    /// Typically positive; zero or negative values may cause the aircraft to stop
    /// or behave unpredictably depending on the flight system.
    pub cruise_speed: f32,

    /// Optional desired roll angle in radians.
    ///
    /// When provided, the aircraft will attempt to maintain this roll angle while
    /// approaching the waypoint. This is useful for creating coordinated turns or
    /// acrobatic maneuvers. If `None`, the flight system typically maintains a default
    /// roll (e.g., zero or coordinated turn behavior). Values typically range between
    /// `-π` and `π`, depending on system limits.
    pub roll: Option<f32>,
}

#[derive(Debug, Default, PartialEq)]
pub struct ControlOutput {
    pub thrust: f32,
    pub moments: Vec3<f32>,
}

impl ControlOutput {
    pub fn new(thrust: f32, moments: Vec3<f32>) -> ControlOutput {
        ControlOutput { thrust, moments }
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleState {
    pub position: Vec3<Length>,
    pub velocity: Vec3<Velocity>,
    pub acceleration: Vec3<Acceleration>,
    pub attitude: Quaternion,
    pub angular_velocity: Vec3<f32>,
    pub roll: f32,
    pub pitch: f32,
    pub heading: f32,
    // flight mode
    // heading
    // gps
}

pub trait Vehicle {
    fn estimate_state(&self, dt: Duration) -> VehicleState;
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

pub trait Controller {
    fn control(&mut self, dt: Duration, command: &Command, state: &VehicleState) -> ControlOutput;
}

pub trait TelemetryPublisher {
    fn send_telemetry(&mut self, dt: Duration, command: &Command, telemetry: &VehicleState);
}

pub struct ConsoleTelemetryPublisher {
    // sample_rate: f64,
}

impl TelemetryPublisher for ConsoleTelemetryPublisher {
    fn send_telemetry(&mut self, _dt: Duration, _command: &Command, telemetry: &VehicleState) {
        info!("Sending telemetry: {:?}", telemetry);
    }
}

#[derive(Debug)]
pub struct ActuatorCommand {
    pub motors: [f32; 8],
    pub rudder: f32,
    pub elevator: f32,
    pub aileron: f32,
    pub throttle: f32,
    pub elevon_left: f32,
    pub elevon_right: f32,
}

impl Default for ActuatorCommand {
    fn default() -> Self {
        ActuatorCommand {
            motors: [0.0; 8],
            rudder: 0.0,
            elevator: 0.0,
            aileron: 0.0,
            throttle: 0.0,
            elevon_left: 0.0,
            elevon_right: 0.0,
        }
    }
}

// == Mixers ==

pub trait Mixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}
