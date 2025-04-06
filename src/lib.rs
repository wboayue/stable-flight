//! [![github]](https://github.com/wboayue/stable-flight-sitl)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-sitl)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! This library implements a Rust-powered flight controller in simulation.
//! Simulation allows safe refining of the flight control logic before real-world deployment.
//! The flight controller is designed to be easily extendable, robust, and approachable leveraging [Rust's advantages](https://wboayue.com/posts/flight-controller-why-rust/) in systems programming.
//! See the following document for an [architecture overview](https://github.com/wboayue/stable-flight-sitl/blob/main/architecture.md).
//!
//! Checkout [README](https://github.com/wboayue/stable-flight-sitl/blob/main/README.md) to get started.
//!
//! [![Watch the demo](https://img.youtube.com/vi/KsTX3pH_htg/0.jpg)](https://youtu.be/KsTX3pH_htg)
//!

pub mod commanders;
pub mod mixers;
pub mod orchestrator;
pub mod simulators;
pub mod stabilizers;
pub mod telemetry;

use std::time::Duration;

use uom::si::f32::*;

#[doc(inline)]
pub use orchestrator::FlightOrchestrator;

use robomath::{Quaternion, Vec2, Vec3};

// Commanders

/// Represents an entity that produces flight control commands.
///
/// The `Commander` trait is responsible for producing the next flight control command given the elapsed time (dt) and the current vehicle state (current_state).
/// This allows for time-dependent and state-dependent command generation, such as ramping values, coordinating timed sequences, or adjusting commands based on the aircraft’s position or velocity.
///
/// Implementors of this trait may include:
/// - **User input handlers**: Translate joystick/keyboard inputs into manual commands (e.g., [Command::Attitude] or [Command::Rate]).
/// - **Navigation systems**: Generate waypoint commands (e.g., [Command::Waypoint]) for autonomous flight to specific locations.
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

/// Represents the high-level control outputs from the stabilization system.
///
/// This struct encapsulates the desired thrust and moments to be applied to the
/// vehicle, which are later translated to specific actuator commands by a `Mixer`.
///
/// # Examples
///
/// Creating a control output for a simple hover command:
/// ```
/// use stable_flight_sitl::ControlOutput;
/// use robomath::{Vec3, vec3};
///
/// let output = ControlOutput::new(
///     0.5, // 50% thrust
///     vec3(0.0, 0.0, 0.0) // No roll, pitch, or yaw moments
/// );
/// ```
#[derive(Debug, Default, PartialEq)]
pub struct ControlOutput {
    /// Normalized thrust value, typically ranging from 0.0 (no thrust) to 1.0 (maximum thrust).
    pub thrust: f32,

    /// Control moments in roll (x), pitch (y), and yaw (z) axes.
    /// Values are typically normalized between -1.0 and 1.0, where the magnitude
    /// represents the proportion of maximum control authority in each axis.
    pub moments: Vec3<f32>,
}

impl ControlOutput {
    /// Creates a new `ControlOutput` with the specified thrust and control moments.
    ///
    /// # Parameters
    ///
    /// - `thrust`: Normalized thrust value, typically between 0.0 and 1.0.
    /// - `moments`: Control moments as a 3D vector (roll, pitch, yaw).
    ///
    /// # Returns
    ///
    /// A new `ControlOutput` instance with the specified parameters.
    pub fn new(thrust: f32, moments: Vec3<f32>) -> ControlOutput {
        ControlOutput { thrust, moments }
    }
}

/// Represents the physical state of a vehicle in 3D space.
///
/// This struct encapsulates all the key state information needed for flight control,
/// including position, velocity, acceleration, attitude, and angular velocity.
/// It uses strongly-typed physical units (from the `uom` crate) for position, velocity,
/// and acceleration to ensure dimensional consistency in calculations.
///
/// # Examples
///
/// Creating a default vehicle state:
/// ```
/// use stable_flight_sitl::VehicleState;
///
/// let state = VehicleState::default();
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct VehicleState {
    /// 3D position vector with physical length units.
    pub position: Vec3<Length>,

    /// 3D velocity vector with physical velocity units.
    pub velocity: Vec3<Velocity>,

    /// 3D acceleration vector with physical acceleration units.
    pub acceleration: Vec3<Acceleration>,

    /// Orientation quaternion representing the vehicle's attitude.
    /// The quaternion represents the rotation from world frame to body frame.
    pub attitude: Quaternion,

    /// Angular velocity vector in roll (x), pitch (y), and yaw (z) axes.
    /// Typically measured in radians per second.
    pub angular_velocity: Vec3<f32>,

    /// Current roll angle in radians.
    pub roll: f32,

    /// Current pitch angle in radians.
    pub pitch: f32,

    /// Current heading (yaw) angle in radians.
    /// Typically 0 represents North, with positive values rotating clockwise.
    pub heading: f32,
    // Note: Additional fields for flight mode, GPS data, etc. may be added in the future.
}

/// Represents a physical vehicle that can be controlled by the flight system.
///
/// The `Vehicle` trait defines the interface for interacting with a physical aircraft,
/// whether it's a real vehicle or a simulated one. It handles two primary responsibilities:
/// - Estimating the current state of the vehicle based on sensor data
/// - Actuating the vehicle's control surfaces and motors based on control commands
///
/// Implementors of this trait may include:
/// - **Simulation vehicles**: Interface with flight simulators like X-Plane, RealFlight, or custom physics engines
/// - **Hardware interfaces**: Connect to real drones, fixed-wing aircraft, or other flying vehicles
/// - **Test vehicles**: Provide predictable responses for testing control algorithms without a full simulation
pub trait Vehicle {
    /// Estimates the current vehicle state based on sensor inputs and time elapsed.
    ///
    /// # Parameters
    /// - `dt`: Time elapsed since the last state estimation.
    ///
    /// # Returns
    /// A `VehicleState` representing the current estimated state of the vehicle.
    fn estimate_state(&self, dt: Duration) -> VehicleState;

    /// Sends actuation commands to the vehicle's control surfaces and/or motors.
    ///
    /// # Parameters
    /// - `actuator_command`: The command values to send to the vehicle's actuators.
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

/// Component responsible for estimating the vehicle's physical state.
///
/// The `StateEstimator` trait defines the interface for components that analyze
/// sensor data and other inputs to estimate the current state of the vehicle,
/// including position, velocity, attitude, and other relevant parameters.
///
/// Implementors of this trait may include:
/// - **Sensor fusion algorithms**: Combine data from multiple sensors (IMU, GPS, etc.)
/// - **Kalman filters**: Estimate state with statistical techniques
/// - **Simulation interfaces**: Get state directly from physics engines
pub trait StateEstimator {
    /// Estimates the current vehicle state based on sensor inputs and time elapsed.
    ///
    /// # Parameters
    /// - `dt`: Time elapsed since the last state estimation.
    ///
    /// # Returns
    /// A `VehicleState` representing the current estimated state of the vehicle.
    fn estimate_state(&self, dt: Duration) -> VehicleState;
}

/// Component responsible for controlling the vehicle's actuators.
///
/// The `Actuator` trait defines the interface for components that translate
/// actuator commands into actual physical actions, such as moving control surfaces
/// or adjusting motor speeds.
///
/// Implementors of this trait may include:
/// - **Motor controllers**: Control electric motors for thrust
/// - **Servo drivers**: Position control surfaces like elevators and ailerons
/// - **Simulation interfaces**: Send control commands to physics engines
pub trait Actuator {
    /// Actuates the vehicle's control surfaces and/or motors based on the command.
    ///
    /// # Parameters
    ///
    /// - `actuator_command`: The command values to apply to the actuators.
    fn actuate(&mut self, actuator_command: &ActuatorCommand);
}

/// Component responsible for flight control and stabilization.
///
/// The `Stabilizer` trait defines the interface for components that implement
/// control algorithms to maintain desired flight dynamics based on commands and
/// current vehicle state. These algorithms typically include PID controllers or
/// more advanced control systems to translate high-level commands into specific
/// control outputs.
///
/// Implementors of this trait may include:
/// - **PID controllers**: Simple feedback control for attitude stability
/// - **LQR controllers**: Optimal control for complex flight dynamics
/// - **Model predictive controllers**: Advanced control with future prediction
/// - **Cascaded control systems**: Hierarchical controllers for different aspects of flight
pub trait Stabilizer {
    /// Generates control outputs based on the current command and vehicle state.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last control step.
    /// - `command`: The current command to be executed.
    /// - `state`: The current estimated state of the vehicle.
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the desired thrust and control moments.
    fn stabilize(&mut self, dt: Duration, command: &Command, state: &VehicleState)
        -> ControlOutput;
}

/// Component responsible for publishing telemetry data from the flight system.
///
/// The `TelemetryPublisher` trait defines the interface for components that send
/// flight data to external systems for logging, monitoring, or visualization.
/// This can include sending data to ground control stations, logging systems,
/// or other monitoring tools.
///
/// Implementors of this trait may include:
/// - **Mavlink publishers**: Send telemetry via the MAVLink protocol
/// - **Data loggers**: Record flight data for later analysis
/// - **Visualization interfaces**: Send data to real-time flight displays
pub trait TelemetryPublisher {
    /// Sends telemetry data to external systems.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last telemetry update.
    /// - `command`: The current command being executed.
    /// - `telemetry`: The current vehicle state to be published.
    fn send_telemetry(&mut self, dt: Duration, command: &Command, telemetry: &VehicleState);
}

/// Represents commands sent to the vehicle's actuators.
///
/// This struct encapsulates all possible actuation commands for various vehicle types,
/// including multirotor motors, fixed-wing control surfaces, and hybrid configurations.
/// Not all fields will be used for every vehicle type; the specific mapping is determined
/// by the `Mixer` implementation.
///
/// # Examples
///
/// Creating actuator commands for a quadcopter:
/// ```
/// use stable_flight_sitl::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.motors = [0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0]; // Four motors at 50% power
/// ```
///
/// Creating actuator commands for a fixed-wing aircraft:
/// ```
/// use stable_flight_sitl::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.throttle = 0.7;    // 70% throttle
/// cmd.elevator = 0.1;    // Slight up elevator
/// cmd.aileron = -0.05;   // Slight left roll
/// cmd.rudder = 0.0;      // No rudder
/// ```
#[derive(Debug)]
pub struct ActuatorCommand {
    /// Motor output values, ranging from 0.0 (off) to 1.0 (full power).
    /// For multirotors, the specific mapping of array indices to physical motors
    /// depends on the vehicle configuration and mixer.
    pub motors: [f32; 8],

    /// Rudder control value, ranging from -1.0 (full left) to 1.0 (full right).
    /// Used primarily for fixed-wing aircraft.
    pub rudder: f32,

    /// Elevator control value, ranging from -1.0 (full down) to 1.0 (full up).
    /// Used primarily for fixed-wing aircraft.
    pub elevator: f32,

    /// Aileron control value, ranging from -1.0 (roll left) to 1.0 (roll right).
    /// Used primarily for fixed-wing aircraft.
    pub aileron: f32,

    /// Throttle control value, ranging from 0.0 (idle) to 1.0 (full power).
    /// Used primarily for fixed-wing aircraft with a single motor/engine.
    pub throttle: f32,

    /// Left elevon control value, ranging from -1.0 to 1.0.
    /// Used for flying wing or delta wing configurations.
    pub elevon_left: f32,

    /// Right elevon control value, ranging from -1.0 to 1.0.
    /// Used for flying wing or delta wing configurations.
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

/// Component responsible for converting control outputs to actuator commands.
///
/// The `Mixer` trait defines the interface for components that translate
/// generalized control outputs (thrust and moments) into specific actuator
/// commands appropriate for the vehicle configuration. It maps abstract control
/// concepts to physical actuator movements.
///
/// Implementors of this trait may include:
/// - **Quadcopter mixers**: Map controls to four motors in different configurations (X, +)
/// - **Hexacopter mixers**: Map controls to six motors
/// - **Fixed-wing mixers**: Map controls to throttle, ailerons, elevator, and rudder
pub trait Mixer {
    /// Transforms abstract control outputs into specific actuator commands.
    ///
    /// This method is the core functionality of the `Mixer` trait, providing the critical
    /// translation layer between the stabilization system's desired control effects and
    /// the actual physical actuator settings needed to achieve those effects.
    ///
    /// # Parameters
    ///
    /// * `controls` - Reference to a [`ControlOutput`] struct containing normalized thrust
    ///   and moment values that represent the desired control effect.
    ///
    /// # Returns
    ///
    /// An [`ActuatorCommand`] containing specific actuator settings appropriate for the
    /// vehicle's physical configuration. Depending on the vehicle type, different fields
    /// in the returned struct will be populated:
    ///
    /// * For multicopters: Primarily the `motors` array
    /// * For fixed-wing: `throttle`, `aileron`, `elevator`, and `rudder`
    /// * For hybrid vehicles: Combination of the above
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight_sitl::{ControlOutput, Mixer};
    /// use stable_flight_sitl::mixers::QuadCopterMixer;
    /// use robomath::{Vec3, vec3};
    ///
    /// let mixer = QuadCopterMixer::default();
    ///
    /// // Create control output with 70% thrust and slight roll right
    /// let control_output = ControlOutput::new(
    ///     0.7,                    // 70% thrust
    ///     vec3(0.1, 0.0, 0.0)     // slight roll right, no pitch, no yaw
    /// );
    ///
    /// // Convert abstract controls into specific motor commands
    /// let actuator_command = mixer.mix(&control_output);
    ///
    /// // Now actuator_command contains specific motor values for a quadcopter
    /// ```
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}
