//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! Flight-control core for the Stable Flight controller: the pipeline traits ([`Commander`],
//! [`StateSource`] + [`Actuator`] = [`Vehicle`], [`Guidance`], [`Stabilizer`], [`Mixer`],
//! [`TelemetryPublisher`]), the data passed between them, copter guidance, the cascaded-PID
//! stabilizers, the mixers, and [`FlightLoop`], which sequences one control tick.
//!
//! This crate is `no_std` and has no simulator, I/O, or timing dependencies, so the same code
//! runs in software-in-the-loop simulation
//! ([`stable-flight-sitl`](https://crates.io/crates/stable-flight-sitl)) and, later, on flight
//! hardware. See the
//! [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md).

#![cfg_attr(not(test), no_std)]

pub mod commanders;
pub mod composition;
pub mod controllers;
pub mod flight_loop;
pub mod guidance;
mod math;
pub mod mixers;
pub mod params;
pub mod stabilizers;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
pub mod timing;
pub mod tuning;
pub mod view;

use core::time::Duration;

#[doc(inline)]
pub use composition::ComposedVehicle;
#[doc(inline)]
pub use flight_loop::FlightLoop;
pub use view::View;

use nalgebra::UnitQuaternion;

pub use math::{euler_angles, wrap_angle};

/// Re-export of the [`nalgebra`] version this crate's public types use; depend on it through
/// here so the versions can't diverge.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight::nalgebra::UnitQuaternion;
///
/// let state = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(0.0, 0.1, 0.0),
///     ..Default::default()
/// };
/// assert!(state.attitude.angle() > 0.0);
/// ```
pub use nalgebra;

/// 2D vector used throughout the pipeline (waypoint positions, tilt, controller limits).
///
/// # Examples
///
/// ```
/// use stable_flight::Vec2;
///
/// let tilt = Vec2::new(0.3, -0.1);
/// assert_eq!(tilt.component_mul(&Vec2::new(2.0, 2.0)), Vec2::new(0.6, -0.2));
/// ```
pub type Vec2 = nalgebra::Vector2<f32>;

/// 3D vector used throughout the pipeline (state vectors, moments, rates). Element-wise
/// products are `component_mul`; `*` between two vectors is a matrix product.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec3;
///
/// let rates = Vec3::new(3.0, 4.0, 0.0);
/// assert_eq!(rates.norm(), 5.0);
/// ```
pub type Vec3 = nalgebra::Vector3<f32>;

/// Standard gravity, in m/s².
pub const GRAVITY: f32 = 9.806_65;

// Commanders

/// Represents an entity that produces flight control commands.
///
/// The `Commander` trait is responsible for producing the next flight control command given the elapsed time (dt) and the current vehicle state (current_state).
/// This allows for time-dependent and state-dependent command generation, such as ramping values, coordinating timed sequences, or adjusting commands based on the aircraft’s position or velocity.
///
/// Implementors of this trait may include:
/// - **User input handlers**: Translate joystick/keyboard inputs into manual commands (e.g., [ControlInput::Attitude] or [ControlInput::Rate]).
/// - **Navigation systems**: Generate waypoint commands (e.g., [ControlInput::Waypoint]) for autonomous flight to specific locations.
/// - **Mission controllers**: Produce commands based on predefined flight plans or mission objectives.
/// - **Formation controllers**: Maintain relative positioning between multiple aircraft in a formation.
///
/// # Examples
///
/// Implementing a simple `Commander` that always returns a constant `Attitude` command:
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight::{AttitudeCommand, Command, Commander, ControlInput};
///
/// struct ConstantAttitudeCommander;
///
/// impl Commander for ConstantAttitudeCommander {
///     fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command {
///         Command::armed(ControlInput::Attitude(AttitudeCommand {
///             throttle: 0.5,  // 50% throttle
///             roll: 0.0,      // No roll
///             pitch: 0.0,     // No pitch
///             yaw: 0.0,       // No yaw
///         }))
///     }
/// }
/// ```
pub trait Commander {
    /// Produces the next command to be executed by the flight control system.
    ///
    /// # Parameters
    ///
    /// - `dt`: The time elapsed since the last command was generated in seconds.
    ///   This allows for time-dependent command generation, such as ramping up values
    ///   gradually or handling timed sequences.
    /// - `current_state`: The current state of the vehicle, as a `VehicleState`.
    ///   Typically includes information like position, velocity, attitude, and other
    ///   relevant data, allowing the commander to generate state-dependent commands.
    ///
    /// # Returns
    ///
    /// A [`Command`]: whether the vehicle should be armed, and the [`ControlInput`] to fly.
    /// `arm` is a level, not an event: return the desired state every tick.
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command;

    /// Which commander a failsafe let fly on the last [`next_command`](Self::next_command)
    /// call, for a commander that has one
    /// ([`FailsafeCommander`](crate::commanders::FailsafeCommander)). [`FlightLoop`] puts it in
    /// the tick's [`TelemetryRecord::failsafe`]. The default reports none, so a commander that
    /// wraps another must forward this to report the inner one's.
    fn failsafe(&self) -> Option<FailsafeState> {
        None
    }
}

/// Which commander a [`FailsafeCommander`](crate::commanders::FailsafeCommander) let fly: its
/// link-fed primary, or the fallback after the link went stale.
///
/// # Examples
///
/// ```
/// use stable_flight::FailsafeState;
///
/// let link_ok = |state: Option<FailsafeState>| state != Some(FailsafeState::Fallback);
/// assert!(link_ok(Some(FailsafeState::Primary)));
/// assert!(link_ok(Some(FailsafeState::AwaitingSticks))); // heard, not yet in control
/// assert!(link_ok(None)); // no failsafe: nothing to report
/// assert!(!link_ok(Some(FailsafeState::Fallback)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum FailsafeState {
    /// The primary (link-fed) commander flies.
    Primary,
    /// The primary went stale, or hasn't been heard yet; the fallback flies.
    Fallback,
    /// The primary is fresh again, but flies sticks over an armed vehicle: the fallback flies
    /// until a stick moves, so a forgotten joystick can't take over.
    AwaitingSticks,
}

/// One tick's command from a [`Commander`]: whether the vehicle should be armed, which
/// [`FlightRegime`] to fly in, and what to fly.
///
/// `arm` is vehicle-level and applies to every [`ControlInput`]. It is a level, not an event:
/// a commander returns the desired state every tick (an edge-triggered source, such as a
/// MAVLink arm request, latches it in the commander). While `arm` is `false`,
/// [`FlightLoop`] holds the actuators at [`ActuatorCommand::default`] (motors off) and
/// doesn't run guidance or the stabilizer; on the tick it turns `true`, it re-primes them so
/// integrators start from zero.
///
/// `regime` is a level too: the regime the commander wants, every tick. Guidance owns the
/// regime actually flown and moves toward the request ([`FlightModeReport`]), so repeating a
/// request can't restart a transition. It is orthogonal to `input`: the input says what to
/// fly, the regime how.
///
/// # Examples
///
/// ```
/// use stable_flight::{AttitudeCommand, Command, ControlInput, FlightRegime};
///
/// let hover = Command::armed(ControlInput::Attitude(AttitudeCommand {
///     throttle: 0.5,
///     ..Default::default()
/// }));
/// assert!(hover.arm);
/// assert_eq!(hover.regime, FlightRegime::Copter);
///
/// let switch_off = false;
/// assert!(!Command::new(switch_off, hover.input).arm);
///
/// let cruise = hover.with_regime(FlightRegime::Plane);
/// assert_eq!(cruise.regime, FlightRegime::Plane);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Command {
    /// `true` to arm (motors may run), `false` to disarm (motors held off). Defaults to
    /// `false`.
    pub arm: bool,

    /// The regime to fly in. Defaults to [`FlightRegime::Copter`].
    pub regime: FlightRegime,

    /// What to fly while armed. Ignored while disarmed.
    pub input: ControlInput,
}

impl Command {
    /// A command with the given arm state, in [`FlightRegime::Copter`].
    pub fn new(arm: bool, input: ControlInput) -> Self {
        Command {
            arm,
            regime: FlightRegime::default(),
            input,
        }
    }

    /// An armed command: fly `input`.
    pub fn armed(input: ControlInput) -> Self {
        Command::new(true, input)
    }

    /// A disarmed command: motors held off; `input` is carried for telemetry only.
    pub fn disarmed(input: ControlInput) -> Self {
        Command::new(false, input)
    }

    /// This command, requesting `regime`.
    pub fn with_regime(self, regime: FlightRegime) -> Self {
        Command { regime, ..self }
    }
}

/// How the vehicle flies: hovering on its rotors, or wing-borne. Requested by a [`Command`];
/// what is actually flown, including transitions, is a [`FlightMode`].
///
/// A copter-only stack flies [`Copter`](FlightRegime::Copter) whatever is requested and
/// reports a `Plane` request as [`ModeRefusal::Unsupported`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum FlightRegime {
    /// Rotor-borne: hover and copter-style flight. Takeoff and landing are flown here.
    #[default]
    Copter,
    /// Wing-borne forward flight.
    Plane,
}

/// The regime guidance is actually flying, including the transitions between regimes. Owned
/// by [`Guidance`] and reported in a [`FlightModeReport`].
///
/// Not to be confused with [`ControlMode`], the kind of the commanded [`ControlInput`]: the
/// input says what to fly, the flight mode how it's being flown.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum FlightMode {
    /// Flying in [`FlightRegime::Copter`].
    #[default]
    Copter,
    /// Flying in [`FlightRegime::Plane`].
    Plane,
    /// Transitioning from copter to plane flight.
    ToPlane,
    /// Transitioning from plane to copter flight.
    ToCopter,
}

impl FlightMode {
    /// The regime this mode flies in or is heading to.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{FlightMode, FlightRegime};
    ///
    /// assert_eq!(FlightMode::ToPlane.target(), FlightRegime::Plane);
    /// assert_eq!(FlightMode::Copter.target(), FlightRegime::Copter);
    /// ```
    pub fn target(&self) -> FlightRegime {
        match self {
            FlightMode::Copter | FlightMode::ToCopter => FlightRegime::Copter,
            FlightMode::Plane | FlightMode::ToPlane => FlightRegime::Plane,
        }
    }
}

/// Why guidance isn't flying, or stopped moving toward, the requested [`FlightRegime`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum ModeRefusal {
    /// This guidance can't fly the requested regime (`Plane` on a copter, `Copter` on a
    /// plane).
    Unsupported,
    /// The vehicle is on the ground; transitions start only in the air.
    OnGround,
    /// Too low to start a transition safely.
    BelowMinAltitude,
    /// A transition didn't complete in time and was abandoned.
    TransitionTimeout,
    /// A transition rolled too far off its target and was abandoned or cut short.
    RollError,
}

/// The flight mode guidance is flying, and why it isn't the requested regime when it can't
/// be. From [`Guidance::flight_mode`] (or [`copter_only`](Self::copter_only)); [`FlightLoop`] puts it in
/// [`TelemetryRecord::flight_mode`].
///
/// # Examples
///
/// ```
/// use stable_flight::{FlightMode, FlightModeReport, ModeRefusal};
///
/// let refused = FlightModeReport {
///     mode: FlightMode::Copter,
///     refusal: Some(ModeRefusal::Unsupported),
/// };
/// assert_ne!(refused, FlightModeReport::default());
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlightModeReport {
    /// What guidance flies this tick.
    pub mode: FlightMode,
    /// Why the requested regime isn't being flown or approached; `None` when it is (or a
    /// transition toward it is under way).
    pub refusal: Option<ModeRefusal>,
}

impl FlightModeReport {
    /// What a copter-only guidance flies when `requested`: copter, refusing a plane request
    /// as [`ModeRefusal::Unsupported`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{FlightMode, FlightModeReport, FlightRegime, ModeRefusal};
    ///
    /// assert_eq!(FlightModeReport::copter_only(FlightRegime::Copter), FlightModeReport::default());
    /// let refused = FlightModeReport::copter_only(FlightRegime::Plane);
    /// assert_eq!(refused.mode, FlightMode::Copter);
    /// assert_eq!(refused.refusal, Some(ModeRefusal::Unsupported));
    /// ```
    pub fn copter_only(requested: FlightRegime) -> Self {
        FlightModeReport {
            mode: FlightMode::Copter,
            refusal: (requested != FlightRegime::Copter).then_some(ModeRefusal::Unsupported),
        }
    }
    /// What a plane-only guidance flies when `requested`: plane, refusing a copter request
    /// as [`ModeRefusal::Unsupported`].
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{FlightMode, FlightModeReport, FlightRegime, ModeRefusal};
    ///
    /// let report = FlightModeReport::plane_only(FlightRegime::Plane);
    /// assert_eq!((report.mode, report.refusal), (FlightMode::Plane, None));
    /// let refused = FlightModeReport::plane_only(FlightRegime::Copter);
    /// assert_eq!(refused.refusal, Some(ModeRefusal::Unsupported));
    /// ```
    pub fn plane_only(requested: FlightRegime) -> Self {
        FlightModeReport {
            mode: FlightMode::Plane,
            refusal: (requested != FlightRegime::Plane).then_some(ModeRefusal::Unsupported),
        }
    }
}

/// The control input part of a [`Command`]: what the flight system should fly.
///
/// The `ControlInput` enum encapsulates different types of commands that can be issued
/// to control an aircraft:
///
/// * `Manual` - Low-level direct control inputs typically generated from user input devices.
///   This includes:
///   - `Attitude`: Controls the aircraft's orientation (roll, pitch, yaw).
///   - `Rate`: Controls the aircraft's rotational rates (rate of change of roll, pitch, yaw).
/// * `AbsoluteAttitude` - An absolute roll, pitch, and heading with collective thrust, from an
///   offboard app (e.g. MAVLink `SET_ATTITUDE_TARGET`).
/// * `AbsoluteRate` - Absolute body rates with collective thrust, from an offboard app or a
///   maneuver script.
/// * `Waypoint` - High-level navigational commands used for autonomous flight.
/// * `Land` - Descend in place until touchdown.
///
/// This separation allows the flight control system to handle both manual piloting
/// and automated navigation with a unified command interface.
///
/// # Examples
///
/// Creating a manual attitude command:
/// ```
/// use stable_flight::{ControlInput, AttitudeCommand};
///
/// let manual_cmd = ControlInput::Attitude(AttitudeCommand {
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
/// use stable_flight::{ControlInput, RateCommand};
///
/// let rate_cmd = ControlInput::Rate(RateCommand {
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
/// use stable_flight::{ControlInput, Vec2, WaypointCommand};
///
/// let waypoint_cmd = ControlInput::Waypoint(WaypointCommand {
///     position_m: Vec2::new(1000.0, 2000.0),    // Target position in world coordinates
///     velocity_mps: Some(Vec2::new(15.0, 0.0)), // Optional velocity to fly through it with
///     altitude_m: 500.0,                        // Target altitude in meters
///     cruise_speed_mps: 25.0,                   // Speed cap on the way
///     heading_rad: None,                        // Heading as guidance picks it
/// });
/// ```
///
/// # Notes
/// - A [`WaypointCommand`] with `velocity_mps: None` stops on the waypoint.
/// - `#[non_exhaustive]`: new command kinds (e.g. fixed-wing) can be added without a breaking
///   change. A [`Guidance`] that can't fly a variant must fall back to neutral, not panic (see
///   [`Guidance::guide`]).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum ControlInput {
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

    /// An absolute attitude in radians and collective thrust, typically from an offboard app.
    ///
    /// Unlike [`ControlInput::Attitude`], the angles are not stick positions: roll and pitch
    /// are flown as given (the stabilizer saturates them to its maximum tilt), and the heading
    /// is an angle, not a rate.
    AbsoluteAttitude(AbsoluteAttitudeCommand),

    /// Hold a horizontal position and descend at a steady rate until the ground stops the
    /// vehicle. Guidance only descends; deciding it has touched down and disarming is the
    /// commander's job ([`TouchdownDetector`](crate::commanders::TouchdownDetector)).
    Land(LandCommand),

    /// Absolute body rates in rad/s and collective thrust, typically from an offboard app or a
    /// maneuver script (a flip, a roll, a loop). Unlike [`ControlInput::Rate`], the rates are
    /// not stick fractions: they are flown as given, at any attitude.
    AbsoluteRate(AbsoluteRateCommand),
}

/// Zero-throttle, centered-stick [`ControlInput::Attitude`].
impl Default for ControlInput {
    fn default() -> Self {
        ControlInput::Attitude(AttitudeCommand::default())
    }
}

impl ControlInput {
    /// Whether this is pilot stick input (attitude or rate). After a link outage, the
    /// failsafe hands an armed vehicle back to sticks only once they move, so a stick left
    /// deflected can't take over unattended. Sticks are also a [stream](Self::is_stream).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::ControlInput;
    ///
    /// assert!(ControlInput::Attitude(Default::default()).is_sticks());
    /// assert!(ControlInput::Rate(Default::default()).is_sticks());
    /// assert!(!ControlInput::AbsoluteAttitude(Default::default()).is_sticks());
    /// assert!(!ControlInput::Waypoint(Default::default()).is_sticks());
    /// assert!(!ControlInput::Land(Default::default()).is_sticks());
    /// ```
    pub fn is_sticks(&self) -> bool {
        matches!(self, ControlInput::Attitude(_) | ControlInput::Rate(_))
    }

    /// Whether this input is a stream that must keep arriving to stay valid (sticks and
    /// absolute attitudes and rates), unlike a waypoint or a land, which stays valid until replaced. A link-fed
    /// commander treats a stream as stale once commands stop, even if other messages arrive.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::ControlInput;
    ///
    /// assert!(ControlInput::Attitude(Default::default()).is_stream());
    /// assert!(ControlInput::Rate(Default::default()).is_stream());
    /// assert!(ControlInput::AbsoluteAttitude(Default::default()).is_stream());
    /// assert!(ControlInput::AbsoluteRate(Default::default()).is_stream());
    /// assert!(!ControlInput::Waypoint(Default::default()).is_stream());
    /// assert!(!ControlInput::Land(Default::default()).is_stream());
    /// ```
    pub fn is_stream(&self) -> bool {
        self.is_sticks()
            || matches!(
                self,
                ControlInput::AbsoluteAttitude(_) | ControlInput::AbsoluteRate(_)
            )
    }

    /// Which kind of input this is, without its values: the flight mode a ground station shows.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{ControlInput, ControlMode};
    ///
    /// assert_eq!(ControlInput::Attitude(Default::default()).mode(), ControlMode::Attitude);
    /// assert_eq!(ControlInput::Rate(Default::default()).mode(), ControlMode::Rate);
    /// assert_eq!(ControlInput::Waypoint(Default::default()).mode(), ControlMode::Waypoint);
    /// assert_eq!(
    ///     ControlInput::AbsoluteAttitude(Default::default()).mode(),
    ///     ControlMode::AbsoluteAttitude,
    /// );
    /// assert_eq!(ControlInput::Land(Default::default()).mode(), ControlMode::Land);
    /// assert_eq!(
    ///     ControlInput::AbsoluteRate(Default::default()).mode(),
    ///     ControlMode::AbsoluteRate,
    /// );
    /// ```
    pub fn mode(&self) -> ControlMode {
        match self {
            ControlInput::Attitude(_) => ControlMode::Attitude,
            ControlInput::Rate(_) => ControlMode::Rate,
            ControlInput::Waypoint(_) => ControlMode::Waypoint,
            ControlInput::AbsoluteAttitude(_) => ControlMode::AbsoluteAttitude,
            ControlInput::Land(_) => ControlMode::Land,
            ControlInput::AbsoluteRate(_) => ControlMode::AbsoluteRate,
        }
    }
}

/// The kind of a [`ControlInput`], one variant per input variant: the flight mode, compact
/// enough for a heartbeat. From [`ControlInput::mode`].
///
/// `#[non_exhaustive]` like [`ControlInput`]: a new input kind adds a mode, and a consumer
/// that doesn't know it reports no mode.
///
/// The variant order is on the link's wire (`stable-flight-link`'s heartbeat): changing it is
/// a protocol change, which bumps that crate's `PROTOCOL_VERSION`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum ControlMode {
    /// Stick inputs command attitude ([`ControlInput::Attitude`]).
    Attitude,
    /// Stick inputs command body rates ([`ControlInput::Rate`]).
    Rate,
    /// Autonomous flight to a waypoint ([`ControlInput::Waypoint`]).
    Waypoint,
    /// An offboard app commands an absolute attitude ([`ControlInput::AbsoluteAttitude`]).
    AbsoluteAttitude,
    /// Autonomous descent to touchdown ([`ControlInput::Land`]).
    Land,
    /// An offboard app or script commands absolute body rates ([`ControlInput::AbsoluteRate`]).
    AbsoluteRate,
}

/// Represents direct manual control of an aircraft's attitude (orientation).
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's orientation
/// (roll, pitch, yaw) and throttle. It is typically used within a `ControlInput::Attitude`
/// variant to issue manual flight commands to the flight system.
///
/// # Value Ranges
///
/// - `throttle`: `0.0` (idle) to `1.0` (full power)
/// - `roll`: `-1.0` (full left bank) to `1.0` (full right bank)
/// - `pitch`: `-1.0` (full nose down) to `1.0` (full nose up)
/// - `yaw`: `-1.0` (full left turn) to `1.0` (full right turn)
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating an `AttitudeCommand` for a slight climb with a right bank:
/// ```
/// use stable_flight::AttitudeCommand;
///
/// let cmd = AttitudeCommand {
///     throttle: 0.75,  // 75% throttle
///     roll: 0.1,       // Slight right bank
///     pitch: 0.05,     // Slight nose up
///     yaw: 0.0,        // No yaw input
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
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
}

/// Represents direct manual control of an aircraft's rotational rates.
///
/// This struct encapsulates pilot inputs from control devices like joysticks,
/// yokes, or keyboard/mouse combinations to control the aircraft's rate of rotation
/// (roll rate, pitch rate, yaw rate) and throttle. It is typically used within a
/// `ControlInput::Rate` variant to issue manual flight commands to the flight system.
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
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `RateCommand` for a moderate right roll and slight nose-up rate:
/// ```
/// use stable_flight::RateCommand;
///
/// let cmd = RateCommand {
///     throttle: 0.6,    // 60% throttle
///     roll_rate: 0.3,   // Moderate right roll rate
///     pitch_rate: 0.1,  // Slight nose-up rate
///     yaw_rate: 0.0,    // No yaw rate
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
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
}

/// Represents a navigational waypoint for autonomous flight.
///
/// A `WaypointCommand` instructs the aircraft to navigate to a specific position
/// in 2D space at a given altitude, at up to a cruise speed, optionally flying through it
/// with a given velocity. This struct is
/// typically used within a `ControlInput::Waypoint` variant to issue autonomous navigation
/// commands to the flight system.
///
/// # Value Ranges
///
/// - `position_m`: X, Y coordinates in 2D space (units and coordinate system depend on the flight system, typically meters in world coordinates)
/// - `velocity_mps`: Optional velocity vector in meters per second (north, east), fed forward while flying to the waypoint
/// - `altitude_m`: Target altitude in meters (typically non-negative; negative values may be interpreted as ground level or cause undefined behavior)
/// - `cruise_speed_mps`: Speed cap in meters per second while flying to the waypoint; zero, negative, or NaN means no cap
/// - `heading_rad`: Optional heading in radians from north; any angle, wrapped by guidance
///
/// Note: The specified ranges are conventional and recommended for typical use.
/// Values outside these ranges may be clamped, ignored, or cause undefined behavior
/// depending on the flight system's implementation.
///
/// # Examples
///
/// Creating a `WaypointCommand` to navigate to a position with a specific altitude and speed:
/// ```
/// use stable_flight::{Vec2, WaypointCommand};
///
/// let cmd = WaypointCommand {
///     position_m: Vec2::new(1000.0, 2000.0),    // Target position (X=1000m, Y=2000m)
///     velocity_mps: Some(Vec2::new(15.0, 0.0)), // Fly through it northward at 15 m/s
///     altitude_m: 500.0,                        // Target altitude of 500 meters
///     cruise_speed_mps: 25.0,                   // At most 25 m/s on the way
///     heading_rad: Some(0.5),                   // Face 0.5 rad east of north
/// };
/// ```
///
/// # Behavior of Optional Fields
///
/// - If `velocity_mps` is `None`, the aircraft stops on the waypoint.
///   [`WaypointFollower`](crate::commanders::WaypointFollower) fills it in along each leg so
///   a route is flown at cruise speed.
///
/// # Default Implementation
///
/// The `Default` trait is implemented for `WaypointCommand`, providing the following
/// default values:
/// - `position_m`: zero
/// - `velocity_mps`: `None`
/// - `altitude_m`: `0.0`
/// - `cruise_speed_mps`: `0.0`
/// - `heading_rad`: `None`
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WaypointCommand {
    /// The target position in 2D space (X, Y coordinates).
    ///
    /// The coordinate system and units depend on the flight system implementation
    /// (typically meters in world coordinates).
    pub position_m: Vec2,

    /// Optional velocity, in meters per second (north, east), to fly through the waypoint
    /// with.
    ///
    /// Guidance feeds it forward while flying to the waypoint (added to the position loop's
    /// demand, then capped at `cruise_speed_mps`), so the aircraft passes the waypoint
    /// instead of stopping on it. `None` stops on the waypoint.
    /// [`WaypointFollower`](crate::commanders::WaypointFollower) fills it in along each leg
    /// when it's `None`.
    pub velocity_mps: Option<Vec2>,

    /// Target altitude in meters.
    ///
    /// Typically non-negative; negative values may be interpreted as ground level
    /// or cause undefined behavior depending on the flight system.
    pub altitude_m: f32,

    /// Cruise speed in meters per second: the most guidance asks for while flying to the
    /// waypoint. Zero, negative, or NaN means no cap (the guidance's tilt limit still
    /// bounds the speed).
    pub cruise_speed_mps: f32,

    /// Optional heading to hold, in radians from north.
    ///
    /// Copter guidance turns its heading-hold target to it at
    /// [`heading_rate_radps`](crate::guidance::copter::CopterGuidanceConfig::heading_rate_radps),
    /// instead of along the leg, and never yields it; on a tailsitter it is the hover frame's
    /// heading, the direction the belly faces. `None` (or NaN) leaves the heading to guidance.
    /// Plane guidance ignores it.
    pub heading_rad: Option<f32>,
}

/// An absolute attitude and collective thrust, for [`ControlInput::AbsoluteAttitude`]: what
/// an offboard app (e.g. MAVLink `SET_ATTITUDE_TARGET`) asks for, as opposed to the normalized
/// stick positions of an [`AttitudeCommand`].
///
/// Per the input policy, nothing is rejected: the stabilizer saturates roll and pitch to its
/// maximum tilt and flies NaN as level, a NaN heading holds the current heading-hold target,
/// and thrust saturates to `[0, 1]` (NaN means none).
///
/// # Examples
///
/// A 10° right bank, holding north, at 45% thrust:
/// ```
/// use stable_flight::{AbsoluteAttitudeCommand, ControlInput};
///
/// let input = ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
///     roll_rad: 10.0_f32.to_radians(),
///     pitch_rad: 0.0,
///     heading_rad: 0.0,
///     thrust: 0.45,
/// });
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AbsoluteAttitudeCommand {
    /// Roll in radians, right positive, per the
    /// [frame contract](VehicleState#frame-contract)'s `roll_rad`.
    pub roll_rad: f32,
    /// Pitch in radians, nose up positive, per the
    /// [frame contract](VehicleState#frame-contract)'s `pitch_rad`.
    pub pitch_rad: f32,
    /// Absolute heading in radians, in the
    /// [frame contract](VehicleState#frame-contract)'s `heading_rad` convention; any value,
    /// wrapped by guidance.
    pub heading_rad: f32,
    /// Collective thrust along the body axis, `0.0` to `1.0`. Unlike an [`AttitudeCommand`]'s
    /// throttle, it is not boosted for tilt: the sender owns that compensation.
    pub thrust: f32,
}

/// Absolute body rates and collective thrust, for [`ControlInput::AbsoluteRate`]: what a
/// maneuver or an offboard app (e.g. MAVLink `SET_ATTITUDE_TARGET` body rates) asks for, as
/// opposed to the stick fractions of a [`RateCommand`].
///
/// Per the input policy, nothing is rejected: the stabilizer saturates the rates to its
/// limits (NaN as zero) and thrust to `[0, 1]` (NaN means none).
///
/// # Examples
///
/// A full-rate roll to the right at 60% thrust:
/// ```
/// use stable_flight::{AbsoluteRateCommand, ControlInput};
///
/// let input = ControlInput::AbsoluteRate(AbsoluteRateCommand {
///     roll_radps: 360.0_f32.to_radians(),
///     thrust: 0.6,
///     ..Default::default()
/// });
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AbsoluteRateCommand {
    /// Roll rate about body `x` in rad/s, right positive.
    pub roll_radps: f32,
    /// Pitch rate about body `y` in rad/s, nose up positive.
    pub pitch_radps: f32,
    /// Yaw rate about body `z` in rad/s, nose right positive.
    pub yaw_radps: f32,
    /// Collective thrust, `0.0` to `1.0`.
    pub thrust: f32,
}

/// Where to land, for [`ControlInput::Land`]: guidance holds `position_m` while it descends.
///
/// # Examples
///
/// ```
/// use stable_flight::{ControlInput, LandCommand, Vec2};
///
/// let input = ControlInput::Land(LandCommand { position_m: Vec2::new(10.0, -5.0) });
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LandCommand {
    /// Horizontal position to hold, in meters, in the same frame as
    /// [`WaypointCommand::position_m`].
    pub position_m: Vec2,
}

/// Represents the high-level control outputs from the stabilization system.
///
/// This struct encapsulates the desired thrust and moments to be applied to the
/// vehicle, which are later translated to specific actuator commands by a `Mixer`.
///
/// Most vehicles fly one set of actuators per output: `thrust` and `moments` reach every
/// motor and servo the mixer drives. A lift+cruise VTOL (quadplane) flies two at once, its
/// lift rotors on `thrust` and `moments` and its pusher and control surfaces on
/// [`cruise`](Self::cruise).
///
/// # Examples
///
/// Creating a control output for a simple hover command:
/// ```
/// use stable_flight::{ControlOutput, Vec3};
///
/// let output = ControlOutput::new(
///     0.5, // 50% thrust
///     Vec3::zeros() // No roll, pitch, or yaw moments
/// );
/// ```
///
/// A quadplane in cruise: lift rotors stopped, the pusher and surfaces flying.
/// ```
/// use stable_flight::{ControlOutput, CruiseOutput, Vec3};
///
/// let output = ControlOutput {
///     cruise: Some(CruiseOutput { thrust: 0.6, moments: Vec3::new(0.1, 0.0, 0.0) }),
///     ..Default::default()
/// };
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ControlOutput {
    /// Normalized thrust value, typically ranging from 0.0 (no thrust) to 1.0 (maximum thrust).
    /// On a lift+cruise VTOL, the lift rotors' collective.
    pub thrust: f32,

    /// Control moments in roll (x), pitch (y), and yaw (z) axes.
    /// Values are typically normalized between -1.0 and 1.0, where the magnitude
    /// represents the proportion of maximum control authority in each axis.
    /// On a lift+cruise VTOL, the lift rotors' moments only.
    pub moments: Vec3,

    /// A lift+cruise VTOL's cruise actuators, flown alongside the lift rotors: the pusher's
    /// thrust and the control surfaces' moments. `None` for every other vehicle, whose servos
    /// take [`moments`](Self::moments).
    pub cruise: Option<CruiseOutput>,
}

impl ControlOutput {
    /// Creates a new `ControlOutput` with the specified thrust and control moments, and no
    /// [`cruise`](Self::cruise) output.
    ///
    /// # Parameters
    ///
    /// - `thrust`: Normalized thrust value, typically between 0.0 and 1.0.
    /// - `moments`: Control moments as a 3D vector (roll, pitch, yaw).
    ///
    /// # Returns
    ///
    /// A new `ControlOutput` instance with the specified parameters.
    pub fn new(thrust: f32, moments: Vec3) -> ControlOutput {
        ControlOutput {
            thrust,
            moments,
            cruise: None,
        }
    }
}

/// A lift+cruise VTOL's cruise output ([`ControlOutput::cruise`]): what its pusher and
/// control surfaces fly while the lift rotors fly [`ControlOutput::thrust`] and
/// [`ControlOutput::moments`].
///
/// # Examples
///
/// ```
/// use stable_flight::{CruiseOutput, Vec3};
///
/// // Pusher at 60%, ailerons rolling right.
/// let cruise = CruiseOutput { thrust: 0.6, moments: Vec3::new(0.2, 0.0, 0.0) };
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CruiseOutput {
    /// The pusher's normalized thrust, in `[0, 1]`.
    pub thrust: f32,
    /// The control surfaces' roll (x), pitch (y), and yaw (z) moments, each in `[-1, 1]`.
    pub moments: Vec3,
}

/// Represents the physical state of a vehicle in 3D space.
///
/// This struct encapsulates all the key state information needed for flight control,
/// including position, velocity, acceleration, attitude, and angular velocity.
/// Quantities are plain SI `f32`s; field names carry the unit (`position_m`, `velocity_mps`, …).
///
/// # Frame contract
///
/// This table is the reference for how every [`StateSource`] reports state; a backend
/// converts from its native frames at its boundary, and `testing::conformance` (`testing`
/// feature) checks it.
///
/// Two frames are involved. Attitude follows aerospace convention: the NED world frame
/// (`x` north, `y` east, `z` down) and the FRD body frame (`x` forward, `y` right, `z` down).
/// Translation uses the same north and east axes but flips `z` to point **up**, so altitude
/// and climb rate are positive: `position_m`/`velocity_mps`/`acceleration_mps2` `z` is NED `z`
/// negated.
///
/// | Quantity | Frame | Units | Positive |
/// |---|---|---|---|
/// | `position_m` | `x` north, `y` east, `z` up | m | `z` is altitude above the backend's datum (e.g. sea level) |
/// | `velocity_mps`, `acceleration_mps2` | as `position_m` | m/s, m/s² | `z` climbing |
/// | `attitude` | NED world → FRD body, ZYX (yaw, pitch, roll); `UnitQuaternion::from_euler_angles(roll, pitch, heading)` | — | [`euler_angles`] of it match the fields below |
/// | `roll_rad` | about body `x` | rad | right wing down |
/// | `pitch_rad` | about body `y` | rad | nose up |
/// | `heading_rad` | about NED `z` | rad | clockwise from north (east = π/2); any range, e.g. [0, 2π) or [-π, π) |
/// | `angular_velocity_radps` | FRD body: `x` roll, `y` pitch, `z` yaw | rad/s | roll right, nose up, nose right |
/// | `airspeed_mps` | speed relative to the air mass (any frame) | m/s | non-negative; `None` without an air-data source |
///
/// Near ±90° pitch the Euler fields are degenerate: roll and heading turn about the same axis,
/// so only their sum (or difference) means anything, and [`euler_angles`] picks one split.
/// `attitude` and `angular_velocity_radps` stay well defined, so code that must work there (a
/// tailsitter's nose-up hover) uses them, not the Euler fields.
///
/// # Examples
///
/// Creating a default vehicle state:
/// ```
/// use stable_flight::VehicleState;
///
/// let state = VehicleState::default();
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VehicleState {
    /// World position (north, east, up); see the [frame contract](VehicleState#frame-contract).
    pub position_m: Vec3,

    /// World velocity (north, east, up).
    pub velocity_mps: Vec3,

    /// World acceleration (north, east, up).
    pub acceleration_mps2: Vec3,

    /// Orientation quaternion representing the vehicle's attitude.
    /// The quaternion represents the rotation from world frame to body frame.
    pub attitude: UnitQuaternion<f32>,

    /// Body angular velocity in roll (x), pitch (y), and yaw (z), in radians per second.
    pub angular_velocity_radps: Vec3,

    /// Current roll angle in radians, right wing down positive.
    pub roll_rad: f32,

    /// Current pitch angle in radians, nose up positive.
    pub pitch_rad: f32,

    /// Current heading (yaw) angle in radians.
    /// 0 is north, with positive values rotating clockwise.
    pub heading_rad: f32,

    /// Speed relative to the air mass in meters per second, or `None` without an air-data
    /// source (e.g. no pitot). A magnitude, so it doesn't depend on the body frame. Consumers
    /// read it through [`valid_airspeed_mps`](Self::valid_airspeed_mps), which treats a NaN
    /// or negative value as `None` (input policy).
    pub airspeed_mps: Option<f32>,
}

impl VehicleState {
    /// [`airspeed_mps`](Self::airspeed_mps) when it is usable: `None` without air data, and
    /// for a NaN or negative reading (input policy).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::VehicleState;
    ///
    /// // (reported, usable)
    /// let cases = [
    ///     (Some(12.0), Some(12.0)),
    ///     (Some(0.0), Some(0.0)),
    ///     (None, None),
    ///     (Some(f32::NAN), None),
    ///     (Some(-1.0), None),
    /// ];
    /// for (airspeed_mps, usable) in cases {
    ///     let state = VehicleState { airspeed_mps, ..Default::default() };
    ///     assert_eq!(state.valid_airspeed_mps(), usable, "{airspeed_mps:?}");
    /// }
    /// ```
    pub fn valid_airspeed_mps(&self) -> Option<f32> {
        self.airspeed_mps
            .filter(|airspeed_mps| *airspeed_mps >= 0.0)
    }

    /// This state in another body frame: `rotation` is the new body frame expressed in the old
    /// one (a constant mount or view rotation).
    ///
    /// The attitude becomes `attitude · rotation`, the body rates `rotation⁻¹ · ω` (the same
    /// rotation seen in the new axes), and the Euler fields are recomputed from the new
    /// attitude. World position, velocity, acceleration, and airspeed don't depend on the body
    /// frame and are unchanged.
    ///
    /// # Examples
    ///
    /// ```
    /// use core::f32::consts::FRAC_PI_2;
    /// use stable_flight::nalgebra::UnitQuaternion;
    /// use stable_flight::VehicleState;
    ///
    /// // A tailsitter hovering nose up, seen in a frame rotated −90° in pitch, reads level.
    /// let nose_up = VehicleState {
    ///     attitude: UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0),
    ///     ..Default::default()
    /// };
    /// let view = nose_up.rotate_body(&UnitQuaternion::from_euler_angles(0.0, -FRAC_PI_2, 0.0));
    /// assert!(view.pitch_rad.abs() < 1e-6);
    /// ```
    pub fn rotate_body(&self, rotation: &UnitQuaternion<f32>) -> Self {
        let attitude = self.attitude * rotation;
        let (roll_rad, pitch_rad, heading_rad) = euler_angles(&attitude);
        VehicleState {
            attitude,
            angular_velocity_radps: rotation.inverse_transform_vector(&self.angular_velocity_radps),
            roll_rad,
            pitch_rad,
            heading_rad,
            ..self.clone()
        }
    }
}

/// Represents a physical vehicle that can be controlled by the flight system.
///
/// A `Vehicle` is anything that is both a [`StateSource`] and an [`Actuator`]; the blanket
/// impl below provides it automatically, so implement those two traits rather than this one.
/// The split mirrors hardware, where sensing (IMU, estimator) and actuation (ESC/servo output)
/// are usually separate drivers, while a simulator bridge can implement both on one type.
///
/// Examples: a simulator bridge such as `RealFlightVehicle`, a hardware IMU/estimator paired
/// with an ESC driver (see [`ComposedVehicle`]), or a test double such as
/// `testing::NullVehicle` (`testing` feature).
///
/// # Timing contract
///
/// [`FlightLoop::step`] calls [`StateSource::state`] once, then
/// [`Actuator::actuate`] once, every tick. A backend must:
///
/// - report state in the [`VehicleState` frame contract](VehicleState#frame-contract);
/// - talk to its simulator or hardware at most once per call, and should not block for longer
///   than a tick (document it if a blocking link can);
/// - document how old the state from `state` is relative to the last `actuate`. A
///   backend whose link sends controls and returns state in one round-trip exchanges in
///   `actuate` and returns the cached result, so tick *n* stabilizes on state reported when
///   the simulator received tick *n − 1*'s controls. A backend with a separate sensor read
///   should read fresh state in `state`;
/// - produce real state on the priming call (`dt == Duration::ZERO`, made once by
///   [`FlightLoop::prime`] before any `actuate`), contacting the simulator if it must, and
///   document what it returns if that fails.
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{Actuator, ActuatorCommand, StateSource, Vehicle, VehicleState};
///
/// struct HoverSim {
///     state: VehicleState,
/// }
///
/// impl StateSource for HoverSim {
///     fn state(&mut self, _dt: Duration) -> VehicleState {
///         self.state.clone()
///     }
/// }
///
/// impl Actuator for HoverSim {
///     fn actuate(&mut self, _command: &ActuatorCommand) {}
/// }
///
/// fn requires_vehicle(_: &impl Vehicle) {}
/// requires_vehicle(&HoverSim { state: VehicleState::default() });
/// ```
pub trait Vehicle: StateSource + Actuator {}

impl<T: StateSource + Actuator> Vehicle for T {}

/// Where the flight loop's vehicle state comes from.
///
/// The name describes the role, not how the state is obtained. Implementors include:
/// - **Simulator backends** that report the simulator's true state (e.g. `RealFlightVehicle`);
/// - **Test doubles** that report a fixed state (`testing::NullVehicle`);
/// - **Estimators** that fuse sensor readings into an estimate (IMU, baro, mag, GNSS).
///
/// Every implementor reports in the [`VehicleState` frame contract](VehicleState#frame-contract).
pub trait StateSource {
    /// Returns the current vehicle state.
    ///
    /// Takes `&mut self` because sources typically advance internal state on each call
    /// (a filter's predict/update, a simulator exchange's bookkeeping).
    ///
    /// # Parameters
    /// - `dt`: Time elapsed since the previous call. The first call, made once before the
    ///   control loop starts, passes `Duration::ZERO`; implementations may use it to
    ///   initialize (see `RealFlightVehicle`).
    fn state(&mut self, dt: Duration) -> VehicleState;
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

/// What the [`Stabilizer`] should hold this tick, as produced by a [`Guidance`].
///
/// A [`ControlInput`] says what the operator or mission wants (stick positions, a waypoint);
/// a `Setpoint` is the inner-loop target derived from it. Attitude targets are physical
/// angles in radians; rates and thrust are normalized. The stabilizer owns the physical
/// limits (maximum tilt and rates) and the thrust curve, and saturates out-of-range or NaN
/// inputs per the input policy.
///
/// `#[non_exhaustive]`: new setpoint kinds (e.g. fixed-wing) can be added without a breaking
/// change. A [`Stabilizer`] that can't fly a variant must fall back to neutral, not panic (see
/// [`Stabilizer::stabilize`]).
///
/// # Examples
///
/// ```
/// use stable_flight::{AttitudeSetpoint, Setpoint};
///
/// // Level, north-facing hover at half linear thrust.
/// let hover = Setpoint::Attitude(AttitudeSetpoint { thrust: 0.5, ..Default::default() });
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum Setpoint {
    /// Hold a roll/pitch attitude and an absolute heading.
    Attitude(AttitudeSetpoint),
    /// Hold body rates, as fractions of the stabilizer's maximum.
    Rate(RateSetpoint),
    /// Hold absolute body rates (rad/s): a copter stabilizer's acro and maneuvers.
    BodyRate(BodyRateSetpoint),
    /// Fly a wing-borne roll/pitch attitude, turns coordinated (no heading): a plane
    /// stabilizer's.
    Plane(PlaneSetpoint),
    /// Hold absolute body rates (rad/s) on the wing, at any attitude: a plane stabilizer's acro
    /// and maneuvers (rolls, loops).
    PlaneRate(BodyRateSetpoint),
    /// Fly a hover and a wing-borne attitude at once, the lift rotors carrying
    /// [`lift_share`](BlendSetpoint::lift_share) of theirs: a lift+cruise VTOL's transitions.
    Blend(BlendSetpoint),
}

impl Setpoint {
    /// The linear thrust the setpoint carries; a [`Blend`](Self::Blend)'s hover thrust (the
    /// lift rotors', before their share).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{RateSetpoint, Setpoint};
    ///
    /// let setpoint = Setpoint::Rate(RateSetpoint { thrust: 0.4, ..Default::default() });
    /// assert_eq!(setpoint.thrust(), 0.4);
    /// ```
    pub fn thrust(&self) -> f32 {
        match self {
            Setpoint::Attitude(attitude) => attitude.thrust,
            Setpoint::Rate(rates) => rates.thrust,
            Setpoint::BodyRate(rates) | Setpoint::PlaneRate(rates) => rates.thrust,
            Setpoint::Plane(plane) => plane.thrust,
            Setpoint::Blend(blend) => blend.hover.thrust,
        }
    }

    /// Whether guidance marked this setpoint ground idle (see
    /// [`AttitudeSetpoint::idle`]); a plane attitude never is, a blend when its hover is.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{PlaneSetpoint, RateSetpoint, Setpoint};
    ///
    /// assert!(Setpoint::Rate(RateSetpoint { idle: true, ..Default::default() }).idle());
    /// assert!(!Setpoint::Plane(PlaneSetpoint::default()).idle());
    /// ```
    pub fn idle(&self) -> bool {
        match self {
            Setpoint::Attitude(attitude) => attitude.idle,
            Setpoint::Rate(rates) => rates.idle,
            Setpoint::BodyRate(rates) | Setpoint::PlaneRate(rates) => rates.idle,
            Setpoint::Plane(_) => false,
            Setpoint::Blend(blend) => blend.hover.idle,
        }
    }

    /// This setpoint with its [`thrust`](Self::thrust) replaced by `thrust`, its other fields
    /// (the [`idle`](Self::idle) mark included) kept.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{RateSetpoint, Setpoint};
    ///
    /// let setpoint = Setpoint::Rate(RateSetpoint { thrust: 0.4, ..Default::default() });
    /// assert_eq!(setpoint.with_thrust(0.7).thrust(), 0.7);
    /// ```
    pub fn with_thrust(self, thrust: f32) -> Self {
        match self {
            Setpoint::Attitude(attitude) => {
                Setpoint::Attitude(AttitudeSetpoint { thrust, ..attitude })
            }
            Setpoint::Rate(rates) => Setpoint::Rate(RateSetpoint { thrust, ..rates }),
            Setpoint::BodyRate(rates) => Setpoint::BodyRate(BodyRateSetpoint { thrust, ..rates }),
            Setpoint::Plane(plane) => Setpoint::Plane(PlaneSetpoint { thrust, ..plane }),
            Setpoint::PlaneRate(rates) => Setpoint::PlaneRate(BodyRateSetpoint { thrust, ..rates }),
            Setpoint::Blend(blend) => Setpoint::Blend(BlendSetpoint {
                hover: AttitudeSetpoint {
                    thrust,
                    ..blend.hover
                },
                ..blend
            }),
        }
    }
}

/// Absolute body-rate target, for [`Setpoint::BodyRate`] (copter) and [`Setpoint::PlaneRate`]
/// (plane). The stabilizer saturates each rate to its limit (NaN as zero).
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BodyRateSetpoint {
    /// Roll rate about body `x` in rad/s, right positive.
    pub roll_radps: f32,
    /// Pitch rate about body `y` in rad/s, nose up positive.
    pub pitch_radps: f32,
    /// Yaw rate about body `z` in rad/s, nose right positive.
    pub yaw_radps: f32,
    /// Linear collective thrust, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
    /// Ground idle, as [`AttitudeSetpoint::idle`]; a plane stabilizer ignores it.
    pub idle: bool,
}

/// The rates of an [`AbsoluteRateCommand`], as given.
///
/// # Examples
///
/// ```
/// use stable_flight::{AbsoluteRateCommand, BodyRateSetpoint};
///
/// let command = AbsoluteRateCommand { roll_radps: 2.0, thrust: 0.5, ..Default::default() };
/// assert_eq!(BodyRateSetpoint::from(&command).roll_radps, 2.0);
/// ```
impl From<&AbsoluteRateCommand> for BodyRateSetpoint {
    fn from(command: &AbsoluteRateCommand) -> Self {
        // Exhaustive destructuring: a new field fails to compile here until it is mapped.
        let AbsoluteRateCommand {
            roll_radps,
            pitch_radps,
            yaw_radps,
            thrust,
        } = *command;
        Self {
            roll_radps,
            pitch_radps,
            yaw_radps,
            thrust,
            idle: false,
        }
    }
}

/// Wing-borne attitude target for a plane stabilizer, for [`Setpoint::Plane`], in the
/// [frame contract](VehicleState#frame-contract)'s body frame (a tailsitter's airplane
/// frame). Turns are coordinated, so there is no heading: the stabilizer derives the yaw rate
/// from bank and airspeed. The stabilizer saturates roll and pitch to its limits; NaN means
/// level.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PlaneSetpoint {
    /// Bank in radians, right wing down positive.
    pub roll_rad: f32,
    /// Pitch in radians, nose up positive.
    pub pitch_rad: f32,
    /// Linear throttle, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
    /// A tailsitter transition ramp: pitch saturates at ±π/2, not the stabilizer's limit, so
    /// the ramp reaches its handover whatever the cruise limit (ArduPilot's tailsitter ramps
    /// past its pitch limits, `tailsitter.cpp:957`).
    pub transition: bool,
}

/// A hover and a wing-borne target flown together, for [`Setpoint::Blend`]: a lift+cruise
/// VTOL (quadplane) in transition, its lift rotors flying `hover` and its surfaces and pusher
/// flying `plane`.
///
/// The stabilizer weights the lift rotors' thrust and moments by `lift_share` and flies the
/// surfaces and pusher in full (PX4's standard VTOL,
/// `src/modules/vtol_att_control/standard.cpp:294-368`). A NaN share counts as 1, the lift
/// rotors carrying it all; others saturate to `[0, 1]`.
///
/// # Examples
///
/// ```
/// use stable_flight::{AttitudeSetpoint, BlendSetpoint, PlaneSetpoint, Setpoint};
///
/// // Halfway through handing the lift to the wing, the pusher at full throttle.
/// let blend = Setpoint::Blend(BlendSetpoint {
///     hover: AttitudeSetpoint { thrust: 0.5, ..Default::default() },
///     plane: PlaneSetpoint { thrust: 1.0, ..Default::default() },
///     lift_share: 0.5,
/// });
/// assert_eq!(blend.thrust(), 0.5);
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BlendSetpoint {
    /// The lift rotors' target, flown by the hover side.
    pub hover: AttitudeSetpoint,
    /// The surfaces' and pusher's target, flown by the plane side.
    pub plane: PlaneSetpoint,
    /// The share of hover lift the lift rotors carry, `0.0` to `1.0`.
    pub lift_share: f32,
}

/// Attitude target for the [`Stabilizer`]; see [`Setpoint`].
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AttitudeSetpoint {
    /// Roll target in radians, right positive, per the
    /// [frame contract](VehicleState#frame-contract)'s `roll_rad`. The stabilizer saturates it
    /// to its maximum tilt; NaN means level.
    pub roll_rad: f32,
    /// Pitch target in radians, nose up positive, per the
    /// [frame contract](VehicleState#frame-contract)'s `pitch_rad`. The stabilizer saturates it
    /// to its maximum tilt; NaN means level.
    pub pitch_rad: f32,
    /// Absolute heading target in radians, in the
    /// [frame contract](VehicleState#frame-contract)'s `heading_rad` convention.
    pub heading_rad: f32,
    /// Linear collective thrust, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
    /// A tailsitter transition ramp: pitch saturates at ±π/2, not the stabilizer's maximum
    /// tilt, so the ramp reaches its handover whatever the hover limit (ArduPilot's tailsitter
    /// ramps past its angle limit, `tailsitter.cpp:871`).
    pub transition: bool,
    /// Ground idle: guidance sets it while a piloted throttle is at or below
    /// [`IDLE_THROTTLE`](crate::guidance::copter::IDLE_THROTTLE) (or NaN), and a copter
    /// stabilizer then clears its integrals. A zero `thrust` alone (the altitude loop in
    /// flight) is not idle.
    pub idle: bool,
}

/// Body-rate target for the [`Stabilizer`]; see [`Setpoint`].
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RateSetpoint {
    /// Roll rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (right positive).
    pub roll_rate: f32,
    /// Pitch rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (nose up
    /// positive).
    pub pitch_rate: f32,
    /// Yaw rate as a fraction of the stabilizer's maximum, `-1.0` to `1.0` (nose right
    /// positive).
    pub yaw_rate: f32,
    /// Linear collective thrust, `0.0` to `1.0`, before the stabilizer's thrust curve.
    pub thrust: f32,
    /// Ground idle, as [`AttitudeSetpoint::idle`].
    pub idle: bool,
}

/// Passes the sticks through as rates, with `throttle` as `thrust`.
///
/// # Examples
///
/// ```
/// use stable_flight::{RateCommand, RateSetpoint};
///
/// let command = RateCommand {
///     throttle: 0.6,
///     roll_rate: 0.1,
///     pitch_rate: -0.2,
///     yaw_rate: 0.3,
/// };
/// let setpoint = RateSetpoint::from(&command);
/// assert_eq!(
///     setpoint,
///     RateSetpoint { roll_rate: 0.1, pitch_rate: -0.2, yaw_rate: 0.3, thrust: 0.6, idle: false }
/// );
/// ```
impl From<&RateCommand> for RateSetpoint {
    fn from(command: &RateCommand) -> Self {
        // Exhaustive destructuring: a new field fails to compile here until it is mapped.
        let RateCommand {
            throttle,
            roll_rate,
            pitch_rate,
            yaw_rate,
        } = *command;
        Self {
            roll_rate,
            pitch_rate,
            yaw_rate,
            thrust: throttle,
            idle: false,
        }
    }
}

/// Turns the commander's [`ControlInput`] into an inner-loop [`Setpoint`], in the requested
/// [`FlightRegime`].
///
/// Guidance only runs while the [`Command`] is armed, so it never sees `arm`. It owns the
/// flight mode: a guidance that flies more than one regime moves toward the requested one and
/// reports what it flies through [`flight_mode`](Self::flight_mode); a copter-only guidance
/// ignores the request and reports nothing, and [`FlightLoop`] reports it as
/// [`FlightModeReport::copter_only`].
///
/// Guidance is the outer loop: waypoint navigation (position and altitude hold), and the
/// interpretation of pilot sticks that needs memory, such as integrating yaw stick into a
/// heading-hold target. The [`Stabilizer`] then tracks the setpoint. [`FlightLoop`] calls both
/// every tick; a guidance that runs its loops slower paces them itself from `dt`. Why they are
/// split is in the
/// [architecture overview](https://github.com/wboayue/stable-flight/blob/main/architecture.md#core-components).
///
/// # Examples
///
/// A guidance that flies rate commands through unchanged, and otherwise levels out with
/// thrust cut:
///
/// ```
/// use stable_flight::{
///     AttitudeSetpoint, ControlInput, FlightRegime, Guidance, Setpoint, VehicleState,
/// };
///
/// struct RateOnly;
///
/// impl Guidance for RateOnly {
///     fn guide(
///         &mut self,
///         _dt: f32,
///         _regime: FlightRegime,
///         input: &ControlInput,
///         state: &VehicleState,
///     ) -> Setpoint {
///         match input {
///             ControlInput::Rate(rate) => Setpoint::Rate(rate.into()),
///             _ => Setpoint::Attitude(AttitudeSetpoint {
///                 heading_rad: state.heading_rad,
///                 ..Default::default()
///             }),
///         }
///     }
/// }
/// ```
pub trait Guidance {
    /// Produces this tick's inner-loop setpoint.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last guidance step in seconds. Must be positive and
    ///   finite; implementations may assert it (runners guarantee it).
    /// - `regime`: The commander's requested regime ([`Command::regime`]), repeated every
    ///   tick.
    /// - `input`: The commander's current control input.
    /// - `state`: The current estimated state of the vehicle.
    ///
    /// # Unsupported inputs
    ///
    /// [`ControlInput`] is `#[non_exhaustive]`. For a variant it can't fly, an implementation
    /// returns a neutral setpoint — level attitude, the current heading, zero thrust (what the
    /// input policy gives a NaN command) — and must not panic. A regime it can't fly is
    /// refused, not an error: it keeps flying the one it can and reports why
    /// ([`ModeRefusal`]).
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint;

    /// Resets guidance and primes it from a known-good vehicle state.
    ///
    /// Clears runtime state (integrators, held outputs) and warms the rest (e.g. the
    /// heading-hold target) so guidance starts from where the vehicle is rather than from
    /// zero. [`FlightLoop`] calls it before the first tick and again each time the
    /// [`Command`] arms, since guidance doesn't run while disarmed. The default does
    /// nothing.
    fn prime(&mut self, _state: &VehicleState) {}

    /// What the last [`guide`](Self::guide) call clipped to a limit. [`FlightLoop`] merges it
    /// into the tick's [`TelemetryRecord::saturation`]. The default reports nothing.
    fn saturation(&self) -> Saturation {
        Saturation::default()
    }

    /// The flight mode the last [`guide`](Self::guide) call flew, for a guidance that flies
    /// more than one regime. [`FlightLoop`] puts it in an armed tick's
    /// [`TelemetryRecord::flight_mode`]. The default, `None`, means copter only: the loop
    /// reports [`FlightModeReport::copter_only`]. A guidance that wraps another forwards this.
    fn flight_mode(&self) -> Option<FlightModeReport> {
        None
    }
}

/// Component responsible for flight control and stabilization: the inner loop.
///
/// The `Stabilizer` trait defines the interface for components that implement
/// control algorithms to track a [`Setpoint`] from [`Guidance`], producing the thrust and
/// moments that the [`Mixer`] turns into actuator commands.
///
/// Implementors of this trait may include:
/// - **PID controllers**: Simple feedback control for attitude stability
/// - **LQR controllers**: Optimal control for complex flight dynamics
/// - **Model predictive controllers**: Advanced control with future prediction
/// - **Cascaded control systems**: Hierarchical controllers for different aspects of flight
pub trait Stabilizer {
    /// Generates control outputs based on the current setpoint and vehicle state.
    ///
    /// # Parameters
    ///
    /// - `dt`: Time elapsed since the last control step in seconds. Must be positive and
    ///   finite; implementations may assert it (runners guarantee it).
    /// - `setpoint`: The attitude or rate target to track.
    /// - `state`: The current estimated state of the vehicle.
    ///
    /// # Returns
    ///
    /// A `ControlOutput` containing the desired thrust and control moments.
    ///
    /// # Unsupported setpoints
    ///
    /// [`Setpoint`] is `#[non_exhaustive]`. For a variant it can't fly, an implementation
    /// returns a neutral output — zero thrust and moments ([`ControlOutput::default`]) — and
    /// must not panic.
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput;

    /// Resets the stabilizer and primes it from a known-good vehicle state.
    ///
    /// Clears runtime state (integrators, derivative history) and warms any filters so
    /// control starts from where the vehicle is. [`FlightLoop`] calls it before the first
    /// tick and again each time the [`Command`] arms, after [`Guidance::prime`]. The default
    /// does nothing.
    fn prime(&mut self, _state: &VehicleState) {}

    /// What the last [`stabilize`](Self::stabilize) call clipped to a limit. [`FlightLoop`]
    /// merges it into the tick's [`TelemetryRecord::saturation`]. The default reports nothing.
    fn saturation(&self) -> Saturation {
        Saturation::default()
    }
}

/// Which limits the pipeline hit on one tick, as reported by [`Guidance::saturation`],
/// [`Stabilizer::saturation`], and [`Mixer::saturation`], merged by [`FlightLoop`] into
/// [`TelemetryRecord::saturation`].
///
/// A saturated tick is not an error: limits are how the pipeline stays safe. Flags that stay
/// set across many ticks point at tuning or authority problems (e.g. a rate loop pinned at
/// full moment, or a mixer giving up collective to hold attitude). New flags may be added, so
/// the struct is `#[non_exhaustive]`; build one with [`Saturation::default`] and set fields.
///
/// # Examples
///
/// ```
/// use stable_flight::Saturation;
///
/// let mut guidance = Saturation::default();
/// guidance.tilt = true;
/// let mut mixer = Saturation::default();
/// mixer.mixer_thrust = true;
///
/// let tick = guidance.merge(mixer);
/// assert!(tick.tilt && tick.mixer_thrust && tick.any());
/// assert!(!Saturation::default().any());
/// ```
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub struct Saturation {
    /// A requested tilt was capped: guidance's waypoint tilt limit, or the stabilizer's
    /// maximum roll/pitch angle.
    pub tilt: bool,
    /// Collective thrust was clipped to `[0, 1]` (the altitude loop, or the attitude-mode
    /// boost). The altitude loop's flag means its thrust sits on a bound, so it is also set
    /// when the tilt boost cuts thrust to 0 while inverted, and in a steady hover if
    /// `hover_thrust` is 1.
    pub thrust: bool,
    /// The rate loop's moment output hit ±1, per axis (roll, pitch, yaw).
    pub moments: [bool; 3],
    /// The mixer lowered collective thrust to keep the moments (a motor would pass full
    /// throttle).
    pub mixer_thrust: bool,
    /// The mixer couldn't deliver the moments in full: it scaled them down, its upward shift
    /// was limited near idle, or a motor was clipped. Expected while armed at idle on the
    /// ground, where any attitude error is clipped at zero thrust.
    pub mixer_moments: bool,
}

impl Saturation {
    /// The union of two reports: a flag set in either is set.
    pub fn merge(self, other: Self) -> Self {
        // Exhaustive destructuring: a new flag fails to compile here until it is merged.
        let Saturation {
            tilt,
            thrust,
            moments,
            mixer_thrust,
            mixer_moments,
        } = other;
        Saturation {
            tilt: self.tilt || tilt,
            thrust: self.thrust || thrust,
            moments: [
                self.moments[0] || moments[0],
                self.moments[1] || moments[1],
                self.moments[2] || moments[2],
            ],
            mixer_thrust: self.mixer_thrust || mixer_thrust,
            mixer_moments: self.mixer_moments || mixer_moments,
        }
    }

    /// True when any flag is set.
    pub fn any(&self) -> bool {
        // Exhaustive destructuring: a new flag fails to compile here until it is checked.
        let Saturation {
            tilt,
            thrust,
            moments,
            mixer_thrust,
            mixer_moments,
        } = *self;
        tilt || thrust || moments.iter().any(|&m| m) || mixer_thrust || mixer_moments
    }
}

/// A pipeline component whose gains and limits can be read and replaced between ticks.
///
/// The config is plain `Copy` data that the component owns as its single source of truth;
/// [`set_config`](Self::set_config) is the one write point and validates before committing,
/// so a running loop can be retuned without ever flying an invalid config. Implemented by
/// [`CopterGuidance`](guidance::copter::CopterGuidance) and
/// [`CopterStabilizer`](stabilizers::copter::CopterStabilizer); [`FlightLoop`] reaches them
/// through [`set_guidance_config`](FlightLoop::set_guidance_config) and
/// [`set_stabilizer_config`](FlightLoop::set_stabilizer_config).
///
/// # Examples
///
/// ```
/// use stable_flight::Configurable;
/// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
///
/// let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
///
/// let mut retuned = *stabilizer.config();
/// retuned.body_rate.roll_gains.p = 0.2;
/// assert!(stabilizer.set_config(retuned).is_ok());
///
/// retuned.body_rate.max_integral = -1.0;
/// assert!(stabilizer.set_config(retuned).is_err());
/// assert_eq!(stabilizer.config().body_rate.roll_gains.p, 0.2); // unchanged
/// ```
pub trait Configurable {
    /// The component's config.
    type Config: Copy;

    /// The config the component flies.
    fn config(&self) -> &Self::Config;

    /// Replaces the config if it is valid; otherwise keeps the current one and returns the
    /// error. Call it between ticks: the new config applies from the next tick, and runtime
    /// state (integrals, derivative history, timers) is kept. A changed integral gain rescales
    /// its integral so the I term carries over without a bump; across a zero gain the
    /// integral resets (`architecture.md#config-ownership`).
    fn set_config(&mut self, config: Self::Config) -> Result<(), controllers::InvalidConfig>;
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
    /// Publishes one control-loop tick to external systems.
    ///
    /// Fire-and-forget: the flight loop cannot act on a telemetry failure, so implementations
    /// handle (log) their own errors instead of returning them.
    ///
    /// # Parameters
    ///
    /// - `record`: Everything the loop produced this tick; see [`TelemetryRecord`].
    fn publish(&mut self, record: &TelemetryRecord);
}

/// One control-loop tick, as handed to a [`TelemetryPublisher`].
///
/// The loop assembles it after actuating, so every stage's output is available. Fields borrow
/// from the loop and live only for the tick; publishers copy what they need to keep. New
/// fields can be added without changing the `publish` signature (struct literals, as in tests,
/// need the new field; `testing::RecordedTick::as_record`, behind the `testing` feature,
/// builds one from an owned copy).
///
/// # Examples
///
/// ```
/// use std::time::Duration;
/// use stable_flight::{
///     ActuatorCommand, Command, ControlOutput, FlightModeReport, Saturation, Setpoint,
///     TelemetryPublisher, TelemetryRecord, VehicleState,
/// };
///
/// struct ThrustLogger;
///
/// impl TelemetryPublisher for ThrustLogger {
///     fn publish(&mut self, record: &TelemetryRecord) {
///         println!("{:?}: thrust {}", record.dt, record.control.thrust);
///     }
/// }
///
/// let command = Command::default();
/// let setpoint = Setpoint::Attitude(Default::default());
/// let state = VehicleState::default();
/// let control = ControlOutput::default();
/// let actuators = ActuatorCommand::default();
///
/// ThrustLogger.publish(&TelemetryRecord {
///     dt: Duration::from_millis(4),
///     command: &command,
///     setpoint: &setpoint,
///     state: &state,
///     control: &control,
///     actuators: &actuators,
///     saturation: Saturation::default(),
///     failsafe: None,
///     flight_mode: FlightModeReport::default(),
/// });
/// ```
#[derive(Debug, Clone, Copy)]
pub struct TelemetryRecord<'a> {
    /// Nominal loop period (`1 / target frequency`), not measured elapsed time.
    pub dt: Duration,
    /// Command the commander produced.
    pub command: &'a Command,
    /// Setpoint guidance derived from `command` (neutral while disarmed).
    pub setpoint: &'a Setpoint,
    /// Estimated vehicle state the stabilizer acted on.
    pub state: &'a VehicleState,
    /// Stabilizer output (thrust and moments; zero while disarmed).
    pub control: &'a ControlOutput,
    /// Mixer output sent to the actuators ([`ActuatorCommand::default`] while disarmed).
    pub actuators: &'a ActuatorCommand,
    /// Limits guidance, the stabilizer, and the mixer hit this tick, merged
    /// ([`Saturation::default`] while disarmed).
    pub saturation: Saturation,
    /// Which commander a failsafe let fly this tick ([`Commander::failsafe`]); `None` without
    /// a failsafe.
    pub failsafe: Option<FailsafeState>,
    /// The flight mode guidance flew this tick ([`Guidance::flight_mode`]), and why it isn't
    /// the requested [`Command::regime`] when it can't be. A disarmed tick, where guidance
    /// doesn't run, records [`FlightModeReport::copter_only`].
    pub flight_mode: FlightModeReport,
}

/// One tick's actuator outputs: motors and servos, as generic slots.
///
/// The [`Mixer`] decides what each slot does
/// ([`TableMixer::quad_x`](crate::mixers::TableMixer::quad_x)'s motor `0` is front-left; a
/// tricopter's servo `0` might be its tail tilt); the backend's mapping decides which output
/// channel each slot drives. Slots a mixer doesn't use stay at their default.
///
/// The default (all zeros: motors off, servos centered) is what [`FlightLoop`] actuates while
/// disarmed.
///
/// # Examples
///
/// A quadcopter uses the first four motors:
/// ```
/// use stable_flight::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.motors[..4].copy_from_slice(&[0.5; 4]); // Four motors at 50% power
/// ```
///
/// A tricopter adds a tail servo for yaw:
/// ```
/// use stable_flight::ActuatorCommand;
///
/// let mut cmd = ActuatorCommand::default();
/// cmd.motors[..3].copy_from_slice(&[0.5; 3]);
/// cmd.servos[0] = 0.2; // Tail tilted for a slight yaw
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ActuatorCommand {
    /// Motor outputs, from 0.0 (off) to 1.0 (full power). The mixer sets which physical motor
    /// each index is.
    pub motors: [f32; ActuatorCommand::SLOTS],

    /// Servo outputs, from -1.0 to 1.0, with 0.0 centered. The mixer sets what each index moves
    /// (a control surface, a tilt servo).
    pub servos: [f32; ActuatorCommand::SLOTS],
}

impl ActuatorCommand {
    /// Slots per kind: `motors` and `servos` each hold this many.
    pub const SLOTS: usize = 8;
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
/// - **Fixed-wing mixers**: Map controls to a throttle motor and control-surface servos
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
    /// vehicle's physical configuration. The mixer assigns each `motors` and `servos` slot its
    /// meaning; slots it doesn't use stay at their default:
    ///
    /// * For multicopters: the `motors` array
    /// * For fixed-wing: a throttle motor, with control surfaces on `servos`
    /// * For tilting or hybrid vehicles: both
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{ControlOutput, Mixer, Vec3};
    /// use stable_flight::mixers::TableMixer;
    ///
    /// let mut mixer = TableMixer::quad_x();
    ///
    /// // Create control output with 70% thrust and slight roll right
    /// let control_output = ControlOutput::new(
    ///     0.7,                    // 70% thrust
    ///     Vec3::new(0.1, 0.0, 0.0)     // slight roll right, no pitch, no yaw
    /// );
    ///
    /// // Convert abstract controls into specific motor commands
    /// let actuator_command = mixer.mix(&control_output);
    ///
    /// // Now actuator_command contains specific motor values for a quadcopter
    /// ```
    fn mix(&mut self, controls: &ControlOutput) -> ActuatorCommand;

    /// What the last [`mix`](Self::mix) call couldn't deliver in full. [`FlightLoop`] merges
    /// it into the tick's [`TelemetryRecord::saturation`]. The default reports nothing.
    fn saturation(&self) -> Saturation {
        Saturation::default()
    }
}
