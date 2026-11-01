//! A [`Commander`] that replays a sequence of waypoints loaded from a CSV file.
//!
//! [`ScriptCommander`] is a CSV front end for the core [`WaypointFollower`]: this module
//! turns text into data (`parse_waypoints`) and handles file I/O ([`ScriptCommander::new`],
//! [`load_waypoints`]); following the waypoints, and looping or holding at the end, is the
//! follower's job. Neither
//! knows how the path was shaped (e.g. a figure-eight); the geometry lives in the CSV.
//!
//! # CSV format
//!
//! One waypoint per line, four comma-separated fields: `x,y,altitude,cruise_speed`
//! (meters and meters-per-second), then up to four optional ones, each left empty for its
//! default:
//!
//! - `regime`: `copter` (the default) or `plane`, the [`FlightRegime`] to fly the leg *to*
//!   that waypoint in (a VTOL transitions as the leg begins; a copter refuses `plane`). A
//!   route that holds on a final `plane` waypoint (`RouteEnd::Hold`) loiters around it; one
//!   that lands transitions back first, since a land asks for copter.
//! - `heading`: degrees from north to face while flying to and holding the waypoint
//!   ([`WaypointCommand::heading_rad`]); empty leaves the heading to guidance.
//! - `hold_s`: seconds to keep flying to the waypoint once reached, before the next one (or
//!   the route's land); empty or `0` moves on at once.
//! - `maneuver`: a [`Maneuver`] flown on reaching the waypoint, before its hold and the next
//!   leg, on `plane` rows only; `name:argument[:argument]`:
//!
//!   | Field | Maneuver |
//!   |---|---|
//!   | `roll:<deg/s>` | [`Maneuver::roll`] at that rate (negative rolls left) |
//!   | `loop:<m>` | [`Maneuver::loop_`] of that radius (negative: an outside loop) |
//!   | `doublet_roll:<deg/s>[:<s>]`, `doublet_pitch:…`, `doublet_yaw:…` | [`Maneuver::doublet`] about that axis at that rate, each half that long ([`DEFAULT_DOUBLET_HALF_S`] when absent) |
//!
//!   The maneuver flies at [`ManeuverConfig`]'s thrust
//!   ([`ScriptCommander::with_maneuver_config`]); the route resumes once it has recovered.
//!
//! Positions are meters north (`x`) and east (`y`) and altitude up, in the vehicle's frame, or
//! of where the vehicle starts once [`ScriptCommander::anchored`].
//!
//! Blank lines and lines starting with `#` are ignored, and an optional non-numeric header row
//! is skipped.
//!
//! ```text
//! # figure-eight, medium/park envelope
//! x,y,altitude,cruise_speed
//! 30.00,0.00,27.50,7.0
//! 0.00,0.00,40.00,7.0
//! # face east over the start for 20 s
//! 0.00,0.00,40.00,7.0,,90,20
//! # on a plane leg, a loop of 25 m at this waypoint
//! 150.00,0.00,60.00,14.0,plane,,,loop:25
//! ```

use std::fs;

use stable_flight::{Vec2, Vec3};
use thiserror::Error;

use stable_flight::commanders::maneuver::{Axis, DEFAULT_DOUBLET_HALF_S, Maneuver, ManeuverConfig};
use stable_flight::commanders::waypoint::{
    DEFAULT_ALTITUDE_ACCEPTANCE_M, PLANE_ALTITUDE_ACCEPTANCE_M,
};
use stable_flight::commanders::{EmptyWaypoints, RouteEnd, WaypointFollower};
use stable_flight::{
    Command, Commander, ControlInput, FlightRegime, LandCommand, VehicleState, WaypointCommand,
};

mod actions;

use actions::{Actions, RowActions, Step};

/// Comma-separated fields on a waypoint row: four numbers, then an optional regime, heading,
/// hold, and maneuver.
const NUMERIC_FIELDS: usize = 4;
const REGIME_FIELD: usize = NUMERIC_FIELDS;
const HEADING_FIELD: usize = 5;
const HOLD_FIELD: usize = 6;
const MANEUVER_FIELD: usize = 7;
const MAX_FIELDS: usize = MANEUVER_FIELD + 1;

/// Errors that can occur while loading or parsing a waypoint script.
#[derive(Debug, Error)]
pub enum ScriptError {
    /// The script file could not be read from disk.
    #[error("failed to read script file {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },

    /// The script contained no waypoints.
    #[error("script contained no waypoints")]
    Empty,

    /// A waypoint row could not be parsed.
    #[error("{path}:{line}: {message}")]
    Parse {
        path: String,
        line: usize,
        message: String,
    },
}

impl From<EmptyWaypoints> for ScriptError {
    fn from(_: EmptyWaypoints) -> Self {
        ScriptError::Empty
    }
}

/// A [`Commander`] that flies a sequence of waypoints loaded from a CSV file.
///
/// A thin wrapper over the core [`WaypointFollower`] (default acceptance radius
/// [`DEFAULT_ACCEPTANCE_RADIUS_M`](stable_flight::commanders::waypoint::DEFAULT_ACCEPTANCE_RADIUS_M),
/// default [`RouteEnd::Loop`]), with the altitude acceptance of each leg's regime
/// ([`with_altitude_acceptance`](Self::with_altitude_acceptance)), and a script's actions,
/// flown once the follower has reached a waypoint while it waits on the next one: its
/// maneuver, then its hold (the waypoint kept commanded, without a fly-through velocity, for
/// its `hold_s`).
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl::commanders::ScriptCommander;
///
/// let commander = ScriptCommander::new("flights/figure_eight.csv")
///     .expect("failed to load script");
/// ```
pub struct ScriptCommander {
    follower: WaypointFollower<Vec<WaypointCommand>>,
    /// The regime of the leg to each waypoint, by index.
    regimes: Vec<FlightRegime>,
    /// Altitude acceptance on copter legs, in meters.
    copter_altitude_acceptance_m: f32,
    /// Altitude acceptance on plane legs, in meters.
    plane_altitude_acceptance_m: f32,
    actions: Actions,
    origin: Origin,
}

/// Where a script's coordinates are measured from.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Origin {
    /// The vehicle's frame, as is.
    Vehicle,
    /// Where the vehicle is on the first tick with a finite position; not seen yet.
    Start,
    /// Anchored here, in the vehicle's frame (m: north, east, up).
    At(Vec3),
}

/// A script row: a waypoint, the regime of the leg to it, and its actions.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Leg {
    waypoint: WaypointCommand,
    regime: FlightRegime,
    actions: RowActions,
}

impl ScriptCommander {
    /// Loads a waypoint script from a CSV file at `path`.
    ///
    /// # Errors
    ///
    /// Returns [`ScriptError::Io`] if the file cannot be read, [`ScriptError::Parse`] if a
    /// row is malformed, or [`ScriptError::Empty`] if the file contains no waypoints.
    pub fn new(path: &str) -> Result<Self, ScriptError> {
        Self::from_legs(load_route(path)?)
    }

    /// Builds a commander from an already-constructed list of waypoints.
    ///
    /// This is the composition seam for any waypoint source — the CSV loader, a
    /// programmatic path generator, or tests.
    ///
    /// # Errors
    ///
    /// Returns [`ScriptError::Empty`] if `waypoints` is empty.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::Vec2;
    /// use stable_flight::WaypointCommand;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let waypoints = vec![WaypointCommand {
    ///     position_m: Vec2::zeros(),
    ///     ..Default::default()
    /// }];
    /// let commander = ScriptCommander::from_waypoints(waypoints).expect("non-empty");
    /// ```
    pub fn from_waypoints(waypoints: Vec<WaypointCommand>) -> Result<Self, ScriptError> {
        Self::from_route(
            waypoints
                .into_iter()
                .map(|waypoint| (waypoint, FlightRegime::Copter))
                .collect(),
        )
    }

    /// Builds a commander from waypoints, each with the regime of the leg to it.
    ///
    /// # Errors
    ///
    /// Returns [`ScriptError::Empty`] if `route` is empty.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{FlightRegime, Vec2, WaypointCommand};
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let at = |north: f32| WaypointCommand { position_m: Vec2::new(north, 0.0), ..Default::default() };
    /// let commander = ScriptCommander::from_route(vec![
    ///     (at(0.0), FlightRegime::Copter),
    ///     (at(200.0), FlightRegime::Plane),
    /// ])
    /// .expect("non-empty");
    /// ```
    pub fn from_route(route: Vec<(WaypointCommand, FlightRegime)>) -> Result<Self, ScriptError> {
        Self::from_legs(
            route
                .into_iter()
                .map(|(waypoint, regime)| Leg {
                    waypoint,
                    regime,
                    actions: RowActions::default(),
                })
                .collect(),
        )
    }

    /// Builds a commander from script rows.
    fn from_legs(legs: Vec<Leg>) -> Result<Self, ScriptError> {
        Ok(ScriptCommander {
            follower: WaypointFollower::new(legs.iter().map(|leg| leg.waypoint).collect())?,
            regimes: legs.iter().map(|leg| leg.regime).collect(),
            copter_altitude_acceptance_m: DEFAULT_ALTITUDE_ACCEPTANCE_M,
            plane_altitude_acceptance_m: PLANE_ALTITUDE_ACCEPTANCE_M,
            actions: Actions::new(
                legs.iter().map(|leg| leg.actions).collect(),
                ManeuverConfig::default(),
            ),
            origin: Origin::Vehicle,
        })
    }

    /// The underlying follower, e.g. for [`WaypointFollower::current_index`] or
    /// [`WaypointFollower::waypoints`].
    pub fn follower(&self) -> &WaypointFollower<Vec<WaypointCommand>> {
        &self.follower
    }

    /// Overrides the horizontal acceptance radius (meters) used to decide when the active
    /// waypoint has been reached. `radius_m` must be non-negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::Vec2;
    /// use stable_flight::WaypointCommand;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position_m: Vec2::zeros(),
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .with_acceptance_radius(5.0);
    /// ```
    pub fn with_acceptance_radius(self, radius_m: f32) -> Self {
        ScriptCommander {
            follower: self.follower.with_acceptance_radius(radius_m),
            ..self
        }
    }

    /// Overrides the altitude acceptance (meters, either way) a waypoint on a `regime` leg also
    /// needs to be reached (defaults [`DEFAULT_ALTITUDE_ACCEPTANCE_M`] for copter legs,
    /// [`PLANE_ALTITUDE_ACCEPTANCE_M`] for plane legs; any other regime is rotor-borne and
    /// shares the copter band). A plane leg ending in a back transition takes its waypoint at
    /// any altitude. `altitude_m` must be non-negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{FlightRegime, Vec2, WaypointCommand};
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position_m: Vec2::zeros(),
    ///     altitude_m: 5.0,
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .with_altitude_acceptance(FlightRegime::Copter, 0.5);
    /// ```
    pub fn with_altitude_acceptance(mut self, regime: FlightRegime, altitude_m: f32) -> Self {
        match regime {
            FlightRegime::Plane => self.plane_altitude_acceptance_m = altitude_m,
            _ => self.copter_altitude_acceptance_m = altitude_m,
        }
        self
    }

    /// Flies the script relative to where the vehicle is on its first tick with a finite
    /// position: `x`, `y`, and altitude become meters north, east, and up of that point, so
    /// one script flies the same route from any spawn point or field. Height 0 is the
    /// vehicle's own position on the ground (its reference point's height above the surface),
    /// not the surface. Build it before arming, with the vehicle on the ground: it anchors
    /// wherever the vehicle is. Until a tick brings a finite position, the commander holds the
    /// motors off. The anchor never moves afterwards, even if the backend relocates the vehicle.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::{Commander, ControlInput, Vec2, Vec3, VehicleState, WaypointCommand};
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let mut commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position_m: Vec2::new(10.0, 0.0),
    ///     altitude_m: 5.0,
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .anchored();
    ///
    /// // On the ground 1186 m up, at north 100 m: the waypoint is 10 m north of there, 5 m up.
    /// let spawn = VehicleState { position_m: Vec3::new(100.0, 0.0, 1186.0), ..Default::default() };
    /// let ControlInput::Waypoint(target) = commander.next_command(0.004, &spawn).input else {
    ///     unreachable!("a waypoint script flies waypoints");
    /// };
    /// assert_eq!(target.position_m, Vec2::new(110.0, 0.0));
    /// assert_eq!(target.altitude_m, 1191.0);
    /// ```
    pub fn anchored(self) -> Self {
        ScriptCommander {
            origin: Origin::Start,
            ..self
        }
    }

    /// The point the script's coordinates are measured from, in the vehicle's frame (m:
    /// north, east, up): zero unless [`anchored`](Self::anchored), and until it has anchored
    /// (an anchor at zero reads the same).
    /// Add it to a [`follower`](Self::follower) waypoint to compare it with the vehicle's state.
    pub fn origin_m(&self) -> Vec3 {
        match self.origin {
            Origin::At(origin_m) => origin_m,
            Origin::Vehicle | Origin::Start => Vec3::zeros(),
        }
    }

    /// Sets what happens after the last waypoint: loop the script (the default) or hold the
    /// last waypoint ([`RouteEnd`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::Vec2;
    /// use stable_flight::WaypointCommand;
    /// use stable_flight::commanders::RouteEnd;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position_m: Vec2::zeros(),
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .with_route_end(RouteEnd::Hold);
    /// assert!(!commander.follower().is_complete());
    /// ```
    pub fn with_route_end(self, route_end: RouteEnd) -> Self {
        ScriptCommander {
            follower: self.follower.with_route_end(route_end),
            ..self
        }
    }

    /// Sets how the script's maneuvers fly (default [`ManeuverConfig::default`]): their thrust,
    /// timeout, path hold, and recovery, usually the airframe's.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ManeuverConfig::validate`]).
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::Vec2;
    /// use stable_flight::WaypointCommand;
    /// use stable_flight::commanders::ManeuverConfig;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position_m: Vec2::zeros(),
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .with_maneuver_config(ManeuverConfig {
    ///     thrust: 0.9,
    ///     ..Default::default()
    /// });
    /// ```
    pub fn with_maneuver_config(mut self, config: ManeuverConfig) -> Self {
        self.actions.set_maneuver_config(config);
        self
    }

    /// The altitude acceptance of a `regime` leg: plane legs have their own, and any other
    /// regime is rotor-borne and shares the copter one.
    fn altitude_acceptance_m(&self, regime: FlightRegime) -> f32 {
        match regime {
            FlightRegime::Plane => self.plane_altitude_acceptance_m,
            _ => self.copter_altitude_acceptance_m,
        }
    }

    /// The altitude acceptance of the leg to the active waypoint, flown in `regime`: the
    /// regime's, except that a plane leg ending in a back transition (the next leg rotor-borne, or the route's land)
    /// takes its waypoint at any altitude, so the back transition starts there instead of
    /// waiting on a loiter for an altitude it doesn't need. PX4 accepts that waypoint
    /// regardless of altitude too (`MissionBlock::is_mission_item_reached_or_completed`).
    fn active_altitude_acceptance_m(&self, regime: FlightRegime) -> f32 {
        let back_transition =
            regime == FlightRegime::Plane && self.next_regime() != FlightRegime::Plane;
        if back_transition {
            return f32::INFINITY;
        }
        self.altitude_acceptance_m(regime)
    }

    /// The regime flown after the active waypoint: the next leg's, the first leg's on a
    /// looping route's wrap, copter for the route's land, and the active leg's on its hold.
    fn next_regime(&self) -> FlightRegime {
        let current = self.follower.current_index();
        if current + 1 < self.follower.waypoints().len() {
            return self.regime(current + 1);
        }
        match self.follower.route_end() {
            RouteEnd::Loop => self.regime(0),
            RouteEnd::Land => FlightRegime::Copter,
            RouteEnd::Hold => self.regime(current),
        }
    }

    /// The regime of the leg to the active waypoint.
    fn active_regime(&self) -> FlightRegime {
        self.regime(self.follower.current_index())
    }

    /// The regime of the leg to waypoint `index`.
    fn regime(&self, index: usize) -> FlightRegime {
        self.regimes.get(index).copied().unwrap_or_default()
    }

    /// The command for an action's tick.
    fn step_command(&self, step: Step) -> Command {
        match step {
            Step::Maneuver { index, input } => {
                Command::armed(input).with_regime(self.regime(index))
            }
            Step::Hold { index } => self.hold_command(index),
        }
    }

    /// Waypoint `index` as the follower would fly it with no leg ahead: without a fly-through
    /// velocity unless it has its own, so the vehicle stops on it.
    fn hold_command(&self, index: usize) -> Command {
        let waypoint = self.follower.waypoints()[index];
        Command::armed(ControlInput::Waypoint(waypoint)).with_regime(self.regime(index))
    }
}

/// Loads and parses a waypoint script from the CSV file at `path`.
///
/// This is the loading seam, separate from sequencing: it turns a file into a
/// `Vec<WaypointCommand>` without constructing a [`ScriptCommander`]. Use it when a consumer
/// needs the raw path but not flight sequencing — for example the `plot_flight` binary, which
/// renders these points as a 3D path. [`ScriptCommander::new`] is built on top of it.
///
/// # Errors
///
/// Returns [`ScriptError::Io`] if the file cannot be read, or [`ScriptError::Parse`] if a row
/// is malformed. An empty file yields an empty vector (the "no waypoints" check lives in
/// [`ScriptCommander::from_waypoints`], which is what requires a flyable path).
///
/// # Examples
///
/// ```no_run
/// use stable_flight_sitl::commanders::script::load_waypoints;
///
/// let waypoints = load_waypoints("flights/figure_eight.csv").expect("failed to load script");
/// println!("{} waypoints", waypoints.len());
/// ```
pub fn load_waypoints(path: &str) -> Result<Vec<WaypointCommand>, ScriptError> {
    Ok(load_route(path)?
        .into_iter()
        .map(|leg| leg.waypoint)
        .collect())
}

/// Loads a script's rows.
fn load_route(path: &str) -> Result<Vec<Leg>, ScriptError> {
    let contents = fs::read_to_string(path).map_err(|source| ScriptError::Io {
        path: path.to_string(),
        source,
    })?;
    parse_route(path, &contents)
}

impl Commander for ScriptCommander {
    /// Delegates to the core [`WaypointFollower`] (advances within the acceptance radius or
    /// once past a waypoint along its leg, and within the altitude acceptance of the active
    /// leg's regime, any altitude on a plane leg ending in a back transition; a plane leg
    /// waits on its waypoint, loitering, for the altitude; then loops, holds, or lands per its
    /// [`RouteEnd`]), asking for the regime of the leg to the current waypoint. Once the follower moves on from a waypoint (to the
    /// next waypoint or the route's land), its actions fly instead, the follower paused
    /// meanwhile: its maneuver, in its leg's regime, then its hold for `hold_s`.
    ///
    /// An [`anchored`](Self::anchored) script sees the state relative to its origin and flies
    /// its waypoints and lands shifted back by it.
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command {
        match self.origin {
            Origin::Vehicle => self.fly(dt, current_state),
            Origin::Start => {
                let position_m = current_state.position_m;
                if !position_m.iter().all(|value| value.is_finite()) {
                    return Command::disarmed(ControlInput::default());
                }
                self.origin = Origin::At(position_m);
                self.fly_from(position_m, dt, current_state)
            }
            Origin::At(origin_m) => self.fly_from(origin_m, dt, current_state),
        }
    }
}

impl ScriptCommander {
    /// The script's command with the state seen from `origin_m`, and its positions shifted
    /// back into the vehicle's frame.
    fn fly_from(&mut self, origin_m: Vec3, dt: f32, current_state: &VehicleState) -> Command {
        let local = VehicleState {
            position_m: current_state.position_m - origin_m,
            ..current_state.clone()
        };
        let mut command = self.fly(dt, &local);
        command.input = shifted(command.input, origin_m);
        command
    }

    /// The script's command for `current_state`, in the script's coordinates.
    fn fly(&mut self, dt: f32, current_state: &VehicleState) -> Command {
        if let Some(step) = self.actions.step(dt, current_state) {
            return self.step_command(step);
        }
        let reached = self.follower.current_index();
        let regime = self.active_regime();
        let altitude_acceptance_m = self.active_altitude_acceptance_m(regime);
        self.follower.set_altitude_acceptance(altitude_acceptance_m);
        self.follower
            .set_wait_for_altitude(regime == FlightRegime::Plane);
        let command = self.follower.next_command(dt, current_state);
        let lands = matches!(command.input, ControlInput::Land(_));
        let current = self.follower.current_index();
        if let Some(step) = self
            .actions
            .start(reached, current, lands, dt, current_state)
        {
            return self.step_command(step);
        }
        command.with_regime(self.active_regime())
    }
}

/// `input` with its positions moved by `by_m` (m: north, east, up); inputs without one pass
/// through.
fn shifted(input: ControlInput, by_m: Vec3) -> ControlInput {
    match input {
        ControlInput::Waypoint(waypoint) => ControlInput::Waypoint(WaypointCommand {
            position_m: waypoint.position_m + by_m.xy(),
            altitude_m: waypoint.altitude_m + by_m.z,
            ..waypoint
        }),
        ControlInput::Land(land) => ControlInput::Land(LandCommand {
            position_m: land.position_m + by_m.xy(),
        }),
        other => other,
    }
}

/// Parses script rows from CSV `contents`. `path` is used only to label parse errors.
///
/// Blank lines and `#` comments are skipped, as is a leading non-numeric header row. Each
/// remaining line must hold four comma-separated, finite floats
/// (`x,y,altitude,cruise_speed`; non-finite values are rejected), then optionally a regime,
/// `copter` or `plane` (any case; `copter` when absent or empty), a finite heading in degrees
/// (none when absent or empty), a finite, non-negative hold in seconds (0 when absent or
/// empty), and a maneuver (none when absent or empty; `plane` rows only).
fn parse_route(path: &str, contents: &str) -> Result<Vec<Leg>, ScriptError> {
    let mut waypoints = Vec::new();

    for (index, raw) in contents.lines().enumerate() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let fields: Vec<&str> = line.split(',').map(str::trim).collect();

        // A leading row whose first field is not numeric is treated as a header.
        let is_header = waypoints.is_empty()
            && fields
                .first()
                .is_some_and(|field| field.parse::<f32>().is_err());
        if is_header {
            continue;
        }

        let line_number = index + 1;
        if !(NUMERIC_FIELDS..=MAX_FIELDS).contains(&fields.len()) {
            return Err(ScriptError::Parse {
                path: path.to_string(),
                line: line_number,
                message: format!(
                    "expected {NUMERIC_FIELDS} to {MAX_FIELDS} fields, found {}",
                    fields.len()
                ),
            });
        }

        let error = |message: String| ScriptError::Parse {
            path: path.to_string(),
            line: line_number,
            message,
        };
        let parse = |field: &str, name: &str| parse_number(field, name).map_err(error);

        // An optional field: `None` when absent or empty.
        let optional = |index: usize| fields.get(index).copied().filter(|f| !f.is_empty());
        let regime = match optional(REGIME_FIELD).map(str::to_ascii_lowercase) {
            None => FlightRegime::Copter,
            Some(regime) if regime == "copter" => FlightRegime::Copter,
            Some(regime) if regime == "plane" => FlightRegime::Plane,
            Some(other) => {
                return Err(ScriptError::Parse {
                    path: path.to_string(),
                    line: line_number,
                    message: format!("invalid regime {other:?}: expected copter or plane"),
                });
            }
        };
        let heading_rad = optional(HEADING_FIELD)
            .map(|field| parse(field, "heading").map(f32::to_radians))
            .transpose()?;
        let hold_s = optional(HOLD_FIELD).map_or(Ok(0.0), |field| parse(field, "hold_s"))?;
        if hold_s < 0.0 {
            return Err(ScriptError::Parse {
                path: path.to_string(),
                line: line_number,
                message: format!("negative hold_s: {hold_s}"),
            });
        }
        let maneuver = optional(MANEUVER_FIELD)
            .map(parse_maneuver)
            .transpose()
            .map_err(error)?;
        if maneuver.is_some() && regime != FlightRegime::Plane {
            return Err(error("a maneuver needs a plane row".to_string()));
        }
        let waypoint = WaypointCommand {
            position_m: Vec2::new(parse(fields[0], "x")?, parse(fields[1], "y")?),
            velocity_mps: None,
            altitude_m: parse(fields[2], "altitude")?,
            cruise_speed_mps: parse(fields[3], "cruise_speed")?,
            heading_rad,
        };
        waypoints.push(Leg {
            waypoint,
            regime,
            actions: RowActions { maneuver, hold_s },
        });
    }

    Ok(waypoints)
}

/// A finite number from `field`, else what's wrong with it, labeled `name`. A non-finite
/// coordinate would stall waypoint advancement and feed a bad position straight into guidance.
fn parse_number(field: &str, name: &str) -> Result<f32, String> {
    let value = field
        .parse::<f32>()
        .map_err(|err| format!("invalid {name}: {err}"))?;
    if !value.is_finite() {
        return Err(format!("non-finite {name}: {field}"));
    }
    Ok(value)
}

/// The maneuver named by a script's `maneuver` field (`name:argument[:argument]`), else what's
/// wrong with it.
fn parse_maneuver(field: &str) -> Result<Maneuver, String> {
    let mut parts = field.split(':').map(str::trim);
    let name = parts.next().unwrap_or_default().to_ascii_lowercase();
    let arguments: Vec<&str> = parts.collect();
    let takes = |least: usize, most: usize, usage: &str| {
        if (least..=most).contains(&arguments.len()) {
            Ok(())
        } else {
            Err(format!("maneuver {name} takes {usage}"))
        }
    };
    let maneuver = match name.as_str() {
        "roll" => {
            takes(1, 1, "a rate in deg/s")?;
            Maneuver::roll(parse_number(arguments[0], "roll rate")?.to_radians())
        }
        "loop" => {
            takes(1, 1, "a radius in m")?;
            Maneuver::loop_(parse_number(arguments[0], "loop radius")?)
        }
        "doublet_roll" | "doublet_pitch" | "doublet_yaw" => {
            takes(1, 2, "a rate in deg/s and an optional half width in s")?;
            let axis = match name.as_str() {
                "doublet_roll" => Axis::Roll,
                "doublet_pitch" => Axis::Pitch,
                _ => Axis::Yaw,
            };
            let rate_radps = parse_number(arguments[0], "doublet rate")?.to_radians();
            let half_s = arguments
                .get(1)
                .map_or(Ok(DEFAULT_DOUBLET_HALF_S), |field| {
                    parse_number(field, "doublet half width")
                })?;
            Maneuver::doublet(axis, rate_radps, half_s)
        }
        other => {
            return Err(format!(
                "unknown maneuver {other:?}: expected roll, loop, doublet_roll, doublet_pitch, \
                 or doublet_yaw"
            ));
        }
    };
    maneuver.map_err(|error| format!("invalid maneuver {field:?}: {error}"))
}

#[cfg(test)]
mod tests;
