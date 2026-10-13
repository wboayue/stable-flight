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
//! (meters and meters-per-second), and an optional fifth, `regime`: `copter` (the default) or
//! `plane`, the [`FlightRegime`] to fly the leg *to* that waypoint in (a VTOL transitions as
//! the leg begins; a copter refuses `plane`). A route that holds on a final `plane` waypoint
//! (`RouteEnd::Hold`) loiters around it; one that lands transitions back first, since a land
//! asks for copter. Blank lines and lines starting with `#` are ignored, and an optional
//! non-numeric header row is skipped.
//!
//! ```text
//! # figure-eight, medium/park envelope
//! x,y,altitude,cruise_speed
//! 30.00,0.00,27.50,7.0
//! 0.00,0.00,40.00,7.0
//! ```

use std::fs;

use stable_flight::Vec2;
use thiserror::Error;

use stable_flight::commanders::waypoint::{
    DEFAULT_ALTITUDE_ACCEPTANCE_M, PLANE_ALTITUDE_ACCEPTANCE_M,
};
use stable_flight::commanders::{EmptyWaypoints, RouteEnd, WaypointFollower};
use stable_flight::{Command, Commander, FlightRegime, VehicleState, WaypointCommand};

/// Comma-separated fields on a waypoint row: four numbers, then an optional regime.
const NUMERIC_FIELDS: usize = 4;
const MAX_FIELDS: usize = NUMERIC_FIELDS + 1;

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
/// ([`with_altitude_acceptance`](Self::with_altitude_acceptance)).
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
}

impl ScriptCommander {
    /// Loads a waypoint script from a CSV file at `path`.
    ///
    /// # Errors
    ///
    /// Returns [`ScriptError::Io`] if the file cannot be read, [`ScriptError::Parse`] if a
    /// row is malformed, or [`ScriptError::Empty`] if the file contains no waypoints.
    pub fn new(path: &str) -> Result<Self, ScriptError> {
        Self::from_route(load_route(path)?)
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
        let (waypoints, regimes) = route.into_iter().unzip();
        Ok(ScriptCommander {
            follower: WaypointFollower::new(waypoints)?,
            regimes,
            copter_altitude_acceptance_m: DEFAULT_ALTITUDE_ACCEPTANCE_M,
            plane_altitude_acceptance_m: PLANE_ALTITUDE_ACCEPTANCE_M,
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
    /// shares the copter band). `altitude_m` must be non-negative.
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
        *self.altitude_acceptance_m(regime) = altitude_m;
        self
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

    /// The altitude acceptance of a `regime` leg: plane legs have their own, and any other
    /// regime is rotor-borne and shares the copter one.
    fn altitude_acceptance_m(&mut self, regime: FlightRegime) -> &mut f32 {
        match regime {
            FlightRegime::Plane => &mut self.plane_altitude_acceptance_m,
            _ => &mut self.copter_altitude_acceptance_m,
        }
    }

    /// The regime of the leg to the active waypoint.
    fn active_regime(&self) -> FlightRegime {
        self.regimes
            .get(self.follower.current_index())
            .copied()
            .unwrap_or_default()
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
        .map(|(waypoint, _)| waypoint)
        .collect())
}

/// Loads a script's waypoints with the regime of the leg to each.
fn load_route(path: &str) -> Result<Vec<(WaypointCommand, FlightRegime)>, ScriptError> {
    let contents = fs::read_to_string(path).map_err(|source| ScriptError::Io {
        path: path.to_string(),
        source,
    })?;
    parse_route(path, &contents)
}

impl Commander for ScriptCommander {
    /// Delegates to the core [`WaypointFollower`] (advances within the acceptance radius or
    /// once past a waypoint along its leg, and within the altitude acceptance of the active
    /// leg's regime; then loops, holds, or lands per its [`RouteEnd`]), asking for the regime
    /// of the leg to the current waypoint.
    fn next_command(&mut self, dt: f32, current_state: &VehicleState) -> Command {
        let altitude_acceptance_m = *self.altitude_acceptance_m(self.active_regime());
        self.follower.set_altitude_acceptance(altitude_acceptance_m);
        let command = self.follower.next_command(dt, current_state);
        command.with_regime(self.active_regime())
    }
}

/// Parses waypoint rows, with the regime of the leg to each, from CSV `contents`. `path` is
/// used only to label parse errors.
///
/// Blank lines and `#` comments are skipped, as is a leading non-numeric header row. Each
/// remaining line must hold four comma-separated, finite floats
/// (`x,y,altitude,cruise_speed`; non-finite values are rejected), then optionally a regime,
/// `copter` or `plane` (any case; `copter` when absent).
fn parse_route(
    path: &str,
    contents: &str,
) -> Result<Vec<(WaypointCommand, FlightRegime)>, ScriptError> {
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
                    "expected {NUMERIC_FIELDS} or {MAX_FIELDS} fields, found {}",
                    fields.len()
                ),
            });
        }

        let parse = |field: &str, name: &str| {
            let value = field.parse::<f32>().map_err(|err| ScriptError::Parse {
                path: path.to_string(),
                line: line_number,
                message: format!("invalid {name}: {err}"),
            })?;
            // Reject NaN/Inf: a non-finite coordinate would stall waypoint advancement
            // and feed a bad position straight into guidance.
            if !value.is_finite() {
                return Err(ScriptError::Parse {
                    path: path.to_string(),
                    line: line_number,
                    message: format!("non-finite {name}: {field}"),
                });
            }
            Ok(value)
        };

        let regime = match fields.get(NUMERIC_FIELDS).map(|f| f.to_ascii_lowercase()) {
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
        let waypoint = WaypointCommand {
            position_m: Vec2::new(parse(fields[0], "x")?, parse(fields[1], "y")?),
            velocity_mps: None,
            altitude_m: parse(fields[2], "altitude")?,
            cruise_speed_mps: parse(fields[3], "cruise_speed")?,
        };
        waypoints.push((waypoint, regime));
    }

    Ok(waypoints)
}

#[cfg(test)]
mod tests;
