//! A [`Commander`] that replays a sequence of waypoints loaded from a CSV file.
//!
//! [`ScriptCommander`] follows and loops a fixed list of [`WaypointCommand`]s. It has a
//! single job — sequencing waypoints — and knows nothing about how the path was shaped
//! (e.g. a figure-eight); the geometry lives entirely in the CSV. Responsibilities are
//! kept separate: [`parse_waypoints`] turns text into data, [`ScriptCommander::new`]
//! handles file I/O, and [`ScriptCommander::next`] advances through the waypoints.
//!
//! # CSV format
//!
//! One waypoint per line, four comma-separated fields: `x,y,altitude,cruise_speed`
//! (meters and meters-per-second). Blank lines and lines starting with `#` are ignored,
//! and an optional non-numeric header row is skipped.
//!
//! ```text
//! # figure-eight, medium/park envelope
//! x,y,altitude,cruise_speed
//! 30.00,0.00,27.50,7.0
//! 0.00,0.00,40.00,7.0
//! ```

use std::fs;

use robomath::vec2;
use thiserror::Error;

use crate::{Command, Commander, VehicleState, WaypointCommand};

/// Default horizontal distance (meters) within which the active waypoint is considered
/// reached and the commander advances to the next one.
const DEFAULT_ACCEPTANCE_RADIUS: f32 = 3.0;

/// The squared form of [`DEFAULT_ACCEPTANCE_RADIUS`], compared against squared distance
/// each tick to avoid a per-tick `sqrt`.
const DEFAULT_ACCEPTANCE_RADIUS_SQUARED: f32 =
    DEFAULT_ACCEPTANCE_RADIUS * DEFAULT_ACCEPTANCE_RADIUS;

/// Number of comma-separated fields expected on each waypoint row.
const FIELDS_PER_WAYPOINT: usize = 4;

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

/// A [`Commander`] that flies a looping sequence of waypoints loaded from a CSV file.
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
    waypoints: Vec<WaypointCommand>,
    current: usize,
    acceptance_radius_squared: f32,
}

impl ScriptCommander {
    /// Loads a waypoint script from a CSV file at `path`.
    ///
    /// # Errors
    ///
    /// Returns [`ScriptError::Io`] if the file cannot be read, [`ScriptError::Parse`] if a
    /// row is malformed, or [`ScriptError::Empty`] if the file contains no waypoints.
    pub fn new(path: &str) -> Result<Self, ScriptError> {
        Self::from_waypoints(load_waypoints(path)?)
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
    /// use robomath::vec2;
    /// use stable_flight_sitl::WaypointCommand;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let waypoints = vec![WaypointCommand {
    ///     position: vec2(0.0, 0.0),
    ///     ..Default::default()
    /// }];
    /// let commander = ScriptCommander::from_waypoints(waypoints).expect("non-empty");
    /// ```
    pub fn from_waypoints(waypoints: Vec<WaypointCommand>) -> Result<Self, ScriptError> {
        if waypoints.is_empty() {
            return Err(ScriptError::Empty);
        }
        Ok(ScriptCommander {
            waypoints,
            current: 0,
            acceptance_radius_squared: DEFAULT_ACCEPTANCE_RADIUS_SQUARED,
        })
    }

    /// Overrides the horizontal acceptance radius (meters) used to decide when the active
    /// waypoint has been reached. `radius` must be non-negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use robomath::vec2;
    /// use stable_flight_sitl::WaypointCommand;
    /// use stable_flight_sitl::commanders::ScriptCommander;
    ///
    /// let commander = ScriptCommander::from_waypoints(vec![WaypointCommand {
    ///     position: vec2(0.0, 0.0),
    ///     ..Default::default()
    /// }])
    /// .expect("non-empty")
    /// .with_acceptance_radius(5.0);
    /// ```
    pub fn with_acceptance_radius(mut self, radius: f32) -> Self {
        debug_assert!(
            radius >= 0.0,
            "acceptance radius must be non-negative, got {radius}"
        );
        self.acceptance_radius_squared = radius * radius;
        self
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
    let contents = fs::read_to_string(path).map_err(|source| ScriptError::Io {
        path: path.to_string(),
        source,
    })?;
    parse_waypoints(path, &contents)
}

impl Commander for ScriptCommander {
    fn next(&mut self, _dt: f32, current_state: &VehicleState) -> Command {
        // Advance when the vehicle is within the acceptance radius of the active waypoint
        // (horizontal distance only). Compare squared distances to avoid a per-tick sqrt.
        // Wrapping the index loops the path forever.
        let target = &self.waypoints[self.current];
        let dx = target.position.x - current_state.position.x.value;
        let dy = target.position.y - current_state.position.y.value;
        if dx * dx + dy * dy <= self.acceptance_radius_squared {
            self.current = (self.current + 1) % self.waypoints.len();
        }

        Command::Waypoint(self.waypoints[self.current])
    }
}

/// Parses waypoint rows from CSV `contents`. `path` is used only to label parse errors.
///
/// Blank lines and `#` comments are skipped, as is a leading non-numeric header row. Each
/// remaining line must hold exactly four comma-separated, finite floats
/// (`x,y,altitude,cruise_speed`); non-finite values (`NaN`/`Inf`) are rejected.
fn parse_waypoints(path: &str, contents: &str) -> Result<Vec<WaypointCommand>, ScriptError> {
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
        if fields.len() != FIELDS_PER_WAYPOINT {
            return Err(ScriptError::Parse {
                path: path.to_string(),
                line: line_number,
                message: format!(
                    "expected {FIELDS_PER_WAYPOINT} fields, found {}",
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
            // and feed a bad position straight into the stabilizer.
            if !value.is_finite() {
                return Err(ScriptError::Parse {
                    path: path.to_string(),
                    line: line_number,
                    message: format!("non-finite {name}: {field}"),
                });
            }
            Ok(value)
        };

        waypoints.push(WaypointCommand {
            position: vec2(parse(fields[0], "x")?, parse(fields[1], "y")?),
            velocity: None,
            altitude: parse(fields[2], "altitude")?,
            cruise_speed: parse(fields[3], "cruise_speed")?,
            roll: None,
        });
    }

    Ok(waypoints)
}

#[cfg(test)]
mod tests;
