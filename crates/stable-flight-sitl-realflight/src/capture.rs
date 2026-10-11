//! Raw RealFlight captures: every [`SimulatorState`] an exchange returns, unconverted, with the
//! channels sent in it, as CSV. [`RecordingBridge`] writes them under any pipeline;
//! [`read_capture`] reads them back for offline checks (e.g. the frame contract, #62, #120).

use std::cell::RefCell;
use std::io::{self, BufRead, Write};

use log::warn;
use realflight_bridge::{BridgeError, ControlInputs, RealFlightBridge, SimulatorState};
use thiserror::Error;

use crate::mapping::CHANNEL_COUNT;

#[cfg(test)]
mod tests;

/// One CSV column: its header name, how to read it from a report, and how to write it back.
/// Booleans record as 0/1.
struct Column {
    name: &'static str,
    get: fn(&SimulatorState) -> f32,
    set: fn(&mut SimulatorState, f32),
}

macro_rules! column {
    ($name:literal, $($field:ident).+) => {
        Column {
            name: $name,
            get: |s| s.$($field).+,
            set: |s, v| s.$($field).+ = v,
        }
    };
    ($name:literal, bool $field:ident) => {
        Column {
            name: $name,
            get: |s| f32::from(u8::from(s.$field)),
            set: |s, v| s.$field = v != 0.0,
        }
    };
}

/// The state columns, in CSV order. The channels sent with the exchange follow them.
const COLUMNS: &[Column] = &[
    column!("physics_time_s", current_physics_time_s),
    column!("roll_deg", roll_deg),
    column!("inclination_deg", inclination_deg),
    column!("azimuth_deg", azimuth_deg),
    column!("roll_rate_dps", roll_rate_dps),
    column!("pitch_rate_dps", pitch_rate_dps),
    column!("yaw_rate_dps", yaw_rate_dps),
    column!("q_w", orientation.w),
    column!("q_x", orientation.x),
    column!("q_y", orientation.y),
    column!("q_z", orientation.z),
    column!("position_x_m", aircraft_position_x_m),
    column!("position_y_m", aircraft_position_y_m),
    column!("altitude_asl_m", altitude_asl_m),
    column!("altitude_agl_m", altitude_agl_m),
    column!("velocity_world_x", velocity_world_mps.x),
    column!("velocity_world_y", velocity_world_mps.y),
    column!("velocity_world_z", velocity_world_mps.z),
    column!("velocity_body_x", velocity_body_mps.x),
    column!("velocity_body_y", velocity_body_mps.y),
    column!("velocity_body_z", velocity_body_mps.z),
    column!("acceleration_world_x", acceleration_world_mps2.x),
    column!("acceleration_world_y", acceleration_world_mps2.y),
    column!("acceleration_world_z", acceleration_world_mps2.z),
    column!("acceleration_body_x", acceleration_body_mps2.x),
    column!("acceleration_body_y", acceleration_body_mps2.y),
    column!("acceleration_body_z", acceleration_body_mps2.z),
    column!("is_touching_ground", bool is_touching_ground),
    column!("is_locked", bool is_locked),
];

/// The name of channel `i`'s column.
fn channel_column(i: usize) -> String {
    format!("ch{i}")
}

/// Writes the CSV header: the state columns, then `ch0`…`ch11`.
///
/// # Errors
///
/// When `out` fails.
pub fn write_header(out: &mut impl Write) -> io::Result<()> {
    let names = COLUMNS.iter().map(|column| column.name.to_string());
    let channels = (0..CHANNEL_COUNT).map(channel_column);
    writeln!(
        out,
        "{}",
        names.chain(channels).collect::<Vec<_>>().join(",")
    )
}

/// Writes one CSV row: the state RealFlight returned, then the channels sent with that
/// exchange.
///
/// # Errors
///
/// When `out` fails.
pub fn write_row(
    out: &mut impl Write,
    state: &SimulatorState,
    control: &ControlInputs,
) -> io::Result<()> {
    let values = COLUMNS
        .iter()
        .map(|column| (column.get)(state))
        .chain(control.channels);
    for (i, value) in values.enumerate() {
        let separator = if i == 0 { "" } else { "," };
        write!(out, "{separator}{value}")?;
    }
    writeln!(out)
}

/// One captured exchange: the state RealFlight returned (the captured fields; the rest are
/// default) and the channels sent.
#[derive(Debug, Clone, PartialEq)]
pub struct CapturedExchange {
    /// The state, as RealFlight reported it.
    pub state: SimulatorState,
    /// The channels sent with the exchange, 0.0 to 1.0.
    pub channels: [f32; CHANNEL_COUNT],
}

/// Why [`read_capture`] failed.
#[derive(Debug, Error)]
pub enum CaptureError {
    /// Reading failed.
    #[error("reading the capture failed: {0}")]
    Io(#[from] io::Error),
    /// The capture is empty.
    #[error("the capture has no header")]
    NoHeader,
    /// The header lacks a column the reader needs.
    #[error("the capture has no `{0}` column")]
    MissingColumn(String),
    /// A value isn't a number.
    #[error("line {line}: `{value}` in column `{column}` isn't a number")]
    BadValue {
        /// 1-based line number.
        line: usize,
        /// The column's name.
        column: String,
        /// The text found.
        value: String,
    },
}

/// Reads a capture written by [`write_header`] and [`write_row`] (e.g. by
/// [`RecordingBridge`]), in order. Columns are found by name, so extra columns are ignored. A
/// short last row (a capture cut off mid-write) is dropped.
///
/// # Errors
///
/// [`CaptureError`] on a read failure, a missing column, or a value that isn't a number.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::capture::{read_capture, write_header, write_row};
/// use stable_flight_sitl_realflight::realflight_bridge::{ControlInputs, SimulatorState};
///
/// let mut csv = Vec::new();
/// write_header(&mut csv).unwrap();
/// let state = SimulatorState { roll_rate_dps: 12.5, ..Default::default() };
/// write_row(&mut csv, &state, &ControlInputs::default()).unwrap();
///
/// let rows = read_capture(csv.as_slice()).unwrap();
/// assert_eq!(rows[0].state.roll_rate_dps, 12.5);
/// ```
pub fn read_capture(reader: impl BufRead) -> Result<Vec<CapturedExchange>, CaptureError> {
    let mut lines = reader.lines();
    let header = lines.next().ok_or(CaptureError::NoHeader)??;
    let names: Vec<&str> = header.split(',').map(str::trim).collect();
    let find = |name: &str| {
        names
            .iter()
            .position(|n| *n == name)
            .ok_or_else(|| CaptureError::MissingColumn(name.to_string()))
    };
    let state_columns = COLUMNS
        .iter()
        .map(|column| Ok((find(column.name)?, column)))
        .collect::<Result<Vec<_>, CaptureError>>()?;
    let channel_columns = (0..CHANNEL_COUNT)
        .map(|i| find(&channel_column(i)))
        .collect::<Result<Vec<_>, CaptureError>>()?;

    let mut rows = Vec::new();
    let mut lines = lines.enumerate().peekable();
    while let Some((index, line)) = lines.next() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        // A capture cut off mid-write (the process killed) ends in a short row: drop it.
        if fields.len() < names.len() && lines.peek().is_none() {
            break;
        }
        let line_number = index + 2;
        let value = |at: usize, column: &str| -> Result<f32, CaptureError> {
            let text = fields.get(at).copied().unwrap_or("").trim();
            text.parse().map_err(|_| CaptureError::BadValue {
                line: line_number,
                column: column.to_string(),
                value: text.to_string(),
            })
        };
        let mut state = SimulatorState::default();
        for &(at, column) in &state_columns {
            (column.set)(&mut state, value(at, column.name)?);
        }
        let mut channels = [0.0; CHANNEL_COUNT];
        for (i, (&at, channel)) in channel_columns.iter().zip(&mut channels).enumerate() {
            *channel = value(at, &channel_column(i))?;
        }
        rows.push(CapturedExchange { state, channels });
    }
    Ok(rows)
}

/// A [`RealFlightBridge`] that passes every call through to `inner` and writes each exchange's
/// state and channels to `out` (a capture), so any pipeline can record raw RealFlight data.
/// A failed write is logged, not returned: the flight goes on. `out` is flushed on drop.
pub struct RecordingBridge<B, W: Write> {
    inner: B,
    out: RefCell<W>,
}

impl<B, W: Write> RecordingBridge<B, W> {
    /// Records `inner`'s exchanges to `out`, after writing the header.
    ///
    /// # Errors
    ///
    /// When writing the header fails.
    pub fn new(inner: B, mut out: W) -> io::Result<Self> {
        write_header(&mut out)?;
        Ok(Self {
            inner,
            out: RefCell::new(out),
        })
    }
}

impl<B: RealFlightBridge, W: Write> RealFlightBridge for RecordingBridge<B, W> {
    fn exchange_data(&self, control: &ControlInputs) -> Result<SimulatorState, BridgeError> {
        let state = self.inner.exchange_data(control)?;
        if let Err(e) = write_row(&mut *self.out.borrow_mut(), &state, control) {
            warn!("Failed to write capture row: {e}");
        }
        Ok(state)
    }

    fn enable_rc(&self) -> Result<(), BridgeError> {
        self.inner.enable_rc()
    }

    fn disable_rc(&self) -> Result<(), BridgeError> {
        self.inner.disable_rc()
    }

    fn reset_aircraft(&self) -> Result<(), BridgeError> {
        self.inner.reset_aircraft()
    }
}

impl<B, W: Write> Drop for RecordingBridge<B, W> {
    fn drop(&mut self) {
        if let Err(e) = self.out.get_mut().flush() {
            warn!("Failed to flush capture: {e}");
        }
    }
}
