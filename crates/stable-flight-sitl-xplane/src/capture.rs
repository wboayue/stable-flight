//! Raw X-Plane captures: every [`XPlaneSample`], unconverted, as CSV, optionally followed by a
//! writer's own columns (e.g. the throttles `engine-sweep-xplane` sends). [`write_header`] and
//! [`write_row`] write them (the `capture-state-xplane` and `engine-sweep-xplane` examples);
//! [`read_capture`] reads the samples back for offline checks (e.g. the frame contract,
//! `check_frames --sim xplane`).

use std::io::{self, BufRead, Write};

use thiserror::Error;

use crate::XPlaneSample;

#[cfg(test)]
mod tests;

/// One CSV column: its header name, how to read it from a sample, and how to write it back.
/// Booleans record as 0/1.
struct Column {
    name: &'static str,
    get: fn(&XPlaneSample) -> f32,
    set: fn(&mut XPlaneSample, f32),
}

macro_rules! column {
    ($name:literal, $field:ident $([$i:literal])?) => {
        Column {
            name: $name,
            get: |s| s.$field $([$i])?,
            set: |s, v| s.$field $([$i])? = v,
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

/// The columns, in CSV order: dataref names, with units where X-Plane's aren't SI.
const COLUMNS: &[Column] = &[
    column!("sim_time_s", sim_time_s),
    column!("q_w", quaternion[0]),
    column!("q_x", quaternion[1]),
    column!("q_y", quaternion[2]),
    column!("q_z", quaternion[3]),
    column!("p_degps", body_rates_degps[0]),
    column!("q_degps", body_rates_degps[1]),
    column!("r_degps", body_rates_degps[2]),
    column!("local_x_m", local_position_m[0]),
    column!("local_y_m", local_position_m[1]),
    column!("local_z_m", local_position_m[2]),
    column!("local_vx_mps", local_velocity_mps[0]),
    column!("local_vy_mps", local_velocity_mps[1]),
    column!("local_vz_mps", local_velocity_mps[2]),
    column!("local_ax_mps2", local_acceleration_mps2[0]),
    column!("local_ay_mps2", local_acceleration_mps2[1]),
    column!("local_az_mps2", local_acceleration_mps2[2]),
    column!("latitude_deg", latitude_deg),
    column!("longitude_deg", longitude_deg),
    column!("elevation_m", elevation_m),
    column!("lat_ref_deg", origin_deg[0]),
    column!("lon_ref_deg", origin_deg[1]),
    column!("true_airspeed_mps", true_airspeed_mps),
    column!("on_ground", bool on_ground),
    column!("paused", bool paused),
];

/// Writes the CSV header: the sample's columns, then `extra` (the writer's own, in the order
/// its rows pass them to [`write_row`]).
///
/// # Errors
///
/// When `out` fails.
pub fn write_header(out: &mut impl Write, extra: &[&str]) -> io::Result<()> {
    let names: Vec<_> = COLUMNS
        .iter()
        .map(|column| column.name)
        .chain(extra.iter().copied())
        .collect();
    writeln!(out, "{}", names.join(","))
}

/// Writes one CSV row: `sample`, then the `extra` values, one per extra column.
///
/// # Errors
///
/// When `out` fails.
pub fn write_row(out: &mut impl Write, sample: &XPlaneSample, extra: &[f32]) -> io::Result<()> {
    let values = COLUMNS
        .iter()
        .map(|column| (column.get)(sample))
        .chain(extra.iter().copied());
    for (i, value) in values.enumerate() {
        let separator = if i == 0 { "" } else { "," };
        write!(out, "{separator}{value}")?;
    }
    writeln!(out)
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

/// Reads the samples of a capture written by [`write_header`] and [`write_row`], in order.
/// Columns are found by name, so extra columns are ignored. A short last row (a capture cut off mid-write) is
/// dropped.
///
/// # Errors
///
/// [`CaptureError`] on a read failure, a missing column, or a value that isn't a number.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::XPlaneSample;
/// use stable_flight_sitl_xplane::capture::{read_capture, write_header, write_row};
///
/// let mut csv = Vec::new();
/// write_header(&mut csv, &["throttle0"]).unwrap();
/// let sample = XPlaneSample { body_rates_degps: [12.5, 0.0, 0.0], ..Default::default() };
/// write_row(&mut csv, &sample, &[0.4]).unwrap();
///
/// let rows = read_capture(csv.as_slice()).unwrap();
/// assert_eq!(rows[0].body_rates_degps[0], 12.5);
/// ```
pub fn read_capture(reader: impl BufRead) -> Result<Vec<XPlaneSample>, CaptureError> {
    let mut lines = reader.lines();
    let header = lines.next().ok_or(CaptureError::NoHeader)??;
    let names: Vec<&str> = header.split(',').map(str::trim).collect();
    let columns = COLUMNS
        .iter()
        .map(|column| {
            let at = names
                .iter()
                .position(|n| *n == column.name)
                .ok_or_else(|| CaptureError::MissingColumn(column.name.to_string()))?;
            Ok((at, column))
        })
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
        let mut sample = XPlaneSample::default();
        for &(at, column) in &columns {
            let text = fields.get(at).copied().unwrap_or("").trim();
            let value = text.parse().map_err(|_| CaptureError::BadValue {
                line: index + 2,
                column: column.name.to_string(),
                value: text.to_string(),
            })?;
            (column.set)(&mut sample, value);
        }
        rows.push(sample);
    }
    Ok(rows)
}
