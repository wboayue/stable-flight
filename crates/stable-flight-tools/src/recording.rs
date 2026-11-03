//! Reading a recorder CSV (`stable_flight_sitl::recorder`): a header line naming the columns,
//! `wall_s` first, then one numeric row per tick.
//!
//! Columns are found by their header names, so any column set reads. Text cells (`sp_kind`)
//! read as NaN, and a row of the wrong width (the last one of a stopped run) is dropped.
//!
//! # Examples
//!
//! ```
//! use stable_flight_tools::recording::Recording;
//!
//! let recording = Recording::parse("wall_s,altitude\n0.0,1.5\n0.1,1.6\n").expect("a recording");
//! let altitude = recording.column("altitude").expect("an altitude column");
//! assert_eq!(recording.rows()[1][altitude], 1.6);
//! ```

use thiserror::Error;

#[cfg(test)]
mod tests;

/// The time column every recorder writes first, seconds since the recorder was made.
pub const TIME: &str = "wall_s";

/// Why a CSV isn't a recording.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum RecordingError {
    /// The text has no header line.
    #[error("empty recording")]
    Empty,
    /// The header's first column isn't [`TIME`].
    #[error("not a recording: the first column isn't `{TIME}`")]
    NoTime,
}

/// A recording's column names, from its header line.
///
/// # Examples
///
/// Reading a recording a line at a time, as it grows:
///
/// ```
/// use stable_flight_tools::recording::Header;
///
/// let header = Header::parse("wall_s,mode,sp_kind").expect("a header");
/// let row = header.row("0.5,1,plane").expect("a whole row");
/// assert_eq!(row[header.column("mode").expect("a mode column")], 1.0);
/// assert!(row[2].is_nan());
/// assert!(header.row("0.6,1").is_none());
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Header {
    columns: Vec<String>,
}

impl Header {
    /// Parses a header line.
    ///
    /// # Errors
    ///
    /// [`RecordingError::NoTime`] when the first column isn't [`TIME`].
    pub fn parse(line: &str) -> Result<Self, RecordingError> {
        let columns: Vec<String> = line.trim_end().split(',').map(str::to_owned).collect();
        if columns.first().map(String::as_str) != Some(TIME) {
            return Err(RecordingError::NoTime);
        }
        Ok(Self { columns })
    }

    /// `name`'s index in a row, or `None` without the column.
    pub fn column(&self, name: &str) -> Option<usize> {
        self.columns.iter().position(|c| c == name)
    }

    /// One data line's cells, text as NaN; `None` when its width isn't the header's.
    pub fn row(&self, line: &str) -> Option<Vec<f32>> {
        let row: Vec<f32> = line
            .trim_end()
            .split(',')
            .map(|cell| cell.trim().parse().unwrap_or(f32::NAN))
            .collect();
        (row.len() == self.columns.len()).then_some(row)
    }
}

/// A parsed recorder CSV: its header and numeric rows.
#[derive(Debug, Clone, PartialEq)]
pub struct Recording {
    header: Header,
    rows: Vec<Vec<f32>>,
}

impl Recording {
    /// Parses a whole recorder CSV, dropping rows of the wrong width.
    ///
    /// # Errors
    ///
    /// No header, or no [`TIME`] column first.
    pub fn parse(text: &str) -> Result<Self, RecordingError> {
        let mut lines = text.lines();
        let header = Header::parse(lines.next().ok_or(RecordingError::Empty)?)?;
        let rows = lines.filter_map(|line| header.row(line)).collect();
        Ok(Self { header, rows })
    }

    /// `name`'s index in a row, or `None` without the column.
    pub fn column(&self, name: &str) -> Option<usize> {
        self.header.column(name)
    }

    /// The rows, in time order, [`TIME`] at index 0.
    pub fn rows(&self) -> &[Vec<f32>] {
        &self.rows
    }

    /// `name`'s (time, value · scale) points within `window` (seconds, inclusive), split at
    /// NaN cells into runs; `None` without the column.
    pub fn series(
        &self,
        name: &str,
        scale: f32,
        (from_s, to_s): (f32, f32),
    ) -> Option<Vec<Vec<(f32, f32)>>> {
        let index = self.column(name)?;
        let mut runs = vec![Vec::new()];
        for row in &self.rows {
            let (time, value) = (row[0], row[index]);
            if !(from_s..=to_s).contains(&time) {
                continue;
            }
            if value.is_nan() {
                if runs.last().is_some_and(|run| !run.is_empty()) {
                    runs.push(Vec::new());
                }
                continue;
            }
            if let Some(run) = runs.last_mut() {
                run.push((time, value * scale));
            }
        }
        runs.retain(|run| !run.is_empty());
        Some(runs)
    }
}
