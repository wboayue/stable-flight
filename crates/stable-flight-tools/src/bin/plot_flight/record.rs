//! `--record`: a recorder CSV (`stable_flight_sitl::recorder`) as time series and a ground
//! track.
//!
//! Columns are found by their header names, so any column set plots: each panel draws the
//! columns it finds and is left out when it finds none. NaN cells (a blend's lift share
//! outside a transition, the cruise side in a hover) break a trace instead of joining across.

use std::error::Error;

use plotters::coord::Shift;
use plotters::prelude::*;

use crate::{FONT_FAMILY, draw_to};

#[cfg(test)]
mod tests;

/// Image width in pixels; the height grows with the panel count.
const IMAGE_WIDTH: u32 = 1800;

/// Each time-series panel's height in pixels.
const PANEL_HEIGHT: u32 = 220;

/// The time column every recorder writes first.
const TIME: &str = "wall_s";

/// One column drawn in a panel, scaled (radians to degrees) and labeled.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Trace {
    column: &'static str,
    label: &'static str,
    scale: f32,
}

/// A time-series panel: a title with its unit, and its traces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Panel {
    title: &'static str,
    traces: &'static [Trace],
}

const fn trace(column: &'static str, label: &'static str) -> Trace {
    Trace {
        column,
        label,
        scale: 1.0,
    }
}

const fn degrees(column: &'static str, label: &'static str) -> Trace {
    Trace {
        column,
        label,
        scale: 180.0 / core::f32::consts::PI,
    }
}

/// Every panel, top to bottom.
pub(super) const PANELS: &[Panel] = &[
    Panel {
        title: "altitude (m)",
        traces: &[trace("altitude", "altitude")],
    },
    Panel {
        title: "speed (m/s)",
        traces: &[
            trace("airspeed_mps", "airspeed"),
            trace("climb_mps", "climb"),
        ],
    },
    Panel {
        title: "roll (deg)",
        traces: &[degrees("roll", "roll"), degrees("sp_roll", "setpoint")],
    },
    Panel {
        title: "pitch (deg)",
        traces: &[degrees("pitch", "pitch"), degrees("sp_pitch", "setpoint")],
    },
    Panel {
        title: "heading (deg)",
        traces: &[
            degrees("heading", "heading"),
            degrees("sp_heading", "setpoint"),
        ],
    },
    Panel {
        title: "lift and thrust",
        traces: &[
            trace("lift_share", "lift share"),
            trace("thrust", "lift thrust"),
            trace("cruise_thrust", "cruise thrust"),
            trace("pusher", "pusher"),
        ],
    },
    Panel {
        title: "motors",
        traces: &[
            trace("motor_0", "0"),
            trace("motor_1", "1"),
            trace("motor_2", "2"),
            trace("motor_3", "3"),
        ],
    },
    Panel {
        title: "surfaces",
        traces: &[
            trace("servo_0", "servo 0"),
            trace("servo_1", "servo 1"),
            trace("servo_2", "servo 2"),
        ],
    },
    Panel {
        title: "flight mode",
        traces: &[trace("mode", "mode")],
    },
];

/// A parsed recorder CSV: its column names and numeric rows.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Recording {
    columns: Vec<String>,
    rows: Vec<Vec<f32>>,
}

impl Recording {
    /// Parses a recorder CSV. Text cells (`sp_kind`) read as NaN; a row with the wrong number
    /// of cells (the last one of a killed run) is dropped.
    ///
    /// # Errors
    ///
    /// No header, or no `wall_s` column.
    pub(super) fn parse(text: &str) -> Result<Self, String> {
        let mut lines = text.lines();
        let header = lines.next().ok_or("empty recording")?;
        let columns: Vec<String> = header.split(',').map(str::to_owned).collect();
        if columns.first().map(String::as_str) != Some(TIME) {
            return Err(format!("not a recording: the first column isn't `{TIME}`"));
        }
        let rows = lines
            .map(|line| {
                line.split(',')
                    .map(|cell| cell.trim().parse().unwrap_or(f32::NAN))
                    .collect::<Vec<f32>>()
            })
            .filter(|row| row.len() == columns.len())
            .collect();
        Ok(Recording { columns, rows })
    }

    fn column(&self, name: &str) -> Option<usize> {
        self.columns.iter().position(|c| c == name)
    }

    /// The panels with at least one of their columns in this recording.
    pub(super) fn panels(&self) -> Vec<Panel> {
        PANELS
            .iter()
            .filter(|panel| panel.traces.iter().any(|t| self.column(t.column).is_some()))
            .copied()
            .collect()
    }

    /// `name`'s (time, value · scale) points within `window` (seconds, inclusive), split at
    /// NaN cells into runs; `None` without the column.
    pub(super) fn series(
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

/// Reads `input`, a recorder CSV, and draws it to `output` (`.svg` or `.png`) over `window`
/// (seconds of `wall_s`, inclusive).
///
/// # Errors
///
/// The file can't be read or parsed, it has nothing in the window, or drawing fails.
pub(super) fn render(input: &str, output: &str, window: (f32, f32)) -> Result<(), Box<dyn Error>> {
    let recording = Recording::parse(&std::fs::read_to_string(input)?)?;
    let panels = recording.panels();
    let time = recording
        .series(TIME, 1.0, window)
        .and_then(|runs| range(runs.iter().flatten().map(|&(t, _)| t)))
        .ok_or_else(|| format!("{input} has no rows in the window"))?;
    let rows = u32::try_from(panels.len().max(3))?;
    let size = (IMAGE_WIDTH, PANEL_HEIGHT * rows + 60);
    let title = format!(
        "{} — {:.0}–{:.0} s",
        std::path::Path::new(input)
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or(input),
        time.0,
        time.1
    );
    draw_to!(output, size, |area| draw(
        area, &recording, &panels, &title, time
    ))
}

/// The time series on the left, the ground track on the right.
fn draw<DB>(
    area: DrawingArea<DB, Shift>,
    recording: &Recording,
    panels: &[Panel],
    title: &str,
    time: (f32, f32),
) -> Result<(), Box<dyn Error>>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
{
    area.fill(&WHITE)?;
    let area = area.titled(title, (FONT_FAMILY, 26))?;
    let (series_area, track_area) = area.split_horizontally(IMAGE_WIDTH * 7 / 10);
    for (panel, panel_area) in panels
        .iter()
        .zip(series_area.split_evenly((panels.len(), 1)))
    {
        draw_panel(&panel_area, recording, panel, time)?;
    }
    draw_track(&track_area, recording, time)?;
    area.present()?;
    Ok(())
}

/// One panel's traces against time.
fn draw_panel<DB>(
    area: &DrawingArea<DB, Shift>,
    recording: &Recording,
    panel: &Panel,
    time: (f32, f32),
) -> Result<(), Box<dyn Error>>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
{
    let traces: Vec<_> = panel
        .traces
        .iter()
        .filter_map(|t| Some((t, recording.series(t.column, t.scale, time)?)))
        .collect();
    let values = traces
        .iter()
        .flat_map(|(_, runs)| runs.iter().flatten().map(|&(_, v)| v));
    let (low, high) = pad(range(values).unwrap_or((0.0, 1.0)));

    let mut chart = ChartBuilder::on(area)
        .caption(panel.title, (FONT_FAMILY, 16))
        .margin(8)
        .x_label_area_size(24)
        .y_label_area_size(50)
        .build_cartesian_2d(time.0..time.1, low..high)?;
    chart
        .configure_mesh()
        .light_line_style(RGBColor(230, 230, 230))
        .label_style((FONT_FAMILY, 12))
        .draw()?;
    for (index, (trace, runs)) in traces.iter().enumerate() {
        let color = Palette99::pick(index).to_rgba();
        for (run_index, run) in runs.iter().enumerate() {
            let series =
                chart.draw_series(LineSeries::new(run.iter().copied(), color.stroke_width(2)))?;
            if run_index == 0 {
                series.label(trace.label).legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2))
                });
            }
        }
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.8))
        .border_style(RGBColor(200, 200, 200))
        .label_font((FONT_FAMILY, 12))
        .draw()?;
    Ok(())
}

/// North against east over the window, with the start marked; empty without position columns.
fn draw_track<DB>(
    area: &DrawingArea<DB, Shift>,
    recording: &Recording,
    time: (f32, f32),
) -> Result<(), Box<dyn Error>>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
{
    let (Some(north), Some(east)) = (recording.column("north_m"), recording.column("east_m"))
    else {
        return Ok(());
    };
    let points: Vec<(f32, f32)> = recording
        .rows
        .iter()
        .filter(|row| (time.0..=time.1).contains(&row[0]))
        .map(|row| (row[east], row[north]))
        .filter(|(e, n)| e.is_finite() && n.is_finite())
        .collect();
    let Some(&start) = points.first() else {
        return Ok(());
    };
    // One scale on both axes: the larger span, centered.
    let (east_range, north_range) = (
        range(points.iter().map(|p| p.0)).unwrap_or((0.0, 1.0)),
        range(points.iter().map(|p| p.1)).unwrap_or((0.0, 1.0)),
    );
    let half = 0.55
        * (east_range.1 - east_range.0)
            .max(north_range.1 - north_range.0)
            .max(1.0);
    let center = (
        0.5 * (east_range.0 + east_range.1),
        0.5 * (north_range.0 + north_range.1),
    );

    let mut chart = ChartBuilder::on(area)
        .caption("ground track (m)", (FONT_FAMILY, 16))
        .margin(12)
        .x_label_area_size(30)
        .y_label_area_size(60)
        .build_cartesian_2d(
            center.0 - half..center.0 + half,
            center.1 - half..center.1 + half,
        )?;
    chart
        .configure_mesh()
        .x_desc("east")
        .y_desc("north")
        .light_line_style(RGBColor(230, 230, 230))
        .label_style((FONT_FAMILY, 12))
        .draw()?;
    chart.draw_series(LineSeries::new(points, BLUE.stroke_width(2)))?;
    chart.draw_series([Circle::new(start, 5, GREEN.filled())])?;
    Ok(())
}

/// The finite minimum and maximum, if any.
fn range(values: impl Iterator<Item = f32>) -> Option<(f32, f32)> {
    values
        .filter(|v| v.is_finite())
        .fold(None, |acc, v| match acc {
            None => Some((v, v)),
            Some((low, high)) => Some((low.min(v), high.max(v))),
        })
}

/// `(low, high)` widened by 5% each way, or by 0.5 when flat.
fn pad((low, high): (f32, f32)) -> (f32, f32) {
    let margin = if high > low { 0.05 * (high - low) } else { 0.5 };
    (low - margin, high + margin)
}
