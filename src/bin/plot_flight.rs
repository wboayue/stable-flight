//! `plot_flight` — render a waypoint script as a static 3D flight path.
//!
//! Reads any waypoint CSV in the `flights/` format (`x,y,altitude,cruise_speed`, meters and
//! m/s) and draws it as a 3D path — horizontal `x`/`y` plane with altitude on the vertical
//! axis. Path segments are colored by cruise speed; faint stems and a ground-projected
//! outline give depth cues. Output format follows the file extension: `.svg` or `.png`.
//!
//! This binary is gated behind the `plot` feature so a bare `cargo build`/`cargo test` (and
//! library consumers) never compile `plotters`.
//!
//! ```bash
//! cargo run --features plot --bin plot_flight -- flights/figure_eight.csv fig8.svg
//! ```

use std::error::Error;
use std::process;

use clap::Parser;
use plotters::coord::Shift;
use plotters::prelude::*;

use stable_flight_sitl::WaypointCommand;
use stable_flight_sitl::commanders::script::load_waypoints;

/// Rendered image dimensions in pixels.
const IMAGE_SIZE: (u32, u32) = (1200, 900);

/// Fraction of each axis span added as padding so the path doesn't touch the chart edges.
const AXIS_PADDING_FRACTION: f32 = 0.05;

/// Padding applied when an axis has zero span (all points share a coordinate), in meters.
const DEGENERATE_AXIS_PADDING: f32 = 1.0;

#[derive(Parser, Debug)]
#[command(
    about = "Render a waypoint script (flights/*.csv) as a static 3D flight path",
    // Let negative degree values (e.g. `--yaw -15`) parse as values, not flags.
    allow_negative_numbers = true
)]
struct Config {
    /// Waypoint CSV to render (columns: x,y,altitude,cruise_speed).
    input: String,

    /// Output image path; the `.svg` or `.png` extension selects the format.
    output: String,

    /// Camera tilt above the horizontal, in degrees (default ~28.6).
    #[arg(long, value_name = "DEG")]
    pitch: Option<f64>,

    /// Camera rotation about the vertical axis, in degrees (default ~28.6).
    #[arg(long, value_name = "DEG")]
    yaw: Option<f64>,
}

fn main() {
    let config = Config::parse();
    if let Err(err) = run(&config) {
        eprintln!("error: {err}");
        process::exit(1);
    }
}

/// Camera orientation for the 3D projection, in radians. `None` fields keep plotters' default.
#[derive(Debug, Clone, Copy, Default)]
struct View {
    pitch: Option<f64>,
    yaw: Option<f64>,
}

/// Loads the input, renders it, and writes the image, dispatching on the output extension.
/// Returns an error for an unknown extension or any load/render failure.
fn run(config: &Config) -> Result<(), Box<dyn Error>> {
    let waypoints = load_waypoints(&config.input)?;
    if waypoints.is_empty() {
        return Err(format!("{} contained no waypoints", config.input).into());
    }

    let view = View {
        pitch: config.pitch.map(f64::to_radians),
        yaw: config.yaw.map(f64::to_radians),
    };
    let title = title_for(&config.input);
    let output = &config.output;
    match extension(output).as_deref() {
        Some("svg") => {
            let area = SVGBackend::new(output, IMAGE_SIZE).into_drawing_area();
            draw_chart(area, &waypoints, &title, view)?;
        }
        Some("png") => {
            let area = BitMapBackend::new(output, IMAGE_SIZE).into_drawing_area();
            draw_chart(area, &waypoints, &title, view)?;
        }
        _ => {
            return Err(
                format!("unsupported output extension for '{output}' (use .svg or .png)").into(),
            );
        }
    }

    println!("wrote {output}");
    Ok(())
}

/// Lowercased file extension of `path`, if any.
fn extension(path: &str) -> Option<String> {
    std::path::Path::new(path)
        .extension()
        .and_then(|ext| ext.to_str())
        .map(str::to_ascii_lowercase)
}

/// Chart caption derived from the input file name (stem), falling back to the raw path.
fn title_for(path: &str) -> String {
    let name = std::path::Path::new(path)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or(path);
    format!("{name} — altitude vertical, color = cruise speed")
}

/// Inclusive coordinate ranges (already padded) used to size the 3D chart.
#[derive(Debug, Clone, Copy, PartialEq)]
struct PlotBounds {
    x: (f32, f32),
    y: (f32, f32),
    altitude: (f32, f32),
    speed: (f32, f32),
}

/// Computes padded coordinate ranges spanning every waypoint. `speed` is left un-padded (it
/// drives color normalization, not an axis). Assumes `waypoints` is non-empty.
fn plot_bounds(waypoints: &[WaypointCommand]) -> PlotBounds {
    let mut x = (f32::INFINITY, f32::NEG_INFINITY);
    let mut y = (f32::INFINITY, f32::NEG_INFINITY);
    let mut altitude = (f32::INFINITY, f32::NEG_INFINITY);
    let mut speed = (f32::INFINITY, f32::NEG_INFINITY);

    for wp in waypoints {
        x = (x.0.min(wp.position.x), x.1.max(wp.position.x));
        y = (y.0.min(wp.position.y), y.1.max(wp.position.y));
        altitude = (altitude.0.min(wp.altitude), altitude.1.max(wp.altitude));
        speed = (speed.0.min(wp.cruise_speed), speed.1.max(wp.cruise_speed));
    }

    PlotBounds {
        x: pad_range(x),
        y: pad_range(y),
        altitude: pad_range(altitude),
        speed,
    }
}

/// Expands `(min, max)` by [`AXIS_PADDING_FRACTION`] of its span, or by a fixed amount when the
/// span is zero, so a flat axis still renders with visible extent.
fn pad_range((min, max): (f32, f32)) -> (f32, f32) {
    let span = max - min;
    if span <= f32::EPSILON {
        (min - DEGENERATE_AXIS_PADDING, max + DEGENERATE_AXIS_PADDING)
    } else {
        let pad = span * AXIS_PADDING_FRACTION;
        (min - pad, max + pad)
    }
}

/// Normalizes `speed` to `0.0..=1.0` within `[min, max]`. Returns `0.5` when every waypoint
/// shares a speed (a degenerate range), so the whole path takes a single mid-gradient color.
fn speed_fraction(speed: f32, min: f32, max: f32) -> f32 {
    let span = max - min;
    if span <= f32::EPSILON {
        0.5
    } else {
        ((speed - min) / span).clamp(0.0, 1.0)
    }
}

/// Maps a normalized speed fraction (`0.0..=1.0`) to a blue→red gradient: slow is blue,
/// fast is red.
fn speed_color(fraction: f32) -> HSLColor {
    // Hue 240° (blue) down to 0° (red), expressed in the 0.0..=1.0 hue space plotters uses.
    let hue = (1.0 - fraction as f64) * (240.0 / 360.0);
    HSLColor(hue, 0.85, 0.5)
}

/// Draws the flight path into `area`. Generic over the drawing backend so the same code serves
/// both SVG and PNG output.
fn draw_chart<DB>(
    area: DrawingArea<DB, Shift>,
    waypoints: &[WaypointCommand],
    title: &str,
    view: View,
) -> Result<(), Box<dyn Error>>
where
    DB: DrawingBackend,
    DB::ErrorType: 'static,
{
    area.fill(&WHITE)?;
    let bounds = plot_bounds(waypoints);

    let mut chart = ChartBuilder::on(&area)
        .caption(title, ("sans-serif", 26))
        .margin(20)
        // x horizontal, altitude vertical (middle axis), y depth.
        .build_cartesian_3d(
            bounds.x.0..bounds.x.1,
            bounds.altitude.0..bounds.altitude.1,
            bounds.y.0..bounds.y.1,
        )?;

    // Override camera angles only when requested, leaving plotters' default view otherwise.
    // Must precede axis/series drawing so both use the same projection.
    if view.pitch.is_some() || view.yaw.is_some() {
        chart.with_projection(|mut pb| {
            if let Some(pitch) = view.pitch {
                pb.pitch = pitch;
            }
            if let Some(yaw) = view.yaw {
                pb.yaw = yaw;
            }
            pb.into_matrix()
        });
    }

    chart
        .configure_axes()
        .light_grid_style(RGBColor(220, 220, 220))
        .label_style(("sans-serif", 12))
        .draw()?;

    let floor = bounds.altitude.0;

    // Ground-projected outline (altitude flattened to the floor): a top-down shadow of the
    // horizontal path. This is where a figure-eight's self-crossing is visible — in 3D the
    // altitude profile can pull the crossing apart into a non-intersecting loop.
    chart.draw_series(LineSeries::new(
        looped(waypoints).map(|wp| (wp.position.x, floor, wp.position.y)),
        RGBColor(150, 150, 150).stroke_width(2),
    ))?;

    // Faint vertical stems from each waypoint down to the floor.
    for wp in waypoints {
        chart.draw_series(LineSeries::new(
            [
                (wp.position.x, floor, wp.position.y),
                (wp.position.x, wp.altitude, wp.position.y),
            ],
            RGBColor(210, 210, 210).stroke_width(1),
        ))?;
    }

    // Flight path: one segment per waypoint pair, colored by the segment's mean cruise speed.
    // Looping connects the last waypoint back to the first, matching how scripts replay.
    let (speed_min, speed_max) = bounds.speed;
    let points: Vec<&WaypointCommand> = waypoints.iter().collect();
    for pair in looped_pairs(&points) {
        let (a, b) = pair;
        let mean_speed = 0.5 * (a.cruise_speed + b.cruise_speed);
        let color = speed_color(speed_fraction(mean_speed, speed_min, speed_max));
        chart.draw_series(LineSeries::new(
            [
                (a.position.x, a.altitude, a.position.y),
                (b.position.x, b.altitude, b.position.y),
            ],
            color.stroke_width(3),
        ))?;
    }

    // Waypoint markers.
    chart.draw_series(waypoints.iter().map(|wp| {
        Circle::new(
            (wp.position.x, wp.altitude, wp.position.y),
            3,
            BLACK.filled(),
        )
    }))?;

    area.present()?;
    Ok(())
}

/// Iterates the waypoints once and then yields the first again, closing the loop for the
/// ground-projection outline.
fn looped(waypoints: &[WaypointCommand]) -> impl Iterator<Item = &WaypointCommand> {
    waypoints.iter().chain(waypoints.first())
}

/// Yields consecutive `(a, b)` waypoint pairs, including the wrap-around pair `(last, first)`
/// so the drawn path forms a closed loop. Empty for fewer than two waypoints.
fn looped_pairs<'a>(
    points: &'a [&'a WaypointCommand],
) -> impl Iterator<Item = (&'a WaypointCommand, &'a WaypointCommand)> {
    points
        .windows(2)
        .map(|w| (w[0], w[1]))
        .chain(match (points.first(), points.last()) {
            (Some(&first), Some(&last)) if points.len() > 1 => Some((last, first)),
            _ => None,
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert2::check;
    use robomath::vec2;

    fn wp(x: f32, y: f32, altitude: f32, cruise_speed: f32) -> WaypointCommand {
        WaypointCommand {
            position: vec2(x, y),
            velocity: None,
            altitude,
            cruise_speed,
            roll: None,
        }
    }

    #[test]
    fn bounds_span_all_waypoints_with_padding() {
        let waypoints = [wp(-10.0, 0.0, 5.0, 3.0), wp(10.0, 20.0, 15.0, 9.0)];
        let bounds = plot_bounds(&waypoints);

        // 5% padding on a 20 m span = 1 m each side.
        check!(bounds.x == (-11.0, 11.0));
        check!(bounds.y == (-1.0, 21.0));
        check!(bounds.altitude == (4.5, 15.5));
        // Speed range is raw (drives color, not an axis).
        check!(bounds.speed == (3.0, 9.0));
    }

    #[test]
    fn bounds_pad_degenerate_axis() {
        // Every waypoint shares altitude and y — those axes get fixed padding, not 0-span.
        let waypoints = [wp(0.0, 5.0, 10.0, 4.0), wp(8.0, 5.0, 10.0, 4.0)];
        let bounds = plot_bounds(&waypoints);

        check!(bounds.altitude == (9.0, 11.0));
        check!(bounds.y == (4.0, 6.0));
        check!(bounds.x == (-0.4, 8.4));
    }

    #[test]
    fn speed_fraction_normalizes_within_range() {
        let cases = [
            (2.0, 2.0, 10.0, 0.0),
            (10.0, 2.0, 10.0, 1.0),
            (6.0, 2.0, 10.0, 0.5),
            // Out-of-range clamps.
            (0.0, 2.0, 10.0, 0.0),
            (99.0, 2.0, 10.0, 1.0),
        ];
        for (speed, min, max, expected) in cases {
            check!(speed_fraction(speed, min, max) == expected);
        }
    }

    #[test]
    fn speed_fraction_constant_when_all_equal() {
        // Degenerate range → single mid-gradient color for the whole path.
        check!(speed_fraction(7.0, 7.0, 7.0) == 0.5);
    }

    #[test]
    fn parses_paths_without_view_flags() {
        let config = Config::try_parse_from(["plot_flight", "in.csv", "out.svg"]).expect("valid");
        check!(config.input == "in.csv");
        check!(config.output == "out.svg");
        check!(config.pitch == None);
        check!(config.yaw == None);
    }

    #[test]
    fn parses_pitch_and_yaw_in_degrees() {
        let config = Config::try_parse_from([
            "plot_flight",
            "in.csv",
            "out.png",
            "--pitch",
            "30",
            "--yaw",
            "-15",
        ])
        .expect("valid");
        check!(config.pitch == Some(30.0));
        check!(config.yaw == Some(-15.0));
    }

    #[test]
    fn rejects_missing_output_and_bad_flag_value() {
        check!(Config::try_parse_from(["plot_flight", "in.csv"]).is_err());
        check!(
            Config::try_parse_from(["plot_flight", "in.csv", "out.svg", "--pitch", "nope"])
                .is_err()
        );
    }
}
