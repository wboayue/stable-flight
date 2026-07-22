//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in `flights/figure_eight.csv` (regenerate them
//! with `cargo run --example generate_figure_eight`), looping the path until interrupted.
//! Shared RealFlight wiring lives in `examples/common`.
//!
//! ```text
//! cargo run --example copter-auto-realflight
//! ```

mod common;

use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};

fn main() {
    let commander = ScriptCommander::new("flights/figure_eight.csv")
        .expect("failed to load figure-eight script");
    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());

    common::run_realflight(commander, stabilizer);
}
