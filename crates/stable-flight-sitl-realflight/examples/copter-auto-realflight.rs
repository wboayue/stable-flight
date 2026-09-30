//! Flies a scripted figure-eight in RealFlight using a [`ScriptCommander`].
//!
//! The commander replays the waypoints in a CSV script (default `flights/figure_eight.csv`,
//! regenerate it with `cargo run --bin generate_figure_eight`), looping the
//! path until interrupted.
//!
//! The script path is the first argument (default `flights/figure_eight.csv`); the RealFlight
//! host defaults to `10.211.55.3:8080` via `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- flights/wide.csv --host 192.168.1.20:8080
//! ```

use clap::Parser;
use env_logger::Builder;

use stable_flight::FlightLoop;
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight_sitl::commanders::ScriptCommander;
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

/// Scripted figure-eight flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// Waypoint CSV to fly (columns: x,y,altitude,cruise_speed).
    #[arg(default_value = "flights/figure_eight.csv")]
    script: String,

    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Replays and loops the waypoints from the CSV script.
    let commander = ScriptCommander::new(&args.script).expect("failed to load flight script");

    // QuadCopterX model; only its channel mapping is used here.
    // Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
    let airframe = airframes::quadcopter_x();

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: airframe.mapping,
    });

    // Default gains, not `airframe.stabilizer`: this example has only flown the defaults.
    // One tuned set for both examples comes with the RealFlight frame fix.
    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());

    let mixer = QuadCopterMixer::default();

    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut flight_loop = FlightLoop::new(commander, vehicle, stabilizer, mixer, telemetry);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run(&mut flight_loop, &shutdown);
}
