//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
//!
//! Flies a quadcopter in RealFlight from a Logitech Extreme 3D joystick, using the
//! hand-tuned gains from [`airframes::quadcopter_x`].
//!
//! The RealFlight host defaults to `10.211.55.3:8080`; override it with `--host`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight
//! cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- --host 192.168.1.20:8080
//! ```

use clap::Parser;
use env_logger::Builder;

use stable_flight::FlightLoop;
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{RealFlightConfiguration, RealFlightVehicle, airframes};

/// Joystick-driven quadcopter flight in RealFlight.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
}

fn main() {
    // Initialize logger
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // Joystick commander drives the vehicle attitude/rate directly.
    let commander = Extreme3DCommander::default();

    // QuadCopterX model: tuned gains and its motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new(&args.host).expect("Failed to create bridge"),
        mapping: airframe.mapping,
    });

    let stabilizer = CopterStabilizer::new(airframe.stabilizer);

    let mixer = QuadCopterMixer::default();

    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut flight_loop = FlightLoop::new(commander, vehicle, stabilizer, mixer, telemetry);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let runner = RealTimeRunner::new(250.0).expect("valid loop frequency"); // Hz
    runner.run(&mut flight_loop, &shutdown);
}
