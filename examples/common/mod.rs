//! Shared wiring for the RealFlight copter examples.
//!
//! The examples differ only in their commander and stabilizer; everything else — the
//! RealFlight vehicle, mixer, telemetry, orchestrator, and run loop — is identical and
//! lives here so each example stays thin.

// Each example includes this module via `mod common;`; not every example uses every item.
#![allow(dead_code)]

use env_logger::Builder;

use realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::telemetry::mavlink::{MavlinkConfiguration, MavlinkPublisher};
use stable_flight_sitl::{Commander, FlightOrchestrator, Stabilizer};

/// Control-loop frequency for the RealFlight examples (Hz).
const TARGET_FREQUENCY: f64 = 250.0;

/// Wires a `commander` and `stabilizer` to a RealFlight QuadCopterX and flies the control
/// loop until interrupted (SIGINT / Ctrl-C).
///
/// The actuator mapping and bridge address match the ArduPilot SITL QuadCopterX model:
/// <https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX>
pub fn run_realflight<C: Commander, S: Stabilizer>(commander: C, stabilizer: S) {
    Builder::from_default_env().format_timestamp_millis().init();

    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
        mapping: ActuatorMapping {
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    let mixer = QuadCopterMixer::default();
    let telemetry = MavlinkPublisher::new(MavlinkConfiguration {});

    let mut orchestrator =
        FlightOrchestrator::new(commander, vehicle, stabilizer, mixer, telemetry);
    orchestrator.run(TARGET_FREQUENCY);
}
