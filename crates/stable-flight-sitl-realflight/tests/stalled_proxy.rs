//! A proxy that accepts and then never answers can't hang the vehicle (#250): every bridge call
//! times out, and the reconnect path runs and fails instead of blocking the tick.

use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{Actuator, ActuatorCommand, StateSource};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{ActuatorMapping, RealFlightConfiguration, RealFlightVehicle};

/// Generous next to the 100 ms I/O timeout per call; a hang never returns.
const WATCHDOG: Duration = Duration::from_secs(5);

/// Past the first reconnect backoff (0.5 s).
const BACKOFF_ELAPSED: Duration = Duration::from_millis(600);

/// Accepts connections and holds them open without reading or writing.
fn silent_proxy() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").expect("binds");
    let address = listener.local_addr().expect("has an address").to_string();
    thread::spawn(move || {
        let mut held = Vec::new();
        for stream in listener.incoming() {
            held.push(stream);
        }
    });
    address
}

#[test]
fn stalled_proxy_times_out_instead_of_blocking_the_tick() {
    let address = silent_proxy();
    let reconnects = Arc::new(AtomicUsize::new(0));
    let (done, finished) = mpsc::channel();

    let attempts = Arc::clone(&reconnects);
    thread::spawn(move || {
        let host = address.clone();
        // `disable_rc` times out here; `reset_aircraft` then finds the bridge out of sync.
        let mut vehicle = RealFlightVehicle::new(RealFlightConfiguration {
            bridge: RealFlightRemoteBridge::new(&address).expect("connects"),
            mapping: ActuatorMapping::default(),
            body_rotation: UnitQuaternion::identity(),
        })
        .with_reconnect(move || {
            attempts.fetch_add(1, Ordering::SeqCst);
            RealFlightRemoteBridge::new(&host).map_err(BridgeError::from)
        });

        // The exchange fails, so the connection is down; once the backoff has passed, the next
        // tick reconnects, and the new bridge's `disable_rc` times out too.
        vehicle.actuate(&ActuatorCommand::default());
        vehicle.state(BACKOFF_ELAPSED);
        vehicle.actuate(&ActuatorCommand::default());

        done.send(()).expect("test is waiting");
    });

    finished
        .recv_timeout(WATCHDOG)
        .expect("a stalled proxy blocked the vehicle");
    assert_eq!(reconnects.load(Ordering::SeqCst), 1);
}
