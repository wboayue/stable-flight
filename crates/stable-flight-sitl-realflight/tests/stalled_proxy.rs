//! A proxy that accepts and then never answers can't hang the vehicle: every bridge call
//! times out, and the reconnect path runs and fails instead of blocking the tick.

use std::io::{Read, Write};
use std::net::TcpListener;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{Actuator, ActuatorCommand, StateSource};
use stable_flight_sitl_realflight::realflight_bridge::{BridgeError, RealFlightRemoteBridge};
use stable_flight_sitl_realflight::{
    ActuatorMapping, RealFlightConfiguration, RealFlightVehicle, StartError,
};

/// Generous next to the 100 ms I/O timeout per call; a hang never returns.
const WATCHDOG: Duration = Duration::from_secs(5);

/// Past the first reconnect backoff (0.5 s).
const BACKOFF_ELAPSED: Duration = Duration::from_millis(600);

/// The proxy's reply to a call that returns nothing: a 4-byte big-endian length, then the
/// postcard `Response { status: Success, payload: None }` (two variant indexes of 0), as
/// realflight-bridge's `bridge::wire` frames it (private there, so encoded by hand).
const SUCCESS_FRAME: [u8; 6] = [0, 0, 0, 2, 0, 0];

/// Accepts connections and holds them open. On the first, answers the first `answered`
/// requests with success (the startup `disable_rc` and reset); after that, and on every later
/// connection, never reads or writes.
fn stalling_proxy(answered: usize) -> String {
    let listener = TcpListener::bind("127.0.0.1:0").expect("binds");
    let address = listener.local_addr().expect("has an address").to_string();
    thread::spawn(move || {
        let mut held = Vec::new();
        for (i, stream) in listener.incoming().enumerate() {
            let Ok(mut stream) = stream else { continue };
            if i == 0 {
                for _ in 0..answered {
                    let mut header = [0u8; 4];
                    stream.read_exact(&mut header).expect("request header");
                    let mut request = vec![0u8; u32::from_be_bytes(header) as usize];
                    stream.read_exact(&mut request).expect("request");
                    stream.write_all(&SUCCESS_FRAME).expect("reply");
                }
            }
            held.push(stream);
        }
    });
    address
}

#[test]
fn stalled_proxy_times_out_instead_of_blocking_the_tick() {
    let address = stalling_proxy(2);
    let reconnects = Arc::new(AtomicUsize::new(0));
    let (done, finished) = mpsc::channel();

    let attempts = Arc::clone(&reconnects);
    thread::spawn(move || {
        let host = address.clone();
        let mut vehicle = RealFlightVehicle::try_new(RealFlightConfiguration {
            bridge: RealFlightRemoteBridge::new(&address).expect("connects"),
            mapping: ActuatorMapping::default(),
            body_rotation: UnitQuaternion::identity(),
        })
        .expect("the proxy answers the startup calls (SUCCESS_FRAME matches the wire format)")
        .with_reconnect(move || {
            attempts.fetch_add(1, Ordering::SeqCst);
            RealFlightRemoteBridge::new(&host).map_err(BridgeError::from)
        });

        // The exchange times out, so the connection is down; once the backoff has passed, the
        // next tick reconnects, and the new bridge's `disable_rc` times out too.
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

#[test]
fn stalled_proxy_at_startup_fails_the_start() {
    let address = stalling_proxy(0);
    let (done, finished) = mpsc::channel();

    thread::spawn(move || {
        // `disable_rc` times out; `reset_aircraft` then finds the bridge out of sync.
        let result = RealFlightVehicle::try_new(RealFlightConfiguration {
            bridge: RealFlightRemoteBridge::new(&address).expect("connects"),
            mapping: ActuatorMapping::default(),
            body_rotation: UnitQuaternion::identity(),
        });
        done.send(matches!(result, Err(StartError::Reset(_))))
            .expect("test is waiting");
    });

    let failed_on_reset = finished
        .recv_timeout(WATCHDOG)
        .expect("a stalled proxy blocked the start");
    assert!(failed_on_reset);
}
