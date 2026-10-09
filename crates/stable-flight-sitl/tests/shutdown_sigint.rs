//! SIGINT (Ctrl-C) requests a graceful stop. A test binary of its own: the signal is
//! process-wide, and a second stop signal would exit the process.
#![cfg(unix)]

use signal_hook::consts::SIGINT;
use stable_flight_sitl::ShutdownSignal;

#[test]
fn sigint_requests_a_graceful_stop() {
    let shutdown = ShutdownSignal::install().expect("handler registers");
    assert!(!shutdown.is_set());

    // Without the handler, SIGINT's default action would kill this test process.
    signal_hook::low_level::raise(SIGINT).expect("raises");

    assert!(shutdown.is_set());
}
