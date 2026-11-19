//! Flies a quadcopter in RealFlight from the native link (`stable-flight-link`, UDP), as a
//! companion computer would: the link's commands fly it, and a failsafe holds position when the
//! link goes quiet (or finishes a land it was flying), handing control back once it has been
//! back for a second.
//!
//! The commander is `FailsafeCommander<LinkCommander, HoldCommander>`: the link arms and
//! disarms (`Control::Arm`) and says what to fly (`Control::Command`); a hold never arms. Params
//! and telemetry go over the same link. The link binds `--link-bind` and talks to the companion
//! at `--link-peer` (the bridge's localhost ports by default). Drive it with
//! `stable-flight-tools`' `link_pilot`, which plays the companion: stop the pilot mid-flight
//! and the quad holds. The RealFlight host defaults to `10.211.55.3:8080`; override it with
//! `--host`.
//!
//! The link also starts and stops an autotune (`Uplink::Autotune`, from the bridge's
//! `MAV_CMD_DO_AUTOTUNE_ENABLE` or `link_pilot --autotune`): the copter design search, flown
//! from an `AutotuneHold` where the quad was, since a ground station's sticks hold no height.
//! The roll or pitch stick takes over and reverts the run; a disarm before completion reverts
//! it too. On completion the tuned gains fly and print for pasting into the airframe; a stop
//! afterwards, in the air or landed, puts the start gains back. Each step goes down the link
//! as an `AutotuneReport`.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight
//! cargo run --bin link_pilot   # in a second terminal
//! ```

use std::time::Duration;

use clap::Parser;
use env_logger::Builder;
use log::{debug, info, warn};

use stable_flight::commanders::{
    AutotuneHold, AutotuneHoldConfig, FailsafeCommander, FailsafeConfig, HoldCommander,
};
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::tuning::autotune::{Autotune, Envelope, StagePlan};
use stable_flight::{Commander, Configurable, FailsafeState, FlightLoop};
use stable_flight_link::autotune::{AutotuneProgress, AutotuneReport, AutotuneRequest};
use stable_flight_link::fc::{FcLink, FcLinkConfig};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_sitl::autotune::log_event;
use stable_flight_sitl::{RealTimeRunner, ShutdownSignal};
use stable_flight_sitl_realflight::realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl_realflight::{
    REMOTE_IO_TIMEOUT, RealFlightConfiguration, RealFlightVehicle, airframes,
};

/// Connect timeout for the RealFlight proxy; a reconnect attempt blocks its tick.
const CONNECT_TIMEOUT: Duration = Duration::from_millis(250);

/// Loop rate, in Hz, and its tick, in s.
const LOOP_HZ: f64 = 250.0;
const LOOP_DT_S: f32 = (1.0 / LOOP_HZ) as f32;

/// A gap between ticks this long means the host froze, spoiling a running probe's timing (as
/// in `copter-autotune-realflight`).
const FREEZE_GAP: Duration = Duration::from_millis(50);

/// The example that tunes, heading the printed gains.
const SOURCE: &str = "copter-companion-realflight";

/// Quadcopter flight in RealFlight from the native link, holding when the link goes quiet.
#[derive(Parser)]
struct Args {
    /// RealFlight host to connect to (`host:port`).
    #[arg(long, default_value = "10.211.55.3:8080")]
    host: String,
    /// Local address the link listens on.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_bind: String,
    /// The companion's link address.
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_peer: String,
}

fn main() {
    Builder::from_default_env().format_timestamp_millis().init();

    let args = Args::parse();

    // QuadCopterX model: tuned gains, mixer, and motor-to-channel mapping.
    let airframe = airframes::quadcopter_x();

    // A lost or stalled RealFlight connection is dropped and rebuilt with backoff; short
    // timeouts bound each attempt's stall.
    let host = args.host.clone();
    let connect =
        move || RealFlightRemoteBridge::with_timeouts(&host, CONNECT_TIMEOUT, REMOTE_IO_TIMEOUT);
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration::from_airframe(
        &airframe,
        connect().expect("Failed to create bridge"),
    ))
    .with_reconnect(connect);

    // Waypoint commands from the link become attitude setpoints here.
    let guidance = CopterGuidance::new(airframe.guidance);
    let stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let mixer = airframe.mixer;

    // The link: the publisher writes it each tick (the flight loop's telemetry), the poller
    // reads it and serves params between ticks, and the link commander flies from its control
    // messages. Queue sizes: 32 param requests, 16 replies, 8 control messages (the default).
    let (link_rx, link_tx) = UdpLink::bind(&args.link_bind, &args.link_peer)
        .expect("failed to open link socket")
        .into_halves();
    let mut fc_link = FcLink::<32, 16>::new();
    let (mut poller, link, link_commander) =
        fc_link.wire_linked(link_rx, link_tx, FcLinkConfig::default());

    // Hold position after 1 s without a message; hand back after 1 s of messages. Around
    // it, the hold an autotune flies from.
    let failsafe = FailsafeCommander::new(
        link_commander,
        HoldCommander::default(),
        FailsafeConfig::default(),
    )
    .expect("valid failsafe config");
    let commander =
        AutotuneHold::new(failsafe, AutotuneHoldConfig::default()).expect("valid hold config");

    let mut flight_loop = FlightLoop::new(commander, vehicle, guidance, stabilizer, mixer, link);

    // Ctrl-C stops the loop cleanly (RealFlightVehicle re-enables RC control on drop).
    let shutdown = ShutdownSignal::install().expect("failed to install Ctrl-C handler");

    let plan = StagePlan::copter_design_search();
    let mut autotune = Autotune::new(&plan, Envelope::default()).expect("valid autotune plan");

    let mut failsafe = None;
    let mut last_tick = Duration::ZERO;
    let runner = RealTimeRunner::new(LOOP_HZ).expect("valid loop frequency");
    runner.run_with(&mut flight_loop, &shutdown, |flight_loop, stats| {
        match poller.poll(flight_loop) {
            Ok(stats) if stats.decode_errors > 0 || stats.dropped > 0 => {
                debug!("link: {stats:?}");
            }
            Ok(_) => {}
            Err(e) => warn!("link receive failed: {e}"),
        }

        // The autotune: the host's timing in, the link's requests, the pilot's override, then
        // a step of the run, each reported down the link.
        if stats.elapsed.saturating_sub(last_tick) > FREEZE_GAP {
            autotune.note_stall();
        }
        last_tick = stats.elapsed;
        autotune.set_host_slow(stats.host_slow);
        autotune.set_state_stale(flight_loop.vehicle().state_is_stale());
        let mut reports = Vec::new();
        while let Some(request) = poller.autotune_request() {
            let progress = match request {
                AutotuneRequest::Start => match autotune.start(flight_loop) {
                    Ok(()) => {
                        info!("autotune: started from the link");
                        let state = flight_loop.state().clone();
                        flight_loop.commander_mut().engage(&state);
                        AutotuneProgress::Started
                    }
                    Err(e) => {
                        warn!("autotune: can't start: {e}");
                        AutotuneProgress::Refused(e)
                    }
                },
                AutotuneRequest::Stop => {
                    flight_loop.commander_mut().release();
                    match autotune.revert(flight_loop) {
                        Some(_) => {
                            info!("autotune: stopped from the link; start gains back");
                            AutotuneProgress::Reverted
                        }
                        None => AutotuneProgress::Idle,
                    }
                }
            };
            reports.push(AutotuneReport::new(progress, autotune.status(), plan.len()));
        }
        if flight_loop.commander_mut().take_override() && autotune.revert(flight_loop).is_some() {
            info!("autotune: the sticks took over; start gains back");
            let status = autotune.status();
            reports.push(AutotuneReport::new(
                AutotuneProgress::Reverted,
                status,
                plan.len(),
            ));
        }
        if let Some(event) = autotune.update(flight_loop, LOOP_DT_S) {
            log_event(&event, flight_loop.stabilizer().config(), SOURCE);
            if !autotune.is_running() {
                flight_loop.commander_mut().release();
            }
            let status = autotune.status();
            reports.extend(AutotuneReport::from_event(&event, status, plan.len()));
        }
        for report in reports {
            if !poller.report_autotune(report) {
                warn!("autotune: report queue full, dropped {report:?}");
            }
        }

        // Log each handover (lifecycle events), not every tick.
        let state = flight_loop.commander().failsafe();
        if state != failsafe {
            match state {
                Some(FailsafeState::Fallback)
                    if flight_loop.commander().inner().fallback().is_landing() =>
                {
                    info!("link quiet: landing");
                }
                Some(FailsafeState::Fallback) => info!("link quiet: holding position"),
                Some(FailsafeState::AwaitingSticks) => {
                    info!("link back: holding until the sticks move");
                }
                Some(FailsafeState::Primary) => info!("link back: flying from the link"),
                _ => {}
            }
            failsafe = state;
        }
    });
}
