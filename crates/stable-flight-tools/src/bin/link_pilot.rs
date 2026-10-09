//! Flies a flight controller over the native link (`stable-flight-link`, UDP) as a companion
//! would, for `copter-companion-realflight`: heartbeats at 5 Hz, an arm request, and a short
//! scripted flight as waypoint commands, relative to where the vehicle starts.
//!
//! It waits for the first `Downlink::State` and takes that position as home, then:
//!
//! 1. climbs to `--altitude` above home and hovers,
//! 2. after `--hover-s`, flies `--distance` north,
//! 3. holds there until stopped.
//!
//! Stopping it (Ctrl-C) plays a companion that died: it just goes quiet, without disarming, so
//! the flight controller's failsafe holds position. Started again, it takes the vehicle's
//! position at that moment as home and flies the script from there.
//!
//! It binds the bridge's link address, so run it instead of the bridge.
//!
//! ```text
//! cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight
//! cargo run --bin link_pilot
//! cargo run --bin link_pilot -- --altitude 8 --distance 20
//! ```

use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use stable_flight::{Command, ControlInput, Vec2, Vec3, WaypointCommand};
use stable_flight_link::udp::{DEFAULT_BRIDGE_ADDR, DEFAULT_FC_ADDR, UdpLink};
use stable_flight_link::{Control, Downlink, MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink};

/// Time between sends: heartbeat, arm, and the current waypoint.
const SEND_INTERVAL: Duration = Duration::from_millis(200);
/// Speed cap toward each waypoint, in m/s.
const CRUISE_SPEED_MPS: f32 = 2.0;

/// Flies a flight controller over the native link with a scripted waypoint flight.
#[derive(Parser)]
struct Args {
    /// Local address the link listens on (the bridge's, by default).
    #[arg(long, default_value = DEFAULT_BRIDGE_ADDR)]
    link_bind: String,
    /// The flight controller's link address.
    #[arg(long, default_value = DEFAULT_FC_ADDR)]
    link_peer: String,
    /// Hover height above home, in meters.
    #[arg(long, default_value_t = 5.0)]
    altitude: f32,
    /// How far north to fly after the hover, in meters.
    #[arg(long, default_value_t = 10.0)]
    distance: f32,
    /// How long to hover before moving, in seconds.
    #[arg(long, default_value_t = 10.0)]
    hover_s: f32,
}

/// The scripted flight, relative to home (north, east, up).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Script {
    altitude_m: f32,
    distance_m: f32,
    hover_s: f32,
}

impl Script {
    /// The waypoint to fly `elapsed_s` into the script, from `home_m` (north, east, up).
    fn waypoint(&self, home_m: Vec3, elapsed_s: f32) -> WaypointCommand {
        let north_m = if elapsed_s < self.hover_s {
            0.0
        } else {
            self.distance_m
        };
        WaypointCommand {
            position_m: Vec2::new(home_m.x + north_m, home_m.y),
            altitude_m: home_m.z + self.altitude_m,
            cruise_speed_mps: CRUISE_SPEED_MPS,
            ..Default::default()
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let script = Script {
        altitude_m: args.altitude,
        distance_m: args.distance,
        hover_s: args.hover_s,
    };
    let mut link = UdpLink::bind(&args.link_bind, &args.link_peer).map_err(|e| {
        format!(
            "failed to open link {} -> {}: {e}",
            args.link_bind, args.link_peer
        )
    })?;
    println!(
        "piloting {} from {}; waiting for the vehicle's state",
        args.link_peer, args.link_bind
    );

    let mut buf = [0u8; MAX_FRAME_LEN];
    let mut home: Option<(Vec3, Instant)> = None;
    let mut last_report = None;
    loop {
        // Read everything waiting; note home from the first state.
        let mut latest = None;
        while let Some(msg) = link.recv_message::<Downlink>(&mut buf)? {
            match msg {
                Downlink::State(state) => latest = Some(state.position_m),
                Downlink::Heartbeat { armed, link_ok, .. }
                    if last_report != Some((armed, link_ok)) =>
                {
                    println!("vehicle: armed={armed} link_ok={link_ok}");
                    last_report = Some((armed, link_ok));
                }
                _ => {}
            }
        }
        if home.is_none()
            && let Some(position_m) = latest.filter(|p| p.iter().all(|c| c.is_finite()))
        {
            println!(
                "home at ({:.1}, {:.1}, {:.1}); arming and climbing {} m",
                position_m.x, position_m.y, position_m.z, script.altitude_m
            );
            home = Some((position_m, Instant::now()));
        }

        // Heartbeat always; arm and the current waypoint once home is known. Repeating them
        // each round covers a flight controller started after the pilot.
        link.send_message(&Uplink::Control(Control::Heartbeat), &mut buf)?;
        if let Some((home_m, start)) = home {
            let waypoint = script.waypoint(home_m, start.elapsed().as_secs_f32());
            link.send_message(&Uplink::Control(Control::Arm(true)), &mut buf)?;
            // `arm` is ignored on a command: arming is `Control::Arm`'s.
            let command = Command::new(false, ControlInput::Waypoint(waypoint));
            link.send_message(&Uplink::Control(Control::Command(command)), &mut buf)?;
        }
        thread::sleep(SEND_INTERVAL);
    }
}

#[cfg(test)]
mod tests {
    use assert2::check;

    use super::*;

    #[test]
    fn hovers_above_home_then_flies_north_and_holds() {
        let script = Script {
            altitude_m: 5.0,
            distance_m: 10.0,
            hover_s: 10.0,
        };
        let home_m = Vec3::new(3.0, 4.0, 100.0);
        // (elapsed_s, expected (north, east, altitude))
        let cases = [
            (0.0, (3.0, 4.0, 105.0)),
            (9.9, (3.0, 4.0, 105.0)),
            (10.0, (13.0, 4.0, 105.0)),
            (600.0, (13.0, 4.0, 105.0)), // holds there
        ];
        for (elapsed_s, (north, east, altitude)) in cases {
            let waypoint = script.waypoint(home_m, elapsed_s);
            check!(
                (
                    waypoint.position_m.x,
                    waypoint.position_m.y,
                    waypoint.altitude_m
                ) == (north, east, altitude),
                "{elapsed_s} s"
            );
            check!(waypoint.cruise_speed_mps == CRUISE_SPEED_MPS);
            check!(waypoint.velocity_mps.is_none(), "stops on each point");
        }
    }
}
