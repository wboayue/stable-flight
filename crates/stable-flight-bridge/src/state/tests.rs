use assert2::check;
use mavlink::Message;
use stable_flight_link::stable_flight::Saturation;

use super::*;
use crate::test_support::{at_altitude, earth, home_zs, landed_states};

const STREAM: Duration = Duration::from_millis(100);
const SLOW: Duration = Duration::from_secs(1);
const MS: Duration = Duration::from_millis(1);

/// A service whose flight controller appeared at `t0`.
fn service(earth: Option<FlatEarth>, t0: Instant) -> StateService {
    let mut service = StateService::new(earth, STREAM, SLOW);
    service.fc_appeared(t0, &mut Outbox::default());
    service
}

/// The names of the messages a poll at `now` sends.
fn poll_names(service: &mut StateService, now: Instant) -> Vec<&'static str> {
    let mut out = Outbox::default();
    service.poll(now, &mut out);
    check!(out.to_fc.is_empty());
    out.to_gcs.iter().map(|m| m.message_name()).collect()
}

/// What a stream poll sends, in order: with global position or without, then the slow
/// messages.
fn stream(global: bool, slow: &[&'static str]) -> Vec<&'static str> {
    let position = ["ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"];
    let saturation = ["NAMED_VALUE_INT"; stable_flight_mavlink::SATURATION_FLAGS];
    let position = if global {
        &position[..]
    } else {
        &position[..2]
    };
    [position, &["VFR_HUD"], &saturation, slow].concat()
}

/// Everything, as the first poll after a state sends it.
fn all() -> Vec<&'static str> {
    stream(true, &["HOME_POSITION", "EXTENDED_SYS_STATE"])
}

#[test]
fn sends_nothing_until_the_first_state_then_every_stream_at_once() {
    let t0 = Instant::now();
    let mut service = service(Some(earth()), t0);

    check!(poll_names(&mut service, t0).is_empty(), "no state yet");
    check!(poll_names(&mut service, t0 + 500 * MS).is_empty());
    service.on_state(&at_altitude(10.0), false, ControlMode::Attitude);
    check!(
        poll_names(&mut service, t0 + 500 * MS) == all(),
        "every stream on the first state"
    );
}

#[test]
fn sends_each_stream_at_its_rate() {
    let t0 = Instant::now();
    let mut service = service(Some(earth()), t0);
    service.on_state(&at_altitude(10.0), false, ControlMode::Attitude);

    // Two seconds of 4 ms polls (the bridge's busiest pace), with no home or landed changes.
    let mut sent = Vec::new();
    for tick in 0..500 {
        sent.extend(poll_names(&mut service, t0 + tick * 4 * MS));
    }
    // (message, expected sends in 2 s: the first poll plus one per period)
    let cases = [
        ("ATTITUDE", 20),
        ("LOCAL_POSITION_NED", 20),
        ("GLOBAL_POSITION_INT", 20),
        ("VFR_HUD", 20),
        (
            "NAMED_VALUE_INT",
            20 * stable_flight_mavlink::SATURATION_FLAGS,
        ),
        ("HOME_POSITION", 2),
        ("EXTENDED_SYS_STATE", 2),
    ];
    for (name, expected) in cases {
        check!(
            sent.iter().filter(|n| **n == name).count() == expected,
            "{name}"
        );
    }
}

#[test]
fn sends_no_global_position_or_home_without_an_origin() {
    let t0 = Instant::now();
    let mut service = service(None, t0);
    service.on_state(&at_altitude(10.0), false, ControlMode::Attitude);
    check!(poll_names(&mut service, t0) == stream(false, &["EXTENDED_SYS_STATE"]));
}

#[test]
fn sends_home_and_landed_state_as_soon_as_they_change() {
    use ControlMode::{Attitude as FLY, Land as LAND};
    use MavLandedState::{
        MAV_LANDED_STATE_IN_AIR as AIR, MAV_LANDED_STATE_LANDING as LANDING,
        MAV_LANDED_STATE_ON_GROUND as GROUND,
    };
    // (armed, mode, altitude_m, expected (HOME_POSITION z, landed state) sent by the next
    // poll). Polls are 10 ms apart after the first, so neither 1 Hz timer fires again.
    let ticks = [
        (false, FLY, 10.0, (Some(-10.0), Some(GROUND))), // the first poll: everything
        (false, FLY, 12.0, (None, None)),                // moved while disarmed
        (true, FLY, 12.0, (Some(-12.0), None)),          // arming re-latches home
        (true, FLY, 13.0, (None, Some(AIR))),            // took off
        (true, FLY, 13.5, (None, None)),
        (true, LAND, 13.5, (None, Some(LANDING))), // a land
        (true, LAND, 12.1, (None, Some(GROUND))),  // landed
    ];

    let t0 = Instant::now();
    let mut service = service(Some(earth()), t0);
    for (i, (armed, mode, altitude_m, (home_z, landed))) in ticks.into_iter().enumerate() {
        service.on_state(&at_altitude(altitude_m), armed, mode);
        let mut out = Outbox::default();
        service.poll(t0 + i as u32 * 10 * MS, &mut out);
        check!(home_zs(&out.to_gcs) == Vec::from_iter(home_z), "tick {i}");
        check!(
            landed_states(&out.to_gcs) == Vec::from_iter(landed),
            "tick {i}"
        );
    }
}

#[test]
fn sends_the_latest_state_with_time_since_the_flight_controller_appeared() {
    let t0 = Instant::now();
    let mut service = service(None, t0);
    // Several states between polls: only the latest goes out.
    for roll_deg in [1.0_f32, 2.0, 10.0] {
        service.on_state(
            &VehicleState {
                roll_rad: roll_deg.to_radians(),
                ..Default::default()
            },
            false,
            ControlMode::Attitude,
        );
    }
    let mut out = Outbox::default();
    service.poll(t0 + 250 * MS, &mut out);

    let [MavMessage::ATTITUDE(attitude), ..] = out.to_gcs.as_slice() else {
        panic!("expected ATTITUDE first, got {:?}", out.to_gcs);
    };
    check!(
        attitude.roll == 10_f32.to_radians(),
        "a 10° roll reads 10° (#102)"
    );
    check!(attitude.time_boot_ms == 250);
}

fn output(thrust: f32, saturation: Saturation) -> Output {
    Output { thrust, saturation }
}

/// `VFR_HUD` throttle and the saturation flags set, by name, from a poll at `at`.
fn throttle_and_saturation(service: &mut StateService, at: Instant) -> (Option<u16>, Vec<String>) {
    let mut out = Outbox::default();
    service.poll(at, &mut out);
    let throttle = out.to_gcs.iter().find_map(|m| match m {
        MavMessage::VFR_HUD(d) => Some(d.throttle),
        _ => None,
    });
    let set = out
        .to_gcs
        .iter()
        .filter_map(|m| match m {
            MavMessage::NAMED_VALUE_INT(d) if d.value == 1 => {
                Some(d.name.to_str().expect("ASCII").to_owned())
            }
            _ => None,
        })
        .collect();
    (throttle, set)
}

#[test]
fn vfr_hud_throttle_is_the_latest_outputs_thrust() {
    let t0 = Instant::now();
    let mut service = service(None, t0);
    service.on_state(&VehicleState::default(), true, ControlMode::Attitude);

    check!(
        throttle_and_saturation(&mut service, t0).0 == Some(0),
        "no output yet"
    );
    service.on_output(&output(0.3, Saturation::default()));
    service.on_output(&output(0.62, Saturation::default()));
    check!(throttle_and_saturation(&mut service, t0 + STREAM).0 == Some(62));

    service.fc_appeared(t0 + 5 * SLOW, &mut Outbox::default());
    service.on_state(&VehicleState::default(), true, ControlMode::Attitude);
    check!(
        throttle_and_saturation(&mut service, t0 + 5 * SLOW).0 == Some(0),
        "a restarted flight controller has sent no output"
    );
}

#[test]
fn reports_every_saturation_since_the_last_stream_once() {
    let mut tilt = Saturation::default();
    tilt.tilt = true;
    let mut yaw = Saturation::default();
    yaw.moments[2] = true;
    let none = Saturation::default();
    // (outputs before the poll, poll offset from t0, expected flags set)
    let polls: [(&[Saturation], Duration, &[&str]); 5] = [
        (&[], Duration::ZERO, &[]),
        (&[tilt, none, yaw, none], STREAM, &["SAT_TILT", "SAT_YAW"]), // ORed
        (&[tilt], STREAM + 10 * MS, &[]),                             // stream not due: held
        (&[none], 2 * STREAM, &["SAT_TILT"]),
        (&[none], 3 * STREAM, &[]), // cleared by the previous stream
    ];

    let t0 = Instant::now();
    let mut service = service(None, t0);
    service.on_state(&VehicleState::default(), true, ControlMode::Attitude);
    for (i, (outputs, at, expected)) in polls.into_iter().enumerate() {
        for saturation in outputs {
            service.on_output(&output(0.5, *saturation));
        }
        check!(
            throttle_and_saturation(&mut service, t0 + at).1 == expected,
            "poll {i}"
        );
    }

    service.on_output(&output(0.5, tilt));
    service.fc_appeared(t0 + 5 * SLOW, &mut Outbox::default());
    service.on_state(&VehicleState::default(), true, ControlMode::Attitude);
    check!(
        throttle_and_saturation(&mut service, t0 + 5 * SLOW)
            .1
            .is_empty(),
        "a restarted flight controller starts clear"
    );
}

#[test]
fn starts_over_when_the_flight_controller_comes_back() {
    let t0 = Instant::now();
    let mut service = service(Some(earth()), t0);
    service.on_state(&at_altitude(10.0), true, ControlMode::Attitude);
    poll_names(&mut service, t0);

    let back = t0 + 10 * SLOW;
    service.fc_appeared(back, &mut Outbox::default());
    check!(poll_names(&mut service, back).is_empty(), "no state since");

    service.on_state(&at_altitude(30.0), false, ControlMode::Attitude);
    let mut out = Outbox::default();
    service.poll(back, &mut out);
    let names: Vec<_> = out.to_gcs.iter().map(|m| m.message_name()).collect();
    check!(names == all());
    check!(home_zs(&out.to_gcs) == [-30.0], "home latched afresh");
    let time_boot_ms = out.to_gcs.iter().find_map(|m| match m {
        MavMessage::ATTITUDE(d) => Some(d.time_boot_ms),
        _ => None,
    });
    check!(
        time_boot_ms == Some(10_000),
        "time keeps counting from the first appearance"
    );
}

#[test]
fn position_is_the_latest_finite_one_since_the_flight_controller_appeared() {
    let t0 = Instant::now();
    let mut service = service(None, t0);
    let at = |x: f32| VehicleState {
        position_m: [x, 4.0, 10.0].into(),
        ..Default::default()
    };
    check!(service.position_m() == None, "no state yet");
    service.on_state(&at(3.0), true, ControlMode::Attitude);
    check!(service.position_m() == Some(Vec2::new(3.0, 4.0)));
    service.on_state(&at(f32::NAN), true, ControlMode::Attitude);
    check!(service.position_m() == None, "not finite");
    service.on_state(&at(3.0), true, ControlMode::Attitude);
    service.fc_appeared(t0, &mut Outbox::default());
    check!(service.position_m() == None, "starts over");
}
