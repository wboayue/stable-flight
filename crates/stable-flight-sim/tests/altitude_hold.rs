//! Altitude hold (`ControlInput::AltitudeHold`) on the sim's `quad_x`, which hovers at thrust
//! 0.84 as RealFlight's QuadCopterX does at 0.82: it idles on the ground until the climb stick
//! rises, takes off, holds its height on a centered stick, climbs and sinks at the configured
//! rates, stops near where the stick is let go, and idles again after a descent the ground
//! stops. A ground station's virtual joystick flies this way through the bridge (#441).

mod common;

use std::cell::Cell;
use std::rc::Rc;

use assert2::{assert, check};
use stable_flight::guidance::copter::{DEFAULT_MAX_CLIMB_MPS, DEFAULT_MAX_SINK_MPS};
use stable_flight::{AltitudeHoldCommand, Command, Commander, ControlInput, VehicleState};
use stable_flight_sim::airframes;

use common::{Plant, SimLoop};

/// The climb stick, set between ticks.
#[derive(Clone, Default)]
struct ClimbStick(Rc<Cell<f32>>);

impl Commander for ClimbStick {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        Command::armed(ControlInput::AltitudeHold(AltitudeHoldCommand {
            climb: self.0.get(),
            ..Default::default()
        }))
    }
}

/// How far a centered stick may let the height move over 30 s, in m: it moved 0.009 m on the
/// sweep (#441); 0.05 m leaves margin for the height it settles from.
const MAX_HOLD_DRIFT_M: f32 = 0.05;

/// How far the achieved full-stick rate may miss the configured one, in m/s: the sweep
/// measured 2.54 up and 1.55 down against 2.5 and 1.5 (#441); 0.15 m/s of margin.
const RATE_TOLERANCE_MPS: f32 = 0.15;

/// How far the vehicle may sink past where a full-down stick is let go, in m: 0.62 m on the
/// sweep and 0.65 m braking to the height lock (#441), with 0.25 m of margin. A sink past this is the direction that ends in the
/// ground (#343).
const MAX_SINK_OVERSHOOT_M: f32 = 0.9;

/// How long after a touchdown the motors may still run, in s: the touchdown detector holds
/// 1 s (`TouchdownConfig::default`), and the sweep idled 1.02–1.03 s after contact (#441).
const MAX_IDLE_DELAY_S: f32 = 1.3;

fn flight() -> (SimLoop<ClimbStick>, ClimbStick) {
    let stick = ClimbStick::default();
    let flight_loop = common::sim_loop_with(&Plant::clean(airframes::quad_x()), stick.clone());
    (flight_loop, stick)
}

/// The height after flying `seconds`.
fn height_after(flight_loop: &mut SimLoop<ClimbStick>, seconds: f32) -> f32 {
    common::fly(flight_loop, seconds, |_, _| {});
    flight_loop.state().position_m.z
}

/// Armed on the ground with the stick centered or down, the motors idle; a climb stick takes
/// off, a centered one then holds the height.
#[test]
fn idles_on_the_ground_then_takes_off_and_holds() {
    let (mut flight_loop, stick) = flight();
    for climb in [0.0, -1.0] {
        stick.0.set(climb);
        common::fly(&mut flight_loop, 2.0, |l, _| {
            assert!(l.setpoint().idle(), "climb stick {climb}");
        });
        check!(flight_loop.state().position_m.z.abs() < 0.01);
    }

    stick.0.set(1.0);
    let climbed_m = height_after(&mut flight_loop, 4.0);
    check!(climbed_m > 5.0, "climbed {climbed_m} m in 4 s");
    stick.0.set(0.0);
    let settled_m = height_after(&mut flight_loop, 10.0);
    let mut drift_m: f32 = 0.0;
    common::fly(&mut flight_loop, 30.0, |l, _| {
        drift_m = drift_m.max((l.state().position_m.z - settled_m).abs());
    });
    check!(drift_m < MAX_HOLD_DRIFT_M);
}

/// Full stick climbs and sinks at the configured rates; let go after a sink, it stops close to
/// where the stick was released.
#[test]
fn full_stick_climbs_and_sinks_at_the_configured_rates() {
    let (mut flight_loop, stick) = flight();
    stick.0.set(1.0);
    height_after(&mut flight_loop, 6.0);
    stick.0.set(0.0);
    height_after(&mut flight_loop, 10.0);

    // (stick, configured rate, up positive)
    for (climb, rate_mps) in [(1.0, DEFAULT_MAX_CLIMB_MPS), (-1.0, -DEFAULT_MAX_SINK_MPS)] {
        stick.0.set(climb);
        // A second to reach the rate (2.5 m/s² from rest), then two at it.
        let start_m = height_after(&mut flight_loop, 1.5);
        let end_m = height_after(&mut flight_loop, 2.0);
        let achieved_mps = (end_m - start_m) / 2.0;
        check!(
            (achieved_mps - rate_mps).abs() < RATE_TOLERANCE_MPS,
            "stick {climb}: {achieved_mps} m/s"
        );
        stick.0.set(0.0);
        let mut low_m = end_m;
        common::fly(&mut flight_loop, 8.0, |l, _| {
            low_m = low_m.min(l.state().position_m.z);
        });
        if climb < 0.0 {
            check!(end_m - low_m < MAX_SINK_OVERSHOOT_M);
        }
    }
}

/// A full-down stick lands the vehicle, and the motors idle once the ground stops it.
#[test]
fn a_descent_to_the_ground_idles_there() {
    let (mut flight_loop, stick) = flight();
    stick.0.set(1.0);
    height_after(&mut flight_loop, 3.0);
    stick.0.set(0.0);
    height_after(&mut flight_loop, 5.0);

    stick.0.set(-1.0);
    let (mut contact_s, mut idle_s) = (None, None);
    common::fly(&mut flight_loop, 30.0, |l, t| {
        if contact_s.is_none() && l.state().position_m.z < 0.05 {
            contact_s = Some(t);
        }
        if idle_s.is_none() && l.setpoint().idle() {
            idle_s = Some(t);
        }
    });
    let contact_s = contact_s.expect("reached the ground");
    let idle_s = idle_s.expect("idled");
    check!(idle_s >= contact_s);
    check!(idle_s - contact_s < MAX_IDLE_DELAY_S);
}
