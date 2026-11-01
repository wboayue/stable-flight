use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use assert2::{assert, check};

use super::*;
use crate::stabilizers::plane::PlaneStabilizer;
use crate::tuning::Tunable;
use crate::tuning::autotune::{AutotuneEvent, AutotuneStatus, Outcome};
use crate::tuning::plant::{
    CONTROL_RADPS2, DAMPING_PER_S, MomentMixer, Wing, wing_config as config,
};
use crate::tuning::{IdentifyConfig, StepConfig, WingDesignConfig};
use crate::{
    AbsoluteAttitudeCommand, Actuator, ActuatorCommand, AttitudeSetpoint, Command, Commander,
    Configurable, ControlInput, FlightLoop, FlightRegime, Guidance, PlaneSetpoint, StateSource,
    VehicleState,
};

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;
/// Where the wing flies, in m.
const ALTITUDE_M: f32 = 100.0;

/// Knobs the tests turn while the loop flies.
#[derive(Clone)]
struct Controls {
    wing: Rc<RefCell<Wing>>,
    airspeed_mps: Rc<Cell<Option<f32>>>,
    altitude_m: Rc<Cell<f32>>,
    /// Guidance flies a tailsitter transition ramp.
    transition: Rc<Cell<bool>>,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            wing: Rc::default(),
            airspeed_mps: Rc::new(Cell::new(Some(15.0))),
            altitude_m: Rc::new(Cell::new(ALTITUDE_M)),
            transition: Rc::default(),
        }
    }
}

/// Armed, wings level.
struct Hold;

impl Commander for Hold {
    fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
        Command::new(
            true,
            ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
                thrust: 0.5,
                ..Default::default()
            }),
        )
    }
}

/// A level plane attitude at half thrust, or a transition ramp (`Controls::transition`).
struct LevelGuidance(Controls);

impl Guidance for LevelGuidance {
    fn guide(
        &mut self,
        _dt: f32,
        _regime: FlightRegime,
        _input: &ControlInput,
        _state: &VehicleState,
    ) -> Setpoint {
        Setpoint::Plane(PlaneSetpoint {
            thrust: 0.5,
            transition: self.0.transition.get(),
            ..Default::default()
        })
    }
}

/// The [`Wing`] as a vehicle: moments in through `servos[0..3]`.
struct WingVehicle(Controls);

impl StateSource for WingVehicle {
    fn state(&mut self, _dt: Duration) -> VehicleState {
        let mut state = self.0.wing.borrow().state();
        state.airspeed_mps = self.0.airspeed_mps.get();
        state.position_m.z = self.0.altitude_m.get();
        state
    }
}

impl Actuator for WingVehicle {
    fn actuate(&mut self, command: &ActuatorCommand) {
        let servos = command.servos;
        let moments = Vec3::new(servos[0], servos[1], servos[2]);
        self.0.wing.borrow_mut().step(DT, moments);
    }
}

type WingLoop = FlightLoop<Hold, WingVehicle, LevelGuidance, PlaneStabilizer, MomentMixer, ()>;

/// An armed loop flying the wing level for a second.
fn flying() -> (WingLoop, Controls) {
    let controls = Controls::default();
    let mut flight_loop = FlightLoop::new(
        Hold,
        WingVehicle(controls.clone()),
        LevelGuidance(controls.clone()),
        PlaneStabilizer::new(config()),
        MomentMixer,
        (),
    );
    flight_loop.prime();
    for _ in 0..250 {
        flight_loop.step(Duration::from_secs_f32(DT));
    }
    (flight_loop, controls)
}

fn started(flight_loop: &WingLoop, plan: &[StagePlan<PlaneSide>]) -> PlaneAutotune {
    let mut autotune =
        PlaneAutotune::new(plan, PlaneEnvelope::for_plane(&config())).expect("valid plan");
    autotune.start(flight_loop).expect("armed and idle");
    autotune
}

type Event = AutotuneEvent<PlaneLoop, PlaneAxisGains>;

/// Steps the loop and the supervisor until the run ends or `max_s` passes, calling `poke`
/// before each tick; returns the events in order.
fn run(
    flight_loop: &mut WingLoop,
    autotune: &mut PlaneAutotune,
    max_s: f32,
    mut poke: impl FnMut(&PlaneAutotune),
) -> Vec<Event> {
    let mut events = Vec::new();
    for _ in 0..(max_s / DT) as usize {
        poke(autotune);
        flight_loop.step(Duration::from_secs_f32(DT));
        if let Some(event) = autotune.update(flight_loop, DT) {
            events.push(event);
        }
        if !autotune.is_running() {
            break;
        }
    }
    events
}

const LOOPS: [PlaneLoop; 4] = [
    PlaneLoop::RollRate,
    PlaneLoop::PitchRate,
    PlaneLoop::Roll,
    PlaneLoop::Pitch,
];

#[test]
fn a_step_test_measures_each_loop_and_keeps_its_gains() {
    let (mut flight_loop, _) = flying();
    let plan = LOOPS.map(|target| StagePlan::plane_step_test(target, &config()));
    let mut autotune = started(&flight_loop, &plan);
    let events = run(&mut flight_loop, &mut autotune, 200.0, |_| {});

    let done: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                steps,
                ..
            } => Some((*target, *gains, *outcome, steps.map(|s| s.count()))),
            _ => None,
        })
        .collect();
    assert!(done.len() == LOOPS.len(), "{events:?}");
    for ((target, gains, outcome, steps), expected) in done.into_iter().zip(LOOPS) {
        check!(target == expected);
        check!(gains == expected.gains(&config()), "{target:?}");
        check!(outcome == Outcome::Measured, "{target:?}");
        check!(steps == Some(6), "{target:?}");
    }
    check!(events.last() == Some(&AutotuneEvent::Complete));
    check!(*flight_loop.stabilizer().config() == config());
}

/// Each limit fails the stage it breaks: the wing banked or pitched past the envelope, its
/// airspeed out of the band, or sunk past the altitude loss. A missing airspeed breaks
/// nothing.
#[test]
fn breaches_fail_the_stage() {
    type Poke = fn(&Controls);
    let rows: [(&str, Poke, Option<AutotuneAbort>); 6] = [
        (
            "bank",
            |c| c.wing.borrow_mut().angles_rad.x = 1.2,
            Some(AutotuneAbort::Tilt),
        ),
        (
            "pitch",
            |c| c.wing.borrow_mut().angles_rad.y = -0.6,
            Some(AutotuneAbort::Tilt),
        ),
        (
            "slow",
            |c| c.airspeed_mps.set(Some(8.0)),
            Some(AutotuneAbort::AirspeedLow),
        ),
        (
            "fast",
            |c| c.airspeed_mps.set(Some(30.0)),
            Some(AutotuneAbort::AirspeedHigh),
        ),
        (
            "sink",
            |c| c.altitude_m.set(ALTITUDE_M - 50.0),
            Some(AutotuneAbort::AltitudeLoss),
        ),
        ("no airspeed", |c| c.airspeed_mps.set(None), None),
    ];
    for (case, poke, reason) in rows {
        let (mut flight_loop, controls) = flying();
        let mut autotune = started(
            &flight_loop,
            &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
        );
        poke(&controls);
        let events = run(&mut flight_loop, &mut autotune, 1.0, |_| {});
        let failed = events.iter().find_map(|event| match event {
            AutotuneEvent::StageFailed { reason, .. } => Some(*reason),
            _ => None,
        });
        check!(failed == reason, "{case}: {events:?}");
        check!(*flight_loop.stabilizer().config() == config(), "{case}");
    }
}

#[test]
fn a_transition_mid_stage_stops_it() {
    let (mut flight_loop, controls) = flying();
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
    );
    let events = run(&mut flight_loop, &mut autotune, 30.0, |autotune| {
        if matches!(autotune.status(), AutotuneStatus::Running { .. }) {
            controls.transition.set(true);
        }
    });
    check!(
        events.last()
            == Some(&AutotuneEvent::StageFailed {
                target: PlaneLoop::Roll,
                reason: AutotuneAbort::Interrupted,
            }),
        "{events:?}"
    );
}

#[test]
fn the_settle_reads_the_held_attitude() {
    let target = PlaneSide::attitude_target(&Setpoint::Plane(PlaneSetpoint {
        roll_rad: 0.3,
        pitch_rad: f32::NAN,
        ..Default::default()
    }));
    check!(target == Vec2::new(0.3, 0.0));
    let hover = Setpoint::Attitude(AttitudeSetpoint {
        roll_rad: 0.3,
        ..Default::default()
    });
    check!(PlaneSide::attitude_target(&hover) == Vec2::zeros());
}

#[test]
fn rejects_an_invalid_envelope() {
    let good = PlaneEnvelope::for_plane(&config());
    let rows: [(&str, PlaneEnvelope); 6] = [
        (
            "max_airspeed_mps",
            PlaneEnvelope {
                max_airspeed_mps: good.min_airspeed_mps,
                ..good
            },
        ),
        (
            "max_bank_rad",
            PlaneEnvelope {
                max_bank_rad: 0.0,
                ..good
            },
        ),
        (
            "settle.tilt_rad",
            PlaneEnvelope {
                max_pitch_rad: good.settle.tilt_rad,
                ..good
            },
        ),
        (
            "recovery_s",
            PlaneEnvelope {
                recovery_s: -1.0,
                ..good
            },
        ),
        (
            "settle_airspeed.band_mps",
            PlaneEnvelope {
                settle_airspeed: AirspeedSettle {
                    band_mps: 0.0,
                    ..good.settle_airspeed
                },
                ..good
            },
        ),
        (
            "settle_airspeed.max_change_mps2",
            PlaneEnvelope {
                settle_airspeed: AirspeedSettle {
                    max_change_mps2: f32::NAN,
                    ..good.settle_airspeed
                },
                ..good
            },
        ),
    ];
    check!(good.validate().is_ok());
    for (field, envelope) in rows {
        let error = envelope.validate().map_err(|e| e.field);
        check!(error == Err(field));
    }
}

#[test]
fn vtol_configs_hold_their_plane_side() {
    let mut quadplane = QuadPlaneStabilizerConfig::default();
    SideConfig::<PlaneSide>::side_mut(&mut quadplane).trim_airspeed_mps = 21.0;
    check!(quadplane.plane.trim_airspeed_mps == 21.0);
    let mut tailsitter = TailsitterStabilizerConfig::default();
    SideConfig::<PlaneSide>::side_mut(&mut tailsitter).trim_airspeed_mps = 19.0;
    check!(SideConfig::<PlaneSide>::side(&tailsitter).trim_airspeed_mps == 19.0);
}

/// While the commander maneuvers no probe starts and the settle doesn't time out; once it
/// stops, the stage runs.
#[test]
fn maneuvering_holds_probes_off() {
    let (mut flight_loop, _) = flying();
    let mut autotune = started(
        &flight_loop,
        &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
    );
    autotune.set_maneuvering(true);
    let timeout_s = PlaneEnvelope::for_plane(&config()).settle_timeout_s;
    let events = run(&mut flight_loop, &mut autotune, timeout_s + 5.0, |_| {});
    check!(
        matches!(autotune.status(), AutotuneStatus::Settling { .. }),
        "{events:?}"
    );
    check!(Tunable::<PlaneLoop>::probe(flight_loop.stabilizer()).is_none());

    autotune.set_maneuvering(false);
    let events = run(&mut flight_loop, &mut autotune, 60.0, |_| {});
    check!(
        events.last() == Some(&AutotuneEvent::Complete),
        "{events:?}"
    );
}

/// The design fits each axis of the wing, designs its gains on the fit over several updates,
/// and verifies them in flight: each stage tuned, its feedforward the wing's damping over its
/// control, committed.
#[test]
fn a_design_tunes_each_axis_of_the_wing() {
    let (mut flight_loop, _) = flying();
    let plan = [PlaneLoop::Roll, PlaneLoop::Pitch]
        .map(|target| StagePlan::plane_design(target, &config()));
    let mut autotune = started(&flight_loop, &plan);
    let mut computed = false;
    let events = run(&mut flight_loop, &mut autotune, 300.0, |autotune| {
        computed |= matches!(autotune.status(), AutotuneStatus::Running { .. });
    });
    assert!(
        events.last() == Some(&AutotuneEvent::Complete),
        "{events:?}"
    );
    check!(computed);
    let tuned: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                ..
            } => Some((*target, *gains, *outcome)),
            _ => None,
        })
        .collect();
    check!(tuned.len() == 2);
    let flown = *flight_loop.stabilizer().config();
    for (target, gains, outcome) in tuned {
        check!(outcome == Outcome::Tuned, "{target:?}");
        let ff = DAMPING_PER_S / CONTROL_RADPS2;
        check!(
            (gains.rate.ff / ff - 1.0).abs() < 0.05,
            "{target:?}: {gains:?}"
        );
        check!(target.gains(&flown) == gains, "{target:?}");
    }
}

/// The airspeed at trim, band, and time, in m/s, m/s, and s.
type Airspeed = fn(f32, f32, f32) -> Option<f32>;

/// A probe waits for the airspeed to hold steady near trim: off trim past the band, or
/// swinging, the settle times out and the stage fails; steady, or without air data, it runs.
#[test]
fn the_settle_waits_for_steady_airspeed_near_trim() {
    let trim_mps = config().trim_airspeed_mps;
    let band_mps = PlaneEnvelope::for_plane(&config()).settle_airspeed.band_mps;
    // (row, airspeed at time t in s, the last event)
    let rows: [(&str, Airspeed, Event); 4] = [
        (
            "steady at trim",
            |trim, _, _| Some(trim),
            AutotuneEvent::Complete,
        ),
        (
            "off trim past the band",
            |trim, band, _| Some(trim + 1.5 * band),
            AutotuneEvent::StageFailed {
                target: PlaneLoop::Roll,
                reason: AutotuneAbort::NotSettled,
            },
        ),
        (
            // Inside the band but changing by up to 1.3 m/s²: a phugoid's swing.
            "swinging inside the band",
            |trim, band, t| Some(trim + 0.9 * band * (core::f32::consts::TAU * t / 4.0).sin()),
            AutotuneEvent::StageFailed {
                target: PlaneLoop::Roll,
                reason: AutotuneAbort::NotSettled,
            },
        ),
        ("no air data", |_, _, _| None, AutotuneEvent::Complete),
    ];
    for (row, airspeed, last) in rows {
        let (mut flight_loop, controls) = flying();
        let mut autotune = started(
            &flight_loop,
            &[StagePlan::plane_step_test(PlaneLoop::Roll, &config())],
        );
        let mut t = 0.0;
        let events = run(&mut flight_loop, &mut autotune, 120.0, |_| {
            t += DT;
            controls.airspeed_mps.set(airspeed(trim_mps, band_mps, t));
        });
        check!(events.last() == Some(&last), "{row}: {events:?}");
    }
}

#[test]
fn rejects_an_invalid_design_plan() {
    let good = PlaneDesignPlan::for_plane(PlaneLoop::Roll, &config());
    let rows = [
        (
            "identify",
            PlaneDesignPlan {
                identify: IdentifyConfig {
                    duration_s: 0.0,
                    ..good.identify
                },
                ..good
            },
        ),
        (
            "design",
            PlaneDesignPlan {
                design: WingDesignConfig {
                    max_moment: 0.0,
                    ..good.design
                },
                ..good
            },
        ),
        (
            "verify",
            PlaneDesignPlan {
                verify: StepConfig {
                    duration_s: 0.0,
                    ..good.verify
                },
                ..good
            },
        ),
        (
            "max_verify_pairs",
            PlaneDesignPlan {
                max_verify_pairs: 0,
                ..good
            },
        ),
        (
            "candidates_per_update",
            PlaneDesignPlan {
                candidates_per_update: 0,
                ..good
            },
        ),
    ];
    check!(good.validate().is_ok());
    // A section's error names the section; a count's, the field.
    for (name, plan) in rows {
        let error = plan.validate().expect_err(name);
        check!(
            error.section == Some(name) || error.field == name,
            "{error:?}"
        );
    }
}

/// A design plan's targets come from the airframe's config: the axis's rate limit for the
/// identification and the design, the throw left by its trim, its integral limit and trim.
#[test]
fn a_design_plan_reads_the_airframe() {
    let mut airframe = config();
    airframe.max_attitude_rate_radps = Vec2::new(1.0, 0.5);
    airframe.rate.trim = Vec3::new(0.0, 0.25, 0.0);
    let plan = PlaneDesignPlan::for_plane(PlaneLoop::Pitch, &airframe);
    check!(plan.identify.amplitude == PLANE_IDENTIFY_SHARE * 0.5);
    check!(plan.design.max_rate_radps == 0.5);
    check!(plan.design.max_moment == PLANE_MOMENT_SHARE * 0.75);
    check!(plan.design.static_trim == 0.25);
    check!(plan.design.max_integral == airframe.rate.max_integral);
    check!(plan.verify == StepTestPlan::plane(PlaneLoop::Pitch, &airframe).step);
    check!(PlaneDesignPlan::for_plane(PlaneLoop::PitchRate, &airframe) == plan);
}
