//! Plane-side autotune in closed loop: the plane supervisor ([`PlaneAutotune`]) running its
//! stages on the airplane, the eXplora's plane side, and the quadplanes' cruise sides, from
//! plane flight in the attitude hold it flies in ([`PlaneTuneHold`]).

mod common;

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use stable_flight::commanders::{PlaneTuneHold, PlaneTuneHoldConfig};
use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use stable_flight::tuning::StepSummary;
use stable_flight::tuning::autotune::{
    AutotuneAbort, AutotuneEvent, AutotuneStatus, Outcome, PlaneAutotune, PlaneEnvelope,
    PlaneMethod, PlaneSide, SideTunable, StagePlan,
};
use stable_flight::{
    Command, Commander, FlightLoop, FlightMode, FlightRegime, Guidance, Mixer, Stabilizer,
    TelemetryPublisher, Vehicle, VehicleState,
};
use stable_flight_sim::airframes;

use common::hover_at;
use common::plane::{Interlude, PlanePlant, north_line, north_line_command, plane_pipeline};
use common::quadplane::{QuadPlanePlant, quadplane_pipeline_at};
use common::tailsitter::{TailsitterPlant, tailsitter_pipeline};

const DT: f32 = 0.004;

/// Seconds of plane flight on the lead before the hold takes over: the transition's wake and
/// the line's pitch settle.
const PLANE_LEAD_S: f32 = 15.0;
/// Seconds on the hold before the run starts.
const HOLD_LEAD_S: f32 = 5.0;
/// The most a lead may take to reach plane flight, in s.
const MAX_LEAD_S: f32 = 200.0;
/// The eXplora climbs this high on its rotors before transitioning, in m.
const EXPLORA_ALTITUDE_M: f32 = 120.0;

/// Flies `lead` until handed to the hold, then the shared [`PlaneTuneHold`].
struct LeadThenHold {
    lead: Box<dyn Commander>,
    hold: Rc<RefCell<PlaneTuneHold>>,
    holding: Rc<RefCell<bool>>,
}

impl Commander for LeadThenHold {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        if *self.holding.borrow() {
            return self.hold.borrow_mut().next_command(dt, state);
        }
        self.lead.next_command(dt, state)
    }
}

/// An airframe whose plane side the tests tune.
#[derive(Clone, Copy, Debug)]
enum Subject {
    Plane,
    Explora,
    Cobra,
    Alia,
}

const SUBJECTS: [Subject; 4] = [
    Subject::Plane,
    Subject::Explora,
    Subject::Cobra,
    Subject::Alia,
];

impl Subject {
    fn plane_config(self) -> PlaneStabilizerConfig {
        match self {
            Subject::Plane => airframes::plane().stabilizer,
            Subject::Explora => airframes::explora().stabilizer.plane,
            Subject::Cobra => airframes::quadplane().stabilizer.plane,
            Subject::Alia => airframes::alia250().stabilizer.plane,
        }
    }

    fn hold_config(self) -> PlaneTuneHoldConfig {
        let (thrust, recover_to) = match self {
            Subject::Plane => (
                airframes::plane().guidance.cruise_thrust,
                FlightRegime::Plane,
            ),
            Subject::Explora => (
                airframes::explora().guidance.plane.cruise_thrust,
                FlightRegime::Copter,
            ),
            Subject::Cobra => (
                airframes::quadplane().guidance.plane.cruise_thrust,
                FlightRegime::Copter,
            ),
            Subject::Alia => (
                airframes::alia250().guidance.plane.cruise_thrust,
                FlightRegime::Copter,
            ),
        };
        PlaneTuneHoldConfig {
            recover_to,
            ..PlaneTuneHoldConfig::new(thrust, self.plane_config().trim_airspeed_mps)
        }
    }
}

/// What a run did: its events, and how far it flew from where the hold started.
struct Run {
    events: Vec<AutotuneEvent<PlaneLoop, stable_flight::stabilizers::plane::PlaneAxisGains>>,
    /// The flight mode at the end.
    mode: FlightMode,
    /// The lowest altitude below the run's start, in m.
    max_sink_m: f32,
    /// Whether the hold turned back at least once.
    turned: bool,
    /// The plane config at the end, and at the start.
    config: (PlaneStabilizerConfig, PlaneStabilizerConfig),
}

/// Flies `subject` into plane flight on its lead, hands it to the hold, then runs `plan`
/// within `envelope`, recovering per the hold's config on a breach and flying `after_s` more.
fn tune(
    subject: Subject,
    hold_config: PlaneTuneHoldConfig,
    plan: &[StagePlan<PlaneSide>],
    envelope: PlaneEnvelope,
    after_s: f32,
) -> Run {
    let hold = Rc::new(RefCell::new(PlaneTuneHold::new(hold_config)));
    let holding = Rc::new(RefCell::new(false));
    let commander = |lead: Box<dyn Commander>| LeadThenHold {
        lead,
        hold: hold.clone(),
        holding: holding.clone(),
    };
    match subject {
        Subject::Plane => {
            let plant = PlanePlant::clean(airframes::plane());
            let lead: Box<dyn Commander> = Box::new(plant.line());
            let mut flight_loop = plane_pipeline(&plant, commander(lead), ());
            run(&mut flight_loop, &hold, &holding, plan, envelope, after_s)
        }
        Subject::Cobra | Subject::Alia => {
            // An in-flight start: height counts from there, so the transition's floors are off;
            // the envelope guards height instead.
            let mut airframe = match subject {
                Subject::Cobra => airframes::quadplane(),
                _ => airframes::alia250(),
            };
            airframe.guidance.transition.min_height_m = 0.0;
            airframe.guidance.transition.plane_min_height_m = 0.0;
            let plant = QuadPlanePlant::clean(airframe);
            let lead: Box<dyn Commander> = Box::new(north_line(
                airframe.start.position_m.z,
                airframe.stabilizer.plane.trim_airspeed_mps,
            ));
            let mut flight_loop =
                quadplane_pipeline_at(&plant, airframe.start, commander(lead), ());
            run(&mut flight_loop, &hold, &holding, plan, envelope, after_s)
        }
        Subject::Explora => {
            let airframe = airframes::explora();
            let plant = TailsitterPlant::clean(airframe);
            let trim_mps = airframe.stabilizer.plane.trim_airspeed_mps;
            let lead: Box<dyn Commander> = Box::new(Interlude::new(
                hover_at(EXPLORA_ALTITUDE_M),
                20.0,
                f32::INFINITY,
                north_line_command(EXPLORA_ALTITUDE_M, trim_mps),
            ));
            let mut flight_loop = tailsitter_pipeline(&plant, commander(lead), ());
            run(&mut flight_loop, &hold, &holding, plan, envelope, after_s)
        }
    }
}

fn mode<C, V, G: Guidance, S, M, T>(flight_loop: &FlightLoop<C, V, G, S, M, T>) -> FlightMode {
    flight_loop
        .guidance()
        .flight_mode()
        .map_or(FlightMode::Plane, |report| report.mode)
}

fn run<C, V, G, S, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    hold: &Rc<RefCell<PlaneTuneHold>>,
    holding: &Rc<RefCell<bool>>,
    plan: &[StagePlan<PlaneSide>],
    envelope: PlaneEnvelope,
    after_s: f32,
) -> Run
where
    C: Commander,
    V: Vehicle,
    G: Guidance,
    S: Stabilizer + SideTunable<PlaneSide>,
    M: Mixer,
    T: TelemetryPublisher,
{
    let tick = Duration::from_secs_f32(DT);
    let side = |flight_loop: &FlightLoop<C, V, G, S, M, T>| {
        *stable_flight::tuning::autotune::SideConfig::<PlaneSide>::side(
            flight_loop.stabilizer().config(),
        )
    };
    let (mut elapsed_s, mut plane_s) = (0.0, 0.0);
    while plane_s < PLANE_LEAD_S {
        flight_loop.step(tick);
        elapsed_s += DT;
        if mode(flight_loop) == FlightMode::Plane {
            plane_s += DT;
        }
        assert!(elapsed_s < MAX_LEAD_S, "no plane flight");
    }
    *holding.borrow_mut() = true;
    for _ in 0..(HOLD_LEAD_S / DT) as usize {
        flight_loop.step(tick);
    }

    let start = side(flight_loop);
    let start_altitude_m = flight_loop.state().position_m.z;
    let mut autotune = PlaneAutotune::new(plan, envelope).expect("valid plan");
    autotune.start(flight_loop).expect("armed");
    let (mut events, mut max_sink_m, mut turned) = (Vec::new(), 0.0f32, false);
    let mut after_left_s = after_s;
    for _ in 0..(1500.0 / DT) as usize {
        {
            let mut hold = hold.borrow_mut();
            hold.allow_turn(matches!(autotune.status(), AutotuneStatus::Settling { .. }));
            autotune.set_maneuvering(hold.turning());
            turned |= hold.turning();
        }
        flight_loop.step(tick);
        max_sink_m = max_sink_m.max(start_altitude_m - flight_loop.state().position_m.z);
        if let Some(event) = autotune.update(flight_loop, DT) {
            if let AutotuneEvent::StageFailed { reason, .. } = event
                && reason.is_breach()
            {
                hold.borrow_mut().recover();
            }
            events.push(event);
        }
        if !autotune.is_running() {
            after_left_s -= DT;
            if after_left_s <= 0.0 {
                break;
            }
        }
    }
    Run {
        events,
        mode: mode(flight_loop),
        max_sink_m,
        turned,
        config: (side(flight_loop), start),
    }
}

/// How long the Alia 250's roll steps may clip, in s: from the hold, its 20° steps at its
/// 15°/s roll rate limit ask for nearly full aileron through its feedforward (3.3), and half
/// of them clip for 0.12–0.24 s (measured here), so 0.3 s leaves 25% margin over the longest.
const ALIA_ROLL_MAX_SATURATED_S: f32 = 0.3;

/// The Alia 250's pitch step, in rad: its plane spec's 5° (`alia_plane.rs`). A 10° step from
/// the hold aborts as reversed.
const ALIA_PITCH_STEP_RAD: f32 = 5.0 * std::f32::consts::PI / 180.0;

/// `subject`'s step test, rate loops first: the plane defaults, with the Alia's own steps.
fn step_test_plan(subject: Subject) -> [StagePlan<PlaneSide>; 4] {
    let config = subject.plane_config();
    [
        PlaneLoop::RollRate,
        PlaneLoop::PitchRate,
        PlaneLoop::Roll,
        PlaneLoop::Pitch,
    ]
    .map(|target| {
        let mut plan = StagePlan::plane_step_test(target, &config);
        let PlaneMethod::StepTest(step_test) = &mut plan.method;
        match (subject, target) {
            (Subject::Alia, PlaneLoop::Roll) => {
                step_test.max_saturated_s = ALIA_ROLL_MAX_SATURATED_S;
            }
            (Subject::Alia, PlaneLoop::Pitch) => step_test.step.amplitude = ALIA_PITCH_STEP_RAD,
            _ => {}
        }
        plan
    })
}

/// Each stage's steps, in order.
fn measured(run: &Run) -> Vec<(PlaneLoop, Outcome, Option<StepSummary>)> {
    run.events
        .iter()
        .filter_map(|event| match event {
            AutotuneEvent::StageDone {
                target,
                outcome,
                steps,
                ..
            } => Some((*target, *outcome, *steps)),
            _ => None,
        })
        .collect()
}

/// On every model the plane step test runs each stage to its six steps from the hold, inside
/// the envelope, turning back when it flies past the return radius, and leaves the gains as
/// they were.
#[test]
fn the_step_test_runs_on_every_plane_side() {
    for subject in SUBJECTS {
        let config = subject.plane_config();
        let hold = PlaneTuneHoldConfig {
            return_radius_m: 1000.0,
            ..subject.hold_config()
        };
        let run = tune(
            subject,
            hold,
            &step_test_plan(subject),
            PlaneEnvelope::for_plane(&config),
            0.0,
        );
        let stages = measured(&run);
        assert!(
            run.events.last() == Some(&AutotuneEvent::Complete),
            "{subject:?}: {:?}",
            run.events.last()
        );
        assert!(stages.len() == 4, "{subject:?}");
        for (target, outcome, steps) in stages {
            assert!(outcome == Outcome::Measured, "{subject:?} {target:?}");
            assert!(
                steps.map(|s| s.count()) == Some(6),
                "{subject:?} {target:?}"
            );
        }
        assert!(run.turned, "{subject:?} turned back");
        assert!(run.config.0 == run.config.1, "{subject:?}");
    }
}

/// A breach ends the run on the last kept gains, and the hold recovers: a VTOL hovers on its
/// rotors where it was, the airplane flies on wings level.
#[test]
fn a_breach_recovers_per_airframe() {
    for (subject, recovered) in [
        (Subject::Cobra, FlightMode::Copter),
        (Subject::Explora, FlightMode::Copter),
        (Subject::Plane, FlightMode::Plane),
    ] {
        // 15° of bank: the roll step (20° on the Alia, 30° on the others) breaks it.
        let envelope = PlaneEnvelope {
            max_bank_rad: 15f32.to_radians(),
            ..PlaneEnvelope::for_plane(&subject.plane_config())
        };
        let plan = [StagePlan::plane_step_test(
            PlaneLoop::Roll,
            &subject.plane_config(),
        )];
        let run = tune(subject, subject.hold_config(), &plan, envelope, 40.0);
        assert!(
            run.events.last()
                == Some(&AutotuneEvent::StageFailed {
                    target: PlaneLoop::Roll,
                    reason: AutotuneAbort::Tilt,
                }),
            "{subject:?}: {:?}",
            run.events.last()
        );
        assert!(run.mode == recovered, "{subject:?}: {:?}", run.mode);
        assert!(
            run.max_sink_m < 20.0,
            "{subject:?} sank {} m",
            run.max_sink_m
        );
    }
}
