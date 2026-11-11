//! Plane-side autotune in closed loop: the plane supervisor ([`PlaneAutotune`]) running its
//! stages on the airplane, the eXplora's plane side, and the quadplanes' cruise sides, from
//! plane flight in the attitude hold it flies in ([`PlaneTuneHold`]).

mod common;

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;

use assert2::check;

use stable_flight::commanders::{PlaneTuneHold, PlaneTuneHoldConfig};
use stable_flight::controllers::RateGains;
use stable_flight::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use stable_flight::tuning::StepConfig;
use stable_flight::tuning::StepSummary;
use stable_flight::tuning::autotune::{
    AutotuneAbort, AutotuneEvent, AutotuneStatus, Outcome, PlaneAutotune, PlaneEnvelope,
    PlaneMethod, PlaneSide, SideConfig, SideTunable, StagePlan,
};
use stable_flight::tuning::{Measurement, WingModel};
use stable_flight::{
    Command, Commander, FlightLoop, FlightMode, FlightRegime, Guidance, Mixer, Setpoint,
    Stabilizer, TelemetryPublisher, Vec2, Vehicle, VehicleState,
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
    /// The eXplora with its plane side trimmed at this airspeed, in m/s, as RealFlight's fit
    /// runs at 18 m/s flew it (`PSTAB_ASPD_TRIM`, #648).
    ExploraTrimmedAt(f32),
    Cobra,
    /// The quadplane with this many times its yaw-rate roll moment (`c_roll_r`): in a bank it
    /// rolls further in, so holding the bank takes a steady aileron the fit from level flight
    /// doesn't see, as RealFlight's Titan Cobra does (#635).
    CobraRollingIn(f32),
    Alia,
}

/// [`Subject::CobraRollingIn`]'s holding past: the design's first 30° roll pair ends 1.09 past
/// with 0.002 given back, as RealFlight's Cobra ended 1.09–1.13 at 12.4 m/s and gave back
/// 0.01–0.03 (#635).
const HOLDS_PAST_C_ROLL_R_SCALE: f32 = 4.0;

/// [`Subject::CobraRollingIn`]'s swinging back but ending past: its pairs give back half their
/// overshoot and end 1.03 past, as RealFlight's Cobra's did at 14 m/s (0.04–0.07 over, ending
/// 1.02–1.04) (#635).
const ENDS_PAST_C_ROLL_R_SCALE: f32 = 6.0;

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
            Subject::Explora | Subject::ExploraTrimmedAt(_) => self.explora().stabilizer.plane,
            Subject::Cobra | Subject::CobraRollingIn(_) => airframes::quadplane().stabilizer.plane,
            Subject::Alia => airframes::alia250().stabilizer.plane,
        }
    }

    fn hold_config(self) -> PlaneTuneHoldConfig {
        let (thrust, recover_to) = match self {
            Subject::Plane => (
                airframes::plane().guidance.cruise_thrust,
                FlightRegime::Plane,
            ),
            Subject::Explora | Subject::ExploraTrimmedAt(_) => (
                airframes::explora().guidance.plane.cruise_thrust,
                FlightRegime::Copter,
            ),
            Subject::Cobra | Subject::CobraRollingIn(_) => (
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

    /// The eXplora as this subject flies it: trimmed at its airspeed if it has one.
    fn explora(self) -> airframes::TailsitterAirframe {
        let mut airframe = airframes::explora();
        if let Subject::ExploraTrimmedAt(trim_mps) = self {
            airframe.stabilizer.plane.trim_airspeed_mps = trim_mps;
        }
        airframe
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
/// within `envelope` from its plane side's own gains, recovering per the hold's config on a
/// breach and flying `after_s` more.
fn tune(
    subject: Subject,
    hold_config: PlaneTuneHoldConfig,
    plan: &[StagePlan<PlaneSide>],
    envelope: PlaneEnvelope,
    after_s: f32,
) -> Run {
    tune_from(subject, hold_config, plan, envelope, (after_s, None))
}

/// [`tune`], the run started on `start`'s plane gains when given.
fn tune_from(
    subject: Subject,
    hold_config: PlaneTuneHoldConfig,
    plan: &[StagePlan<PlaneSide>],
    envelope: PlaneEnvelope,
    (after_s, start): (f32, Option<PlaneStabilizerConfig>),
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
            run(
                &mut flight_loop,
                (&hold, hold_config),
                &holding,
                plan,
                envelope,
                (after_s, start),
            )
        }
        Subject::Cobra | Subject::CobraRollingIn(_) | Subject::Alia => {
            let airframe = in_flight_quadplane(subject);
            let plant = QuadPlanePlant::clean(airframe);
            let lead: Box<dyn Commander> = Box::new(north_line(
                airframe.start.position_m.z,
                airframe.stabilizer.plane.trim_airspeed_mps,
            ));
            let mut flight_loop =
                quadplane_pipeline_at(&plant, airframe.start, commander(lead), ());
            run(
                &mut flight_loop,
                (&hold, hold_config),
                &holding,
                plan,
                envelope,
                (after_s, start),
            )
        }
        Subject::Explora | Subject::ExploraTrimmedAt(_) => {
            let airframe = subject.explora();
            let plant = TailsitterPlant::clean(airframe);
            let trim_mps = airframe.stabilizer.plane.trim_airspeed_mps;
            let lead: Box<dyn Commander> = Box::new(Interlude::new(
                hover_at(EXPLORA_ALTITUDE_M),
                20.0,
                f32::INFINITY,
                north_line_command(EXPLORA_ALTITUDE_M, trim_mps),
            ));
            let mut flight_loop = tailsitter_pipeline(&plant, commander(lead), ());
            run(
                &mut flight_loop,
                (&hold, hold_config),
                &holding,
                plan,
                envelope,
                (after_s, start),
            )
        }
    }
}

fn mode<C, V, G: Guidance, S, M, T>(flight_loop: &FlightLoop<C, V, G, S, M, T>) -> FlightMode {
    flight_loop
        .guidance()
        .flight_mode()
        .map_or(FlightMode::Plane, |report| report.mode)
}

/// Flies the lead into plane flight, hands over to the hold at the thrust the line flew
/// level on (a pilot's throttle for level flight), then runs `plan`.
fn run<C, V, G, S, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    (hold, hold_config): (&Rc<RefCell<PlaneTuneHold>>, PlaneTuneHoldConfig),
    holding: &Rc<RefCell<bool>>,
    plan: &[StagePlan<PlaneSide>],
    envelope: PlaneEnvelope,
    (after_s, start): (f32, Option<PlaneStabilizerConfig>),
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
        *SideConfig::<PlaneSide>::side(flight_loop.stabilizer().config())
    };
    let (mut elapsed_s, mut plane_s) = (0.0, 0.0);
    let mut line_thrust = 0.0;
    while plane_s < PLANE_LEAD_S {
        flight_loop.step(tick);
        elapsed_s += DT;
        if mode(flight_loop) == FlightMode::Plane {
            plane_s += DT;
        }
        if let Setpoint::Plane(setpoint) = flight_loop.setpoint() {
            // The last second's mean.
            line_thrust += (setpoint.thrust - line_thrust) * DT;
        }
        assert!(elapsed_s < MAX_LEAD_S, "no plane flight");
    }
    *hold.borrow_mut() = PlaneTuneHold::new(PlaneTuneHoldConfig {
        thrust: line_thrust,
        ..hold_config
    });
    *holding.borrow_mut() = true;
    for _ in 0..(HOLD_LEAD_S / DT) as usize {
        flight_loop.step(tick);
    }

    if let Some(start) = start {
        let mut config = *flight_loop.stabilizer().config();
        *config.side_mut() = start;
        flight_loop
            .set_stabilizer_config(config)
            .expect("valid start gains");
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

/// How long the Alia 250's roll steps may clip, in s, in its step tests and its design. From
/// the hold, its 20° steps at its 15°/s roll rate limit ask for nearly full aileron through its
/// feedforward (3.4), and half of them clip for 0.13–0.29 s (measured here): 3% of margin. The
/// design converges from every start at 0.25–0.3; below, its roll from a third of the gains
/// finds no design, and at 0.32 and up its pitch from its own gains goes unverified (#688).
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
        let PlaneMethod::StepTest(step_test) = &mut plan.method else {
            unreachable!("a step test's plan");
        };
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

/// The Alia 250's largest roll overshoot, as a fraction of the step: its plane spec's
/// (`alia_plane.rs`), where its 15°/s roll rate limit holds every roll step to a ramp.
const ALIA_MAX_ROLL_OVERSHOOT: f32 = 0.15;

/// `subject`'s design, roll then pitch: the plane defaults, with the Alia's own steps and roll
/// overshoot.
fn design_plan(subject: Subject, config: &PlaneStabilizerConfig) -> [StagePlan<PlaneSide>; 2] {
    [PlaneLoop::Roll, PlaneLoop::Pitch].map(|target| {
        let mut plan = StagePlan::plane_design(target, config);
        let PlaneMethod::Design(design) = &mut plan.method else {
            unreachable!("a design's plan");
        };
        match (subject, target) {
            (Subject::Alia, PlaneLoop::Roll) => {
                design.design.max_overshoot = ALIA_MAX_ROLL_OVERSHOOT;
                // Its step tests' clip allowance: holding the 15°/s rate limit takes 0.90–0.92
                // of the aileron on its fits, past the design's 0.8, so every candidate's model
                // steps clip, and from a third of its gains no design passes (#458).
                design.design.max_saturated_s = ALIA_ROLL_MAX_SATURATED_S;
            }
            (Subject::Alia, PlaneLoop::Pitch) => {
                design.verify.amplitude = ALIA_PITCH_STEP_RAD;
                design.design.amplitude_rad = ALIA_PITCH_STEP_RAD;
            }
            _ => {}
        }
        plan
    })
}

/// `config`'s rate gains (P, I, and feedforward, both axes) times `factor`.
fn scaled(config: PlaneStabilizerConfig, factor: f32) -> PlaneStabilizerConfig {
    let mut scaled = config;
    for gains in [&mut scaled.rate.roll, &mut scaled.rate.pitch] {
        gains.p *= factor;
        gains.i *= factor;
        gains.ff *= factor;
    }
    scaled
}

/// The plane spec's misses on `subject` flying `config` (`common::plane_spec`): the roll step
/// capped at its bank limit, the Alia's 5° pitch step and 15% roll overshoot.
fn spec_misses(subject: Subject, config: PlaneStabilizerConfig) -> Vec<String> {
    use common::plane_spec::{PITCH_STEP, ROLL_STEP, misses_on};
    let roll = StepConfig {
        amplitude: ROLL_STEP.amplitude.min(config.max_bank_rad),
        ..ROLL_STEP
    };
    let (pitch, max_overshoot) = match subject {
        Subject::Alia => (
            StepConfig {
                amplitude: ALIA_PITCH_STEP_RAD,
                ..PITCH_STEP
            },
            [ALIA_MAX_ROLL_OVERSHOOT, common::spec::MAX_OVERSHOOT],
        ),
        _ => (PITCH_STEP, [common::spec::MAX_OVERSHOOT; 2]),
    };
    let steps = [roll, pitch];
    match subject {
        Subject::Plane => {
            let mut airframe = airframes::plane();
            airframe.stabilizer = config;
            misses_on(&PlanePlant::clean(airframe), steps, max_overshoot)
        }
        Subject::Explora | Subject::ExploraTrimmedAt(_) => {
            let mut airframe = subject.explora();
            airframe.stabilizer.plane = config;
            misses_on(&TailsitterPlant::clean(airframe), steps, max_overshoot)
        }
        Subject::Cobra | Subject::CobraRollingIn(_) | Subject::Alia => {
            let mut airframe = in_flight_quadplane(subject);
            airframe.stabilizer.plane = config;
            misses_on(&QuadPlanePlant::clean(airframe), steps, max_overshoot)
        }
    }
}

/// `subject`'s quadplane (the Cobra's, the Alia's), started in flight: height counts from
/// there, so the transition's floors are off; the envelope guards height instead.
fn in_flight_quadplane(subject: Subject) -> airframes::QuadPlaneAirframe {
    let mut airframe = match subject {
        Subject::Alia => airframes::alia250(),
        _ => airframes::quadplane(),
    };
    if let Subject::CobraRollingIn(scale) = subject {
        airframe.model.wing.aero.c_roll_r *= scale;
    }
    airframe.guidance.transition.min_height_m = 0.0;
    airframe.guidance.transition.plane_min_height_m = 0.0;
    airframe
}

/// From its own gains, a third of them, or three times them, the design tunes `subject`'s roll
/// and pitch to the plane spec.
fn design_converges(subject: Subject) {
    for (start_name, factor) in [("own", 1.0), ("x0.3", 0.3), ("x3", 3.0)] {
        let run = design_run(subject, scaled(subject.plane_config(), factor));
        let tuned = run.config.0;
        check!(
            run.events.last() == Some(&AutotuneEvent::Complete),
            "{subject:?} {start_name}: {:?}",
            run.events.last()
        );
        let misses = spec_misses(subject, tuned);
        check!(
            misses.is_empty(),
            "{subject:?} {start_name}: {misses:?} on roll {:?} τ {}, pitch {:?} τ {}",
            tuned.rate.roll,
            tuned.time_constant_s.x,
            tuned.rate.pitch,
            tuned.time_constant_s.y
        );
    }
}

/// The design of `subject`'s roll and pitch from `start`, its hold far from turning back.
fn design_run(subject: Subject, start: PlaneStabilizerConfig) -> Run {
    let hold = PlaneTuneHoldConfig {
        return_radius_m: 1000.0,
        ..subject.hold_config()
    };
    let plan = design_plan(subject, &start);
    let envelope = PlaneEnvelope::for_plane(&start);
    tune_from(subject, hold, &plan, envelope, (0.0, Some(start)))
}

#[test]
fn the_design_tunes_the_airplane_to_the_spec() {
    design_converges(Subject::Plane);
}

#[test]
fn the_design_tunes_the_explora_to_the_spec() {
    design_converges(Subject::Explora);
}

#[test]
fn the_design_tunes_the_titan_cobra_to_the_spec() {
    design_converges(Subject::Cobra);
}

/// A held bank that needs a steady aileron overshoots and stays past the target; the design
/// raises I rather than slowing the time constant, which only widens the offset (#635).
#[test]
fn the_design_tunes_a_cobra_that_holds_past_its_bank_to_the_spec() {
    design_converges(Subject::CobraRollingIn(HOLDS_PAST_C_ROLL_R_SCALE));
}

/// An overshoot part swing, part offset: the design raises I and slows the time constant
/// together, where a slower time constant alone left the offset. From a third of its gains it
/// lands 0.057 over on the spec's step, so only its own are held (#635).
#[test]
fn the_design_tunes_a_cobra_that_ends_past_its_bank_to_the_spec() {
    let subject = Subject::CobraRollingIn(ENDS_PAST_C_ROLL_R_SCALE);
    let run = design_run(subject, subject.plane_config());
    check!(
        run.events.last() == Some(&AutotuneEvent::Complete),
        "{:?}",
        run.events.last()
    );
    let misses = spec_misses(subject, run.config.0);
    check!(misses.is_empty(), "{misses:?}");
}

/// The Alia 250 rolls into its banks as X-Plane's does (its yaw-rate roll moment fitted to
/// X-Plane's held aileron, #458), so its roll steps hold past the target until the rate loop's
/// I supplies the aileron; the design doubles I on each such pair and reaches it within its
/// eight (#537).
#[test]
fn the_design_tunes_the_alia_250_to_the_spec() {
    design_converges(Subject::Alia);
}

/// A fit term the sim eXplora's cruise is held to: (loop, name, the term in a fit,
/// RealFlight's lowest and highest across its runs, most the sim's may fall outside them).
type FitRow = (
    PlaneLoop,
    &'static str,
    fn(&WingModel) -> f32,
    f32,
    f32,
    f32,
);

/// The eXplora's cruise fits as RealFlight's did (#587, #619; runs 1–3 at 13.8–13.9 m/s): its
/// pitch stiff in angle of attack and barely damped, a flying wing's, and its roll's damping
/// and control. Each margin is the sim's measured distance outside RealFlight's range, plus
/// room. Both fit from [`realflight_fit_start`]'s gains.
#[test]
fn explora_fits_as_in_realflight() {
    use PlaneLoop::{PitchRate, RollRate};
    let rows: [FitRow; 7] = [
        // Measured 21.9, 1.0 below.
        (RollRate, "damping", |m| m.damping_per_s, 22.9, 23.5, 2.0),
        // Measured 194, 10 above.
        (
            RollRate,
            "control",
            |m| m.control_radps2,
            180.0,
            184.0,
            20.0,
        ),
        // Measured 61.3, 7.5 below: the stiffness barely follows `c_pitch_alpha` here.
        (
            PitchRate,
            "stiffness",
            |m| m.stiffness_per_s2,
            68.8,
            93.4,
            10.0,
        ),
        // Measured 0.83, inside.
        (PitchRate, "damping", |m| m.damping_per_s, 0.0, 1.3, 0.5),
        // Measured 86.8, 4.8 above: the cruise wash over the elevons.
        (PitchRate, "control", |m| m.control_radps2, 80.0, 82.0, 5.0),
        // Measured 18.8, 0.5 above.
        (
            PitchRate,
            "path turn",
            |m| m.path_turn_per_s,
            18.25,
            18.29,
            1.0,
        ),
        // Measured 0.090, 0.001 above.
        (
            PitchRate,
            "trim moment",
            |m| m.trim_moment,
            0.089,
            0.089,
            0.01,
        ),
    ];
    let run = design_run(Subject::Explora, realflight_fit_start());
    check_fits(&run, &rows);
}

/// Checks each of `rows` against `run`'s fit of its loop.
fn check_fits(run: &Run, rows: &[FitRow]) {
    for &(target, term, value, realflight_min, realflight_max, margin) in rows {
        let sim = value(&wing_fit(run, target));
        check!(
            sim >= realflight_min - margin && sim <= realflight_max + margin,
            "{target:?} {term}: {sim} against RealFlight's {realflight_min}–{realflight_max}"
        );
    }
}

/// `run`'s fit of `target`.
fn wing_fit(run: &Run, target: PlaneLoop) -> WingModel {
    let fit = run.events.iter().find_map(|event| match event {
        AutotuneEvent::Measured {
            target: measured,
            measurement: Measurement::Wing(model),
        } if *measured == target => Some(*model),
        _ => None,
    });
    fit.unwrap_or_else(|| panic!("no {target:?} fit in {:?}", run.events))
}

/// The eXplora's cruise at 18 m/s fits as RealFlight's did there (#648; runs 1–2 at 17.9 m/s,
/// trimmed at 18 and started on [`realflight_fast_fit_start`]'s gains): the elevons' pitch
/// authority and the flight path's turn grown with airspeed as RealFlight's, and the trim
/// command fallen. Margins as [`explora_fits_as_in_realflight`]'s. The pitch stiffness and
/// damping aren't held: they move with the start gains (#618), and RealFlight's two runs fit
/// 364 and 586/s².
#[test]
fn explora_fits_at_18_mps_as_in_realflight() {
    use PlaneLoop::{PitchRate, RollRate};
    let rows: [FitRow; 5] = [
        // Measured 26.9, 3.8 below: the sim's roll damping grows as V^0.8, RealFlight's V^1.1.
        (RollRate, "damping", |m| m.damping_per_s, 30.7, 31.1, 5.0),
        // Measured 305, 11 below.
        (
            RollRate,
            "control",
            |m| m.control_radps2,
            316.0,
            319.0,
            20.0,
        ),
        // Measured 126, 3 above: the cruise wash over the elevons, which keeps their authority
        // growing slower than V² as RealFlight's does (V^1.65).
        (
            PitchRate,
            "control",
            |m| m.control_radps2,
            123.0,
            123.0,
            5.0,
        ),
        // Measured 23.14, inside.
        (
            PitchRate,
            "path turn",
            |m| m.path_turn_per_s,
            23.14,
            23.15,
            1.0,
        ),
        // Measured 0.073, 0.001 below.
        (
            PitchRate,
            "trim moment",
            |m| m.trim_moment,
            0.074,
            0.074,
            0.01,
        ),
    ];
    let run = design_run(
        Subject::ExploraTrimmedAt(FAST_FIT_MPS),
        realflight_fast_fit_start(),
    );
    check_fits(&run, &rows);
}

/// The eXplora's pitch trim command falls with airspeed as RealFlight's does: as V^-0.74 on
/// its level legs at 13.4–19 m/s, and V^-0.73 between its fits at 14 and 18 m/s
/// (`docs/findings/explora-trim-vs-airspeed.md`). The rate loop's airspeed scaling assumes
/// V^-2, so a trim that falls slower leaves its integral to unwind the difference.
#[test]
fn explora_trim_falls_with_airspeed_as_in_realflight() {
    let slow = design_run(Subject::Explora, realflight_fit_start());
    let fast = design_run(
        Subject::ExploraTrimmedAt(FAST_FIT_MPS),
        realflight_fast_fit_start(),
    );
    let trim = |run: &Run| wing_fit(run, PlaneLoop::PitchRate).trim_moment;
    let slow_mps = Subject::Explora.plane_config().trim_airspeed_mps;
    let exponent = (trim(&fast) / trim(&slow)).ln() / (FAST_FIT_MPS / slow_mps).ln();
    check!(
        (exponent - REALFLIGHT_TRIM_EXPONENT).abs() <= TRIM_EXPONENT_BAND,
        "V^{exponent}: {} at {slow_mps} m/s, {} at {FAST_FIT_MPS}",
        trim(&slow),
        trim(&fast)
    );
}

/// The airspeed RealFlight's second fit runs trimmed at, in m/s (#648).
const FAST_FIT_MPS: f32 = 18.0;

/// How the RealFlight eXplora's level trim command goes with airspeed: V^-0.74 over about 20k
/// steady rows at 13.4–19 m/s (`docs/findings/explora-trim-vs-airspeed.md`).
const REALFLIGHT_TRIM_EXPONENT: f32 = -0.74;

/// How far the sim's trim exponent may stray from RealFlight's: RealFlight's own pauses, at
/// constant thrust, fit V^-1.0, 0.26 off its level legs' (#648). Measured V^-0.82.
const TRIM_EXPONENT_BAND: f32 = 0.25;

/// The gains RealFlight's fit runs started from (#587): the eXplora's hand tune then, roll P
/// 0.05, I 0.1, FF 0.1, pitch P 0.08, I 0.8, FF 0.05, τ 0.4 s. The pitch fit's damping and
/// stiffness move with pitch P (#618), so the sim fits from the same gains.
fn realflight_fit_start() -> PlaneStabilizerConfig {
    let mut config = Subject::Explora.plane_config();
    config.rate.roll = RateGains {
        p: 0.05,
        i: 0.1,
        ff: 0.1,
    };
    config.rate.pitch = RateGains {
        p: 0.08,
        i: 0.8,
        ff: 0.05,
    };
    config.time_constant_s = Vec2::new(0.4, 0.4);
    config
}

/// The gains RealFlight's 18 m/s fit runs started from (#648): `explora_vtol`'s, scaled by
/// (14/18)² to fly at 18 m/s as they do trimmed at 14 (`--rate-scale 0.605`).
fn realflight_fast_fit_start() -> PlaneStabilizerConfig {
    scaled(
        Subject::ExploraTrimmedAt(FAST_FIT_MPS).plane_config(),
        REALFLIGHT_FAST_RATE_SCALE,
    )
}

/// `--rate-scale` on RealFlight's 18 m/s fit runs: (14/18)², rounded as flown (#648).
const REALFLIGHT_FAST_RATE_SCALE: f32 = 0.605;
