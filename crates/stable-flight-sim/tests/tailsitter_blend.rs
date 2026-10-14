//! Exploration: blending the hover and plane stabilizers' moments through a tailsitter's
//! transitions, against `TailsitterStabilizer`'s switch at the handoff. Flies forward and back
//! on `airframes::tailsitter()` per schedule and plant, prints one table, asserts nothing.
//!
//! ```text
//! cargo test -p stable-flight-sim --release --test tailsitter_blend -- --ignored --nocapture
//! ```
//!
//! Neither reference stack blends a tailsitter's moments: PX4 blends throttle only
//! (`vtol_att_control/tailsitter.cpp:398-404`), ArduPilot stays on the VTOL controller and
//! scales its gains (`ArduPlane/tailsitter.cpp:106`, `Q_TAILSIT_GSCMSK`).

mod common;

use std::cell::Cell;
use std::f32::consts::FRAC_PI_2;
use std::rc::Rc;

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::stabilizers::plane::PlaneStabilizer;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::{
    AttitudeSetpoint, ControlInput, ControlOutput, FlightLoop, FlightMode, FlightModeReport,
    FlightRegime, Guidance, PlaneSetpoint, Saturation, Setpoint, Stabilizer, TelemetryPublisher,
    TelemetryRecord, Vec3, VehicleState, View,
};
use stable_flight_sim::airframes::{self, TailsitterAirframe};
use stable_flight_sim::{Gust, Wind};

use common::tailsitter::{FORWARD_AT_S, TailsitterPlant, forward_and_back, view_angles};
use common::{TICK, fly};

/// When `Copter` is requested: settled on the plane line (`tailsitter_transition.rs`).
const BACK_AT_S: f32 = 45.0;

/// Where the hover after the back transition holds, m north (`tailsitter_transition.rs`).
const BACK_HOVER_NORTH_M: f32 = 380.0;

/// Seconds flown in all: the back transition and 25 s of hover after it.
const FLIGHT_S: f32 = BACK_AT_S + 30.0;

/// Seconds after a handoff that its window (moment jumps, rates, saturation) still covers.
const AFTER_HANDOFF_S: f32 = 3.0;

/// Seconds after the back handoff that the climb (#343) is measured over.
const CLIMB_AFTER_S: f32 = 10.0;

/// Seconds after a transition ends that altitude loss still counts (PX4 `VT_QC_T_ALT_LOSS`).
const LOSS_AFTER_S: f32 = 5.0;

/// Passes `G` through, publishing its flight mode each tick for [`Blend`], which runs after
/// it in the same tick.
struct ModeTap<G> {
    inner: G,
    mode: Rc<Cell<FlightMode>>,
}

impl<G: Guidance> Guidance for ModeTap<G> {
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint {
        let setpoint = self.inner.guide(dt, regime, input, state);
        if let Some(report) = self.inner.flight_mode() {
            self.mode.set(report.mode);
        }
        setpoint
    }

    fn prime(&mut self, state: &VehicleState) {
        self.inner.prime(state);
        self.mode.set(FlightMode::Copter);
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }

    fn flight_mode(&self) -> Option<FlightModeReport> {
        self.inner.flight_mode()
    }
}

/// How much of the moments the plane side gives, `w` in [0, 1]; the hover side gives `1 − w`.
#[derive(Debug, Clone, Copy)]
enum Schedule {
    /// The setpoint's side alone: `TailsitterStabilizer` rebuilt, a harness check.
    Switch,
    /// Smoothstep over each ramp's tilt: 0 at hover to 1 at `forward_done_rad` going forward,
    /// 1 at level to 0 at `back_done_rad` coming back.
    Tilt,
    /// `(airspeed / ref)²`, clamped, in `ToPlane`, `Plane`, and `ToCopter`: the surfaces'
    /// share of dynamic pressure. 0 in `Copter`.
    Pressure { ref_mps: f32 },
    /// The outgoing side fades over `ramp_s` after each handoff (PX4's throttle blend, applied
    /// to moments).
    Time { ramp_s: f32 },
}

impl Schedule {
    fn label(&self) -> String {
        match self {
            Schedule::Switch => "switch".into(),
            Schedule::Tilt => "tilt".into(),
            Schedule::Pressure { ref_mps } => format!("q@{ref_mps:.0}m/s"),
            Schedule::Time { ramp_s } => format!("time {ramp_s}s"),
        }
    }
}

/// Both of a tailsitter's stabilizers, their moments mixed by a [`Schedule`]. Thrust is the
/// setpoint's side's, as `TailsitterStabilizer`'s, so only the moments differ from it.
///
/// The other side's setpoint comes from the guidance's, wings level: view pitch `θ − π/2` for
/// airplane pitch `θ`, heading held where the hover view points. Airplane bank (a view heading
/// change) is dropped, so in a banked turn the hover side only damps.
struct Blend {
    /// Flown in `ToPlane` and `Plane`, and in `ToCopter` and `Copter`.
    forward: Schedule,
    back: Schedule,
    mode: Rc<Cell<FlightMode>>,
    hover: View<CopterStabilizer>,
    plane: PlaneStabilizer,
    forward_done_rad: f32,
    back_done_rad: f32,
    /// The mode the last tick flew, and seconds since it began.
    last_mode: FlightMode,
    in_mode_s: f32,
    hover_on: bool,
    plane_on: bool,
    saturation: Saturation,
}

impl Blend {
    fn new(
        airframe: &TailsitterAirframe,
        (forward, back): (Schedule, Schedule),
        mode: Rc<Cell<FlightMode>>,
    ) -> Self {
        Blend {
            forward,
            back,
            mode,
            hover: View::tailsitter(CopterStabilizer::new(airframe.stabilizer.hover)),
            plane: PlaneStabilizer::new(airframe.stabilizer.plane),
            forward_done_rad: airframe.guidance.transition.forward_done_rad,
            back_done_rad: airframe.guidance.transition.back_done_rad,
            last_mode: FlightMode::Copter,
            in_mode_s: f32::INFINITY,
            hover_on: false,
            plane_on: false,
            saturation: Saturation::default(),
        }
    }

    fn weight(&self, setpoint: &Setpoint, state: &VehicleState) -> f32 {
        let on_plane = matches!(setpoint, Setpoint::Plane(_) | Setpoint::PlaneRate(_));
        let native = if on_plane { 1.0 } else { 0.0 };
        let mode = self.mode.get();
        let tilt_rad = FRAC_PI_2 - stable_flight::euler_angles(&state.attitude).1;
        let schedule = match mode {
            FlightMode::ToPlane | FlightMode::Plane => self.forward,
            _ => self.back,
        };
        match schedule {
            Schedule::Switch => native,
            Schedule::Tilt => match mode {
                FlightMode::ToPlane => smoothstep(tilt_rad / self.forward_done_rad),
                FlightMode::ToCopter => {
                    smoothstep((tilt_rad - self.back_done_rad) / (FRAC_PI_2 - self.back_done_rad))
                }
                _ => native,
            },
            Schedule::Pressure { ref_mps } => match mode {
                FlightMode::Copter => 0.0,
                _ => {
                    let airspeed_mps = state.valid_airspeed_mps().unwrap_or(0.0);
                    ((airspeed_mps / ref_mps).powi(2)).min(1.0)
                }
            },
            Schedule::Time { ramp_s } => {
                let faded = (self.in_mode_s / ramp_s).min(1.0);
                match mode {
                    FlightMode::Plane => faded,
                    FlightMode::Copter => 1.0 - faded,
                    _ => native,
                }
            }
        }
    }
}

fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// `setpoint` for the hover side: itself, or a wings-level plane attitude in the view.
fn hover_setpoint(setpoint: &Setpoint, state: &VehicleState) -> Option<Setpoint> {
    match setpoint {
        Setpoint::Plane(plane) => Some(Setpoint::Attitude(AttitudeSetpoint {
            roll_rad: 0.0,
            pitch_rad: plane.pitch_rad - FRAC_PI_2,
            heading_rad: view_angles(state).2,
            thrust: plane.thrust,
        })),
        Setpoint::PlaneRate(_) => None,
        hover => Some(*hover),
    }
}

/// `setpoint` for the plane side: itself, or a wings-level airplane attitude from the view's.
fn plane_setpoint(setpoint: &Setpoint) -> Option<Setpoint> {
    match setpoint {
        Setpoint::Attitude(attitude) => Some(Setpoint::Plane(PlaneSetpoint {
            roll_rad: 0.0,
            pitch_rad: attitude.pitch_rad + FRAC_PI_2,
            thrust: attitude.thrust,
        })),
        Setpoint::Rate(_) | Setpoint::BodyRate(_) => None,
        plane => Some(*plane),
    }
}

impl Stabilizer for Blend {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        let mode = self.mode.get();
        if mode == self.last_mode {
            self.in_mode_s += dt;
        } else {
            self.last_mode = mode;
            self.in_mode_s = dt;
        }
        let on_plane = matches!(setpoint, Setpoint::Plane(_) | Setpoint::PlaneRate(_));
        let hover_setpoint = hover_setpoint(setpoint, state);
        let plane_setpoint = plane_setpoint(setpoint);
        let mut weight = self.weight(setpoint, state);
        if hover_setpoint.is_none() {
            weight = 1.0;
        }
        if plane_setpoint.is_none() {
            weight = 0.0;
        }

        let run_hover = weight < 1.0;
        let run_plane = weight > 0.0;
        if run_hover && !self.hover_on {
            self.hover.prime(state);
        }
        if run_plane && !self.plane_on {
            self.plane.prime(state);
        }
        self.hover_on = run_hover;
        self.plane_on = run_plane;

        let mut thrust = 0.0;
        let mut moments = Vec3::zeros();
        self.saturation = Saturation::default();
        if let (true, Some(hover_setpoint)) = (run_hover, hover_setpoint) {
            let output = self.hover.stabilize(dt, &hover_setpoint, state);
            moments += output.moments * (1.0 - weight);
            if !on_plane {
                thrust = output.thrust;
            }
            self.saturation = self.saturation.merge(self.hover.saturation());
        }
        if let (true, Some(plane_setpoint)) = (run_plane, plane_setpoint) {
            let output = self.plane.stabilize(dt, &plane_setpoint, state);
            moments += output.moments * weight;
            if on_plane {
                thrust = output.thrust;
            }
            self.saturation = self.saturation.merge(self.plane.saturation());
        }
        // The setpoint's side didn't run (weight 0 or 1 against it): its thrust anyway.
        if (on_plane && !run_plane) || (!on_plane && !run_hover) {
            thrust = setpoint.thrust();
        }
        ControlOutput::new(thrust, moments)
    }

    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.hover_on = true;
        self.plane_on = true;
        self.last_mode = FlightMode::Copter;
        self.in_mode_s = f32::INFINITY;
    }

    fn saturation(&self) -> Saturation {
        self.saturation
    }
}

/// One tick, as published.
struct Tick {
    mode: FlightMode,
    setpoint: Setpoint,
    thrust: f32,
    saturation: Saturation,
    state: VehicleState,
    moments: Vec3,
    saturated: bool,
}

#[derive(Default)]
struct Recorder {
    ticks: Vec<Tick>,
}

impl TelemetryPublisher for Recorder {
    fn publish(&mut self, record: &TelemetryRecord) {
        let saturation = record.saturation;
        self.ticks.push(Tick {
            mode: record.flight_mode.mode,
            setpoint: *record.setpoint,
            thrust: record.control.thrust,
            saturation,
            state: record.state.clone(),
            moments: record.control.moments,
            saturated: saturation.mixer_moments || saturation.moments.iter().any(|&m| m),
        });
    }
}

/// What one transition did. NaN where it never got there.
struct Leg {
    /// Seconds from the request to the handoff.
    duration_s: f32,
    /// Below the height at the request, until [`LOSS_AFTER_S`] after the handoff, in m.
    loss_m: f32,
    /// Largest tick-to-tick change in the moments, from the request until
    /// [`AFTER_HANDOFF_S`] after the handoff.
    jump: f32,
    /// When it happened, in s from the handoff (negative: before it).
    jump_at_s: f32,
    /// Largest body rate in that window, in deg/s.
    rate_degps: f32,
    /// Share of that window's ticks with moments saturated, in percent.
    saturated_pct: f32,
}

struct Outcome {
    forward: Leg,
    back: Leg,
    /// Above the height at the back request, until [`CLIMB_AFTER_S`] after its handoff, in m.
    climb_m: f32,
    /// Off the hover point at the end, in m.
    final_m: f32,
}

fn tick_time_s(index: usize) -> f32 {
    (index + 1) as f32 * TICK.as_secs_f32()
}

fn leg(ticks: &[Tick], from_s: f32, mode: FlightMode, until_s: f32) -> Leg {
    let request = (0..ticks.len())
        .find(|&i| tick_time_s(i) >= from_s)
        .unwrap_or(ticks.len() - 1);
    let handoff = (request..ticks.len()).find(|&i| {
        ticks[i].mode == mode && (i == 0 || ticks[i - 1].mode != mode) && tick_time_s(i) <= until_s
    });
    let Some(handoff) = handoff else {
        return Leg {
            duration_s: f32::NAN,
            loss_m: f32::NAN,
            jump: f32::NAN,
            jump_at_s: f32::NAN,
            rate_degps: f32::NAN,
            saturated_pct: f32::NAN,
        };
    };
    let ticks_per_s = 1.0 / TICK.as_secs_f32();
    let end = |seconds: f32| (handoff + (seconds * ticks_per_s) as usize).min(ticks.len() - 1);
    let start_m = ticks[request].state.position_m.z;
    let lowest_m = ticks[request..=end(LOSS_AFTER_S)]
        .iter()
        .map(|t| t.state.position_m.z)
        .fold(f32::INFINITY, f32::min);
    let window = &ticks[request..=end(AFTER_HANDOFF_S)];
    let (jump_index, jump) = window
        .windows(2)
        .map(|pair| (pair[1].moments - pair[0].moments).norm())
        .enumerate()
        .fold(
            (0, 0.0),
            |best, (i, j)| if j > best.1 { (i + 1, j) } else { best },
        );
    let jump_at_s = tick_time_s(request + jump_index) - tick_time_s(handoff);
    let rate_degps = window
        .iter()
        .map(|t| t.state.angular_velocity_radps.norm().to_degrees())
        .fold(0.0, f32::max);
    let saturated = window.iter().filter(|t| t.saturated).count();
    Leg {
        duration_s: tick_time_s(handoff) - tick_time_s(request),
        loss_m: (start_m - lowest_m).max(0.0),
        jump,
        jump_at_s,
        rate_degps,
        saturated_pct: 100.0 * saturated as f32 / window.len() as f32,
    }
}

fn outcome(ticks: &[Tick]) -> Outcome {
    let forward = leg(ticks, FORWARD_AT_S, FlightMode::Plane, BACK_AT_S);
    let back = leg(ticks, BACK_AT_S, FlightMode::Copter, FLIGHT_S);
    let back_at = (0..ticks.len())
        .find(|&i| tick_time_s(i) >= BACK_AT_S)
        .unwrap_or(ticks.len() - 1);
    let climb_until_s = BACK_AT_S + back.duration_s + CLIMB_AFTER_S;
    let highest_m = ticks[back_at..]
        .iter()
        .enumerate()
        .take_while(|(i, _)| tick_time_s(back_at + i) <= climb_until_s)
        .map(|(_, t)| t.state.position_m.z)
        .fold(f32::NEG_INFINITY, f32::max);
    let last = &ticks[ticks.len() - 1].state.position_m;
    let climb_m = if back.duration_s.is_nan() {
        f32::NAN
    } else {
        highest_m - ticks[back_at].state.position_m.z
    };
    Outcome {
        forward,
        back,
        climb_m,
        final_m: (last - Vec3::new(BACK_HOVER_NORTH_M, 0.0, last.z))
            .xy()
            .norm(),
    }
}

fn fly_with<S: Stabilizer>(
    plant: &TailsitterPlant,
    stabilizer: S,
    mode: Rc<Cell<FlightMode>>,
) -> Vec<Tick> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        forward_and_back(&airframe, BACK_AT_S, BACK_HOVER_NORTH_M),
        plant.vehicle(),
        ModeTap {
            inner: TailsitterGuidance::new(airframe.guidance),
            mode,
        },
        stabilizer,
        airframe.mixer,
        Recorder::default(),
    );
    flight_loop.prime();
    fly(&mut flight_loop, FLIGHT_S, |_, _| {});
    flight_loop.into_parts().5.ticks
}

fn run(plant: &TailsitterPlant, schedule: Option<(Schedule, Schedule)>) -> Outcome {
    let mode = Rc::new(Cell::new(FlightMode::Copter));
    let ticks = match schedule {
        None => fly_with(
            plant,
            TailsitterStabilizer::new(plant.airframe.stabilizer),
            mode,
        ),
        Some(schedule) => fly_with(
            plant,
            Blend::new(&plant.airframe, schedule, mode.clone()),
            mode,
        ),
    };
    outcome(&ticks)
}

fn plants() -> Vec<(&'static str, TailsitterPlant)> {
    let airframe = airframes::tailsitter();
    let mut capped = airframe;
    capped.stabilizer.plane.max_pitch_rad = 30f32.to_radians();
    // 3 m/s across the span at its peak, 1 − cos over 2 s, half a second into each transition.
    let gust = |start_s: f32| Wind {
        gust: Some(Gust {
            start_s,
            duration_s: 2.0,
            peak_mps: Vec3::new(0.0, 3.0, 0.0),
        }),
        ..Wind::default()
    };
    vec![
        ("clean", TailsitterPlant::clean(airframe)),
        ("cap 30° (#388)", TailsitterPlant::clean(capped)),
        (
            "gust fwd",
            TailsitterPlant::in_wind(airframe, gust(FORWARD_AT_S + 0.5)),
        ),
        (
            "gust back",
            TailsitterPlant::in_wind(airframe, gust(BACK_AT_S + 0.5)),
        ),
    ]
}

#[test]
#[ignore = "exploration: prints a table, asserts nothing"]
fn compares_transition_blends() {
    let airframe = airframes::tailsitter();
    let min_mps = airframe.guidance.transition.min_airspeed_mps;
    let trim_mps = airframe.stabilizer.plane.trim_airspeed_mps;
    let tilt = Schedule::Tilt;
    let q_min = Schedule::Pressure { ref_mps: min_mps };
    let q_trim = Schedule::Pressure { ref_mps: trim_mps };
    let time = Schedule::Time { ramp_s: 1.0 };
    let both = |schedule| Some((schedule, schedule));
    let schedules = [
        None,
        both(Schedule::Switch),
        both(tilt),
        both(q_min),
        both(q_trim),
        both(Schedule::Time { ramp_s: 0.5 }),
        both(time),
        // Forward / back.
        Some((tilt, time)),
        Some((time, tilt)),
        Some((tilt, q_min)),
        Some((time, q_min)),
        Some((q_min, time)),
        Some((Schedule::Switch, tilt)),
        Some((tilt, Schedule::Switch)),
    ];
    println!(
        "\nforward: s to Plane, m lost, max |Δmoment|/tick, max rate deg/s, % saturated; back: \
         the same to Copter, then climb m, final m off the hover point\n"
    );
    println!(
        "{:<15} {:<16} | {:>5} {:>5} {:>5} {:>5} {:>5} {:>4} | {:>5} {:>5} {:>5} {:>5} {:>5} {:>4} | {:>5} {:>6}",
        "plant",
        "blend",
        "s",
        "loss",
        "jump",
        "at",
        "rate",
        "sat",
        "s",
        "loss",
        "jump",
        "at",
        "rate",
        "sat",
        "climb",
        "final"
    );
    for (name, plant) in plants() {
        for schedule in schedules {
            let label = match schedule {
                None => "stock".into(),
                Some((forward, back)) if forward.label() == back.label() => forward.label(),
                Some((forward, back)) => format!("{}/{}", forward.label(), back.label()),
            };
            let o = run(&plant, schedule);
            let (f, b) = (&o.forward, &o.back);
            println!(
                "{name:<15} {label:<16} | {:>5.2} {:>5.2} {:>5.3} {:>5.2} {:>5.0} {:>4.0} | {:>5.2} {:>5.2} {:>5.3} {:>5.2} {:>5.0} {:>4.0} | {:>5.1} {:>6.1}",
                f.duration_s,
                f.loss_m,
                f.jump,
                f.jump_at_s,
                f.rate_degps,
                f.saturated_pct,
                b.duration_s,
                b.loss_m,
                b.jump,
                b.jump_at_s,
                b.rate_degps,
                b.saturated_pct,
                o.climb_m,
                o.final_m,
            );
        }
        println!();
    }
}

/// Tick by tick around the back handoff on the clean airframe, per `handoff_s`: the setpoint,
/// thrust, moments, and saturation, to find what steps the moments after the handoff.
#[test]
#[ignore = "exploration: prints a trace, asserts nothing"]
fn traces_the_back_handoff() {
    for handoff_s in [0.25, 0.5, 1.0] {
        let mut airframe = airframes::tailsitter();
        airframe.guidance.transition.handoff_s = handoff_s;
        let plant = TailsitterPlant::clean(airframe);
        let mode = Rc::new(Cell::new(FlightMode::Copter));
        let ticks = fly_with(&plant, TailsitterStabilizer::new(airframe.stabilizer), mode);
        let handoff = (1..ticks.len())
            .find(|&i| {
                tick_time_s(i) > BACK_AT_S
                    && ticks[i].mode == FlightMode::Copter
                    && ticks[i - 1].mode != FlightMode::Copter
            })
            .expect("hands over");
        println!(
            "\nhandoff_s {handoff_s}: handoff at {:.3} s",
            tick_time_s(handoff)
        );
        let ticks_per_s = (1.0 / TICK.as_secs_f32()) as usize;
        let mut worst = (0, 0.0);
        for i in handoff + 1..handoff + 2 * ticks_per_s {
            let jump = (ticks[i].moments - ticks[i - 1].moments).norm();
            if jump > worst.1 {
                worst = (i, jump);
            }
        }
        println!(
            "largest jump {:.3} at +{:.3} s",
            worst.1,
            tick_time_s(worst.0) - tick_time_s(handoff)
        );
        let zero: Vec<f32> = (handoff..ticks.len())
            .filter(|&i| ticks[i].thrust <= 0.0)
            .map(|i| tick_time_s(i) - tick_time_s(handoff))
            .collect();
        let height_m = ticks[handoff].state.position_m.z;
        println!(
            "zero-thrust ticks after handoff: {} ({:.2} s), first +{:.3} s, last +{:.3} s; height at handoff {height_m:.1} m",
            zero.len(),
            zero.len() as f32 * TICK.as_secs_f32(),
            zero.first().copied().unwrap_or(f32::NAN),
            zero.last().copied().unwrap_or(f32::NAN),
        );
        let from = worst.0.saturating_sub(3);
        for i in (handoff - 1..handoff + 3).chain(from..worst.0 + 4) {
            let t = &ticks[i];
            let sp = match t.setpoint {
                Setpoint::Attitude(a) => format!(
                    "att r{:6.2} p{:7.2} h{:7.2} T{:.3}",
                    a.roll_rad.to_degrees(),
                    a.pitch_rad.to_degrees(),
                    a.heading_rad.to_degrees(),
                    a.thrust
                ),
                Setpoint::Plane(a) => format!(
                    "pln r{:6.2} p{:7.2}          T{:.3}",
                    a.roll_rad.to_degrees(),
                    a.pitch_rad.to_degrees(),
                    a.thrust
                ),
                other => format!("{other:?}"),
            };
            let s = t.saturation;
            println!(
                "+{:6.3} {:?} {sp} | thr {:.3} m [{:6.3} {:6.3} {:6.3}] | sat tilt{} thr{} m{:?} mixT{} mixM{}",
                tick_time_s(i) - tick_time_s(handoff),
                t.mode,
                t.thrust,
                t.moments.x,
                t.moments.y,
                t.moments.z,
                s.tilt as u8,
                s.thrust as u8,
                s.moments.map(u8::from),
                s.mixer_thrust as u8,
                s.mixer_moments as u8,
            );
        }
    }
}
