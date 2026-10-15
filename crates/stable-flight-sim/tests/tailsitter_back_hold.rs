//! Exploration: after a tailsitter's back transition, hold the handover altitude until the
//! attitude upset settles, then descend to the hover waypoint, against descending at once.
//! Crossed with the #393 back tilt blend of the moments. Prints one table, asserts nothing.
//!
//! ```text
//! cargo test -p stable-flight-sim --release --test tailsitter_back_hold -- --ignored --nocapture
//! ```
//!
//! The blend is `explore/tailsitter-blend`'s `tailsitter_blend.rs`, back side only.

mod common;

use std::cell::Cell;
use std::f32::consts::FRAC_PI_2;
use std::rc::Rc;

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::stabilizers::plane::PlaneStabilizer;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::{
    AttitudeSetpoint, Command, ControlInput, ControlOutput, FlightLoop, FlightMode,
    FlightModeReport, FlightRegime, Guidance, PlaneSetpoint, Saturation, Setpoint, Stabilizer,
    TelemetryPublisher, TelemetryRecord, Vec3, VehicleState, View,
};
use stable_flight_sim::airframes::{self, TailsitterAirframe};
use stable_flight_sim::{Gust, Wind};

use common::plane::Interlude;
use common::tailsitter::{TailsitterPlant, forward, view_angles};
use common::{TICK, fly, waypoint};

/// When `Copter` is requested: settled on the plane line (`tailsitter_transition.rs`).
const BACK_AT_S: f32 = 45.0;

/// Where the hover after the back transition holds, m north (`tailsitter_transition.rs`).
const BACK_HOVER_NORTH_M: f32 = 380.0;

/// Seconds flown in all: the back transition and 30 s of hover after it.
const FLIGHT_S: f32 = BACK_AT_S + 30.0;

/// Seconds after the handover the upset window covers.
const UPSET_S: f32 = 3.0;

/// Longest hold, in s: a release that never fires still descends.
const MAX_HOLD_S: f32 = 5.0;

/// Shortest hold, in s: the upset starts a few ticks after the handover, so a release mustn't
/// fire on the still-vertical handover attitude.
const MIN_HOLD_S: f32 = 0.5;

/// How long a release condition must hold continuously, in s.
const RELEASE_DWELL_S: f32 = 0.25;

/// Settled: within this of the waypoint's altitude, in m, ...
const SETTLED_ALT_M: f32 = 0.5;
/// ... and the hover view within this of level, in deg, to the end of the flight.
const SETTLED_TILT_DEG: f32 = 5.0;

/// What ends the hold after the handover.
#[derive(Debug, Clone, Copy)]
enum Release {
    /// No hold: stock, descends at once.
    None,
    /// After a fixed time, in s.
    Time(f32),
    /// Hover view within `tilt_deg` of level and body rate under `rate_degps`.
    Attitude { tilt_deg: f32, rate_degps: f32 },
    /// Vertical speed within `mps` of zero.
    Still { mps: f32 },
}

impl Release {
    fn label(&self) -> String {
        match self {
            Release::None => "descend".into(),
            Release::Time(s) => format!("hold {s}s"),
            Release::Attitude {
                tilt_deg,
                rate_degps,
            } => format!("hold att<{tilt_deg:.0}°,{rate_degps:.0}°/s"),
            Release::Still { mps } => format!("hold |vz|<{mps}"),
        }
    }

    /// Whether `state` meets the condition (time is checked by the caller).
    fn met(&self, state: &VehicleState) -> bool {
        match *self {
            Release::None => true,
            Release::Time(_) => false,
            Release::Attitude {
                tilt_deg,
                rate_degps,
            } => {
                tilt_deg_of(state) < tilt_deg
                    && state.angular_velocity_radps.norm().to_degrees() < rate_degps
            }
            Release::Still { mps } => state.velocity_mps.z.abs() < mps,
        }
    }
}

/// Passes `G` through, publishing its flight mode each tick for [`Blend`], and after a back
/// handover raises a waypoint's altitude to the handover altitude until `release` fires.
struct HoldTap<G> {
    inner: G,
    mode: Rc<Cell<FlightMode>>,
    release: Release,
    /// Whether the settling window raises the waypoint's altitude to the handover's.
    hold_altitude: bool,
    /// A thrust floor (linear) through the settling window, if any.
    floor: Option<f32>,
    /// Fastest the thrust setpoint may fall through the settling window, per s, if capped.
    slew_per_s: Option<f32>,
    /// Last tick's thrust setpoint.
    last_thrust: f32,
    /// The held altitude, in m, and seconds held; `None` when not holding.
    hold: Option<(f32, f32)>,
    /// Seconds the release condition has held continuously.
    met_s: f32,
}

impl<G: Guidance> Guidance for HoldTap<G> {
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint {
        let previous = self.inner.flight_mode().map(|report| report.mode);
        if previous == Some(FlightMode::ToCopter) && !matches!(self.release, Release::None) {
            // Armed on every ToCopter tick, so the hold starts from the last ramp altitude.
            self.hold = Some((state.position_m.z, 0.0));
            self.met_s = 0.0;
        }
        let holding = previous == Some(FlightMode::Copter) && self.hold.is_some();
        let input = match (holding, input, self.hold) {
            (true, ControlInput::Waypoint(command), Some((hold_m, _))) if self.hold_altitude => {
                let mut command = *command;
                command.altitude_m = command.altitude_m.max(hold_m);
                ControlInput::Waypoint(command)
            }
            _ => *input,
        };
        let mut setpoint = self.inner.guide(dt, regime, &input, state);
        if let (true, Some(floor), Setpoint::Attitude(attitude)) =
            (holding, self.floor, &mut setpoint)
        {
            attitude.thrust = attitude.thrust.max(floor);
        }
        if let (true, Some(rate), Setpoint::Attitude(attitude)) =
            (holding, self.slew_per_s, &mut setpoint)
        {
            attitude.thrust = attitude.thrust.max(self.last_thrust - rate * dt);
        }
        self.last_thrust = setpoint.thrust();
        let mode = self
            .inner
            .flight_mode()
            .map_or(FlightMode::Copter, |r| r.mode);
        self.mode.set(mode);
        if mode != FlightMode::Copter && mode != FlightMode::ToCopter {
            self.hold = None;
        }
        if let (true, Some((hold_m, held_s))) = (holding, self.hold) {
            let held_s = held_s + dt;
            self.met_s = if self.release.met(state) {
                self.met_s + dt
            } else {
                0.0
            };
            let released = match self.release {
                Release::Time(s) => held_s >= s,
                _ => held_s >= MIN_HOLD_S && self.met_s >= RELEASE_DWELL_S,
            };
            self.hold = (!released && held_s < MAX_HOLD_S).then_some((hold_m, held_s));
        }
        setpoint
    }

    fn prime(&mut self, state: &VehicleState) {
        self.inner.prime(state);
        self.mode.set(FlightMode::Copter);
        self.hold = None;
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }

    fn flight_mode(&self) -> Option<FlightModeReport> {
        self.inner.flight_mode()
    }
}

/// Both of a tailsitter's stabilizers. Outside `ToCopter`, the setpoint's side alone; in
/// `ToCopter` the moments mix by smoothstep over the back ramp's tilt (1 at level to 0 at
/// `back_done_rad`), the #393 back tilt blend. Thrust is the setpoint's side's.
struct Blend {
    mode: Rc<Cell<FlightMode>>,
    hover: View<CopterStabilizer>,
    plane: PlaneStabilizer,
    back_done_rad: f32,
    hover_on: bool,
    plane_on: bool,
    saturation: Saturation,
}

impl Blend {
    fn new(airframe: &TailsitterAirframe, mode: Rc<Cell<FlightMode>>) -> Self {
        Blend {
            mode,
            hover: View::tailsitter(CopterStabilizer::new(airframe.stabilizer.hover)),
            plane: PlaneStabilizer::new(airframe.stabilizer.plane),
            back_done_rad: airframe.guidance.transition.back_done_rad,
            hover_on: false,
            plane_on: false,
            saturation: Saturation::default(),
        }
    }

    fn weight(&self, on_plane: bool, state: &VehicleState) -> f32 {
        if self.mode.get() != FlightMode::ToCopter {
            return if on_plane { 1.0 } else { 0.0 };
        }
        let tilt_rad = FRAC_PI_2 - stable_flight::euler_angles(&state.attitude).1;
        smoothstep((tilt_rad - self.back_done_rad) / (FRAC_PI_2 - self.back_done_rad))
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
            transition: plane.transition,
            idle: false,
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
            transition: attitude.transition,
        })),
        Setpoint::Rate(_) | Setpoint::BodyRate(_) => None,
        plane => Some(*plane),
    }
}

impl Stabilizer for Blend {
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        let on_plane = matches!(setpoint, Setpoint::Plane(_) | Setpoint::PlaneRate(_));
        let hover_setpoint = hover_setpoint(setpoint, state);
        let plane_setpoint = plane_setpoint(setpoint);
        let mut weight = self.weight(on_plane, state);
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

        let mut thrust = setpoint.thrust();
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
        ControlOutput::new(thrust, moments)
    }

    fn prime(&mut self, state: &VehicleState) {
        self.hover.prime(state);
        self.plane.prime(state);
        self.hover_on = true;
        self.plane_on = true;
    }

    fn saturation(&self) -> Saturation {
        self.saturation
    }
}

/// One tick, as published.
struct Tick {
    mode: FlightMode,
    thrust: f32,
    state: VehicleState,
    saturated: bool,
}

#[derive(Default)]
struct Recorder {
    ticks: Vec<Tick>,
}

impl TelemetryPublisher for Recorder {
    fn publish(&mut self, record: &TelemetryRecord) {
        let s = record.saturation;
        self.ticks.push(Tick {
            mode: record.flight_mode.mode,
            thrust: record.control.thrust,
            state: record.state.clone(),
            saturated: s.mixer_moments || s.mixer_thrust || s.moments.iter().any(|&m| m),
        });
    }
}

/// What happened from the back handover on. NaN where it never handed over.
struct Outcome {
    /// Seconds from the request to the handover.
    back_s: f32,
    /// Largest hover-view tilt in the [`UPSET_S`] after the handover, in deg.
    tilt_deg: f32,
    /// Largest body rate in that window, in deg/s.
    rate_degps: f32,
    /// Fastest sink in that window, in m/s.
    upset_sink_mps: f32,
    /// Fastest sink from the handover to the end, in m/s.
    sink_mps: f32,
    /// Share of the upset window's ticks saturated (mixer or moments), in percent.
    saturated_pct: f32,
    /// Largest thrust in the upset window.
    thrust: f32,
    /// Highest above the handover height, in m.
    climb_m: f32,
    /// Lowest below the waypoint, in m.
    dip_m: f32,
    /// Seconds from the handover until within [`SETTLED_ALT_M`] of the waypoint and
    /// [`SETTLED_TILT_DEG`] of level for good.
    settled_s: f32,
}

fn tick_time_s(index: usize) -> f32 {
    (index + 1) as f32 * TICK.as_secs_f32()
}

fn tilt_deg_of(state: &VehicleState) -> f32 {
    let (roll_rad, pitch_rad, _) = view_angles(state);
    roll_rad.abs().max(pitch_rad.abs()).to_degrees()
}

fn max_of(values: impl Iterator<Item = f32>) -> f32 {
    values.fold(f32::NEG_INFINITY, f32::max)
}

fn outcome(ticks: &[Tick], waypoint_m: f32) -> Outcome {
    let handover = (1..ticks.len()).find(|&i| {
        tick_time_s(i) > BACK_AT_S
            && ticks[i].mode == FlightMode::Copter
            && ticks[i - 1].mode != FlightMode::Copter
    });
    let Some(handover) = handover else {
        let nan = f32::NAN;
        return Outcome {
            back_s: nan,
            tilt_deg: nan,
            rate_degps: nan,
            upset_sink_mps: nan,
            sink_mps: nan,
            saturated_pct: nan,
            thrust: nan,
            climb_m: nan,
            dip_m: nan,
            settled_s: nan,
        };
    };
    let upset_end = (handover + (UPSET_S / TICK.as_secs_f32()) as usize).min(ticks.len());
    let upset = &ticks[handover..upset_end];
    let after = &ticks[handover..];
    let handover_m = ticks[handover].state.position_m.z;
    let lowest_m = after
        .iter()
        .map(|t| t.state.position_m.z)
        .fold(f32::INFINITY, f32::min);
    let unsettled = after.iter().rposition(|t| {
        (t.state.position_m.z - waypoint_m).abs() > SETTLED_ALT_M
            || tilt_deg_of(&t.state) > SETTLED_TILT_DEG
    });
    Outcome {
        back_s: tick_time_s(handover) - BACK_AT_S,
        tilt_deg: max_of(upset.iter().map(|t| tilt_deg_of(&t.state))),
        rate_degps: max_of(
            upset
                .iter()
                .map(|t| t.state.angular_velocity_radps.norm().to_degrees()),
        ),
        upset_sink_mps: max_of(upset.iter().map(|t| -t.state.velocity_mps.z)).max(0.0),
        sink_mps: max_of(after.iter().map(|t| -t.state.velocity_mps.z)).max(0.0),
        saturated_pct: 100.0 * upset.iter().filter(|t| t.saturated).count() as f32
            / upset.len() as f32,
        thrust: max_of(upset.iter().map(|t| t.thrust)),
        climb_m: max_of(after.iter().map(|t| t.state.position_m.z)) - handover_m,
        dip_m: (waypoint_m - lowest_m).max(0.0),
        settled_s: unsettled.map_or(0.0, |i| (i + 1) as f32 * TICK.as_secs_f32()),
    }
}

fn fly_with<S: Stabilizer>(
    plant: &TailsitterPlant,
    waypoint_m: f32,
    release: Release,
    stabilizer: S,
    mode: Rc<Cell<FlightMode>>,
) -> Vec<Tick> {
    fly_settling(plant, waypoint_m, release, true, None, stabilizer, mode)
}

fn fly_settling<S: Stabilizer>(
    plant: &TailsitterPlant,
    waypoint_m: f32,
    release: Release,
    hold_altitude: bool,
    floor: Option<f32>,
    stabilizer: S,
    mode: Rc<Cell<FlightMode>>,
) -> Vec<Tick> {
    fly_shaped(
        plant,
        waypoint_m,
        release,
        hold_altitude,
        floor,
        None,
        stabilizer,
        mode,
    )
}

#[allow(clippy::too_many_arguments)]
fn fly_shaped<S: Stabilizer>(
    plant: &TailsitterPlant,
    waypoint_m: f32,
    release: Release,
    hold_altitude: bool,
    floor: Option<f32>,
    slew_per_s: Option<f32>,
    stabilizer: S,
    mode: Rc<Cell<FlightMode>>,
) -> Vec<Tick> {
    let airframe = plant.airframe;
    let commander = Interlude::new(
        forward(&airframe),
        BACK_AT_S,
        f32::INFINITY,
        Command::new(
            true,
            ControlInput::Waypoint(waypoint(BACK_HOVER_NORTH_M, 0.0, waypoint_m)),
        ),
    );
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        HoldTap {
            inner: TailsitterGuidance::new(airframe.guidance),
            mode,
            release,
            hold_altitude,
            floor,
            slew_per_s,
            last_thrust: 0.0,
            hold: None,
            met_s: 0.0,
        },
        stabilizer,
        airframe.mixer,
        Recorder::default(),
    );
    flight_loop.prime();
    fly(&mut flight_loop, FLIGHT_S, |_, _| {});
    flight_loop.into_parts().5.ticks
}

fn run(plant: &TailsitterPlant, waypoint_m: f32, blend: bool, release: Release) -> Outcome {
    let mode = Rc::new(Cell::new(FlightMode::Copter));
    let ticks = if blend {
        let stabilizer = Blend::new(&plant.airframe, mode.clone());
        fly_with(plant, waypoint_m, release, stabilizer, mode)
    } else {
        let stabilizer = TailsitterStabilizer::new(plant.airframe.stabilizer);
        fly_with(plant, waypoint_m, release, stabilizer, mode)
    };
    outcome(&ticks, waypoint_m)
}

fn plants() -> Vec<(&'static str, TailsitterPlant)> {
    let airframe = airframes::tailsitter();
    // 3 m/s across the span at its peak, 1 − cos over 2 s, half a second into the back ramp.
    let gust = Wind {
        gust: Some(Gust {
            start_s: BACK_AT_S + 0.5,
            duration_s: 2.0,
            peak_mps: Vec3::new(0.0, 3.0, 0.0),
        }),
        ..Wind::default()
    };
    vec![
        ("clean", TailsitterPlant::clean(airframe)),
        ("gust back", TailsitterPlant::in_wind(airframe, gust)),
    ]
}

/// Tick by tick from the back request on the clean airframe at the 30 m waypoint, every
/// 0.2 s: descend at once against a 1 s hold.
#[test]
#[ignore = "exploration: prints a trace, asserts nothing"]
fn traces_the_back_handover() {
    let plant = TailsitterPlant::clean(airframes::tailsitter());
    for release in [Release::None, Release::Time(1.0)] {
        let mode = Rc::new(Cell::new(FlightMode::Copter));
        let stabilizer = TailsitterStabilizer::new(plant.airframe.stabilizer);
        let ticks = fly_with(&plant, 30.0, release, stabilizer, mode);
        println!("\n{}", release.label());
        let every = (0.2 / TICK.as_secs_f32()) as usize;
        let from = (BACK_AT_S / TICK.as_secs_f32()) as usize;
        for i in (from..from + 9 * 5 * every).step_by(every) {
            let t = &ticks[i];
            let (roll, pitch, _) = view_angles(&t.state);
            println!(
                "{:6.2} {:?} z {:6.2} vz {:5.2} roll {:6.1} pitch {:6.1} thr {:.2}",
                tick_time_s(i) - BACK_AT_S,
                t.mode,
                t.state.position_m.z,
                t.state.velocity_mps.z,
                roll.to_degrees(),
                pitch.to_degrees(),
                t.thrust,
            );
        }
    }
}

#[test]
#[ignore = "exploration: prints a table, asserts nothing"]
fn compares_holding_after_the_back_handover() {
    let releases = [
        Release::None,
        Release::Time(1.0),
        Release::Time(2.0),
        Release::Attitude {
            tilt_deg: 10.0,
            rate_degps: 30.0,
        },
        Release::Still { mps: 0.3 },
    ];
    println!(
        "\nupset window: {UPSET_S} s after the handover. tilt: hover view, deg; rate: deg/s; \
         sink m/s (upset window / whole descent); sat: % of upset ticks; climb above the \
         handover, dip under the waypoint, m; settled: s from the handover\n"
    );
    println!(
        "{:<10} {:>4} {:<6} {:<22} | {:>5} {:>5} {:>5} {:>5} {:>5} {:>4} {:>5} | {:>5} {:>5} {:>7}",
        "plant",
        "wp",
        "moment",
        "after handover",
        "back",
        "tilt",
        "rate",
        "sink",
        "max",
        "sat",
        "thr",
        "climb",
        "dip",
        "settled"
    );
    for (name, plant) in plants() {
        for waypoint_m in [30.0, 20.0] {
            for blend in [false, true] {
                for release in releases {
                    let o = run(&plant, waypoint_m, blend, release);
                    println!(
                        "{name:<10} {waypoint_m:>4.0} {:<6} {:<22} | {:>5.2} {:>5.1} {:>5.0} {:>5.2} {:>5.2} {:>4.0} {:>5.2} | {:>5.1} {:>5.2} {:>7.1}",
                        if blend { "tilt" } else { "switch" },
                        release.label(),
                        o.back_s,
                        o.tilt_deg,
                        o.rate_degps,
                        o.upset_sink_mps,
                        o.sink_mps,
                        o.saturated_pct,
                        o.thrust,
                        o.climb_m,
                        o.dip_m,
                        o.settled_s,
                    );
                }
            }
            println!();
        }
    }
}

/// Is the upset propwash-limited? The back ramp's thrust and a hover thrust floor
/// (`ThrustLimits::min_thrust`, kept under `hover_thrust`), descending at once and holding.
#[test]
#[ignore = "exploration: prints a table, asserts nothing"]
fn sweeps_ramp_thrust_and_hover_floor() {
    let base = airframes::tailsitter();
    let hover_thrust = base.guidance.hover.altitude.hover_thrust;
    println!("\nhover_thrust {hover_thrust:.3} (linear)\n");
    println!(
        "{:>5} {:>5} {:<22} | {:>5} {:>5} {:>5} {:>5} {:>5} {:>5} | {:>6} {:>5} {:>7}",
        "back",
        "floor",
        "after handover",
        "back",
        "tilt",
        "rate",
        "sink",
        "max",
        "thr",
        "ramp+",
        "dip",
        "settled"
    );
    let releases = [
        Release::None,
        Release::Attitude {
            tilt_deg: 10.0,
            rate_degps: 30.0,
        },
    ];
    for back_thrust in [0.6, 0.75, 0.9] {
        for floor in [0.0, 0.5, 0.6] {
            if floor >= hover_thrust {
                continue;
            }
            let mut airframe = base;
            airframe.guidance.transition.back_thrust = back_thrust;
            airframe.guidance.hover.thrust.min_thrust = floor;
            let plant = TailsitterPlant::clean(airframe);
            for release in releases {
                let mode = Rc::new(Cell::new(FlightMode::Copter));
                let stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
                let ticks = fly_with(&plant, 30.0, release, stabilizer, mode);
                let o = outcome(&ticks, 30.0);
                let ramp_m = max_of(
                    ticks
                        .iter()
                        .skip((BACK_AT_S / TICK.as_secs_f32()) as usize)
                        .map(|t| t.state.position_m.z),
                ) - 30.0;
                println!(
                    "{back_thrust:>5.2} {floor:>5.2} {:<22} | {:>5.2} {:>5.1} {:>5.0} {:>5.2} {:>5.2} {:>5.2} | {:>6.1} {:>5.2} {:>7.1}",
                    release.label(),
                    o.back_s,
                    o.tilt_deg,
                    o.rate_degps,
                    o.upset_sink_mps,
                    o.sink_mps,
                    o.thrust,
                    ramp_m,
                    o.dip_m,
                    o.settled_s,
                );
            }
        }
    }
}

/// Holding `hover_thrust` as a thrust floor until the attitude settles, alone and with the
/// altitude hold, per ramp thrust and the standing hover floor.
#[test]
#[ignore = "exploration: prints a table, asserts nothing"]
fn sweeps_hover_thrust_floor_until_settled() {
    let base = airframes::tailsitter();
    let hover_thrust = base.guidance.hover.altitude.hover_thrust;
    let settle = Release::Attitude {
        tilt_deg: 10.0,
        rate_degps: 30.0,
    };
    let variants: [(&str, Release, bool, Option<f32>); 4] = [
        ("descend", Release::None, false, None),
        ("hold alt", settle, true, None),
        ("hover floor", settle, false, Some(hover_thrust)),
        ("hold alt + floor", settle, true, Some(hover_thrust)),
    ];
    println!("\nhover_thrust {hover_thrust:.3} (linear); release: tilt <10°, rate <30°/s\n");
    println!(
        "{:>5} {:>5} {:<17} | {:>5} {:>5} {:>5} {:>5} {:>5} | {:>6} {:>5} {:>7}",
        "back",
        "floor",
        "settling",
        "tilt",
        "rate",
        "sink",
        "max",
        "thr",
        "ramp+",
        "dip",
        "settled"
    );
    for back_thrust in [0.6, 0.75, 0.9] {
        for standing in [0.0, 0.6] {
            let mut airframe = base;
            airframe.guidance.transition.back_thrust = back_thrust;
            airframe.guidance.hover.thrust.min_thrust = standing;
            for (name, release, hold_altitude, floor) in variants {
                for (plant_name, plant) in [
                    ("clean", TailsitterPlant::clean(airframe)),
                    (
                        "gust",
                        TailsitterPlant::in_wind(
                            airframe,
                            Wind {
                                gust: Some(Gust {
                                    start_s: BACK_AT_S + 0.5,
                                    duration_s: 2.0,
                                    peak_mps: Vec3::new(0.0, 3.0, 0.0),
                                }),
                                ..Wind::default()
                            },
                        ),
                    ),
                ] {
                    let mode = Rc::new(Cell::new(FlightMode::Copter));
                    let stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
                    let ticks = fly_settling(
                        &plant,
                        30.0,
                        release,
                        hold_altitude,
                        floor,
                        stabilizer,
                        mode,
                    );
                    let o = outcome(&ticks, 30.0);
                    let ramp_m = max_of(
                        ticks
                            .iter()
                            .skip((BACK_AT_S / TICK.as_secs_f32()) as usize)
                            .map(|t| t.state.position_m.z),
                    ) - 30.0;
                    println!(
                        "{back_thrust:>5.2} {standing:>5.2} {:<17} | {:>5.1} {:>5.0} {:>5.2} {:>5.2} {:>5.2} | {:>6.1} {:>5.2} {:>7.1} {plant_name}",
                        name,
                        o.tilt_deg,
                        o.rate_degps,
                        o.upset_sink_mps,
                        o.sink_mps,
                        o.thrust,
                        ramp_m,
                        o.dip_m,
                        o.settled_s,
                    );
                }
            }
            println!();
        }
    }
}

/// Softer than a hover-thrust floor through the settling window: a floor at a fraction of
/// `hover_thrust`, or a cap on how fast the thrust setpoint may fall.
#[test]
#[ignore = "exploration: prints a table, asserts nothing"]
fn sweeps_soft_floor_and_slew_cap() {
    let base = airframes::tailsitter();
    let hover_thrust = base.guidance.hover.altitude.hover_thrust;
    let settle = Release::Attitude {
        tilt_deg: 10.0,
        rate_degps: 30.0,
    };
    let mut variants: Vec<(String, Release, Option<f32>, Option<f32>)> =
        vec![("descend".into(), Release::None, None, None)];
    for fraction in [0.85, 0.9, 1.0] {
        variants.push((
            format!("floor {fraction}×hover"),
            settle,
            Some(fraction * hover_thrust),
            None,
        ));
    }
    for rate in [0.1, 0.2, 0.4, 0.8] {
        variants.push((format!("slew {rate}/s"), settle, None, Some(rate)));
    }
    println!("\nhover_thrust {hover_thrust:.3} (linear); release: tilt <10°, rate <30°/s\n");
    println!(
        "{:>5} {:>5} {:<16} | {:>5} {:>5} {:>5} {:>5} {:>5} | {:>6} {:>5} {:>7}",
        "back",
        "floor",
        "settling",
        "tilt",
        "rate",
        "sink",
        "max",
        "thr",
        "ramp+",
        "dip",
        "settled"
    );
    for (back_thrust, standing) in [(0.75, 0.6), (0.75, 0.7), (0.75, 0.77)] {
        let mut airframe = base;
        airframe.guidance.transition.back_thrust = back_thrust;
        airframe.guidance.hover.thrust.min_thrust = standing;
        let gust = Wind {
            gust: Some(Gust {
                start_s: BACK_AT_S + 0.5,
                duration_s: 2.0,
                peak_mps: Vec3::new(0.0, 3.0, 0.0),
            }),
            ..Wind::default()
        };
        for (plant_name, plant) in [
            ("clean", TailsitterPlant::clean(airframe)),
            ("gust", TailsitterPlant::in_wind(airframe, gust)),
        ] {
            for (name, release, floor, slew) in &variants {
                let mode = Rc::new(Cell::new(FlightMode::Copter));
                let stabilizer = TailsitterStabilizer::new(airframe.stabilizer);
                let ticks = fly_shaped(
                    &plant, 30.0, *release, false, *floor, *slew, stabilizer, mode,
                );
                let o = outcome(&ticks, 30.0);
                let ramp_m = max_of(
                    ticks
                        .iter()
                        .skip((BACK_AT_S / TICK.as_secs_f32()) as usize)
                        .map(|t| t.state.position_m.z),
                ) - 30.0;
                println!(
                    "{back_thrust:>5.2} {standing:>5.2} {:<16} | {:>5.1} {:>5.0} {:>5.2} {:>5.2} {:>5.2} | {:>6.1} {:>5.2} {:>7.1} {plant_name}",
                    name,
                    o.tilt_deg,
                    o.rate_degps,
                    o.upset_sink_mps,
                    o.sink_mps,
                    o.thrust,
                    ramp_m,
                    o.dip_m,
                    o.settled_s,
                );
            }
            println!();
        }
    }
}
