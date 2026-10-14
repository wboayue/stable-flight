//! The tailsitter pipeline on a `SimTailsitter`: a [`TailsitterPlant`] flown from resting on
//! its tail at the origin.

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::{
    Command, Commander, ControlInput, FlightLoop, FlightMode, Guidance, ModeRefusal,
    TelemetryPublisher, Vec3, VehicleState, View,
};
use stable_flight_sim::airframes::TailsitterAirframe;
use stable_flight_sim::{Perturbations, SimTailsitter, Wind};

use super::plane::{Interlude, north_line_command};
use super::{Hover, RECORDED, fly, hover_at, waypoint};

/// Height the transition tests hover at and fly the plane leg at, in m: clear of the
/// transition's `min_height_m` and of PX4's 20 m altitude-loss band (`VT_QC_T_ALT_LOSS`).
pub const TRANSITION_ALTITUDE_M: f32 = 30.0;

/// Seconds of hover, from resting on the tail, before `Plane` is requested: the climb to
/// [`TRANSITION_ALTITUDE_M`] and settling.
pub const FORWARD_AT_S: f32 = 20.0;

/// Hovers over the origin at [`TRANSITION_ALTITUDE_M`], then from [`FORWARD_AT_S`] flies the
/// line north as a plane.
pub type Forward = Interlude<Hover>;

/// [`Forward`], then from its `back_at_s` hovers over a waypoint.
pub type ForwardAndBack = Interlude<Forward>;

/// The tailsitter pipeline on a [`SimTailsitter`], publishing to `T`.
pub type TailsitterPipeline<C, T> =
    FlightLoop<C, SimTailsitter, TailsitterGuidance, TailsitterStabilizer, TableMixer<4>, T>;

/// The tailsitter pipeline recording its last [`RECORDED`] ticks.
pub type TailsitterLoop<C> = TailsitterPipeline<C, RecordingTelemetry<RECORDED>>;

/// What a tailsitter test flies: an airframe, the air it flies through, and how its plant
/// departs from the clean model (the copter's [`Plant`](super::Plant) for the tailsitter).
#[derive(Debug, Clone, Copy)]
pub struct TailsitterPlant {
    pub airframe: TailsitterAirframe,
    pub wind: Wind,
    pub perturbations: Perturbations,
}

impl TailsitterPlant {
    /// `airframe` in calm air, unperturbed.
    pub fn clean(airframe: TailsitterAirframe) -> Self {
        TailsitterPlant {
            airframe,
            wind: Wind::default(),
            perturbations: Perturbations::default(),
        }
    }

    /// `airframe` in `wind`, unperturbed.
    pub fn in_wind(airframe: TailsitterAirframe, wind: Wind) -> Self {
        TailsitterPlant {
            wind,
            ..TailsitterPlant::clean(airframe)
        }
    }

    /// The tailsitter, resting on its tail at the origin.
    pub fn vehicle(&self) -> SimTailsitter {
        SimTailsitter::new(self.airframe.model, &self.airframe.mixer)
            .expect("valid airframe")
            .with_wind(self.wind)
            .with_perturbations(self.perturbations)
            .expect("valid perturbations")
    }
}

/// `plant` on its tail at the origin, flown by `commander`, primed.
pub fn tailsitter_loop<C: Commander>(plant: &TailsitterPlant, commander: C) -> TailsitterLoop<C> {
    tailsitter_pipeline(plant, commander, RecordingTelemetry::new())
}

/// `plant` on its tail at the origin, flown by `commander`, publishing to `telemetry`, primed.
pub fn tailsitter_pipeline<C: Commander, T: TelemetryPublisher>(
    plant: &TailsitterPlant,
    commander: C,
    telemetry: T,
) -> TailsitterPipeline<C, T> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        TailsitterGuidance::new(airframe.guidance),
        TailsitterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        telemetry,
    );
    flight_loop.prime();
    flight_loop
}

/// Largest deviations from a hover point seen while holding it.
#[derive(Debug, Default)]
pub struct HoverDeviation {
    /// Off the point's altitude, in m.
    pub altitude_m: f32,
    /// Horizontal distance from the point, in m.
    pub horizontal_m: f32,
    /// Hover-view roll or pitch: the nose off vertical.
    pub tilt_deg: f32,
    /// Hover-view heading.
    pub heading_deg: f32,
    /// Horizontal distance from the point at the last state observed, in m.
    pub final_m: f32,
}

impl HoverDeviation {
    /// Widens the deviations to cover `state`, hovering over `point_m` (north, east, up).
    pub fn observe(&mut self, state: &VehicleState, point_m: Vec3) {
        let (roll_rad, pitch_rad, heading_rad) = view_angles(state);
        let off_m = state.position_m - point_m;
        self.altitude_m = self.altitude_m.max(off_m.z.abs());
        self.final_m = off_m.xy().norm();
        self.horizontal_m = self.horizontal_m.max(self.final_m);
        self.tilt_deg = self
            .tilt_deg
            .max(roll_rad.abs().max(pitch_rad.abs()).to_degrees());
        self.heading_deg = self.heading_deg.max(heading_rad.abs().to_degrees());
    }
}

/// `state`'s roll, pitch, and heading in the hover view, in rad: all zero hovering nose up,
/// belly north.
pub fn view_angles(state: &VehicleState) -> (f32, f32, f32) {
    stable_flight::euler_angles(&(state.attitude * View::tailsitter(()).rotation()))
}

/// [`Forward`] at `airframe`'s trim airspeed.
pub fn forward(airframe: &TailsitterAirframe) -> Forward {
    let airspeed_mps = airframe.stabilizer.plane.trim_airspeed_mps;
    Interlude::new(
        hover_at(TRANSITION_ALTITUDE_M),
        FORWARD_AT_S,
        f32::INFINITY,
        north_line_command(TRANSITION_ALTITUDE_M, airspeed_mps),
    )
}

/// [`Forward`], then from `back_at_s` a `Copter` request to hover `north_m` up the line at
/// [`TRANSITION_ALTITUDE_M`].
pub fn forward_and_back(
    airframe: &TailsitterAirframe,
    back_at_s: f32,
    north_m: f32,
) -> ForwardAndBack {
    let hover = waypoint(north_m, 0.0, TRANSITION_ALTITUDE_M);
    Interlude::new(
        forward(airframe),
        back_at_s,
        f32::INFINITY,
        Command::new(true, ControlInput::Waypoint(hover)),
    )
}

/// One tick of a transition flight.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Simulated seconds since the flight began.
    pub time_s: f32,
    pub mode: FlightMode,
    pub refusal: Option<ModeRefusal>,
    pub state: VehicleState,
}

/// Flies `flight_loop` for `seconds`, sampling every tick.
pub fn trace<C: Commander>(flight_loop: &mut TailsitterLoop<C>, seconds: f32) -> Trace {
    let start_s = flight_loop.vehicle().time_s() as f32;
    let mut samples = Vec::new();
    fly(flight_loop, seconds, |flight_loop, elapsed_s| {
        let report = flight_loop
            .guidance()
            .flight_mode()
            .expect("tailsitter reports its mode");
        let state = flight_loop.state().clone();
        samples.push(Sample {
            time_s: start_s + elapsed_s,
            mode: report.mode,
            refusal: report.refusal,
            state,
        });
    });
    Trace { samples }
}

/// The ticks of a transition flight, for asking when modes changed and how high it flew.
pub struct Trace {
    pub samples: Vec<Sample>,
}

impl Trace {
    /// The first tick at or after `from_s` flying `mode`.
    pub fn entered(&self, mode: FlightMode, from_s: f32) -> Option<&Sample> {
        self.samples
            .iter()
            .find(|sample| sample.time_s >= from_s && sample.mode == mode)
    }

    /// The first tick at or after `time_s`.
    pub fn at(&self, time_s: f32) -> Option<&Sample> {
        self.samples.iter().find(|sample| sample.time_s >= time_s)
    }

    /// The ticks from `from_s` until `until_s`.
    pub fn window(&self, from_s: f32, until_s: f32) -> impl Iterator<Item = &Sample> {
        self.samples
            .iter()
            .filter(move |sample| (from_s..until_s).contains(&sample.time_s))
    }

    /// The lowest and highest heights from `from_s` until `until_s`, in m.
    pub fn height_range_m(&self, from_s: f32, until_s: f32) -> (f32, f32) {
        self.window(from_s, until_s).fold(
            (f32::INFINITY, f32::NEG_INFINITY),
            |(low, high), sample| {
                (
                    low.min(sample.state.position_m.z),
                    high.max(sample.state.position_m.z),
                )
            },
        )
    }
}
