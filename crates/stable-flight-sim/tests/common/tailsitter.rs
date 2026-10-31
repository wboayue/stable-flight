//! The tailsitter pipeline on a `SimTailsitter`: a [`TailsitterPlant`] flown from resting on
//! its tail at the origin, whole or its hover side alone in the nose-up view.

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::view::{View, tailsitter_rotation};
use stable_flight::{Commander, FlightLoop, TelemetryPublisher, VehicleState, WaypointCommand};
use stable_flight_sim::airframes::TailsitterAirframe;
use stable_flight_sim::{Perturbations, SimTailsitter, Wind};

use super::RECORDED;
use super::transition::{
    Forward, ForwardAndBack, forward_and_back_at, forward_and_back_to, forward_at,
};

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

/// The tailsitter's hover side alone on a [`SimTailsitter`]: copter guidance and stabilizer
/// in the nose-up [`View`], which hosts the autotune as `copter-autotune-realflight --airframe
/// explora` flies it (`TailsitterStabilizer` doesn't).
pub type HoverViewLoop<C> = FlightLoop<
    C,
    SimTailsitter,
    View<CopterGuidance>,
    View<CopterStabilizer>,
    TableMixer<4>,
    RecordingTelemetry<RECORDED>,
>;

/// `plant`'s hover side on `hover`'s gains, on its tail at the origin, flown by `commander`,
/// primed.
pub fn hover_view_loop<C: Commander>(
    plant: &TailsitterPlant,
    hover: CopterStabilizerConfig,
    commander: C,
) -> HoverViewLoop<C> {
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        View::tailsitter(CopterGuidance::new(plant.airframe.guidance.hover)),
        View::tailsitter(CopterStabilizer::new(hover)),
        plant.airframe.mixer,
        RecordingTelemetry::new(),
    );
    flight_loop.prime();
    flight_loop
}

/// `state`'s roll, pitch, and heading in the hover view, in rad: all zero hovering nose up,
/// belly north.
pub fn view_angles(state: &VehicleState) -> (f32, f32, f32) {
    stable_flight::euler_angles(&(state.attitude * tailsitter_rotation()))
}

/// [`Forward`] at `airframe`'s trim airspeed.
pub fn forward(airframe: &TailsitterAirframe) -> Forward {
    forward_at(airframe.stabilizer.plane.trim_airspeed_mps)
}

/// [`ForwardAndBack`] at `airframe`'s trim airspeed: from `back_at_s` a `Copter` request to
/// hover `north_m` up the line.
pub fn forward_and_back(
    airframe: &TailsitterAirframe,
    back_at_s: f32,
    north_m: f32,
) -> ForwardAndBack {
    forward_and_back_at(
        airframe.stabilizer.plane.trim_airspeed_mps,
        back_at_s,
        north_m,
    )
}

/// [`ForwardAndBack`] at `airframe`'s trim airspeed: from `back_at_s` a `Copter` request to
/// fly to `hover`.
pub fn forward_and_back_onto(
    airframe: &TailsitterAirframe,
    back_at_s: f32,
    hover: WaypointCommand,
) -> ForwardAndBack {
    forward_and_back_to(
        airframe.stabilizer.plane.trim_airspeed_mps,
        back_at_s,
        hover,
    )
}
