//! The tailsitter pipeline on a `SimTailsitter`: a [`TailsitterPlant`] flown from resting on
//! its tail at the origin.

use stable_flight::guidance::vtol::TailsitterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::vtol::TailsitterStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::{Commander, FlightLoop, TelemetryPublisher, VehicleState, View};
use stable_flight_sim::airframes::TailsitterAirframe;
use stable_flight_sim::{Perturbations, SimTailsitter, Wind};

use super::RECORDED;
use super::transition::{Forward, ForwardAndBack, forward_and_back_at, forward_at};

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

/// `state`'s roll, pitch, and heading in the hover view, in rad: all zero hovering nose up,
/// belly north.
pub fn view_angles(state: &VehicleState) -> (f32, f32, f32) {
    stable_flight::euler_angles(&(state.attitude * View::tailsitter(()).rotation()))
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
