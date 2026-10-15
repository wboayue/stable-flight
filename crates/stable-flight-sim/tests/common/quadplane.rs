//! The copter pipeline on a `SimQuadPlane`'s lift rotors: a [`QuadPlanePlant`] flown from
//! resting on its gear at the origin.

use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::{Commander, FlightLoop};
use stable_flight_sim::airframes::QuadPlaneAirframe;
use stable_flight_sim::{Perturbations, SimQuadPlane, Wind};

use super::RECORDED;

/// The copter pipeline on a [`SimQuadPlane`], recording its last [`RECORDED`] ticks.
pub type QuadPlaneLoop<C> = FlightLoop<
    C,
    SimQuadPlane,
    CopterGuidance,
    CopterStabilizer,
    TableMixer<7>,
    RecordingTelemetry<RECORDED>,
>;

/// What a quadplane test flies: an airframe, the air it flies through, and how its plant
/// departs from the clean model (the copter's [`Plant`](super::Plant) for the quadplane).
#[derive(Debug, Clone, Copy)]
pub struct QuadPlanePlant {
    pub airframe: QuadPlaneAirframe,
    pub wind: Wind,
    pub perturbations: Perturbations,
}

impl QuadPlanePlant {
    /// `airframe` in calm air, unperturbed.
    pub fn clean(airframe: QuadPlaneAirframe) -> Self {
        QuadPlanePlant {
            airframe,
            wind: Wind::default(),
            perturbations: Perturbations::default(),
        }
    }

    /// The quadplane, resting on its gear at the origin.
    pub fn vehicle(&self) -> SimQuadPlane {
        SimQuadPlane::new(self.airframe.model, &self.airframe.mixer)
            .expect("valid airframe")
            .with_wind(self.wind)
            .with_perturbations(self.perturbations)
            .expect("valid perturbations")
    }
}

/// `plant` on its gear at the origin, flown by `commander` on the hover configs, primed.
pub fn quadplane_loop<C: Commander>(plant: &QuadPlanePlant, commander: C) -> QuadPlaneLoop<C> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        CopterGuidance::new(airframe.guidance),
        CopterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        RecordingTelemetry::new(),
    );
    flight_loop.prime();
    flight_loop
}
