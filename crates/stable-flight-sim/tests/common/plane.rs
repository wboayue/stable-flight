//! The plane pipeline on a `SimPlane`: a [`PlanePlant`] flown from its start in flight.

use stable_flight::guidance::plane::PlaneGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::plane::PlaneStabilizer;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::{
    Command, Commander, ControlInput, FlightLoop, FlightRegime, Vec2, WaypointCommand,
};
use stable_flight_sim::airframes::PlaneAirframe;
use stable_flight_sim::{Perturbations, SimPlane, Wind};

use super::RECORDED;

/// The plane pipeline on a [`SimPlane`], recording its last [`RECORDED`] ticks.
pub type PlaneLoop<C> = FlightLoop<
    C,
    SimPlane,
    PlaneGuidance,
    PlaneStabilizer,
    TableMixer<4>,
    RecordingTelemetry<RECORDED>,
>;

/// What a plane test flies: an airframe, the air it flies through, and how its plant departs
/// from the clean model (the copter's [`Plant`](super::Plant) for the airplane).
#[derive(Debug, Clone, Copy)]
pub struct PlanePlant {
    pub airframe: PlaneAirframe,
    pub wind: Wind,
    pub perturbations: Perturbations,
}

impl PlanePlant {
    /// `airframe` in calm air, unperturbed.
    pub fn clean(airframe: PlaneAirframe) -> Self {
        PlanePlant {
            airframe,
            wind: Wind::default(),
            perturbations: Perturbations::default(),
        }
    }

    /// The airplane at the airframe's start.
    pub fn vehicle(&self) -> SimPlane {
        SimPlane::new(self.airframe.model, &self.airframe.mixer)
            .expect("valid airframe")
            .with_start(self.airframe.start)
            .with_wind(self.wind)
            .with_perturbations(self.perturbations)
            .expect("valid perturbations")
    }
}

/// `plant` at its start, flown by `commander`, primed.
pub fn plane_loop<C: Commander>(plant: &PlanePlant, commander: C) -> PlaneLoop<C> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        PlaneGuidance::new(airframe.guidance),
        PlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        RecordingTelemetry::new(),
    );
    flight_loop.prime();
    flight_loop
}

/// Armed, wing-borne `input`.
pub fn plane_command(input: ControlInput) -> Command {
    Command::new(true, input).with_regime(FlightRegime::Plane)
}

/// Flies the line north through the origin, at `altitude_m` and `airspeed_mps`.
pub fn north_line(altitude_m: f32, airspeed_mps: f32) -> ConstantCommander {
    ConstantCommander::new(plane_command(ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(100_000.0, 0.0),
        velocity_mps: Some(Vec2::new(airspeed_mps, 0.0)),
        altitude_m,
        cruise_speed_mps: airspeed_mps,
    })))
}
