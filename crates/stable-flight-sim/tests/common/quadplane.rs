//! The quadplane pipeline on a `SimQuadPlane`: a [`QuadPlanePlant`] flown from resting on its
//! gear at the origin.

use stable_flight::guidance::vtol::QuadPlaneGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
use stable_flight::testing::RecordingTelemetry;
use stable_flight::{Commander, FlightLoop, Setpoint, TelemetryPublisher};
use stable_flight_sim::airframes::QuadPlaneAirframe;
use stable_flight_sim::{Perturbations, SimQuadPlane, Start, Wind};

use super::RECORDED;
use super::transition::{Forward, ForwardAndBack, forward_and_back_at, forward_at};

/// The quadplane pipeline on a [`SimQuadPlane`], publishing to `T`.
pub type QuadPlanePipeline<C, T> =
    FlightLoop<C, SimQuadPlane, QuadPlaneGuidance, QuadPlaneStabilizer, TableMixer<8>, T>;

/// The quadplane pipeline recording its last [`RECORDED`] ticks.
pub type QuadPlaneLoop<C> = QuadPlanePipeline<C, RecordingTelemetry<RECORDED>>;

/// How far the last recorded tick's nose was off its hover heading-hold target, in degrees: 0
/// once the target follows a heading the lift rotors can't hold. NaN without a hover setpoint.
pub fn hover_heading_error_deg(telemetry: &RecordingTelemetry<RECORDED>) -> f32 {
    let Some(tick) = telemetry.last() else {
        return f32::NAN;
    };
    let target_rad = match &tick.setpoint {
        Setpoint::Attitude(attitude) => attitude.heading_rad,
        Setpoint::Blend(blend) => blend.hover.heading_rad,
        _ => f32::NAN,
    };
    super::heading_error_deg(tick.state.heading_rad, target_rad)
}

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

    /// `airframe` in `wind`, unperturbed.
    pub fn in_wind(airframe: QuadPlaneAirframe, wind: Wind) -> Self {
        QuadPlanePlant {
            wind,
            ..QuadPlanePlant::clean(airframe)
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

/// `plant` on its gear at the origin, flown by `commander`, primed.
pub fn quadplane_loop<C: Commander>(plant: &QuadPlanePlant, commander: C) -> QuadPlaneLoop<C> {
    quadplane_pipeline(plant, commander, RecordingTelemetry::new())
}

/// `plant` on its gear at the origin, flown by `commander`, publishing to `telemetry`, primed.
pub fn quadplane_pipeline<C: Commander, T: TelemetryPublisher>(
    plant: &QuadPlanePlant,
    commander: C,
    telemetry: T,
) -> QuadPlanePipeline<C, T> {
    pipeline_from(plant, plant.vehicle(), commander, telemetry)
}

/// [`quadplane_pipeline`] from `start` (in flight, say) instead of the gear.
pub fn quadplane_pipeline_at<C: Commander, T: TelemetryPublisher>(
    plant: &QuadPlanePlant,
    start: Start,
    commander: C,
    telemetry: T,
) -> QuadPlanePipeline<C, T> {
    pipeline_from(
        plant,
        plant.vehicle().with_start(start),
        commander,
        telemetry,
    )
}

/// `plant`'s pipeline on `vehicle`, primed.
fn pipeline_from<C: Commander, T: TelemetryPublisher>(
    plant: &QuadPlanePlant,
    vehicle: SimQuadPlane,
    commander: C,
    telemetry: T,
) -> QuadPlanePipeline<C, T> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        vehicle,
        QuadPlaneGuidance::new(airframe.guidance),
        QuadPlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        telemetry,
    );
    flight_loop.prime();
    flight_loop
}

/// [`Forward`] at `airframe`'s trim airspeed.
pub fn forward(airframe: &QuadPlaneAirframe) -> Forward {
    forward_at(airframe.stabilizer.plane.trim_airspeed_mps)
}

/// [`ForwardAndBack`] at `airframe`'s trim airspeed: from `back_at_s` a `Copter` request to
/// hover `north_m` up the line.
pub fn forward_and_back(
    airframe: &QuadPlaneAirframe,
    back_at_s: f32,
    north_m: f32,
) -> ForwardAndBack {
    forward_and_back_at(
        airframe.stabilizer.plane.trim_airspeed_mps,
        back_at_s,
        north_m,
    )
}
