//! The plane pipeline on a `SimPlane`: a [`PlanePlant`] flown from its start in flight, and
//! the commanders the plane tests fly it with.

use stable_flight::guidance::plane::PlaneGuidance;
use stable_flight::mixers::TableMixer;
use stable_flight::stabilizers::plane::PlaneStabilizer;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::tuning::{StepConfig, StepProbe, StepStatus};
use stable_flight::{
    AbsoluteAttitudeCommand, Command, Commander, ControlInput, FlightLoop, FlightRegime,
    TelemetryPublisher, TelemetryRecord, Vec2, VehicleState, WaypointCommand,
};
use stable_flight_sim::airframes::PlaneAirframe;
use stable_flight_sim::{Perturbations, SimPlane, Wind};

use super::RECORDED;

/// The plane pipeline on a [`SimPlane`], publishing to `T`.
pub type PlanePipeline<C, T> =
    FlightLoop<C, SimPlane, PlaneGuidance, PlaneStabilizer, TableMixer<4>, T>;

/// The plane pipeline recording its last [`RECORDED`] ticks.
pub type PlaneLoop<C> = PlanePipeline<C, RecordingTelemetry<RECORDED>>;

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

    /// The altitude of the airframe's start, in m.
    pub fn altitude_m(&self) -> f32 {
        self.airframe.start.position_m.z
    }

    /// The line north through the start, at its altitude and the trim airspeed.
    pub fn line(&self) -> ConstantCommander {
        let airspeed_mps = self.airframe.stabilizer.trim_airspeed_mps;
        north_line(self.altitude_m(), airspeed_mps)
    }
}

/// `plant` at its start, flown by `commander`, primed.
pub fn plane_loop<C: Commander>(plant: &PlanePlant, commander: C) -> PlaneLoop<C> {
    plane_pipeline(plant, commander, RecordingTelemetry::new())
}

/// `plant` at its start, flown by `commander`, publishing to `telemetry`, primed.
pub fn plane_pipeline<C: Commander, T: TelemetryPublisher>(
    plant: &PlanePlant,
    commander: C,
    telemetry: T,
) -> PlanePipeline<C, T> {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        commander,
        plant.vehicle(),
        PlaneGuidance::new(airframe.guidance),
        PlaneStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        telemetry,
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

/// Loiters around `center_m` (north, east) at `altitude_m` and `airspeed_mps`.
pub fn loiter(center_m: Vec2, altitude_m: f32, airspeed_mps: f32) -> ConstantCommander {
    ConstantCommander::new(plane_command(ControlInput::Waypoint(WaypointCommand {
        position_m: center_m,
        velocity_mps: None,
        altitude_m,
        cruise_speed_mps: airspeed_mps,
    })))
}

/// Holds `roll_rad`, `pitch_rad`, and north at `thrust`.
pub fn attitude_command(roll_rad: f32, pitch_rad: f32, thrust: f32) -> Command {
    plane_command(ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
        roll_rad,
        pitch_rad,
        heading_rad: 0.0,
        thrust,
    }))
}

/// The attitude axis a [`StepCommander`] steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    Roll,
    Pitch,
}

/// Flies `lead` for `lead_s`, then steps `axis` through a core [`StepProbe`] in absolute
/// attitude mode, holding the other axis where the step found it, at `thrust`.
pub struct StepCommander<C> {
    lead: C,
    lead_s: f32,
    elapsed_s: f32,
    axis: Axis,
    probe: StepProbe,
    thrust: f32,
    held: Option<(f32, f32)>,
}

impl<C> StepCommander<C> {
    pub fn new(lead: C, lead_s: f32, axis: Axis, step: StepConfig, thrust: f32) -> Self {
        StepCommander {
            lead,
            lead_s,
            elapsed_s: 0.0,
            axis,
            probe: StepProbe::new(step),
            thrust,
            held: None,
        }
    }

    /// The probe's status: running until the lead and the step's window have passed.
    pub fn status(&self) -> StepStatus {
        self.probe.status()
    }
}

impl<C: Commander> Commander for StepCommander<C> {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        if self.elapsed_s < self.lead_s {
            return self.lead.next_command(dt, state);
        }
        let (roll_rad, pitch_rad) = *self.held.get_or_insert((state.roll_rad, state.pitch_rad));
        let command = match self.axis {
            Axis::Roll => (self.probe.step(dt, state.roll_rad, false), pitch_rad),
            Axis::Pitch => (roll_rad, self.probe.step(dt, state.pitch_rad, false)),
        };
        attitude_command(command.0, command.1, self.thrust)
    }
}

/// Seconds the surfaces were clipped while an absolute attitude was commanded: the rate loop
/// at ±1 on any axis, or the mixer unable to deliver the moments.
#[derive(Debug, Default)]
pub struct ClipTimer {
    pub clipped_s: f32,
}

impl TelemetryPublisher for ClipTimer {
    fn publish(&mut self, record: &TelemetryRecord) {
        let stepping = matches!(record.command.input, ControlInput::AbsoluteAttitude(_));
        let saturation = record.saturation;
        let clipped = saturation.mixer_moments || saturation.moments.contains(&true);
        if stepping && clipped {
            self.clipped_s += record.dt.as_secs_f32();
        }
    }
}
