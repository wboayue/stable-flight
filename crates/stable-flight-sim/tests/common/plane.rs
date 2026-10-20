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

/// Level-flight altitude band, either way, in m: ArduPilot's for a loiter altitude regained
/// (Plane-4.5 `Tools/autotest/arduplane.py:2727`), held for a whole window.
pub const ALTITUDE_BAND_M: f32 = 1.0;

/// Level-flight airspeed band, either way, in m/s: ArduPilot's for a commanded airspeed held
/// (`arduplane.py:897`, 5 s minimum), held for a whole window.
pub const AIRSPEED_BAND_MPS: f32 = 1.0;

/// Largest deviations from level flight up the north line seen while holding it.
#[derive(Debug, Default)]
pub struct LineDeviation {
    /// Off the held altitude, in m.
    pub altitude_m: f32,
    /// Off the held airspeed, in m/s.
    pub airspeed_mps: f32,
    /// Off the line (east), in m.
    pub cross_track_m: f32,
}

impl LineDeviation {
    /// Widens the deviations to cover `state`, holding `altitude_m` at `airspeed_mps`.
    pub fn observe(&mut self, state: &VehicleState, altitude_m: f32, airspeed_mps: f32) {
        let airspeed = state.airspeed_mps.unwrap_or(f32::NAN);
        self.altitude_m = self.altitude_m.max((state.position_m.z - altitude_m).abs());
        self.airspeed_mps = self.airspeed_mps.max((airspeed - airspeed_mps).abs());
        self.cross_track_m = self.cross_track_m.max(state.position_m.y.abs());
    }
}

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
    ConstantCommander::new(north_line_command(altitude_m, airspeed_mps))
}

/// [`north_line`]'s command.
pub fn north_line_command(altitude_m: f32, airspeed_mps: f32) -> Command {
    plane_command(ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(100_000.0, 0.0),
        velocity_mps: Some(Vec2::new(airspeed_mps, 0.0)),
        altitude_m,
        cruise_speed_mps: airspeed_mps,
        heading_rad: None,
    }))
}

/// Loiters around `center_m` (north, east) at `altitude_m` and `airspeed_mps`.
pub fn loiter(center_m: Vec2, altitude_m: f32, airspeed_mps: f32) -> ConstantCommander {
    ConstantCommander::new(plane_command(ControlInput::Waypoint(WaypointCommand {
        position_m: center_m,
        velocity_mps: None,
        altitude_m,
        cruise_speed_mps: airspeed_mps,
        heading_rad: None,
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
///
/// A commander doesn't see the surfaces, so the probe's `saturated_s` stays 0: [`ClipTimer`]
/// measures clipping.
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

/// Flies `lead`, except from `from_s` until `until_s` (simulated seconds from the first tick),
/// when it sends `command`.
pub struct Interlude<C> {
    lead: C,
    from_s: f32,
    until_s: f32,
    command: Command,
    elapsed_s: f32,
}

impl<C> Interlude<C> {
    pub fn new(lead: C, from_s: f32, until_s: f32, command: Command) -> Self {
        Interlude {
            lead,
            from_s,
            until_s,
            command,
            elapsed_s: 0.0,
        }
    }
}

impl<C: Commander> Commander for Interlude<C> {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        if (self.from_s..self.until_s).contains(&self.elapsed_s) {
            return self.command;
        }
        self.lead.next_command(dt, state)
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
