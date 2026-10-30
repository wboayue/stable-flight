//! The step-response spec the plane's attitude loops are held to: each step both ways in
//! absolute attitude mode from level flight on the line, through core's step probe.
//!
//! Sources (verified in source, #348):
//! - Rise: PX4 v1.15.0 `fw_att_control_params.c:56,71` (`FW_R_TC`, `FW_P_TC`, 0.4 s). The
//!   attitude loop asks for the error over its time constant τ, as PX4's does, so the response
//!   is first order in τ, whose 10–90% rise is 2.2τ: the configured τ, not a fixed number,
//!   sets the target.
//! - Overshoot: neither ArduPlane's autotune (it limits P and D by slew rate,
//!   `AP_AutoTune.cpp`) nor PX4's (system identification) has a step target, so this borrows
//!   the copter spec's ([`spec::MAX_OVERSHOOT`]).
//! - Settling: inside the probe's 10% band before the window ends. Clipping: within
//!   [`MAX_CLIPPED_S`], beyond which a step measures the surfaces' limit, not the loop.

use stable_flight::tuning::{StepConfig, StepMetrics, StepStatus};

use super::plane::{Axis, ClipTimer, PlanePlant, StepCommander, north_line, plane_pipeline};
use super::quadplane::{QuadPlanePlant, quadplane_pipeline_at};
use super::{fly, spec};

/// How long a step's surfaces may clip, in s: 5 ticks at 250 Hz, half a tick clear of `f32`
/// rounding. The airplane's and the Alia's plane steps don't clip (0 s measured, #310), so
/// the band is margin.
const MAX_CLIPPED_S: f32 = 0.022;

/// 10–90% rise of a first-order response, in time constants.
const RISE_PER_TIME_CONSTANT: f32 = 2.2;

/// Seconds on the line before the step: the prop spins up from rest and the loops settle.
const LEAD_S: f32 = 20.0;

/// Seconds after the window the check allows the probe to report.
const MARGIN_S: f32 = 0.5;

/// The roll step: 30°, a turn's bank, held 4 s.
pub const ROLL_STEP: StepConfig = StepConfig {
    amplitude: 30.0 * core::f32::consts::PI / 180.0,
    duration_s: 4.0,
    settle_band: 0.1,
    filter_hz: 0.0,
};

/// The pitch step: 10°, a climb or dive the airspeed survives at cruise throttle, held 4 s.
pub const PITCH_STEP: StepConfig = StepConfig {
    amplitude: 10.0 * core::f32::consts::PI / 180.0,
    ..ROLL_STEP
};

/// A plant whose plane side the spec steps: the airplane, or a quadplane in plane flight.
pub trait SpecPlant {
    /// One axis step, with the seconds its surfaces clipped ([`ClipTimer`]) as `saturated_s`;
    /// `None` if the probe didn't finish.
    fn respond(&self, axis: Axis, step: StepConfig) -> Option<StepMetrics>;

    /// The plane stabilizer's attitude time constants, roll (`x`) and pitch (`y`), in s.
    fn time_constants_s(&self) -> stable_flight::Vec2;
}

impl SpecPlant for PlanePlant {
    fn respond(&self, axis: Axis, step: StepConfig) -> Option<StepMetrics> {
        let thrust = self.airframe.guidance.cruise_thrust;
        let commander = StepCommander::new(self.line(), LEAD_S, axis, step, thrust);
        let flight_loop = plane_pipeline(self, commander, ClipTimer::default());
        measured(flight_loop, step)
    }

    fn time_constants_s(&self) -> stable_flight::Vec2 {
        self.airframe.stabilizer.time_constant_s
    }
}

/// A quadplane from its airframe's start (in flight), on the line north at the start's altitude
/// and trim airspeed: plane flight by the end of the lead, with the transition's height floor
/// off in the plant's airframe.
impl SpecPlant for QuadPlanePlant {
    fn respond(&self, axis: Axis, step: StepConfig) -> Option<StepMetrics> {
        let airframe = &self.airframe;
        let line = north_line(
            airframe.start.position_m.z,
            airframe.stabilizer.plane.trim_airspeed_mps,
        );
        let thrust = airframe.guidance.plane.cruise_thrust;
        let commander = StepCommander::new(line, LEAD_S, axis, step, thrust);
        let flight_loop =
            quadplane_pipeline_at(self, airframe.start, commander, ClipTimer::default());
        measured(flight_loop, step)
    }

    fn time_constants_s(&self) -> stable_flight::Vec2 {
        self.airframe.stabilizer.plane.time_constant_s
    }
}

/// Flies `flight_loop` through the lead and `step`'s window and reads the step's metrics.
fn measured<C, V, G, S, M>(
    mut flight_loop: stable_flight::FlightLoop<StepCommander<C>, V, G, S, M, ClipTimer>,
    step: StepConfig,
) -> Option<StepMetrics>
where
    C: stable_flight::Commander,
    V: stable_flight::Vehicle,
    G: stable_flight::Guidance,
    S: stable_flight::Stabilizer,
    M: stable_flight::Mixer,
{
    fly(
        &mut flight_loop,
        LEAD_S + step.duration_s + MARGIN_S,
        |_, _| {},
    );
    let (commander, .., clips) = flight_loop.into_parts();
    match commander.status() {
        StepStatus::Done(metrics) => Some(StepMetrics {
            saturated_s: clips.clipped_s,
            ..metrics
        }),
        _ => None,
    }
}

/// The targets a step misses, given its metrics and its axis's attitude time constant in s;
/// empty when it meets them all.
pub fn missed_targets(m: &StepMetrics, time_constant_s: f32) -> Vec<&'static str> {
    missed_targets_within(m, time_constant_s, spec::MAX_OVERSHOOT)
}

/// [`missed_targets`] with the overshoot allowed up to `max_overshoot`.
fn missed_targets_within(
    m: &StepMetrics,
    time_constant_s: f32,
    max_overshoot: f32,
) -> Vec<&'static str> {
    let max_rise_s = RISE_PER_TIME_CONSTANT * time_constant_s;
    let checks = [
        ("overshoot", m.overshoot <= max_overshoot),
        ("unsettled", m.settling_time_s.is_some()),
        ("clipped", m.saturated_s <= MAX_CLIPPED_S),
        (
            "rise",
            m.rise_time_s.is_some_and(|rise_s| rise_s <= max_rise_s),
        ),
    ];
    spec::failed(&checks)
}

/// `axis`'s attitude time constant on `plant`, in s.
fn time_constant_s(plant: &impl SpecPlant, axis: Axis) -> f32 {
    let time_constant_s = plant.time_constants_s();
    match axis {
        Axis::Roll => time_constant_s.x,
        Axis::Pitch => time_constant_s.y,
        Axis::Yaw => unreachable!("an attitude step is roll or pitch"),
    }
}

/// Steps roll and pitch both ways on `plant` and returns a line for each step that misses the
/// spec (empty when every step meets it), naming the step, the targets it missed, and its
/// metrics.
pub fn misses(plant: &impl SpecPlant) -> Vec<String> {
    misses_on(plant, [ROLL_STEP, PITCH_STEP], [spec::MAX_OVERSHOOT; 2])
}

/// [`misses`] on steps of `[roll, pitch]`, each overshoot allowed up to its `max_overshoot`.
pub fn misses_on(
    plant: &impl SpecPlant,
    steps: [StepConfig; 2],
    max_overshoot: [f32; 2],
) -> Vec<String> {
    let mut misses = Vec::new();
    let axes = [
        (Axis::Roll, steps[0], max_overshoot[0]),
        (Axis::Pitch, steps[1], max_overshoot[1]),
    ];
    for (axis, step, max_overshoot) in axes {
        for step in [step, step.reversed()] {
            let row = format!("{axis:?} step {}", step.amplitude);
            match plant.respond(axis, step) {
                None => misses.push(format!("{row}: no measurement")),
                Some(m) => {
                    let missed =
                        missed_targets_within(&m, time_constant_s(plant, axis), max_overshoot);
                    if !missed.is_empty() {
                        misses.push(format!("{row}: {} in {m:?}", missed.join(", ")));
                    }
                }
            }
        }
    }
    misses
}
