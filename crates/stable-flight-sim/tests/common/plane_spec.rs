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
//! - Settling: inside the probe's 10% band before the window ends. Clipping: within the copter
//!   angle steps' allowance ([`StepTestPlan::copter`]), beyond which a step measures the
//!   surfaces' limit, not the loop.

use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::StepTestPlan;
use stable_flight::tuning::{StepConfig, StepMetrics, StepStatus};

use super::plane::{Axis, ClipTimer, PlanePlant, StepCommander, plane_pipeline};
use super::{fly, spec};

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

/// One axis step on `plant`: its metrics and the seconds its surfaces clipped; `None` if the
/// probe didn't finish.
pub fn respond(plant: &PlanePlant, axis: Axis, step: StepConfig) -> Option<(StepMetrics, f32)> {
    let thrust = plant.airframe.guidance.cruise_thrust;
    let commander = StepCommander::new(plant.line(), LEAD_S, axis, step, thrust);
    let mut flight_loop = plane_pipeline(plant, commander, ClipTimer::default());
    fly(
        &mut flight_loop,
        LEAD_S + step.duration_s + MARGIN_S,
        |_, _| {},
    );
    let (commander, .., clips) = flight_loop.into_parts();
    match commander.status() {
        StepStatus::Done(metrics) => Some((metrics, clips.clipped_s)),
        _ => None,
    }
}

/// The targets a step misses, given its metrics, the seconds its surfaces clipped, and its
/// axis's attitude time constant in s; empty when it meets them all.
pub fn missed_targets(m: &StepMetrics, clipped_s: f32, time_constant_s: f32) -> Vec<&'static str> {
    let max_rise_s = RISE_PER_TIME_CONSTANT * time_constant_s;
    let max_clipped_s = StepTestPlan::copter(TunedLoop::Roll).max_saturated_s;
    let checks = [
        ("overshoot", m.overshoot <= spec::MAX_OVERSHOOT),
        ("unsettled", m.settling_time_s.is_some()),
        ("clipped", clipped_s <= max_clipped_s),
        (
            "rise",
            m.rise_time_s.is_some_and(|rise_s| rise_s <= max_rise_s),
        ),
    ];
    checks
        .iter()
        .filter(|(_, met)| !met)
        .map(|(name, _)| *name)
        .collect()
}

/// `axis`'s attitude time constant on `plant`, in s.
fn time_constant_s(plant: &PlanePlant, axis: Axis) -> f32 {
    let time_constant_s = plant.airframe.stabilizer.time_constant_s;
    match axis {
        Axis::Roll => time_constant_s.x,
        Axis::Pitch => time_constant_s.y,
    }
}

/// Steps roll and pitch both ways on `plant` and returns a line for each step that misses the
/// spec (empty when every step meets it), naming the step, the targets it missed, and its
/// metrics.
pub fn misses(plant: &PlanePlant) -> Vec<String> {
    let mut misses = Vec::new();
    for (axis, step) in [(Axis::Roll, ROLL_STEP), (Axis::Pitch, PITCH_STEP)] {
        for step in [step, step.reversed()] {
            let row = format!("{axis:?} step {}", step.amplitude);
            match respond(plant, axis, step) {
                None => misses.push(format!("{row}: no measurement")),
                Some((m, clipped_s)) => {
                    let missed = missed_targets(&m, clipped_s, time_constant_s(plant, axis));
                    if !missed.is_empty() {
                        misses.push(format!(
                            "{row}: {} in {m:?}, clipped {clipped_s} s",
                            missed.join(", ")
                        ));
                    }
                }
            }
        }
    }
    misses
}
