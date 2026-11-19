//! The flight controller's autotune reports as ground-station text: see [`statustext`].

use mavlink::common::{MavSeverity, STATUSTEXT_DATA};
use stable_flight_link::autotune::{AutotuneProgress, AutotuneReport};
use stable_flight_link::stable_flight::controllers::Gains;
use stable_flight_link::stable_flight::stabilizers::copter::TunedLoop;
use stable_flight_link::stable_flight::tuning::autotune::{AutotuneAbort, Outcome, StartError};

use crate::encode;

#[cfg(test)]
mod tests;

/// `report` as `STATUSTEXT`, "AutoTune: " first as ArduPilot's are, so a pilot reads a run in
/// QGC's messages: each stage's start and result, each gain a search tried, the end. Severity
/// follows what the pilot must do (QGC pops up `ERROR`, speaks `NOTICE`):
///
/// - `INFO`: a stage starts, a search tries or lowers a gain, a design fits again;
/// - `NOTICE`: the run starts, a stage commits its gains, the run completes or reverts;
/// - `WARNING`: a start refused;
/// - `ERROR`: a stage failed, which ends the run.
///
/// `None` for a progress this bridge doesn't know (`AutotuneProgress` is non-exhaustive).
pub(crate) fn statustext(report: &AutotuneReport) -> Option<STATUSTEXT_DATA> {
    use MavSeverity::*;
    let (severity, text) = match report.progress {
        AutotuneProgress::Started => (
            MAV_SEVERITY_NOTICE,
            format!("started, {} stages", report.stages),
        ),
        AutotuneProgress::Refused(StartError::NotArmed) => {
            (MAV_SEVERITY_WARNING, "not started: disarmed".into())
        }
        AutotuneProgress::Refused(StartError::Running) => {
            (MAV_SEVERITY_WARNING, "already running".into())
        }
        AutotuneProgress::StageStarted(target) => (
            MAV_SEVERITY_INFO,
            format!("{}/{} {}", report.stage, report.stages, name(target)),
        ),
        AutotuneProgress::Tried {
            target,
            gains,
            passed,
        } => (
            MAV_SEVERITY_INFO,
            format!(
                "{} {} {}",
                name(target),
                pid(gains),
                if passed { "pass" } else { "fail" }
            ),
        ),
        AutotuneProgress::StageDone {
            target,
            gains,
            outcome,
        } => (
            MAV_SEVERITY_NOTICE,
            format!("{} {} {}", name(target), outcome_word(outcome), pid(gains)),
        ),
        AutotuneProgress::Lowered { target, gains } => (
            MAV_SEVERITY_INFO,
            format!("{} lowered {}", name(target), pid(gains)),
        ),
        AutotuneProgress::Unsettled(target) => (
            MAV_SEVERITY_INFO,
            format!("{} unsettled, fitting again", name(target)),
        ),
        AutotuneProgress::StageFailed { target, reason } => (
            MAV_SEVERITY_ERROR,
            format!("{} failed: {}", name(target), reason_words(reason)),
        ),
        AutotuneProgress::Complete => (
            MAV_SEVERITY_NOTICE,
            "complete: keep, or stop to revert".into(),
        ),
        AutotuneProgress::Reverted => (MAV_SEVERITY_NOTICE, "reverted to the start gains".into()),
        AutotuneProgress::Idle => (MAV_SEVERITY_INFO, "nothing to stop".into()),
        _ => return None,
    };
    Some(encode::statustext_at(
        &format!("AutoTune: {text}"),
        severity,
    ))
}

fn name(target: TunedLoop) -> &'static str {
    match target {
        TunedLoop::RollRate => "roll rate",
        TunedLoop::PitchRate => "pitch rate",
        TunedLoop::YawRate => "yaw rate",
        TunedLoop::Roll => "roll",
        TunedLoop::Pitch => "pitch",
        TunedLoop::Heading => "heading",
    }
}

/// `gains` in three significant digits each ("P0.620 I0.310 D0.0100").
fn pid(gains: Gains) -> String {
    format!(
        "P{} I{} D{}",
        significant(gains.p),
        significant(gains.i),
        significant(gains.d)
    )
}

/// `value` to three significant digits: 12.3, 0.620, 0.0123.
fn significant(value: f32) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let magnitude = value.abs().log10().floor() as i32;
    let decimals = (2 - magnitude).max(0) as usize;
    format!("{value:.decimals$}")
}

/// `outcome` in one short word: a design's gains come from a fitted model, a step test's are
/// only tested; "designed" and "measured" would put the longest stage text past 50 bytes.
fn outcome_word(outcome: Outcome) -> &'static str {
    match outcome {
        Outcome::Tuned => "tuned",
        Outcome::Measured => "tested",
        Outcome::Designed => "fitted",
        Outcome::Kept(_) => "kept",
    }
}

fn reason_words(reason: AutotuneAbort) -> &'static str {
    match reason {
        AutotuneAbort::Tilt => "tilt",
        AutotuneAbort::Rate => "rate",
        AutotuneAbort::AltitudeLoss => "sank",
        AutotuneAbort::AirspeedLow => "too slow",
        AutotuneAbort::AirspeedHigh => "too fast",
        AutotuneAbort::NotSettled => "not settled",
        AutotuneAbort::Invalid => "steps clipped",
        AutotuneAbort::Stalled => "host stalled",
        AutotuneAbort::Probe(_) => "probe gave up",
        AutotuneAbort::Interrupted => "interrupted",
        AutotuneAbort::Rejected => "gains rejected",
        AutotuneAbort::Design(_) => "no design",
        AutotuneAbort::Unverified => "unverified",
        AutotuneAbort::HostFault => "host fault",
    }
}
