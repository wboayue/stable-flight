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
    let (severity, text) = line(report)?;
    Some(encode::statustext_at(&text, severity))
}

/// [`statustext`]'s text, prefix included, before encoding: within 50 bytes, so nothing is cut.
fn line(report: &AutotuneReport) -> Option<(MavSeverity, String)> {
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
            with_gains(gains, |pid| {
                format!(
                    "{} {pid} {}",
                    name(target),
                    if passed { "pass" } else { "fail" }
                )
            }),
        ),
        AutotuneProgress::StageDone {
            target,
            gains,
            outcome,
        } => (
            MAV_SEVERITY_NOTICE,
            with_gains(gains, |pid| {
                format!("{} {} {pid}", name(target), outcome_word(outcome))
            }),
        ),
        AutotuneProgress::Lowered { target, gains } => (
            MAV_SEVERITY_INFO,
            with_gains(gains, |pid| format!("{} lowered {pid}", name(target))),
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
    Some((severity, format!("{PREFIX}{text}")))
}

/// What every autotune text starts with, as ArduPilot's do.
const PREFIX: &str = "AutoTune: ";

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

/// The text `around` makes of `gains`, with as many significant digits per gain, three down
/// to one, as keep it and [`PREFIX`] within `STATUSTEXT`'s 50 bytes. Three fit any gain of
/// 0.01 or more; smaller ones (a D of 0.00123 is eight characters with its letter) take fewer,
/// rather than the text being cut mid-digit. At one digit the longest line, a lowered pitch
/// rate, is 49 bytes.
fn with_gains(gains: Gains, around: impl Fn(&str) -> String) -> String {
    let fits = |text: &String| PREFIX.len() + text.len() <= encode::MAX_STATUSTEXT_LEN;
    let mut text = around(&pid(gains, 3));
    for digits in [2, 1] {
        if fits(&text) {
            break;
        }
        text = around(&pid(gains, digits));
    }
    text
}

/// `gains` in `digits` significant digits each ("P0.620 I0.310 D0.0100" at three).
fn pid(gains: Gains, digits: i32) -> String {
    format!(
        "P{} I{} D{}",
        significant(gains.p, digits),
        significant(gains.i, digits),
        significant(gains.d, digits)
    )
}

/// `value` to `digits` significant digits (at three: 12.3, 0.620, 0.0123), or all its integer
/// digits when it has more.
fn significant(value: f32, digits: i32) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let magnitude = value.abs().log10().floor() as i32;
    let decimals = (digits - 1 - magnitude).max(0) as usize;
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
