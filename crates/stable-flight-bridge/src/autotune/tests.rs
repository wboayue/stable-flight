use assert2::check;
use stable_flight_link::stable_flight::tuning::autotune::KeepReason;

use super::*;

const GAINS: Gains = Gains {
    p: 0.62,
    i: 0.31,
    d: 0.01,
};

fn report(stage: u8, progress: AutotuneProgress) -> AutotuneReport {
    AutotuneReport {
        stage,
        stages: 9,
        progress,
    }
}

/// Each report reads as its text at its severity, within `STATUSTEXT`'s 50 bytes.
#[test]
fn each_report_reads_as_text() {
    use MavSeverity::*;
    let pitch = TunedLoop::PitchRate;
    let rows = [
        (
            AutotuneProgress::Started,
            MAV_SEVERITY_NOTICE,
            "AutoTune: started, 9 stages",
        ),
        (
            AutotuneProgress::Refused(StartError::NotArmed),
            MAV_SEVERITY_WARNING,
            "AutoTune: not started: disarmed",
        ),
        (
            AutotuneProgress::Refused(StartError::Running),
            MAV_SEVERITY_WARNING,
            "AutoTune: already running",
        ),
        (
            AutotuneProgress::StageStarted(pitch),
            MAV_SEVERITY_INFO,
            "AutoTune: 3/9 pitch rate",
        ),
        (
            AutotuneProgress::Tried {
                target: pitch,
                gains: GAINS,
                passed: false,
            },
            MAV_SEVERITY_INFO,
            "AutoTune: pitch rate P0.620 I0.310 D0.0100 fail",
        ),
        (
            AutotuneProgress::StageDone {
                target: pitch,
                gains: GAINS,
                outcome: Outcome::Designed,
            },
            MAV_SEVERITY_NOTICE,
            "AutoTune: pitch rate fitted P0.620 I0.310 D0.0100",
        ),
        (
            AutotuneProgress::StageDone {
                target: TunedLoop::Roll,
                gains: Gains {
                    p: 12.34,
                    i: 0.0,
                    d: 0.0,
                },
                outcome: Outcome::Kept(KeepReason::NoPass),
            },
            MAV_SEVERITY_NOTICE,
            "AutoTune: roll kept P12.3 I0 D0",
        ),
        (
            AutotuneProgress::Lowered {
                target: TunedLoop::YawRate,
                gains: GAINS,
            },
            MAV_SEVERITY_INFO,
            "AutoTune: yaw rate lowered P0.620 I0.310 D0.0100",
        ),
        (
            AutotuneProgress::Unsettled(pitch),
            MAV_SEVERITY_INFO,
            "AutoTune: pitch rate unsettled, fitting again",
        ),
        (
            AutotuneProgress::StageFailed {
                target: TunedLoop::Pitch,
                reason: AutotuneAbort::AltitudeLoss,
            },
            MAV_SEVERITY_ERROR,
            "AutoTune: pitch failed: sank",
        ),
        (
            AutotuneProgress::Complete,
            MAV_SEVERITY_NOTICE,
            "AutoTune: complete: keep, or stop to revert",
        ),
        (
            AutotuneProgress::Reverted,
            MAV_SEVERITY_NOTICE,
            "AutoTune: reverted to the start gains",
        ),
        (
            AutotuneProgress::Idle,
            MAV_SEVERITY_INFO,
            "AutoTune: nothing to stop",
        ),
    ];
    for (progress, severity, text) in rows {
        let data = statustext(&report(3, progress)).expect("known progress");
        check!(data.severity == severity, "{text}");
        check!(data.text.to_str().ok() == Some(text));
        check!(text.len() <= 50, "{text}");
    }
}

/// Gains print in three significant digits; zero and non-finite values as they are.
#[test]
fn gains_print_in_three_significant_digits() {
    let rows = [
        (0.62, "0.620"),
        (0.0123, "0.0123"),
        (12.34, "12.3"),
        (1234.0, "1234"),
        (-0.5, "-0.500"),
        (0.0, "0"),
        (f32::NAN, "NaN"),
    ];
    for (value, expected) in rows {
        check!(significant(value) == expected, "{value}");
    }
}
