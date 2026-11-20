use assert2::check;
use stable_flight_link::stable_flight::tuning::autotune::KeepReason;

use super::*;

const GAINS: Gains = Gains {
    p: 0.62,
    i: 0.31,
    d: 0.01,
};

/// The widest gains to print: three significant digits under 0.01 take eight characters each.
const TINY: Gains = Gains {
    p: 0.00123,
    i: 0.00123,
    d: 0.00123,
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
            // The RealFlight QuadCopterX's tuned pitch rate (#703): 51 bytes at three digits.
            AutotuneProgress::Lowered {
                target: pitch,
                gains: Gains {
                    p: 0.7908,
                    i: 0.1582,
                    d: 0.001,
                },
            },
            MAV_SEVERITY_INFO,
            "AutoTune: pitch rate lowered P0.79 I0.16 D0.0010",
        ),
        (
            AutotuneProgress::Lowered {
                target: pitch,
                gains: TINY,
            },
            MAV_SEVERITY_INFO,
            "AutoTune: pitch rate lowered P0.001 I0.001 D0.001",
        ),
        (
            AutotuneProgress::StageDone {
                target: pitch,
                gains: TINY,
                outcome: Outcome::Measured,
            },
            MAV_SEVERITY_NOTICE,
            "AutoTune: pitch rate tested P0.001 I0.001 D0.001",
        ),
        (
            AutotuneProgress::Tried {
                target: pitch,
                gains: TINY,
                passed: true,
            },
            MAV_SEVERITY_INFO,
            "AutoTune: pitch rate P0.0012 I0.0012 D0.0012 pass",
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

/// Gains print in the significant digits asked for, all integer digits kept; zero and
/// non-finite values as they are.
#[test]
fn gains_print_in_significant_digits() {
    // (value, digits, text)
    let rows = [
        (0.62, 3, "0.620"),
        (0.0123, 3, "0.0123"),
        (12.34, 3, "12.3"),
        (1234.0, 3, "1234"),
        (-0.5, 3, "-0.500"),
        (0.00123, 2, "0.0012"),
        (0.00123, 1, "0.001"),
        (12.34, 1, "12"),
        (0.0, 3, "0"),
        (f32::NAN, 3, "NaN"),
    ];
    for (value, digits, expected) in rows {
        check!(
            significant(value, digits) == expected,
            "{value} at {digits}"
        );
    }
}

/// Every gain-carrying text fits `STATUSTEXT`'s 50 bytes, uncut, for every loop and outcome and
/// gains from 0.001 to 100.
#[test]
fn gain_texts_fit_uncut() {
    let loops = [
        TunedLoop::RollRate,
        TunedLoop::PitchRate,
        TunedLoop::YawRate,
        TunedLoop::Roll,
        TunedLoop::Pitch,
        TunedLoop::Heading,
    ];
    let outcomes = [
        Outcome::Tuned,
        Outcome::Measured,
        Outcome::Designed,
        Outcome::Kept(KeepReason::NoPass),
    ];
    for target in loops {
        for value in [0.00123, 0.0123, 0.123, 1.23, 12.3, 123.0] {
            let gains = Gains {
                p: value,
                i: value,
                d: value,
            };
            let mut progresses = vec![
                AutotuneProgress::Lowered { target, gains },
                AutotuneProgress::Tried {
                    target,
                    gains,
                    passed: false,
                },
            ];
            progresses.extend(outcomes.map(|outcome| AutotuneProgress::StageDone {
                target,
                gains,
                outcome,
            }));
            for progress in progresses {
                let (_, text) = line(&report(3, progress)).expect("known progress");
                check!(text.len() <= encode::MAX_STATUSTEXT_LEN, "{text}");
            }
        }
    }
}
