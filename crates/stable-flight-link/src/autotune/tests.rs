use assert2::check;
use stable_flight::tuning::autotune::KeepReason;
use stable_flight::tuning::{ProbeAbort, StepAbort, StepSummary};

use super::*;
use crate::{Downlink, MAX_FRAME_LEN, Message, Uplink, encode_vec};

const GAINS: Gains = Gains {
    p: 0.62,
    i: 0.31,
    d: 0.01,
};

/// Each event maps to its compact progress, measurements dropped; a probe's measurement isn't
/// reported.
#[test]
fn reports_each_event_without_its_measurements() {
    let roll = TunedLoop::RollRate;
    let rows = [
        (
            AutotuneEvent::StageStarted(roll),
            Some(AutotuneProgress::StageStarted(roll)),
        ),
        (
            AutotuneEvent::Tried {
                target: roll,
                gains: GAINS,
                passed: true,
                steps: StepSummary::default(),
            },
            Some(AutotuneProgress::Tried {
                target: roll,
                gains: GAINS,
                passed: true,
            }),
        ),
        (
            AutotuneEvent::StageDone {
                target: roll,
                gains: GAINS,
                outcome: Outcome::Kept(KeepReason::NoPass),
                steps: Some(StepSummary::default()),
                baseline: None,
            },
            Some(AutotuneProgress::StageDone {
                target: roll,
                gains: GAINS,
                outcome: Outcome::Kept(KeepReason::NoPass),
            }),
        ),
        (
            AutotuneEvent::Lowered {
                target: roll,
                gains: GAINS,
            },
            Some(AutotuneProgress::Lowered {
                target: roll,
                gains: GAINS,
            }),
        ),
        (
            AutotuneEvent::Unsettled {
                target: roll,
                gains: GAINS,
            },
            Some(AutotuneProgress::Unsettled(roll)),
        ),
        (
            AutotuneEvent::StageFailed {
                target: roll,
                reason: AutotuneAbort::Tilt,
            },
            Some(AutotuneProgress::StageFailed {
                target: roll,
                reason: AutotuneAbort::Tilt,
            }),
        ),
        (AutotuneEvent::Complete, Some(AutotuneProgress::Complete)),
        (AutotuneEvent::Reverted, Some(AutotuneProgress::Reverted)),
        (
            AutotuneEvent::Stalled {
                target: roll,
                measurement: None,
            },
            None,
        ),
    ];
    let status = AutotuneStatus::Settling { stage: 0 };
    for (event, progress) in rows {
        let report = AutotuneReport::from_event(&event, status, 9);
        check!(report.map(|r| r.progress) == progress, "{event:?}");
    }
}

/// The stage is the status's, 1-based; all of them once complete, 0 without one; counts
/// saturate at 255.
#[test]
fn numbers_the_stage_from_the_status() {
    let rows = [
        (AutotuneStatus::Idle, 9, (0, 9)),
        (AutotuneStatus::Settling { stage: 0 }, 9, (1, 9)),
        (
            AutotuneStatus::Running {
                stage: 4,
                target: TunedLoop::YawRate,
            },
            9,
            (5, 9),
        ),
        (
            AutotuneStatus::Failed {
                stage: 2,
                reason: AutotuneAbort::NotSettled,
            },
            9,
            (3, 9),
        ),
        (AutotuneStatus::Complete, 9, (9, 9)),
        (AutotuneStatus::Reverted, 9, (0, 9)),
        (AutotuneStatus::Settling { stage: 300 }, 400, (255, 255)),
    ];
    for (status, stages, expected) in rows {
        let report = AutotuneReport::new(AutotuneProgress::Started, status, stages);
        check!((report.stage, report.stages) == expected, "{status:?}");
    }
}

/// Every request and every progress round-trips through a frame, well within
/// [`MAX_FRAME_LEN`].
#[test]
fn requests_and_reports_round_trip() {
    for request in [AutotuneRequest::Start, AutotuneRequest::Stop] {
        let msg = Uplink::Autotune(request);
        let mut frame = encode_vec(&msg);
        check!(Uplink::decode(&mut frame) == Ok(msg));
    }
    let roll = TunedLoop::Pitch;
    let progresses = [
        AutotuneProgress::Started,
        AutotuneProgress::Refused(StartError::NotArmed),
        AutotuneProgress::Refused(StartError::Running),
        AutotuneProgress::StageStarted(roll),
        AutotuneProgress::Tried {
            target: roll,
            gains: GAINS,
            passed: false,
        },
        AutotuneProgress::StageDone {
            target: roll,
            gains: GAINS,
            outcome: Outcome::Tuned,
        },
        AutotuneProgress::Lowered {
            target: roll,
            gains: GAINS,
        },
        AutotuneProgress::Unsettled(roll),
        AutotuneProgress::StageFailed {
            target: roll,
            reason: AutotuneAbort::Probe(ProbeAbort::Step(StepAbort::Runaway)),
        },
        AutotuneProgress::Complete,
        AutotuneProgress::Reverted,
        AutotuneProgress::Idle,
    ];
    for progress in progresses {
        let msg = Downlink::Autotune(AutotuneReport {
            stage: u8::MAX,
            stages: u8::MAX,
            progress,
        });
        let mut frame = encode_vec(&msg);
        check!(frame.len() <= MAX_FRAME_LEN / 4, "{progress:?}");
        check!(Downlink::decode(&mut frame) == Ok(msg), "{progress:?}");
    }
}
