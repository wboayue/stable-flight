//! Autotune over the link: a ground station starts or stops a run ([`AutotuneRequest`]), and
//! the flight controller reports its progress ([`AutotuneReport`]). The flight controller's
//! host drives its [`Autotune`](stable_flight::tuning::autotune::Autotune) supervisor: it takes
//! requests from [`FcPoller::autotune_request`](crate::fc::FcPoller::autotune_request) and
//! hands reports to [`FcPoller::report_autotune`](crate::fc::FcPoller::report_autotune).

use serde::{Deserialize, Serialize};
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{
    AutotuneAbort, AutotuneEvent, AutotuneStatus, Outcome, StartError,
};

#[cfg(test)]
mod tests;

/// What a ground station asks of the flight controller's autotune: [`Uplink::Autotune`](crate::Uplink::Autotune).
/// The flight controller picks the plan (its airframe's), so a ground station can't run one
/// meant for another size of vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AutotuneRequest {
    /// Start a run where the vehicle is; answered with [`AutotuneProgress::Started`] or
    /// [`AutotuneProgress::Refused`].
    Start,
    /// Undo the run, mid-run or after it ended, putting back the gains it started from;
    /// answered with [`AutotuneProgress::Reverted`], or [`AutotuneProgress::Idle`] when there
    /// was no run to undo.
    Stop,
}

/// One step of a run, compact enough for the link: [`Downlink::Autotune`](crate::Downlink::Autotune).
///
/// # Examples
///
/// ```
/// use stable_flight_link::autotune::{AutotuneProgress, AutotuneReport};
/// use stable_flight_link::stable_flight::stabilizers::copter::TunedLoop;
/// use stable_flight_link::stable_flight::tuning::autotune::{AutotuneEvent, AutotuneStatus};
/// use stable_flight_link::{Downlink, MAX_FRAME_LEN, Message};
///
/// let event = AutotuneEvent::StageStarted(TunedLoop::RollRate);
/// let status = AutotuneStatus::Settling { stage: 0 };
/// let report = AutotuneReport::from_event(&event, status, 9).unwrap();
/// assert_eq!((report.stage, report.stages), (1, 9));
/// assert_eq!(report.progress, AutotuneProgress::StageStarted(TunedLoop::RollRate));
///
/// let mut frame = [0u8; MAX_FRAME_LEN];
/// let len = Downlink::Autotune(report).encode(&mut frame).unwrap();
/// assert_eq!(Downlink::decode(&mut frame[..len]), Ok(Downlink::Autotune(report)));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct AutotuneReport {
    /// The stage it concerns, 1-based; every stage after completion, 0 when none (before a
    /// start, refused, reverted).
    pub stage: u8,
    /// The plan's stage count.
    pub stages: u8,
    /// What happened.
    pub progress: AutotuneProgress,
}

/// What happened in a run: the supervisor's [`AutotuneEvent`]s without their measurements,
/// plus the answers to an [`AutotuneRequest`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum AutotuneProgress {
    /// The run started.
    Started,
    /// The run didn't start, and why.
    Refused(StartError),
    /// A stage started on a loop ([`AutotuneEvent::StageStarted`]).
    StageStarted(TunedLoop),
    /// A search judged a gain ([`AutotuneEvent::Tried`]).
    Tried {
        /// The loop.
        target: TunedLoop,
        /// The gains flown.
        gains: Gains,
        /// Whether they met the criterion.
        passed: bool,
    },
    /// A stage committed its loop's gains ([`AutotuneEvent::StageDone`]).
    StageDone {
        /// The loop.
        target: TunedLoop,
        /// The gains committed.
        gains: Gains,
        /// What the stage did to them.
        outcome: Outcome,
    },
    /// A search lowered a start that wouldn't settle ([`AutotuneEvent::Lowered`]).
    Lowered {
        /// The loop.
        target: TunedLoop,
        /// The gains committed.
        gains: Gains,
    },
    /// A design didn't hold its settle and fits again ([`AutotuneEvent::Unsettled`]).
    Unsettled(TunedLoop),
    /// A stage failed and the run ended ([`AutotuneEvent::StageFailed`]).
    StageFailed {
        /// The loop.
        target: TunedLoop,
        /// Why.
        reason: AutotuneAbort,
    },
    /// Every stage committed; the tuned gains fly ([`AutotuneEvent::Complete`]).
    Complete,
    /// The run's start gains are back ([`AutotuneEvent::Reverted`]).
    Reverted,
    /// An [`AutotuneRequest::Stop`] found no run to undo.
    Idle,
}

impl AutotuneReport {
    /// `progress` with the stage from `status`, the supervisor's status after it, in a plan of
    /// `stages` stages (saturating at 255).
    pub fn new(progress: AutotuneProgress, status: AutotuneStatus, stages: usize) -> Self {
        let stages = saturate(stages);
        let stage = match status {
            AutotuneStatus::Settling { stage }
            | AutotuneStatus::Running { stage, .. }
            | AutotuneStatus::Failed { stage, .. } => saturate(stage.saturating_add(1)),
            AutotuneStatus::Complete => stages,
            AutotuneStatus::Idle | AutotuneStatus::Reverted => 0,
        };
        Self {
            stage,
            stages,
            progress,
        }
    }

    /// The report of `event`, with the stage from `status` (as in [`new`](Self::new)); `None`
    /// for a probe's measurement ([`AutotuneEvent::Measured`], [`AutotuneEvent::Stalled`]),
    /// whose data is too large for the link and only of use on the flight controller's log.
    pub fn from_event(
        event: &AutotuneEvent,
        status: AutotuneStatus,
        stages: usize,
    ) -> Option<Self> {
        let progress = match *event {
            AutotuneEvent::StageStarted(target) => AutotuneProgress::StageStarted(target),
            AutotuneEvent::Tried {
                target,
                gains,
                passed,
                ..
            } => AutotuneProgress::Tried {
                target,
                gains,
                passed,
            },
            AutotuneEvent::StageDone {
                target,
                gains,
                outcome,
                ..
            } => AutotuneProgress::StageDone {
                target,
                gains,
                outcome,
            },
            AutotuneEvent::Lowered { target, gains } => AutotuneProgress::Lowered { target, gains },
            AutotuneEvent::Unsettled { target, .. } => AutotuneProgress::Unsettled(target),
            AutotuneEvent::StageFailed { target, reason } => {
                AutotuneProgress::StageFailed { target, reason }
            }
            AutotuneEvent::Complete => AutotuneProgress::Complete,
            AutotuneEvent::Reverted => AutotuneProgress::Reverted,
            AutotuneEvent::Measured { .. } | AutotuneEvent::Stalled { .. } => return None,
            // `AutotuneEvent` is non-exhaustive: a new event isn't reported until mapped here.
            _ => return None,
        };
        Some(Self::new(progress, status, stages))
    }
}

fn saturate(count: usize) -> u8 {
    u8::try_from(count).unwrap_or(u8::MAX)
}
