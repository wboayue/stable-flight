//! Autotune over the link: a ground station starts or stops a run ([`AutotuneRequest`]), and
//! the flight controller reports its progress ([`AutotuneReport`]). The flight controller's
//! host drives its [`Autotune`] supervisor between ticks with [`serve`]: it takes the requests
//! from [`FcPoller::autotune_request`], flies the run from an [`AutotuneHold`], and hands each
//! step to [`FcPoller::report_autotune`].

use serde::{Deserialize, Serialize};
use stable_flight::commanders::AutotuneHold;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{
    Autotune, AutotuneAbort, AutotuneEvent, AutotuneStatus, CopterSide, Outcome, SideTunable,
    StartError, held,
};
use stable_flight::{Commander, FlightLoop};

use crate::LinkRx;
use crate::fc::FcPoller;

#[cfg(test)]
mod tests;

/// What a ground station asks of the flight controller's autotune:
/// [`Uplink::Autotune`](crate::Uplink::Autotune). The flight controller picks the plan (its airframe's), so a ground station can't run one
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

/// One step of a run, compact enough for the link:
/// [`Downlink::Autotune`](crate::Downlink::Autotune).
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
    /// The stage it concerns, 1-based: the stage a step ran in, the first on a start, every
    /// stage on completion, the running one when a start is refused mid-run, 0 when none
    /// (refused before a start, reverted, nothing to stop).
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
    /// `progress` with the stage from `status`, the supervisor's status once it happened, in a
    /// plan of `stages` stages (saturating at 255): the answer to a request.
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

    /// The report of `event`, with the stage from `before`, the supervisor's status before the
    /// [`update`](Autotune::update) that returned it: after a stage's last step the supervisor
    /// has moved on to the next stage. A revert concerns no stage (0). `None` for a probe's
    /// measurement ([`AutotuneEvent::Measured`], [`AutotuneEvent::Stalled`]), whose data is too
    /// large for the link and only of use on the flight controller's log.
    pub fn from_event(
        event: &AutotuneEvent,
        before: AutotuneStatus,
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
            AutotuneEvent::Reverted => {
                return Some(Self::new(
                    AutotuneProgress::Reverted,
                    AutotuneStatus::Reverted,
                    stages,
                ));
            }
            AutotuneEvent::Measured { .. } | AutotuneEvent::Stalled { .. } => return None,
            // `AutotuneEvent` is non-exhaustive: a new event isn't reported until mapped here.
            _ => return None,
        };
        Some(Self::new(progress, before, stages))
    }
}

/// What one [`serve`] did, for the host to log.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Served {
    /// The answer to the last request served (several in one call are rare), or
    /// [`AutotuneProgress::Reverted`] when the pilot's sticks took over.
    pub answer: Option<AutotuneProgress>,
    /// What the run's update did ([`Autotune::update`]), measurements included.
    pub event: Option<AutotuneEvent>,
    /// Reports the full queue dropped: the publisher hasn't run for several.
    pub dropped: u32,
}

/// Runs `autotune` between ticks for a flight controller flown from the link, from an
/// [`AutotuneHold`] around its commander ([`held`] couples the two), reporting each step down
/// the link:
///
/// 1. Serves `poller`'s requests: a [`Start`](AutotuneRequest::Start) starts the run and
///    engages the hold where the vehicle is ([`held::start`], or is refused); a
///    [`Stop`](AutotuneRequest::Stop) releases the hold and reverts the run ([`held::stop`]).
/// 2. Reverts the run if the pilot took over from the hold mid-run, then advances it by `dt` s
///    ([`held::step`]). The hold stays engaged once the run ends, until the pilot takes over
///    or a stop.
///
/// Each answer and step goes to [`FcPoller::report_autotune`]. The host still tells the
/// supervisor about its own timing ([`Autotune::note_stall`], [`Autotune::set_host_slow`],
/// [`Autotune::set_state_stale`]) and logs what [`Served`] returns.
///
/// # Examples
///
/// ```
/// use stable_flight_link::autotune::{AutotuneRequest, serve};
/// use stable_flight_link::fc::{FcLink, FcLinkConfig};
/// use stable_flight_link::loopback::Loopback;
/// use stable_flight_link::stable_flight::commanders::{AutotuneHold, AutotuneHoldConfig};
/// use stable_flight_link::stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight_link::stable_flight::mixers::TableMixer;
/// use stable_flight_link::stable_flight::stabilizers::copter::{
///     CopterStabilizer, CopterStabilizerConfig,
/// };
/// use stable_flight_link::stable_flight::testing::{ConstantCommander, NullVehicle};
/// use stable_flight_link::stable_flight::tuning::autotune::{Autotune, Envelope, StagePlan};
/// use stable_flight_link::stable_flight::FlightLoop;
/// use stable_flight_link::{MAX_FRAME_LEN, SendMessage, Uplink};
///
/// let loopback = Loopback::<16>::new();
/// let (mut companion, fc_end) = loopback.split();
/// let (rx, tx) = fc_end.into_halves();
/// let mut link = FcLink::<16, 8>::new();
/// let (mut poller, telemetry) = link.wire_with(rx, tx, FcLinkConfig::default());
/// let commander = AutotuneHold::new(ConstantCommander::default(), AutotuneHoldConfig::default())?;
/// let mut flight_loop = FlightLoop::new(
///     commander,
///     NullVehicle::default(),
///     CopterGuidance::new(CopterGuidanceConfig::default()),
///     CopterStabilizer::new(CopterStabilizerConfig::default()),
///     TableMixer::quad_x(),
///     telemetry,
/// );
/// let mut autotune = Autotune::new(&StagePlan::copter_design_search(), Envelope::default())?;
///
/// companion.send_message(&Uplink::Autotune(AutotuneRequest::Start), &mut [0u8; MAX_FRAME_LEN])?;
/// poller.poll(&mut flight_loop)?;
/// // Between ticks: the disarmed vehicle refuses the start.
/// let served = serve(&mut poller, &mut autotune, &mut flight_loop, 0.004);
/// assert!(served.answer.is_some());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn serve<R, C, V, G, S, M, T>(
    poller: &mut FcPoller<'_, R>,
    autotune: &mut Autotune,
    flight_loop: &mut FlightLoop<AutotuneHold<C>, V, G, S, M, T>,
    dt: f32,
) -> Served
where
    R: LinkRx,
    C: Commander,
    S: SideTunable<CopterSide>,
{
    let stages = autotune.stages();
    let mut served = Served {
        answer: None,
        event: None,
        dropped: 0,
    };
    let mut report = |poller: &mut FcPoller<'_, R>, report: AutotuneReport| {
        if !poller.report_autotune(report) {
            served.dropped += 1;
        }
    };

    while let Some(request) = poller.autotune_request() {
        let answer = match request {
            AutotuneRequest::Start => match held::start(autotune, flight_loop) {
                Ok(()) => AutotuneProgress::Started,
                Err(refused) => AutotuneProgress::Refused(refused),
            },
            AutotuneRequest::Stop => match held::stop(autotune, flight_loop) {
                Some(_) => AutotuneProgress::Reverted,
                None => AutotuneProgress::Idle,
            },
        };
        report(
            poller,
            AutotuneReport::new(answer, autotune.status(), stages),
        );
        served.answer = Some(answer);
    }

    let before = autotune.status();
    let step = held::step(autotune, flight_loop, dt);
    if step.reverted {
        let answer = AutotuneProgress::Reverted;
        report(
            poller,
            AutotuneReport::new(answer, autotune.status(), stages),
        );
        served.answer = Some(answer);
    }
    let event = step.event;
    if let Some(step_report) = event
        .as_ref()
        .and_then(|e| AutotuneReport::from_event(e, before, stages))
    {
        report(poller, step_report);
    }
    Served { event, ..served }
}

fn saturate(count: usize) -> u8 {
    u8::try_from(count).unwrap_or(u8::MAX)
}
