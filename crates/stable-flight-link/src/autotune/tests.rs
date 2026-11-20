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

/// An event's stage is the one it ran in, from the status before the update that returned it;
/// completion counts every stage, and a revert none.
#[test]
fn numbers_an_event_from_the_status_before_it() {
    let roll = TunedLoop::Roll;
    let running = AutotuneStatus::Running {
        stage: 2,
        target: roll,
    };
    let done = AutotuneEvent::StageDone {
        target: roll,
        gains: GAINS,
        outcome: Outcome::Tuned,
        steps: None,
        baseline: None,
    };
    let rows = [
        ("stage done", done, running, 3),
        (
            "stage failed",
            AutotuneEvent::StageFailed {
                target: roll,
                reason: AutotuneAbort::Rate,
            },
            running,
            3,
        ),
        (
            "stage started",
            AutotuneEvent::StageStarted(roll),
            AutotuneStatus::Settling { stage: 3 },
            4,
        ),
        (
            "complete",
            AutotuneEvent::Complete,
            AutotuneStatus::Complete,
            9,
        ),
        ("disarmed mid-run", AutotuneEvent::Reverted, running, 0),
    ];
    for (case, event, before, stage) in rows {
        let report = AutotuneReport::from_event(&event, before, 9).expect("reported");
        check!(report.stage == stage, "{case}");
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

/// A stop and the pilot's take-over in one [`serve`] report one revert: the stop's.
#[test]
fn a_stop_and_a_take_over_together_revert_once() {
    use core::cell::Cell;
    use core::time::Duration;
    use std::rc::Rc;

    use stable_flight::commanders::AutotuneHoldConfig;
    use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
    use stable_flight::mixers::TableMixer;
    use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
    use stable_flight::testing::NullVehicle;
    use stable_flight::tuning::autotune::{Envelope, StagePlan};
    use stable_flight::{AttitudeCommand, Command, ControlInput, VehicleState};

    use crate::fc::{FcLink, FcLinkConfig};
    use crate::loopback::Loopback;
    use crate::{RecvMessage, SendMessage};

    struct Sticks(Rc<Cell<Command>>);
    impl Commander for Sticks {
        fn next_command(&mut self, _dt: f32, _state: &VehicleState) -> Command {
            self.0.get()
        }
    }
    let sticks = |roll| {
        Command::armed(ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            roll,
            ..Default::default()
        }))
    };
    const DT: f32 = 0.004;

    let loopback = Loopback::<64>::new();
    let (mut companion, fc_end) = loopback.split();
    let (rx, tx) = fc_end.into_halves();
    let mut link = FcLink::<16, 8>::new();
    let (mut poller, telemetry) = link.wire_with(rx, tx, FcLinkConfig::default());
    let command = Rc::new(Cell::new(sticks(0.0)));
    let hold = AutotuneHold::new(Sticks(Rc::clone(&command)), AutotuneHoldConfig::default())
        .expect("valid");
    let state = VehicleState {
        position_m: [0.0, 0.0, 10.0].into(),
        ..Default::default()
    };
    let mut flight_loop = FlightLoop::new(
        hold,
        NullVehicle::with_state(state),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        TableMixer::quad_x(),
        telemetry,
    );
    let mut autotune =
        Autotune::new(&StagePlan::copter_design_search(), Envelope::default()).expect("valid");
    let mut buf = [0u8; MAX_FRAME_LEN];
    let request = |companion: &mut _, request| {
        SendMessage::send_message(
            companion,
            &Uplink::Autotune(request),
            &mut [0u8; MAX_FRAME_LEN],
        )
        .expect("sent");
    };
    flight_loop.prime();
    flight_loop.step(Duration::from_secs_f32(DT));

    request(&mut companion, AutotuneRequest::Start);
    poller.poll(&mut flight_loop).expect("polled");
    let served = serve(&mut poller, &mut autotune, &mut flight_loop, DT);
    assert2::assert!(served.answer == Some(AutotuneProgress::Started));

    command.set(sticks(0.5));
    flight_loop.step(Duration::from_secs_f32(DT));
    request(&mut companion, AutotuneRequest::Stop);
    poller.poll(&mut flight_loop).expect("polled");
    let served = serve(&mut poller, &mut autotune, &mut flight_loop, DT);
    check!(served.answer == Some(AutotuneProgress::Reverted));
    flight_loop.step(Duration::from_secs_f32(DT));

    let mut reverts = 0;
    while let Some(msg) = companion
        .recv_message::<Downlink>(&mut buf)
        .expect("decodes")
    {
        if let Downlink::Autotune(report) = msg {
            reverts += usize::from(report.progress == AutotuneProgress::Reverted);
        }
    }
    check!(reverts == 1);
}
