//! Autotune from a ground station: a companion flies the copter over the link
//! (`FcLink::wire_linked`) on attitude sticks whose throttle holds no height, as QGC's virtual
//! joystick does, and starts and stops an autotune with `Uplink::Autotune`. The host drives the
//! supervisor as a flight controller does: requests in, `AutotuneHold` engaged for the run,
//! reports out. The run completes from the hold, the reports arrive in order, and a stop puts the
//! start gains back, after the run or when the pilot's sticks take over mid-run.

mod common;

use std::time::Duration;

use assert2::{assert, check};
use stable_flight::commanders::{
    AutotuneHold, AutotuneHoldConfig, FailsafeCommander, FailsafeConfig, HoldCommander,
};
use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::autotune::{Autotune, Envelope, StagePlan};
use stable_flight::{AttitudeCommand, Command, Configurable, ControlInput, Vec3, WaypointCommand};
use stable_flight_link::autotune::{self, AutotuneProgress, AutotuneReport, AutotuneRequest};
use stable_flight_link::commander::LinkCommander;
use stable_flight_link::fc::{FcLink, FcLinkConfig, FcPoller};
use stable_flight_link::loopback::{Loopback, LoopbackLink, LoopbackRx};
use stable_flight_link::publisher::LinkPublisher;
use stable_flight_link::{Control, Downlink, MAX_FRAME_LEN, RecvMessage, SendMessage, Uplink};
use stable_flight_sim::airframes;

use common::{AUTOTUNE_TIMEOUT_S, Pipeline, Plant, TICK};

/// Loopback queue depth, in frames.
const DEPTH: usize = 64;

/// The companion's heartbeat period.
const HEARTBEAT: Duration = Duration::from_millis(100);

/// How often the companion resends its sticks: QGC's 25 Hz `MANUAL_CONTROL`. Sticks are a
/// stream, stale once they stop (`ControlInput::is_stream`).
const STICKS_PERIOD: Duration = Duration::from_millis(40);

/// Where the companion's waypoint climbs to before the sticks take over, in m.
const ALTITUDE_M: f32 = 5.0;

/// The sticks' throttle during the run: well under `quad_x`'s hover thrust (about 0.5), so the
/// vehicle would sink on it without the hold.
const STICK_THROTTLE: f32 = 0.3;

/// How far the hold may let the height move from where the run started, in m. The design
/// search moved it 0.027 m from the hold and 0.066 m from a waypoint hover (`common::hover`);
/// 0.1 m leaves a margin over both, where the sticks' throttle alone sinks the vehicle past the
/// envelope's 3 m (`Envelope::max_altitude_loss_m`) and fails the stage.
const MAX_HEIGHT_CHANGE_M: f32 = 0.1;

/// How far the hold may let the vehicle drift sideways, in m: the probes' tilt steps push it
/// 1.31 m from the hold and from a waypoint hover alike; 0.2 m of margin.
const MAX_DRIFT_M: f32 = 1.5;

type Commander = AutotuneHold<FailsafeCommander<LinkCommander<'static>, HoldCommander>>;
type Loop = Pipeline<Commander, LinkPublisher<'static, LoopbackTx>>;
type LoopbackTx = stable_flight_link::loopback::LoopbackTx<'static, DEPTH>;

/// A companion, the flight controller's link stack and loop, and the autotune its host runs.
struct Flight {
    companion: LoopbackLink<'static, DEPTH>,
    poller: FcPoller<'static, LoopbackRx<'static, DEPTH>>,
    flight_loop: Loop,
    autotune: Autotune,
    ticks: usize,
    buf: [u8; MAX_FRAME_LEN],
    /// Every autotune report the companion received, in order.
    reports: Vec<AutotuneReport>,
    /// The sticks the companion streams, once it flies them.
    sticks: Option<Command>,
}

impl Flight {
    /// `quad_x` on the ground, flown from the link, with the copter design search ready.
    fn new() -> Self {
        let loopback = Box::leak(Box::new(Loopback::<DEPTH>::new()));
        let (companion, fc_end) = loopback.split();
        let (rx, tx) = fc_end.into_halves();
        let fc_link = Box::leak(Box::new(FcLink::<16, 8>::new()));
        let (poller, telemetry, link_commander) =
            fc_link.wire_linked(rx, tx, FcLinkConfig::default());
        let failsafe = FailsafeCommander::new(
            link_commander,
            HoldCommander::default(),
            FailsafeConfig::default(),
        )
        .expect("valid failsafe");
        let commander =
            AutotuneHold::new(failsafe, AutotuneHoldConfig::default()).expect("valid hold");
        let flight_loop =
            common::pipeline(&Plant::clean(airframes::quad_x()), commander, telemetry);
        let plan = StagePlan::copter_design_search();
        Self {
            companion,
            poller,
            flight_loop,
            autotune: Autotune::new(&plan, Envelope::default()).expect("valid plan"),
            ticks: 0,
            buf: [0; MAX_FRAME_LEN],
            reports: Vec::new(),
            sticks: None,
        }
    }

    fn send(&mut self, uplink: Uplink) {
        self.companion
            .send_message(&uplink, &mut self.buf)
            .expect("loopback has room");
    }

    /// One tick: the companion's heartbeat, the loop, then the host's work between ticks.
    fn tick(&mut self) {
        if self
            .ticks
            .is_multiple_of(common::ticks(HEARTBEAT.as_secs_f32()))
        {
            self.send(Uplink::Control(Control::Heartbeat));
        }
        if let Some(sticks) = self.sticks
            && self
                .ticks
                .is_multiple_of(common::ticks(STICKS_PERIOD.as_secs_f32()))
        {
            self.send(Uplink::Control(Control::Command(sticks)));
        }
        self.ticks += 1;
        self.flight_loop.step(TICK);
        self.poller
            .poll(&mut self.flight_loop)
            .expect("loopback receives");
        let served = autotune::serve(
            &mut self.poller,
            &mut self.autotune,
            &mut self.flight_loop,
            TICK.as_secs_f32(),
        );
        assert!(served.dropped == 0);
        let mut recv = [0u8; MAX_FRAME_LEN];
        while let Some(message) = self
            .companion
            .recv_message::<Downlink>(&mut recv)
            .expect("valid frame")
        {
            if let Downlink::Autotune(report) = message {
                self.reports.push(report);
            }
        }
    }

    fn fly(&mut self, seconds: f32) {
        for _ in 0..common::ticks(seconds) {
            self.tick();
        }
    }

    /// Arms, climbs to `ALTITUDE_M` on a waypoint, settles, then flies the sticks.
    fn take_off(&mut self) {
        self.fly(2.0);
        self.send(Uplink::Control(Control::Arm(true)));
        let climb = Command::armed(ControlInput::Waypoint(WaypointCommand {
            altitude_m: ALTITUDE_M,
            ..Default::default()
        }));
        self.send(Uplink::Control(Control::Command(climb)));
        self.fly(common::HOVER_SETTLE_S);
        self.sticks(0.0);
    }

    /// Streams attitude sticks at `STICK_THROTTLE`, rolled `roll`, from the next tick.
    fn sticks(&mut self, roll: f32) {
        self.sticks = Some(Command::armed(ControlInput::Attitude(AttitudeCommand {
            throttle: STICK_THROTTLE,
            roll,
            ..Default::default()
        })));
    }

    /// Flies until the run ends or the timeout, returning the farthest the vehicle got from
    /// `start_m`: (height, sideways), in m.
    fn fly_run(&mut self, start_m: Vec3) -> (f32, f32) {
        let (mut height_m, mut drift_m): (f32, f32) = (0.0, 0.0);
        for _ in 0..common::ticks(AUTOTUNE_TIMEOUT_S) {
            self.tick();
            let moved_m = self.flight_loop.state().position_m - start_m;
            height_m = height_m.max(moved_m.z.abs());
            drift_m = drift_m.max(moved_m.xy().norm());
            if !self.autotune.is_running() && self.reports.len() > 1 {
                break;
            }
        }
        // Let the last report reach the companion.
        self.fly(0.1);
        (height_m, drift_m)
    }
}

/// Started from the link on sticks that would sink it, the copter holds where it was, the run
/// completes, and the reports say so in order; the tuned gains fly until a stop reverts them.
#[test]
fn a_link_started_autotune_completes_from_the_hold_and_reverts_on_stop() {
    let mut flight = Flight::new();
    flight.take_off();
    let start_gains = *flight.flight_loop.stabilizer().config();
    let start_m = flight.flight_loop.state().position_m;

    flight.send(Uplink::Autotune(AutotuneRequest::Start));
    let (height_m, drift_m) = flight.fly_run(start_m);

    let progress: Vec<_> = flight.reports.iter().map(|r| r.progress).collect();
    check!(progress.first() == Some(&AutotuneProgress::Started));
    check!(
        progress.last() == Some(&AutotuneProgress::Complete),
        "{progress:?}"
    );
    let started: Vec<_> = flight
        .reports
        .iter()
        .filter_map(|r| match r.progress {
            AutotuneProgress::StageStarted(target) => Some((r.stage, target)),
            _ => None,
        })
        .collect();
    let plan = StagePlan::copter_design_search();
    let expected: Vec<_> = (1..).zip(plan.iter().map(|stage| stage.target)).collect();
    check!(started == expected);
    // Each stage's result carries the stage it ran in.
    let done: Vec<_> = flight
        .reports
        .iter()
        .filter_map(|r| match r.progress {
            AutotuneProgress::StageDone { target, .. } => Some((r.stage, target)),
            _ => None,
        })
        .collect();
    check!(done == expected);
    check!(
        !progress
            .iter()
            .any(|p| matches!(p, AutotuneProgress::StageFailed { .. })),
        "{progress:?}"
    );
    check!(flight.reports.iter().all(|r| r.stages == 9));
    check!(height_m < MAX_HEIGHT_CHANGE_M);
    check!(drift_m < MAX_DRIFT_M);
    check!(!flight.flight_loop.commander().is_engaged());
    let tuned = *flight.flight_loop.stabilizer().config();
    check!(tuned != start_gains);
    check!(TunedLoop::RollRate.gains(&tuned) != TunedLoop::RollRate.gains(&start_gains));

    // After the run, the sticks fly again; a stop puts the start gains back.
    flight.reports.clear();
    flight.send(Uplink::Autotune(AutotuneRequest::Stop));
    flight.fly(0.1);
    let progress: Vec<_> = flight.reports.iter().map(|r| r.progress).collect();
    check!(progress == [AutotuneProgress::Reverted]);
    check!(*flight.flight_loop.stabilizer().config() == start_gains);

    // A second stop has nothing to undo.
    flight.reports.clear();
    flight.send(Uplink::Autotune(AutotuneRequest::Stop));
    flight.fly(0.1);
    let progress: Vec<_> = flight.reports.iter().map(|r| r.progress).collect();
    check!(progress == [AutotuneProgress::Idle]);
}

/// The pilot's roll stick mid-run takes the vehicle back from the hold and reverts the run;
/// a start on the ground is refused.
#[test]
fn the_pilots_sticks_end_a_link_started_autotune() {
    let mut flight = Flight::new();
    flight.send(Uplink::Autotune(AutotuneRequest::Start));
    flight.fly(0.1);
    let progress: Vec<_> = flight.reports.iter().map(|r| r.progress).collect();
    check!(
        progress
            == [AutotuneProgress::Refused(
                stable_flight::tuning::autotune::StartError::NotArmed
            )]
    );

    flight.take_off();
    let start_gains = *flight.flight_loop.stabilizer().config();
    flight.reports.clear();
    flight.send(Uplink::Autotune(AutotuneRequest::Start));
    flight.fly(20.0);
    check!(flight.autotune.is_running());
    check!(flight.flight_loop.commander().is_engaged());

    flight.sticks(0.2);
    flight.fly(0.1);
    let progress: Vec<_> = flight.reports.iter().map(|r| r.progress).collect();
    check!(
        progress.last() == Some(&AutotuneProgress::Reverted),
        "{progress:?}"
    );
    check!(!flight.autotune.is_running());
    check!(!flight.flight_loop.commander().is_engaged());
    check!(*flight.flight_loop.stabilizer().config() == start_gains);
}
