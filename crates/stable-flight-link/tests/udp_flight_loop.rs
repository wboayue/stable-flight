//! The flight-controller side of the link, end to end over localhost UDP: a `FlightLoop` whose
//! telemetry is a `LinkPublisher`, an `UplinkRouter` and `ParamServer` polled between ticks
//! (as `RealTimeRunner::run_with` does; wired by `testing::FcLink`), and a host socket standing
//! in for the bridge.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use assert2::{assert, check};
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::params::Params;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::{ConstantCommander, NullVehicle};
use stable_flight::{AttitudeCommand, Command, ControlInput, FlightLoop};
use stable_flight_link::publisher::LinkPublisher;
use stable_flight_link::testing::FcLink;
use stable_flight_link::udp::{UdpLink, UdpTx};
use stable_flight_link::{
    Downlink, MAX_FRAME_LEN, ParamRequest, ParamValue, RecvMessage, SendMessage, Uplink,
};

type TestLoop<'a> = FlightLoop<
    ConstantCommander,
    NullVehicle,
    CopterGuidance,
    CopterStabilizer,
    QuadCopterMixer,
    LinkPublisher<'a, UdpTx>,
>;

const DT: Duration = Duration::from_millis(4);
const TIMEOUT: Duration = Duration::from_secs(5);

/// The bridge's end: sends uplinks and collects downlinks.
struct Host {
    link: UdpLink,
    received: Vec<Downlink>,
}

impl Host {
    fn send(&mut self, msg: &Uplink) {
        let mut buf = [0u8; MAX_FRAME_LEN];
        self.link.send_message(msg, &mut buf).expect("sends");
    }

    fn collect(&mut self) {
        let mut buf = [0u8; MAX_FRAME_LEN];
        while let Some(msg) = self.link.recv_message(&mut buf).expect("valid downlink") {
            self.received.push(msg);
        }
    }

    fn params(&self) -> Vec<&ParamValue> {
        self.received
            .iter()
            .filter_map(|msg| match msg {
                Downlink::Param(value) => Some(value),
                _ => None,
            })
            .collect()
    }
}

#[test]
fn host_lists_and_sets_params_and_receives_telemetry() {
    // Both ends on ephemeral ports: bind the host first, connect it once the FC end exists.
    let host_socket = UdpSocket::bind("127.0.0.1:0").expect("binds");
    let link = UdpLink::bind("127.0.0.1:0", host_socket.local_addr().unwrap()).expect("binds");
    host_socket
        .connect(link.local_addr().unwrap())
        .expect("connects");
    let mut host = Host {
        link: UdpLink::try_from(host_socket).expect("connected"),
        received: Vec::new(),
    };
    let (rx, tx) = link.into_halves();

    let mut fc_link = FcLink::<16, 8>::new();
    let (mut poller, telemetry) = fc_link.wire(rx, tx);
    let mut flight_loop: TestLoop = FlightLoop::new(
        ConstantCommander::new(Command::armed(ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            ..Default::default()
        }))),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        telemetry,
    );
    let count = flight_loop.param_count();

    // One tick as `RealTimeRunner::run_with` runs it, then let the host read.
    let mut tick_until =
        |flight_loop: &mut TestLoop, host: &mut Host, done: &dyn Fn(&Host) -> bool| {
            let deadline = Instant::now() + TIMEOUT;
            while !done(host) {
                assert!(Instant::now() < deadline, "timed out");
                poller.poll(flight_loop).expect("link up");
                flight_loop.step(DT);
                std::thread::sleep(Duration::from_millis(1));
                host.collect();
            }
        };

    flight_loop.prime();
    host.send(&Uplink::Param(ParamRequest::List));
    tick_until(&mut flight_loop, &mut host, &|host| {
        host.params().len() >= count
    });

    let listing = host.params();
    check!(listing.len() == count);
    for (i, value) in listing.iter().enumerate() {
        check!(usize::from(value.index) == i);
        check!(usize::from(value.count) == count);
    }
    check!(
        host.received
            .iter()
            .any(|msg| matches!(msg, Downlink::State(_)))
    );
    check!(
        host.received
            .iter()
            .any(|msg| matches!(msg, Downlink::Heartbeat { armed: true, .. }))
    );

    host.received.clear();
    host.send(&Uplink::Param(ParamRequest::Set {
        name: "STAB_ATT_AMAX_X".try_into().unwrap(),
        value: 30.0,
    }));
    tick_until(&mut flight_loop, &mut host, &|host| {
        !host.params().is_empty()
    });

    let echo = host.params()[0];
    check!(echo.name.as_str() == "STAB_ATT_AMAX_X");
    check!(echo.value == 30.0);
    let stored = flight_loop.param("STAB_ATT_AMAX_X").expect("exists");
    check!((stored - 30.0).abs() < 1e-4);
}
