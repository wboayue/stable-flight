use std::cell::RefCell;
use std::io::ErrorKind;
use std::rc::Rc;
use std::time::Instant;

use assert2::{assert, check};
use stable_flight::{Vec2, Vec3};
use xplane_bridge::testing::FakeXPlane;
use xplane_bridge::{Command, Frame, Rate, Stats, Sub, Value};

use super::*;
use crate::sample::tests::hovering;

/// A [`FakeXPlane`] the test keeps a handle to after the vehicle takes (and drops) it.
#[derive(Clone, Default)]
struct Shared(Rc<RefCell<FakeXPlane>>);

impl Shared {
    fn fake(&self) -> std::cell::RefMut<'_, FakeXPlane> {
        self.0.borrow_mut()
    }

    /// How many times `dataref` was written `value`.
    fn count_writes<T: Value>(&self, dataref: &Dataref<T, Writable>, value: T) -> usize {
        let path = dataref.path();
        self.0
            .borrow()
            .writes()
            .iter()
            .filter(|(p, v)| *p == path && *v == value.to_wire())
            .count()
    }
}

impl DatarefSource for Shared {
    fn subscribe<T: Value, A>(
        &mut self,
        dataref: &Dataref<T, A>,
        rate: Rate,
    ) -> Result<Sub<T>, XPlaneError> {
        self.fake().subscribe(dataref, rate)
    }

    fn unsubscribe<T: Value>(&mut self, sub: Sub<T>) -> Result<(), XPlaneError> {
        self.fake().unsubscribe(sub)
    }

    fn poll(&mut self) -> Result<Option<Frame>, XPlaneError> {
        self.fake().poll()
    }

    fn resubscribe(&mut self) -> Result<(), XPlaneError> {
        self.fake().resubscribe()
    }

    fn stats(&self) -> Stats {
        self.0.borrow().stats()
    }
}

impl DatarefSink for Shared {
    fn write<T: Value>(
        &mut self,
        dataref: &Dataref<T, Writable>,
        value: T,
    ) -> Result<(), XPlaneError> {
        self.fake().write(dataref, value)
    }

    fn command(&mut self, command: &Command) -> Result<(), XPlaneError> {
        self.fake().command(command)
    }
}

const DT: Duration = Duration::from_millis(4);

fn alia() -> ActuatorMapping {
    ActuatorMapping {
        motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
        idle_engines: &[4],
        neutral_wings: &[2],
    }
}

/// The handles the vehicle's subscriptions get: indices are allocated in order from the same
/// start in every `Protocol`.
fn handles() -> StateDatarefs {
    StateDatarefs::subscribe(&mut FakeXPlane::new()).expect("subscribed")
}

/// A vehicle connected to a fake that has sent `hovering()`.
fn connected(mapping: ActuatorMapping) -> (XPlaneVehicle<Shared>, Shared) {
    let shared = Shared::default();
    shared
        .fake()
        .push_frame(handles().frame(&hovering(), Instant::now(), &[]));
    let vehicle = XPlaneVehicle::connect(XPlaneConfiguration::new(shared.clone(), mapping))
        .expect("connected");
    (vehicle, shared)
}

fn engine(i: usize) -> Dataref<f32, Writable> {
    engine::THRO_USE.at(i).expect("engine")
}

/// Connecting waits for the state, then takes the throttles, zeroes every engine it writes,
/// and holds the neutral wings' surfaces at 0°; the first state is anchored at the aircraft.
#[test]
fn connects_and_takes_control() {
    let (mut vehicle, shared) = connected(alia());

    check!(shared.fake().last_write(&overrides::THROTTLES) == Some(true));
    check!(shared.fake().last_write(&overrides::CONTROL_SURFACES) == Some(true));
    for i in 0..5 {
        check!(
            shared.fake().last_write(&engine(i)) == Some(0.0),
            "engine {i}"
        );
    }
    check!(shared.fake().last_write(&engine(5)) == None);
    for surface in [wing::AILERON1_DEG, wing::ELEVATOR2_DEG, wing::RUDDER1_DEG] {
        check!(shared.fake().last_write(&surface.at(2).expect("wing")) == Some(0.0));
        check!(shared.fake().last_write(&surface.at(3).expect("wing")) == None);
    }

    let state = vehicle.state(Duration::ZERO);
    check!(state.position_m == Vec3::new(0.0, 0.0, hovering().elevation_m));
    let anchor = vehicle.anchor().expect("anchored");
    check!(anchor.latitude_deg == f64::from(hovering().latitude_deg));
}

/// Without neutral wings, the surfaces stay X-Plane's.
#[test]
fn leaves_surfaces_without_neutral_wings() {
    let (_vehicle, shared) = connected(ActuatorMapping {
        neutral_wings: &[],
        ..alia()
    });
    check!(shared.fake().last_write(&overrides::CONTROL_SURFACES) == None);
}

/// An invalid mapping fails before X-Plane is touched; a dead bridge and a silent X-Plane fail
/// with their own errors.
#[test]
fn reports_connect_failures() {
    let invalid = ActuatorMapping {
        idle_engines: &[1],
        ..alia()
    };
    let shared = Shared::default();
    let result = XPlaneVehicle::connect(XPlaneConfiguration::new(shared.clone(), invalid));
    check!(matches!(result, Err(ConnectError::Mapping(_))));
    check!(shared.fake().writes().is_empty());

    let shared = Shared::default();
    shared.fake().fail_with(Some(ErrorKind::ConnectionRefused));
    let result = XPlaneVehicle::connect(XPlaneConfiguration::new(shared, alia()));
    check!(matches!(result, Err(ConnectError::Bridge(_))));

    let config = XPlaneConfiguration {
        ready_timeout: Duration::from_millis(20),
        ..XPlaneConfiguration::new(Shared::default(), alia())
    };
    let result = XPlaneVehicle::connect(config);
    check!(
        matches!(result, Err(ConnectError::NotReady { ref missing, .. }) if missing.len() == 25)
    );
}

/// State holds between frames and follows each new one.
#[test]
fn holds_state_between_frames() {
    let (mut vehicle, shared) = connected(alia());
    let first = vehicle.state(Duration::ZERO);
    check!(vehicle.state(DT) == first);

    let mut moved = hovering();
    moved.sim_time_s += 0.01;
    moved.local_position_m[0] += 0.5;
    shared
        .fake()
        .push_frame(handles().frame(&moved, Instant::now(), &[]));
    let state = vehicle.state(DT);
    check!((state.position_m.xy() - Vec2::new(0.0, 0.5)).norm() < 1e-3);
}

/// Each tick writes the mapped engines' throttles and zeroes the idle one.
#[test]
fn writes_engines_each_tick() {
    let (mut vehicle, shared) = connected(alia());
    let command = ActuatorCommand {
        motors: [0.1, 0.2, 0.3, 0.4, 0.9, 0.9, 0.9, 0.9],
        ..Default::default()
    };
    vehicle.actuate(&command);

    let fake = shared.fake();
    check!(fake.last_write(&engine(1)) == Some(0.1));
    check!(fake.last_write(&engine(0)) == Some(0.2));
    check!(fake.last_write(&engine(2)) == Some(0.3));
    check!(fake.last_write(&engine(3)) == Some(0.4));
    check!(fake.last_write(&engine(4)) == Some(0.0));
    check!(fake.last_write(&engine(5)) == None);
}

/// A silence resubscribes every 0.25 s once it passes 0.5 s, and frames returning take control
/// again (a restarted X-Plane has cleared the overrides).
#[test]
fn resubscribes_through_a_silence_and_retakes_control() {
    let (mut vehicle, shared) = connected(alia());
    // Fresh frames for the first 0.4 s: nothing to resubscribe.
    let start = Instant::now();
    for i in 0..100 {
        let at = start + DT * i;
        shared
            .fake()
            .push_frame(handles().frame(&hovering(), at, &[]));
        vehicle.state(DT);
    }
    let before = shared.stats().resubscribes;
    check!(before == 0);

    // 1 s of silence: resubscribes from 0.5 s, every 0.25 s.
    for _ in 0..250 {
        vehicle.state(DT);
    }
    let during = shared.stats().resubscribes;
    check!((2..=3).contains(&during));

    let takeovers = shared.count_writes(&overrides::THROTTLES, true);
    shared
        .fake()
        .push_frame(handles().frame(&hovering(), Instant::now(), &[]));
    vehicle.state(DT);
    check!(shared.count_writes(&overrides::THROTTLES, true) == takeovers + 1);
}

/// While frames flow, a dataref that stopped arriving triggers a resubscribe.
#[test]
fn resubscribes_a_lost_dataref() {
    let (mut vehicle, shared) = connected(alia());
    let start = Instant::now();
    for i in 0..200u32 {
        let at = start + DT * i;
        shared
            .fake()
            .push_frame(handles().frame(&hovering(), at, &["local_vz"]));
        vehicle.state(DT);
    }
    check!(shared.stats().resubscribes >= 1);
}

/// Failed sends and receives are logged, not fatal: the tick goes on with the last state.
#[test]
fn survives_bridge_errors() {
    let (mut vehicle, shared) = connected(alia());
    let state = vehicle.state(Duration::ZERO);
    shared.fake().fail_with(Some(ErrorKind::ConnectionReset));
    check!(vehicle.state(DT) == state);
    vehicle.actuate(&ActuatorCommand::default());
    for _ in 0..200 {
        vehicle.state(DT);
    }
    shared.fake().fail_with(None);
}

/// Dropping the vehicle hands the throttles and surfaces back to X-Plane.
#[test]
fn hands_control_back_on_drop() {
    let (vehicle, shared) = connected(alia());
    drop(vehicle);
    check!(shared.fake().last_write(&overrides::THROTTLES) == Some(false));
    check!(shared.fake().last_write(&overrides::CONTROL_SURFACES) == Some(false));

    let (vehicle, shared) = connected(ActuatorMapping {
        neutral_wings: &[],
        ..alia()
    });
    drop(vehicle);
    assert!(shared.fake().last_write(&overrides::THROTTLES) == Some(false));
    check!(shared.fake().last_write(&overrides::CONTROL_SURFACES) == None);
}

/// A pause and a relocation are followed: the paused flag toggles, and a jump re-anchors.
#[test]
fn follows_pause_and_relocation() {
    let (mut vehicle, shared) = connected(alia());
    let mut paused = hovering();
    paused.paused = true;
    shared
        .fake()
        .push_frame(handles().frame(&paused, Instant::now(), &[]));
    vehicle.state(DT);
    check!(vehicle.paused);

    let relocated = XPlaneSample {
        sim_time_s: 200.0,
        origin_deg: [33.0, -122.0],
        local_position_m: [-42_413.0, 4.0, -45_366.0],
        latitude_deg: 37.62,
        longitude_deg: -122.38,
        ..hovering()
    };
    shared
        .fake()
        .push_frame(handles().frame(&relocated, Instant::now(), &[]));
    let state = vehicle.state(DT);
    check!(!vehicle.paused);
    check!(state.position_m.xy().norm() < 1e-3);
    check!(vehicle.anchor().expect("anchored").latitude_deg == f64::from(37.62f32));
}
