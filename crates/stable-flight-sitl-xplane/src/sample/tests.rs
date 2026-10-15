use std::time::Instant;

use assert2::{assert, check};
use xplane_bridge::testing::FakeXPlane;

use super::*;

/// A sample with a distinct value in every field.
pub(crate) fn hovering() -> XPlaneSample {
    XPlaneSample {
        sim_time_s: 100.0,
        quaternion: [1.0, 0.0, 0.0, 0.0],
        body_rates_degps: [1.0, 2.0, 3.0],
        local_position_m: [-237.0, 30.0, -33_867.0],
        local_velocity_mps: [0.1, 0.2, 0.3],
        local_acceleration_mps2: [0.01, 0.02, 0.03],
        latitude_deg: 46.3046,
        longitude_deg: 10.9969,
        elevation_m: 1520.0,
        origin_deg: [46.0, 11.0],
        true_airspeed_mps: 0.4,
        on_ground: false,
        paused: false,
    }
}

/// Every dataref is subscribed once, at the state rate.
#[test]
fn subscribes_every_dataref() {
    let mut fake = FakeXPlane::new();
    StateDatarefs::subscribe(&mut fake).expect("subscribed");
    let subscriptions = fake.subscriptions();
    check!(
        subscriptions
            .get(&xplane_bridge::datarefs::position::LOCAL_Z)
            .is_some()
    );
    check!(
        subscriptions
            .get(&xplane_bridge::datarefs::time::PAUSED)
            .is_some()
    );
    check!(
        subscriptions
            .get(&Dataref::<f32>::new(TRUE_AIRSPEED).expect("path"))
            .is_some()
    );
    check!(
        subscriptions
            .get(&Dataref::<bool>::new(ON_GROUND).expect("path"))
            .is_some()
    );
}

/// A sample reads back field for field once complete; until then it is `None` and the
/// missing datarefs are named.
#[test]
fn reads_a_complete_sample_and_names_the_missing() {
    let mut fake = FakeXPlane::new();
    let datarefs = StateDatarefs::subscribe(&mut fake).expect("subscribed");
    let mut latest = Latest::default();

    latest.apply(&datarefs.frame(&hovering(), Instant::now(), &["P", "paused"]));
    check!(datarefs.sample(&latest) == None);
    check!(datarefs.missing(&latest) == ["P", "paused"]);

    latest.apply(&datarefs.frame(&hovering(), Instant::now(), &[]));
    check!(datarefs.sample(&latest) == Some(hovering()));
    check!(datarefs.missing(&latest).is_empty());
}

/// Incomplete: nothing yet, a dataref missing, or one lagging the newest by more than the
/// allowed age.
#[test]
fn flags_missing_and_stale_datarefs() {
    let mut fake = FakeXPlane::new();
    let datarefs = StateDatarefs::subscribe(&mut fake).expect("subscribed");
    let max_age = Duration::from_millis(500);
    let start = Instant::now();
    let mut latest = Latest::default();
    check!(datarefs.incomplete(&latest, max_age));

    latest.apply(&datarefs.frame(&hovering(), start, &["R"]));
    check!(datarefs.incomplete(&latest, max_age));

    latest.apply(&datarefs.frame(&hovering(), start, &[]));
    check!(!datarefs.incomplete(&latest, max_age));

    // Everything but R keeps arriving.
    latest.apply(&datarefs.frame(&hovering(), start + Duration::from_millis(400), &["R"]));
    check!(!datarefs.incomplete(&latest, max_age));
    latest.apply(&datarefs.frame(&hovering(), start + Duration::from_millis(600), &["R"]));
    check!(datarefs.incomplete(&latest, max_age));
}

/// `wait_ready` returns the first complete sample, resubscribing while one is missing.
#[test]
fn waits_until_every_dataref_arrives() {
    let mut fake = FakeXPlane::new();
    let datarefs = StateDatarefs::subscribe(&mut fake).expect("subscribed");
    fake.push_frame(datarefs.frame(&hovering(), Instant::now(), &["P"]));
    // Nothing for longer than a resubscribe interval, then the rest.
    for _ in 0..(2 * RESUBSCRIBE_INTERVAL.as_millis() / POLL_INTERVAL.as_millis()) {
        fake.push_silence();
    }
    fake.push_frame(datarefs.frame(&hovering(), Instant::now(), &[]));
    let mut latest = Latest::default();

    let sample = wait_ready(&mut fake, &datarefs, &mut latest, Duration::from_secs(5));

    assert!(sample.expect("ready") == hovering());
    check!(fake.stats().resubscribes >= 1);
}

/// `wait_ready` gives up after the timeout, naming what never arrived, and passes bridge
/// errors through.
#[test]
fn reports_what_never_arrived() {
    let mut fake = FakeXPlane::new();
    let datarefs = StateDatarefs::subscribe(&mut fake).expect("subscribed");
    fake.push_frame(datarefs.frame(&hovering(), Instant::now(), &["true_airspeed"]));
    let mut latest = Latest::default();
    let timeout = Duration::from_millis(30);

    let error = wait_ready(&mut fake, &datarefs, &mut latest, timeout);
    check!(matches!(
        error,
        Err(ConnectError::NotReady { ref missing, .. }) if *missing == ["true_airspeed"]
    ));

    fake.fail_with(Some(std::io::ErrorKind::ConnectionRefused));
    let error = wait_ready(&mut fake, &datarefs, &mut latest, timeout);
    check!(matches!(error, Err(ConnectError::Bridge(_))));
}
