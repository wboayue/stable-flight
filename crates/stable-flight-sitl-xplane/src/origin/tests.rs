use assert2::{assert, check};

use super::*;
use crate::vehicle_state;

/// The spike's airport: local origin at (46, 11), aircraft about 33.9 km north of it
/// (xplane-bridge `design.md`, Spike findings: Reloads).
fn at_airport() -> XPlaneSample {
    XPlaneSample {
        sim_time_s: 100.0,
        quaternion: [1.0, 0.0, 0.0, 0.0],
        local_position_m: [-237.0, 30.0, -33_867.0],
        latitude_deg: 46.3046,
        longitude_deg: 10.9969,
        origin_deg: [46.0, 11.0],
        ..Default::default()
    }
}

/// `sample` flown north at `speed_mps` for `dt_s`: local z falls, latitude rises.
fn flown_north(sample: &XPlaneSample, speed_mps: f32, dt_s: f32) -> XPlaneSample {
    let mut next = *sample;
    next.sim_time_s += dt_s;
    next.local_velocity_mps = [0.0, 0.0, -speed_mps];
    next.local_position_m[2] -= speed_mps * dt_s;
    next.latitude_deg += speed_mps * dt_s / METERS_PER_DEGREE as f32;
    next
}

fn north_east(origin: &LocalOrigin, sample: &XPlaneSample) -> Vec2 {
    vehicle_state(sample, origin.offset_m()).position_m.xy()
}

/// The first sample anchors there: position (0, 0), the anchor at the aircraft.
#[test]
fn anchors_at_the_first_sample() {
    let mut origin = LocalOrigin::default();
    let sample = at_airport();
    let event = origin.update(&sample);

    let anchor = Anchor {
        latitude_deg: f64::from(sample.latitude_deg),
        longitude_deg: f64::from(sample.longitude_deg),
    };
    check!(event == Some(OriginEvent::Anchored(anchor)));
    check!(origin.anchor() == Some(anchor));
    check!(north_east(&origin, &sample).norm() < 1e-3);
}

/// Flying without an origin change changes nothing but position.
#[test]
fn follows_flight_without_events() {
    let mut origin = LocalOrigin::default();
    let start = at_airport();
    origin.update(&start);
    let mut sample = start;
    for _ in 0..100 {
        sample = flown_north(&sample, 20.0, 0.01);
        check!(origin.update(&sample) == None);
    }
    // 20 m north, as the (f32-rounded) local z moved.
    let flown_m = start.local_position_m[2] - sample.local_position_m[2];
    check!((flown_m - 20.0).abs() < 0.1);
    check!((north_east(&origin, &sample) - Vec2::new(flown_m, 0.0)).norm() < 1e-3);
}

/// X-Plane moving its local origin under a flying aircraft (a whole degree, local position
/// jumping by ~111 km) leaves position continuous: the offset absorbs the jump.
#[test]
fn keeps_position_continuous_when_the_origin_moves() {
    // (speed m/s, frame s)
    let rows = [(0.0, 0.01), (30.0, 0.01), (30.0, 0.5)];
    for (speed_mps, dt_s) in rows {
        let mut origin = LocalOrigin::default();
        let start = at_airport();
        origin.update(&start);
        let before = flown_north(&start, speed_mps, dt_s);
        origin.update(&before);
        let position_before = north_east(&origin, &before);

        // One degree north: local z grows by a degree's meters.
        let mut after = flown_north(&before, speed_mps, dt_s);
        after.origin_deg = [47.0, 11.0];
        after.local_position_m[2] += METERS_PER_DEGREE as f32;
        let event = origin.update(&after);

        check!(
            matches!(event, Some(OriginEvent::Shifted { .. })),
            "{speed_mps} {dt_s}"
        );
        let expected = position_before + Vec2::new(speed_mps * dt_s, 0.0);
        check!(
            (north_east(&origin, &after) - expected).norm() < 0.05,
            "{speed_mps} {dt_s}"
        );
    }
}

/// A jump farther than the aircraft could fly re-anchors there, whether or not X-Plane moved
/// its local origin with it.
#[test]
fn re_anchors_when_the_aircraft_relocates() {
    // (new local origin, local position, latitude, longitude): KSFO from the spike's airport,
    // and 5 km away in the same origin cell.
    let rows = [
        ([33.0, -122.0], [-42_413.0, 4.0, -45_366.0], 37.62, -122.38),
        ([46.0, 11.0], [-237.0, 30.0, -38_867.0], 46.3496, 10.9969),
    ];
    for (origin_deg, local_position_m, latitude_deg, longitude_deg) in rows {
        let mut origin = LocalOrigin::default();
        origin.update(&at_airport());
        let moved = XPlaneSample {
            sim_time_s: 102.0,
            origin_deg,
            local_position_m,
            latitude_deg,
            longitude_deg,
            ..at_airport()
        };
        let event = origin.update(&moved);

        let anchor = Anchor {
            latitude_deg: f64::from(latitude_deg),
            longitude_deg: f64::from(longitude_deg),
        };
        check!(
            event == Some(OriginEvent::Relocated(anchor)),
            "{origin_deg:?}"
        );
        check!(origin.anchor() == Some(anchor), "{origin_deg:?}");
        check!(north_east(&origin, &moved).norm() < 1e-3, "{origin_deg:?}");
    }
}

/// The relocation margin: a jump within 100 m of the flown distance is flight, past it a
/// relocation.
#[test]
fn tells_a_relocation_from_flight_by_the_margin() {
    // (jump north m, relocated)
    let rows = [(90.0, false), (110.0, true)];
    for (jump_m, relocated) in rows {
        let mut origin = LocalOrigin::default();
        let start = at_airport();
        origin.update(&start);
        let mut jumped = start;
        jumped.sim_time_s += 0.01;
        jumped.local_position_m[2] -= jump_m;
        let event = origin.update(&jumped);
        check!(
            matches!(event, Some(OriginEvent::Relocated(_))) == relocated,
            "{jump_m}"
        );
    }
}

/// Simulator time running backward (a restart) counts as no time, not negative reach.
#[test]
fn clamps_backward_time() {
    let mut origin = LocalOrigin::default();
    let start = at_airport();
    origin.update(&start);
    let mut restarted = flown_north(&start, 0.0, 0.0);
    restarted.sim_time_s = 1.0;
    restarted.local_position_m[2] -= 50.0;
    assert!(origin.update(&restarted) == None);
}
