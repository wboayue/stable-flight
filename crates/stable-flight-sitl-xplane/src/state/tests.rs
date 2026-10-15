use approx::assert_relative_eq;
use assert2::check;
use stable_flight::testing::conformance::Scenario;

use super::*;

/// X-Plane's native sample for `scenario`: the quaternion of its Euler angles, rates in deg/s,
/// and the OpenGL local frame (x east, y up, z south), with up also as the elevation.
fn native_fixture(scenario: &Scenario) -> XPlaneSample {
    let [roll, pitch, heading] =
        [scenario.roll_deg, scenario.pitch_deg, scenario.heading_deg].map(f32::to_radians);
    let q = UnitQuaternion::from_euler_angles(roll, pitch, heading);
    let opengl = |[north, east, up]: [f32; 3]| [east, up, -north];
    let velocity = Vec3::from(scenario.velocity_neu_mps);
    XPlaneSample {
        sim_time_s: 12.0,
        quaternion: [q.w, q.i, q.j, q.k],
        body_rates_degps: scenario.body_rates_degps,
        local_position_m: opengl(scenario.position_neu_m),
        local_velocity_mps: opengl(scenario.velocity_neu_mps),
        local_acceleration_mps2: opengl(scenario.acceleration_neu_mps2),
        latitude_deg: 46.3,
        longitude_deg: 11.2,
        elevation_m: scenario.position_neu_m[2],
        origin_deg: [46.0, 11.0],
        true_airspeed_mps: velocity.norm(),
        on_ground: false,
        paused: false,
    }
}

/// Converts X-Plane's native sample for `scenario`, at the local origin.
fn state_for(scenario: &Scenario) -> VehicleState {
    vehicle_state(&native_fixture(scenario), Vec2::zeros())
}

stable_flight::conformance_tests! {
    state_for;
    conforms_level: LEVEL;
    conforms_roll_right_30: ROLL_RIGHT_30;
    conforms_pitch_up_15: PITCH_UP_15;
    conforms_heading_east: HEADING_EAST;
    conforms_banked_climb_east: BANKED_CLIMB_EAST;
    conforms_climbing: CLIMBING;
    conforms_moving_northeast: MOVING_NORTHEAST;
    conforms_yaw_right_rate: YAW_RIGHT_RATE;
    conforms_combined_rates: COMBINED_RATES;
}

/// Each local axis lands on its contract axis: x east, y up, z south.
#[test]
fn maps_opengl_axes_to_north_east_up() {
    // (local vector, north/east/up)
    let rows = [
        ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        ([0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]),
    ];
    for (local, expected) in rows {
        let sample = XPlaneSample {
            quaternion: [1.0, 0.0, 0.0, 0.0],
            local_velocity_mps: local,
            local_acceleration_mps2: local,
            ..Default::default()
        };
        let state = vehicle_state(&sample, Vec2::zeros());
        check!(state.velocity_mps == Vec3::from(expected), "{local:?}");
        check!(state.acceleration_mps2 == Vec3::from(expected), "{local:?}");
    }
}

/// Up is the elevation above sea level, not the height over the local origin's tangent
/// plane; the offset moves north and east only.
#[test]
fn places_position_by_offset_and_elevation() {
    let sample = XPlaneSample {
        quaternion: [1.0, 0.0, 0.0, 0.0],
        local_position_m: [10.0, 400.0, -20.0],
        elevation_m: 350.0,
        ..Default::default()
    };
    let state = vehicle_state(&sample, Vec2::new(-5.0, 2.0));
    check!(state.position_m == Vec3::new(15.0, 12.0, 350.0));
}

/// Body rates convert from deg/s, axis for axis, and airspeed passes through.
#[test]
fn converts_rates_and_airspeed() {
    let sample = XPlaneSample {
        quaternion: [1.0, 0.0, 0.0, 0.0],
        body_rates_degps: [90.0, -45.0, 180.0],
        true_airspeed_mps: 31.5,
        ..Default::default()
    };
    let state = vehicle_state(&sample, Vec2::zeros());
    assert_relative_eq!(
        state.angular_velocity_radps,
        Vec3::new(
            core::f32::consts::FRAC_PI_2,
            -core::f32::consts::FRAC_PI_4,
            core::f32::consts::PI
        ),
        epsilon = 1e-6
    );
    check!(state.airspeed_mps == Some(31.5));
}

/// A quaternion that is unit only to `f32` precision is normalized, and the Euler fields match
/// it: the spike's in-flight reading (`psi` 179.21°, `theta` 21.65°, `phi` 20.43°;
/// xplane-bridge `design.md`, Spike findings: Quaternion).
#[test]
fn reads_the_spike_quaternion() {
    let sample = XPlaneSample {
        quaternion: [0.040, -0.184, 0.175, 0.966],
        ..Default::default()
    };
    let state = vehicle_state(&sample, Vec2::zeros());
    check!((state.attitude.norm() - 1.0).abs() < 1e-6);
    check!((state.roll_rad.to_degrees() - 20.43).abs() < 0.5);
    check!((state.pitch_rad.to_degrees() - 21.65).abs() < 0.5);
    check!((state.heading_rad.to_degrees().rem_euclid(360.0) - 179.21).abs() < 0.5);
}
