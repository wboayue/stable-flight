use assert2::check;

use super::*;
use crate::geo::{EARTH_RADIUS_M, GeodeticOrigin};

#[test]
fn maps_normalized_thrust_to_throttle_percent() {
    // (thrust, expected percent)
    let cases = [
        (0.0, 0),
        (0.5, 50),
        (0.734, 73),
        (1.0, 100),
        (-0.2, 0),
        (1.7, 100),
        (f32::NAN, 0),
    ];

    for (thrust, expected) in cases {
        check!(throttle_percent(thrust) == expected, "thrust {thrust}");
    }
}

fn state_with(position_m: [f32; 3], velocity_mps: [f32; 3]) -> VehicleState {
    VehicleState {
        position_m: position_m.into(),
        velocity_mps: velocity_mps.into(),
        ..Default::default()
    }
}

#[test]
fn local_position_is_ned_down_positive() {
    // (position_m, velocity_mps (north, east, up), expected (z, vz))
    let cases = [
        ([3.0, 4.0, 100.0], [1.0, 2.0, 5.0], (-100.0, -5.0)), // climbing at 100 m
        ([0.0, 0.0, -2.0], [0.0, 0.0, -1.5], (2.0, 1.5)),     // below datum, descending
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], (0.0, 0.0)),
    ];

    for (position_m, velocity_mps, (z, vz)) in cases {
        let data = local_position_ned(&state_with(position_m, velocity_mps), 7);
        check!(data.time_boot_ms == 7);
        check!(
            (data.x, data.y) == (position_m[0], position_m[1]),
            "north/east unchanged"
        );
        check!((data.vx, data.vy) == (velocity_mps[0], velocity_mps[1]));
        check!(data.z == z, "{position_m:?}");
        check!(data.vz == vz, "{velocity_mps:?}");
    }
}

#[test]
fn attitude_reports_euler_angles_with_yaw_in_minus_pi_to_pi() {
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_6, PI};

    // (roll, pitch, heading, expected yaw)
    let cases = [
        (FRAC_PI_6, 0.0, 0.0, 0.0),              // 30° right roll
        (0.0, 0.2, FRAC_PI_2, FRAC_PI_2),        // nose up, facing east
        (0.0, 0.0, 3.0 * FRAC_PI_2, -FRAC_PI_2), // west, from a [0, 2π) backend
        (0.0, 0.0, -FRAC_PI_2, -FRAC_PI_2),      // west, from a [-π, π) backend
        (0.0, 0.0, PI, -PI),                     // south wraps to -π
        (0.0, 0.0, 5.0 * FRAC_PI_2, FRAC_PI_2),  // east, after a full turn
    ];

    for (roll, pitch, heading, yaw) in cases {
        let state = VehicleState {
            roll_rad: roll,
            pitch_rad: pitch,
            heading_rad: heading,
            angular_velocity_radps: [0.1, -0.2, 0.3].into(),
            ..Default::default()
        };
        let data = attitude(&state, 0);
        check!((data.roll, data.pitch) == (roll, pitch));
        approx::assert_relative_eq!(data.yaw, yaw, epsilon = 1e-5);
        check!((data.rollspeed, data.pitchspeed, data.yawspeed) == (0.1, -0.2, 0.3));
    }
}

#[test]
fn vfr_hud_heading_is_whole_degrees_from_north() {
    use core::f32::consts::FRAC_PI_2;

    // (heading_rad, expected degrees)
    let cases = [
        (0.0, 0),
        (FRAC_PI_2, 90),
        (-FRAC_PI_2, 270),
        (359.6_f32.to_radians(), 0), // rounds up to 360, wraps
        (359.4_f32.to_radians(), 359),
        (-1e-7, 0),
        (f32::NAN, 0),
    ];

    for (heading_rad, expected) in cases {
        check!(
            heading_deg(heading_rad) == expected,
            "heading {heading_rad}"
        );
    }
}

#[test]
fn vfr_hud_reports_horizontal_speed_climb_and_altitude() {
    // (velocity_mps (north, east, up), altitude_m, expected speed)
    let cases = [
        ([3.0, 4.0, 2.0], 120.0, 5.0),
        ([-6.0, 8.0, -1.0], 10.0, 10.0),
        ([0.0, 0.0, 3.0], 0.0, 0.0), // pure climb: no ground speed
    ];

    for (velocity_mps, altitude_m, speed) in cases {
        let state = state_with([0.0, 0.0, altitude_m], velocity_mps);
        let data = vfr_hud(&state, 0.5);
        check!(data.groundspeed == speed, "{velocity_mps:?}");
        check!(data.airspeed == speed);
        check!(data.climb == velocity_mps[2]);
        check!(data.alt == altitude_m);
        check!(data.throttle == 50);
    }
}

#[test]
fn vfr_hud_airspeed_falls_back_to_ground_speed_without_air_data() {
    // (airspeed_mps, expected airspeed); ground speed is 5 m/s
    let cases = [
        (Some(7.0), 7.0),
        (Some(0.0), 0.0),
        (None, 5.0),
        (Some(f32::NAN), 5.0),
        (Some(-1.0), 5.0),
    ];

    for (airspeed_mps, expected) in cases {
        let state = VehicleState {
            airspeed_mps,
            ..state_with([0.0, 0.0, 0.0], [3.0, 4.0, 0.0])
        };
        check!(
            vfr_hud(&state, 0.5).airspeed == expected,
            "{airspeed_mps:?}"
        );
    }
}

const ORIGIN: GeodeticOrigin = GeodeticOrigin {
    latitude_deg: 60.0, // cos = 0.5: a degree of longitude is half a degree of latitude
    longitude_deg: -120.0,
    altitude_m: 100.0,
};

fn earth() -> FlatEarth {
    FlatEarth::new(ORIGIN).expect("valid origin")
}

#[test]
fn global_position_projects_local_offsets_around_the_origin() {
    let m_per_deg = EARTH_RADIUS_M.to_radians() as f32;
    // (position_m (north, east, up), expected (lat_deg, lon_deg, alt_m))
    let cases = [
        ([0.0, 0.0, 0.0], (60.0, -120.0, 100.0)),
        ([m_per_deg / 1000.0, 0.0, 0.0], (60.001, -120.0, 100.0)), // north
        ([-m_per_deg / 1000.0, 0.0, 0.0], (59.999, -120.0, 100.0)), // south
        ([0.0, m_per_deg / 1000.0, 0.0], (60.0, -119.998, 100.0)), // east: twice the degrees
        ([0.0, 0.0, 25.5], (60.0, -120.0, 125.5)),                 // climb
    ];

    for (position_m, (lat_deg, lon_deg, alt_m)) in cases {
        let data = global_position_int(&earth(), &state_with(position_m, [0.0; 3]), 0.0, 7)
            .expect("finite position");
        check!(data.time_boot_ms == 7);
        check!(
            data.lat == (lat_deg * DEG_E7).round() as i32,
            "{position_m:?}"
        );
        check!(
            data.lon == (lon_deg * DEG_E7).round() as i32,
            "{position_m:?}"
        );
        check!(data.alt == (alt_m * MM_PER_M) as i32, "{position_m:?}");
    }
}

#[test]
fn global_position_reports_altitude_above_home_and_ned_velocity_in_cm() {
    // (altitude_m, home_altitude_m, velocity_mps (north, east, up), expected relative_alt mm, (vx, vy, vz))
    let cases = [
        (12.5, 2.0, [1.0, -2.5, 3.0], 10_500, (100, -250, -300)), // climbing above home
        (1.0, 2.0, [0.0, 0.0, -0.5], -1_000, (0, 0, 50)),         // below home, descending
        (2.0, 2.0, [f32::NAN, 400.0, 0.0], 0, (0, i16::MAX, 0)),  // NaN and overflow saturate
    ];

    for (altitude_m, home_altitude_m, velocity_mps, relative_alt, velocity_cmps) in cases {
        let data = global_position_int(
            &earth(),
            &state_with([0.0, 0.0, altitude_m], velocity_mps),
            home_altitude_m,
            0,
        )
        .expect("finite position");
        check!(
            data.relative_alt == relative_alt,
            "{altitude_m} over {home_altitude_m}"
        );
        check!(
            (data.vx, data.vy, data.vz) == velocity_cmps,
            "{velocity_mps:?}"
        );
    }
}

#[test]
fn global_position_heading_is_centidegrees_from_north() {
    use core::f32::consts::FRAC_PI_2;

    // (heading_rad, expected centidegrees)
    let cases = [
        (0.0, 0),
        (FRAC_PI_2, 9000),
        (-FRAC_PI_2, 27000),
        (359.996_f32.to_radians(), 0), // rounds up to 36000, wraps
        (359.99_f32.to_radians(), 35999),
        (f32::NAN, u16::MAX), // unknown
        (f32::INFINITY, u16::MAX),
    ];

    for (heading_rad, expected) in cases {
        check!(
            heading_cdeg(heading_rad) == expected,
            "heading {heading_rad}"
        );
    }
}

#[test]
fn global_position_is_skipped_for_a_non_finite_position() {
    let cases = [
        [f32::NAN, 0.0, 0.0],
        [0.0, f32::INFINITY, 0.0],
        [0.0, 0.0, f32::NAN],
    ];

    for position_m in cases {
        let state = state_with(position_m, [0.0; 3]);
        check!(
            global_position_int(&earth(), &state, 0.0, 0).is_none(),
            "{position_m:?}"
        );
    }
}

#[test]
fn home_position_is_global_and_local_ned_with_unknown_orientation() {
    let m_per_deg = EARTH_RADIUS_M.to_radians() as f32;
    // (home_m (north, east, up), expected (lat_deg, lon_deg, alt_m))
    let cases = [
        ([0.0, 0.0, 0.0], (60.0, -120.0, 100.0)),
        ([m_per_deg / 1000.0, 0.0, 12.5], (60.001, -120.0, 112.5)),
        ([0.0, -m_per_deg / 1000.0, -3.0], (60.0, -120.002, 97.0)), // west: twice the degrees
    ];

    for (home_m, (lat_deg, lon_deg, alt_m)) in cases {
        let data = home_position(&earth(), home_m.into());
        check!(
            data.latitude == (lat_deg * DEG_E7).round() as i32,
            "{home_m:?}"
        );
        check!(
            data.longitude == (lon_deg * DEG_E7).round() as i32,
            "{home_m:?}"
        );
        check!(data.altitude == (alt_m * MM_PER_M) as i32, "{home_m:?}");
        let [north, east, up] = home_m;
        check!(
            [data.x, data.y, data.z] == [north, east, -up],
            "NED, down positive"
        );
        check!(
            [data.approach_x, data.approach_y, data.approach_z] == [data.x, data.y, data.z],
            "approach ends at home"
        );
        check!(data.q.iter().all(|q| q.is_nan()), "orientation unknown");
    }
}

#[test]
fn extended_sys_state_reports_the_landed_and_vtol_states() {
    use MavLandedState as L;
    use MavVtolState as V;

    for landed_state in [
        L::MAV_LANDED_STATE_UNDEFINED,
        L::MAV_LANDED_STATE_ON_GROUND,
        L::MAV_LANDED_STATE_IN_AIR,
    ] {
        for vtol_state in [V::MAV_VTOL_STATE_UNDEFINED, V::MAV_VTOL_STATE_FW] {
            let data = extended_sys_state(landed_state, vtol_state);
            check!(data.landed_state == landed_state);
            check!(data.vtol_state == vtol_state);
        }
    }
}

#[test]
fn each_flight_mode_maps_to_its_vtol_state() {
    use MavVtolState as V;

    let rows = [
        (FlightMode::Copter, V::MAV_VTOL_STATE_MC),
        (FlightMode::Plane, V::MAV_VTOL_STATE_FW),
        (FlightMode::ToPlane, V::MAV_VTOL_STATE_TRANSITION_TO_FW),
        (FlightMode::ToCopter, V::MAV_VTOL_STATE_TRANSITION_TO_MC),
    ];
    for (mode, expected) in rows {
        check!(vtol_state(mode) == expected, "{mode:?}");
    }
}
