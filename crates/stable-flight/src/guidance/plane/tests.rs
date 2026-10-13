use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::{
    AbsoluteAttitudeCommand, AbsoluteRateCommand, AttitudeCommand, FlightMode, LandCommand,
    ModeRefusal, RateCommand, Vec2, WaypointCommand,
};

fn guide(input: ControlInput) -> Setpoint {
    PlaneGuidance::new(PlaneGuidanceConfig::default()).guide(
        0.01,
        FlightRegime::Plane,
        &input,
        &VehicleState::default(),
    )
}

fn sticks(roll: f32, pitch: f32, throttle: f32) -> ControlInput {
    ControlInput::Attitude(AttitudeCommand {
        throttle,
        roll,
        pitch,
        yaw: 1.0, // unused: coordinated turns
    })
}

/// FBWA: (roll, pitch, throttle sticks) → (bank°, pitch°, thrust).
#[test]
fn sticks_map_to_bank_and_asymmetric_pitch() {
    let cases = [
        ((0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
        ((1.0, 0.0, 1.0), (45.0, 0.0, 1.0)),
        ((-0.5, 0.0, 0.2), (-22.5, 0.0, 0.2)),
        ((0.0, 1.0, 0.5), (0.0, 20.0, 0.5)),
        ((0.0, -1.0, 0.5), (0.0, -25.0, 0.5)),
        ((2.0, -3.0, 1.5), (45.0, -25.0, 1.0)), // saturated
        ((f32::NAN, f32::NAN, f32::NAN), (0.0, 0.0, 0.0)),
    ];
    for ((roll, pitch, throttle), (bank, pitch_deg, thrust)) in cases {
        let Setpoint::Plane(setpoint) = guide(sticks(roll, pitch, throttle)) else {
            panic!("sticks give a plane attitude");
        };
        assert_relative_eq!(setpoint.roll_rad.to_degrees(), bank, epsilon = 1e-4);
        assert_relative_eq!(setpoint.pitch_rad.to_degrees(), pitch_deg, epsilon = 1e-4);
        check!(setpoint.thrust == thrust);
    }
}

/// Rate sticks fly acro at stick × max rate; absolute rates pass through.
#[test]
fn rate_inputs_give_plane_rates() {
    let Setpoint::PlaneRate(acro) = guide(ControlInput::Rate(RateCommand {
        throttle: 0.6,
        roll_rate: 1.0,
        pitch_rate: -0.5,
        yaw_rate: 1.0,
    })) else {
        panic!("rate sticks give plane rates");
    };
    assert_relative_eq!(acro.roll_radps, 180f32.to_radians());
    assert_relative_eq!(acro.pitch_radps, -90f32.to_radians());
    check!(acro.yaw_radps == 0.0); // yaw rate 0 by default
    check!(acro.thrust == 0.6);

    let command = AbsoluteRateCommand {
        roll_radps: 7.0,
        pitch_radps: -1.0,
        yaw_radps: 0.2,
        thrust: 1.4,
    };
    let Setpoint::PlaneRate(rates) = guide(ControlInput::AbsoluteRate(command)) else {
        panic!("absolute rates give plane rates");
    };
    check!((rates.roll_radps, rates.pitch_radps, rates.yaw_radps) == (7.0, -1.0, 0.2));
    check!(rates.thrust == 1.0);
}

/// An absolute attitude passes through (heading unused); a waypoint at the vehicle (at rest,
/// on its altitude, no speed target) holds trim pitch and cruise thrust, wings level; a land
/// is neutral.
#[test]
fn offboard_waypoint_and_land_inputs() {
    let config = PlaneGuidanceConfig {
        trim_pitch_rad: 0.05,
        cruise_thrust: 0.55,
        ..Default::default()
    };
    let mut guidance = PlaneGuidance::new(config);
    let state = VehicleState::default();
    let mut fly = |input| guidance.guide(0.01, FlightRegime::Plane, &input, &state);

    let absolute = fly(ControlInput::AbsoluteAttitude(AbsoluteAttitudeCommand {
        roll_rad: 0.4,
        pitch_rad: 0.8,
        heading_rad: 2.0,
        thrust: 0.3,
    }));
    check!(
        absolute
            == Setpoint::Plane(PlaneSetpoint {
                roll_rad: 0.4,
                pitch_rad: 0.8,
                thrust: 0.3
            })
    );

    let cruise = fly(ControlInput::Waypoint(WaypointCommand::default()));
    check!(
        cruise
            == Setpoint::Plane(PlaneSetpoint {
                roll_rad: 0.0,
                pitch_rad: 0.05,
                thrust: 0.55
            })
    );

    let land = fly(ControlInput::Land(LandCommand::default()));
    check!(land == Setpoint::Plane(PlaneSetpoint::default()));
}

/// Plane only: a `Copter` request is reported unsupported.
#[test]
fn reports_plane_only() {
    let mut guidance = PlaneGuidance::new(PlaneGuidanceConfig::default());
    let state = VehicleState::default();
    for (regime, refusal) in [
        (FlightRegime::Plane, None),
        (FlightRegime::Copter, Some(ModeRefusal::Unsupported)),
    ] {
        guidance.guide(0.01, regime, &sticks(0.0, 0.0, 0.5), &state);
        check!(
            guidance.flight_mode()
                == Some(FlightModeReport {
                    mode: FlightMode::Plane,
                    refusal
                })
        );
    }
}

#[test]
fn validates_config() {
    type Break = fn(&mut PlaneGuidanceConfig);
    let cases: [(Break, &str); 5] = [
        (|c| c.max_bank_rad = 2.0, "max_bank_rad"),
        (|c| c.max_pitch_down_rad = -0.1, "max_pitch_down_rad"),
        (|c| c.max_rate_radps.y = f32::NAN, "max_rate_radps.y"),
        (|c| c.trim_pitch_rad = 2.0, "trim_pitch_rad"),
        (|c| c.cruise_thrust = 1.1, "cruise_thrust"),
    ];
    check!(PlaneGuidanceConfig::default().validate().is_ok());
    for (corrupt, field) in cases {
        let mut config = PlaneGuidanceConfig::default();
        corrupt(&mut config);
        check!(config.validate().map_err(|e| e.field) == Err(field));
        let mut guidance = PlaneGuidance::new(PlaneGuidanceConfig::default());
        check!(guidance.set_config(config).is_err());
    }
}

/// Waypoint flight, from a vehicle flying north at 15 m/s airspeed at 30 m: the setpoint's
/// bank, pitch, and thrust against the waypoint. (waypoint, airspeed, expected signs of
/// (bank, pitch − trim, thrust − cruise))
#[test]
fn waypoint_flight_steers_climbs_and_holds_speed() {
    let along_north = |east: f32, altitude: f32, speed: f32| WaypointCommand {
        position_m: Vec2::new(200.0, east),
        velocity_mps: Some(Vec2::new(14.0, 0.0)),
        altitude_m: altitude,
        cruise_speed_mps: speed,
    };
    let sign = |v: f32| if v.abs() < 1e-4 { 0 } else { v.signum() as i32 };
    let cases = [
        // On the line, at altitude, at speed: nothing to do.
        (along_north(0.0, 30.0, 15.0), Some(15.0), (0, 0, 0)),
        // The line is east (right): bank right.
        (along_north(20.0, 30.0, 15.0), Some(15.0), (1, 0, 0)),
        (along_north(-20.0, 30.0, 15.0), Some(15.0), (-1, 0, 0)),
        // Above the vehicle: pitch up; below: down.
        (along_north(0.0, 40.0, 15.0), Some(15.0), (0, 1, 0)),
        (along_north(0.0, 20.0, 15.0), Some(15.0), (0, -1, 0)),
        // Slow: more thrust; fast: less.
        (along_north(0.0, 30.0, 18.0), Some(15.0), (0, 0, 1)),
        (along_north(0.0, 30.0, 12.0), Some(15.0), (0, 0, -1)),
        // No airspeed: cruise, plus a climb's share when climbing.
        (along_north(0.0, 30.0, 15.0), None, (0, 0, 0)),
        (along_north(0.0, 40.0, 15.0), None, (0, 1, 1)),
    ];
    let config = PlaneGuidanceConfig::default();
    for (waypoint, airspeed_mps, (bank, pitch, thrust)) in cases {
        let state = VehicleState {
            position_m: crate::Vec3::new(0.0, 0.0, 30.0),
            velocity_mps: crate::Vec3::new(15.0, 0.0, 0.0),
            airspeed_mps,
            ..Default::default()
        };
        let mut guidance = PlaneGuidance::new(config);
        let Setpoint::Plane(setpoint) = guidance.guide(
            0.01,
            FlightRegime::Plane,
            &ControlInput::Waypoint(waypoint),
            &state,
        ) else {
            panic!("waypoints give plane attitudes");
        };
        check!(
            (
                sign(setpoint.roll_rad),
                sign(setpoint.pitch_rad - config.trim_pitch_rad),
                sign(setpoint.thrust - config.cruise_thrust),
            ) == (bank, pitch, thrust),
            "{waypoint:?} {airspeed_mps:?}: {setpoint:?}"
        );
        check!(setpoint.roll_rad.abs() <= config.max_bank_rad);
    }
}

/// A waypoint with no leg is a loiter: on its circle, flying the clockwise tangent, the bank
/// is the circle's (atan(V²/(r·g))).
#[test]
fn waypoint_without_a_leg_loiters() {
    let config = PlaneGuidanceConfig::default();
    let (radius, speed) = (config.loiter_radius_m, 15.0_f32);
    let state = VehicleState {
        // North of the center, flying east: clockwise.
        position_m: crate::Vec3::new(radius, 0.0, 30.0),
        velocity_mps: crate::Vec3::new(0.0, speed, 0.0),
        airspeed_mps: Some(speed),
        ..Default::default()
    };
    let point = WaypointCommand {
        position_m: Vec2::zeros(),
        altitude_m: 30.0,
        cruise_speed_mps: speed,
        ..Default::default()
    };
    let Setpoint::Plane(setpoint) = PlaneGuidance::new(config).guide(
        0.01,
        FlightRegime::Plane,
        &ControlInput::Waypoint(point),
        &state,
    ) else {
        panic!("waypoints give plane attitudes");
    };
    let expected = (speed * speed / (radius * crate::GRAVITY)).atan();
    assert_relative_eq!(setpoint.roll_rad, expected, epsilon = 1e-4);
}

/// The navigation integrals build in waypoint flight and clear on `prime` or on any other
/// input.
#[test]
fn navigation_integrals_clear_on_prime_and_other_inputs() {
    let config = PlaneGuidanceConfig::default();
    let state = VehicleState {
        position_m: crate::Vec3::new(0.0, 0.0, 20.0),
        velocity_mps: crate::Vec3::new(15.0, 0.0, 0.0),
        airspeed_mps: Some(15.0),
        ..Default::default()
    };
    let climb = ControlInput::Waypoint(WaypointCommand {
        position_m: Vec2::new(500.0, 0.0),
        velocity_mps: Some(Vec2::new(14.0, 0.0)),
        altitude_m: 21.0,
        cruise_speed_mps: 15.0,
    });
    let pitch = |guidance: &mut PlaneGuidance| match guidance.guide(
        0.1,
        FlightRegime::Plane,
        &climb,
        &state,
    ) {
        Setpoint::Plane(setpoint) => setpoint.pitch_rad,
        other => panic!("{other:?}"),
    };
    type Reset = fn(&mut PlaneGuidance, &VehicleState);
    let resets: [(&str, Reset); 2] = [
        ("prime", |g, s| g.prime(s)),
        ("sticks", |g, s| {
            g.guide(0.1, FlightRegime::Plane, &sticks(0.0, 0.0, 0.5), s);
        }),
    ];
    for (name, reset) in resets {
        let mut guidance = PlaneGuidance::new(config);
        let first = pitch(&mut guidance);
        for _ in 0..50 {
            pitch(&mut guidance);
        }
        check!(pitch(&mut guidance) > first, "{name}: the integral builds");
        reset(&mut guidance, &state);
        assert_relative_eq!(pitch(&mut guidance), first, epsilon = 1e-6);
    }
}
