use std::collections::HashSet;

use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    LateralControllerConfig, NotchConfig, YawControllerConfig,
};
use crate::estimation::wind::WindConfig;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig, ThrustLimits};
use crate::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
use crate::guidance::vtol::{LiftTransitionConfig, OffNoseSpeedConfig};
use crate::mixers::TableMixer;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::testing::{ConstantCommander, NullVehicle};
use crate::{Command, FlightLoop};

fn names<C: ParamTable>() -> Vec<&'static str> {
    C::PARAMS.iter().map(Param::name).collect()
}

fn all_names() -> Vec<&'static str> {
    let mut all = names::<CopterGuidanceConfig>();
    all.extend(names::<CopterStabilizerConfig>());
    all
}

/// Fails to compile when a config gains a field, so its param table gets updated with it.
#[test]
fn tables_cover_every_config_field() {
    let CopterStabilizerConfig {
        body_rate:
            BodyRateControllerConfig {
                roll_gains: Gains { p: _, i: _, d: _ },
                pitch_gains: Gains { p: _, i: _, d: _ },
                yaw_gains: Gains { p: _, i: _, d: _ },
                max_integral: _,
                yaw_max_integral: _,
            },
        attitude:
            AttitudeControllerConfig {
                roll_gains: Gains { p: _, i: _, d: _ },
                pitch_gains: Gains { p: _, i: _, d: _ },
                max_angles_rad: _,
                max_integral: _,
                max_rate_radps: _,
            },
        yaw:
            YawControllerConfig {
                gains: Gains { p: _, i: _, d: _ },
                max_integral: _,
                max_rate_radps: _,
            },
        full_scale_rates_radps: _,
        thrust_curve_exponent: _,
        gyro_cutoff_hz: _,
        gyro_notch:
            NotchConfig {
                center_hz: _,
                bandwidth_hz: _,
                attenuation_db: _,
            },
    } = CopterStabilizerConfig::default();
    let CopterGuidanceConfig {
        altitude:
            AltitudeControllerConfig {
                gains: Gains { p: _, i: _, d: _ },
                max_integral: _,
                hover_thrust: _,
            },
        lateral:
            LateralControllerConfig {
                position_p: _,
                velocity: Gains { p: _, i: _, d: _ },
                max_integral: _,
                max_accel_mps2: _,
                brake_mps2: _,
            },
        navigation_period_s: _,
        max_tilt_rad: _,
        max_stick_angles_rad: _,
        stick_yaw_rate_radps: _,
        idle_throttle: _,
        max_descent_mps: _,
        descent_accel_mps2: _,
        climb_decel_mps2: _,
        altitude_leash_m: _,
        land_speed_mps: _,
        waypoint_yaw_radps: _,
        heading_rate_radps: _,
        waypoint_yaw_min_leg_m: _,
        heading_yield_rad: _,
        heading_yield_rate_radps: _,
        heading_yield_s: _,
        wind_heading_min_mps: _,
        wind_heading_from_wind_rad: _,
        wind_heading_max_sd_rad: _,
        wind_heading_deadband_rad: _,
        airborne_height_m: _,
        feedforward_hold_band_mps: _,
        thrust:
            ThrustLimits {
                min_thrust: _,
                sink_limit_mps: _,
                sink_gain: _,
                max_boost_thrust: _,
            },
        wind:
            WindConfig {
                ballistic_kgpm2: _,
                momentum_per_s: _,
                force_noise_mps2: _,
                process_noise_mps_per_rt_s: _,
            },
    } = CopterGuidanceConfig::default();
    // one param per scalar, vector component, or gain term in the destructuring above
    check!(names::<CopterStabilizerConfig>().len() == 35);
    check!(names::<CopterGuidanceConfig>().len() == 45);
}

/// Fails to compile when the plane guidance config gains a field, so its param table gets
/// updated with it.
#[test]
fn plane_guidance_table_covers_every_config_field() {
    let PlaneGuidanceConfig {
        max_bank_rad: _,
        max_pitch_up_rad: _,
        max_pitch_down_rad: _,
        max_rate_radps: _,
        trim_pitch_rad: _,
        cruise_thrust: _,
        l1_period_s: _,
        l1_damping: _,
        loiter_radius_m: _,
        altitude_p: _,
        altitude_i: _,
        altitude_max_integral: _,
        tecs:
            TecsConfig {
                time_constant_s: _,
                max_climb_mps: _,
                min_sink_mps: _,
                max_sink_mps: _,
                thrust_damping: _,
                integrator_gain: _,
                pitch_damping: _,
                pitch_integrator_gain: _,
                speed_weight: _,
                roll_compensation: _,
                min_airspeed_mps: _,
                max_airspeed_mps: _,
                underspeed_share: _,
                height_demand_time_constant_s: _,
                airspeed_rate_time_constant_s: _,
                kinetic_rate_high_pass_s: _,
            },
        min_thrust: _,
        max_thrust: _,
        climb_thrust_per_rad: _,
    } = PlaneGuidanceConfig::default();
    // one param per scalar or vector component in the destructuring above
    check!(names::<PlaneGuidanceConfig>().len() == 33);
}

/// Fails to compile when the lift transition or off-nose config gains a field, so its param
/// table gets updated with it.
#[test]
fn quadplane_tables_cover_every_config_field() {
    let LiftTransitionConfig {
        min_airspeed_mps: _,
        lift_ramp_s: _,
        timeout_s: _,
        min_height_m: _,
        plane_min_height_m: _,
        back_ramp_s: _,
        back_speed_mps: _,
        back_timeout_s: _,
        back_decel_mps2: _,
        back_decel_i: _,
        forward_thrust: _,
        pusher_slew_per_s: _,
        handoff_s: _,
        roll_limit_rad: _,
        forward_pitch_max_rad: _,
    } = LiftTransitionConfig::default();
    let OffNoseSpeedConfig {
        speed_mps: _,
        free_rad: _,
        full_rad: _,
        tail_first_divisor: _,
    } = OffNoseSpeedConfig::default();
    // one param per field above
    check!(names::<LiftTransitionConfig>().len() == 15);
    check!(names::<OffNoseSpeedConfig>().len() == 4);
}

#[test]
fn names_fit_mavlink_and_are_unique() {
    let all = all_names();
    for name in &all {
        check!(name.len() <= MAX_NAME_LEN, "{name}");
        check!(
            name.bytes()
                .all(|b| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_'),
            "{name}"
        );
    }
    let unique: HashSet<_> = all.iter().collect();
    check!(unique.len() == all.len());
    for name in names::<CopterStabilizerConfig>() {
        check!(name.starts_with("STAB_"), "{name}");
    }
    for name in names::<CopterGuidanceConfig>() {
        check!(name.starts_with("GUID_"), "{name}");
    }
}

/// Each param reads back what it wrote and touches no other param: a typo mapping two names
/// to one field would fail here.
fn check_isolated_round_trips<C: ParamTable + Default>() {
    for (k, param) in C::PARAMS.iter().enumerate() {
        let mut config = C::default();
        let before: Vec<f32> = C::PARAMS.iter().map(|p| p.get(&config)).collect();
        let sentinel = before[k] + 0.375;
        param.set(&mut config, sentinel);
        for (j, other) in C::PARAMS.iter().enumerate() {
            let expected = if j == k { sentinel } else { before[j] };
            assert_relative_eq!(other.get(&config), expected, max_relative = 1e-5);
        }
    }
}

#[test]
fn each_param_round_trips_without_touching_the_others() {
    check_isolated_round_trips::<CopterStabilizerConfig>();
    check_isolated_round_trips::<CopterGuidanceConfig>();
}

#[test]
fn angles_and_rates_are_degrees_stored_as_radians() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let mut guidance = CopterGuidance::new(CopterGuidanceConfig::default());

    stabilizer.set_param("STAB_ATT_AMAX_Y", 30.0).unwrap();
    stabilizer.set_param("STAB_FS_RATE_Z", 180.0).unwrap();
    guidance.set_param("GUID_TILT_MAX", 20.0).unwrap();

    let (s, g) = (stabilizer.config(), guidance.config());
    assert_relative_eq!(s.attitude.max_angles_rad.y, 30f32.to_radians());
    assert_relative_eq!(s.full_scale_rates_radps.z, core::f32::consts::PI);
    assert_relative_eq!(g.max_tilt_rad, 20f32.to_radians());
    assert_relative_eq!(
        stabilizer.param("STAB_ATT_AMAX_Y").unwrap(),
        30.0,
        max_relative = 1e-6
    );
}

#[test]
fn rejected_values_leave_the_config_unchanged() {
    // (name, value, error)
    let body_rate_max_integral = ParamError::Invalid(InvalidConfig {
        section: Some("body_rate"),
        field: "max_integral",
        requirement: "in [0, 1]",
    });
    let body_rate_yaw_max_integral = ParamError::Invalid(InvalidConfig {
        section: Some("body_rate"),
        field: "yaw_max_integral",
        requirement: "in [0, 1]",
    });
    let cases = [
        ("NO_SUCH_PARAM", 1.0, ParamError::Unknown),
        ("GUID_ALT_P", 1.0, ParamError::Unknown), // the guidance's, not the stabilizer's
        ("STAB_RATE_RLL_P", f32::NAN, ParamError::NotFinite),
        ("STAB_RATE_RLL_P", f32::INFINITY, ParamError::NotFinite),
        ("STAB_RATE_IMAX", -1.0, body_rate_max_integral),
        ("STAB_RATE_IMAX_Z", -1.0, body_rate_yaw_max_integral),
        // A value saved in the old unit, rad of error.
        ("STAB_RATE_IMAX_Z", 5.0, body_rate_yaw_max_integral),
    ];
    for (name, value, error) in cases {
        let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
        check!(
            stabilizer.set_param(name, value) == Err(error),
            "{name} = {value}"
        );
        check!(*stabilizer.config() == CopterStabilizerConfig::default());
    }
}

#[test]
fn visit_lists_every_param_in_table_order() {
    let guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    let mut visited = Vec::new();
    guidance.visit_params(&mut |name, value| visited.push((name, value)));

    let config = CopterGuidanceConfig::default();
    let expected: Vec<_> = CopterGuidanceConfig::PARAMS
        .iter()
        .map(|p| (p.name(), p.get(&config)))
        .collect();
    check!(visited == expected);
}

#[test]
fn unknown_name_reads_as_none() {
    let guidance = CopterGuidance::new(CopterGuidanceConfig::default());
    check!(guidance.param("STAB_RATE_RLL_P") == None);
    check!(guidance.param("GUID_ALT_P") == Some(guidance.config().altitude.gains.p));
}

fn flight_loop()
-> FlightLoop<ConstantCommander, NullVehicle, CopterGuidance, CopterStabilizer, TableMixer<4>, ()> {
    FlightLoop::new(
        ConstantCommander::new(Command::default()),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        TableMixer::quad_x(),
        (),
    )
}

#[test]
fn flight_loop_routes_each_name_to_its_component() {
    let mut flight_loop = flight_loop();

    flight_loop.set_param("GUID_LAT_POS_P", 0.7).unwrap();
    flight_loop.set_param("STAB_YAW_P", 3.5).unwrap();

    check!(flight_loop.guidance().config().lateral.position_p == 0.7);
    check!(flight_loop.stabilizer().config().yaw.gains.p == 3.5);
    check!(flight_loop.param("GUID_LAT_POS_P") == Some(0.7));
    check!(flight_loop.param("STAB_YAW_P") == Some(3.5));
    check!(flight_loop.param("NO_SUCH_PARAM") == None);
    check!(flight_loop.set_param("NO_SUCH_PARAM", 1.0) == Err(ParamError::Unknown));
}

#[test]
fn flight_loop_visits_guidance_then_stabilizer() {
    let flight_loop = flight_loop();
    let mut visited = Vec::new();
    flight_loop.visit_params(&mut |name, _| visited.push(name));
    check!(visited == all_names());
}

#[test]
fn errors_describe_the_rejection() {
    let invalid = ParamError::Invalid(InvalidConfig {
        section: Some("yaw"),
        field: "max_rate_radps",
        requirement: "non-negative and finite",
    });
    check!(ParamError::Unknown.to_string() == "unknown param");
    check!(ParamError::NotFinite.to_string() == "param value must be finite");
    check!(
        invalid.to_string()
            == "invalid param value: yaw.max_rate_radps must be non-negative and finite"
    );
}

#[test]
fn indices_follow_table_order_and_reject_out_of_range() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let names = names::<CopterStabilizerConfig>();
    check!(stabilizer.param_count() == names.len());
    for (i, name) in names.iter().enumerate() {
        check!(stabilizer.param_name(i) == Some(*name));
        check!(stabilizer.param_index(name) == Some(i));
        check!(stabilizer.param_at(i) == stabilizer.param(name));
    }

    let end = names.len();
    check!(stabilizer.param_name(end) == None);
    check!(stabilizer.param_at(end) == None);
    check!(stabilizer.set_param_at(end, 1.0) == Err(ParamError::Unknown));
    check!(stabilizer.param_index("NO_SUCH_PARAM") == None);
}

#[test]
fn set_by_index_writes_the_indexed_param() {
    let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
    let i = stabilizer.param_index("STAB_YAW_D").unwrap();
    stabilizer.set_param_at(i, 0.25).unwrap();
    check!(stabilizer.config().yaw.gains.d == 0.25);
}

/// The `Params` contract: a successful write stores the value as given (up to deg ↔ rad
/// rounding), so a link can confirm it by echoing what it sent.
#[test]
fn successful_writes_read_back_the_value_given() {
    let mut flight_loop = flight_loop();
    for i in 0..flight_loop.param_count() {
        let name = flight_loop.param_name(i).unwrap();
        // 90% of each default is in range: no default sits at a lower bound
        let value = flight_loop.param_at(i).unwrap() * 0.9;
        flight_loop
            .set_param_at(i, value)
            .unwrap_or_else(|e| panic!("{name} = {value}: {e}"));
        assert_relative_eq!(flight_loop.param_at(i).unwrap(), value, max_relative = 1e-6);
        check!(flight_loop.param(name) == flight_loop.param_at(i));
    }
}

#[test]
fn flight_loop_indexes_guidance_then_stabilizer() {
    let mut flight_loop = flight_loop();
    let guidance_count = names::<CopterGuidanceConfig>().len();
    check!(flight_loop.param_count() == all_names().len());
    check!(flight_loop.param_name(guidance_count - 1) == Some("GUID_WND_P_NSE"));
    check!(flight_loop.param_name(guidance_count) == Some("STAB_RATE_RLL_P"));
    check!(flight_loop.param_index("STAB_RATE_RLL_P") == Some(guidance_count));
    check!(flight_loop.param_name(all_names().len()) == None);

    flight_loop.set_param_at(guidance_count, 0.3).unwrap();
    check!(flight_loop.stabilizer().config().body_rate.roll_gains.p == 0.3);
    check!(flight_loop.set_param_at(all_names().len(), 1.0) == Err(ParamError::Unknown));
}
