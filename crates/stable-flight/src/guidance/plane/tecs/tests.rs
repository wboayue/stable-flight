use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::Vec3;

const DT: f32 = 0.01;

fn limits() -> Limits {
    Limits {
        pitch_up_rad: 20f32.to_radians(),
        pitch_down_rad: 25f32.to_radians(),
        trim_pitch_rad: 2f32.to_radians(),
        min_thrust: 0.2,
        max_thrust: 1.0,
        cruise_thrust: 0.5,
    }
}

/// Level at `altitude_m`, flying north at `airspeed_mps` in still air, banked `roll_rad`.
fn level(altitude_m: f32, airspeed_mps: f32, roll_rad: f32) -> VehicleState {
    VehicleState {
        position_m: Vec3::new(0.0, 0.0, altitude_m),
        velocity_mps: Vec3::new(airspeed_mps, 0.0, 0.0),
        roll_rad,
        airspeed_mps: Some(airspeed_mps),
        ..Default::default()
    }
}

/// One step from a fresh state: (pitch, thrust).
fn first_step(target: (f32, f32), state: &VehicleState) -> (f32, f32) {
    let airspeed = state.airspeed_mps.unwrap_or(f32::NAN);
    Tecs::default().step(
        DT,
        &TecsConfig::default(),
        &limits(),
        target,
        airspeed,
        state,
    )
}

/// On the target's height and airspeed, level and unaccelerated: trim pitch and cruise thrust.
#[test]
fn on_target_flies_trim_and_cruise() {
    let (pitch, thrust) = first_step((30.0, 15.0), &level(30.0, 15.0, 0.0));
    assert_relative_eq!(pitch, limits().trim_pitch_rad, epsilon = 1e-6);
    assert_relative_eq!(thrust, limits().cruise_thrust, epsilon = 1e-6);
}

/// Each error moves pitch and thrust the energy way: a height error is total energy (thrust)
/// with pitch toward it; an airspeed error is total energy with pitch away from it (trading
/// height for speed). (target altitude, target airspeed) → signs of (pitch − trim, thrust −
/// cruise), from level at 30 m and 15 m/s.
#[test]
fn errors_move_pitch_and_thrust_by_energy() {
    let sign = |v: f32| if v.abs() < 1e-5 { 0 } else { v.signum() as i32 };
    let cases = [
        ((40.0, 15.0), (1, 1)),
        ((20.0, 15.0), (-1, -1)),
        ((30.0, 18.0), (-1, 1)),
        ((30.0, 12.0), (1, -1)),
    ];
    let limits = limits();
    for (target, expected) in cases {
        let (pitch, thrust) = first_step(target, &level(30.0, 15.0, 0.0));
        check!(
            (
                sign(pitch - limits.trim_pitch_rad),
                sign(thrust - limits.cruise_thrust)
            ) == expected,
            "{target:?}: {pitch}, {thrust}"
        );
    }
}

/// The height demand slews toward the target at the climb and sink limits (the flown demand
/// lags it, between the start and the slew), and the airspeed demand at half the climb's
/// energy rate over V: (target altitude, target airspeed) → (slewed height, airspeed demand)
/// after one second, from 30 m at 15 m/s.
#[test]
fn demands_slew_at_their_limits() {
    let config = TecsConfig::default();
    let accel = 0.5 * GRAVITY * config.max_climb_mps / 15.0;
    let cases = [
        ((100.0, 15.0), (30.0 + config.max_climb_mps, 15.0)),
        ((0.0, 15.0), (30.0 - config.max_sink_mps, 15.0)),
        ((31.0, 15.0), (31.0, 15.0)),
        ((30.0, 25.0), (30.0, 15.0 + accel)),
        // Clamped to the most airspeed first.
        ((30.0, 40.0), (30.0, 15.0 + accel)),
    ];
    let state = level(30.0, 15.0, 0.0);
    for (target, (height, airspeed)) in cases {
        let mut tecs = Tecs::default();
        for _ in 0..100 {
            tecs.step(DT, &config, &limits(), target, 15.0, &state);
        }
        let demand = tecs.demand.expect("seeded");
        assert_relative_eq!(demand.slewed_height_m, height, epsilon = 1e-3);
        assert_relative_eq!(demand.airspeed_mps, airspeed, epsilon = 1e-3);
        let (low, high) = (height.min(30.0), height.max(30.0));
        check!((low..=high).contains(&demand.height_m), "{demand:?}");
    }
}

/// The height demand starts moving at the aircraft's climb rate, within the climb and sink
/// limits: entering plane flight sinking, it sinks along rather than pitching up to stop the
/// sink (#503). Climb rate (m/s) → the first step's demand rate (m/s), holding the height it
/// starts at.
#[test]
fn height_demand_starts_at_the_climb_rate() {
    let config = TecsConfig::default();
    let cases = [
        (0.0, 0.0),
        (-3.0, -3.0),
        (2.0, 2.0),
        // Past the sink and climb limits (5 m/s each).
        (-8.0, -config.max_sink_mps),
        (7.0, config.max_climb_mps),
    ];
    for (climb_mps, rate_mps) in cases {
        let mut state = level(300.0, 15.0, 0.0);
        state.velocity_mps.z = climb_mps;
        let mut tecs = Tecs::default();
        tecs.step(DT, &config, &limits(), (300.0, 15.0), 15.0, &state);
        let demand = tecs.demand.expect("seeded");
        let demand_rate_mps = (demand.height_m - 300.0) / DT;
        check!(
            (demand_rate_mps - rate_mps).abs() < 0.05,
            "{climb_mps} m/s: {demand_rate_mps}"
        );
    }
}

/// Each loop integrates with its own gain: (thrust gain, pitch gain) → (thrust integral grows,
/// pitch integral grows), flying 30 m low and 3 m/s slow, so both errors stay open.
#[test]
fn thrust_and_pitch_integrate_with_their_own_gains() {
    let cases = [
        ((0.3, 0.3), (true, true)),
        ((0.0, 0.3), (false, true)),
        ((0.3, 0.0), (true, false)),
    ];
    let state = level(30.0, 15.0, 0.0);
    for ((integrator_gain, pitch_integrator_gain), (thrust_grows, pitch_grows)) in cases {
        let config = TecsConfig {
            integrator_gain,
            pitch_integrator_gain,
            ..TecsConfig::default()
        };
        let mut tecs = Tecs::default();
        for _ in 0..100 {
            tecs.step(DT, &config, &limits(), (60.0, 18.0), 15.0, &state);
        }
        check!(
            (tecs.thrust_integral != 0.0) == thrust_grows,
            "{integrator_gain}, {pitch_integrator_gain}: {tecs:?}"
        );
        check!(
            (tecs.pitch_integral_rad != 0.0) == pitch_grows,
            "{integrator_gain}, {pitch_integrator_gain}: {tecs:?}"
        );
    }
}

/// A most airspeed below the least reads as the least: the demand holds there, whatever the
/// target.
#[test]
fn most_airspeed_below_the_least_reads_as_the_least() {
    let config = TecsConfig {
        min_airspeed_mps: 12.0,
        max_airspeed_mps: 8.0,
        ..TecsConfig::default()
    };
    for target_mps in [5.0, 12.0, 20.0] {
        let mut tecs = Tecs::default();
        let state = level(30.0, 12.0, 0.0);
        for _ in 0..100 {
            tecs.step(DT, &config, &limits(), (30.0, target_mps), 12.0, &state);
        }
        assert_relative_eq!(tecs.demand.expect("seeded").airspeed_mps, 12.0);
    }
}

/// A climb the pitch limit can't make runs full thrust: the unmet height stays in the total
/// energy error. Pitch stays at its limit, and its integral stops growing there (#454: the
/// speed loop took the climb's power).
#[test]
fn pitch_limited_climb_takes_full_thrust() {
    let config = TecsConfig::default();
    let limits = Limits {
        pitch_up_rad: 3f32.to_radians(),
        ..limits()
    };
    // Not climbing, at speed.
    let state = level(30.0, 15.0, 0.0);
    let mut tecs = Tecs::default();
    let mut last = (0.0, 0.0);
    for _ in 0..3000 {
        last = tecs.step(DT, &config, &limits, (200.0, 15.0), 15.0, &state);
    }
    check!(last.0 == limits.pitch_up_rad);
    check!(last.1 == limits.max_thrust);
    // Pinned from the first steps, the integral barely starts (unchecked, it winds to its
    // bound, 0.49 rad here).
    check!(
        tecs.pitch_integral_rad.abs() < 0.01,
        "{}",
        tecs.pitch_integral_rad
    );
}

/// Where the height demand's slew stops on the target, pitch moves smoothly: the lag keeps the
/// demand's rate continuous. Unlagged, the rate stepped by the whole climb limit in one tick,
/// 25° of pitch here, and the eXplora's setpoint jumped 28° in RealFlight (#454).
#[test]
fn pitch_moves_smoothly_where_the_height_slew_stops() {
    let config = TecsConfig::default();
    let state = level(30.0, 15.0, 0.0);
    let mut tecs = Tecs::default();
    let mut previous: Option<f32> = None;
    let mut largest_deg = 0.0_f32;
    // The slew reaches 32 m after 0.4 s; fly 2 s.
    for _ in 0..200 {
        let (pitch, _) = tecs.step(DT, &config, &limits(), (32.0, 15.0), 15.0, &state);
        if let Some(last) = previous {
            largest_deg = largest_deg.max((pitch - last).abs().to_degrees());
        }
        previous = Some(pitch);
    }
    check!(largest_deg < 1.0, "{largest_deg}°");
}

/// Below 0.9 of the least airspeed: full thrust, and pitch holds airspeed alone (nose down,
/// even with a climb asked). It clears back at the least airspeed.
#[test]
fn underspeed_runs_full_thrust_and_pitches_for_airspeed() {
    let config = TecsConfig::default();
    let limits = limits();
    let slow_mps = 0.8 * config.min_airspeed_mps;
    let mut tecs = Tecs::default();
    let (pitch, thrust) = tecs.step(
        DT,
        &config,
        &limits,
        (60.0, 15.0),
        slow_mps,
        &level(30.0, slow_mps, 0.0),
    );
    check!(tecs.underspeed);
    check!(thrust == limits.max_thrust);
    check!(pitch < limits.trim_pitch_rad, "{pitch}");

    // Over the latch but under the least airspeed: still latched.
    let between_mps = config.min_airspeed_mps * 0.95;
    tecs.step(
        DT,
        &config,
        &limits,
        (60.0, 15.0),
        between_mps,
        &level(30.0, between_mps, 0.0),
    );
    check!(tecs.underspeed);
    // Back at the least airspeed: cleared.
    let least_mps = config.min_airspeed_mps;
    tecs.step(
        DT,
        &config,
        &limits,
        (60.0, 15.0),
        least_mps,
        &level(30.0, least_mps, 0.0),
    );
    check!(!tecs.underspeed);
}

/// The airspeed demand starts within the airspeed limits: from under the least airspeed but
/// over the underspeed latch, thrust rises toward the least airspeed instead of holding the
/// slow speed.
#[test]
fn airspeed_demand_starts_at_least_the_least_airspeed() {
    let config = TecsConfig::default();
    let slow_mps = 0.95 * config.min_airspeed_mps;
    let mut tecs = Tecs::default();
    let (_, thrust) = tecs.step(
        DT,
        &config,
        &limits(),
        (30.0, slow_mps),
        slow_mps,
        &level(30.0, slow_mps, 0.0),
    );
    check!(!tecs.underspeed);
    check!(tecs.demand.expect("seeded").airspeed_mps == config.min_airspeed_mps);
    check!(thrust > limits().cruise_thrust);
}

/// A bank adds thrust for its drag, on target, and leaves pitch alone: (bank°, thrust above
/// cruise).
#[test]
fn bank_adds_thrust_for_its_drag() {
    let config = TecsConfig::default();
    let limits = limits();
    let max_rate = GRAVITY * config.max_climb_mps;
    let share = |bank_deg: f32| {
        let cos = bank_deg.to_radians().cos();
        config.roll_compensation * (1.0 / (cos * cos).max(MIN_COS2_BANK) - 1.0) / max_rate
            * (limits.max_thrust - limits.cruise_thrust)
    };
    for bank_deg in [0.0_f32, 30.0, 45.0, 85.0] {
        let (pitch, thrust) = first_step((30.0, 15.0), &level(30.0, 15.0, bank_deg.to_radians()));
        assert_relative_eq!(
            thrust,
            (limits.cruise_thrust + share(bank_deg)).min(limits.max_thrust),
            epsilon = 1e-5
        );
        assert_relative_eq!(pitch, limits.trim_pitch_rad, epsilon = 1e-6);
    }
}

/// A non-finite target, airspeed, height, or climb gives trim and cruise and leaves the state
/// alone.
#[test]
fn non_finite_inputs_give_trim_and_cruise() {
    let config = TecsConfig::default();
    let limits = limits();
    let nan = f32::NAN;
    let mut no_height = level(30.0, 15.0, 0.0);
    no_height.position_m.z = nan;
    let mut no_climb = level(30.0, 15.0, 0.0);
    no_climb.velocity_mps.z = f32::INFINITY;
    let cases = [
        ((nan, 15.0), 15.0, level(30.0, 15.0, 0.0)),
        ((30.0, nan), 15.0, level(30.0, 15.0, 0.0)),
        ((30.0, 15.0), nan, level(30.0, 15.0, 0.0)),
        ((30.0, 15.0), 15.0, no_height),
        ((30.0, 15.0), 15.0, no_climb),
    ];
    for (target, airspeed, state) in cases {
        let mut tecs = Tecs::default();
        let out = tecs.step(DT, &config, &limits, target, airspeed, &state);
        check!(
            out == (limits.trim_pitch_rad, limits.cruise_thrust),
            "{target:?}"
        );
        check!(tecs == Tecs::default());
    }
}

/// `reset` forgets the demands and integrals: the next step matches a fresh one.
#[test]
fn reset_starts_fresh() {
    let config = TecsConfig::default();
    let state = level(30.0, 15.0, 0.0);
    let mut tecs = Tecs::default();
    for _ in 0..200 {
        tecs.step(DT, &config, &limits(), (60.0, 18.0), 15.0, &state);
    }
    tecs.reset();
    check!(tecs == Tecs::default());
}

/// Each field out of range is rejected by name.
#[test]
fn validates_config() {
    type Edit = fn(&mut TecsConfig);
    let cases: [(&str, Edit); 11] = [
        ("time_constant_s", |c| c.time_constant_s = 0.0),
        ("max_climb_mps", |c| c.max_climb_mps = -1.0),
        ("min_sink_mps", |c| c.min_sink_mps = 0.0),
        ("max_sink_mps", |c| c.max_sink_mps = f32::NAN),
        ("thrust_damping", |c| c.thrust_damping = -0.1),
        ("integrator_gain", |c| c.integrator_gain = -0.1),
        ("pitch_damping", |c| c.pitch_damping = f32::INFINITY),
        ("pitch_integrator_gain", |c| c.pitch_integrator_gain = -0.1),
        ("speed_weight", |c| c.speed_weight = 2.5),
        ("min_airspeed_mps", |c| c.min_airspeed_mps = 0.0),
        ("max_airspeed_mps", |c| c.max_airspeed_mps = -5.0),
    ];
    check!(TecsConfig::default().validate() == Ok(()));
    for (field, edit) in cases {
        let mut config = TecsConfig::default();
        edit(&mut config);
        let error = config.validate().expect_err(field);
        check!(error.field == field, "{error:?}");
    }
}

/// The airspeed rate comes from the airspeed itself: the backend's acceleration, however it
/// reads, moves nothing.
#[test]
fn airspeed_rate_ignores_the_backends_acceleration() {
    let config = TecsConfig::default();
    let calm = level(30.0, 15.0, 0.0);
    let spiking = VehicleState {
        acceleration_mps2: Vec3::new(8.0, 0.0, -5.0),
        ..calm
    };
    let (mut quiet, mut noisy) = (Tecs::default(), Tecs::default());
    for _ in 0..50 {
        let a = quiet.step(DT, &config, &limits(), (35.0, 16.0), 15.0, &calm);
        let b = noisy.step(DT, &config, &limits(), (35.0, 16.0), 15.0, &spiking);
        check!(a == b);
    }
}

/// On an airspeed ramp, the rate settles on the ramp's slope within a few time constants.
#[test]
fn airspeed_rate_follows_a_ramp() {
    let mut tecs = Tecs::default();
    let slope_mps2 = 0.5;
    let mut airspeed_mps = 15.0;
    let steps = (5.0 * AIRSPEED_RATE_TC_S / DT) as usize;
    for _ in 0..steps {
        tecs.step(
            DT,
            &TecsConfig::default(),
            &limits(),
            (30.0, 15.0),
            airspeed_mps,
            &level(30.0, airspeed_mps, 0.0),
        );
        airspeed_mps += slope_mps2 * DT;
    }
    let filtered = tecs.filtered_airspeed_mps.expect("seeded");
    let rate = (airspeed_mps - slope_mps2 * DT - filtered) / AIRSPEED_RATE_TC_S;
    assert_relative_eq!(rate, slope_mps2, max_relative = 0.05);
}
