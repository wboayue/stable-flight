use approx::assert_relative_eq;
use assert2::check;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{GRAVITY, Vec3};

use super::*;
use crate::airframes;
use crate::lockstep::vehicle_state;
use crate::perturbations::{GyroNoise, Vibration};

const TICK: Duration = Duration::from_millis(4);

fn vehicle() -> SimCopter {
    let airframe = airframes::quad_x();
    SimCopter::new(airframe.model, &airframe.mixer).expect("valid airframe")
}

fn motors(command: f32) -> ActuatorCommand {
    ActuatorCommand {
        motors: [command; ActuatorCommand::SLOTS],
        ..Default::default()
    }
}

/// Runs `ticks` loop ticks: read state, then actuate `command`, as `FlightLoop::step` does.
fn fly(vehicle: &mut SimCopter, command: &ActuatorCommand, ticks: usize) -> VehicleState {
    let mut state = vehicle.state(Duration::ZERO);
    for _ in 0..ticks {
        state = vehicle.state(TICK);
        vehicle.actuate(command);
    }
    state
}

#[test]
fn starts_at_rest_on_the_ground() {
    let mut vehicle = vehicle();
    let state = vehicle.state(Duration::ZERO);
    check!(state == vehicle_state(&Body::default(), Vec3::zeros()));
    check!(vehicle.on_ground());
    check!(vehicle.time_s() == 0.0);
}

#[test]
fn priming_call_does_not_step() {
    let mut vehicle = vehicle();
    vehicle.actuate(&motors(1.0));
    vehicle.state(Duration::ZERO);
    check!(vehicle.time_s() == 0.0);
    check!(vehicle.motor_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
}

#[test]
fn state_steps_the_last_actuated_command() {
    let mut vehicle = vehicle();
    vehicle.state(TICK);
    // Nothing commanded yet: the motors stay stopped.
    check!(vehicle.motor_thrusts_n()[0] == 0.0);
    vehicle.actuate(&motors(1.0));
    vehicle.state(TICK);
    check!(vehicle.motor_thrusts_n()[0] > 0.0);
    assert_relative_eq!(vehicle.time_s(), 0.008, epsilon = 1e-6);
}

#[test]
fn idle_stays_on_the_ground_at_rest() {
    let mut vehicle = vehicle();
    let state = fly(&mut vehicle, &motors(0.3), 250);
    check!(vehicle.on_ground());
    check!(state.position_m == Vec3::zeros());
    assert_relative_eq!(state.acceleration_mps2, Vec3::zeros(), epsilon = 1e-4);
}

#[test]
fn full_throttle_takes_off_and_climbs() {
    let mut vehicle = vehicle();
    let state = fly(&mut vehicle, &motors(1.0), 250);
    check!(!vehicle.on_ground());
    check!(state.position_m.z > 1.0);
    check!(state.velocity_mps.z > 0.0);
    check!(state.acceleration_mps2.z > 0.0);
}

#[test]
fn cutting_the_motors_falls_back_and_lands() {
    let mut vehicle = vehicle();
    fly(&mut vehicle, &motors(1.0), 125);
    // Coasting up after the cut: gravity plus drag against the climb, once the props spin down.
    let coasting = fly(&mut vehicle, &motors(0.0), 50);
    check!(coasting.velocity_mps.z > 0.0);
    check!(coasting.acceleration_mps2.z < -GRAVITY);
    let landed = fly(&mut vehicle, &motors(0.0), 2000);
    check!(vehicle.on_ground());
    check!(landed.position_m.z == 0.0);
    check!(landed.velocity_mps == Vec3::zeros());
}

/// A tick longer than the max step is split into equal steps no longer than it, so a slower
/// loop still integrates finely: a 10 ms tick at 1 ms steps matches ten 1 ms ticks.
#[test]
fn long_ticks_split_into_max_steps() {
    let model = MultirotorModel {
        max_step_s: 0.001,
        ..airframes::quad_x().model
    };
    let mixer = airframes::quad_x().mixer;
    let mut coarse = SimCopter::new(model, &mixer).expect("valid airframe");
    let mut fine = coarse.clone();
    coarse.actuate(&motors(1.0));
    fine.actuate(&motors(1.0));
    let long = coarse.state(Duration::from_millis(10));
    let mut short = fine.state(Duration::ZERO);
    for _ in 0..10 {
        short = fine.state(Duration::from_millis(1));
    }
    assert_relative_eq!(long.position_m, short.position_m, epsilon = 1e-6);
    assert_relative_eq!(long.velocity_mps, short.velocity_mps, epsilon = 1e-5);
    assert_relative_eq!(coarse.time_s(), 0.010, epsilon = 1e-9);
}

#[test]
fn servos_are_ignored() {
    let mut vehicle = vehicle();
    let command = ActuatorCommand {
        servos: [1.0; ActuatorCommand::SLOTS],
        ..Default::default()
    };
    let state = fly(&mut vehicle, &command, 50);
    check!(vehicle.motor_thrusts_n() == [0.0; ActuatorCommand::SLOTS]);
    check!(state.position_m == Vec3::zeros());
}

#[test]
fn airspeed_is_against_the_wind() {
    let wind = Wind {
        steady_mps: Vec3::new(3.0, 4.0, 0.0),
        gust: None,
    };
    let mut vehicle = vehicle().with_wind(wind);
    // At rest on the ground in a 5 m/s wind.
    let state = vehicle.state(Duration::ZERO);
    assert_relative_eq!(state.airspeed_mps.unwrap_or(f32::NAN), 5.0, epsilon = 1e-5);
}

#[test]
fn wind_pushes_an_airborne_vehicle_downwind() {
    let wind = Wind {
        steady_mps: Vec3::new(0.0, 5.0, 0.0),
        gust: None,
    };
    let mut vehicle = vehicle().with_wind(wind);
    let state = fly(&mut vehicle, &motors(1.0), 500);
    check!(state.velocity_mps.y > 0.5);
    check!(state.position_m.x.abs() < 1e-3);
}

#[test]
fn rejects_an_invalid_model_or_a_servo_mixer() {
    let airframe = airframes::quad_x();
    let model = MultirotorModel {
        max_step_s: 0.0,
        ..airframe.model
    };
    check!(
        SimCopter::new(model, &airframe.mixer).err() == Some(ModelError::NotPositive("max_step_s"))
    );
    check!(SimCopter::new(airframe.model, &TableMixer::tri()).err() == Some(ModelError::Servo(0)));
}

#[test]
fn same_commands_give_the_same_state() {
    let run = || {
        let mut vehicle = vehicle().with_wind(Wind {
            steady_mps: Vec3::new(1.0, -2.0, 0.0),
            gust: None,
        });
        let mut command = motors(0.7);
        command.motors[0] = 0.75;
        fly(&mut vehicle, &command, 500)
    };
    check!(run() == run());
}

fn perturbed(perturbations: Perturbations) -> SimCopter {
    vehicle()
        .with_perturbations(perturbations)
        .expect("valid perturbations")
}

/// Each `state` call advances `time_scale` times its `dt` of simulated time.
#[test]
fn time_scale_runs_the_physics_ahead_of_the_ticks() {
    // (time scale, simulated s after 10 ticks of 4 ms)
    let cases = [(1.0, 0.04), (1.12, 0.0448), (2.0, 0.08)];
    for (time_scale, expected_s) in cases {
        let mut vehicle = perturbed(Perturbations {
            time_scale,
            ..Default::default()
        });
        for _ in 0..10 {
            vehicle.state(Duration::from_millis(4));
        }
        assert_relative_eq!(vehicle.time_s(), expected_s, epsilon = 1e-6);
    }
}

#[test]
fn rejects_invalid_perturbations() {
    let perturbations = Perturbations {
        actuator_delay_s: 1.0,
        ..Default::default()
    };
    check!(vehicle().with_perturbations(perturbations).err() == Some(ModelError::DelayTooLong));
}

/// A command reaches the motors one tick (the hold) plus `actuator_delay_s` after `actuate`,
/// to the 1 ms tick.
#[test]
fn delays_motor_commands_by_actuator_delay() {
    const MS: Duration = Duration::from_millis(1);
    for delay_ms in [0_u32, 10, 25] {
        let mut vehicle = perturbed(Perturbations {
            actuator_delay_s: delay_ms as f32 * 1e-3,
            ..Default::default()
        });
        vehicle.actuate(&motors(1.0));
        // The step that starts `delay_ms` after `actuate` is the first to spin the motors.
        for _ in 0..delay_ms {
            vehicle.state(MS);
        }
        check!(vehicle.motor_thrusts_n()[0] == 0.0, "{delay_ms} ms");
        vehicle.state(MS);
        check!(vehicle.motor_thrusts_n()[0] > 0.0, "{delay_ms} ms");
    }
}

/// At rest on the ground the true rates are zero, so the reported rates are the noise alone.
#[test]
fn gyro_noise_rms_reads_back() {
    const SAMPLES: usize = 10_000;
    let rms_radps = Vec3::new(0.01, 0.02, 0.05);
    let mut vehicle = perturbed(Perturbations {
        gyro: GyroNoise {
            rms_radps,
            vibration: None,
            seed: 11,
        },
        ..Default::default()
    });
    let sum_squares = (0..SAMPLES)
        .map(|_| vehicle.state(TICK).angular_velocity_radps)
        .fold(Vec3::zeros(), |sum, rates| {
            sum + rates.component_mul(&rates)
        });
    let measured_radps = (sum_squares / SAMPLES as f32).map(libm::sqrtf);
    check!(vehicle.on_ground());
    // The RMS estimate's standard error is RMS/√(2N), 0.7% here.
    assert_relative_eq!(measured_radps, rms_radps, max_relative = 0.03);
}

/// Zero crossings over a second of 1 kHz samples give the tone's frequency, and the peak its
/// amplitude.
#[test]
fn vibration_tone_reads_back() {
    const MS: Duration = Duration::from_millis(1);
    let amplitude_radps = Vec3::new(0.05, 0.1, 0.02);
    let frequency_hz = 80.0;
    let mut vehicle = perturbed(Perturbations {
        gyro: GyroNoise {
            vibration: Some(Vibration {
                amplitude_radps,
                frequency_hz,
            }),
            ..Default::default()
        },
        ..Default::default()
    });
    let rolls: Vec<f32> = (0..1000)
        .map(|_| vehicle.state(MS).angular_velocity_radps.x)
        .collect();
    let crossings = rolls
        .windows(2)
        .filter(|pair| (pair[0] < 0.0) != (pair[1] < 0.0))
        .count();
    check!((crossings as f32 / 2.0 - frequency_hz).abs() <= 1.0);
    let peak_radps = rolls
        .iter()
        .fold(0.0_f32, |peak, roll| peak.max(roll.abs()));
    // 1 kHz samples land within 0.04 cycle of the peak: sin ≥ 0.97.
    assert_relative_eq!(peak_radps, amplitude_radps.x, max_relative = 0.03);
}

/// From rest in the air, the disturbance moment accelerates each axis at `M / I`.
#[test]
fn disturbance_moment_accelerates_the_body() {
    let moment_nm = Vec3::new(0.01, -0.02, 0.005);
    let mut vehicle = perturbed(Perturbations {
        disturbance_moment_nm: moment_nm,
        ..Default::default()
    });
    vehicle.lockstep.body.position_ned_m.z = -10.0;
    let h_s = 0.001;
    let state = vehicle.state(Duration::from_millis(1));
    let expected_radps2 = moment_nm.component_div(&vehicle.dynamics.model.inertia_kgm2);
    assert_relative_eq!(
        state.angular_velocity_radps / h_s,
        expected_radps2,
        max_relative = 1e-4
    );
}

/// The multirotor's loads: thrust along body −z, drag against the air-relative velocity, and
/// damping against the body rates.
#[test]
fn loads_point_the_contract_way() {
    let model = airframes::quad_x().model;
    let mut dynamics = vehicle().dynamics;
    dynamics.begin(1.0);
    // All motors at full command after a step far longer than the lag: full thrust.
    let full_n = 4.0 * model.max_thrust_n;
    let alpha = 1.0 - libm::expf(-1.0 / model.motor_time_constant_s);
    let thrust_n = full_n * alpha * alpha;
    // (roll rad, body velocity NED m/s, wind NED m/s, body rates rad/s, expected force NED N)
    let cases = [
        (
            0.0,
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, -thrust_n),
        ),
        (
            0.0,
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::zeros(),
            Vec3::new(-2.0, 1.0, 0.0) * model.drag_n_per_mps + Vec3::new(0.0, 0.0, -thrust_n),
        ),
        // Banked right, the thrust leans east (NED +y).
        (
            0.2,
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::new(1.0, -1.0, 0.5),
            Vec3::new(0.0, libm::sinf(0.2), -libm::cosf(0.2)) * thrust_n,
        ),
    ];
    for (roll_rad, velocity_ned_mps, wind_ned_mps, rates_radps, force_ned_n) in cases {
        let mut dynamics = dynamics.clone();
        let body = Body {
            velocity_ned_mps,
            attitude: UnitQuaternion::from_euler_angles(roll_rad, 0.0, 0.0),
            angular_velocity_radps: rates_radps,
            ..Body::default()
        };
        let loads = dynamics.loads(&motors(1.0), &body, wind_ned_mps);
        assert_relative_eq!(loads.force_ned_n, force_ned_n, epsilon = 1e-5);
        assert_relative_eq!(
            loads.damping_nm,
            model
                .angular_damping_nm_per_radps
                .component_mul(&rates_radps)
        );
        assert_relative_eq!(
            dynamics.lift_n(&body),
            thrust_n * libm::cosf(roll_rad),
            epsilon = 1e-5
        );
    }
}
