use approx::assert_relative_eq;
use assert2::check;
use stable_flight::mixers::TableMixer;
use stable_flight::{ControlOutput, Mixer};

use super::*;
use crate::MultirotorModel;
use crate::airframes;

fn quad_x_motors() -> (Motors, MultirotorModel) {
    let model = airframes::quad_x().model;
    let motors = Motors::new(TableMixer::quad_x().rows(), &model.motor_layout());
    (motors, model)
}

/// Runs `commands` long enough for every motor to settle on it.
fn settled(motors: &mut Motors, commands: &[f32; SLOTS]) {
    motors.spin(commands, 1.0);
}

#[test]
fn counts_the_mixer_motors() {
    check!(quad_x_motors().0.count() == 4);
    let hex = Motors::new(
        TableMixer::hex_x().rows(),
        &quad_x_motors().1.motor_layout(),
    );
    check!(hex.count() == 6);
}

#[test]
fn skips_servo_rows() {
    let model = quad_x_motors().1;
    let tri = Motors::new(TableMixer::tri().rows(), &model.motor_layout());
    check!(tri.count() == 3);
}

#[test]
fn thrust_is_quadratic_in_the_command() {
    let (mut motors, model) = quad_x_motors();
    // (command, fraction of max thrust)
    let cases = [
        (0.0, 0.0),
        (0.5, 0.25),
        (1.0, 1.0),
        (1.5, 1.0),
        (-0.2, 0.0),
        (f32::NAN, 0.0),
    ];
    for (command, fraction) in cases {
        settled(&mut motors, &[command; SLOTS]);
        let thrusts_n = motors.thrusts_n();
        assert_relative_eq!(thrusts_n[0], model.max_thrust_n * fraction);
        // Unused slots stay stopped.
        check!(thrusts_n[4] == 0.0);
    }
}

/// A power-commanded motor (exponent 2/3, X-Plane's electric engines) gives more than half
/// its thrust at half command.
#[test]
fn thrust_follows_the_model_exponent() {
    let layout = MotorLayout {
        thrust_exponent: 2.0 / 3.0,
        ..quad_x_motors().1.motor_layout()
    };
    let mut motors = Motors::new(TableMixer::quad_x().rows(), &layout);
    // (command, fraction of max thrust)
    let cases = [(0.0, 0.0), (0.5, 0.629_960_5), (1.0, 1.0)];
    for (command, fraction) in cases {
        settled(&mut motors, &[command; SLOTS]);
        assert_relative_eq!(
            motors.thrusts_n()[0],
            layout.max_thrust_n * fraction,
            max_relative = 1e-5
        );
    }
}

#[test]
fn lag_moves_by_alpha_each_step() {
    let (mut motors, model) = quad_x_motors();
    motors.spin(&[1.0; SLOTS], 0.25);
    assert_relative_eq!(motors.thrusts_n()[0], model.max_thrust_n * 0.0625);
    motors.spin(&[1.0; SLOTS], 0.25);
    // 0.25 + 0.75 · 0.25 = 0.4375
    assert_relative_eq!(motors.thrusts_n()[0], model.max_thrust_n * 0.4375 * 0.4375);
}

#[test]
fn equal_thrust_makes_no_torque() {
    let (mut motors, model) = quad_x_motors();
    settled(&mut motors, &[0.6; SLOTS]);
    let (thrust_n, torque_nm) = motors.wrench();
    assert_relative_eq!(thrust_n, 4.0 * model.max_thrust_n * 0.36, epsilon = 1e-5);
    assert_relative_eq!(torque_nm, Vec3::zeros(), epsilon = 1e-6);
}

/// The mixer's response to each moment turns the simulated body the same way: the sim's
/// torque signs match the frame contract the mixer is written for.
#[test]
fn mixed_moments_torque_the_body_the_same_way() {
    let (mut motors, _) = quad_x_motors();
    let mut mixer = TableMixer::quad_x();
    // (moments, axis that must turn positive)
    let cases = [
        (Vec3::new(0.5, 0.0, 0.0), 0),
        (Vec3::new(0.0, 0.5, 0.0), 1),
        (Vec3::new(0.0, 0.0, 0.5), 2),
    ];
    for (moments, axis) in cases {
        for sign in [1.0, -1.0] {
            let command = mixer.mix(&ControlOutput::new(0.6, moments * sign));
            settled(&mut motors, &command.motors);
            let (_, torque_nm) = motors.wrench();
            check!(torque_nm[axis] * sign > 0.0, "moments {moments:?} × {sign}");
            for other in (0..3).filter(|other| *other != axis) {
                check!(
                    torque_nm[other].abs() < 1e-5,
                    "moments {moments:?} × {sign}"
                );
            }
        }
    }
}

/// One slow motor at an equal hover command leaves the others' torque unbalanced by its
/// missing thrust, about each axis its row reaches.
#[test]
fn mismatched_motors_torque_at_hover() {
    let (mut motors, model) = quad_x_motors();
    let rows = TableMixer::quad_x()
        .rows()
        .map(|row| (row.roll(), row.pitch(), row.yaw()));
    let hover = 0.6;
    let full_n = model.max_thrust_n * hover * hover;
    for slow in 0..4 {
        let mut scale = [1.0; SLOTS];
        scale[slow] = 0.9;
        motors.set_thrust_scale(scale);
        settled(&mut motors, &[hover; SLOTS]);
        let (thrust_n, torque_nm) = motors.wrench();
        // Equal motors cancel, so the torque is the slow motor's lost thrust on its lever.
        let lost_n = 0.1 * full_n;
        let (roll, pitch, yaw) = rows[slow];
        let expected_nm = -lost_n
            * Vec3::new(
                roll * model.moment_arm_m,
                pitch * model.moment_arm_m,
                yaw * model.yaw_torque_per_thrust_m,
            );
        assert_relative_eq!(thrust_n, 4.0 * full_n - lost_n, epsilon = 1e-5);
        assert_relative_eq!(torque_nm, expected_nm, epsilon = 1e-6);
        check!(torque_nm.x.abs() > 0.0, "motor {slow}");
    }
}

/// `excluding` leaves one slot to another group, and `single` drives a slot no row names:
/// each spins only its own slots.
#[test]
fn groups_spin_only_their_own_slots() {
    let model = quad_x_motors().1;
    let mut lift = Motors::excluding(TableMixer::quad_x().rows(), &model.motor_layout(), 3);
    check!(lift.count() == 3);
    let mut pusher = Motors::single(4, 10.0);
    check!(pusher.count() == 1);
    check!(Motors::single(8, 10.0).count() == 0);

    let commands = [1.0; SLOTS];
    settled(&mut lift, &commands);
    settled(&mut pusher, &commands);
    let lift_n = lift.thrusts_n();
    check!(lift_n[..3].iter().all(|thrust_n| *thrust_n > 0.0));
    check!(lift_n[3..] == [0.0; SLOTS - 3]);
    let mut pusher_n = [0.0; SLOTS];
    pusher_n[4] = 10.0;
    check!(pusher.thrusts_n() == pusher_n);
    check!(pusher.wrench() == (10.0, Vec3::zeros()));
    assert_relative_eq!(pusher.max_total_thrust_n(), 10.0);
}
