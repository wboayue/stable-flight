use approx::assert_relative_eq;
use assert2::check;
use stable_flight::mixers::TableMixer;
use stable_flight::{ControlOutput, Mixer};

use super::*;

const SLOTS: usize = ActuatorCommand::SLOTS;

fn servos(values: &[f32]) -> [f32; SLOTS] {
    let mut servos = [0.0; SLOTS];
    servos[..values.len()].copy_from_slice(values);
    servos
}

/// A moment through the mixer reads back as that deflection, on separate surfaces and on
/// elevons that share roll and pitch.
#[test]
fn reads_back_the_moment_the_mixer_meant() {
    let max = Vec3::new(0.3, 0.4, 0.5);
    // (case, mixer rows, moments, expected deflection rad)
    let plane = TableMixer::plane();
    let elevons = TableMixer::tailsitter_2p();
    let cases = [
        (
            "plane roll",
            *plane.rows(),
            Vec3::new(0.5, 0.0, 0.0),
            Vec3::new(0.15, 0.0, 0.0),
        ),
        (
            "plane mixed",
            *plane.rows(),
            Vec3::new(-0.2, 0.5, 1.0),
            Vec3::new(-0.06, 0.2, 0.5),
        ),
        (
            "elevons pitch",
            *elevons.rows(),
            Vec3::new(0.0, 0.5, 0.0),
            Vec3::new(0.0, 0.2, 0.0),
        ),
        (
            "elevons roll",
            *elevons.rows(),
            Vec3::new(0.5, 0.0, 0.0),
            Vec3::new(0.15, 0.0, 0.0),
        ),
        // The tailsitter yaws with its motors: no rudder.
        (
            "elevons yaw",
            *elevons.rows(),
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::zeros(),
        ),
    ];
    for (case, rows, moments, expected) in cases {
        let mut mixer = TableMixer::new(rows, Default::default()).expect("valid preset");
        let command = mixer.mix(&ControlOutput::new(0.5, moments));
        let mut surfaces = Surfaces::new(&rows);
        surfaces.move_to(&command.servos, 1.0);
        assert_relative_eq!(surfaces.deflection_rad(&max), expected, epsilon = 1e-6);
        check!(
            surfaces.deflection_rad(&max).iter().all(|d| d.is_finite()),
            "{case}"
        );
    }
}

#[test]
fn servos_lag_saturate_and_center_on_nan() {
    let max = Vec3::repeat(1.0);
    let mut surfaces = Surfaces::new(TableMixer::plane().rows());
    surfaces.move_to(&servos(&[1.0]), 0.25);
    assert_relative_eq!(surfaces.deflection_rad(&max).x, 0.25);
    surfaces.move_to(&servos(&[1.0]), 0.25);
    assert_relative_eq!(surfaces.deflection_rad(&max).x, 0.4375);
    // (servo command, settled aileron)
    let cases = [(3.0, 1.0), (-2.0, -1.0), (f32::NAN, 0.0)];
    for (command, expected) in cases {
        surfaces.move_to(&servos(&[command]), 1.0);
        check!(
            surfaces.deflection_rad(&max).x == expected,
            "command {command}"
        );
    }
}

#[test]
fn a_motor_only_mixer_moves_nothing() {
    let mut surfaces = Surfaces::new(TableMixer::quad_x().rows());
    surfaces.move_to(&[1.0; SLOTS], 1.0);
    check!(surfaces.deflection_rad(&Vec3::repeat(1.0)) == Vec3::zeros());
}
