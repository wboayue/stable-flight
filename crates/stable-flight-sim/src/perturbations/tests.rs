use assert2::check;

use super::*;

fn all_on() -> Perturbations {
    Perturbations {
        actuator_delay_s: MAX_ACTUATOR_DELAY_S,
        gyro: GyroNoise {
            rms_radps: Vec3::repeat(0.02),
            vibration: Some(Vibration {
                amplitude_radps: Vec3::repeat(0.05),
                frequency_hz: 90.0,
            }),
            seed: 1,
        },
        disturbance_moment_nm: Vec3::new(-0.01, 0.02, 0.0),
        motor_thrust_scale: [0.9; ActuatorCommand::SLOTS],
        time_scale: 1.12,
    }
}

#[test]
fn accepts_off_and_all_on() {
    check!(Perturbations::default().validate() == Ok(()));
    check!(all_on().validate() == Ok(()));
}

/// An edit applied to valid perturbations.
type Change = fn(&mut Perturbations);

#[test]
fn rejects_each_out_of_range_field() {
    fn vibration(p: &mut Perturbations) -> &mut Vibration {
        p.gyro.vibration.as_mut().expect("all_on has a vibration")
    }
    // (change to valid perturbations, expected error)
    let cases: [(Change, ModelError); 12] = [
        (
            |p| p.actuator_delay_s = -0.001,
            ModelError::Negative("actuator_delay_s"),
        ),
        (
            |p| p.actuator_delay_s = f32::NAN,
            ModelError::Negative("actuator_delay_s"),
        ),
        (
            |p| p.actuator_delay_s = MAX_ACTUATOR_DELAY_S + 0.001,
            ModelError::DelayTooLong("actuator_delay_s"),
        ),
        (
            |p| p.gyro.rms_radps.y = -0.1,
            ModelError::Negative("gyro.rms_radps.y"),
        ),
        (
            |p| vibration(p).amplitude_radps.z = f32::INFINITY,
            ModelError::Negative("gyro.vibration.amplitude_radps.z"),
        ),
        (
            |p| vibration(p).frequency_hz = 0.0,
            ModelError::NotPositive("gyro.vibration.frequency_hz"),
        ),
        (
            |p| p.disturbance_moment_nm.x = f32::NAN,
            ModelError::NotFinite("disturbance_moment_nm"),
        ),
        (
            |p| p.motor_thrust_scale[3] = 0.0,
            ModelError::ThrustScale(3),
        ),
        (
            |p| p.motor_thrust_scale[7] = f32::NAN,
            ModelError::ThrustScale(7),
        ),
        (
            |p| p.motor_thrust_scale[0] = -1.0,
            ModelError::ThrustScale(0),
        ),
        (
            |p| p.time_scale = 0.0,
            ModelError::NotPositive("time_scale"),
        ),
        (
            |p| p.time_scale = f32::INFINITY,
            ModelError::NotPositive("time_scale"),
        ),
    ];
    for (change, error) in cases {
        let mut perturbations = all_on();
        change(&mut perturbations);
        check!(perturbations.validate() == Err(error));
    }
}
