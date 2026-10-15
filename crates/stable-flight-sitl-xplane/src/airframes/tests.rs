use assert2::check;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{ActuatorCommand, ControlOutput};
use stable_flight::{AttitudeSetpoint, Mixer, Setpoint, Stabilizer, Vec3, VehicleState};

use super::*;

/// Each engine's throttle for `command` through the Alia's mapping, engines 0–4.
fn engines(command: &ActuatorCommand) -> [f32; 5] {
    let mut throttles = [f32::NAN; 5];
    for (engine, throttle) in alia250().mapping.throttles(command) {
        throttles[engine] = throttle;
    }
    throttles
}

#[test]
fn alia250_is_valid() {
    let airframe = alia250();
    check!(airframe.mapping.validate() == Ok(()));
    check!(airframe.guidance.validate() == Ok(()));
    check!(airframe.stabilizer.validate() == Ok(()));
}

/// A nose-right yaw raises engines 1 and 3: the sweep turned the aircraft nose left with
/// engines 0 and 2 up (#351). Roll right raises the left engines (1, 2), pitch up the front
/// ones (0, 1). The pusher stays at 0.
#[test]
fn alia250_moments_drive_the_engines_the_sweep_measured() {
    // (moments, engines that rise)
    let cases = [
        (Vec3::new(0.0, 0.0, 0.5), [1, 3]),
        (Vec3::new(0.5, 0.0, 0.0), [1, 2]),
        (Vec3::new(0.0, 0.5, 0.0), [0, 1]),
    ];
    for (moments, raised) in cases {
        let mut mixer = alia250().mixer;
        let command = mixer.mix(&ControlOutput::new(0.5, moments));
        let throttles = engines(&command);
        for engine in 0..4 {
            let other = (0..4).find(|e| raised.contains(e) != raised.contains(&engine));
            let other = other.expect("a lowered engine");
            check!(
                (throttles[engine] > throttles[other]) == raised.contains(&engine),
                "{moments:?}: {throttles:?}"
            );
        }
        check!(throttles[4] == 0.0);
    }
}

/// The hover thrust, through the stabilizer's curve and the mixer, runs each lift engine at
/// the measured liftoff throttle.
#[test]
fn alia250_hover_thrust_runs_the_liftoff_throttle() {
    let airframe = alia250();
    let mut stabilizer = CopterStabilizer::new(airframe.stabilizer);
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: airframe.guidance.altitude.hover_thrust,
        ..Default::default()
    });
    let output = stabilizer.stabilize(0.004, &setpoint, &VehicleState::default());
    let mut mixer = airframe.mixer;
    let throttles = engines(&mixer.mix(&output));
    for throttle in &throttles[..4] {
        check!(
            (throttle - ALIA_LIFTOFF_THROTTLE).abs() < 1e-3,
            "{throttles:?}"
        );
    }
}
