use assert2::check;
use stable_flight::stabilizers::copter::CopterStabilizer;
use stable_flight::{ActuatorCommand, ControlOutput, CruiseOutput};
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

/// Each surface link's deflection for `command`, as (wing, degrees).
fn deflections(command: &ActuatorCommand) -> Vec<(usize, f32)> {
    alia250()
        .mapping
        .surfaces
        .iter()
        .map(|link| (link.wing, link.deflection_deg(command.servos[link.servo])))
        .collect()
}

/// The plane side and transitions are the sim Alia's, which #392 tuned on a model fitted to
/// this aircraft: its gains (the rate feedforward to the 4th digit, worked out from the model
/// there), limits, and total energy control. X-Plane's own is the cruise thrust, the pusher it
/// held level at trim (the sim works its own out from the model).
#[test]
fn alia250_flies_the_sim_alias_plane_side() {
    let sim = stable_flight_sim::airframes::alia250();
    let xplane = alia250();

    let (mut ours, theirs) = (xplane.stabilizer.plane, sim.stabilizer.plane);
    for (axis, (ff, sim_ff)) in [
        (ours.rate.roll.ff, theirs.rate.roll.ff),
        (ours.rate.pitch.ff, theirs.rate.pitch.ff),
        (ours.rate.yaw.ff, theirs.rate.yaw.ff),
    ]
    .into_iter()
    .enumerate()
    {
        check!((ff - sim_ff).abs() < 1e-3, "axis {axis}: {ff} vs {sim_ff}");
    }
    ours.rate.roll.ff = theirs.rate.roll.ff;
    ours.rate.pitch.ff = theirs.rate.pitch.ff;
    ours.rate.yaw.ff = theirs.rate.yaw.ff;
    check!(ours == theirs);

    let plane = PlaneGuidanceConfig {
        cruise_thrust: sim.guidance.plane.cruise_thrust,
        ..xplane.guidance.plane
    };
    check!(plane == sim.guidance.plane);
    check!(xplane.guidance.transition == sim.guidance.transition);
}

/// The hover gains, lift mixing, and thrust law are the sim Alia's, which the autotune tuned
/// (`stable-flight-sim` `tests/alia.rs`). X-Plane's own: the roll and pitch rate I that trims
/// its nose-up pitch and the angle I that holds a tilt against the moment its airframe builds
/// while drifting, both of which the sim leaves out, and the pitch rate P its step search
/// tuned (#540, #576).
#[test]
fn alia250_hovers_on_the_sim_alias_tune() {
    let sim = stable_flight_sim::airframes::alia250_hover();
    let mut xplane = alia250();
    let hover = &mut xplane.stabilizer.hover;
    check!(hover.body_rate.roll_gains.i == 0.2);
    check!(hover.body_rate.pitch_gains.i == 0.2);
    check!(hover.body_rate.pitch_gains.p == 1.78);
    check!(hover.attitude.roll_gains.i == 0.25);
    check!(hover.attitude.pitch_gains.i == 0.15);
    check!(hover.body_rate.max_integral == 0.2);
    hover.body_rate.roll_gains.i = 0.0;
    hover.body_rate.pitch_gains.i = 0.0;
    hover.body_rate.max_integral = sim.stabilizer.body_rate.max_integral;
    hover.body_rate.pitch_gains.p = sim.stabilizer.body_rate.pitch_gains.p;
    hover.attitude.roll_gains.i = sim.stabilizer.attitude.roll_gains.i;
    hover.attitude.pitch_gains.i = sim.stabilizer.attitude.pitch_gains.i;
    check!(*hover == sim.stabilizer);

    // Worked out each its own way from the same liftoff throttle.
    let guidance = &mut xplane.guidance.hover;
    check!((guidance.altitude.hover_thrust - sim.guidance.altitude.hover_thrust).abs() < 1e-4);
    guidance.altitude.hover_thrust = sim.guidance.altitude.hover_thrust;
    check!(*guidance == sim.guidance);

    // The lift rotors mix as the sim's copter, with and without a cruise output.
    for moments in [Vec3::new(0.3, -0.2, 0.5), Vec3::new(-0.6, 0.4, -0.9)] {
        let hover = ControlOutput::new(0.6, moments);
        let blended = ControlOutput {
            cruise: Some(CruiseOutput {
                thrust: 0.5,
                moments,
            }),
            ..hover
        };
        let expected = sim.mixer.clone().mix(&hover).motors;
        for control in [hover, blended] {
            let lift = xplane.mixer.mix(&control).motors;
            check!(lift[..4] == expected[..4], "{control:?}");
        }
    }
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

/// The lift rotors alone on `quad_x`, yaw reversed, at the Alia's mixer config (how
/// `copter-autotune-xplane` flies its hover) mix as its quadplane mixer does in copter flight,
/// clipping included, so gains tuned on the one fly the other. The pusher and surfaces stay at
/// 0.
#[test]
fn alia250_lift_rotors_mix_as_quad_x_with_reversed_yaw() {
    let alia = alia250();
    let mut lift_only = TableMixer::quad_x()
        .with_reversed_yaw()
        .with_config(alia.mixer.config())
        .expect("the Alia's mixer config is valid");
    // (thrust, moments)
    let cases = [
        (0.9, Vec3::zeros()),
        (0.9, Vec3::new(0.3, 0.0, 0.0)),
        (0.9, Vec3::new(0.0, -0.3, 0.0)),
        (0.9, Vec3::new(0.0, 0.0, 0.3)),
        (0.9, Vec3::new(-0.2, 0.3, -0.4)),
        (0.95, Vec3::new(1.0, 1.0, 1.0)),
        (0.05, Vec3::new(-1.0, 0.5, -1.0)),
    ];
    for (thrust, moments) in cases {
        let control = ControlOutput::new(thrust, moments);
        let quadplane = alia250().mixer.mix(&control);
        let lift = lift_only.mix(&control);
        check!(
            lift.motors[..4] == quadplane.motors[..4],
            "{thrust} {moments:?}"
        );
        check!(lift.motors[4..] == [0.0; 4], "{thrust} {moments:?}");
        check!(lift.servos == [0.0; 8], "{thrust} {moments:?}");
    }
}

/// Full cruise moments deflect each surface as X-Plane's own yoke does at full travel
/// (xplane-bridge `design.md`, Spike findings: Surfaces): right roll drops the left ailerons
/// 10° and raises the right 20°, nose up raises both elevators 20° (X-Plane's −20), nose right
/// reads −30° on the left fin and +30° on the right. The pusher flies the cruise thrust up to
/// the mixer's 0.8.
#[test]
fn alia250_surfaces_move_as_xplanes_yoke() {
    // (cruise moments, (wing, degrees) for every link)
    let cases = [
        (
            Vec3::new(1.0, 0.0, 0.0),
            [(2, 10.0), (4, 10.0), (3, -20.0), (5, -20.0)].as_slice(),
        ),
        (
            Vec3::new(-1.0, 0.0, 0.0),
            [(2, -20.0), (4, -20.0), (3, 10.0), (5, 10.0)].as_slice(),
        ),
        (
            Vec3::new(0.0, 1.0, 0.0),
            [(8, -20.0), (9, -20.0)].as_slice(),
        ),
        (
            Vec3::new(0.0, 0.0, 1.0),
            [(10, -30.0), (11, 30.0)].as_slice(),
        ),
    ];
    for (moments, expected) in cases {
        let mut mixer = alia250().mixer;
        let command = mixer.mix(&ControlOutput {
            cruise: Some(CruiseOutput {
                thrust: 1.0,
                moments,
            }),
            ..Default::default()
        });
        let moved: Vec<_> = deflections(&command)
            .into_iter()
            .filter(|&(_, deg)| deg != 0.0)
            .collect();
        check!(moved == expected, "{moments:?}");
        check!(engines(&command) == [0.0, 0.0, 0.0, 0.0, 1.0]);
    }
}

/// The hover thrust, through the stabilizer's curve and the mixer, runs each lift engine at
/// the measured liftoff throttle; the cruise thrust runs the pusher at the cruise throttle.
#[test]
fn alia250_trims_run_the_measured_throttles() {
    let airframe = alia250();
    let mut stabilizer = CopterStabilizer::new(airframe.stabilizer.hover);
    let setpoint = Setpoint::Attitude(AttitudeSetpoint {
        thrust: airframe.guidance.hover.altitude.hover_thrust,
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

    let cruise = ControlOutput {
        cruise: Some(CruiseOutput {
            thrust: airframe.guidance.plane.cruise_thrust,
            moments: Vec3::zeros(),
        }),
        ..Default::default()
    };
    let pusher = engines(&mixer.mix(&cruise))[4];
    check!((pusher - ALIA_CRUISE_THROTTLE).abs() < 1e-6);
}
