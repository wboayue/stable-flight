use super::*;
use stable_flight::Mixer;
use stable_flight::Vec3;
use stable_flight::controllers::{LateralControllerConfig, YawControllerConfig};
use stable_flight::estimation::wind::WindConfig;

#[test]
fn quadcopter_x_maps_its_four_motors_validly() {
    let mapping = quadcopter_x().mapping;

    assert_eq!(mapping.validate(), Ok(()));
    assert!(mapping.motors[..4].iter().all(Option::is_some));
    assert!(mapping.motors[4..].iter().all(Option::is_none));
    assert!(mapping.servos.iter().all(Option::is_none));
}

/// Pins the flight-checked QuadCopterX set (#62), so a retune (or a core default change it
/// inherits) shows up in review.
#[test]
fn quadcopter_x_gains_are_the_flight_checked_set() {
    let airframe = quadcopter_x();
    let gains = |p, i, d| Gains { p, i, d };

    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: gains(0.5, 0.2, 0.004),
            pitch_gains: gains(0.5, 0.2, 0.004),
            yaw_gains: gains(0.6, 0.6, 0.0),
            max_integral: 1.0,
        },
        attitude: AttitudeControllerConfig {
            roll_gains: gains(3.0, 0.0, 0.1),
            pitch_gains: gains(3.0, 0.0, 0.1),
            max_integral: 100.0,
            max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
            max_rate_radps: Vec2::new(3.0, 3.0),
        },
        yaw: YawControllerConfig {
            gains: gains(2.5, 0.0, 0.06),
            max_integral: 100.0,
            max_rate_radps: 2.0,
        },
        full_scale_rates_radps: Vec3::new(3.0, 3.0, 3.0),
        thrust_curve_exponent: 2.0,
    };
    assert_eq!(airframe.stabilizer, stabilizer, "stabilizer");

    let guidance = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            gains: gains(1.5, 0.2, 2.0),
            max_integral: 5.0,
            hover_thrust: 0.82,
        },
        lateral: LateralControllerConfig {
            position_p: 0.5,
            velocity: gains(1.26, 0.4, 0.0),
            max_integral: 12.5,
            max_accel_mps2: 0.0,
            brake_mps2: 0.0,
        },
        navigation_period_s: 0.02,
        max_tilt_rad: 27.0_f32.to_radians(),
        max_stick_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
        max_descent_mps: 3.0,
        descent_accel_mps2: 1.0,
        climb_decel_mps2: 2.5,
        land_speed_mps: 0.7,
        waypoint_yaw_radps: 60.0_f32.to_radians(),
        heading_rate_radps: 60.0_f32.to_radians(),
        waypoint_yaw_min_leg_m: 3.0,
        heading_yield_rad: 30.0_f32.to_radians(),
        heading_yield_s: 1.0,
        wind_heading_min_mps: 0.0,
        thrust: ThrustLimits {
            min_thrust: 0.0,
            sink_limit_mps: 0.0,
            sink_gain: 0.0,
            max_boost_thrust: 1.0,
        },
        wind: WindConfig::default(),
    };
    assert_eq!(airframe.guidance, guidance, "guidance");
}

/// Steady yaw moment (normalized, nose right) QuadCopterX needs to hover in RealFlight,
/// measured (#192): a mean motor yaw split of about 0.10 over the core `TableMixer::quad_x`'s
/// yaw scaling of 0.25. Re-measure if that scaling changes.
const MEASURED_HOVER_YAW_MOMENT: f32 = 0.40;

/// The flown rate gains hold zero yaw rate against the model's hover yaw imbalance with 25%
/// margin; with less integral authority, acro drifts in yaw.
#[test]
fn quadcopter_x_holds_zero_yaw_rate_against_the_hover_imbalance() {
    let body_rate = quadcopter_x().stabilizer.body_rate;
    for disturbance in [MEASURED_HOVER_YAW_MOMENT, 1.25 * MEASURED_HOVER_YAW_MOMENT] {
        let rate = stable_flight::testing::settled_yaw_rate(&body_rate, disturbance);
        assert!(
            rate.abs() < 1e-3,
            "settled at {rate} rad/s against {disturbance}"
        );
    }
}

#[test]
fn explora_vtol_maps_two_motors_and_two_elevons_validly() {
    let mapping = explora_vtol().mapping;

    assert_eq!(mapping.validate(), Ok(()));
    assert_eq!(mapping.motors[..2], [Some(5), Some(4)]);
    assert!(mapping.motors[2..].iter().all(Option::is_none));
    assert_eq!(mapping.servos[..2], [Some(0), Some(1)]);
    assert!(mapping.servos[2..].iter().all(Option::is_none));
}

/// RealFlight-level on its tail is nose-up hover: 90° pitch in the vehicle frame.
#[test]
fn explora_vtol_reads_nose_up_at_realflight_level() {
    let (roll, pitch, _) = stable_flight::euler_angles(&explora_vtol().body_rotation);
    assert!(roll.abs() < 1e-3);
    assert!((pitch - PI / 2.0).abs() < 1e-3, "pitch {pitch}");
}

/// Both motors at about 0.63 hold the eXplora's level hover in RealFlight (#123).
const MEASURED_HOVER_MOTOR: f32 = 0.63;

/// `hover_thrust`, through the stabilizer's thrust curve and the mixer, lands both motors on
/// the measured hover throttle, with the elevons centered.
#[test]
fn explora_vtol_hover_thrust_reaches_the_measured_hover_motor() {
    let airframe = explora_vtol();
    let curved = airframe
        .guidance
        .hover
        .altitude
        .hover_thrust
        .powf(airframe.stabilizer.hover.thrust_curve_exponent);
    let mut mixer = airframe.mixer;
    let output = mixer.mix(&stable_flight::ControlOutput::new(curved, Vec3::zeros()));
    for motor in &output.motors[..2] {
        assert!(
            (motor - MEASURED_HOVER_MOTOR).abs() < 0.01,
            "motors {:?}",
            output.motors
        );
    }
    assert!(output.servos[..2] == [0.0, 0.0]);
}

/// Each airframe's thrust floor sits below its hover thrust, so waypoint flight can descend
/// and a land can touch down.
#[test]
fn thrust_floor_stays_below_hover() {
    for (name, guidance) in [
        ("quadcopter_x", quadcopter_x().guidance),
        ("explora_vtol", explora_vtol().guidance.hover),
        ("titan_cobra", titan_cobra().guidance.hover),
    ] {
        assert!(
            guidance.thrust.min_thrust < guidance.altitude.hover_thrust,
            "{name}: {guidance:?}"
        );
    }
}

/// The eXplora's tailsitter configs validate, and fit together: both sides share a thrust
/// curve so a handoff keeps the motor commands continuous, and the forward ramp can reach its
/// completion.
#[test]
fn explora_vtol_transition_fits_both_sides() {
    let airframe = explora_vtol();
    let (guidance, stabilizer) = (airframe.guidance, airframe.stabilizer);
    assert_eq!(guidance.validate(), Ok(()));
    assert_eq!(stabilizer.validate(), Ok(()));
    let transition = guidance.transition;
    assert_eq!(
        stabilizer.plane.thrust_curve_exponent,
        stabilizer.hover.thrust_curve_exponent
    );
    assert!(transition.forward_max_tilt_rad >= transition.forward_done_rad);
}

/// The Cobra's mapping drives each of its eight channels once: lift rotors, pusher, ailerons,
/// and the two A-tail panels (#433), none reversed, the tail panels trimmed +0.2.
/// The plane side is the sim eXplora's (stable-flight-sim `airframes::explora`), which #342
/// tuned the rate loop on: its rate gains, trim, airspeeds, and cruise.
#[test]
fn explora_vtol_flies_the_sim_exploras_plane_side() {
    let sim = stable_flight_sim::airframes::explora();
    let realflight = explora_vtol();

    assert_eq!(realflight.stabilizer.plane, sim.stabilizer.plane);
    assert_eq!(realflight.guidance.plane, sim.guidance.plane);
}

#[test]
fn titan_cobra_maps_each_channel_once() {
    let mapping = titan_cobra().mapping;

    assert_eq!(mapping.validate(), Ok(()));
    assert_eq!(
        mapping.motors,
        [
            Some(6),
            Some(4),
            Some(5),
            Some(7),
            Some(2),
            None,
            None,
            None
        ]
    );
    assert_eq!(mapping.servos[..3], [Some(0), Some(3), Some(1)]);
    assert!(mapping.servos[3..].iter().all(Option::is_none));
    assert!(mapping.reversed_servos.iter().all(|&r| !r));
    assert_eq!(
        mapping.servo_trims,
        [0.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    );
    let mut channels: Vec<usize> = mapping
        .motors
        .iter()
        .chain(&mapping.servos)
        .flatten()
        .copied()
        .collect();
    channels.sort_unstable();
    assert_eq!(channels, (0..8).collect::<Vec<_>>());
}

/// The Cobra's lift rotors mix as `quad_x` does at the Cobra's mixer config, so the copter
/// autotune flies them on `quad_x` with that config and the Cobra's mapping; in a hover (the quadplane stabilizer's zero cruise
/// output) the pusher is stopped and the surfaces centered.
#[test]
fn titan_cobra_lift_rotors_mix_as_quad_x() {
    let lift_only = stable_flight::ControlOutput::new(0.6, Vec3::new(0.2, -0.3, 0.1));
    let hover = stable_flight::ControlOutput {
        cruise: Some(stable_flight::CruiseOutput::default()),
        ..lift_only
    };
    let mut mixer = titan_cobra().mixer;
    let output = mixer.mix(&hover);
    let lift = TableMixer::quad_x()
        .with_config(mixer.config())
        .expect("the Cobra's config")
        .mix(&lift_only);
    assert_eq!(output.motors[..4], lift.motors[..4]);
    assert_eq!(output.motors[4], 0.0);
    assert_eq!(output.servos[..3], [0.0; 3]);
}

/// The Cobra's quadplane configs validate.
#[test]
fn titan_cobra_configs_validate() {
    let airframe = titan_cobra();
    assert_eq!(airframe.guidance.validate(), Ok(()));
    assert_eq!(airframe.stabilizer.validate(), Ok(()));
}

/// Steady nose-up moment (normalized) the Cobra needed to hover level in RealFlight at the
/// default thrust factor 0.9, whose unit moment moves each motor 0.1: front motors 0.60, rear
/// 0.52 (#433), and 0.376–0.378 over two autotunes (#438).
const MEASURED_HOVER_PITCH_MOMENT_AT_0_9: f32 = 0.37;

/// The pitch rate integrator can hold the Cobra's nose-heavy trim with 25% margin, the trim
/// rescaled to its mixer: the same motor split is a smaller moment where a unit moves the
/// motors further.
#[test]
fn titan_cobra_pitch_integrator_covers_the_hover_trim() {
    let airframe = titan_cobra();
    let headroom = 1.0 - airframe.mixer.config().thrust_factor;
    let trim = MEASURED_HOVER_PITCH_MOMENT_AT_0_9 * (1.0 - 0.9) / headroom;
    let body_rate = airframe.stabilizer.hover.body_rate;
    let authority = body_rate.pitch_gains.i * body_rate.max_integral;
    assert!(
        authority >= 1.25 * trim,
        "pitch I authority {authority}, trim {trim}"
    );
}

/// Pins the Cobra's flown hover set (#433), so a retune (or a core default change it
/// inherits) shows up in review.
#[test]
fn titan_cobra_hover_gains_are_the_flown_set() {
    let gains = |p, i, d| Gains { p, i, d };
    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            roll_gains: gains(1.08, 0.2, 0.027),
            pitch_gains: gains(0.5, 0.5, 0.004),
            yaw_gains: gains(0.6, 0.6, 0.0),
            max_integral: 1.0,
        },
        attitude: AttitudeControllerConfig {
            roll_gains: gains(3.0, 0.0, 0.1),
            pitch_gains: gains(3.0, 0.0, 0.1),
            max_integral: 100.0,
            max_angles_rad: Vec2::new(PI / 4.0, PI / 4.0),
            max_rate_radps: Vec2::new(3.0, 3.0),
        },
        yaw: YawControllerConfig {
            gains: gains(2.5, 0.0, 0.06),
            max_integral: 100.0,
            max_rate_radps: 2.0,
        },
        full_scale_rates_radps: Vec3::new(3.0, 3.0, 3.0),
        thrust_curve_exponent: 2.0,
    };
    assert_eq!(titan_cobra().stabilizer.hover, stabilizer);
}

/// The Cobra's lift rotors hover on the motor command they flew at, 0.9 · 0.79² (#433), now
/// with twice the pitch range: a unit pitch moment moves each lift rotor 0.2 where it moved
/// 0.1 (#438).
#[test]
fn titan_cobra_hovers_on_its_flown_command_with_twice_the_pitch_range() {
    use stable_flight::{ControlOutput, Mixer, Vec3};
    let airframe = titan_cobra();
    let mut mixer = airframe.mixer;
    let collective = airframe
        .guidance
        .hover
        .altitude
        .hover_thrust
        .powf(airframe.stabilizer.hover.thrust_curve_exponent);
    let mut lift_rotors = |pitch| {
        let motors = mixer
            .mix(&ControlOutput {
                thrust: collective,
                moments: Vec3::new(0.0, pitch, 0.0),
                ..Default::default()
            })
            .motors;
        [motors[0], motors[1], motors[2], motors[3]]
    };
    let level = lift_rotors(0.0);
    let nose_up = lift_rotors(1.0);
    for (motor, (at_level, pitched)) in level.iter().zip(&nose_up).enumerate() {
        assert!(
            (at_level - 0.9 * 0.79 * 0.79).abs() < 1e-5,
            "motor {motor}: {at_level}"
        );
        assert!(
            ((pitched - at_level).abs() - 0.2).abs() < 1e-5,
            "motor {motor}: {at_level} -> {pitched}"
        );
    }
}

/// The Cobra's forward lift ramp starts at its ArduPilot minimum airspeed, `ARSPD_FBW_MIN`
/// 11, which is also its plane side's minimum.
#[test]
fn titan_cobra_transitions_at_its_minimum_airspeed() {
    let airframe = titan_cobra();
    assert_eq!(airframe.guidance.transition.min_airspeed_mps, 11.0);
    assert_eq!(airframe.stabilizer.plane.min_airspeed_mps, 11.0);
}

/// The Cobra's pusher commands are those of its flights (#433): `TRIM_THROTTLE` 47 at cruise,
/// 0.9 at the forward transition's cap, and forward thrust 0.216 at `tan(pitch)` 0.2, the gain
/// that held a 100 m copter leg within 0.5 m of altitude where 0 lost 4.7 m.
#[test]
fn titan_cobra_pusher_commands_are_as_flown() {
    use stable_flight::{ControlOutput, CruiseOutput, Mixer, Vec3};
    let airframe = titan_cobra();
    let mut mixer = airframe.mixer;
    let mut pusher = |thrust| {
        let cruise = CruiseOutput {
            thrust,
            moments: Vec3::zeros(),
        };
        mixer
            .mix(&ControlOutput {
                cruise: Some(cruise),
                ..Default::default()
            })
            .motors[4]
    };
    let guidance = airframe.guidance;
    // (what, pusher command, as flown)
    let cases = [
        ("cruise", pusher(guidance.plane.cruise_thrust), 0.47),
        (
            "transition",
            pusher(guidance.transition.forward_thrust),
            0.9,
        ),
        ("forward thrust", pusher(0.2 * guidance.forward.gain), 0.216),
    ];
    for (what, command, expected) in cases {
        assert!((command - expected).abs() < 1e-4, "{what}: {command}");
    }
}
