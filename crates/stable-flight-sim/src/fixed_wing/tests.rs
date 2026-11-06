use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::airframes;

#[test]
fn the_plane_airframe_is_valid() {
    check!(airframes::plane().model.validate() == Ok(()));
}

/// Each class of parameter is checked: positive, non-negative, a fraction, finite, and the motor
/// delay's ceiling.
#[test]
fn rejects_each_out_of_range_parameter() {
    let model = airframes::plane().model;
    // (case, edit, error)
    type Edit = fn(&mut FixedWingModel);
    let cases: [(&str, Edit, ModelError); 17] = [
        (
            "zero span",
            |m| m.span_m = 0.0,
            ModelError::NotPositive("span_m"),
        ),
        (
            "negative inertia",
            |m| m.inertia_kgm2.y = -1.0,
            ModelError::NotPositive("inertia_kgm2.y"),
        ),
        (
            "zero stall angle",
            |m| m.aero.alpha_stall_rad = 0.0,
            ModelError::NotPositive("aero.alpha_stall_rad"),
        ),
        (
            "NaN servo lag",
            |m| m.servo_time_constant_s = f32::NAN,
            ModelError::NotPositive("servo_time_constant_s"),
        ),
        (
            "negative deflection",
            |m| m.max_deflection_rad.z = -0.1,
            ModelError::Negative("max_deflection_rad.z"),
        ),
        (
            "negative drag",
            |m| m.aero.c_drag_0 = -0.1,
            ModelError::Negative("aero.c_drag_0"),
        ),
        (
            "negative linear falloff",
            |m| m.thrust_falloff = ThrustFalloff::Linear { n_per_mps: -1.0 },
            ModelError::Negative("thrust_falloff.n_per_mps"),
        ),
        (
            "zero pitch speed",
            |m| m.thrust_falloff = ThrustFalloff::PitchSpeed { full_mps: 0.0 },
            ModelError::NotPositive("thrust_falloff.full_mps"),
        ),
        (
            "negative motor delay",
            |m| m.motor_delay_s = -0.01,
            ModelError::Negative("motor_delay_s"),
        ),
        (
            "motor delay past the ceiling",
            |m| m.motor_delay_s = 2.0 * MAX_ACTUATOR_DELAY_S,
            ModelError::DelayTooLong("motor_delay_s"),
        ),
        (
            "post-stall pitch share past 1",
            |m| m.aero.post_stall_pitch_share = 1.5,
            ModelError::NotFraction("aero.post_stall_pitch_share"),
        ),
        (
            "NaN surface stall share",
            |m| m.aero.surface_stall_share = f32::NAN,
            ModelError::NotFraction("aero.surface_stall_share"),
        ),
        (
            "zero pitch trim knee",
            |m| {
                m.aero.c_pitch_0_knee = Some(PitchTrimKnee {
                    knee_mps: 0.0,
                    ..TRIM_KNEE
                })
            },
            ModelError::NotPositive("aero.c_pitch_0_knee.knee_mps"),
        ),
        (
            "zero pitch trim floor",
            |m| {
                m.aero.c_pitch_0_knee = Some(PitchTrimKnee {
                    floor_mps: 0.0,
                    ..TRIM_KNEE
                })
            },
            ModelError::NotPositive("aero.c_pitch_0_knee.floor_mps"),
        ),
        (
            "NaN pitch trim exponent",
            |m| {
                m.aero.c_pitch_0_knee = Some(PitchTrimKnee {
                    exponent: f32::NAN,
                    ..TRIM_KNEE
                })
            },
            ModelError::NotFinite("aero.c_pitch_0_knee.exponent"),
        ),
        (
            "NaN pitch trim low exponent",
            |m| {
                m.aero.c_pitch_0_knee = Some(PitchTrimKnee {
                    low_exponent: f32::NAN,
                    ..TRIM_KNEE
                })
            },
            ModelError::NotFinite("aero.c_pitch_0_knee.low_exponent"),
        ),
        (
            "infinite coefficient",
            |m| m.aero.c_yaw_r = f32::INFINITY,
            ModelError::NotFinite("aero.c_yaw_r"),
        ),
    ];
    for (case, edit, error) in cases {
        let mut edited = model;
        edit(&mut edited);
        check!(edited.validate() == Err(error), "{case}");
    }
}

/// A prop's thrust falls to zero at its pitch speed, which slows with the motor: at a quarter
/// of full thrust (half the rpm) it stops pushing at half the full-command speed. Linear falloff
/// stops at the same speed at any command.
/// A valid knee for [`rejects_each_out_of_range_parameter`] to break one field of.
const TRIM_KNEE: PitchTrimKnee = PitchTrimKnee {
    knee_mps: 14.0,
    exponent: 1.0,
    low_exponent: 0.5,
    floor_mps: 7.0,
};

#[test]
fn thrust_falls_off_with_airspeed_by_the_falloff_law() {
    let pitch_speed = FixedWingModel {
        thrust_falloff: ThrustFalloff::PitchSpeed { full_mps: 30.0 },
        ..airframes::plane().model
    };
    let linear = FixedWingModel {
        thrust_falloff: ThrustFalloff::Linear {
            n_per_mps: pitch_speed.max_thrust_n / 30.0,
        },
        ..pitch_speed
    };
    // (case, model, thrust share, airspeed m/s, share of static thrust left)
    let cases = [
        ("pitch speed, still air", pitch_speed, 0.25, 0.0, 1.0),
        ("pitch speed, flying backward", pitch_speed, 0.25, -5.0, 1.0),
        ("pitch speed, full, halfway", pitch_speed, 1.0, 15.0, 0.5),
        (
            "pitch speed, quarter, a quarter",
            pitch_speed,
            0.25,
            3.75,
            0.75,
        ),
        (
            "pitch speed, quarter, at its speed",
            pitch_speed,
            0.25,
            15.0,
            0.0,
        ),
        (
            "pitch speed, quarter, past it",
            pitch_speed,
            0.25,
            20.0,
            0.0,
        ),
        ("pitch speed, stopped", pitch_speed, 0.0, 5.0, 0.0),
        (
            "pitch speed, stopped, still air",
            pitch_speed,
            0.0,
            0.0,
            1.0,
        ),
        ("linear, full, halfway", linear, 1.0, 15.0, 0.5),
        ("linear, quarter, halfway", linear, 0.25, 15.0, 0.5),
        ("linear, past its speed", linear, 0.25, 40.0, 0.0),
    ];
    for (case, model, thrust_share, airspeed_mps, left) in cases {
        let share = model.falloff_share(thrust_share, airspeed_mps);
        check!(!share.is_nan(), "{case}");
        assert_relative_eq!(share, left, epsilon = 1e-6);
    }
    assert_relative_eq!(
        pitch_speed.full_thrust_n(15.0),
        0.5 * pitch_speed.max_thrust_n
    );
}
