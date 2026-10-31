use assert2::check;

use super::*;
use crate::airframes;

#[test]
fn the_plane_airframe_is_valid() {
    check!(airframes::plane().model.validate() == Ok(()));
}

/// Each class of parameter is checked: positive, non-negative, finite, and the motor delay's
/// ceiling.
#[test]
fn rejects_each_out_of_range_parameter() {
    let model = airframes::plane().model;
    // (case, edit, error)
    type Edit = fn(&mut FixedWingModel);
    let cases: [(&str, Edit, ModelError); 9] = [
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
