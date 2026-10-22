//! [`PlaneStabilizerConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::PlaneStabilizerConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for PlaneStabilizerConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("PSTAB_RLL_P", rate.roll.p),
        param!("PSTAB_RLL_I", rate.roll.i),
        param!("PSTAB_RLL_FF", rate.roll.ff),
        param!("PSTAB_PIT_P", rate.pitch.p),
        param!("PSTAB_PIT_I", rate.pitch.i),
        param!("PSTAB_PIT_FF", rate.pitch.ff),
        param!("PSTAB_YAW_P", rate.yaw.p),
        param!("PSTAB_YAW_I", rate.yaw.i),
        param!("PSTAB_YAW_FF", rate.yaw.ff),
        param!("PSTAB_IMAX", rate.max_integral),
        param!("PSTAB_RLL_TRIM", rate.trim.x),
        param!("PSTAB_PIT_TRIM", rate.trim.y),
        param!("PSTAB_YAW_TRIM", rate.trim.z),
        param!("PSTAB_TC_X", time_constant_s.x),
        param!("PSTAB_TC_Y", time_constant_s.y),
        param!(deg "PSTAB_RMAX_X", max_attitude_rate_radps.x),
        param!(deg "PSTAB_RMAX_Y", max_attitude_rate_radps.y),
        param!(deg "PSTAB_BRATE_X", max_body_rate_radps.x),
        param!(deg "PSTAB_BRATE_Y", max_body_rate_radps.y),
        param!(deg "PSTAB_BRATE_Z", max_body_rate_radps.z),
        param!(deg "PSTAB_BANK_MAX", max_bank_rad),
        param!(deg "PSTAB_PIT_MAX", max_pitch_rad),
        param!("PSTAB_ASPD_TRIM", trim_airspeed_mps),
        param!("PSTAB_ASPD_MIN", min_airspeed_mps),
        param!("PSTAB_ASPD_MAX", max_airspeed_mps),
        param!("PSTAB_THR_EXPO", thrust_curve_exponent),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::PlaneStabilizerConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const RATE: ParamMeta = ParamMeta::in_group("Plane Stabilizer", "Rate");
    const ATTITUDE: ParamMeta = ParamMeta::in_group("Plane Stabilizer", "Attitude");
    const AIRSPEED: ParamMeta = ParamMeta::in_group("Plane Stabilizer", "Airspeed");
    const THRUST: ParamMeta = ParamMeta::in_group("Plane Stabilizer", "Thrust");

    /// A rate-loop gain: non-negative.
    const fn gain(name: &'static str, description: &'static str) -> ParamMeta {
        RATE.param(name, description).at_least(0.0)
    }

    /// A rate-loop trim: a normalized moment.
    const fn trim(name: &'static str, description: &'static str) -> ParamMeta {
        RATE.param(name, description).at_least(-1.0).at_most(1.0)
    }

    impl ParamMetadata for PlaneStabilizerConfig {
        const METADATA: &'static [ParamMeta] = &[
            gain("PSTAB_RLL_P", "Roll rate proportional gain"),
            gain("PSTAB_RLL_I", "Roll rate integral gain"),
            gain("PSTAB_RLL_FF", "Roll rate feedforward gain"),
            gain("PSTAB_PIT_P", "Pitch rate proportional gain"),
            gain("PSTAB_PIT_I", "Pitch rate integral gain"),
            gain("PSTAB_PIT_FF", "Pitch rate feedforward gain"),
            gain("PSTAB_YAW_P", "Yaw rate proportional gain"),
            gain("PSTAB_YAW_I", "Yaw rate integral gain"),
            gain("PSTAB_YAW_FF", "Yaw rate feedforward gain"),
            RATE.param("PSTAB_IMAX", "Rate loop integral limit, per axis")
                .with_units("rad")
                .at_least(0.0),
            trim(
                "PSTAB_RLL_TRIM",
                "Roll moment at trim airspeed and zero rate error",
            ),
            trim(
                "PSTAB_PIT_TRIM",
                "Pitch moment at trim airspeed and zero rate error",
            ),
            trim(
                "PSTAB_YAW_TRIM",
                "Yaw moment at trim airspeed and zero rate error",
            ),
            ATTITUDE
                .param("PSTAB_TC_X", "Roll attitude time constant")
                .with_units("s")
                .above(0.0),
            ATTITUDE
                .param("PSTAB_TC_Y", "Pitch attitude time constant")
                .with_units("s")
                .above(0.0),
            ATTITUDE
                .param(
                    "PSTAB_RMAX_X",
                    "Largest roll rate the attitude loop asks for",
                )
                .with_units("deg/s")
                .at_least(0.0),
            ATTITUDE
                .param(
                    "PSTAB_RMAX_Y",
                    "Largest pitch rate the attitude loop asks for",
                )
                .with_units("deg/s")
                .at_least(0.0),
            RATE.param("PSTAB_BRATE_X", "Largest commanded roll rate (acro)")
                .with_units("deg/s")
                .at_least(0.0),
            RATE.param("PSTAB_BRATE_Y", "Largest commanded pitch rate (acro)")
                .with_units("deg/s")
                .at_least(0.0),
            RATE.param("PSTAB_BRATE_Z", "Largest commanded yaw rate (acro)")
                .with_units("deg/s")
                .at_least(0.0),
            ATTITUDE
                .param("PSTAB_BANK_MAX", "Largest bank an attitude setpoint gets")
                .with_units("deg")
                .at_least(0.0)
                .at_most(180.0),
            ATTITUDE
                .param("PSTAB_PIT_MAX", "Largest pitch an attitude setpoint gets")
                .with_units("deg")
                .at_least(0.0)
                .at_most(180.0),
            AIRSPEED
                .param("PSTAB_ASPD_TRIM", "Airspeed the gains are tuned at")
                .with_units("m/s")
                .above(0.0),
            AIRSPEED
                .param(
                    "PSTAB_ASPD_MIN",
                    "Airspeed below which scaling and integrals hold",
                )
                .with_units("m/s")
                .above(0.0),
            AIRSPEED
                .param("PSTAB_ASPD_MAX", "Airspeed above which scaling holds")
                .with_units("m/s")
                .above(0.0),
            THRUST
                .param(
                    "PSTAB_THR_EXPO",
                    "Throttle to motor command exponent (1: linear)",
                )
                .above(0.0),
        ];
    }
}
