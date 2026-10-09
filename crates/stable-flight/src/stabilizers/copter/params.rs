//! [`CopterStabilizerConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::CopterStabilizerConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for CopterStabilizerConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("STAB_RATE_P", body_rate.gains.p),
        param!("STAB_RATE_I", body_rate.gains.i),
        param!("STAB_RATE_D", body_rate.gains.d),
        param!("STAB_RATE_YAW_P", body_rate.yaw_gains.p),
        param!("STAB_RATE_YAW_I", body_rate.yaw_gains.i),
        param!("STAB_RATE_YAW_D", body_rate.yaw_gains.d),
        param!("STAB_RATE_IMAX", body_rate.max_integral),
        param!("STAB_ATT_P", attitude.gains.p),
        param!("STAB_ATT_I", attitude.gains.i),
        param!("STAB_ATT_D", attitude.gains.d),
        param!(deg "STAB_ATT_AMAX_X", attitude.max_angles_rad.x),
        param!(deg "STAB_ATT_AMAX_Y", attitude.max_angles_rad.y),
        param!("STAB_ATT_IMAX", attitude.max_integral),
        param!(deg "STAB_ATT_RMAX_X", attitude.max_rate_radps.x),
        param!(deg "STAB_ATT_RMAX_Y", attitude.max_rate_radps.y),
        param!("STAB_YAW_P", yaw.gains.p),
        param!("STAB_YAW_I", yaw.gains.i),
        param!("STAB_YAW_D", yaw.gains.d),
        param!("STAB_YAW_IMAX", yaw.max_integral),
        param!(deg "STAB_YAW_RMAX", yaw.max_rate_radps),
        param!(deg "STAB_FS_RATE_X", full_scale_rates_radps.x),
        param!(deg "STAB_FS_RATE_Y", full_scale_rates_radps.y),
        param!(deg "STAB_FS_RATE_Z", full_scale_rates_radps.z),
        param!("STAB_THR_EXPO", thrust_curve_exponent),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::CopterStabilizerConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const RATE: ParamMeta = ParamMeta::in_group("Stabilizer", "Rate");
    const ATTITUDE: ParamMeta = ParamMeta::in_group("Stabilizer", "Attitude");
    const YAW: ParamMeta = ParamMeta::in_group("Stabilizer", "Yaw");
    const STICKS: ParamMeta = ParamMeta::in_group("Stabilizer", "Sticks");
    const THRUST: ParamMeta = ParamMeta::in_group("Stabilizer", "Thrust");

    impl ParamMetadata for CopterStabilizerConfig {
        const METADATA: &'static [ParamMeta] = &[
            RATE.param("STAB_RATE_P", "Roll/pitch rate loop proportional gain"),
            RATE.param("STAB_RATE_I", "Roll/pitch rate loop integral gain"),
            RATE.param("STAB_RATE_D", "Roll/pitch rate loop derivative gain"),
            RATE.param("STAB_RATE_YAW_P", "Yaw rate loop proportional gain"),
            RATE.param("STAB_RATE_YAW_I", "Yaw rate loop integral gain"),
            RATE.param("STAB_RATE_YAW_D", "Yaw rate loop derivative gain"),
            RATE.param("STAB_RATE_IMAX", "Rate loop integral limit, per axis")
                .with_units("rad")
                .at_least(0.0),
            ATTITUDE.param("STAB_ATT_P", "Roll/pitch angle loop proportional gain"),
            ATTITUDE.param("STAB_ATT_I", "Roll/pitch angle loop integral gain"),
            ATTITUDE.param("STAB_ATT_D", "Roll/pitch angle loop derivative gain"),
            ATTITUDE
                .param("STAB_ATT_AMAX_X", "Largest roll angle")
                .with_units("deg")
                .at_least(0.0),
            ATTITUDE
                .param("STAB_ATT_AMAX_Y", "Largest pitch angle")
                .with_units("deg")
                .at_least(0.0),
            ATTITUDE
                .param("STAB_ATT_IMAX", "Angle loop integral limit, per axis")
                .with_units("rad*s")
                .at_least(0.0),
            ATTITUDE
                .param(
                    "STAB_ATT_RMAX_X",
                    "Largest roll rate the angle loop commands",
                )
                .with_units("deg/s")
                .at_least(0.0),
            ATTITUDE
                .param(
                    "STAB_ATT_RMAX_Y",
                    "Largest pitch rate the angle loop commands",
                )
                .with_units("deg/s")
                .at_least(0.0),
            YAW.param("STAB_YAW_P", "Heading-hold loop proportional gain"),
            YAW.param("STAB_YAW_I", "Heading-hold loop integral gain"),
            YAW.param("STAB_YAW_D", "Heading-hold loop derivative gain"),
            YAW.param("STAB_YAW_IMAX", "Heading-hold loop integral limit")
                .with_units("rad*s")
                .at_least(0.0),
            YAW.param("STAB_YAW_RMAX", "Largest yaw rate heading hold commands")
                .with_units("deg/s")
                .at_least(0.0),
            STICKS
                .param("STAB_FS_RATE_X", "Roll rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            STICKS
                .param("STAB_FS_RATE_Y", "Pitch rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            STICKS
                .param("STAB_FS_RATE_Z", "Yaw rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            THRUST
                .param("STAB_THR_EXPO", "Thrust curve exponent (1 is linear)")
                .above(0.0),
        ];
    }
}
