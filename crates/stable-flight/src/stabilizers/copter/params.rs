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
