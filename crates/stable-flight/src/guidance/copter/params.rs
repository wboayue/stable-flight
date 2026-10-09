//! [`CopterGuidanceConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::CopterGuidanceConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for CopterGuidanceConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("GUID_ALT_P", altitude.gains.p),
        param!("GUID_ALT_I", altitude.gains.i),
        param!("GUID_ALT_D", altitude.gains.d),
        param!("GUID_ALT_IMAX", altitude.max_integral),
        param!("GUID_ALT_HOVER", altitude.hover_thrust),
        param!("GUID_LAT_POS_P", lateral.position_p),
        param!("GUID_LAT_VEL_P", lateral.velocity.p),
        param!("GUID_LAT_VEL_I", lateral.velocity.i),
        param!("GUID_LAT_VEL_D", lateral.velocity.d),
        param!("GUID_LAT_IMAX", lateral.max_integral),
        param!("GUID_NAV_PERIOD", navigation_period_s),
        param!(deg "GUID_TILT_MAX", max_tilt_rad),
        param!(deg "GUID_STICK_X", max_stick_angles_rad.x),
        param!(deg "GUID_STICK_Y", max_stick_angles_rad.y),
    ];
}
