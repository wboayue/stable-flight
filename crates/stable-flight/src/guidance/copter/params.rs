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
        param!("GUID_DESC_MAX", max_descent_mps),
        param!("GUID_DESC_ACC", descent_accel_mps2),
        param!("GUID_LAND_SPD", land_speed_mps),
        param!("GUID_THR_MIN", thrust.min_thrust),
        param!("GUID_SINK_LIM", thrust.sink_limit_mps),
        param!("GUID_SINK_GAIN", thrust.sink_gain),
        param!("GUID_BOOST_MAX", thrust.max_boost_thrust),
        param!(deg "GUID_STICK_X", max_stick_angles_rad.x),
        param!(deg "GUID_STICK_Y", max_stick_angles_rad.y),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::CopterGuidanceConfig;
    use crate::commanders::touchdown::DESCENDING_MPS;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const ALTITUDE: ParamMeta = ParamMeta::in_group("Guidance", "Altitude");
    const LATERAL: ParamMeta = ParamMeta::in_group("Guidance", "Lateral");
    const NAVIGATION: ParamMeta = ParamMeta::in_group("Guidance", "Navigation");
    const LIMITS: ParamMeta = ParamMeta::in_group("Guidance", "Limits");
    const LANDING: ParamMeta = ParamMeta::in_group("Guidance", "Landing");
    const STICKS: ParamMeta = ParamMeta::in_group("Guidance", "Sticks");

    impl ParamMetadata for CopterGuidanceConfig {
        const METADATA: &'static [ParamMeta] = &[
            ALTITUDE.param("GUID_ALT_P", "Altitude loop proportional gain"),
            ALTITUDE.param("GUID_ALT_I", "Altitude loop integral gain"),
            ALTITUDE.param("GUID_ALT_D", "Altitude loop derivative gain"),
            ALTITUDE
                .param("GUID_ALT_IMAX", "Altitude loop integral limit")
                .with_units("m*s")
                .at_least(0.0),
            ALTITUDE
                .param("GUID_ALT_HOVER", "Linear thrust that holds a level hover")
                .above(0.0)
                .at_most(1.0),
            LATERAL
                .param("GUID_LAT_POS_P", "Position error to velocity gain")
                .with_units("1/s")
                .at_least(0.0),
            LATERAL.param("GUID_LAT_VEL_P", "Velocity loop proportional gain"),
            LATERAL.param("GUID_LAT_VEL_I", "Velocity loop integral gain"),
            LATERAL.param("GUID_LAT_VEL_D", "Velocity loop derivative gain"),
            LATERAL
                .param("GUID_LAT_IMAX", "Velocity loop integral limit, per axis")
                .with_units("m")
                .at_least(0.0),
            NAVIGATION
                .param(
                    "GUID_NAV_PERIOD",
                    "Time between lateral position loop runs (0: every tick)",
                )
                .with_units("s")
                .at_least(0.0),
            LIMITS
                .param("GUID_TILT_MAX", "Largest tilt waypoint flight requests")
                .with_units("deg")
                .above(0.0)
                .below(90.0),
            LIMITS
                .param("GUID_DESC_MAX", "Fastest descent waypoint flight requests")
                .with_units("m/s")
                .above(0.0),
            LIMITS
                .param(
                    "GUID_DESC_ACC",
                    "Deceleration a waypoint descent stops on the waypoint with",
                )
                .with_units("m/s/s")
                .above(0.0),
            LANDING
                .param("GUID_LAND_SPD", "Descent rate while landing")
                .with_units("m/s")
                .above(DESCENDING_MPS),
            LIMITS
                .param("GUID_THR_MIN", "Least thrust while flying (0: none)")
                .at_least(0.0)
                .below(1.0),
            LIMITS
                .param(
                    "GUID_SINK_LIM",
                    "Sink rate past which piloted flight gets more thrust (0: off)",
                )
                .with_units("m/s")
                .at_least(0.0),
            LIMITS
                .param(
                    "GUID_SINK_GAIN",
                    "Thrust added per m/s of sink past the limit",
                )
                .with_units("s/m")
                .at_least(0.0),
            STICKS
                .param(
                    "GUID_BOOST_MAX",
                    "Most thrust the attitude-mode tilt boost gives",
                )
                .above(0.0)
                .at_most(1.0),
            STICKS
                .param("GUID_STICK_X", "Roll angle at full stick in attitude mode")
                .with_units("deg")
                .at_least(0.0),
            STICKS
                .param("GUID_STICK_Y", "Pitch angle at full stick in attitude mode")
                .with_units("deg")
                .at_least(0.0),
        ];
    }
}
