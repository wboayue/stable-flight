//! [`PlaneGuidanceConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::PlaneGuidanceConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for PlaneGuidanceConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!(deg "PGUID_BANK_MAX", max_bank_rad),
        param!(deg "PGUID_PIT_UP", max_pitch_up_rad),
        param!(deg "PGUID_PIT_DN", max_pitch_down_rad),
        param!(deg "PGUID_ACRO_X", max_rate_radps.x),
        param!(deg "PGUID_ACRO_Y", max_rate_radps.y),
        param!(deg "PGUID_ACRO_Z", max_rate_radps.z),
        param!(deg "PGUID_TRIM_PIT", trim_pitch_rad),
        param!("PGUID_CRUISE", cruise_thrust),
        param!("PGUID_L1_PERIOD", l1_period_s),
        param!("PGUID_L1_DAMP", l1_damping),
        param!("PGUID_LOIT_RAD", loiter_radius_m),
        param!("PGUID_ALT_P", altitude_p),
        param!("PGUID_ALT_I", altitude_i),
        param!("PGUID_ALT_IMAX", altitude_max_integral),
        param!("PGUID_SPD_P", speed_p),
        param!("PGUID_SPD_I", speed_i),
        param!("PGUID_SPD_IMAX", speed_max_integral),
        param!("PGUID_THR_MIN", min_thrust),
        param!("PGUID_THR_MAX", max_thrust),
        param!("PGUID_CLMB_THR", climb_thrust_per_rad),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::PlaneGuidanceConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const STICKS: ParamMeta = ParamMeta::in_group("Plane Guidance", "Sticks");
    const CRUISE: ParamMeta = ParamMeta::in_group("Plane Guidance", "Cruise");
    const NAVIGATION: ParamMeta = ParamMeta::in_group("Plane Guidance", "Navigation");

    /// A non-negative gain or limit.
    const fn at_least_zero(meta: ParamMeta) -> ParamMeta {
        meta.at_least(0.0)
    }

    impl ParamMetadata for PlaneGuidanceConfig {
        const METADATA: &'static [ParamMeta] = &[
            STICKS
                .param("PGUID_BANK_MAX", "Bank at full roll stick")
                .with_units("deg")
                .at_least(0.0)
                .at_most(90.0),
            STICKS
                .param("PGUID_PIT_UP", "Pitch up at full back stick")
                .with_units("deg")
                .at_least(0.0)
                .at_most(90.0),
            STICKS
                .param("PGUID_PIT_DN", "Pitch down at full forward stick")
                .with_units("deg")
                .at_least(0.0)
                .at_most(90.0),
            STICKS
                .param("PGUID_ACRO_X", "Roll rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            STICKS
                .param("PGUID_ACRO_Y", "Pitch rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            STICKS
                .param("PGUID_ACRO_Z", "Yaw rate at full stick in rate mode")
                .with_units("deg/s")
                .at_least(0.0),
            CRUISE
                .param("PGUID_TRIM_PIT", "Pitch for wings-level cruise")
                .with_units("deg")
                .at_least(-90.0)
                .at_most(90.0),
            CRUISE
                .param(
                    "PGUID_CRUISE",
                    "Throttle for cruise: the airspeed loop's feedforward",
                )
                .at_least(0.0)
                .at_most(1.0),
            NAVIGATION
                .param(
                    "PGUID_L1_PERIOD",
                    "L1 period: shorter captures the path tighter",
                )
                .with_units("s")
                .above(0.0),
            NAVIGATION
                .param("PGUID_L1_DAMP", "L1 damping ratio")
                .above(0.0),
            NAVIGATION
                .param(
                    "PGUID_LOIT_RAD",
                    "Loiter radius around a waypoint with no leg",
                )
                .with_units("m")
                .above(0.0),
            at_least_zero(
                NAVIGATION
                    .param("PGUID_ALT_P", "Pitch per meter of altitude error")
                    .with_units("rad/m"),
            ),
            at_least_zero(NAVIGATION.param("PGUID_ALT_I", "Altitude loop integral gain")),
            at_least_zero(
                NAVIGATION
                    .param("PGUID_ALT_IMAX", "Altitude loop integral limit")
                    .with_units("m*s"),
            ),
            at_least_zero(NAVIGATION.param("PGUID_SPD_P", "Thrust per m/s of airspeed error")),
            at_least_zero(NAVIGATION.param("PGUID_SPD_I", "Airspeed loop integral gain")),
            at_least_zero(
                NAVIGATION
                    .param("PGUID_SPD_IMAX", "Airspeed loop integral limit")
                    .with_units("m"),
            ),
            NAVIGATION
                .param("PGUID_THR_MIN", "Least thrust in waypoint flight")
                .at_least(0.0)
                .at_most(1.0),
            NAVIGATION
                .param("PGUID_THR_MAX", "Most thrust in waypoint flight")
                .at_least(0.0)
                .at_most(1.0),
            at_least_zero(NAVIGATION.param(
                "PGUID_CLMB_THR",
                "Without airspeed: thrust added per radian of climb pitch",
            )),
        ];
    }
}
