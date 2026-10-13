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
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::PlaneGuidanceConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const STICKS: ParamMeta = ParamMeta::in_group("Plane Guidance", "Sticks");
    const CRUISE: ParamMeta = ParamMeta::in_group("Plane Guidance", "Cruise");

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
                .param("PGUID_CRUISE", "Throttle for wings-level cruise")
                .at_least(0.0)
                .at_most(1.0),
        ];
    }
}
