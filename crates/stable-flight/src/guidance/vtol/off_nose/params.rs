//! [`OffNoseSpeedConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::OffNoseSpeedConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for OffNoseSpeedConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("OFFNOSE_SPD", speed_mps),
        param!(deg "OFFNOSE_FREE", free_rad),
        param!(deg "OFFNOSE_FULL", full_rad),
        param!("OFFNOSE_DIV", tail_first_divisor),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::OffNoseSpeedConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const OFF_NOSE: ParamMeta = ParamMeta::in_group("Guidance", "Off-nose speed");

    impl ParamMetadata for OffNoseSpeedConfig {
        const METADATA: &'static [ParamMeta] = &[
            OFF_NOSE
                .param(
                    "OFFNOSE_SPD",
                    "Copter-flight speed cap just off the nose; divided tail first (0: off)",
                )
                .with_units("m/s")
                .at_least(0.0),
            OFF_NOSE
                .param("OFFNOSE_FREE", "Heading error the speed isn't cut within")
                .with_units("deg")
                .at_least(0.0)
                .below(180.0),
            OFF_NOSE
                .param("OFFNOSE_FULL", "Heading error from which the cut is whole")
                .with_units("deg")
                .above(0.0)
                .at_most(180.0),
            OFF_NOSE
                .param("OFFNOSE_DIV", "What the speed is divided by tail first")
                .at_least(1.0),
        ];
    }
}
