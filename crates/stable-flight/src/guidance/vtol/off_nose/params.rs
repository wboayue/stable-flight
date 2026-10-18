//! [`OffNoseSpeedConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::OffNoseSpeedConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for OffNoseSpeedConfig {
    const PARAMS: &'static [Param<Self>] = &[param!("OFFNOSE_SPD", speed_mps)];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::OffNoseSpeedConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const OFF_NOSE: ParamMeta = ParamMeta::in_group("Guidance", "Off-nose speed");

    impl ParamMetadata for OffNoseSpeedConfig {
        const METADATA: &'static [ParamMeta] = &[OFF_NOSE
            .param(
                "OFFNOSE_SPD",
                "Copter-flight speed cap just off the nose; a third tail first (0: off)",
            )
            .with_units("m/s")
            .at_least(0.0)];
    }
}
