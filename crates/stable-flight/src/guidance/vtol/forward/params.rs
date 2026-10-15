//! [`ForwardThrustConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::ForwardThrustConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for ForwardThrustConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("FWD_THR_GAIN", gain),
        param!(deg "FWD_PIT_LIM", pitch_limit_rad),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::ForwardThrustConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const FORWARD: ParamMeta = ParamMeta::in_group("Guidance", "Forward thrust");

    impl ParamMetadata for ForwardThrustConfig {
        const METADATA: &'static [ParamMeta] = &[
            FORWARD
                .param(
                    "FWD_THR_GAIN",
                    "Pusher thrust per tan of hover nose-down pitch (0: off)",
                )
                .at_least(0.0),
            FORWARD
                .param("FWD_PIT_LIM", "Most nose-down pitch with forward thrust on")
                .with_units("deg")
                .at_least(0.0)
                .below(90.0),
        ];
    }
}
