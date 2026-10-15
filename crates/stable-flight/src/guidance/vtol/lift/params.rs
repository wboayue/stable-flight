//! [`LiftTransitionConfig`]'s params; the names and units are tabled in [`crate::params`].
//! Fields that mean what a [`TransitionConfig`](super::super::TransitionConfig) field means
//! share its name.

use super::LiftTransitionConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for LiftTransitionConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!("TRAN_ASPD_MIN", min_airspeed_mps),
        param!("TRAN_FW_RAMP", lift_ramp_s),
        param!("TRAN_TIMEOUT", timeout_s),
        param!("TRAN_MIN_HGT", min_height_m),
        param!("TRAN_VT_RAMP", back_ramp_s),
        param!("TRAN_VT_SPD", back_speed_mps),
        param!("TRAN_VT_TIMEOUT", back_timeout_s),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::LiftTransitionConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const FORWARD: ParamMeta = ParamMeta::in_group("Transition", "Forward");
    const BACK: ParamMeta = ParamMeta::in_group("Transition", "Back");
    const LIMITS: ParamMeta = ParamMeta::in_group("Transition", "Limits");

    impl ParamMetadata for LiftTransitionConfig {
        const METADATA: &'static [ParamMeta] = &[
            FORWARD
                .param("TRAN_ASPD_MIN", "Airspeed that starts the lift ramp-down")
                .with_units("m/s")
                .at_least(0.0),
            FORWARD
                .param(
                    "TRAN_FW_RAMP",
                    "Time the lift rotors take to hand the lift to the wing",
                )
                .with_units("s")
                .above(0.0),
            LIMITS
                .param(
                    "TRAN_TIMEOUT",
                    "Longest forward transition before it reverts",
                )
                .with_units("s")
                .above(0.0),
            LIMITS
                .param(
                    "TRAN_MIN_HGT",
                    "Height above arming a forward transition needs",
                )
                .with_units("m")
                .at_least(0.0),
            BACK.param(
                "TRAN_VT_RAMP",
                "Time the lift rotors take to take the lift back",
            )
            .with_units("s")
            .above(0.0),
            BACK.param(
                "TRAN_VT_SPD",
                "Forward speed below which the back transition completes",
            )
            .with_units("m/s")
            .at_least(0.0),
            BACK.param(
                "TRAN_VT_TIMEOUT",
                "Longest back transition before it completes",
            )
            .with_units("s")
            .above(0.0),
        ];
    }
}
