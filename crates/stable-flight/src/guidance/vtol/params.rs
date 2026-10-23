//! [`TransitionConfig`]'s params; the names and units are tabled in [`crate::params`].

use super::TransitionConfig;
use crate::params::{Param, ParamTable, param};

impl ParamTable for TransitionConfig {
    const PARAMS: &'static [Param<Self>] = &[
        param!(deg "TRAN_FW_RATE", forward_rate_radps),
        param!(deg "TRAN_FW_DONE", forward_done_rad),
        param!(deg "TRAN_FW_MAX", forward_max_tilt_rad),
        param!("TRAN_ASPD_MIN", min_airspeed_mps),
        param!(deg "TRAN_VT_RATE", back_rate_radps),
        param!(deg "TRAN_VT_DONE", back_done_rad),
        param!("TRAN_TIMEOUT", timeout_s),
        param!("TRAN_MIN_HGT", min_height_m),
        param!("TRAN_FW_MIN_HGT", plane_min_height_m),
        param!("TRAN_FW_THR", forward_thrust),
        param!("TRAN_VT_THR", back_thrust),
        param!("TRAN_VT_SINK_LIM", back_sink_limit_mps),
        param!("TRAN_VT_SINK_GN", back_sink_gain),
        param!("TRAN_HANDOFF", handoff_s),
        param!(deg "TRAN_ROLL_MAX", roll_limit_rad),
    ];
}

#[cfg(feature = "param-metadata")]
mod metadata {
    use super::TransitionConfig;
    use crate::params::metadata::{ParamMeta, ParamMetadata};

    const FORWARD: ParamMeta = ParamMeta::in_group("Transition", "Forward");
    const BACK: ParamMeta = ParamMeta::in_group("Transition", "Back");
    const LIMITS: ParamMeta = ParamMeta::in_group("Transition", "Limits");

    impl ParamMetadata for TransitionConfig {
        const METADATA: &'static [ParamMeta] = &[
            FORWARD
                .param("TRAN_FW_RATE", "Nose-down rate of the forward ramp")
                .with_units("deg/s")
                .above(0.0),
            FORWARD
                .param(
                    "TRAN_FW_DONE",
                    "Tilt from vertical that completes the forward transition",
                )
                .with_units("deg")
                .above(0.0)
                .below(90.0),
            FORWARD
                .param(
                    "TRAN_FW_MAX",
                    "Tilt where the forward ramp waits on airspeed",
                )
                .with_units("deg")
                .above(0.0)
                .at_most(90.0),
            FORWARD
                .param(
                    "TRAN_ASPD_MIN",
                    "Airspeed the forward transition needs (0: off)",
                )
                .with_units("m/s")
                .at_least(0.0),
            BACK.param("TRAN_VT_RATE", "Nose-up rate of the back ramp")
                .with_units("deg/s")
                .above(0.0),
            BACK.param(
                "TRAN_VT_DONE",
                "Tilt from vertical that completes the back transition",
            )
            .with_units("deg")
            .above(0.0)
            .below(90.0),
            LIMITS
                .param(
                    "TRAN_TIMEOUT",
                    "Longest transition: forward reverts, back completes",
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
            LIMITS
                .param(
                    "TRAN_FW_MIN_HGT",
                    "Height above arming plane flight falls back below (0: off)",
                )
                .with_units("m")
                .at_least(0.0),
            FORWARD
                .param("TRAN_FW_THR", "Least thrust through the forward transition")
                .at_least(0.0)
                .at_most(1.0),
            BACK.param("TRAN_VT_THR", "Thrust through the back transition")
                .at_least(0.0)
                .at_most(1.0),
            BACK.param(
                "TRAN_VT_SINK_LIM",
                "Sink past which the back transition adds thrust (0: off)",
            )
            .with_units("m/s")
            .at_least(0.0),
            BACK.param(
                "TRAN_VT_SINK_GN",
                "Thrust added per m/s of sink past TRAN_VT_SINK_LIM",
            )
            .with_units("s/m")
            .at_least(0.0),
            LIMITS
                .param("TRAN_HANDOFF", "Thrust blend time at a controller handoff")
                .with_units("s")
                .at_least(0.0),
            LIMITS
                .param("TRAN_ROLL_MAX", "Roll error that cuts a transition short")
                .with_units("deg")
                .above(0.0)
                .at_most(180.0),
        ];
    }
}
