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
        param!("TRAN_FW_MIN_HGT", plane_min_height_m),
        param!("TRAN_VT_RAMP", back_ramp_s),
        param!("TRAN_VT_SPD", back_speed_mps),
        param!("TRAN_VT_TIMEOUT", back_timeout_s),
        param!("TRAN_VT_DEC", back_decel_mps2),
        param!("TRAN_VT_DEC_I", back_decel_i),
        param!("TRAN_FW_THR", forward_thrust),
        param!("TRAN_PSH_SLEW", pusher_slew_per_s),
        param!("TRAN_HANDOFF", handoff_s),
        param!(deg "TRAN_ROLL_MAX", roll_limit_rad),
        param!(deg "TRAN_FW_PIT_MAX", forward_pitch_max_rad),
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
                    "Longest wait on airspeed before the forward transition reverts",
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
            BACK.param(
                "TRAN_VT_DEC",
                "Deceleration the back transition pitches up to hold",
            )
            .with_units("m/s/s")
            .at_least(0.0),
            BACK.param(
                "TRAN_VT_DEC_I",
                "How fast the back transition's pitch-up grows (0: off)",
            )
            .with_units("rad*s/m")
            .at_least(0.0),
            FORWARD
                .param(
                    "TRAN_FW_THR",
                    "Pusher thrust through the forward transition",
                )
                .at_least(0.0)
                .at_most(1.0),
            FORWARD
                .param(
                    "TRAN_PSH_SLEW",
                    "Fastest the pusher's thrust rises going forward",
                )
                .with_units("1/s")
                .above(0.0),
            LIMITS
                .param(
                    "TRAN_HANDOFF",
                    "Pusher thrust blend time at the handoff to plane",
                )
                .with_units("s")
                .at_least(0.0),
            LIMITS
                .param("TRAN_ROLL_MAX", "Bank that cuts a transition short")
                .with_units("deg")
                .above(0.0)
                .at_most(180.0),
            FORWARD
                .param(
                    "TRAN_FW_PIT_MAX",
                    "Most pitch waiting on airspeed; twice it plus 2 deg on the ramp",
                )
                .with_units("deg")
                .above(0.0)
                .below(90.0),
        ];
    }
}
