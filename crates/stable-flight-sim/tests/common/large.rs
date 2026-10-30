//! The steps a full-size multirotor ([`airframes::alia250_hover`]) is held to: core's
//! large-copter steps for roll and pitch (`LARGE_COPTER_RATE_STEP`, `LARGE_COPTER_TILT_STEP`,
//! which `StagePlan::large_copter_search` tunes on), and slower yaw and heading steps for the
//! loops that plan doesn't tune.
//!
//! [`airframes::alia250_hover`]: stable_flight_sim::airframes::alia250_hover

use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::StepConfig;
use stable_flight::tuning::autotune::{
    COPTER_RATE_STEP, COPTER_TILT_STEP, LARGE_COPTER_MAX_SATURATED_S, LARGE_COPTER_RATE_STEP,
    LARGE_COPTER_TILT_STEP,
};

/// Yaw rate steps: 0.2 rad/s for 3 s, rising in about 0.9 s.
pub const YAW_RATE_STEP: StepConfig = StepConfig {
    amplitude: 0.2,
    duration_s: 3.0,
    ..COPTER_RATE_STEP
};

/// Heading steps: 0.5 rad (29°) for 8 s, rising in about 1.7 s.
pub const HEADING_STEP: StepConfig = StepConfig {
    amplitude: 0.5,
    duration_s: 8.0,
    ..COPTER_TILT_STEP
};

/// How long a heading step may clip, in s: the yaw moment saturates while the heading turns
/// (0.63 s measured on the Alia).
const MAX_HEADING_SATURATED_S: f32 = 1.0;

/// The steps and clip allowance `target` is held to on the large-copter plan (yaw and
/// heading, which the plan doesn't tune, on their own steps).
pub fn step(target: TunedLoop) -> (StepConfig, f32) {
    match target {
        TunedLoop::RollRate | TunedLoop::PitchRate => {
            (LARGE_COPTER_RATE_STEP, LARGE_COPTER_MAX_SATURATED_S)
        }
        TunedLoop::Roll | TunedLoop::Pitch => {
            (LARGE_COPTER_TILT_STEP, LARGE_COPTER_MAX_SATURATED_S)
        }
        TunedLoop::YawRate => (YAW_RATE_STEP, LARGE_COPTER_MAX_SATURATED_S),
        TunedLoop::Heading => (HEADING_STEP, MAX_HEADING_SATURATED_S),
    }
}
