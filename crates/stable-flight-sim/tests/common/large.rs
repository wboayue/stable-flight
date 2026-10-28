//! The autotune plan and envelope for a full-size multirotor ([`airframes::alia250_hover`]): core's
//! copter probes, scaled to an aircraft with a fraction of a quad's angular authority.
//!
//! Core's copter steps (a 1 rad/s rate step for 0.25 s, 10° angle steps) and its settle gate
//! (level within 10 s) are sized for small quads. On the Alia a rate step never rises within
//! its window, so the overshoot criterion passes on any gain, and an angle step
//! builds sideways speed the hover can't brake within the settle timeout (#351).
//!
//! [`airframes::alia250`]: stable_flight_sim::airframes::alia250

use stable_flight::stabilizers::copter::TunedLoop;
use stable_flight::tuning::StepConfig;
use stable_flight::tuning::autotune::{
    COPTER_ANGLE_SEARCH_STEP, COPTER_RATE_STEP, Envelope, Method, SearchPlan, StagePlan,
};

/// Rate steps: 0.2 rad/s for 1.5 s, rising in about 0.6 s on the Alia.
pub const RATE_STEP: StepConfig = StepConfig {
    amplitude: 0.2,
    duration_s: 1.5,
    ..COPTER_RATE_STEP
};

/// Angle steps: 5° for 2.5 s, rising in about 1.2 s.
pub const ANGLE_STEP: StepConfig = StepConfig {
    amplitude: 0.087,
    duration_s: 2.5,
    ..COPTER_ANGLE_SEARCH_STEP
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
    ..COPTER_ANGLE_SEARCH_STEP
};

/// How long a step may clip, in s: a rate step's moment saturates for its first few ticks.
const MAX_SATURATED_S: f32 = 0.2;

/// How long a heading step may clip, in s: the yaw moment saturates while the heading turns
/// (0.63 s measured on the Alia).
const MAX_HEADING_SATURATED_S: f32 = 1.0;

/// Core's rate and angle searches on [`RATE_STEP`] and [`ANGLE_STEP`].
pub fn plan() -> [StagePlan; 4] {
    let rate = search(SearchPlan::copter_rate(), RATE_STEP);
    let angle = search(SearchPlan::copter_angle(), ANGLE_STEP);
    let stage = |target: TunedLoop| StagePlan {
        target,
        method: Method::Search(if target.is_rate() { rate } else { angle }),
    };
    [
        stage(TunedLoop::RollRate),
        stage(TunedLoop::PitchRate),
        stage(TunedLoop::Roll),
        stage(TunedLoop::Pitch),
    ]
}

/// `plan` stepping by `step`, clipping for up to [`MAX_SATURATED_S`].
fn search(mut plan: SearchPlan, step: StepConfig) -> SearchPlan {
    plan.test.step = step;
    plan.test.max_saturated_s = MAX_SATURATED_S;
    plan
}

/// The steps and clip allowance `target` is held to on this plan (yaw and heading, which the
/// plan doesn't tune, on their own steps).
pub fn step(target: TunedLoop) -> (StepConfig, f32) {
    match target {
        TunedLoop::RollRate | TunedLoop::PitchRate => (RATE_STEP, MAX_SATURATED_S),
        TunedLoop::Roll | TunedLoop::Pitch => (ANGLE_STEP, MAX_SATURATED_S),
        TunedLoop::YawRate => (YAW_RATE_STEP, MAX_SATURATED_S),
        TunedLoop::Heading => (HEADING_STEP, MAX_HEADING_SATURATED_S),
    }
}

/// Core's copter envelope, waiting 30 s to settle: the Alia brakes a step's sideways speed
/// slowly.
pub fn envelope() -> Envelope {
    Envelope {
        settle_timeout_s: 30.0,
        ..Envelope::default()
    }
}
