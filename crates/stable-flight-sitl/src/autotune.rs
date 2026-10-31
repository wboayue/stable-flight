//! Simulator-independent pieces of an autotune flight: the commander that hovers on a script
//! and then lands ([`ThenLand`]), the checks a run stops or starts on ([`is_inverted`],
//! [`ThenLand::at_hover_point`]), and the log of each [`AutotuneEvent`] ([`log_event`]), down
//! to the tuned gains printed for pasting into an airframe. Each backend's autotune example
//! wires its own vehicle around them.

use std::cell::Cell;
use std::rc::Rc;

use log::{debug, info, warn};

use stable_flight::commanders::TouchdownDetector;
use stable_flight::controllers::Gains;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::AutotuneEvent;
use stable_flight::tuning::{Measurement, Stat, StepMetrics, StepSummary};
use stable_flight::{Command, Commander, ControlInput, LandCommand, VehicleState};

use crate::commanders::ScriptCommander;

#[cfg(test)]
mod tests;

/// How close to the script's hover altitude the vehicle must be before an autotune starts, in m.
const START_ALTITUDE_TOLERANCE_M: f32 = 0.5;

/// Flies a script until told to land, then lands where it is and disarms on touchdown.
///
/// The script climbs to a hover point and holds there while the autotune runs; its
/// [`LandSwitch`] starts the landing (the host flips it once the autotune ends and the state
/// is fresh).
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::autotune::ThenLand;
/// use stable_flight_sitl::commanders::ScriptCommander;
/// use stable_flight_sitl::stable_flight::{Commander, ControlInput, VehicleState, WaypointCommand};
///
/// let hover = WaypointCommand { altitude_m: 5.0, ..Default::default() };
/// let script = ScriptCommander::from_waypoints(vec![hover]).unwrap();
/// let (mut commander, switch) = ThenLand::new(script);
///
/// switch.land();
/// assert!(switch.is_landing());
/// let command = commander.next_command(0.004, &VehicleState::default());
/// assert!(matches!(command.input, ControlInput::Land(_)));
/// ```
pub struct ThenLand {
    script: ScriptCommander,
    land: Rc<Cell<bool>>,
    landing: Option<LandCommand>,
    touchdown: TouchdownDetector,
}

impl ThenLand {
    /// Flies `script` until the returned switch says land.
    pub fn new(script: ScriptCommander) -> (Self, LandSwitch) {
        let land = Rc::new(Cell::new(false));
        let commander = Self {
            script,
            land: Rc::clone(&land),
            landing: None,
            touchdown: TouchdownDetector::default(),
        };
        (commander, LandSwitch(land))
    }

    /// Whether the script has reached its last waypoint, the hover point, and the vehicle is
    /// within 0.5 m of its altitude: the autotune starts there.
    pub fn at_hover_point(&self, state: &VehicleState) -> bool {
        let follower = self.script.follower();
        let Some(hover) = follower.waypoints().last() else {
            return false;
        };
        let altitude_m = state.position_m.z - self.script.origin_m().z;
        follower.is_complete() && (altitude_m - hover.altitude_m).abs() < START_ALTITUDE_TOLERANCE_M
    }
}

impl Commander for ThenLand {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        if !self.land.get() {
            return self.script.next_command(dt, state);
        }
        let land = *self.landing.get_or_insert(LandCommand {
            position_m: state.position_m.xy(),
        });
        let down = self.touchdown.update(dt, state.velocity_mps.z);
        Command::new(!down, ControlInput::Land(land))
    }
}

/// Starts a [`ThenLand`]'s landing from outside the flight loop, which owns the commander.
pub struct LandSwitch(Rc<Cell<bool>>);

impl LandSwitch {
    /// Lands: the commander's next command is a land where the vehicle is.
    pub fn land(&self) {
        self.0.set(true);
    }

    /// Whether the landing has started.
    pub fn is_landing(&self) -> bool {
        self.0.get()
    }
}

/// Whether the vehicle is tilted past 90° in `frame`, the frame its loops fly in (the
/// stabilizer's `Tunable::probe_frame`: identity for a copter, the view for a tailsitter, where
/// nose-up hover is level): the cosine of the tilt (the NED-down component of `frame`'s z axis)
/// is negative. False for a NaN attitude. A run stops when it stays inverted, since an inverted
/// vehicle never touches down to end it.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::autotune::is_inverted;
/// use stable_flight_sitl::stable_flight::VehicleState;
/// use stable_flight_sitl::stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight_sitl::stable_flight::view::tailsitter_rotation;
///
/// let body = UnitQuaternion::identity();
/// let flipped = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(3.0, 0.0, 0.0),
///     ..Default::default()
/// };
/// assert!(is_inverted(&flipped, &body));
/// assert!(!is_inverted(&VehicleState::default(), &body));
///
/// // A tailsitter hovering nose up is level in its view.
/// let nose_up = VehicleState {
///     attitude: tailsitter_rotation().inverse(),
///     ..Default::default()
/// };
/// assert!(!is_inverted(&nose_up, &tailsitter_rotation()));
/// ```
pub fn is_inverted(state: &VehicleState, frame: &UnitQuaternion<f32>) -> bool {
    // The frame's attitude as `VehicleState::rotate_body` builds it.
    (state.attitude * frame).to_rotation_matrix()[(2, 2)] < 0.0
}

/// Logs an autotune event: lifecycle at `info!`/`warn!`, each step (including ones that run
/// again) at `debug!`. On completion it prints `config`'s tuned gains for pasting
/// as an airframe snippet, headed by `source`, the example that tuned them.
pub fn log_event(event: &AutotuneEvent, config: &CopterStabilizerConfig, source: &str) {
    match event {
        AutotuneEvent::StageStarted(target) => info!("autotune: stage {target:?}"),
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Step(metrics),
        } => debug!("autotune: {target:?} step {}", format_step(metrics)),
        AutotuneEvent::Stalled {
            target,
            measurement: Some(Measurement::Step(metrics)),
        } => debug!(
            "autotune: {target:?} step {} (stalled, runs again)",
            format_step(metrics)
        ),
        AutotuneEvent::Stalled { target, .. } => {
            debug!("autotune: {target:?} probe stopped (host slow), runs again");
        }
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Identify(model),
        } => info!(
            "autotune: {target:?} fit: delay {} ticks, lag {}, accel {} ({model:?})",
            model.delay_ticks,
            format_ms(model.lag_s()),
            model
                .accel_radps2()
                .map_or("-".to_string(), |accel| format!("{accel:.0} rad/s²")),
        ),
        AutotuneEvent::Tried {
            target,
            gains,
            passed,
            steps,
        } => info!(
            "autotune: {target:?} {} {}: {}",
            format_gains(gains),
            if *passed { "passed" } else { "failed" },
            format_summary(steps),
        ),
        AutotuneEvent::StageDone {
            target,
            gains,
            outcome,
            steps,
            baseline,
        } => {
            if let Some(baseline) = baseline {
                info!("autotune: {target:?} baseline {}", format_summary(baseline));
            }
            if let Some(steps) = steps {
                info!("autotune: {target:?} steps {}", format_summary(steps));
            }
            info!(
                "autotune: {target:?} {outcome:?}: flies {}",
                format_gains(gains)
            );
        }
        AutotuneEvent::Lowered { target, gains } => warn!(
            "autotune: {target:?} start above the search's max, lowered to {}",
            format_gains(gains)
        ),
        AutotuneEvent::Unsettled { target, gains } => warn!(
            "autotune: {target:?} design {} didn't settle; fitting again",
            format_gains(gains)
        ),
        AutotuneEvent::StageFailed { target, reason } => {
            warn!("autotune: {target:?} failed: {reason:?}; the last kept gains fly");
        }
        AutotuneEvent::Complete => {
            info!("autotune: complete");
            println!("{}", airframe_snippet(config, source));
        }
        AutotuneEvent::Reverted => warn!("autotune: disarmed, gains reverted"),
        event => debug!("autotune: {event:?}"),
    }
}

/// A summary's means, for a baseline-vs-verify table: rise and settling in ms, overshoot and
/// final value as fractions of the step, `-` for a metric with no samples.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::autotune::format_brief;
/// use stable_flight_sitl::stable_flight::tuning::{StepMetrics, StepSummary};
///
/// let mut summary = StepSummary::default();
/// summary.add(&StepMetrics { rise_time_s: Some(0.05), final_value: 1.0, ..Default::default() });
/// assert_eq!(
///     format_brief(&summary),
///     "rise 50.00 ms, settle - ms, overshoot 0.00, final 1.00"
/// );
/// ```
pub fn format_brief(summary: &StepSummary) -> String {
    format!(
        "rise {} ms, settle {} ms, overshoot {}, final {}",
        format_mean(&summary.rise_time_s, 1e3),
        format_mean(&summary.settling_time_s, 1e3),
        format_mean(&summary.overshoot, 1.0),
        format_mean(&summary.final_value, 1.0),
    )
}

/// One step's response: the step, then its metrics (normalized to the step).
fn format_step(metrics: &StepMetrics) -> String {
    format!(
        "{:+.3}: overshoot {:.2}, rise {}, settle {}, bounce {:.2}, final {:.2}, clipped {:.0} ms",
        metrics.amplitude,
        metrics.overshoot,
        format_ms(metrics.rise_time_s),
        format_ms(metrics.settling_time_s),
        metrics.bounce_back,
        metrics.final_value,
        metrics.saturated_s * 1e3,
    )
}

fn format_ms(time_s: Option<f32>) -> String {
    time_s.map_or("-".to_string(), |s| format!("{:.0} ms", s * 1e3))
}

/// A stage's steps: each metric's mean [min-max]; overshoot per direction; steps that never
/// rose or settled.
fn format_summary(summary: &StepSummary) -> String {
    let stat = |stat: &Stat, scale: f32| match stat.mean() {
        Some(mean) => format!(
            "{:.2} [{:.2}-{:.2}]",
            mean * scale,
            stat.min * scale,
            stat.max * scale
        ),
        None => "-".to_string(),
    };
    format!(
        "n {}: overshoot {} (+ {}, - {}), rise {} ms, settle {} ms, bounce {}, final {}; \
         not risen {}, not settled {}",
        summary.count(),
        stat(&summary.overshoot, 1.0),
        format_mean(&summary.overshoot_positive, 1.0),
        format_mean(&summary.overshoot_negative, 1.0),
        stat(&summary.rise_time_s, 1e3),
        stat(&summary.settling_time_s, 1e3),
        stat(&summary.bounce_back, 1.0),
        stat(&summary.final_value, 1.0),
        summary.not_risen,
        summary.not_settled,
    )
}

/// A metric's mean times `scale`, or `-` with no samples.
fn format_mean(stat: &Stat, scale: f32) -> String {
    stat.mean()
        .map_or("-".to_string(), |mean| format!("{:.2}", mean * scale))
}

fn format_gains(gains: &Gains) -> String {
    format!(
        "Gains {{ p: {:.4}, i: {:.4}, d: {:.4} }}",
        gains.p, gains.i, gains.d
    )
}

/// The tuned gains, laid out like an airframe's `CopterStabilizerConfig` for pasting and headed
/// by `source`, with the yaw integral limit the yaw I was tuned on: it holds the yaw trim only
/// up to that limit.
fn airframe_snippet(config: &CopterStabilizerConfig, source: &str) -> String {
    let gains = |target: TunedLoop| format_gains(&target.gains(config));
    format!(
        "// {source}\n\
         body_rate: BodyRateControllerConfig {{\n    \
             roll_gains: {},\n    \
             pitch_gains: {},\n    \
             yaw_gains: {},\n    \
             yaw_max_integral: {:.1},\n    \
             ..\n\
         }},\n\
         attitude: AttitudeControllerConfig {{\n    \
             roll_gains: {},\n    \
             pitch_gains: {},\n    \
             ..\n\
         }},\n\
         yaw: YawControllerConfig {{\n    \
             gains: {},\n    \
             ..\n\
         }},",
        gains(TunedLoop::RollRate),
        gains(TunedLoop::PitchRate),
        gains(TunedLoop::YawRate),
        config.body_rate.yaw_max_integral,
        gains(TunedLoop::Roll),
        gains(TunedLoop::Pitch),
        gains(TunedLoop::Heading),
    )
}
