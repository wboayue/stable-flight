//! Simulator-independent pieces of an autotune flight: the commanders that hover on a script
//! and then land ([`ThenLand`]) or fly a script into plane flight, hold the plane there, and
//! come home ([`PlaneTuneFlight`]), the checks a run stops or starts on ([`is_inverted`],
//! [`ThenLand::at_hover_point`]), and the log of each [`AutotuneEvent`] ([`log_event`]), down
//! to the tuned gains printed for pasting into an airframe ([`TunedConfig`]). Each backend's
//! autotune example wires its own vehicle around them.

use std::cell::{Cell, RefCell};
use std::fmt::Debug;
use std::rc::Rc;

use log::{debug, info, warn};

use stable_flight::commanders::waypoint::PLANE_ALTITUDE_ACCEPTANCE_M;
use stable_flight::commanders::{PlaneTuneHold, PlaneTuneHoldConfig, TouchdownDetector};
use stable_flight::controllers::Gains;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::stabilizers::plane::{PlaneAxisGains, PlaneLoop, PlaneStabilizerConfig};
use stable_flight::tuning::autotune::AutotuneEvent;
use stable_flight::tuning::{Measurement, Stat, StepMetrics, StepSummary};
use stable_flight::{
    Command, Commander, ControlInput, FlightRegime, LandCommand, Vec2, VehicleState,
    WaypointCommand,
};

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
    landing: Landing,
}

impl ThenLand {
    /// Flies `script` until the returned switch says land.
    pub fn new(script: ScriptCommander) -> (Self, LandSwitch) {
        let land = Rc::new(Cell::new(false));
        let commander = Self {
            script,
            land: Rc::clone(&land),
            landing: Landing::default(),
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
        self.landing.next_command(dt, state)
    }
}

/// A land that disarms on touchdown, on a position given up front or where the vehicle is on
/// its first command.
#[derive(Default)]
struct Landing {
    at: Option<LandCommand>,
    touchdown: TouchdownDetector,
}

impl Landing {
    /// A land on `position_m`.
    fn on(position_m: Vec2) -> Self {
        Self {
            at: Some(LandCommand { position_m }),
            ..Self::default()
        }
    }

    /// The land, disarmed once the touchdown detector sees the vehicle down.
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        let land = *self.at.get_or_insert(LandCommand {
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

/// Where a [`PlaneTuneFlight`] comes home to: the plane loiter around its script's start, and
/// how close it must come before it lands there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneHome {
    /// The loiter's altitude, in m above the start: where the back transition starts.
    pub altitude_m: f32,
    /// How close to the start, horizontally, the flight home ends in a land, in m: outside the
    /// plane guidance's loiter radius, with room for its turn onto it, so the loiter reaches it.
    pub radius_m: f32,
}

/// Flies a script into plane flight, then the [`PlaneTuneHold`] a plane autotune runs in, then
/// home: a VTOL's plane autotune flight.
///
/// - **Lead:** the script (a hover climb, a transition, a level plane leg) until the host
///   hands over ([`PlaneTuneSwitch::hold`]): on the last leg ([`on_last_leg`](Self::on_last_leg)),
///   once the vehicle flies it straight and level, so the hold starts on a level attitude and
///   the thrust the leg flew level on. At the leg's end the guidance loiters, banked and nose
///   up: a hold started there keeps that pitch, climbs, and slows.
/// - **Hold:** the hold, its turns and recovery driven through the switch.
/// - **Finish** ([`PlaneTuneSwitch::finish`]): after a hold that recovered (an envelope
///   breach: the vehicle hovers on its rotors), or before any hold, a land where the vehicle
///   is. Otherwise a plane loiter around the script's start at the [`PlaneHome`] altitude, at
///   the hold's trim airspeed, then, within its radius of the start and the plane altitude
///   acceptance of that altitude, a land on the start: a VTOL transitions back to land, and
///   disarms on touchdown.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl::autotune::{PlaneHome, PlaneTuneFlight};
/// use stable_flight_sitl::commanders::ScriptCommander;
/// use stable_flight_sitl::stable_flight::commanders::PlaneTuneHoldConfig;
/// use stable_flight_sitl::stable_flight::{
///     Commander, ControlInput, FlightRegime, Vec3, VehicleState, WaypointCommand,
/// };
///
/// let leg = WaypointCommand { altitude_m: 100.0, ..Default::default() };
/// let script = ScriptCommander::from_waypoints(vec![leg]).unwrap();
/// let home = PlaneHome { altitude_m: 30.0, radius_m: 100.0 };
/// let (mut commander, switch) = PlaneTuneFlight::new(script, home);
///
/// let cruising = VehicleState {
///     position_m: Vec3::new(0.0, 0.0, 100.0),
///     airspeed_mps: Some(14.0),
///     ..Default::default()
/// };
/// switch.hold(PlaneTuneHoldConfig::new(0.85, 14.0));
/// let held = commander.next_command(0.004, &cruising);
/// assert!(matches!(held.input, ControlInput::AbsoluteAttitude(_)));
///
/// // Home: a plane loiter around the start at 30 m.
/// switch.finish();
/// let home = commander.next_command(0.004, &cruising);
/// assert_eq!(home.regime, FlightRegime::Plane);
/// let ControlInput::Waypoint(target) = home.input else { unreachable!() };
/// assert_eq!(target.altitude_m, 30.0);
/// ```
pub struct PlaneTuneFlight {
    lead: ScriptCommander,
    home: PlaneHome,
    phase: Rc<RefCell<TunePhase>>,
}

/// Where a [`PlaneTuneFlight`] is.
enum TunePhase {
    Lead,
    Hold {
        hold: PlaneTuneHold,
        /// The hold's trim airspeed, which home flies, in m/s.
        trim_airspeed_mps: f32,
    },
    /// Flying home in plane at this airspeed, in m/s.
    Home {
        airspeed_mps: f32,
    },
    Landing(Landing),
}

impl PlaneTuneFlight {
    /// Flies `script` until the returned switch hands over to the hold; `home` is around the
    /// script's start.
    pub fn new(script: ScriptCommander, home: PlaneHome) -> (Self, PlaneTuneSwitch) {
        let phase = Rc::new(RefCell::new(TunePhase::Lead));
        let commander = Self {
            lead: script,
            home,
            phase: Rc::clone(&phase),
        };
        (commander, PlaneTuneSwitch(phase))
    }

    /// Whether the script flies its last leg, toward its last waypoint, and hasn't reached
    /// it.
    pub fn on_last_leg(&self) -> bool {
        let follower = self.lead.follower();
        follower.current_index() + 1 >= follower.waypoints().len() && !follower.is_complete()
    }

    /// The home loiter's target: over the start at the home altitude, in the vehicle's frame
    /// (m north, east, up).
    fn home_m(&self) -> (Vec2, f32) {
        let origin_m = self.lead.origin_m();
        (origin_m.xy(), origin_m.z + self.home.altitude_m)
    }

    /// Whether the flight home is within reach of the start: inside the home radius and the
    /// plane altitude acceptance of the home altitude.
    fn home_reached(&self, state: &VehicleState) -> bool {
        let (position_m, altitude_m) = self.home_m();
        let distance_m = (state.position_m.xy() - position_m).norm();
        let height_off_m = (state.position_m.z - altitude_m).abs();
        distance_m < self.home.radius_m && height_off_m < PLANE_ALTITUDE_ACCEPTANCE_M
    }

    /// The home loiter's command: a plane waypoint without a leg, which the plane guidance
    /// loiters around.
    fn home_command(&self, airspeed_mps: f32) -> Command {
        let (position_m, altitude_m) = self.home_m();
        Command::new(
            true,
            ControlInput::Waypoint(WaypointCommand {
                position_m,
                altitude_m,
                cruise_speed_mps: airspeed_mps,
                ..Default::default()
            }),
        )
        .with_regime(FlightRegime::Plane)
    }
}

impl Commander for PlaneTuneFlight {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        let mut phase = self.phase.borrow_mut();
        if matches!(*phase, TunePhase::Home { .. }) && self.home_reached(state) {
            *phase = TunePhase::Landing(Landing::on(self.home_m().0));
        }
        match &mut *phase {
            TunePhase::Lead => self.lead.next_command(dt, state),
            TunePhase::Hold { hold, .. } => hold.next_command(dt, state),
            TunePhase::Home { airspeed_mps } => self.home_command(*airspeed_mps),
            TunePhase::Landing(landing) => landing.next_command(dt, state),
        }
    }
}

/// Drives a [`PlaneTuneFlight`] from outside the flight loop, which owns the commander: hands
/// over to the hold, reaches it between ticks, and ends the flight.
pub struct PlaneTuneSwitch(Rc<RefCell<TunePhase>>);

impl PlaneTuneSwitch {
    /// Hands the script over to a hold on `config` from the next command; a no-op past the
    /// lead.
    pub fn hold(&self, config: PlaneTuneHoldConfig) {
        let mut phase = self.0.borrow_mut();
        if matches!(*phase, TunePhase::Lead) {
            *phase = TunePhase::Hold {
                hold: PlaneTuneHold::new(config),
                trim_airspeed_mps: config.trim_airspeed_mps,
            };
        }
    }

    /// Whether the hold flies.
    pub fn is_holding(&self) -> bool {
        matches!(*self.0.borrow(), TunePhase::Hold { .. })
    }

    /// [`PlaneTuneHold::allow_turn`], while the hold flies.
    pub fn allow_turn(&self, allowed: bool) {
        if let TunePhase::Hold { hold, .. } = &mut *self.0.borrow_mut() {
            hold.allow_turn(allowed);
        }
    }

    /// [`PlaneTuneHold::turning`]: false outside the hold.
    pub fn turning(&self) -> bool {
        matches!(&*self.0.borrow(), TunePhase::Hold { hold, .. } if hold.turning())
    }

    /// [`PlaneTuneHold::recover`], while the hold flies.
    pub fn recover(&self) {
        if let TunePhase::Hold { hold, .. } = &mut *self.0.borrow_mut() {
            hold.recover();
        }
    }

    /// Ends the flight: home in plane after a hold that didn't recover, else a land where the
    /// vehicle is. A no-op once finished.
    pub fn finish(&self) {
        let mut phase = self.0.borrow_mut();
        let next = match &*phase {
            TunePhase::Hold {
                hold,
                trim_airspeed_mps,
            } if !hold.is_recovering() => TunePhase::Home {
                airspeed_mps: *trim_airspeed_mps,
            },
            TunePhase::Lead | TunePhase::Hold { .. } => TunePhase::Landing(Landing::default()),
            _ => return,
        };
        *phase = next;
    }

    /// Whether [`finish`](Self::finish) was called.
    pub fn is_finished(&self) -> bool {
        !matches!(*self.0.borrow(), TunePhase::Lead | TunePhase::Hold { .. })
    }

    /// Whether the land has started.
    pub fn is_landing(&self) -> bool {
        matches!(*self.0.borrow(), TunePhase::Landing(_))
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

/// A stabilizer config an autotune tunes, as [`log_event`] prints it: a stage's gains, and the
/// tuned gains laid out like an airframe's for pasting.
pub trait TunedConfig {
    /// A stage's gains ([`AutotuneEvent`]'s `G`).
    type Gains: Debug;

    /// `gains` as an airframe writes them.
    fn format_gains(gains: &Self::Gains) -> String;

    /// The tuned gains, headed by `source`, the example that tuned them.
    fn airframe_snippet(&self, source: &str) -> String;
}

/// Logs an autotune event: lifecycle at `info!`/`warn!`, each step (including ones that run
/// again) at `debug!`. On completion it prints `config`'s tuned gains for pasting
/// as an airframe snippet, headed by `source`, the example that tuned them.
pub fn log_event<T: Debug, C: TunedConfig>(
    event: &AutotuneEvent<T, C::Gains>,
    config: &C,
    source: &str,
) {
    let format_gains = C::format_gains;
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
        AutotuneEvent::Measured {
            target,
            measurement: Measurement::Wing(model),
        } => info!(
            "autotune: {target:?} fit: stiffness {:.1}/s², damping {:.1}/s, control {:.0} rad/s², \
             lag {}, delay {}, path turn {:.2}/s, trim moment {:.3}",
            model.stiffness_per_s2,
            model.damping_per_s,
            model.control_radps2,
            format_ms(Some(model.lag_s)),
            format_ms(Some(model.delay_s)),
            model.path_turn_per_s,
            model.trim_moment,
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
            println!("{}", config.airframe_snippet(source));
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

/// A copter's loops: [`Gains`], and a `CopterStabilizerConfig` snippet.
impl TunedConfig for CopterStabilizerConfig {
    type Gains = Gains;

    fn format_gains(gains: &Gains) -> String {
        format!(
            "Gains {{ p: {:.4}, i: {:.4}, d: {:.4} }}",
            gains.p, gains.i, gains.d
        )
    }

    /// The tuned gains, laid out like an airframe's `CopterStabilizerConfig`, with the yaw
    /// integral limit the yaw I was tuned on: it holds the yaw trim only up to that limit.
    fn airframe_snippet(&self, source: &str) -> String {
        let gains = |target: TunedLoop| Self::format_gains(&target.gains(self));
        format!(
            "// {source}\n\
             body_rate: BodyRateControllerConfig {{\n    \
                 roll_gains: {},\n    \
                 pitch_gains: {},\n    \
                 yaw_gains: {},\n    \
                 yaw_max_integral: {:.2},\n    \
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
            self.body_rate.yaw_max_integral,
            gains(TunedLoop::Roll),
            gains(TunedLoop::Pitch),
            gains(TunedLoop::Heading),
        )
    }
}

/// A plane's roll and pitch: [`PlaneAxisGains`], and a `PlaneStabilizerConfig` snippet.
impl TunedConfig for PlaneStabilizerConfig {
    type Gains = PlaneAxisGains;

    fn format_gains(gains: &PlaneAxisGains) -> String {
        format!(
            "{}, time constant {:.3} s",
            format_rate_gains(gains),
            gains.time_constant_s
        )
    }

    /// The tuned gains, laid out like an airframe's `PlaneStabilizerConfig`: the rate loop's
    /// roll and pitch, and the attitude loops' time constants.
    fn airframe_snippet(&self, source: &str) -> String {
        let roll = PlaneLoop::Roll.gains(self);
        let pitch = PlaneLoop::Pitch.gains(self);
        format!(
            "// {source}\n\
             rate: PlaneRateControllerConfig {{\n    \
                 roll: {},\n    \
                 pitch: {},\n    \
                 ..\n\
             }},\n\
             time_constant_s: Vec2::new({:.3}, {:.3}),",
            format_rate_gains(&roll),
            format_rate_gains(&pitch),
            roll.time_constant_s,
            pitch.time_constant_s,
        )
    }
}

fn format_rate_gains(gains: &PlaneAxisGains) -> String {
    let rate = &gains.rate;
    format!(
        "RateGains {{ p: {:.4}, i: {:.4}, ff: {:.4} }}",
        rate.p, rate.i, rate.ff
    )
}
