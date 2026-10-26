//! The autotune method comparison behind `architecture.md#autotune` (#312): where the step
//! search's time goes, and three faster methods on the same plants, each held to the
//! `step_response` spec on its own plant. Every test is an `#[ignore]`d sweep that prints a
//! table; none asserts.
//!
//! ```text
//! cargo test --release -p stable-flight-sim --test autotune_methods -- --ignored --nocapture --test-threads 1
//! ```
//!
//! - **Identification:** a shrinking square wave on the attitude setpoint (PX4's excitation
//!   shape, `mc_autotune_attitude_control.cpp:603-639` at `ae4fd97`), every loop closed; the
//!   axis's moment command and body rate fitted by least squares to a second-order model with a
//!   delay (PX4's `ArxRls<2,2,1>`, its delay chosen here by least residual).
//! - **Design** from the model: PX4's pole placement (`pid_design.hpp:68-103`), or loop
//!   shaping (the rate PD's zero on the fitted lag, P for a phase margin). Angle P from the
//!   model on the designed rate loop.
//! - **Gain ramp:** ArduPilot's QuikTune (`AP_Scripting/applets/VTOL-quicktune.lua` at
//!   `e96bd90`): D then P raised until the rate PID's output slew oscillates
//!   (`Filter/SlewLimiter.cpp:31-81`), then backed off.
//!
//! Each is described from its source and implemented here from that description.

mod common;

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::f32::consts::PI;
use std::rc::Rc;

use stable_flight::controllers::Gains;
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::nalgebra::{Complex, ComplexField, SMatrix, SVector};
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::{Autotune, AutotuneEvent, Envelope, SearchPlan, StagePlan};
use stable_flight::tuning::{StepConfig, StepProbe, StepStatus, StepSummary};
use stable_flight::{
    ActuatorCommand, ControlInput, FlightLoop, FlightModeReport, FlightRegime, GRAVITY, Guidance,
    Saturation, Setpoint, TelemetryPublisher, TelemetryRecord, Vec3, VehicleState,
};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{GyroNoise, Perturbations, Vibration, Wind};

use common::{HOVER_SETTLE_S, Plant, TICK, fly, hover, hover_follower, seconds, spec, ticks};

/// The control tick, in s.
const DT: f32 = 0.004;

/// The airframes every sweep flies.
fn airframes() -> [(&'static str, Airframe); 3] {
    [
        ("quad_x", airframes::quad_x()),
        ("heavy_quad", airframes::heavy_quad()),
        ("racer", airframes::racer()),
    ]
}

/// Simulated seconds and probes per stage of `StagePlan::copter_search` from the airframes'
/// own gains on the clean plant: the settle before each step takes most of each stage.
#[test]
#[ignore = "method sweep (#312)"]
fn search_time_per_stage() {
    for (name, airframe) in airframes() {
        let mut flight_loop = hover(&Plant::clean(airframe));
        let plan = StagePlan::copter_search();
        let mut autotune = Autotune::new(&plan, Envelope::default()).expect("valid plan");
        autotune.start(&flight_loop).expect("armed, idle");
        let (mut start_s, mut probes) = (0.0, 0);
        for tick in 1..=ticks(common::AUTOTUNE_TIMEOUT_S) {
            flight_loop.step(TICK);
            let now_s = seconds(tick);
            match autotune.update(&mut flight_loop, TICK.as_secs_f32()) {
                Some(AutotuneEvent::StageStarted(_)) => (start_s, probes) = (now_s, 0),
                Some(AutotuneEvent::Measured { .. }) => probes += 1,
                Some(AutotuneEvent::StageDone { target, .. }) => {
                    println!(
                        "{name} {target:?}: {:.0} s, {probes} probes",
                        now_s - start_s
                    );
                }
                _ => {}
            }
            if !autotune.is_running() {
                println!("{name}: {now_s:.0} s in all");
                break;
            }
        }
    }
}

// ---- Excitation ----------------------------------------------------------------------------

/// Peak of the identification's square wave on the attitude setpoint, in rad.
const EXCITATION_RAD: f32 = 4.0 * PI / 180.0;

/// Seconds of excitation per axis: 3, 5, 10, and 20 s fitted the same models.
const EXCITATION_S: f32 = 3.0;

/// Time constant of the square wave's washout, in s (PX4: 0.2 s).
const WASHOUT_TC_S: f32 = 0.2;

/// What the excitation adds to the attitude setpoint this run.
#[derive(Clone, Copy, Debug, Default)]
struct Drive {
    /// 0 roll, 1 pitch; `None` off.
    axis: Option<usize>,
    amplitude_rad: f32,
    /// A sine of this frequency (QuikTune's dither) instead of the square wave, in Hz.
    sine_hz: Option<f32>,
    elapsed_s: f32,
}

/// PX4's unipolar square: half-periods from 1.0 s down to 0.2 s by 0.1 s, then 0.6 s down to
/// 0.2 s, repeating. 1 or 0 at `t_s`.
fn square(t_s: f32) -> f32 {
    let half_s = |i: usize| match i {
        0..=8 => 1.0 - 0.1 * i as f32,
        _ => 0.6 - 0.1 * ((i - 9) % 5) as f32,
    };
    let (mut edge_s, mut level, mut i) = (0.0, 1.0, 0);
    loop {
        edge_s += half_s(i);
        if t_s < edge_s {
            return level;
        }
        level = 1.0 - level;
        i += 1;
    }
}

/// Copter guidance with [`Drive`]'s signal added to the attitude setpoint's roll or pitch.
struct Excited {
    inner: CopterGuidance,
    drive: Rc<Cell<Drive>>,
    washout_rad: f32,
}

impl Guidance for Excited {
    fn guide(
        &mut self,
        dt: f32,
        regime: FlightRegime,
        input: &ControlInput,
        state: &VehicleState,
    ) -> Setpoint {
        let mut setpoint = self.inner.guide(dt, regime, input, state);
        let mut drive = self.drive.get();
        let (Some(axis), Setpoint::Attitude(attitude)) = (drive.axis, &mut setpoint) else {
            self.washout_rad = 0.0;
            return setpoint;
        };
        drive.elapsed_s += dt;
        self.drive.set(drive);
        let signal_rad = match drive.sine_hz {
            Some(hz) => drive.amplitude_rad * (2.0 * PI * hz * drive.elapsed_s).sin(),
            None => {
                let raw_rad = drive.amplitude_rad * square(drive.elapsed_s);
                self.washout_rad += (raw_rad - self.washout_rad) * (dt / WASHOUT_TC_S).min(1.0);
                raw_rad - self.washout_rad
            }
        };
        if axis == 0 {
            attitude.roll_rad += signal_rad;
        } else {
            attitude.pitch_rad += signal_rad;
        }
        setpoint
    }

    fn prime(&mut self, state: &VehicleState) {
        self.inner.prime(state);
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }

    fn flight_mode(&self) -> Option<FlightModeReport> {
        self.inner.flight_mode()
    }
}

/// One tick's roll and pitch, each (moment command, body rate in rad/s).
type AxisTick = [(f32, f32); 2];

/// Each tick's roll and pitch (moment command, body rate in rad/s) while on.
#[derive(Clone, Default)]
struct Recorder {
    on: Rc<Cell<bool>>,
    ticks: Rc<RefCell<Vec<AxisTick>>>,
}

impl TelemetryPublisher for Recorder {
    fn publish(&mut self, record: &TelemetryRecord) {
        if self.on.get() {
            let moment = record.control.moments;
            let rate_radps = record.state.angular_velocity_radps;
            self.ticks
                .borrow_mut()
                .push([(moment.x, rate_radps.x), (moment.y, rate_radps.y)]);
        }
    }
}

/// A copter pipeline on [`Excited`] guidance, recorded by a [`Recorder`].
type ExcitedLoop = FlightLoop<
    common::Hover,
    stable_flight_sim::SimCopter,
    Excited,
    CopterStabilizer,
    stable_flight::mixers::TableMixer<4>,
    Recorder,
>;

/// `plant` hovering on [`Excited`] guidance, recorded by `recorder`.
fn excitable(plant: &Plant, drive: &Rc<Cell<Drive>>, recorder: &Recorder) -> ExcitedLoop {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        hover_follower(),
        plant.vehicle(),
        Excited {
            inner: CopterGuidance::new(airframe.guidance),
            drive: drive.clone(),
            washout_rad: 0.0,
        },
        CopterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        recorder.clone(),
    );
    flight_loop.prime();
    fly(&mut flight_loop, HOVER_SETTLE_S, |_, _| {});
    flight_loop
}

// ---- Identification ------------------------------------------------------------------------

/// Longest delay tried, in ticks.
const MAX_DELAY_TICKS: usize = 10;

/// A rate axis's model: `y[k] = −a1·y[k−1] − a2·y[k−2] + b0·u[k−d] + b1·u[k−d−1] +
/// b2·u[k−d−2]`, `u` the moment command, `y` the body rate in rad/s, `d` the delay in ticks.
/// An integrator and a lag fit it as `a1 = −(1 + p)`, `a2 = p`, `p` the lag's pole.
#[derive(Clone, Copy, Debug)]
struct RateModel {
    a1: f32,
    a2: f32,
    b: [f32; 3],
    delay_ticks: usize,
}

impl RateModel {
    /// The lag's time constant, in s, from its pole `a2`; `None` unless `0 < a2 < 1`.
    fn lag_s(&self) -> Option<f32> {
        (self.a2 > 0.0 && self.a2 < 1.0).then(|| -DT / self.a2.ln())
    }
}

/// Excites roll, then pitch, [`EXCITATION_S`] each, and fits each axis.
fn identify(plant: &Plant) -> [RateModel; 2] {
    let drive = Rc::new(Cell::new(Drive::default()));
    let recorder = Recorder::default();
    let mut flight_loop = excitable(plant, &drive, &recorder);
    [0, 1].map(|axis| {
        recorder.ticks.borrow_mut().clear();
        recorder.on.set(true);
        drive.set(Drive {
            axis: Some(axis),
            amplitude_rad: EXCITATION_RAD,
            ..Drive::default()
        });
        fly(&mut flight_loop, EXCITATION_S, |_, _| {});
        drive.set(Drive::default());
        recorder.on.set(false);
        fly(&mut flight_loop, 1.0, |_, _| {});
        let ticks = recorder.ticks.borrow();
        let (moment, rate_radps): (Vec<f32>, Vec<f32>) = ticks.iter().map(|t| t[axis]).unzip();
        (1..=MAX_DELAY_TICKS)
            .map(|delay| fit(&moment, &rate_radps, delay))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("delays to try")
            .0
    })
}

/// Least squares fit at `delay_ticks`, with a constant term for the trim moment; the model
/// and its RMS residual in rad/s.
fn fit(moment: &[f32], rate_radps: &[f32], delay_ticks: usize) -> (RateModel, f64) {
    let regressors = |k: usize| {
        SVector::<f64, 6>::from([
            -rate_radps[k - 1] as f64,
            -rate_radps[k - 2] as f64,
            moment[k - delay_ticks] as f64,
            moment[k - delay_ticks - 1] as f64,
            moment[k - delay_ticks - 2] as f64,
            1.0,
        ])
    };
    let rows = delay_ticks + 2..rate_radps.len();
    let mut normal = SMatrix::<f64, 6, 6>::zeros();
    let mut projected = SVector::<f64, 6>::zeros();
    for k in rows.clone() {
        let phi = regressors(k);
        normal += phi * phi.transpose();
        projected += phi * rate_radps[k] as f64;
    }
    let theta = normal.try_inverse().expect("excited") * projected;
    let squared: f64 = rows
        .clone()
        .map(|k| (rate_radps[k] as f64 - regressors(k).dot(&theta)).powi(2))
        .sum();
    let model = RateModel {
        a1: theta[0] as f32,
        a2: theta[1] as f32,
        b: [theta[2] as f32, theta[3] as f32, theta[4] as f32],
        delay_ticks,
    };
    (model, (squared / rows.len() as f64).sqrt())
}

// ---- Design --------------------------------------------------------------------------------

/// PX4's design rise time, in s (`MC_AT_RISE_TIME`), and its detune.
const POLE_PLACEMENT_RISE_S: f32 = 0.14;
const POLE_PLACEMENT_DETUNE: f32 = 0.7;

/// Phase margins the loop shaping is swept at, in degrees.
const PHASE_MARGINS_DEG: [f32; 3] = [55.0, 60.0, 65.0];

/// How a sweep row turns a model into rate gains.
#[derive(Clone, Copy, Debug)]
enum Design {
    /// PX4's pole placement for a critically damped response at [`POLE_PLACEMENT_RISE_S`],
    /// mapped to parallel gains, the start's rate I kept. It assumes one tick of delay.
    PolePlacement,
    /// Loop shaping: D/P at the fitted lag (the PD's zero on the motor lag), P for this phase
    /// margin in degrees on the model's response, delay included; the start's rate I kept.
    LoopShaping(f32),
}

/// Rate gains from `model`, keeping `start`'s I; `None` when the rule has no design.
fn rate_gains(design: Design, model: &RateModel, start: Gains) -> Option<Gains> {
    match design {
        Design::PolePlacement => Some(pole_placement(model, start)),
        Design::LoopShaping(margin_deg) => loop_shaping(model, start, margin_deg),
    }
}

/// Generalized minimum variance PID (Yamamoto): closed-loop poles `1 + t1·z⁻¹ + t2·z⁻²` from
/// the rise time at critical damping, the controller polynomial `f0 + f1·z⁻¹ + f2·z⁻²`
/// solving `(1 − z⁻¹)·A + z⁻¹·F = T`, scaled by the model's static gain `b0 + b1 + b2`: P is
/// `−f1 − 2·f2`, D `f2·dt` (I, `f0 + f1 + f2` over `dt`, gives way to the start's).
fn pole_placement(model: &RateModel, start: Gains) -> Gains {
    let rho = DT / POLE_PLACEMENT_RISE_S;
    let t1 = -2.0 * (-2.0 * rho).exp();
    let t2 = (-4.0 * rho).exp();
    // f0 = t1 − a1 + 1 sets I, which the start's replaces.
    let _ = t1;
    let f1 = t2 - model.a2 + model.a1;
    let f2 = model.a2;
    let static_gain = model.b.iter().sum::<f32>();
    let scale = POLE_PLACEMENT_DETUNE / static_gain;
    Gains {
        p: scale * (-f1 - 2.0 * f2),
        i: start.i,
        d: (scale * f2 * DT).max(0.0),
    }
}

/// D/P at the fitted lag; P puts the crossover where the phase of `C·G` is `margin_deg` above
/// −180°, `C = 1 + (D/P)·(1 − z⁻¹)/dt`.
fn loop_shaping(model: &RateModel, start: Gains, margin_deg: f32) -> Option<Gains> {
    let ratio_s = model.lag_s()?;
    let response = |w_radps: f32| {
        let z1 = Complex::new(0.0, -w_radps * DT).exp();
        let numerator = (Complex::from(model.b[0]) + z1 * model.b[1] + z1 * z1 * model.b[2])
            * z1.powu(model.delay_ticks as u32);
        let denominator = Complex::from(1.0) + z1 * model.a1 + z1 * z1 * model.a2;
        let controller = Complex::from(1.0) + (Complex::from(1.0) - z1) * (ratio_s / DT);
        controller * numerator / denominator
    };
    // Walk up in frequency, unwrapping the phase, to the last point above the margin.
    const POINTS: usize = 2000;
    let nyquist_radps = PI / DT;
    let (mut unwrapped, mut last, mut crossover_radps) = (0.0f32, 0.0f32, None);
    for i in 1..POINTS {
        let w_radps = nyquist_radps * i as f32 / POINTS as f32;
        let mut phase = response(w_radps).argument() + unwrapped;
        while phase - last > PI {
            unwrapped -= 2.0 * PI;
            phase -= 2.0 * PI;
        }
        while phase - last < -PI {
            unwrapped += 2.0 * PI;
            phase += 2.0 * PI;
        }
        last = phase;
        if phase.to_degrees() < margin_deg - 180.0 {
            break;
        }
        crossover_radps = Some(w_radps);
    }
    let p = 1.0 / response(crossover_radps?).modulus();
    Some(Gains {
        p,
        i: start.i,
        d: ratio_s * p,
    })
}

/// The largest angle P in the angle search's range whose model steps, on the designed rate
/// loop, meet the angle criterion within its clip allowance; backed off 10% as the search's.
fn angle_p(model: &RateModel, config: &CopterStabilizerConfig, axis: usize) -> Option<f32> {
    let plan = SearchPlan::copter_angle();
    let rate = if axis == 0 {
        config.body_rate.roll_gains
    } else {
        config.body_rate.pitch_gains
    };
    let angle = if axis == 0 {
        config.attitude.roll_gains
    } else {
        config.attitude.pitch_gains
    };
    let max_rate_radps = config.attitude.max_rate_radps[axis];
    let passes = |p: f32| {
        let loops = ModelLoops {
            rate: (rate, config.body_rate.max_integral),
            angle: Some((
                Gains { p, ..angle },
                config.attitude.max_integral,
                max_rate_radps,
            )),
        };
        let (steps, clipped_s) = loops.steps(model, plan.test.step);
        plan.criterion.passes(&steps) && clipped_s <= plan.test.max_saturated_s
    };
    largest(plan.min, plan.max, passes).map(|p| p * 0.9)
}

/// The largest value in `[low, high]` that passes, by geometric bisection; `None` if `low`
/// fails.
fn largest(low: f32, high: f32, passes: impl Fn(f32) -> bool) -> Option<f32> {
    if !passes(low) {
        return None;
    }
    if passes(high) {
        return Some(high);
    }
    let (mut pass, mut fail) = (low, high);
    for _ in 0..25 {
        let mid = (pass * fail).sqrt();
        if passes(mid) {
            pass = mid;
        } else {
            fail = mid;
        }
    }
    Some(pass)
}

/// The rate loop (gains, `max_integral`) and optionally the angle loop around it (gains,
/// `max_integral`, rate cap in rad/s), flown on a [`RateModel`].
struct ModelLoops {
    rate: (Gains, f32),
    angle: Option<(Gains, f32, f32)>,
}

impl ModelLoops {
    /// Both-way steps through core's step probe: the summary and the longest clipped time, s.
    fn steps(&self, model: &RateModel, step: StepConfig) -> (StepSummary, f32) {
        let mut summary = StepSummary::default();
        let mut longest_clipped_s = 0f32;
        for config in [step, step.reversed()] {
            let mut plant = ModelPlant::new(*model);
            let mut rate_pid = Pid::new(self.rate);
            let mut angle_pid = self.angle.map(|(gains, max_i, _)| Pid::new((gains, max_i)));
            let max_rate_radps = self.angle.map_or(f32::INFINITY, |angle| angle.2);
            let mut probe = StepProbe::new(config);
            let (mut rate_radps, mut angle_rad, mut clipped) = (0.0f32, 0.0f32, false);
            loop {
                let measured = if self.angle.is_some() {
                    angle_rad
                } else {
                    rate_radps
                };
                let setpoint = probe.step(DT, measured, clipped);
                match probe.status() {
                    StepStatus::Running { .. } => {}
                    StepStatus::Done(metrics) => {
                        summary.add(&metrics);
                        longest_clipped_s = longest_clipped_s.max(metrics.saturated_s);
                        break;
                    }
                    StepStatus::Aborted(_) => break,
                }
                let rate_setpoint_radps = match angle_pid.as_mut() {
                    Some(pid) => pid
                        .step(setpoint - angle_rad)
                        .clamp(-max_rate_radps, max_rate_radps),
                    None => setpoint,
                };
                let demand = rate_pid.step(rate_setpoint_radps - rate_radps);
                let moment = demand.clamp(-1.0, 1.0);
                clipped = moment != demand;
                rate_radps = plant.step(moment);
                angle_rad += rate_radps * DT;
            }
        }
        (summary, longest_clipped_s)
    }
}

/// A [`RateModel`] driven tick by tick.
struct ModelPlant {
    model: RateModel,
    rates_radps: [f32; 2],
    moments: VecDeque<f32>,
}

impl ModelPlant {
    fn new(model: RateModel) -> Self {
        ModelPlant {
            model,
            rates_radps: [0.0; 2],
            moments: VecDeque::from(vec![0.0; model.delay_ticks + 2]),
        }
    }

    /// Applies this tick's moment; the next tick's rate, in rad/s.
    fn step(&mut self, moment: f32) -> f32 {
        let d = self.model.delay_ticks;
        self.moments.push_front(moment);
        self.moments.truncate(d + 2);
        let delayed = |i: usize| self.moments[d - 1 + i];
        let next_radps = -self.model.a1 * self.rates_radps[0] - self.model.a2 * self.rates_radps[1]
            + self.model.b[0] * delayed(0)
            + self.model.b[1] * delayed(1)
            + self.model.b[2] * delayed(2);
        self.rates_radps = [next_radps, self.rates_radps[0]];
        next_radps
    }
}

/// Core's PID form (P, I, D on the error; the integral clamped), at [`DT`].
struct Pid {
    gains: Gains,
    max_integral: f32,
    integral: f32,
    previous_error: Option<f32>,
}

impl Pid {
    fn new((gains, max_integral): (Gains, f32)) -> Self {
        Pid {
            gains,
            max_integral,
            integral: 0.0,
            previous_error: None,
        }
    }

    fn step(&mut self, error: f32) -> f32 {
        self.integral = (self.integral + error * DT).clamp(-self.max_integral, self.max_integral);
        let derivative = self
            .previous_error
            .map_or(0.0, |previous| (error - previous) / DT);
        self.previous_error = Some(error);
        self.gains.p * error + self.gains.i * self.integral + self.gains.d * derivative
    }
}

// ---- Rows ----------------------------------------------------------------------------------

/// A row's departure from the clean plant, as `robustness`'s sets.
#[derive(Clone, Copy, Debug)]
enum Set {
    Clean,
    Delay10Ms,
    Delay25Ms,
    Noise,
    CgOffset,
    WeakMotor,
}

fn perturbations(set: Set, airframe: &Airframe) -> Perturbations {
    let delay = |actuator_delay_s| Perturbations {
        actuator_delay_s,
        ..Default::default()
    };
    match set {
        Set::Clean => Perturbations::default(),
        Set::Delay10Ms => delay(0.01),
        Set::Delay25Ms => delay(0.025),
        Set::Noise => Perturbations {
            gyro: GyroNoise {
                rms_radps: Vec3::repeat(0.02),
                vibration: Some(Vibration {
                    amplitude_radps: Vec3::repeat(0.1),
                    frequency_hz: 90.0,
                }),
                seed: 1,
            },
            ..Default::default()
        },
        Set::CgOffset => {
            let moment_nm = airframe.model.mass_kg * GRAVITY * 0.01;
            Perturbations {
                disturbance_moment_nm: Vec3::new(moment_nm, moment_nm, 0.0),
                ..Default::default()
            }
        }
        Set::WeakMotor => {
            let mut motor_thrust_scale = [1.0; ActuatorCommand::SLOTS];
            motor_thrust_scale[0] = 0.9;
            Perturbations {
                motor_thrust_scale,
                ..Default::default()
            }
        }
    }
}

/// The gains a row starts from, as `robustness`'s.
#[derive(Clone, Copy, Debug)]
enum Start {
    /// Core's default gains on every tuned loop.
    Low,
    /// The airframe's rate P and D × 3; core's default angle gains.
    High,
}

fn starting(airframe: Airframe, start: Start) -> Airframe {
    let defaults = CopterStabilizerConfig::default();
    common::retuned(airframe, |target, gains| match (start, target.is_rate()) {
        (Start::High, true) => Gains {
            p: gains.p * 3.0,
            d: gains.d * 3.0,
            ..gains
        },
        _ => target.gains(&defaults),
    })
}

/// `airframe` under `set` from `start`.
fn plant(airframe: Airframe, set: Set, start: Start) -> Plant {
    Plant {
        airframe: starting(airframe, start),
        wind: Wind::default(),
        perturbations: perturbations(set, &airframe),
    }
}

/// Spec misses of `config` flown on `plant`'s airframe and perturbations.
fn misses(plant: &Plant, config: CopterStabilizerConfig, loops: &[TunedLoop]) -> usize {
    let tuned = Plant {
        airframe: Airframe {
            stabilizer: config,
            ..plant.airframe
        },
        ..*plant
    };
    spec::misses(&tuned, loops).len()
}

/// Identified models, then each design's gains and spec misses, per row.
#[test]
#[ignore = "method sweep (#312)"]
fn identified_designs_meet_the_spec() {
    let designs = [Design::PolePlacement]
        .into_iter()
        .chain(PHASE_MARGINS_DEG.into_iter().map(Design::LoopShaping));
    let designs: Vec<Design> = designs.collect();
    let rows = [
        (Set::Clean, Start::Low),
        (Set::Clean, Start::High),
        (Set::Delay10Ms, Start::Low),
        (Set::Delay25Ms, Start::Low),
        (Set::Noise, Start::Low),
        (Set::CgOffset, Start::Low),
        (Set::WeakMotor, Start::Low),
    ];
    for (name, airframe) in airframes() {
        for (set, start) in rows {
            let plant = plant(airframe, set, start);
            let models = identify(&plant);
            let lag_ms = models[0].lag_s().map(|s| s * 1000.0);
            print!(
                "{name} {set:?} {start:?}: delay {} ticks, lag {lag_ms:.1?} ms |",
                models[0].delay_ticks
            );
            for design in &designs {
                print!(" {design:?}: {} |", designed(&plant, &models, *design));
            }
            println!();
        }
    }
}

/// One design's rate and angle gains on `models`, and its spec misses, as a table cell.
fn designed(plant: &Plant, models: &[RateModel; 2], design: Design) -> String {
    let mut config = plant.airframe.stabilizer;
    for (axis, model) in models.iter().enumerate() {
        let (rate_loop, angle_loop) = if axis == 0 {
            (TunedLoop::RollRate, TunedLoop::Roll)
        } else {
            (TunedLoop::PitchRate, TunedLoop::Pitch)
        };
        let Some(rate) = rate_gains(design, model, rate_loop.gains(&config)) else {
            return "no rate design".into();
        };
        *rate_loop.gains_mut(&mut config) = rate;
        let Some(p) = angle_p(model, &config, axis) else {
            return "no angle design".into();
        };
        angle_loop.gains_mut(&mut config).p = p;
    }
    let rate = config.body_rate.roll_gains;
    format!(
        "rate P {:.2} D {:.4}, angle P {:.1}, {} misses",
        rate.p,
        rate.d,
        config.attitude.roll_gains.p,
        misses(plant, config, &spec::TUNED)
    )
}

// ---- Gain ramp (QuikTune) ------------------------------------------------------------------

/// Seconds for a ramped gain to double (QuikTune `DOUBLE_TIME`, lua:195).
const DOUBLE_S: f32 = 10.0;

/// Share of the oscillating gain kept (1 − `GAIN_MARGIN` 60%, lua:616-620).
const KEPT_SHARE: f32 = 0.4;

/// Least share of its start a gain can end at (1 − `MAX_REDUCE` 20%, lua:432-445).
const LEAST_SHARE: f32 = 0.8;

/// Pause after each gain, in s (lua:196).
const STAGE_PAUSE_S: f32 = 4.0;

/// The dither a clean plant needs to oscillate on, in rad and Hz: a square wave's edges kick
/// the angle loop's D and trip the detector at once.
const DITHER_RAD: f32 = 0.5 * PI / 180.0;
const DITHER_HZ: f32 = 2.0;

/// Slew thresholds swept, in moment per second (QuikTune's `OSC_SMAX` default is 5).
const SLEW_THRESHOLDS: [f32; 4] = [5.0, 20.0, 40.0, 80.0];

/// ArduPilot's slew measure of a PID output (`SlewLimiter.cpp:31-81`): its rate through a
/// 25 Hz low-pass; positive and negative peaks each held 300 ms, then decaying over 1 s; their
/// mean, risen into at `2·dt/τ` and capped at the raw mean.
#[derive(Default)]
struct Slew {
    previous: Option<f32>,
    filtered: f32,
    positive: (f32, f32),
    negative: (f32, f32),
    smoothed: f32,
    now_s: f32,
}

impl Slew {
    const TAU_S: f32 = 1.0;
    const HOLD_S: f32 = 0.3;
    const FILTER_HZ: f32 = 25.0;

    fn step(&mut self, output: f32) -> f32 {
        self.now_s += DT;
        let rate = self
            .previous
            .map_or(0.0, |previous| (output - previous) / DT);
        self.previous = Some(output);
        self.filtered += (rate - self.filtered) * (1.0 - (-2.0 * PI * Self::FILTER_HZ * DT).exp());
        for (peak, sample) in [
            (&mut self.positive, self.filtered),
            (&mut self.negative, -self.filtered),
        ] {
            if self.now_s - peak.1 > Self::HOLD_S {
                peak.0 *= 1.0 - DT / Self::TAU_S;
            }
            if sample > peak.0 {
                *peak = (sample, self.now_s);
            }
        }
        let raw = 0.5 * (self.positive.0 + self.negative.0);
        self.smoothed = (self.smoothed + (raw - self.smoothed) * (2.0 * DT / Self::TAU_S)).min(raw);
        self.smoothed
    }
}

/// QuikTune on roll then pitch rate, D then P each, at `threshold`: the tuned config and the
/// simulated seconds it took.
fn quiktune(plant: &Plant, threshold: f32) -> (CopterStabilizerConfig, f32) {
    const TIMEOUT_S: f32 = 600.0;
    let drive = Rc::new(Cell::new(Drive::default()));
    let recorder = Recorder::default();
    let mut flight_loop = excitable(plant, &drive, &recorder);
    let mut config = plant.airframe.stabilizer;
    let mut elapsed_s = 0.0;
    recorder.on.set(true);
    for (axis, target) in [(0, TunedLoop::RollRate), (1, TunedLoop::PitchRate)] {
        let start = target.gains(&config);
        for term_is_d in [true, false] {
            drive.set(Drive {
                axis: Some(axis),
                amplitude_rad: DITHER_RAD,
                sine_hz: Some(DITHER_HZ),
                elapsed_s: 0.0,
            });
            let start_value = if term_is_d { start.d } else { start.p };
            let set = |config: &mut CopterStabilizerConfig, value: f32| {
                let gains = target.gains_mut(config);
                if term_is_d {
                    gains.d = value;
                } else {
                    gains.p = value;
                }
            };
            let mut slew = Slew::default();
            let mut value = start_value.max(0.001);
            loop {
                value *= 2f32.powf(DT / DOUBLE_S);
                set(&mut config, value);
                flight_loop
                    .set_stabilizer_config(config)
                    .expect("valid gains");
                flight_loop.step(TICK);
                elapsed_s += DT;
                let moment = recorder.ticks.borrow().last().map_or(0.0, |t| t[axis].0);
                // A non-finite slew is an oscillation too.
                let slew_now = slew.step(moment);
                let oscillating = !slew_now.is_finite() || slew_now > threshold;
                if oscillating || elapsed_s > TIMEOUT_S {
                    set(
                        &mut config,
                        (value * KEPT_SHARE).max(start_value * LEAST_SHARE),
                    );
                    flight_loop
                        .set_stabilizer_config(config)
                        .expect("valid gains");
                    break;
                }
            }
            drive.set(Drive::default());
            fly(&mut flight_loop, STAGE_PAUSE_S, |_, _| {});
            elapsed_s += STAGE_PAUSE_S;
        }
        // P follows a D that ended below its start (lua:626-635).
        let gains = target.gains_mut(&mut config);
        if start.d > 0.0 && gains.d < start.d {
            gains.p *= (gains.d / start.d).max(0.5);
        }
    }
    (config, elapsed_s)
}

/// QuikTune at each slew threshold: time, tuned rate gains, and spec misses of the rate loops
/// (it tunes no angle loop).
#[test]
#[ignore = "method sweep (#312)"]
fn quiktune_from_low_and_high_starts() {
    let rate_loops = [TunedLoop::RollRate, TunedLoop::PitchRate];
    for threshold in SLEW_THRESHOLDS {
        for (name, airframe) in airframes() {
            for set in [Set::Clean, Set::Delay25Ms] {
                for start in [Start::Low, Start::High] {
                    let plant = plant(airframe, set, start);
                    let (config, elapsed_s) = quiktune(&plant, threshold);
                    let rate = config.body_rate.roll_gains;
                    println!(
                        "slew {threshold} {name} {set:?} {start:?}: {elapsed_s:.0} s, rate P {:.2} D {:.4}, {} rate misses",
                        rate.p,
                        rate.d,
                        misses(&plant, config, &rate_loops)
                    );
                }
            }
        }
    }
}
