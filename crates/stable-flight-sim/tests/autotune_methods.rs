//! The autotune method comparison behind `architecture.md#autotune` (#312): where the step
//! search's time goes, and three faster methods on the same plants, each held to the
//! `step_response` spec on its own plant. Every test is an `#[ignore]`d sweep that prints a
//! table; none asserts.
//!
//! ```text
//! cargo test --release -p stable-flight-sim --test autotune_methods -- --ignored --nocapture --test-threads 1
//! ```
//!
//! - **Identification:** core's `IdentifyProbe` on the roll and pitch angle loops
//!   (`COPTER_IDENTIFY`): PX4's shrinking square wave on the setpoint, every loop closed, and a
//!   least-squares fit of the axis's body rate to its moment command with a delay.
//! - **Design** from the model: PX4's pole placement (`pid_design.hpp:68-103`), or loop
//!   shaping (the rate PD's zero on the fitted lag, P for a phase margin). Angle P from the
//!   model on the designed rate loop.
//! - **Gain ramp:** ArduPilot's QuikTune (`AP_Scripting/applets/VTOL-quicktune.lua` at
//!   `e96bd90`): D then P raised until the rate PID's output slew oscillates
//!   (`Filter/SlewLimiter.cpp:31-81`), then backed off.
//!
//! Each is described from its source and implemented here from that description. The model
//! steps fly core's attitude and body-rate controllers, so a design is judged on the law the
//! vehicle flies.
//!
//! TODO(#517): design through core's loop-shaping function once it lands.

mod common;

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::f32::consts::PI;
use std::rc::Rc;

use stable_flight::controllers::{AttitudeController, BodyRateController, Gains};
use stable_flight::guidance::copter::CopterGuidance;
use stable_flight::nalgebra::{Complex, ComplexField};
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::{
    Autotune, AutotuneEvent, COPTER_IDENTIFY, Envelope, SearchPlan, StagePlan,
};
use stable_flight::tuning::{
    Measurement, ProbeConfig, ProbeStatus, RateModel, StepConfig, StepProbe, StepStatus,
    StepSummary,
};
use stable_flight::{
    ControlInput, FlightLoop, FlightModeReport, FlightRegime, Guidance, Saturation, Setpoint,
    TelemetryPublisher, TelemetryRecord, Vec2, Vec3, VehicleState,
};
use stable_flight_sim::Wind;
use stable_flight_sim::airframes::{self, Airframe};

use common::robustness::{Set, Start, perturbations, starting};
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

// ---- Identification ------------------------------------------------------------------------

/// Simulated seconds an identification may run: its 3 s plus margin.
const IDENTIFY_TIMEOUT_S: f32 = 5.0;

/// Roll's then pitch's model, each from core's identification probe on its angle loop
/// ([`COPTER_IDENTIFY`]) from a settled hover.
fn identify(plant: &Plant) -> [RateModel; 2] {
    [TunedLoop::Roll, TunedLoop::Pitch].map(|target| {
        let config = ProbeConfig::Identify(COPTER_IDENTIFY);
        match common::probe(plant, target, config, IDENTIFY_TIMEOUT_S) {
            Some(ProbeStatus::Done(Measurement::Identify(model))) => model,
            other => panic!("{target:?}: {other:?}"),
        }
    })
}

/// `model`'s denominator `1 + a1·z⁻¹ + a2·z⁻²` coefficients: `a = −r`, `r` its rate weights.
fn denominator(model: &RateModel) -> (f32, f32) {
    (-model.rate_weights[0], -model.rate_weights[1])
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
    // t1 = −2·e^(−2ρ) enters only f0, which sets I.
    let t2 = (-4.0 * rho).exp();
    let (a1, a2) = denominator(model);
    let f1 = t2 - a2 + a1;
    let f2 = a2;
    let static_gain = model.moment_weights.iter().sum::<f32>();
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
        let m = model.moment_weights;
        let numerator =
            (Complex::from(m[0]) + z1 * m[1] + z1 * z1 * m[2]) * z1.powu(model.delay_ticks as u32);
        let (a1, a2) = denominator(model);
        let denominator = Complex::from(1.0) + z1 * a1 + z1 * z1 * a2;
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

/// The largest angle P in the angle search's range whose model steps, on `config`'s rate
/// loop, meet the angle criterion within its clip allowance; backed off 10% as the search's.
fn angle_p(
    model: &RateModel,
    config: &CopterStabilizerConfig,
    angle_loop: TunedLoop,
) -> Option<f32> {
    let plan = SearchPlan::copter_angle();
    let passes = |p: f32| {
        let mut candidate = *config;
        angle_loop.gains_mut(&mut candidate).p = p;
        let (steps, clipped_s) = model_steps(model, &candidate, angle_loop, plan.test.step);
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

/// Both-way steps of `target`, a roll or pitch rate or angle loop, on `model`: core's step
/// probe on core's attitude and body-rate controllers with `config`, the other axes held at
/// zero. The summary and the longest clipped time, in s.
fn model_steps(
    model: &RateModel,
    config: &CopterStabilizerConfig,
    target: TunedLoop,
    step: StepConfig,
) -> (StepSummary, f32) {
    let axis = usize::from(matches!(target, TunedLoop::PitchRate | TunedLoop::Pitch));
    let angle_loop = matches!(target, TunedLoop::Roll | TunedLoop::Pitch);
    let on_axis2 = |value: f32| Vec2::from_fn(|i, _| if i == axis { value } else { 0.0 });
    let on_axis3 = |value: f32| Vec3::from_fn(|i, _| if i == axis { value } else { 0.0 });
    let mut summary = StepSummary::default();
    let mut longest_clipped_s = 0f32;
    for step in [step, step.reversed()] {
        let mut plant = ModelPlant::new(*model);
        let mut attitude = AttitudeController::default();
        let mut body_rate = BodyRateController::default();
        let mut probe = StepProbe::new(step);
        let (mut rate_radps, mut angle_rad, mut clipped) = (0.0f32, 0.0f32, false);
        loop {
            let measured = if angle_loop { angle_rad } else { rate_radps };
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
            let rate_setpoint_radps = if angle_loop {
                let rates = attitude.control(
                    &config.attitude,
                    DT,
                    on_axis2(setpoint),
                    on_axis2(angle_rad),
                );
                rates[axis]
            } else {
                setpoint
            };
            let moments = body_rate.control(
                &config.body_rate,
                DT,
                on_axis3(rate_setpoint_radps),
                on_axis3(rate_radps),
            );
            // Core saturates the moment at ±1: a step there is clipped.
            clipped = moments[axis].abs() >= 1.0;
            rate_radps = plant.step(moments[axis]);
            angle_rad += rate_radps * DT;
        }
    }
    (summary, longest_clipped_s)
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
        let [r0, r1] = self.model.rate_weights;
        let [m0, m1, m2] = self.model.moment_weights;
        let next_radps = r0 * self.rates_radps[0]
            + r1 * self.rates_radps[1]
            + m0 * delayed(0)
            + m1 * delayed(1)
            + m2 * delayed(2);
        self.rates_radps = [next_radps, self.rates_radps[0]];
        next_radps
    }
}

// ---- Rows ----------------------------------------------------------------------------------

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
fn designs_from_identified_models() {
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
            let lag = models[0]
                .lag_s()
                .map_or("no lag".into(), |s| format!("lag {:.1} ms", s * 1000.0));
            let mut row = format!(
                "{name} {set:?} {start:?}: delay {} ticks, {lag} |",
                models[0].delay_ticks
            );
            for design in &designs {
                let cell = match design_config(&plant, &models, *design) {
                    Err(reason) => reason.to_string(),
                    Ok(config) => {
                        let rate = config.body_rate.roll_gains;
                        format!(
                            "rate P {:.2} D {:.4}, angle P {:.1}, {} misses",
                            rate.p,
                            rate.d,
                            config.attitude.roll_gains.p,
                            misses(&plant, config, &spec::TUNED)
                        )
                    }
                };
                row += &format!(" {design:?}: {cell} |");
            }
            println!("{row}");
        }
    }
}

/// `plant`'s stabilizer with roll and pitch rate and angle gains designed from `models`, or
/// which design failed.
fn design_config(
    plant: &Plant,
    models: &[RateModel; 2],
    design: Design,
) -> Result<CopterStabilizerConfig, &'static str> {
    let mut config = plant.airframe.stabilizer;
    let loops = [
        (TunedLoop::RollRate, TunedLoop::Roll),
        (TunedLoop::PitchRate, TunedLoop::Pitch),
    ];
    for (model, (rate_loop, angle_loop)) in models.iter().zip(loops) {
        let rate = rate_gains(design, model, rate_loop.gains(&config)).ok_or("no rate design")?;
        *rate_loop.gains_mut(&mut config) = rate;
        let p = angle_p(model, &config, angle_loop).ok_or("no angle design")?;
        angle_loop.gains_mut(&mut config).p = p;
    }
    Ok(config)
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

/// The dither QuikTune's ramp oscillates on, added to the attitude setpoint this run.
#[derive(Clone, Copy, Debug, Default)]
struct Drive {
    /// 0 roll, 1 pitch; `None` off.
    axis: Option<usize>,
    elapsed_s: f32,
}

/// Copter guidance with a [`DITHER_RAD`] sine at [`DITHER_HZ`] added to the attitude
/// setpoint's roll or pitch while [`Drive`] names an axis.
struct Dithered {
    inner: CopterGuidance,
    drive: Rc<Cell<Drive>>,
}

impl Guidance for Dithered {
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
            return setpoint;
        };
        drive.elapsed_s += dt;
        self.drive.set(drive);
        let signal_rad = DITHER_RAD * (2.0 * PI * DITHER_HZ * drive.elapsed_s).sin();
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

/// Each tick's roll and pitch moment command.
#[derive(Clone, Default)]
struct Recorder {
    moments: Rc<RefCell<Vec<Vec3>>>,
}

impl TelemetryPublisher for Recorder {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.moments.borrow_mut().push(record.control.moments);
    }
}

/// A copter pipeline on [`Dithered`] guidance, recorded by a [`Recorder`].
type DitheredLoop = FlightLoop<
    common::Hover,
    stable_flight_sim::SimCopter,
    Dithered,
    CopterStabilizer,
    stable_flight::mixers::TableMixer<4>,
    Recorder,
>;

/// `plant` hovering on [`Dithered`] guidance, recorded by `recorder`.
fn dithered(plant: &Plant, drive: &Rc<Cell<Drive>>, recorder: &Recorder) -> DitheredLoop {
    let airframe = plant.airframe;
    let mut flight_loop = FlightLoop::new(
        hover_follower(),
        plant.vehicle(),
        Dithered {
            inner: CopterGuidance::new(airframe.guidance),
            drive: drive.clone(),
        },
        CopterStabilizer::new(airframe.stabilizer),
        airframe.mixer,
        recorder.clone(),
    );
    flight_loop.prime();
    fly(&mut flight_loop, HOVER_SETTLE_S, |_, _| {});
    flight_loop
}

/// QuikTune on roll then pitch rate, D then P each, at `threshold`: the tuned config and the
/// simulated seconds it took.
fn quiktune(plant: &Plant, threshold: f32) -> (CopterStabilizerConfig, f32) {
    const STAGE_TIMEOUT_S: f32 = 600.0;
    let drive = Rc::new(Cell::new(Drive::default()));
    let recorder = Recorder::default();
    let mut flight_loop = dithered(plant, &drive, &recorder);
    let mut config = plant.airframe.stabilizer;
    let mut elapsed_s = 0.0;
    for (axis, target) in [(0, TunedLoop::RollRate), (1, TunedLoop::PitchRate)] {
        let start = target.gains(&config);
        for term_is_d in [true, false] {
            drive.set(Drive {
                axis: Some(axis),
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
            let mut stage_s = 0.0;
            loop {
                value *= 2f32.powf(DT / DOUBLE_S);
                set(&mut config, value);
                flight_loop
                    .set_stabilizer_config(config)
                    .expect("valid gains");
                flight_loop.step(TICK);
                elapsed_s += DT;
                stage_s += DT;
                let moment = recorder.moments.borrow().last().map_or(0.0, |m| m[axis]);
                // A non-finite slew is an oscillation too.
                let slew_now = slew.step(moment);
                let oscillating = !slew_now.is_finite() || slew_now > threshold;
                if oscillating || stage_s > STAGE_TIMEOUT_S {
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
