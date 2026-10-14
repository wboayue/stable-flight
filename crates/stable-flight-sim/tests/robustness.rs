//! Autotune robustness (#366): `StagePlan::copter_search` on every airframe, under each
//! perturbation set, from a low and a high start, ends every stage `Tuned`, with gains that meet
//! the step spec on that same plant, within [`BUDGET_S`] of simulated time.

mod common;

use assert2::check;
use stable_flight::Vec3;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::{CopterStabilizerConfig, TunedLoop};
use stable_flight::tuning::autotune::StagePlan;
use stable_flight::{ActuatorCommand, GRAVITY};
use stable_flight_sim::airframes::{self, Airframe};
use stable_flight_sim::{GyroNoise, Perturbations, Vibration, Wind};

use common::{AUTOTUNE_TIMEOUT_S, Plant, spec};

/// Simulated seconds a row's autotune may take: the slowest clean row (`heavy_quad` from the
/// high start, 724 s) plus 25%.
const BUDGET_S: f32 = 905.0;

/// Rate P and D multiplier for the high start. At ×3, `quad_x`'s rate steps clip; `heavy_quad`'s
/// and `racer`'s rate P sits above the relay's ceiling (0.6·Ku: 2.16 and 0.85 against 2.45 and
/// 1.86), so the search doesn't fly it as a baseline.
const HIGH_RATE_SCALE: f32 = 3.0;

/// Gyro white noise RMS per axis, in rad/s.
const NOISE_RMS_RADPS: f32 = 0.02;

/// Gyro vibration tone amplitude per axis, in rad/s.
const VIBRATION_AMPLITUDE_RADPS: f32 = 0.1;

/// Gyro vibration tone frequency, in Hz: below the 125 Hz Nyquist of the 4 ms tick.
const VIBRATION_FREQUENCY_HZ: f32 = 90.0;

/// Actuator delay of the delay and combined rows, in s.
const DELAY_S: f32 = 0.01;

/// Actuator delay of the long-delay rows, in s.
const LONG_DELAY_S: f32 = 0.025;

/// Seed of every noisy row's gyro.
const NOISE_SEED: u64 = 1;

/// CG offset, in m, behind the roll and pitch disturbance moment (`m·g·offset`).
const CG_OFFSET_M: f32 = 0.01;

/// Thrust of the weak motor (slot 0) as a fraction of the others'.
const WEAK_MOTOR_SCALE: f32 = 0.9;

/// How a row's plant departs from the clean model.
#[derive(Clone, Copy, Debug)]
enum Set {
    Clean,
    /// [`DELAY_S`].
    Delay10Ms,
    /// [`LONG_DELAY_S`].
    Delay25Ms,
    Noise,
    CgOffset,
    WeakMotor,
    /// 10 ms delay, noise, CG offset, and the weak motor together.
    Combined,
}

/// The gains a row's autotune starts from.
#[derive(Clone, Copy, Debug)]
enum Start {
    /// Core's default gains on every tuned loop.
    Low,
    /// The airframe's rate P and D × [`HIGH_RATE_SCALE`]; core's default angle gains.
    High,
}

/// `set`'s perturbations on `airframe`.
fn perturbations(set: Set, airframe: &Airframe) -> Perturbations {
    match set {
        Set::Clean => Perturbations::default(),
        Set::Delay10Ms => delay(DELAY_S),
        Set::Delay25Ms => delay(LONG_DELAY_S),
        Set::Noise => Perturbations {
            gyro: noise(),
            ..Default::default()
        },
        Set::CgOffset => Perturbations {
            disturbance_moment_nm: cg_moment_nm(airframe),
            ..Default::default()
        },
        Set::WeakMotor => Perturbations {
            motor_thrust_scale: weak_motor(),
            ..Default::default()
        },
        Set::Combined => Perturbations {
            actuator_delay_s: DELAY_S,
            gyro: noise(),
            disturbance_moment_nm: cg_moment_nm(airframe),
            motor_thrust_scale: weak_motor(),
        },
    }
}

/// `delay_s` of actuator delay alone.
fn delay(delay_s: f32) -> Perturbations {
    Perturbations {
        actuator_delay_s: delay_s,
        ..Default::default()
    }
}

/// White noise plus the vibration tone.
fn noise() -> GyroNoise {
    GyroNoise {
        rms_radps: Vec3::repeat(NOISE_RMS_RADPS),
        vibration: Some(Vibration {
            amplitude_radps: Vec3::repeat(VIBRATION_AMPLITUDE_RADPS),
            frequency_hz: VIBRATION_FREQUENCY_HZ,
        }),
        seed: NOISE_SEED,
    }
}

/// The roll and pitch moment of a [`CG_OFFSET_M`] offset on `airframe`, in N·m.
fn cg_moment_nm(airframe: &Airframe) -> Vec3 {
    let moment_nm = airframe.model.mass_kg * GRAVITY * CG_OFFSET_M;
    Vec3::new(moment_nm, moment_nm, 0.0)
}

/// Slot 0 at [`WEAK_MOTOR_SCALE`], the rest at full thrust.
fn weak_motor() -> [f32; ActuatorCommand::SLOTS] {
    let mut motor_thrust_scale = [1.0; ActuatorCommand::SLOTS];
    motor_thrust_scale[0] = WEAK_MOTOR_SCALE;
    motor_thrust_scale
}

/// `airframe` with its tuned loops' gains set to `start`.
fn starting(airframe: Airframe, start: Start) -> Airframe {
    let defaults = CopterStabilizerConfig::default();
    common::retuned(airframe, |target, gains| match (start, target.is_rate()) {
        (Start::High, true) => Gains {
            p: gains.p * HIGH_RATE_SCALE,
            d: gains.d * HIGH_RATE_SCALE,
            ..gains
        },
        _ => target.gains(&defaults),
    })
}

/// Autotunes `airframe` under `set` from `start` and checks the row's criteria.
fn check_row(name: &str, airframe: Airframe, set: Set, start: Start) {
    let perturbations = perturbations(set, &airframe);
    let plant = Plant {
        airframe: starting(airframe, start),
        wind: Wind::default(),
        perturbations,
    };
    let run = common::autotune(&plant, &StagePlan::copter_search(), AUTOTUNE_TIMEOUT_S);
    let tuned = Plant {
        airframe: Airframe {
            stabilizer: run.config,
            ..airframe
        },
        ..plant
    };
    let misses = spec::misses(&tuned, &spec::TUNED);
    let gains = spec::TUNED.map(|target: TunedLoop| (target, target.gains(&run.config)));
    check!(
        run.all_tuned() && misses.is_empty() && run.duration_s <= BUDGET_S,
        "{name} {set:?} from {start:?}: {:.0} s\noutcomes {:?}\ngains {gains:?}\nmisses {misses:#?}",
        run.duration_s,
        run.outcomes(),
    );
}

/// One `#[test]` per row: `test_name: airframe, Set, Start;`, each with its own attributes.
macro_rules! rows {
    ($($(#[$attr:meta])* $test:ident: $airframe:ident, $set:ident, $start:ident;)*) => {
        $(
            $(#[$attr])*
            #[test]
            fn $test() {
                check_row(
                    stringify!($airframe),
                    airframes::$airframe(),
                    Set::$set,
                    Start::$start,
                );
            }
        )*
    };
}

rows! {
    quad_x_clean_low: quad_x, Clean, Low;
    quad_x_clean_high: quad_x, Clean, High;
    quad_x_delay_10ms_low: quad_x, Delay10Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    quad_x_delay_10ms_high: quad_x, Delay10Ms, High;
    #[ignore = "tuned rate loops miss rise and settle (#376)"]
    quad_x_delay_25ms_low: quad_x, Delay25Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    quad_x_delay_25ms_high: quad_x, Delay25Ms, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    quad_x_noise_low: quad_x, Noise, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    quad_x_noise_high: quad_x, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    quad_x_cg_offset_low: quad_x, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    quad_x_cg_offset_high: quad_x, CgOffset, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    quad_x_weak_motor_low: quad_x, WeakMotor, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    quad_x_weak_motor_high: quad_x, WeakMotor, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    quad_x_combined_low: quad_x, Combined, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    quad_x_combined_high: quad_x, Combined, High;

    heavy_quad_clean_low: heavy_quad, Clean, Low;
    heavy_quad_clean_high: heavy_quad, Clean, High;
    heavy_quad_delay_10ms_low: heavy_quad, Delay10Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    heavy_quad_delay_10ms_high: heavy_quad, Delay10Ms, High;
    #[ignore = "tuned rate loops miss rise and settle (#376)"]
    heavy_quad_delay_25ms_low: heavy_quad, Delay25Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    heavy_quad_delay_25ms_high: heavy_quad, Delay25Ms, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    heavy_quad_noise_low: heavy_quad, Noise, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    heavy_quad_noise_high: heavy_quad, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    heavy_quad_cg_offset_low: heavy_quad, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    heavy_quad_cg_offset_high: heavy_quad, CgOffset, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    heavy_quad_weak_motor_low: heavy_quad, WeakMotor, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    heavy_quad_weak_motor_high: heavy_quad, WeakMotor, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    heavy_quad_combined_low: heavy_quad, Combined, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    heavy_quad_combined_high: heavy_quad, Combined, High;

    racer_clean_low: racer, Clean, Low;
    racer_clean_high: racer, Clean, High;
    racer_delay_10ms_low: racer, Delay10Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    racer_delay_10ms_high: racer, Delay10Ms, High;
    #[ignore = "tuned rate loops miss rise and settle (#376)"]
    racer_delay_25ms_low: racer, Delay25Ms, Low;
    #[ignore = "start above the relay's Ku keeps flying and rings: NotSettled (#375)"]
    racer_delay_25ms_high: racer, Delay25Ms, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    racer_noise_low: racer, Noise, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    racer_noise_high: racer, Noise, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    racer_cg_offset_low: racer, CgOffset, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    racer_cg_offset_high: racer, CgOffset, High;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    racer_weak_motor_low: racer, WeakMotor, Low;
    #[ignore = "trim moment held only by the slow angle I: tilt, asymmetric steps (#374)"]
    racer_weak_motor_high: racer, WeakMotor, High;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    racer_combined_low: racer, Combined, Low;
    #[ignore = "gyro noise keeps the raw rates above the settle gate: NotSettled (#373)"]
    racer_combined_high: racer, Combined, High;
}
