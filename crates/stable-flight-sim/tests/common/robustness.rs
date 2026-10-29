//! The autotune robustness table's plants (#366): how a row's plant departs from the clean
//! model ([`Set`]) and the gains it starts from ([`Start`]), shared by `robustness` and the
//! `autotune_methods` sweep.

use stable_flight::Vec3;
use stable_flight::controllers::Gains;
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::{ActuatorCommand, GRAVITY};
use stable_flight_sim::airframes::Airframe;
use stable_flight_sim::{GyroNoise, Perturbations, Vibration};

/// Rate gain multiplier for the high start: at ×3, `quad_x`'s rate steps clip.
pub const HIGH_RATE_SCALE: f32 = 3.0;

/// Gyro white noise RMS per axis, in rad/s.
pub const NOISE_RMS_RADPS: f32 = 0.02;

/// Gyro vibration tone amplitude per axis, in rad/s.
pub const VIBRATION_AMPLITUDE_RADPS: f32 = 0.1;

/// Gyro vibration tone frequency, in Hz: below the 125 Hz Nyquist of the 4 ms tick.
pub const VIBRATION_FREQUENCY_HZ: f32 = 90.0;

/// Actuator delay of the delay and combined rows, in s.
pub const DELAY_S: f32 = 0.01;

/// Actuator delay of the long-delay rows, in s.
pub const LONG_DELAY_S: f32 = 0.025;

/// Seed of every noisy row's gyro.
pub const NOISE_SEED: u64 = 1;

/// CG offset, in m, behind the roll and pitch disturbance moment (`m·g·offset`).
pub const CG_OFFSET_M: f32 = 0.01;

/// Thrust of the weak motor (slot 0) as a fraction of the others'.
pub const WEAK_MOTOR_SCALE: f32 = 0.9;

/// How a row's plant departs from the clean model.
#[derive(Clone, Copy, Debug)]
pub enum Set {
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
pub enum Start {
    /// Core's default gains on every tuned loop.
    Low,
    /// The airframe's rate gains × [`HIGH_RATE_SCALE`]; core's default angle gains.
    High,
}

/// `set`'s perturbations on `airframe`.
pub fn perturbations(set: Set, airframe: &Airframe) -> Perturbations {
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
            ..Default::default()
        },
    }
}

/// `delay_s` of actuator delay alone.
pub fn delay(delay_s: f32) -> Perturbations {
    Perturbations {
        actuator_delay_s: delay_s,
        ..Default::default()
    }
}

/// White noise plus the vibration tone.
pub fn noise() -> GyroNoise {
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
pub fn cg_moment_nm(airframe: &Airframe) -> Vec3 {
    let moment_nm = airframe.model.mass_kg * GRAVITY * CG_OFFSET_M;
    Vec3::new(moment_nm, moment_nm, 0.0)
}

/// Slot 0 at [`WEAK_MOTOR_SCALE`], the rest at full thrust.
pub fn weak_motor() -> [f32; ActuatorCommand::SLOTS] {
    let mut motor_thrust_scale = [1.0; ActuatorCommand::SLOTS];
    motor_thrust_scale[0] = WEAK_MOTOR_SCALE;
    motor_thrust_scale
}

/// `airframe` with its tuned loops' gains set to `start`.
pub fn starting(airframe: Airframe, start: Start) -> Airframe {
    let defaults = CopterStabilizerConfig::default();
    super::retuned(airframe, |target, gains| match (start, target.is_rate()) {
        (Start::High, true) => Gains {
            p: gains.p * HIGH_RATE_SCALE,
            i: gains.i * HIGH_RATE_SCALE,
            d: gains.d * HIGH_RATE_SCALE,
        },
        _ => target.gains(&defaults),
    })
}
