//! Simulated airframes: physics paired with the mixer and flight-controller configs that fly
//! them.

use stable_flight::GRAVITY;
use stable_flight::Vec3;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
    YawControllerConfig,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::{MixerConfig, Slot, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::model::MultirotorModel;

#[cfg(test)]
mod tests;

/// A simulated airframe: its physics, the mixer that lays out its motors, and the guidance and
/// stabilizer configs tuned for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Airframe {
    /// Physical parameters.
    pub model: MultirotorModel,
    /// The mixer; [`SimVehicle`](crate::SimVehicle) takes the motor layout from its rows.
    pub mixer: TableMixer<4>,
    /// Guidance (position-loop) configuration.
    pub guidance: CopterGuidanceConfig,
    /// Stabilizer configuration.
    pub stabilizer: CopterStabilizerConfig,
}

/// A 1 kg X quad, 250 mm motor to motor, thrust-to-weight about 2.4: roughly a 5-inch
/// freestyle quad with a camera, chosen to be plausible, not to match a real one.
///
/// Roll and pitch rate and angle gains are the sim autotune's (`StagePlan::copter_search`,
/// `tests/autotune.rs`); yaw rate and heading are hand-tuned on the sim's step responses
/// (`tests/step_response.rs`). Guidance uses the core defaults, with the hover thrust worked
/// out from the model rather than measured.
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::copter::CopterGuidance;
/// use stable_flight::stabilizers::copter::CopterStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimVehicle, airframes};
///
/// let airframe = airframes::quad_x();
/// let vehicle = SimVehicle::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     CopterGuidance::new(airframe.guidance),
///     CopterStabilizer::new(airframe.stabilizer),
///     airframe.mixer,
///     RecordingTelemetry::<16>::new(),
/// );
/// ```
pub fn quad_x() -> Airframe {
    let model = MultirotorModel {
        mass_kg: 1.0,
        inertia_kgm2: Vec3::new(0.01, 0.01, 0.018),
        // 250 mm motor to motor on the diagonal: 125 mm arms, at 45° to each axis.
        moment_arm_m: 0.125 * core::f32::consts::FRAC_1_SQRT_2,
        max_thrust_n: 6.0,
        motor_time_constant_s: 0.03,
        yaw_torque_per_thrust_m: 0.016,
        drag_n_per_mps: 0.3,
        angular_damping_nm_per_radps: Vec3::new(0.002, 0.002, 0.004),
        ground_friction: 0.5,
        max_step_s: 0.001,
    };
    let mixer = TableMixer::quad_x();
    let defaults = CopterStabilizerConfig::default();
    let angle = |p| Gains {
        p,
        ..defaults.attitude.roll_gains
    };
    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            // Sim autotune (#347); the symmetric model's roll and pitch differ by the search's
            // step size.
            roll_gains: Gains {
                p: 1.95,
                i: 0.0,
                d: 0.0288,
            },
            pitch_gains: Gains {
                p: 1.70,
                i: 0.0,
                d: 0.0258,
            },
            // Prop drag gives yaw about 7 rad/s² per unit moment: P 1.5 rises in 0.16 s on a
            // 0.5 rad/s step without clipping (#347).
            yaw_gains: Gains {
                p: 1.5,
                ..defaults.body_rate.yaw_gains
            },
            ..defaults.body_rate
        },
        // Sim autotune (#347).
        attitude: AttitudeControllerConfig {
            roll_gains: angle(9.0),
            pitch_gains: angle(9.0),
            ..defaults.attitude
        },
        // A 10° heading step without overshoot or clipping on the yaw rate loop above (#347).
        yaw: YawControllerConfig {
            gains: Gains {
                p: 3.5,
                i: 0.0,
                d: 0.0,
            },
            ..defaults.yaw
        },
        ..defaults
    };
    let guidance = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            hover_thrust: hover_thrust(&model, &mixer, &MixerConfig::default(), &stabilizer),
            ..CopterGuidanceConfig::default().altitude
        },
        ..CopterGuidanceConfig::default()
    };
    Airframe {
        model,
        mixer,
        guidance,
        stabilizer,
    }
}

/// The linear thrust setpoint that holds `model` level in a hover on `mixer`'s motors: the
/// motor command whose thrust carries the weight (`max_thrust_n · command²`), undone through
/// the mixer's thrust factor (`mixer_config`, which `TableMixer` doesn't expose) and the
/// stabilizer's thrust curve.
fn hover_thrust<const N: usize>(
    model: &MultirotorModel,
    mixer: &TableMixer<N>,
    mixer_config: &MixerConfig,
    stabilizer: &CopterStabilizerConfig,
) -> f32 {
    let motors = mixer
        .rows()
        .iter()
        .filter(|row| matches!(row.slot(), Slot::Motor(_)))
        .count();
    let motor_n = model.mass_kg * GRAVITY / motors as f32;
    let motor_command = libm::sqrtf(motor_n / model.max_thrust_n);
    libm::powf(
        motor_command / mixer_config.thrust_factor,
        1.0 / stabilizer.thrust_curve_exponent,
    )
}
