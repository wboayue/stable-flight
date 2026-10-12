//! A table-driven [`Mixer`]: one row per motor or servo, each output a weighted sum of the
//! roll, pitch, and yaw moments, plus collective thrust for motors.
//!
//! Presets cover the common frames ([`TableMixer::quad_x`], [`TableMixer::hex_x`],
//! [`TableMixer::tri`], ...; see [`presets`](self#presets)); [`TableMixer::new`] takes a custom
//! table.
//!
//! # Mixing
//!
//! Moments are saturated to `[-1, 1]` and thrust to `[0, 1]` (NaN → neutral), then:
//!
//! - **Motor rows** get `thrust · thrust_factor` plus `headroom · (roll · r + pitch · p)` and
//!   `headroom · yaw_ratio · yaw · y`, where `r`, `p`, `y` are the row's factors and
//!   `headroom = 1 − thrust_factor` ([`MixerConfig`]). All motor rows are then desaturated
//!   together into `[0, 1]`, airmode-style: the moments are kept and collective thrust gives
//!   way; moments too large to fit at all are scaled down, keeping their ratios. Shifting up is
//!   limited to twice the collective, so it fades out toward idle, and at zero thrust motors
//!   below zero are clipped.
//! - **Servo rows** get `roll · r + pitch · p + yaw · y` over their full range, with no thrust
//!   term, each clamped to `[-1, 1]` on its own.
//!
//! [`Mixer::saturation`] reports `mixer_thrust` when the motors were shifted down, and
//! `mixer_moments` when the motor moments were scaled down or clipped, or a servo was clamped.
//!
//! # Presets
//!
//! Factors follow the frame contract: a positive roll moment rolls right, pitch raises the
//! nose, yaw turns the nose right. Motor roll and pitch factors are normalized per axis so the
//! largest is 1, so gains carry across frames. On multirotors, yaw comes from prop drag: a positive yaw moment
//! speeds up the counterclockwise props (seen from above), whose reaction torque is clockwise
//! (flight-checked in RealFlight on the quad, #62).
//!
//! Motors are numbered clockwise seen from above, starting from the motor on the nose, or the
//! front-left one when none is on the nose; coaxial frames number the top layer, then the
//! bottom. [`TableMixer::quad_x`] is the exception: it keeps the order the RealFlight
//! QuadCopterX airframe maps (front-left, front-right, rear-left, rear-right). Motor 0 spins
//! clockwise and spin alternates around the frame; on coaxial frames the top and bottom props
//! of each arm spin opposite ways. Each preset's rustdoc has its diagram.
//!
//! | Preset | Motors | Servos |
//! |--------|--------|--------|
//! | [`quad_x`](TableMixer::quad_x) | 0 FL, 1 FR, 2 RL, 3 RR | |
//! | [`quad_plus`](TableMixer::quad_plus) | 0 F, 1 R, 2 B, 3 L | |
//! | [`hex_x`](TableMixer::hex_x) | 0 FL, 1 FR, 2 R, 3 RR, 4 RL, 5 L | |
//! | [`hex_plus`](TableMixer::hex_plus) | 0 F, 1 FR, 2 RR, 3 B, 4 RL, 5 FL | |
//! | [`octo_x`](TableMixer::octo_x) | 0–7 from just left of the nose | |
//! | [`y6`](TableMixer::y6) | top 0 FL, 1 FR, 2 B; bottom 3–5 under them | |
//! | [`x8`](TableMixer::x8) | top 0 FL, 1 FR, 2 RR, 3 RL; bottom 4–7 under them | |
//! | [`tri`](TableMixer::tri) | 0 FL, 1 FR, 2 B | 0 tail tilt (yaw) |
//! | [`bi`](TableMixer::bi) | 0 L, 1 R | 0 left tilt, 1 right tilt |
//! | [`tailsitter_2p_hover`](TableMixer::tailsitter_2p_hover) | 0 L, 1 R | 0 left elevon, 1 right elevon |

use thiserror::Error;

use crate::math::{desaturate, saturate_throttle, saturate_unit};
use crate::{ActuatorCommand, ControlOutput, Mixer, Saturation, Vec3};

mod presets;
#[cfg(test)]
mod tests;

/// One [`ActuatorCommand`] slot a [`MixRow`] drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    /// `motors[i]`, in `[0, 1]`.
    Motor(u8),
    /// `servos[i]`, in `[-1, 1]`.
    Servo(u8),
}

/// One output of a [`TableMixer`]: the slot it drives and how much each moment moves it.
///
/// # Examples
///
/// ```
/// use stable_flight::mixers::MixRow;
///
/// // Front-left motor of an X quad, spinning clockwise.
/// let front_left = MixRow::motor(0, 1.0, 1.0, -1.0);
/// // A tricopter's tail tilt servo: yaw only.
/// let tail = MixRow::servo(0, 0.0, 0.0, 1.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MixRow {
    slot: Slot,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl MixRow {
    /// A motor row: collective thrust plus the weighted moments, desaturated with the other
    /// motors. A table's `m` motor rows drive `motors[0..m]`, in any row order.
    pub const fn motor(index: u8, roll: f32, pitch: f32, yaw: f32) -> Self {
        MixRow {
            slot: Slot::Motor(index),
            roll,
            pitch,
            yaw,
        }
    }

    /// A servo row: the weighted moments, clamped to `[-1, 1]`.
    pub const fn servo(index: u8, roll: f32, pitch: f32, yaw: f32) -> Self {
        MixRow {
            slot: Slot::Servo(index),
            roll,
            pitch,
            yaw,
        }
    }

    /// The slot this row drives.
    pub const fn slot(&self) -> Slot {
        self.slot
    }

    fn moment(&self, moments: Vec3) -> f32 {
        self.roll * moments.x + self.pitch * moments.y + self.yaw * moments.z
    }

    fn is_finite(&self) -> bool {
        self.roll.is_finite() && self.pitch.is_finite() && self.yaw.is_finite()
    }
}

/// How a [`TableMixer`] splits each motor's range between thrust and moments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MixerConfig {
    /// Share of each motor's range given to collective thrust, in `(0, 1)`; the rest
    /// (`1 − thrust_factor`) is headroom for roll and pitch.
    pub thrust_factor: f32,
    /// Yaw's motor factor relative to roll and pitch's (finite, `≥ 0`). Above 1 because prop
    /// drag gives far less yaw authority than differential thrust gives roll and pitch.
    pub yaw_ratio: f32,
}

impl Default for MixerConfig {
    /// Thrust factor 0.9, yaw ratio 2.5: motor moments of 0.1 for roll and pitch, 0.25 for yaw.
    fn default() -> Self {
        MixerConfig {
            thrust_factor: 0.9,
            yaw_ratio: 2.5,
        }
    }
}

/// Why [`TableMixer::new`] rejected a table or config.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum MixerError {
    /// The table has no rows.
    #[error("mixer table is empty")]
    Empty,
    /// A row's slot index is past the [`ActuatorCommand`] arrays ([`ActuatorCommand::SLOTS`]).
    #[error("mixer slot {0:?} is out of range")]
    SlotOutOfRange(Slot),
    /// Two rows drive the same slot.
    #[error("mixer slot {0:?} appears twice")]
    DuplicateSlot(Slot),
    /// Motor slots skip an index: `m` motor rows must drive `motors[0..m]` (the backend's
    /// mapping, not the table, picks each motor's channel).
    #[error("mixer slot {0:?} leaves a gap in the motor slots")]
    MotorGap(Slot),
    /// A row has a NaN or infinite factor.
    #[error("mixer slot {0:?} has a non-finite factor")]
    NonFiniteFactor(Slot),
    /// [`MixerConfig::thrust_factor`] is not in `(0, 1)`.
    #[error("mixer thrust factor must be in (0, 1)")]
    ThrustFactor,
    /// [`MixerConfig::yaw_ratio`] is negative or not finite.
    #[error("mixer yaw ratio must be finite and non-negative")]
    YawRatio,
}

/// A table-driven mixer for multirotors and other thrust-and-surface frames: `N` rows, each
/// driving one motor or servo slot (see the [module docs](self) for the mixing rules and
/// presets).
///
/// # Examples
///
/// A preset:
///
/// ```
/// use stable_flight::mixers::TableMixer;
/// use stable_flight::{ControlOutput, Mixer, Vec3};
///
/// let mut mixer = TableMixer::quad_x();
/// let output = mixer.mix(&ControlOutput::new(0.5, Vec3::new(0.2, 0.0, 0.0)));
/// // Rolling right: the left motors (0 and 2) push harder than the right ones.
/// assert!(output.motors[0] > output.motors[1]);
/// assert!(output.motors[2] > output.motors[3]);
/// ```
///
/// A tricopter yaws with its tail servo:
///
/// ```
/// use stable_flight::mixers::TableMixer;
/// use stable_flight::{ControlOutput, Mixer, Vec3};
///
/// let mut mixer = TableMixer::tri();
/// let output = mixer.mix(&ControlOutput::new(0.5, Vec3::new(0.0, 0.0, 0.3)));
/// assert_eq!(output.servos[0], 0.3);
/// assert_eq!(output.motors[0], output.motors[1]);
/// ```
///
/// A custom table, with its own scaling:
///
/// ```
/// use stable_flight::mixers::{MixRow, MixerConfig, TableMixer};
///
/// // Two motors side by side, each tilted by its own servo (a bicopter, numbered by hand).
/// let rows = [
///     MixRow::motor(0, 1.0, 0.0, 0.0),
///     MixRow::motor(1, -1.0, 0.0, 0.0),
///     MixRow::servo(0, 0.0, 1.0, -1.0),
///     MixRow::servo(1, 0.0, 1.0, 1.0),
/// ];
/// let config = MixerConfig { thrust_factor: 0.8, ..MixerConfig::default() };
/// let mixer = TableMixer::new(rows, config).expect("valid table");
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TableMixer<const N: usize> {
    rows: [MixRow; N],
    config: MixerConfig,
    /// Motor rows, which drive slots `0..motor_count`.
    motor_count: usize,
    /// What the last [`mix`](Mixer::mix) gave up.
    saturation: Saturation,
}

impl<const N: usize> TableMixer<N> {
    /// A mixer over `rows`, scaled by `config`.
    ///
    /// # Errors
    ///
    /// [`MixerError`] when the table is empty, a slot index is 8 or more, two rows share a
    /// slot, the motor slots skip an index, a factor is not finite, or `config` is out of
    /// range.
    pub fn new(rows: [MixRow; N], config: MixerConfig) -> Result<Self, MixerError> {
        validate_rows(&rows)?;
        validate_config(&config)?;
        Ok(Self::from_valid(rows, config))
    }

    /// This mixer with `config` instead (e.g. a preset with another thrust factor).
    ///
    /// # Errors
    ///
    /// [`MixerError::ThrustFactor`] or [`MixerError::YawRatio`] when `config` is out of range.
    pub fn with_config(self, config: MixerConfig) -> Result<Self, MixerError> {
        validate_config(&config)?;
        Ok(Self { config, ..self })
    }

    /// Rows and config already known valid (the presets; a test runs each through `new`).
    fn from_valid(rows: [MixRow; N], config: MixerConfig) -> Self {
        let motor_count = motor_count(&rows);
        TableMixer {
            rows,
            config,
            motor_count,
            saturation: Saturation::default(),
        }
    }
}

impl<const N: usize> Mixer for TableMixer<N> {
    /// Mixes thrust (`[0, 1]`) and moments (`[-1, 1]`) into motors in `[0, 1]` and servos in
    /// `[-1, 1]`, per the [module docs](self).
    fn mix(&mut self, command: &ControlOutput) -> ActuatorCommand {
        // Out-of-range or NaN inputs saturate rather than panic (see `crate::math`).
        let thrust = saturate_throttle(command.thrust);
        let moments = command.moments.map(saturate_unit);

        let MixerConfig {
            thrust_factor,
            yaw_ratio,
        } = self.config;
        let collective = thrust * thrust_factor;
        let headroom = 1.0 - thrust_factor;
        let motor_moments = Vec3::new(
            moments.x * headroom,
            moments.y * headroom,
            moments.z * headroom * yaw_ratio,
        );

        let mut command = ActuatorCommand::default();
        let mut servo_clamped = false;
        for row in &self.rows {
            match row.slot {
                Slot::Motor(index) => {
                    if let Some(motor) = command.motors.get_mut(usize::from(index)) {
                        *motor = collective + row.moment(motor_moments);
                    }
                }
                Slot::Servo(index) => {
                    let value = row.moment(moments);
                    servo_clamped |= value.abs() > 1.0;
                    if let Some(servo) = command.servos.get_mut(usize::from(index)) {
                        *servo = value.clamp(-1.0, 1.0);
                    }
                }
            }
        }
        // Motor slots are `0..motor_count` (validated), so the slice holds exactly the motors.
        let desaturation = command
            .motors
            .get_mut(..self.motor_count)
            .map(|motors| desaturate(motors, collective))
            .unwrap_or_default();

        self.saturation = Saturation {
            mixer_thrust: desaturation.lowered,
            mixer_moments: desaturation.moments_lost || servo_clamped,
            ..Saturation::default()
        };
        command
    }

    /// `mixer_thrust` when the motors were shifted down (collective gave way), and
    /// `mixer_moments` when the motor moments were scaled down or clipped, or a servo clamped.
    fn saturation(&self) -> Saturation {
        self.saturation
    }
}

fn motor_count(rows: &[MixRow]) -> usize {
    rows.iter()
        .filter(|row| matches!(row.slot, Slot::Motor(_)))
        .count()
}

fn validate_rows(rows: &[MixRow]) -> Result<(), MixerError> {
    if rows.is_empty() {
        return Err(MixerError::Empty);
    }
    for (i, row) in rows.iter().enumerate() {
        let (Slot::Motor(index) | Slot::Servo(index)) = row.slot;
        if usize::from(index) >= ActuatorCommand::SLOTS {
            return Err(MixerError::SlotOutOfRange(row.slot));
        }
        if rows[..i].iter().any(|earlier| earlier.slot == row.slot) {
            return Err(MixerError::DuplicateSlot(row.slot));
        }
        if !row.is_finite() {
            return Err(MixerError::NonFiniteFactor(row.slot));
        }
    }
    // With unique indices, the motors fill `0..count` exactly when none is `count` or above.
    let motor_count = motor_count(rows);
    if let Some(row) = rows
        .iter()
        .find(|row| matches!(row.slot, Slot::Motor(index) if usize::from(index) >= motor_count))
    {
        return Err(MixerError::MotorGap(row.slot));
    }
    Ok(())
}

fn validate_config(config: &MixerConfig) -> Result<(), MixerError> {
    // Comparisons are false for NaN, so NaN fails both.
    if !(config.thrust_factor > 0.0 && config.thrust_factor < 1.0) {
        return Err(MixerError::ThrustFactor);
    }
    if !(config.yaw_ratio >= 0.0 && config.yaw_ratio.is_finite()) {
        return Err(MixerError::YawRatio);
    }
    Ok(())
}
