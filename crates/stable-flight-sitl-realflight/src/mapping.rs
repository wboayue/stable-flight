//! [`ActuatorMapping`]: which RealFlight channel each [`ActuatorCommand`] slot drives.

use realflight_bridge::ControlInputs;
use stable_flight::ActuatorCommand;
use thiserror::Error;

#[cfg(test)]
mod tests;

/// Number of RealFlight channels ([`ControlInputs::channels`]); a test checks it.
pub const CHANNEL_COUNT: usize = 12;

/// Maps [`ActuatorCommand`] slots to RealFlight channels.
///
/// `motors[i]` and `servos[i]` name the RealFlight channel (0–11) that the mixer's motor or
/// servo `i` drives; `None` leaves the slot unused. The mixer sets what each slot means, so a
/// mapping only fits its own mixer. RealFlight channels run from 0.0 to 1.0: motors pass
/// through, and servos (-1.0 to 1.0) are rescaled so 0.0 lands on center (0.5), after negating
/// any slot in `reversed_servos` (a surface the model moves the other way from the mixer's
/// convention) and adding its `servo_trims` offset (a surface whose neutral sits off the
/// channel's center), clamped to the servo range. Channels no slot drives are sent as 0.0, which would hold a servo there at full
/// deflection, so a mapping covers every surface and switch the model has; channels it leaves
/// unmapped drive nothing on the model (check with `channel-sweep-realflight`; each airframe's
/// docs say what its unmapped channels do).
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_realflight::ActuatorMapping;
///
/// let mut mapping = ActuatorMapping::default();
/// mapping.motors[0] = Some(0); // Motor 0 to channel 0
/// mapping.servos[0] = Some(3); // Servo 0 (say, a tail tilt) to channel 3
/// assert!(mapping.validate().is_ok());
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ActuatorMapping {
    /// RealFlight channel for each motor slot.
    pub motors: [Option<usize>; ActuatorCommand::SLOTS],
    /// RealFlight channel for each servo slot.
    pub servos: [Option<usize>; ActuatorCommand::SLOTS],
    /// Servo slots whose command is negated before it reaches the channel.
    pub reversed_servos: [bool; ActuatorCommand::SLOTS],
    /// Each servo slot's neutral, in servo units (-1.0 to 1.0, 0.0 the channel's center),
    /// added after any reversal: ArduPilot's `SERVOn_TRIM`, `(trim_us − 1500) / 500`.
    pub servo_trims: [f32; ActuatorCommand::SLOTS],
}

/// Why an [`ActuatorMapping`] is invalid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum MappingError {
    /// A slot names a channel RealFlight doesn't have.
    #[error("channel {channel} is out of range (RealFlight has {CHANNEL_COUNT})")]
    ChannelOutOfRange {
        /// The offending channel.
        channel: usize,
    },
    /// Two slots drive the same channel.
    #[error("channel {channel} is mapped twice")]
    DuplicateChannel {
        /// The offending channel.
        channel: usize,
    },
}

impl ActuatorMapping {
    /// Checks that every mapped channel exists and that no two slots share one.
    ///
    /// # Errors
    ///
    /// The first [`MappingError`] found, motors before servos.
    pub fn validate(&self) -> Result<(), MappingError> {
        let mut used = [false; CHANNEL_COUNT];
        for &channel in self.motors.iter().chain(&self.servos).flatten() {
            let slot = used
                .get_mut(channel)
                .ok_or(MappingError::ChannelOutOfRange { channel })?;
            if *slot {
                return Err(MappingError::DuplicateChannel { channel });
            }
            *slot = true;
        }
        Ok(())
    }

    /// Writes `command` to RealFlight channels. Skips any channel out of range, so an
    /// unvalidated mapping can't panic the tick.
    pub(crate) fn channels(&self, command: &ActuatorCommand) -> ControlInputs {
        let mut input = ControlInputs::default();
        write(&mut input, &self.motors, &command.motors, |m| m);
        let servos: [f32; ActuatorCommand::SLOTS] = core::array::from_fn(|i| {
            let servo = if self.reversed_servos[i] {
                -command.servos[i]
            } else {
                command.servos[i]
            };
            (servo + self.servo_trims[i]).clamp(-1.0, 1.0)
        });
        write(&mut input, &self.servos, &servos, servo_to_channel);
        input
    }
}

/// Writes each mapped slot's value, through `convert`, to its channel.
fn write(
    input: &mut ControlInputs,
    slots: &[Option<usize>],
    values: &[f32],
    convert: impl Fn(f32) -> f32,
) {
    for (slot, &value) in slots.iter().zip(values) {
        if let Some(channel) = slot.and_then(|c| input.channels.get_mut(c)) {
            *channel = convert(value);
        }
    }
}

/// Rescales a servo command (-1.0 to 1.0, 0.0 centered) to a RealFlight channel (0.0 to 1.0).
fn servo_to_channel(servo: f32) -> f32 {
    (servo + 1.0) / 2.0
}
