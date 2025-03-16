//! # QuadCopter Mixer Module
//!
//! This module provides a `QuadCopterMixer` that implements the `Mixer` trait to convert
//! control outputs (collective thrust and moments) into individual motor commands for a
//! quadcopter in a standard X configuration.
//!
//! The mixer assumes that the throttle input is in the range `[0, 1]` and that the moments
//! (roll, pitch, yaw) are in the range `[-1, 1]`. It uses configurable scaling factors to
//! adjust the influence of each moment on the motor commands.
//!
//! ## Motor Configuration (Standard X)
//!
//! - **Motor 0 (Front Left):** CCW rotation, contributes `+roll`, `+pitch`, `-yaw`
//! - **Motor 1 (Front Right):** CW rotation, contributes `-roll`, `+pitch`, `+yaw`
//! - **Motor 2 (Rear Left):** CW rotation, contributes `+roll`, `-pitch`, `+yaw`
//! - **Motor 3 (Rear Right):** CCW rotation, contributes `-roll`, `-pitch`, `-yaw`
//!
//! ## Scaling Factors
//!
//! The following constants are defined for default scaling:
//!
//! - `NUM_MOTORS`: The number of motors, used to evenly distribute the collective thrust.
//! - `DEFAULT_ROLL_PITCH_SCALING`: Scaling factor for roll and pitch moments.
//! - `DEFAULT_YAW_SCALING`: Scaling factor for the yaw moment.

use super::Mixer;
use crate::{ActuatorCommand, ControlOutput};

#[cfg(test)]
mod tests;

const THRUST_FACTOR: f32 = 0.9; // Thrust factor for baseline thrust calculation
const CONTROL_FACTOR: f32 = 1.0 - THRUST_FACTOR; // Control factor for moment contributions

const DEFAULT_ROLL_PITCH_SCALING: f32 = CONTROL_FACTOR;
const DEFAULT_YAW_SCALING: f32 = DEFAULT_ROLL_PITCH_SCALING * 4.0;

/// Mixer for a quadcopter in a standard X configuration.
///
/// This mixer converts control outputs (thrust and moments) into individual motor commands.
/// The roll, pitch, and yaw contributions are scaled using configurable factors.
pub struct QuadCopterMixer {
    /// Scaling factor for roll and pitch moments.
    roll_pitch_scaling: f32,
    /// Scaling factor for the yaw moment.
    yaw_scaling: f32,
}

impl Default for QuadCopterMixer {
    /// Creates a new `QuadCopterMixer` with default scaling factors.
    fn default() -> Self {
        QuadCopterMixer {
            roll_pitch_scaling: DEFAULT_ROLL_PITCH_SCALING,
            yaw_scaling: DEFAULT_YAW_SCALING,
        }
    }
}

impl Mixer for QuadCopterMixer {
    /// Mixes control outputs into individual motor commands.
    ///
    /// # Parameters
    ///
    /// - `command`: A reference to a `ControlOutput` containing the desired collective thrust
    ///   (in the range [0, 1]) and moments (roll, pitch, yaw in the range [-1, 1]).
    ///
    /// # Returns
    ///
    /// An `ActuatorCommand` with motor outputs in the range [0, 100], representing the percentage
    /// of maximum thrust for each motor.
    ///
    /// # Details
    ///
    /// The mixer performs the following steps:
    ///
    /// 1. **Compute Moment Contributions:**  
    ///    - Roll: `command.moments.x / roll_pitch_scaling`  
    ///    - Pitch: `command.moments.y / roll_pitch_scaling`  
    ///    - Yaw: `command.moments.z / yaw_scaling`
    ///
    /// 2. **Compute Baseline Thrust:**  
    ///    The total thrust is evenly distributed among all motors:  
    ///    `baseline = command.thrust / NUM_MOTORS`
    ///
    /// 3. **Combine Contributions for Each Motor:**  
    ///    The motor outputs are adjusted by adding or subtracting the computed roll, pitch, and yaw
    ///    contributions according to the motor configuration:
    ///
    ///    - Motor 0 (Front Left): `baseline - yaw + pitch + roll`
    ///    - Motor 1 (Front Right): `baseline + yaw + pitch - roll`
    ///    - Motor 2 (Rear Left): `baseline + yaw - pitch + roll`
    ///    - Motor 3 (Rear Right): `baseline - yaw - pitch - roll`
    ///
    /// 4. **Clamp and Scale:**  
    ///    Each raw motor command is clamped to the range [0, 1] and then scaled to a percentage
    ///    by multiplying by 100.
    fn mix(&self, command: &ControlOutput) -> ActuatorCommand {
        assert!(
            command.thrust >= 0.0 && command.thrust <= 1.0,
            "Thrust ({}) must be in the range [0, 1]",
            command.thrust
        );
        assert!(
            command.moments.x >= -1.0 && command.moments.x <= 1.0,
            "Roll moment ({}) must be in the range [-1, 1]",
            command.moments.x
        );
        assert!(
            command.moments.y >= -1.0 && command.moments.y <= 1.0,
            "Pitch moment ({}) must be in the range [-1, 1]",
            command.moments.y
        );
        assert!(
            command.moments.z >= -1.0 && command.moments.z <= 1.0,
            "Yaw moment ({}) must be in the range [-1, 1]",
            command.moments.z
        );

        // println!("command: {:?}", command);

        // Compute scaled moment contributions
        let mut roll = command.moments.x * self.roll_pitch_scaling;
        let mut pitch = command.moments.y * self.roll_pitch_scaling; // pitch is scaled more aggressively
        let mut yaw = command.moments.z * self.yaw_scaling;

        // Compute baseline thrust per motor (even distribution)
        let baseline = command.thrust * THRUST_FACTOR;

        if baseline <= 0.0 {
            return ActuatorCommand::default();
        }

        // Compute raw motor outputs based on standard X configuration:
        // Motor 0: Front Left  - CCW rotation -> +roll, +pitch, -yaw
        // Motor 1: Front Right - CW rotation  -> -roll, +pitch, +yaw
        // Motor 2: Rear Left   - CW rotation  -> +roll, -pitch, +yaw
        // Motor 3: Rear Right  - CCW rotation -> -roll, -pitch, -yaw
        let mut raw_outputs = [
            baseline + yaw + pitch + roll, // Motor 0: Front Left
            baseline - yaw + pitch - roll, // Motor 1: Front Right
            baseline - yaw - pitch + roll, // Motor 2: Rear Left
            baseline + yaw - pitch - roll, // Motor 3: Rear Right

                                           // baseline - yaw + pitch + roll, // Motor 0: Front Left
                                           // baseline + yaw + pitch - roll, // Motor 1: Front Right
                                           // baseline + yaw - pitch + roll, // Motor 2: Rear Left
                                           // baseline - yaw - pitch - roll, // Motor 3: Rear Right
        ];

        // Clamp each output to [0, 1] and scale to a percentage (0 to 100)
        for output in raw_outputs.iter_mut() {
            *output = output.clamp(0.0, 1.0);
        }

        // Assemble the actuator command with the computed motor outputs
        let mut actuator_command = ActuatorCommand::default();

        // assign for quad X
        actuator_command.motors[0] = raw_outputs[0];
        actuator_command.motors[1] = raw_outputs[1];
        actuator_command.motors[2] = raw_outputs[2];
        actuator_command.motors[3] = raw_outputs[3];

        //    println!("actuator_command: {:?}", actuator_command);

        actuator_command
    }
}
