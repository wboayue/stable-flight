use crate::{ActuatorCommand, ControlOutput};

use super::Mixer;

#[cfg(test)]
mod tests;

pub struct QuadCopterMixer {}

impl Mixer for QuadCopterMixer {
    // Mix control outputs to individual motor commands
    // Assumes throttle in range [0, 1] and moments in range [-1, 1]
    fn mix(&self, command: &ControlOutput) -> ActuatorCommand {
        // Calculate thrust offsets required to achieve desired moments around each axis
        // Each motor contributes to roll, pitch and yaw based on its position and rotation direction

        // Roll contribution (X-axis moment) - moment in range [-1, 1]
        let roll = command.moments.x / 4.0;

        // Pitch contribution (Y-axis moment) - moment in range [-1, 1]
        let pitch = command.moments.y / 4.0;

        // Yaw contribution (Z-axis moment) - moment in range [-1, 1]
        let yaw = command.moments.z / 8.0;

        // Calculate the baseline thrust per motor (equal distribution of collective thrust)
        // Thrust in range [0, 1]
        let baseline = command.thrust / 4.0;

        // Motor configurations in a standard X configuration:
        // Motor 0: Front Left  - CCW rotation - contributes +roll, +pitch, -yaw
        // Motor 1: Front Right - CW rotation  - contributes -roll, +pitch, +yaw
        // Motor 2: Rear Left   - CW rotation  - contributes +roll, -pitch, +yaw
        // Motor 3: Rear Right  - CCW rotation - contributes -roll, -pitch, -yaw

        let mut output = ActuatorCommand::default();

        // Step 1: Compute raw motor outputs
        let mut raw_outputs = [0.0; 8];
        raw_outputs[0] = baseline - yaw + pitch + roll; // front left
        raw_outputs[1] = baseline + yaw + pitch - roll; // front right
        raw_outputs[2] = baseline + yaw - pitch + roll; // rear left
        raw_outputs[3] = baseline - yaw - pitch - roll; // rear right

        // Step 2: Find the min and max of raw outputs to determine scaling
        let min_raw = raw_outputs.iter().fold(f32::INFINITY, |a, &b| a.min(b));
        let max_raw = raw_outputs.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));

        // Step 3: Compute scaling factor to fit outputs within [0, 1]
        let mut scale = 1.0;
        let desired_min = 0.0;
        let desired_max = 1.0;

        if min_raw < desired_min || max_raw > desired_max {
            // Compute the range of raw outputs
            let raw_range = max_raw - min_raw;

            // Compute scaling factors needed to fit within [0, 1]
            let scale_to_max = if max_raw > desired_max {
                (desired_max - desired_min) / (max_raw - min_raw)
            } else {
                f32::INFINITY
            };
            let scale_to_min = if min_raw < desired_min {
                (desired_max - desired_min) / (max_raw - min_raw)
            } else {
                f32::INFINITY
            };

            // Take the smaller scaling factor (most restrictive)
            scale = scale_to_max.min(scale_to_min);

            // If raw_range is 0 (all outputs are equal), avoid division by zero
            if raw_range == 0.0 {
                scale = 1.0;
            }

            // Step 4: Apply scaling and shift to fit within [0, 1]
            for raw in raw_outputs.clone().iter_mut() {
                // Scale the outputs around their mean to fit within [0, 1]
                // First, normalize around the mean of raw outputs
                let mean_raw = (max_raw + min_raw) / 2.0;
                *raw = (*raw - mean_raw) * scale + mean_raw;

                // Shift to ensure the range fits exactly in [0, 1]
                let scaled_min = raw_outputs.iter().fold(f32::INFINITY, |a, &b| a.min(b));
                *raw -= scaled_min; // Shift so minimum is 0

                // Final adjustment to ensure max fits (if necessary)
                let scaled_max = raw_outputs.iter().fold(f32::NEG_INFINITY, |a, &b| a.max(b));
                if scaled_max > 0.0 {
                    *raw *= desired_max / scaled_max; // Scale down to fit max at 1.0
                }
            }
        }

        // Step 5: Assign scaled outputs to ActuatorCommand and scale to [0, 100]
        for (i, motor) in output.motors.iter_mut().enumerate() {
            *motor = raw_outputs[i] * 100.0;
            // Ensure rounding doesn't push us slightly out of bounds
            if *motor < 0.0 {
                *motor = 0.0;
            }
            if *motor > 100.0 {
                *motor = 100.0;
            }
        }

        output
    }
}
