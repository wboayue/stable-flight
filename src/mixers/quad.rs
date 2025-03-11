use crate::{ActuatorCommand, ControlOutput};

use super::Mixer;

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

        output.motors[0] = baseline - yaw + pitch + roll; // front left
        output.motors[1] = baseline + yaw + pitch - roll; // front right
        output.motors[2] = baseline + yaw - pitch + roll; // rear left
        output.motors[3] = baseline - yaw - pitch - roll; // rear right

        // clamp and scale output to [0, 100]
        for motor in output.motors.iter_mut() {
            *motor = motor.clamp(0.0, 1.0) * 100.0;
        }

        output
    }
}
