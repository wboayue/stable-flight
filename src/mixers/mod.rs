use crate::{ActuatorCommand, ControlOutput, Mixer};

pub struct QuadCopterMixer {}

impl Mixer for QuadCopterMixer {
    fn mix_control_outputs(&self, controls: &ControlOutput) -> ActuatorCommand {
        println!("Mixing control outputs for QuadCopter");
        ActuatorCommand { channels: [0.0; 6] }
    }
}

pub struct TailSitterMixer {}

impl Mixer for TailSitterMixer {
    fn mix_control_outputs(&self, controls: &ControlOutput) -> ActuatorCommand {
        println!("Mixing control outputs for TailSitter");
        ActuatorCommand { channels: [0.0; 6] }
    }
}
