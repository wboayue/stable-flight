use log::info;

use crate::{ActuatorCommand, ControlOutput};

use crate::Mixer;

pub struct TailSitterMixer {}

impl Mixer for TailSitterMixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand {
        info!("Mixing control outputs for TailSitter");
        ActuatorCommand::default()
    }
}
