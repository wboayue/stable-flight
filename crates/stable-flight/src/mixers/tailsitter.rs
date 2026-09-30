use log::debug;

use crate::{ActuatorCommand, ControlOutput};

use crate::Mixer;

pub struct TailSitterMixer {}

impl Mixer for TailSitterMixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand {
        debug!("Mixing control outputs for TailSitter");
        ActuatorCommand::default()
    }
}
