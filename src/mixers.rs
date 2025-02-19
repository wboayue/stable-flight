use crate::{ControlOutput, Mixer};


pub struct QuadCopterMixer {}

impl Mixer for QuadCopterMixer {
    fn mix_control_outputs(&self, controls: &ControlOutput) {
        println!("Mixing control outputs for QuadCopter");
    }
}

pub struct TailSitterMixer {}

impl Mixer for TailSitterMixer {
    fn mix_control_outputs(&self, controls: &ControlOutput) {
        println!("Mixing control outputs for TailSitter");
    }
    
}