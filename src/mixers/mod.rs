use crate::{ActuatorCommand, ControlOutput};

pub mod quad;
pub mod tailsitter;

pub use quad::QuadCopterMixer;
pub use tailsitter::TailSitterMixer;

pub trait Mixer {
    fn mix(&self, controls: &ControlOutput) -> ActuatorCommand;
}
