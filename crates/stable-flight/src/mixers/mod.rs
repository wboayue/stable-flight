//! Provides functionality to translate generalized control outputs (thrust and moments) into specific actuator commands tailored to the vehicle configuration.

pub mod quad;
pub mod tailsitter;

pub use quad::QuadCopterMixer;
pub use tailsitter::TailSitterMixer;
