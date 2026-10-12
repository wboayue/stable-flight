//! Provides functionality to translate generalized control outputs (thrust and moments) into specific actuator commands tailored to the vehicle configuration.

pub mod table;
pub mod tailsitter;

pub use table::{MixRow, MixerConfig, MixerError, Slot, TableMixer};
pub use tailsitter::TailSitterMixer;
