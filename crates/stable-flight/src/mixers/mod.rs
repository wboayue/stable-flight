//! Provides functionality to translate generalized control outputs (thrust and moments) into specific actuator commands tailored to the vehicle configuration.

pub mod table;

pub use table::{MixRow, MixerConfig, MixerError, Slot, TableMixer};
