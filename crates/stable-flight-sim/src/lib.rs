//! A deterministic rigid-body multirotor simulator for Stable Flight's closed-loop tests.
//!
//! [`SimCopter`] is a [`Vehicle`](stable_flight::Vehicle) a
//! [`FlightLoop`](stable_flight::FlightLoop) flies in lockstep: each tick integrates the last
//! motor command over the tick's `dt`, with no wall clock and only seeded noise, so the same
//! inputs give the same flight, bit for bit. It aims at plausible, repeatable dynamics for
//! `cargo test` and CI, not fidelity; RealFlight covers realism.
//!
//! - [`MultirotorModel`]: mass, inertia, motors, drag, ground friction. Motor geometry comes from
//!   the mixer's rows, so the simulated vehicle answers each moment the way the mixer expects.
//! - [`Wind`]: steady wind plus an optional deterministic [`Gust`].
//! - [`Perturbations`]: actuator delay, gyro noise and vibration, a disturbance moment, and
//!   motor mismatch, all off by default. Ranges to sweep, so a method that passes on them
//!   isn't fitted to one clean plant.
//! - [`airframes`]: physics paired with the configs that fly it ([`airframes::quad_x`],
//!   [`airframes::heavy_quad`], [`airframes::racer`]).
//!
//! `no_std` with fixed-size math, so it can later drive hardware-in-the-loop runs.
//!
//! # Examples
//!
//! ```
//! use std::time::Duration;
//! use stable_flight::guidance::copter::CopterGuidance;
//! use stable_flight::stabilizers::copter::CopterStabilizer;
//! use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
//! use stable_flight::{AttitudeCommand, Command, ControlInput, FlightLoop};
//! use stable_flight_sim::{SimCopter, airframes};
//!
//! let airframe = airframes::quad_x();
//! let vehicle = SimCopter::new(airframe.model, &airframe.mixer).expect("valid airframe");
//! // Armed, level, full stick throttle.
//! let climb = Command::new(
//!     true,
//!     ControlInput::Attitude(AttitudeCommand { throttle: 1.0, ..Default::default() }),
//! );
//! let mut flight_loop = FlightLoop::new(
//!     ConstantCommander::new(climb),
//!     vehicle,
//!     CopterGuidance::new(airframe.guidance),
//!     CopterStabilizer::new(airframe.stabilizer),
//!     airframe.mixer,
//!     RecordingTelemetry::<16>::new(),
//! );
//! flight_loop.prime();
//! for _ in 0..500 {
//!     flight_loop.step(Duration::from_millis(4));
//! }
//! assert!(flight_loop.state().position_m.z > 2.0);
//! ```

#![cfg_attr(not(test), no_std)]

pub mod airframes;
mod body;
mod copter;
mod delay;
mod ground;
mod gyro;
mod model;
mod motors;
mod perturbations;
mod wind;

pub use copter::SimCopter;
pub use model::{ModelError, MultirotorModel};
pub use perturbations::{GyroNoise, MAX_ACTUATOR_DELAY_S, Perturbations, Vibration};
pub use wind::{Gust, Wind};
