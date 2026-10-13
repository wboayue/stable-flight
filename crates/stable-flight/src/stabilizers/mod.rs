//! Inner-loop control: track a [`Setpoint`](crate::Setpoint) from guidance against vehicle
//! state feedback.

pub mod copter;
pub mod plane;
pub mod vtol;
