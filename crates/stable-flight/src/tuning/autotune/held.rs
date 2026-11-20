//! A copter run flown from an [`AutotuneHold`]: how the hold and the [`Autotune`] supervisor
//! move together, whatever starts and stops the run (a ground station over the link, a
//! transmitter switch).
//!
//! - [`start`] starts the run and engages the hold where the vehicle is.
//! - [`stop`] releases the hold and reverts the run.
//! - [`step`], between ticks, reverts the run when the pilot took over from the hold mid-run,
//!   then advances it.
//!
//! The hold stays engaged once a run completes or fails, flying the gains it ended on, until
//! the pilot takes over (a roll or pitch stick, a land, a disarm) or a [`stop`]. ArduPilot's
//! AUTOTUNE mode holds the same way until the pilot changes mode. Handing back at the end
//! would put the copter on sticks that may not hold its height: a ground station's virtual
//! joystick flies throttle, not height, and a stage that failed on a sink would hand the
//! copter to the throttle that let it sink.
//!
//! # Examples
//!
//! ```
//! use stable_flight::commanders::{AutotuneHold, AutotuneHoldConfig};
//! use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
//! use stable_flight::mixers::TableMixer;
//! use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
//! use stable_flight::testing::{ConstantCommander, NullVehicle};
//! use stable_flight::tuning::autotune::{Autotune, Envelope, StagePlan, StartError, held};
//! use stable_flight::FlightLoop;
//!
//! let commander = AutotuneHold::new(ConstantCommander::default(), AutotuneHoldConfig::default())?;
//! let mut flight_loop = FlightLoop::new(
//!     commander,
//!     NullVehicle::default(),
//!     CopterGuidance::new(CopterGuidanceConfig::default()),
//!     CopterStabilizer::new(CopterStabilizerConfig::default()),
//!     TableMixer::quad_x(),
//!     (),
//! );
//! let mut autotune = Autotune::new(&StagePlan::copter_design_search(), Envelope::default())?;
//!
//! // Disarmed: the run doesn't start, and the hold stays off.
//! assert_eq!(held::start(&mut autotune, &mut flight_loop), Err(StartError::NotArmed));
//! assert!(!flight_loop.commander().is_engaged());
//! let step = held::step(&mut autotune, &mut flight_loop, 0.004);
//! assert!(!step.reverted && step.event.is_none());
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use crate::commanders::AutotuneHold;
use crate::{Commander, FlightLoop};

use super::{Autotune, AutotuneEvent, CopterSide, SideTunable, StartError};

#[cfg(test)]
mod tests;

/// What one [`step`] did.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeldStep {
    /// The pilot took over from the hold mid-run, and the run reverted to its start
    /// ([`AutotuneEvent::Reverted`]).
    pub reverted: bool,
    /// What the run's [`update`](Autotune::update) did.
    pub event: Option<AutotuneEvent>,
}

/// Starts `autotune` where the vehicle is and engages the hold there.
///
/// # Errors
///
/// [`Autotune::start`]'s, leaving the hold as it was.
pub fn start<C, V, G, S, M, T>(
    autotune: &mut Autotune,
    flight_loop: &mut FlightLoop<AutotuneHold<C>, V, G, S, M, T>,
) -> Result<(), StartError>
where
    C: Commander,
    S: SideTunable<CopterSide>,
{
    autotune.start(flight_loop)?;
    let state = flight_loop.state().clone();
    flight_loop.commander_mut().engage(&state);
    Ok(())
}

/// Hands the vehicle back to the commander the hold wraps and [reverts](Autotune::revert) the
/// run: [`AutotuneEvent::Reverted`], or `None` when there was no run to undo.
pub fn stop<C, V, G, S, M, T>(
    autotune: &mut Autotune,
    flight_loop: &mut FlightLoop<AutotuneHold<C>, V, G, S, M, T>,
) -> Option<AutotuneEvent>
where
    C: Commander,
    S: SideTunable<CopterSide>,
{
    flight_loop.commander_mut().release();
    autotune.revert(flight_loop)
}

/// Between ticks: reverts the run if the pilot took over from the hold
/// ([`take_override`](AutotuneHold::take_override)) while it ran, then advances it by `dt` s.
/// A take-over after the run ended only hands the vehicle back: the tuned gains stay.
pub fn step<C, V, G, S, M, T>(
    autotune: &mut Autotune,
    flight_loop: &mut FlightLoop<AutotuneHold<C>, V, G, S, M, T>,
    dt: f32,
) -> HeldStep
where
    C: Commander,
    S: SideTunable<CopterSide>,
{
    let taken_over = flight_loop.commander_mut().take_override();
    let reverted = taken_over && autotune.is_running() && autotune.revert(flight_loop).is_some();
    HeldStep {
        reverted,
        event: autotune.update(flight_loop, dt),
    }
}
