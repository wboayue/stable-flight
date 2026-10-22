//! What a script does at a reached waypoint before flying on: its maneuver, then its hold.

use stable_flight::commanders::maneuver::{Maneuver, ManeuverConfig, ManeuverRunner};
use stable_flight::{ControlInput, VehicleState};

#[cfg(test)]
mod tests;

/// A row's actions, flown in order once its waypoint is reached.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(super) struct RowActions {
    /// A maneuver flown from where the waypoint was reached.
    pub maneuver: Option<Maneuver>,
    /// Seconds to keep flying to the waypoint after the maneuver, if any.
    pub hold_s: f32,
}

/// What the script commands this tick in place of the route.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Step {
    /// Row `index`'s maneuver: fly `input`.
    Maneuver {
        /// The row.
        index: usize,
        /// The maneuver's command.
        input: ControlInput,
    },
    /// Hold row `index`'s waypoint.
    Hold {
        /// The row.
        index: usize,
    },
}

/// The action being flown.
#[derive(Debug, Clone, Copy)]
enum Active {
    /// Row `index`'s maneuver, flown by the runner.
    Maneuver { index: usize },
    /// Row `index`'s hold, with `remaining_s` left.
    Hold { index: usize, remaining_s: f32 },
}

/// Sequences each row's actions: started when the route moves on from the row's waypoint,
/// stepped each tick while they last, the route paused meanwhile.
#[derive(Debug, Clone)]
pub(super) struct Actions {
    /// Each row's actions, by index.
    rows: Vec<RowActions>,
    /// The action being flown.
    active: Option<Active>,
    /// The route has started its land.
    landing: bool,
    runner: ManeuverRunner,
}

impl Actions {
    /// Flies `rows[i]` once waypoint `i` is reached, maneuvers by a runner with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ManeuverConfig::validate`]).
    pub fn new(rows: Vec<RowActions>, config: ManeuverConfig) -> Self {
        Actions {
            rows,
            active: None,
            landing: false,
            runner: ManeuverRunner::new(config),
        }
    }

    /// Replaces the maneuvers' settings.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ManeuverConfig::validate`]).
    pub fn set_maneuver_config(&mut self, config: ManeuverConfig) {
        self.runner = ManeuverRunner::new(config);
    }

    /// The action in progress after `dt` more of it at `state`; `None` once none is left.
    pub fn step(&mut self, dt: f32, state: &VehicleState) -> Option<Step> {
        match self.active? {
            Active::Maneuver { index } => {
                if let Some(input) = self.runner.next(dt, state) {
                    return Some(Step::Maneuver { index, input });
                }
                self.active = None;
                self.begin_hold(index)
            }
            Active::Hold { index, remaining_s } => {
                let remaining_s = remaining_s - dt;
                // A NaN `dt` ends the hold.
                if remaining_s.is_nan() || remaining_s <= 0.0 {
                    self.active = None;
                    return None;
                }
                self.active = Some(Active::Hold { index, remaining_s });
                Some(Step::Hold { index })
            }
        }
    }

    /// After a route tick that began on waypoint `reached` and ended on `current`, `lands`
    /// when it commanded the route's land: starts `reached`'s actions if the route moved on
    /// from it (to another waypoint or into the land), returning the first tick of them.
    pub fn start(
        &mut self,
        reached: usize,
        current: usize,
        lands: bool,
        dt: f32,
        state: &VehicleState,
    ) -> Option<Step> {
        let moved_on = current != reached || (lands && !self.landing);
        self.landing = lands;
        if !moved_on {
            return None;
        }
        let row = self.rows.get(reached).copied().unwrap_or_default();
        let Some(maneuver) = row.maneuver else {
            return self.begin_hold(reached);
        };
        self.runner.start(maneuver, state);
        self.active = Some(Active::Maneuver { index: reached });
        self.step(dt, state)
    }

    /// Starts row `index`'s hold if it has one, returning its first tick.
    fn begin_hold(&mut self, index: usize) -> Option<Step> {
        let hold_s = self.rows.get(index).map_or(0.0, |row| row.hold_s);
        // NaN or non-positive: no hold.
        if hold_s.is_nan() || hold_s <= 0.0 {
            return None;
        }
        self.active = Some(Active::Hold {
            index,
            remaining_s: hold_s,
        });
        Some(Step::Hold { index })
    }
}
