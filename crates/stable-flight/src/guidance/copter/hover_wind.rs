//! When the hover wind estimate updates: airborne above the ground it was primed on, and not
//! paused by a VTOL flying outside hover.

use crate::VehicleState;
use crate::estimation::wind::{WindConfig, WindEstimate, WindEstimator};

/// A [`WindEstimator`] fed only while the vehicle is airborne and the owner hasn't paused it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct HoverWind {
    estimator: WindEstimator,
    /// Altitude, in meters, primed on the ground; `None` until primed.
    ground_altitude_m: Option<f32>,
    /// The owner flies outside hover (a transition, a pusher): the drag model doesn't hold.
    paused: bool,
}

impl HoverWind {
    /// Feeds `state` to the estimator while airborne (more than `airborne_height_m` above the
    /// ground altitude) and not paused.
    pub(crate) fn update(
        &mut self,
        dt: f32,
        config: &WindConfig,
        airborne_height_m: f32,
        state: &VehicleState,
    ) {
        if !self.paused && self.airborne(airborne_height_m, state) {
            self.estimator.update(dt, config, state);
        }
    }

    /// On the ground (no more than `airborne_height_m` above the last ground altitude, or
    /// never primed), latches the ground altitude and forgets the wind; in the air (a VTOL
    /// back in hover), keeps both. A vehicle re-armed on ground more than
    /// `airborne_height_m` above where it was last primed counts as airborne, and feeds the
    /// estimator from the ground until it lifts off.
    pub(crate) fn prime(&mut self, airborne_height_m: f32, state: &VehicleState) {
        if !self.airborne(airborne_height_m, state) {
            self.ground_altitude_m = Some(state.position_m.z).filter(|z| z.is_finite());
            self.estimator.reset();
        }
    }

    /// Pauses or resumes the updates; the estimate is kept either way.
    pub(crate) fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
    }

    /// The estimator's wind.
    pub(crate) fn estimate(&self) -> Option<WindEstimate> {
        self.estimator.estimate()
    }

    /// Whether `state` is more than `airborne_height_m` above the ground altitude.
    fn airborne(&self, airborne_height_m: f32, state: &VehicleState) -> bool {
        self.ground_altitude_m
            .is_some_and(|ground_m| state.position_m.z > ground_m + airborne_height_m)
    }
}
