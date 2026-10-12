//! [`View`]: runs a [`Guidance`] or [`Stabilizer`] in a body frame rotated by a constant, so a
//! tailsitter hovers nose up on the copter controllers.
//!
//! A tailsitter's body frame is the airplane's (x along the nose), so hover sits at pitch 90°,
//! the Euler singularity, where roll and heading turn about the same axis. Like ArduPilot, the
//! hover controllers run in a **view** rotated 90° in pitch, where nose-up hover reads level:
//! ArduPilot builds its tailsitter's VTOL attitude, position, and navigation controllers on an
//! AHRS view with `ROTATION_PITCH_90` (`ArduPlane/quadplane.cpp`, `create_view`), which rotates
//! the attitude and gyro into the view (`libraries/AP_AHRS/AP_AHRS_View.cpp`). PX4 rotates the
//! other way: its tailsitter's body frame is the hover frame, and its fixed-wing controllers
//! take the rotated view (`_q_mc_to_fw` in `fw_att_control`); a `View` serves either.
//!
//! In the view the vehicle's belly is forward: tilting the nose toward the belly is how a
//! tailsitter tips into forward flight, belly down. Its axes are the airplane's turned: view
//! roll is airplane yaw, view pitch is pitch, and view yaw is airplane −roll. Gains for a viewed
//! controller are in those view axes.
//!
//! The view's Euler angles are singular at view pitch ±90°, which is forward flight, not hover;
//! the transition between the two needs a quaternion attitude error (#124).

use nalgebra::UnitQuaternion;

use crate::controllers::InvalidConfig;
use crate::tuning::{ProbeConfig, ProbeStatus, Tunable};
use crate::{
    Configurable, ControlInput, ControlOutput, Guidance, Saturation, Setpoint, Stabilizer,
    VehicleState,
};

#[cfg(test)]
mod tests;

/// The smallest share of a view axis in a body axis that carries a saturation flag across:
/// above float noise (a 90° rotation leaves ~1e-7 off-axis).
const AXIS_SHARE: f32 = 1e-3;

/// A [`Guidance`] or [`Stabilizer`] run in a body frame rotated by a constant.
///
/// `rotation` is the view's frame expressed in the vehicle's body frame. Each tick the inner
/// component sees the state in the view ([`VehicleState::rotate_body`]); a viewed stabilizer's
/// moments are rotated back into the body frame for the mixer, and its per-axis saturation
/// flags with them. Config ([`Configurable`], so params too) and autotune probes ([`Tunable`])
/// pass through to the inner component, in view axes. The autotune supervisor
/// ([`Autotune`](crate::tuning::autotune::Autotune)) still checks its envelope's tilt in the
/// body frame, so it can't tune a tailsitter in its view yet
/// ([#321](https://github.com/wboayue/stable-flight/issues/321)).
///
/// Wrap guidance and the stabilizer each in the same view, as ArduPilot builds each tailsitter
/// controller on its view (see the [module docs](self)).
///
/// # Examples
///
/// ```
/// use core::f32::consts::FRAC_PI_2;
/// use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
/// use stable_flight::nalgebra::UnitQuaternion;
/// use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
/// use stable_flight::{AttitudeSetpoint, Setpoint, Stabilizer, VehicleState, View};
///
/// let guidance = View::tailsitter(CopterGuidance::new(CopterGuidanceConfig::default()));
/// let mut stabilizer = View::tailsitter(CopterStabilizer::new(CopterStabilizerConfig::default()));
///
/// // Hovering nose up is level in the view: no attitude error, so no moments.
/// let hover = VehicleState {
///     attitude: UnitQuaternion::from_euler_angles(0.0, FRAC_PI_2, 0.0),
///     ..Default::default()
/// };
/// let setpoint = Setpoint::Attitude(AttitudeSetpoint { thrust: 0.5, ..Default::default() });
/// let output = stabilizer.stabilize(0.004, &setpoint, &hover);
/// assert!(output.moments.norm() < 1e-5);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct View<T> {
    inner: T,
    rotation: UnitQuaternion<f32>,
}

impl<T> View<T> {
    /// `inner`, run in the view `rotation` (the view's frame expressed in the body frame).
    pub fn new(inner: T, rotation: UnitQuaternion<f32>) -> Self {
        View { inner, rotation }
    }

    /// `inner`, run in a tailsitter's hover view: the airplane body frame turned −90° in pitch,
    /// so nose-up hover reads level (ArduPilot's `ROTATION_PITCH_90` view).
    pub fn tailsitter(inner: T) -> Self {
        Self::new(
            inner,
            UnitQuaternion::from_euler_angles(0.0, -core::f32::consts::FRAC_PI_2, 0.0),
        )
    }

    /// The view's frame expressed in the body frame.
    pub fn rotation(&self) -> &UnitQuaternion<f32> {
        &self.rotation
    }

    /// The wrapped component.
    pub fn inner(&self) -> &T {
        &self.inner
    }

    /// Unwraps the component.
    pub fn into_inner(self) -> T {
        self.inner
    }

    fn view_state(&self, state: &VehicleState) -> VehicleState {
        state.rotate_body(&self.rotation)
    }
}

impl<G: Guidance> Guidance for View<G> {
    fn guide(&mut self, dt: f32, input: &ControlInput, state: &VehicleState) -> Setpoint {
        let state = self.view_state(state);
        self.inner.guide(dt, input, &state)
    }

    fn prime(&mut self, state: &VehicleState) {
        let state = self.view_state(state);
        self.inner.prime(&state);
    }

    fn saturation(&self) -> Saturation {
        self.inner.saturation()
    }
}

impl<S: Stabilizer> Stabilizer for View<S> {
    /// Stabilizes in the view, then rotates the moments back into the body frame.
    fn stabilize(&mut self, dt: f32, setpoint: &Setpoint, state: &VehicleState) -> ControlOutput {
        let state = self.view_state(state);
        let output = self.inner.stabilize(dt, setpoint, &state);
        ControlOutput {
            moments: self.rotation * output.moments,
            ..output
        }
    }

    fn prime(&mut self, state: &VehicleState) {
        let state = self.view_state(state);
        self.inner.prime(&state);
    }

    /// The inner stabilizer's report, its per-axis `moments` flags carried into body axes: a
    /// body axis is flagged when a saturated view axis feeds it (for a 90° view, the flags are
    /// relabeled).
    fn saturation(&self) -> Saturation {
        let inner = self.inner.saturation();
        let to_body = self.rotation.to_rotation_matrix();
        let moments = core::array::from_fn(|body| {
            (0..3).any(|view| inner.moments[view] && to_body[(body, view)].abs() > AXIS_SHARE)
        });
        Saturation { moments, ..inner }
    }
}

impl<T: Configurable> Configurable for View<T> {
    type Config = T::Config;

    fn config(&self) -> &T::Config {
        self.inner.config()
    }

    fn set_config(&mut self, config: T::Config) -> Result<(), InvalidConfig> {
        self.inner.set_config(config)
    }
}

impl<S: Tunable> Tunable for View<S> {
    type Target = S::Target;

    fn start_probe(&mut self, target: S::Target, config: ProbeConfig) {
        self.inner.start_probe(target, config);
    }

    fn probe(&self) -> Option<(S::Target, ProbeStatus)> {
        self.inner.probe()
    }

    fn stop_probe(&mut self) -> Option<(S::Target, ProbeStatus)> {
        self.inner.stop_probe()
    }
}
