//! Wind from the air's push on a rotor-borne vehicle: drag inverted to air velocity, one axis
//! at a time, and filtered on world wind.
//!
//! Thrust acts along the thrust frame's `z`, so the specific force along its `x` and `y` is
//! the air's push alone. Each axis's drag model, a bluff-body term `½ρv²/B` plus rotor
//! momentum drag `μv` (ArduPilot's and PX4's drag fusion: `NavEKF3_core::FuseDragForces`,
//! `Ekf::fuseDrag`), is inverted to the air velocity along it; the wind along it is
//! the ground velocity's component less that. A two-state Kalman filter on wind (north, east)
//! takes each axis as a scalar measurement whose noise is the force noise over the drag's
//! slope there, so an axis with little drag (a tailsitter's span in hover) counts for little,
//! and the wind across it is learned once a turn puts more drag on it. Both stacks fit the
//! drag frame to the body with thrust along `z` and treat X and Y alike apart from their
//! coefficients; per-axis noise is what lets a flat-plate airframe use it.
//!
//! The measurement is averaged over [`UPDATE_PERIOD_S`], as both stacks do, before each
//! update. Allocation-free and fixed-size.

use nalgebra::{Matrix2, UnitQuaternion};

use crate::controllers::{InvalidConfig, check_limit, check_positive};
use crate::math::neu_to_ned;
use crate::{GRAVITY, Vec2, Vec3, VehicleState};

#[cfg(test)]
mod tests;

/// Sea-level air density, in kg/m³ (ISA).
pub const AIR_DENSITY_KGPM3: f32 = 1.225;

/// Seconds the measurement is averaged over before each update: about 5 Hz, as both stacks
/// average their specific force (`NavEKF3_core::SampleDragData`,
/// `EstimatorInterface::setDragData`).
pub const UPDATE_PERIOD_S: f32 = 0.2;

/// Default [`WindConfig::force_noise_mps2`]: ArduPilot's `EK3_DRAG_M_NSE` (`AP_NavEKF3.cpp`).
pub const DEFAULT_FORCE_NOISE_MPS2: f32 = 0.5;

/// Default [`WindConfig::process_noise_mps_per_rt_s`]: lets a steady wind's estimate move about
/// 0.1 m/s a second, slow beside a hover's drift.
pub const DEFAULT_PROCESS_NOISE_MPS_PER_RT_S: f32 = 0.1;

/// Default [`WindConfig::initial_sd_mps`], in m/s: past any wind a rotor-borne vehicle flies
/// in, so the first measurements set the estimate.
pub const DEFAULT_INITIAL_SD_MPS: f32 = 10.0;

/// Default [`WindConfig::gate_sigma`]: both stacks gate drag at 5σ
/// (`NavEKF3_core::FuseDragForces`, `Ekf::fuseDrag`).
pub const DEFAULT_GATE_SIGMA: f32 = 5.0;

/// Cosine of the most tilt, thrust axis from vertical, a tick is used at: 45°. The drag model
/// is a hover's; tilted further (a tailsitter pitching over, a hard maneuver) the wing's lift
/// and the thrust's own tilt dominate the push.
const MIN_TILT_COSINE: f32 = core::f32::consts::FRAC_1_SQRT_2;

/// Default [`WindConfig::max_axial_speed_mps`], in m/s: at 1 m/s a sim tailsitter's calm-air
/// estimate is 0.14 m/s off 10 s after its takeoff, 0.22 m/s with no skip (#608).
pub const DEFAULT_MAX_AXIAL_SPEED_MPS: f32 = 1.0;

/// Least drag slope, in 1/s, an axis's noise is divided by: below it the axis tells almost
/// nothing about the wind, and the noise stays finite.
const MIN_DRAG_SLOPE_PER_S: f32 = 1e-3;

/// The drag model, per thrust-frame axis (`x`, `y`), and the filter's noise.
///
/// With every coefficient 0 (the default, as both stacks ship drag fusion off) the estimator
/// never updates.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WindConfig {
    /// Ballistic coefficient along `x` and `y`, in kg/m²: mass over drag coefficient times
    /// area, so the bluff-body drag along the axis is `½ρ·v·|V|/B` per kg, `v` the air along
    /// the axis and `|V|` the airspeed with the air along the thrust axis counted in (the
    /// vehicle's climb or sink, the wind taken as horizontal), as PX4's `Ekf::fuseDrag`
    /// predicts it (PX4 counts the other axis's air too, which inverting one axis at a time
    /// can't). 0 leaves the term out. Must be non-negative and finite.
    pub ballistic_kgpm2: Vec2,
    /// Momentum drag along `x` and `y`, in 1/s: drag acceleration per m/s of air velocity.
    /// 0 leaves the term out. Must be non-negative and finite.
    pub momentum_per_s: Vec2,
    /// Specific-force noise on each averaged measurement, in m/s². Must be positive and
    /// finite. Defaults to [`DEFAULT_FORCE_NOISE_MPS2`].
    pub force_noise_mps2: f32,
    /// How fast the wind may wander, in m/s per √s (process noise). Must be non-negative and
    /// finite. Defaults to [`DEFAULT_PROCESS_NOISE_MPS_PER_RT_S`].
    pub process_noise_mps_per_rt_s: f32,
    /// Standard deviation of each wind component, in m/s, before the first measurement; it
    /// applies from the estimator's next [`reset`](WindEstimator::reset). Higher: the first
    /// measurements move the estimate more. Must be positive and finite. Defaults to
    /// [`DEFAULT_INITIAL_SD_MPS`].
    pub initial_sd_mps: f32,
    /// How many standard deviations an averaged measurement may be off the estimate before
    /// it is rejected. Higher: takes in a sudden gust sooner, and a bad measurement with it.
    /// Must be positive and finite. Defaults to [`DEFAULT_GATE_SIGMA`].
    pub gate_sigma: f32,
    /// Most ground speed along the thrust axis, in m/s, a tick is used at: the drag model is
    /// a hover's, and climbing or sinking a cambered wing lifts toward its belly, which reads
    /// as wind. Higher: uses more of a climb, and its lift; lower: holds the estimate through
    /// gentler climbs and descents. Must be positive and finite. Defaults to
    /// [`DEFAULT_MAX_AXIAL_SPEED_MPS`].
    pub max_axial_speed_mps: f32,
}

impl Default for WindConfig {
    /// No drag model (the estimator off), and the default noises.
    fn default() -> Self {
        WindConfig {
            ballistic_kgpm2: Vec2::zeros(),
            momentum_per_s: Vec2::zeros(),
            force_noise_mps2: DEFAULT_FORCE_NOISE_MPS2,
            process_noise_mps_per_rt_s: DEFAULT_PROCESS_NOISE_MPS_PER_RT_S,
            initial_sd_mps: DEFAULT_INITIAL_SD_MPS,
            gate_sigma: DEFAULT_GATE_SIGMA,
            max_axial_speed_mps: DEFAULT_MAX_AXIAL_SPEED_MPS,
        }
    }
}

impl WindConfig {
    /// Checks that the coefficients and `process_noise_mps_per_rt_s` are non-negative and
    /// finite, and that `force_noise_mps2`, `initial_sd_mps`, `gate_sigma`, and
    /// `max_axial_speed_mps` are positive and finite.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::estimation::wind::WindConfig;
    ///
    /// let mut config = WindConfig::default();
    /// assert!(config.validate().is_ok());
    /// config.force_noise_mps2 = 0.0;
    /// assert!(config.validate().is_err());
    /// ```
    // Inlined so the firmware's checks of a constant config fold away at compile time:
    // outlined, it takes 1.3 kB of flash (`just fw-size`).
    #[inline(always)]
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("ballistic_kgpm2.x", self.ballistic_kgpm2.x)?;
        check_limit("ballistic_kgpm2.y", self.ballistic_kgpm2.y)?;
        check_limit("momentum_per_s.x", self.momentum_per_s.x)?;
        check_limit("momentum_per_s.y", self.momentum_per_s.y)?;
        check_positive("force_noise_mps2", self.force_noise_mps2)?;
        check_limit(
            "process_noise_mps_per_rt_s",
            self.process_noise_mps_per_rt_s,
        )?;
        check_positive("initial_sd_mps", self.initial_sd_mps)?;
        check_positive("gate_sigma", self.gate_sigma)?;
        check_positive("max_axial_speed_mps", self.max_axial_speed_mps)
    }

    /// Whether any drag coefficient is set, so the estimator can update.
    pub fn is_on(&self) -> bool {
        self.ballistic_kgpm2
            .iter()
            .chain(&self.momentum_per_s)
            .any(|c| *c > 0.0)
    }

    /// The drag per kg, in m/s² along the thrust frame's `x` and `y`, on a vehicle moving
    /// through the air at `air_mps` along those axes (its velocity less the wind's), level:
    /// `−(k|v| + μ)v` per axis, opposing the air velocity. Zero on an axis without a
    /// coefficient.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::Vec2;
    /// use stable_flight::estimation::wind::WindConfig;
    ///
    /// // Momentum drag 0.3/s: 2 m/s forward through the air is pushed back at 0.6 m/s².
    /// let config = WindConfig { momentum_per_s: Vec2::new(0.3, 0.3), ..Default::default() };
    /// let drag = config.drag_mps2(Vec2::new(2.0, 0.0));
    /// assert!((drag - Vec2::new(-0.6, 0.0)).norm() < 1e-6);
    /// ```
    pub fn drag_mps2(&self, air_mps: Vec2) -> Vec2 {
        let along =
            |i: usize| -AxisDrag::new(self, i).push_mps2(air_mps[i], 0.0) * air_mps[i].signum();
        Vec2::new(along(0), along(1))
    }
}

/// The wind and its uncertainty.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct WindEstimate {
    /// The air mass's velocity (north, east), in m/s: where the wind blows toward.
    pub velocity_mps: Vec2,
    /// Covariance of `velocity_mps`, in m²/s².
    pub covariance_m2ps2: Matrix2<f32>,
}

impl WindEstimate {
    /// Standard deviation of the wind along the horizontal unit vector `direction`, in m/s.
    pub fn sd_along_mps(&self, direction: Vec2) -> f32 {
        libm::sqrtf(
            (direction.transpose() * self.covariance_m2ps2 * direction)
                .x
                .max(0.0),
        )
    }
}

/// Estimates the wind from the vehicle state while the caller says it is airborne on rotors.
///
/// The config is passed to each call rather than stored, so the owner's config stays the
/// single source of truth.
///
/// # Examples
///
/// ```
/// use stable_flight::estimation::wind::{WindConfig, WindEstimator};
/// use stable_flight::{Vec2, Vec3, VehicleState};
///
/// // A level quad with momentum drag 0.3/s, not moving, pushed east at 0.6 m/s²: the air
/// // moves east at 2 m/s.
/// let config = WindConfig {
///     momentum_per_s: Vec2::new(0.3, 0.3),
///     ..Default::default()
/// };
/// let state = VehicleState {
///     acceleration_mps2: Vec3::new(0.0, 0.6, 0.0),
///     ..Default::default()
/// };
/// let mut estimator = WindEstimator::default();
/// for _ in 0..2500 {
///     estimator.update(0.004, &config, &state);
/// }
/// let wind = estimator.estimate().expect("updated").velocity_mps;
/// assert!((wind - Vec2::new(0.0, 2.0)).norm() < 0.01);
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WindEstimator {
    /// The wind and its covariance; `None` until the first measurement, which seeds the
    /// covariance from [`WindConfig::initial_sd_mps`].
    filter: Option<WindEstimate>,
    /// The measurement being averaged.
    window: Window,
}

/// Sums over one averaging period.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Window {
    elapsed_s: f32,
    ticks: u32,
    /// Specific force along the thrust frame's `x` and `y`, in m/s².
    specific_force_mps2: Vec2,
    /// Ground velocity along each axis, in m/s.
    ground_along_mps: Vec2,
    /// Each axis's horizontal part (north, east).
    axis_x: Vec2,
    axis_y: Vec2,
    /// Ground velocity along the thrust axis, in m/s.
    axial_mps: f32,
}

impl WindEstimator {
    /// Adds this tick's state, `state.attitude` taken as the thrust frame (thrust along its
    /// `z`), and once [`UPDATE_PERIOD_S`] has gathered, updates the wind from the period's
    /// average. A tick with a non-finite attitude, velocity, or acceleration, with the thrust
    /// axis tilted more than 45° from vertical, or moving along it faster than
    /// [`max_axial_speed_mps`](WindConfig::max_axial_speed_mps), is skipped. Does nothing while
    /// `config` has no drag coefficient.
    ///
    /// Each axis's measurement counts by the drag's slope at the air velocity the estimate
    /// predicts, as both EKFs linearize their drag at the state (ArduPilot's
    /// `NavEKF3_core::FuseDragForces`, PX4's `Ekf::fuseDrag`), or at the measured one where
    /// that is steeper. Bluff-body drag has no slope in still air: taken at a measured push near
    /// zero alone, the slope would leave a wrong estimate there for good, and taken at the
    /// prediction alone, a fresh estimate of no wind would discount a real wind's push (#608).
    pub fn update(&mut self, dt: f32, config: &WindConfig, state: &VehicleState) {
        if !config.is_on() {
            return;
        }
        let Some(sample) = Sample::from_state(state, config.max_axial_speed_mps) else {
            return;
        };
        let window = &mut self.window;
        window.elapsed_s += dt;
        window.ticks += 1;
        window.specific_force_mps2 += sample.specific_force_mps2;
        window.ground_along_mps += sample.ground_along_mps;
        window.axis_x += sample.axis_x;
        window.axis_y += sample.axis_y;
        window.axial_mps += sample.axial_mps;
        if window.elapsed_s < UPDATE_PERIOD_S {
            return;
        }
        let window = core::mem::take(&mut self.window);
        let n = window.ticks as f32;
        let initial_variance_m2ps2 = config.initial_sd_mps * config.initial_sd_mps;
        let estimate = self.filter.get_or_insert(WindEstimate {
            velocity_mps: Vec2::zeros(),
            covariance_m2ps2: Matrix2::identity() * initial_variance_m2ps2,
        });
        estimate.predict(window.elapsed_s, config);
        let force = window.specific_force_mps2 / n;
        let ground = window.ground_along_mps / n;
        let axes = [window.axis_x / n, window.axis_y / n];
        let axial_mps = window.axial_mps / n;
        for (i, axis) in axes.into_iter().enumerate() {
            let drag = AxisDrag::new(config, i);
            if let Some(air_mps) = drag.air_velocity(force[i], axial_mps) {
                let predicted_air_mps = ground[i] - axis.dot(&estimate.velocity_mps);
                let slope_per_s = drag
                    .slope_per_s(predicted_air_mps, axial_mps)
                    .max(drag.slope_per_s(air_mps, axial_mps));
                let noise_mps = config.force_noise_mps2 / slope_per_s.max(MIN_DRAG_SLOPE_PER_S);
                estimate.correct(
                    axis,
                    ground[i] - air_mps,
                    noise_mps * noise_mps,
                    config.gate_sigma,
                );
            }
        }
    }

    /// The wind, once a measurement has been taken.
    pub fn estimate(&self) -> Option<WindEstimate> {
        self.filter
    }

    /// Forgets the wind and any partial average.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// The filter's steps, on the estimate it holds.
impl WindEstimate {
    /// Grows the covariance for `elapsed_s` of wandering wind.
    fn predict(&mut self, elapsed_s: f32, config: &WindConfig) {
        let q = config.process_noise_mps_per_rt_s;
        self.covariance_m2ps2 += Matrix2::identity() * (q * q * elapsed_s);
    }

    /// Fuses `measured_mps`, the wind along `axis` (horizontal, not unit: a tilted axis
    /// reaches less of it), with variance `variance_m2ps2`; rejected past `gate_sigma`
    /// standard deviations.
    fn correct(&mut self, axis: Vec2, measured_mps: f32, variance_m2ps2: f32, gate_sigma: f32) {
        let p_h = self.covariance_m2ps2 * axis;
        let innovation_variance = axis.dot(&p_h) + variance_m2ps2;
        let innovation = measured_mps - axis.dot(&self.velocity_mps);
        let gate = gate_sigma * gate_sigma * innovation_variance;
        let within_gate = innovation * innovation <= gate;
        // `<=` is false for a NaN innovation: rejected.
        if !within_gate || !innovation_variance.is_finite() {
            return;
        }
        let gain = p_h / innovation_variance;
        self.velocity_mps += gain * innovation;
        self.covariance_m2ps2 -= gain * p_h.transpose();
        // Symmetric against rounding.
        let off = 0.5 * (self.covariance_m2ps2[(0, 1)] + self.covariance_m2ps2[(1, 0)]);
        self.covariance_m2ps2[(0, 1)] = off;
        self.covariance_m2ps2[(1, 0)] = off;
    }
}

/// The specific force on the vehicle (what an accelerometer reads: acceleration less gravity),
/// in m/s², in NED, the frame `state.attitude` rotates the body into: `(0, 0, −g)` at rest.
///
/// # Examples
///
/// ```
/// use stable_flight::estimation::wind::specific_force_ned;
/// use stable_flight::{GRAVITY, Vec3, VehicleState};
///
/// let resting = specific_force_ned(&VehicleState::default());
/// assert_eq!(resting, Vec3::new(0.0, 0.0, -GRAVITY));
/// ```
pub fn specific_force_ned(state: &VehicleState) -> Vec3 {
    let a = state.acceleration_mps2;
    Vec3::new(a.x, a.y, -a.z - GRAVITY)
}

/// One tick's measurement.
struct Sample {
    specific_force_mps2: Vec2,
    ground_along_mps: Vec2,
    axis_x: Vec2,
    axis_y: Vec2,
    axial_mps: f32,
}

impl Sample {
    /// From `state`, its attitude taken as the thrust frame; `None` if anything used is
    /// non-finite, or the thrust axis tilted past 45° or moving along it faster than
    /// `max_axial_speed_mps`.
    fn from_state(state: &VehicleState, max_axial_speed_mps: f32) -> Option<Self> {
        let attitude: &UnitQuaternion<f32> = &state.attitude;
        // NED, as the attitude rotates the FRD body into it.
        let specific_force_ned = specific_force_ned(state);
        let velocity_ned = neu_to_ned(state.velocity_mps);
        // One matrix serves them all: its columns are the body axes in NED.
        let rotation = attitude.to_rotation_matrix();
        // `<` is false for NaN; the finiteness check below catches it.
        if rotation.matrix()[(2, 2)] < MIN_TILT_COSINE {
            return None;
        }
        let force = rotation.inverse_transform_vector(&specific_force_ned);
        let axis_x: Vec3 = rotation.matrix().column(0).into();
        let axis_y: Vec3 = rotation.matrix().column(1).into();
        let axis_z: Vec3 = rotation.matrix().column(2).into();
        let axial_mps = axis_z.dot(&velocity_ned);
        // `>` is false for NaN; the finiteness check below catches it.
        if axial_mps.abs() > max_axial_speed_mps {
            return None;
        }
        let sample = Sample {
            specific_force_mps2: force.xy(),
            ground_along_mps: Vec2::new(axis_x.dot(&velocity_ned), axis_y.dot(&velocity_ned)),
            axis_x: axis_x.xy(),
            axis_y: axis_y.xy(),
            axial_mps,
        };
        let finite = [
            sample.specific_force_mps2,
            sample.ground_along_mps,
            sample.axis_x,
            sample.axis_y,
        ]
        .iter()
        .all(|v| v.iter().all(|c| c.is_finite()))
            && sample.axial_mps.is_finite();
        finite.then_some(sample)
    }
}

/// One axis's drag per kg against air velocity `v` along it, with `w` m/s of air along the
/// thrust axis: `−(k|V| + μ)v`, `|V| = √(v² + w²)`, `k = ½ρ/B`. A climb or sink adds to the
/// bluff body's airspeed, so the same push reads as less air along the axis.
struct AxisDrag {
    /// Bluff-body term, in 1/m (0 without one).
    k_per_m: f32,
    /// Momentum term, in 1/s.
    mu_per_s: f32,
}

/// Newton steps refining [`AxisDrag::air_velocity`]'s root: from the still-air root, above it,
/// four land within 1e-4 of it across hover speeds and climbs (`axis_drag_round_trips`).
const AIR_VELOCITY_STEPS: usize = 4;

impl AxisDrag {
    /// Axis `i` (0 `x`, 1 `y`) of `config`.
    fn new(config: &WindConfig, i: usize) -> Self {
        let ballistic = config.ballistic_kgpm2[i];
        AxisDrag {
            k_per_m: if ballistic > 0.0 {
                0.5 * AIR_DENSITY_KGPM3 / ballistic
            } else {
                0.0
            },
            mu_per_s: config.momentum_per_s[i],
        }
    }

    /// The air velocity, in m/s, along the axis that the drag `force_mps2` comes from, with
    /// `axial_mps` of air along the thrust axis; `None` without a drag model on the axis.
    fn air_velocity(&self, force_mps2: f32, axial_mps: f32) -> Option<f32> {
        let (k, mu) = (self.k_per_m, self.mu_per_s);
        let push = force_mps2.abs();
        let speed = if k > 0.0 {
            // k·s² + μ·s = |f|, the positive root: the speed without air along the thrust
            // axis, at or above the root with it.
            let still_air = (libm::sqrtf(mu * mu + 4.0 * k * push) - mu) / (2.0 * k);
            // Drag rises and curves up with speed, so Newton's steps from above stay above the
            // root and close on it.
            (0..AIR_VELOCITY_STEPS).fold(still_air, |s, _| {
                let excess = self.push_mps2(s, axial_mps) - push;
                let slope = self.slope_per_s(s, axial_mps);
                if slope > 0.0 { s - excess / slope } else { s }
            })
        } else if mu > 0.0 {
            push / mu
        } else {
            return None;
        };
        // Drag opposes the air velocity.
        Some(if force_mps2 > 0.0 { -speed } else { speed })
    }

    /// The drag's push per kg, in m/s², against `air_mps` of air along the axis and
    /// `axial_mps` along the thrust axis: `(k|V| + μ)|v|`, its size without its sign.
    fn push_mps2(&self, air_mps: f32, axial_mps: f32) -> f32 {
        let airspeed_mps = libm::sqrtf(air_mps * air_mps + axial_mps * axial_mps);
        (self.k_per_m * airspeed_mps + self.mu_per_s) * air_mps.abs()
    }

    /// The drag's slope against the air velocity along the axis, at `air_mps` along it and
    /// `axial_mps` along the thrust axis, in 1/s: `k(|V| + v²/|V|) + μ`.
    fn slope_per_s(&self, air_mps: f32, axial_mps: f32) -> f32 {
        let airspeed_mps = libm::sqrtf(air_mps * air_mps + axial_mps * axial_mps);
        let bluff = if airspeed_mps > 0.0 {
            self.k_per_m * (airspeed_mps + air_mps * air_mps / airspeed_mps)
        } else {
            0.0
        };
        bluff + self.mu_per_s
    }
}
