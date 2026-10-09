//! A flat name → `f32` view over the config tree, for live tuning from a ground station.
//!
//! The config is the only store: [`Params::param`] reads a field of the component's config,
//! and [`Params::set_param`] writes a copy and commits it through
//! [`Configurable::set_config`], which validates it (an invalid value is rejected and the
//! config unchanged) and applies the integrator policy. Any [`Configurable`] whose config
//! implements [`ParamTable`] gets [`Params`]; [`FlightLoop`](crate::FlightLoop) combines its
//! guidance's and stabilizer's.
//!
//! Names are uppercase ASCII of at most [`MAX_NAME_LEN`] bytes (MAVLink's `param_id`),
//! prefixed by component (`STAB_`, `GUID_`) and loop. Vector components end in `_X`/`_Y`/`_Z`
//! (roll/pitch/yaw for angles and rates). Angles and angular rates are in degrees and deg/s,
//! as ground stations show them, and convert to the config's radians at this boundary;
//! everything else is in the config field's own unit.
//!
//! Params are addressed by index, `0..param_count()`, in a stable table order; name lookup
//! scans the names. Index access is what paced output uses (a cursor that sends a few params
//! per tick).
//!
//! Descriptions, groups, and valid ranges for ground stations are in the `metadata` module
//! (feature `param-metadata`), one table per config next to its [`ParamTable`].
//!
//! # Copter stabilizer ([`CopterStabilizerConfig`](crate::stabilizers::copter::CopterStabilizerConfig))
//!
//! | Name | Field | Unit |
//! |---|---|---|
//! | `STAB_RATE_P`, `_I`, `_D` | `body_rate.gains` | |
//! | `STAB_RATE_YAW_P`, `_I`, `_D` | `body_rate.yaw_gains` | |
//! | `STAB_RATE_IMAX` | `body_rate.max_integral` | rad |
//! | `STAB_ATT_P`, `_I`, `_D` | `attitude.gains` | |
//! | `STAB_ATT_AMAX_X`, `_Y` | `attitude.max_angles_rad` | deg |
//! | `STAB_ATT_IMAX` | `attitude.max_integral` | rad·s |
//! | `STAB_ATT_RMAX_X`, `_Y` | `attitude.max_rate_radps` | deg/s |
//! | `STAB_YAW_P`, `_I`, `_D` | `yaw.gains` | |
//! | `STAB_YAW_IMAX` | `yaw.max_integral` | rad·s |
//! | `STAB_YAW_RMAX` | `yaw.max_rate_radps` | deg/s |
//! | `STAB_FS_RATE_X`, `_Y`, `_Z` | `full_scale_rates_radps` | deg/s |
//! | `STAB_THR_EXPO` | `thrust_curve_exponent` | |
//!
//! # Copter guidance ([`CopterGuidanceConfig`](crate::guidance::copter::CopterGuidanceConfig))
//!
//! | Name | Field | Unit |
//! |---|---|---|
//! | `GUID_ALT_P`, `_I`, `_D` | `altitude.gains` | |
//! | `GUID_ALT_IMAX` | `altitude.max_integral` | m·s |
//! | `GUID_ALT_HOVER` | `altitude.hover_thrust` | |
//! | `GUID_LAT_POS_P` | `lateral.position_p` | 1/s |
//! | `GUID_LAT_VEL_P`, `_I`, `_D` | `lateral.velocity` | |
//! | `GUID_LAT_IMAX` | `lateral.max_integral` | m |
//! | `GUID_NAV_PERIOD` | `navigation_period_s` | s |
//! | `GUID_TILT_MAX` | `max_tilt_rad` | deg |
//! | `GUID_STICK_X`, `_Y` | `max_stick_angles_rad` | deg |
//!
//! # Examples
//!
//! ```
//! use stable_flight::params::Params;
//! use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
//! use stable_flight::Configurable;
//!
//! let mut stabilizer = CopterStabilizer::new(CopterStabilizerConfig::default());
//!
//! stabilizer.set_param("STAB_RATE_P", 0.2).expect("valid value");
//! assert_eq!(stabilizer.param("STAB_RATE_P"), Some(0.2));
//! assert_eq!(stabilizer.config().body_rate.gains.p, 0.2);
//!
//! stabilizer.set_param("STAB_ATT_AMAX_X", 30.0).expect("valid value"); // degrees
//! assert!((stabilizer.config().attitude.max_angles_rad.x - 30f32.to_radians()).abs() < 1e-6);
//!
//! assert!(stabilizer.set_param("STAB_RATE_IMAX", -1.0).is_err()); // fails validation
//! assert!(stabilizer.set_param("NO_SUCH_PARAM", 1.0).is_err());
//!
//! let mut count = 0;
//! stabilizer.visit_params(&mut |_name, _value| count += 1);
//! assert_eq!(count, 24);
//! ```

use core::fmt;

use crate::Configurable;
use crate::controllers::InvalidConfig;

#[cfg(feature = "param-metadata")]
pub mod metadata;
#[cfg(test)]
mod tests;

/// Longest param name in bytes: MAVLink's `param_id` field.
pub const MAX_NAME_LEN: usize = 16;

/// One named `f32` field of a config `C`: how to read it and how to write it, unvalidated.
pub struct Param<C> {
    name: &'static str,
    get: fn(&C) -> f32,
    set: fn(&mut C, f32),
}

impl<C> Param<C> {
    /// A param named `name`, read by `get` and written (unvalidated) by `set`, in the param's
    /// unit. `get` must read back what `set` wrote, up to unit-conversion rounding: no
    /// clamping or quantizing (the [`Params`] stores-or-rejects contract). Use it to give your
    /// own config a [`ParamTable`]:
    ///
    /// ```
    /// use stable_flight::params::{Param, ParamTable};
    ///
    /// #[derive(Clone, Copy)]
    /// struct TrimConfig {
    ///     roll_trim_rad: f32,
    /// }
    ///
    /// impl ParamTable for TrimConfig {
    ///     const PARAMS: &'static [Param<Self>] = &[Param::new(
    ///         "TRIM_ROLL",
    ///         |c| c.roll_trim_rad.to_degrees(),
    ///         |c, v| c.roll_trim_rad = v.to_radians(),
    ///     )];
    /// }
    ///
    /// let mut config = TrimConfig { roll_trim_rad: 0.0 };
    /// TrimConfig::PARAMS[0].set(&mut config, 90.0);
    /// assert!((config.roll_trim_rad - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
    /// ```
    pub const fn new(name: &'static str, get: fn(&C) -> f32, set: fn(&mut C, f32)) -> Self {
        Self { name, get, set }
    }

    /// The param's name, e.g. `"STAB_RATE_P"`.
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// The param's value in `config`, in the param's unit.
    pub fn get(&self, config: &C) -> f32 {
        (self.get)(config)
    }

    /// Writes `value` (in the param's unit) into `config` without validating it.
    pub fn set(&self, config: &mut C, value: f32) {
        (self.set)(config, value)
    }
}

/// A config that exposes its fields as named params.
///
/// Names must be unique within a table and across the components combined into one view
/// (e.g. a [`FlightLoop`](crate::FlightLoop)'s guidance and stabilizer), and at most
/// [`MAX_NAME_LEN`] bytes.
pub trait ParamTable: Copy + 'static {
    /// Every param, in a stable order.
    const PARAMS: &'static [Param<Self>];
}

/// Read, write, and list a component's tunable values by index or by name.
///
/// Implementors provide the four index methods; name lookup, [`param`](Params::param),
/// [`set_param`](Params::set_param), and [`visit_params`](Params::visit_params) are built on
/// them.
///
/// **Contract: a successful write stores the value as given**, up to the rounding of a unit
/// conversion (degrees ↔ radians); an out-of-range value is rejected, never clamped or
/// adjusted. Implementors must uphold it (for the blanket impl: each [`Param`] reads back what
/// it wrote, and `set_config` commits the config unchanged or rejects it), because a link
/// confirms a write by echoing the value it was sent.
pub trait Params {
    /// Number of params; their indices are `0..param_count()`.
    fn param_count(&self) -> usize;

    /// The name of the param at `index`, or `None` if it is out of range.
    fn param_name(&self, index: usize) -> Option<&'static str>;

    /// The value of the param at `index`, or `None` if it is out of range.
    fn param_at(&self, index: usize) -> Option<f32>;

    /// Sets the param at `index` to `value`. Out of range is [`ParamError::Unknown`]. On error
    /// the config is unchanged.
    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError>;

    /// The index of the param called `name`, or `None` if there is no such param. On a
    /// duplicate name, the first index.
    fn param_index(&self, name: &str) -> Option<usize> {
        (0..self.param_count()).find(|&i| self.param_name(i) == Some(name))
    }

    /// The value of the param called `name`, or `None` if there is no such param.
    fn param(&self, name: &str) -> Option<f32> {
        self.param_at(self.param_index(name)?)
    }

    /// Sets the param called `name` to `value`. On error the config is unchanged.
    fn set_param(&mut self, name: &str, value: f32) -> Result<(), ParamError> {
        let index = self.param_index(name).ok_or(ParamError::Unknown)?;
        self.set_param_at(index, value)
    }

    /// Calls `f` with every param's name and value, in index order.
    fn visit_params(&self, f: &mut dyn FnMut(&'static str, f32)) {
        for i in 0..self.param_count() {
            if let (Some(name), Some(value)) = (self.param_name(i), self.param_at(i)) {
                f(name, value);
            }
        }
    }
}

impl<T> Params for T
where
    T: Configurable,
    T::Config: ParamTable,
{
    fn param_count(&self) -> usize {
        T::Config::PARAMS.len()
    }

    fn param_name(&self, index: usize) -> Option<&'static str> {
        T::Config::PARAMS.get(index).map(Param::name)
    }

    fn param_at(&self, index: usize) -> Option<f32> {
        T::Config::PARAMS.get(index).map(|p| p.get(self.config()))
    }

    fn set_param_at(&mut self, index: usize, value: f32) -> Result<(), ParamError> {
        let param = T::Config::PARAMS.get(index).ok_or(ParamError::Unknown)?;
        if !value.is_finite() {
            return Err(ParamError::NotFinite);
        }
        let mut config = *self.config();
        param.set(&mut config, value);
        self.set_config(config).map_err(ParamError::Invalid)
    }
}

/// Why a param write ([`Params::set_param`], [`Params::set_param_at`]) was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParamError {
    /// No param has that name or index.
    Unknown,
    /// The value is NaN or infinite.
    NotFinite,
    /// The new config failed validation.
    Invalid(InvalidConfig),
}

impl fmt::Display for ParamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParamError::Unknown => f.write_str("unknown param"),
            ParamError::NotFinite => f.write_str("param value must be finite"),
            ParamError::Invalid(e) => write!(f, "invalid param value: {e}"),
        }
    }
}

impl core::error::Error for ParamError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            ParamError::Invalid(e) => Some(e),
            _ => None,
        }
    }
}

/// A [`Param`] for a config field path: `param!("NAME", a.b)`, or `param!(deg "NAME", a.b)` to
/// expose a radian field in degrees.
macro_rules! param {
    (deg $name:literal, $($field:ident).+) => {
        $crate::params::Param::new(
            $name,
            |c| c.$($field).+.to_degrees(),
            |c, v| c.$($field).+ = v.to_radians(),
        )
    };
    ($name:literal, $($field:ident).+) => {
        $crate::params::Param::new($name, |c| c.$($field).+, |c, v| c.$($field).+ = v)
    };
}
pub(crate) use param;
