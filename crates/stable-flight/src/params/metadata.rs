//! Descriptions, units, grouping, and valid ranges for params, for ground stations
//! (`param-metadata` feature): a host-side companion to each [`ParamTable`], so flight-controller
//! firmware, which never enables the feature, carries none of it.
//!
//! A config that implements [`ParamMetadata`] describes each of its params in a
//! [`METADATA`](ParamMetadata::METADATA) table with the same names in the same order as its
//! [`PARAMS`](ParamTable::PARAMS). Ranges restate what the config's `validate` accepts, in the
//! param's unit; the core's tests check each against `validate`, so a range can't drift from
//! validation unnoticed.
//!
//! # Examples
//!
//! ```
//! use stable_flight::params::metadata::ParamMetadata;
//! use stable_flight::stabilizers::copter::CopterStabilizerConfig;
//!
//! let rate_imax = CopterStabilizerConfig::METADATA
//!     .iter()
//!     .find(|m| m.name() == "STAB_RATE_IMAX")
//!     .expect("described");
//! assert_eq!(rate_imax.units(), "rad");
//! assert_eq!(rate_imax.min().map(|b| b.value()), Some(0.0));
//! assert_eq!(rate_imax.max(), None);
//! ```

use super::ParamTable;

#[cfg(test)]
mod tests;

/// One end of a param's valid range, in the param's unit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bound {
    value: f32,
    inclusive: bool,
}

impl Bound {
    /// The bound's value.
    pub const fn value(&self) -> f32 {
        self.value
    }

    /// Whether `value` itself is valid (`[` rather than `(`).
    pub const fn inclusive(&self) -> bool {
        self.inclusive
    }
}

/// What a ground station shows for one param: its description, unit, where it is grouped, and
/// its valid range. Built from a per-group template:
///
/// ```
/// use stable_flight::params::metadata::ParamMeta;
///
/// const LIMITS: ParamMeta = ParamMeta::in_group("Guidance", "Limits");
/// const TILT: ParamMeta = LIMITS
///     .param("GUID_TILT_MAX", "Largest tilt waypoint flight may request")
///     .with_units("deg")
///     .above(0.0)
///     .below(90.0);
///
/// assert_eq!(TILT.group(), "Limits");
/// assert!(!TILT.max().expect("bounded").inclusive());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamMeta {
    name: &'static str,
    description: &'static str,
    units: &'static str,
    category: &'static str,
    group: &'static str,
    min: Option<Bound>,
    max: Option<Bound>,
}

impl ParamMeta {
    /// A template for the params of one `group` (e.g. `"Rate"`) within a `category` (the
    /// component, e.g. `"Stabilizer"`): no name, unit, or range yet. Name it with
    /// [`param`](Self::param).
    pub const fn in_group(category: &'static str, group: &'static str) -> Self {
        Self {
            name: "",
            description: "",
            units: "",
            category,
            group,
            min: None,
            max: None,
        }
    }

    /// This template as the param `name`, described by a one-line `description`.
    pub const fn param(self, name: &'static str, description: &'static str) -> Self {
        Self {
            name,
            description,
            ..self
        }
    }

    /// With `units` as a ground station shows them, e.g. `"deg/s"`; empty for a dimensionless
    /// param (gains, fractions).
    pub const fn with_units(self, units: &'static str) -> Self {
        Self { units, ..self }
    }

    /// Valid from `value` up, `value` included.
    pub const fn at_least(self, value: f32) -> Self {
        self.with_min(value, true)
    }

    /// Valid above `value`, `value` excluded.
    pub const fn above(self, value: f32) -> Self {
        self.with_min(value, false)
    }

    /// Valid up to `value`, `value` included.
    pub const fn at_most(self, value: f32) -> Self {
        self.with_max(value, true)
    }

    /// Valid below `value`, `value` excluded.
    pub const fn below(self, value: f32) -> Self {
        self.with_max(value, false)
    }

    const fn with_min(self, value: f32, inclusive: bool) -> Self {
        Self {
            min: Some(Bound { value, inclusive }),
            ..self
        }
    }

    const fn with_max(self, value: f32, inclusive: bool) -> Self {
        Self {
            max: Some(Bound { value, inclusive }),
            ..self
        }
    }

    /// The param's name, as in its [`ParamTable`].
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// One-line description.
    pub const fn description(&self) -> &'static str {
        self.description
    }

    /// Units, e.g. `"deg/s"`; empty when dimensionless.
    pub const fn units(&self) -> &'static str {
        self.units
    }

    /// The component the param belongs to, e.g. `"Stabilizer"`.
    pub const fn category(&self) -> &'static str {
        self.category
    }

    /// The loop or feature within the component, e.g. `"Rate"`.
    pub const fn group(&self) -> &'static str {
        self.group
    }

    /// The lower end of the valid range; `None` when unbounded below.
    pub const fn min(&self) -> Option<Bound> {
        self.min
    }

    /// The upper end of the valid range; `None` when unbounded above.
    pub const fn max(&self) -> Option<Bound> {
        self.max
    }
}

/// A [`ParamTable`] whose params are described for ground stations.
pub trait ParamMetadata: ParamTable {
    /// One entry per param: the same names, in the same order, as
    /// [`PARAMS`](ParamTable::PARAMS).
    const METADATA: &'static [ParamMeta];
}
