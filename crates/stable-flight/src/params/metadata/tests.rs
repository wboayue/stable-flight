use assert2::check;

use super::*;
use crate::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use crate::guidance::plane::{PlaneGuidance, PlaneGuidanceConfig};
use crate::guidance::vtol::{LiftTransition, LiftTransitionConfig, Transition, TransitionConfig};
use crate::params::Params;
use crate::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use crate::stabilizers::plane::{PlaneStabilizer, PlaneStabilizerConfig};

/// Names in `T::METADATA`, then in `T::PARAMS`.
fn names<T: ParamMetadata>() -> (Vec<&'static str>, Vec<&'static str>) {
    (
        T::METADATA.iter().map(ParamMeta::name).collect(),
        T::PARAMS.iter().map(|p| p.name()).collect(),
    )
}

#[test]
fn describes_every_param_in_table_order() {
    // (table, (metadata names, param names), metadata). Every `ParamMetadata` impl gets a row
    // here and in `ranges_match_what_validation_accepts`.
    let cases: [(&str, _, &[ParamMeta]); 6] = [
        (
            "stabilizer",
            names::<CopterStabilizerConfig>(),
            CopterStabilizerConfig::METADATA,
        ),
        (
            "guidance",
            names::<CopterGuidanceConfig>(),
            CopterGuidanceConfig::METADATA,
        ),
        (
            "plane stabilizer",
            names::<PlaneStabilizerConfig>(),
            PlaneStabilizerConfig::METADATA,
        ),
        (
            "plane guidance",
            names::<PlaneGuidanceConfig>(),
            PlaneGuidanceConfig::METADATA,
        ),
        (
            "transition",
            names::<TransitionConfig>(),
            TransitionConfig::METADATA,
        ),
        (
            "lift transition",
            names::<LiftTransitionConfig>(),
            LiftTransitionConfig::METADATA,
        ),
    ];
    for (table, (described, params), metadata) in cases {
        check!(described == params, "{table}");
        for meta in metadata {
            check!(!meta.description().is_empty(), "{}", meta.name());
            check!(!meta.category().is_empty(), "{}", meta.name());
            check!(!meta.group().is_empty(), "{}", meta.name());
        }
    }
}

/// A step just past a bound at `value`: small, but well clear of `f32` and degree-conversion
/// rounding.
fn step(value: f32) -> f32 {
    (value.abs() * 1e-3).max(1e-3)
}

/// Far beyond any physical setting, to show a side really is unbounded.
const FAR: f32 = 1e6;

/// Checks each param's range against what `fresh()`'s `set_param` (that is, `validate`)
/// accepts, from a fresh component per write: a bound is accepted when inclusive and rejected
/// when not, just past it is rejected, just inside it is accepted, and an unbounded side
/// accepts [`FAR`].
fn check_ranges<P: Params>(metadata: &[ParamMeta], fresh: impl Fn() -> P) {
    let accepts = |name: &str, value: f32| fresh().set_param(name, value).is_ok();
    for meta in metadata {
        let name = meta.name();
        match meta.min() {
            Some(min) => {
                let v = min.value();
                check!(accepts(name, v) == min.inclusive(), "{name} at min {v}");
                check!(!accepts(name, v - step(v)), "{name} below min {v}");
                check!(accepts(name, v + step(v)), "{name} just above min {v}");
            }
            None => {
                check!(accepts(name, -FAR), "{name} unbounded below");
            }
        }
        match meta.max() {
            Some(max) => {
                let v = max.value();
                check!(accepts(name, v) == max.inclusive(), "{name} at max {v}");
                check!(!accepts(name, v + step(v)), "{name} above max {v}");
                check!(accepts(name, v - step(v)), "{name} just below max {v}");
            }
            None => {
                check!(accepts(name, FAR), "{name} unbounded above");
            }
        }
    }
}

#[test]
fn ranges_match_what_validation_accepts() {
    check_ranges(CopterStabilizerConfig::METADATA, || {
        CopterStabilizer::new(CopterStabilizerConfig::default())
    });
    check_ranges(CopterGuidanceConfig::METADATA, || {
        CopterGuidance::new(CopterGuidanceConfig::default())
    });
    check_ranges(PlaneStabilizerConfig::METADATA, || {
        PlaneStabilizer::new(PlaneStabilizerConfig::default())
    });
    check_ranges(PlaneGuidanceConfig::METADATA, || {
        PlaneGuidance::new(PlaneGuidanceConfig::default())
    });
    check_ranges(TransitionConfig::METADATA, || {
        Transition::new(TransitionConfig::default())
    });
    check_ranges(LiftTransitionConfig::METADATA, || {
        LiftTransition::new(LiftTransitionConfig::default())
    });
}
