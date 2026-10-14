use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::airframes;

fn model() -> TailsitterModel {
    airframes::tailsitter().model
}

#[test]
fn rejects_out_of_range_parameters() {
    // (case, model, error)
    let cases = [
        (
            "wing",
            TailsitterModel {
                wing: FixedWingModel {
                    mass_kg: 0.0,
                    ..model().wing
                },
                ..model()
            },
            ModelError::NotPositive("mass_kg"),
        ),
        (
            "arm",
            TailsitterModel {
                motor_arm_m: -0.1,
                ..model()
            },
            ModelError::NotPositive("motor_arm_m"),
        ),
        (
            "disc",
            TailsitterModel {
                prop_wash: PropWash {
                    disc_area_m2: f32::NAN,
                    ..model().prop_wash
                },
                ..model()
            },
            ModelError::NotPositive("prop_wash.disc_area_m2"),
        ),
        (
            "fraction above 1",
            TailsitterModel {
                prop_wash: PropWash {
                    surface_fraction: 1.5,
                    ..model().prop_wash
                },
                ..model()
            },
            ModelError::NotFraction("prop_wash.surface_fraction"),
        ),
        (
            "fraction negative",
            TailsitterModel {
                prop_wash: PropWash {
                    surface_fraction: -0.1,
                    ..model().prop_wash
                },
                ..model()
            },
            ModelError::NotFraction("prop_wash.surface_fraction"),
        ),
    ];
    for (case, model, error) in cases {
        check!(model.validate() == Err(error), "{case}");
    }
    check!(super::TailsitterModel { ..model() }.validate() == Ok(()));
}

/// Momentum theory: nose first, the far wake adds thrust over disc area; tail first, the sink
/// eats into the jet until, sinking as fast as it, the surfaces see no flow at all.
#[test]
fn wash_pressure_follows_momentum_theory() {
    let wash = PropWash {
        disc_area_m2: 0.1,
        surface_fraction: 1.0,
    };
    // 20 N over 0.1 m²: 200 Pa, a static jet of √(400/ρ) m/s.
    let jet_mps = libm::sqrtf(400.0 / AIR_DENSITY_KGPM3);
    let free_pa = |speed_mps: f32| 0.5 * AIR_DENSITY_KGPM3 * speed_mps * speed_mps;
    // (case, thrust N, axial m/s, added pressure Pa)
    let cases = [
        ("no thrust", 0.0, 0.0, 0.0),
        ("hovering", 20.0, 0.0, 200.0),
        ("climbing", 20.0, 5.0, 200.0),
        (
            "sinking at the jet's speed",
            20.0,
            -jet_mps,
            -free_pa(jet_mps),
        ),
    ];
    for (case, thrust_n, axial_mps, expected_pa) in cases {
        let pressure_pa = wash.pressure_pa(thrust_n, axial_mps);
        check!(
            (pressure_pa - expected_pa).abs() <= 1e-3,
            "{case}: {pressure_pa} Pa"
        );
    }
    // Sinking 2 m/s: the jet less the sink, in place of the sink's own pressure.
    let sinking_pa = wash.pressure_pa(20.0, -2.0);
    assert_relative_eq!(
        sinking_pa + free_pa(2.0),
        free_pa(jet_mps - 2.0),
        max_relative = 1e-4
    );
    check!(sinking_pa < wash.pressure_pa(20.0, 0.0));
    // The surface fraction scales it.
    let half = PropWash {
        surface_fraction: 0.5,
        ..wash
    };
    check!(half.pressure_pa(20.0, 0.0) == 100.0);
}
