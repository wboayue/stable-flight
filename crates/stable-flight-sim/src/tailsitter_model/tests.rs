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
        (
            "roll fraction negative",
            TailsitterModel {
                prop_wash: PropWash {
                    roll_fraction: -0.1,
                    ..model().prop_wash
                },
                ..model()
            },
            ModelError::NotFraction("prop_wash.roll_fraction"),
        ),
        (
            "damping fraction above 1",
            TailsitterModel {
                prop_wash: PropWash {
                    damping_fraction: 1.5,
                    ..model().prop_wash
                },
                ..model()
            },
            ModelError::NotFraction("prop_wash.damping_fraction"),
        ),
    ];
    for (case, model, error) in cases {
        check!(model.validate() == Err(error), "{case}");
    }
    check!(super::TailsitterModel { ..model() }.validate() == Ok(()));
}

/// Momentum theory: nose first, the far wake adds thrust over disc area; tail first, the sink
/// eats into the jet until, sinking as fast as it, the surfaces see no flow at all. The wash
/// damps at the speed of that flow (its pressure's), times the damping fraction.
#[test]
fn wash_pressure_follows_momentum_theory() {
    let wash = PropWash {
        disc_area_m2: 0.1,
        surface_fraction: 1.0,
        roll_fraction: 1.0,
        damping_fraction: 1.0,
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
        let Wash {
            pressure_pa,
            damping_mps,
            ..
        } = wash.wash(thrust_n, axial_mps);
        check!(
            (pressure_pa - expected_pa).abs() <= 1e-3,
            "{case}: {pressure_pa} Pa"
        );
        let flow_mps = libm::sqrtf(2.0 * expected_pa.max(0.0) / AIR_DENSITY_KGPM3);
        check!(
            (damping_mps - flow_mps).abs() <= 1e-3,
            "{case}: {damping_mps} m/s"
        );
    }
    // Sinking 2 m/s: the jet less the sink, in place of the sink's own pressure.
    let sinking_pa = wash.wash(20.0, -2.0).pressure_pa;
    assert_relative_eq!(
        sinking_pa + free_pa(2.0),
        free_pa(jet_mps - 2.0),
        max_relative = 1e-4
    );
    check!(sinking_pa < wash.wash(20.0, 0.0).pressure_pa);
    // The surface fraction scales the pressure, the roll fraction the roll action's, the
    // damping fraction the damping speed.
    let half = PropWash {
        surface_fraction: 0.5,
        roll_fraction: 0.1,
        damping_fraction: 0.25,
        ..wash
    };
    check!(half.wash(20.0, 0.0).pressure_pa == 100.0);
    check!(half.wash(20.0, 0.0).roll_pressure_pa == 20.0);
    assert_relative_eq!(half.wash(20.0, 0.0).damping_mps, 0.25 * jet_mps);
}

/// The damping share is `∫y²` over the washed strip over `∫y²` to the tip: (arm m, diameter m,
/// span m, share).
#[test]
fn span_damping_share_weights_the_strip_by_its_distance_squared() {
    let cases = [
        // 0.2 to 0.45 m of a 0.7 m half-span: (0.45³ − 0.2³) / 0.7³.
        (0.325, 0.25, 1.4, (0.091_125 - 0.008) / 0.343),
        // The whole half-span: all of it.
        (0.5, 1.0, 1.0, 1.0),
        // Past the tip, clipped to it: 0.4 to 0.5 m of 0.5.
        (0.5, 0.2, 1.0, (0.125 - 0.064) / 0.125),
        // Across the center, clipped to it: 0 to 0.2 m of 1.
        (0.05, 0.3, 2.0, 0.008),
    ];
    for (arm_m, diameter_m, span_m, share) in cases {
        assert_relative_eq!(
            PropWash::span_damping_share(arm_m, diameter_m, span_m),
            share,
            max_relative = 1e-5
        );
    }
}
