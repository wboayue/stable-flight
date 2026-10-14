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

/// Momentum theory: the far wake's added dynamic pressure is thrust over disc area.
#[test]
fn wash_pressure_is_thrust_over_disc_area() {
    let wash = PropWash {
        disc_area_m2: 0.1,
        surface_fraction: 0.5,
    };
    check!(wash.pressure_pa(0.0) == 0.0);
    check!(wash.pressure_pa(20.0) == 100.0);
}
