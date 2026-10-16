use assert2::check;

use super::*;

/// The Alia layout: four lift motors on engines 0–3 in `quad_x` order, the pusher idle.
fn alia() -> ActuatorMapping {
    ActuatorMapping {
        motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
        idle_engines: &[4],
        neutral_wings: &[2, 3, 8, 9],
        surfaces: &[],
    }
}

/// Every out-of-range engine or wing, and every engine used twice, is reported.
#[test]
fn validates_engines_and_wings() {
    let motors = |m: [Option<usize>; 2]| {
        let mut motors = [None; ActuatorCommand::SLOTS];
        motors[..2].copy_from_slice(&m);
        motors
    };
    // (mapping, expected)
    let rows = [
        (alia(), Ok(())),
        (ActuatorMapping::default(), Ok(())),
        (
            ActuatorMapping {
                motors: motors([Some(16), None]),
                ..alia()
            },
            Err(MappingError::EngineOutOfRange { engine: 16 }),
        ),
        (
            ActuatorMapping {
                motors: motors([Some(2), Some(2)]),
                ..alia()
            },
            Err(MappingError::DuplicateEngine { engine: 2 }),
        ),
        (
            ActuatorMapping {
                idle_engines: &[3],
                ..alia()
            },
            Err(MappingError::DuplicateEngine { engine: 3 }),
        ),
        (
            ActuatorMapping {
                idle_engines: &[20],
                ..alia()
            },
            Err(MappingError::EngineOutOfRange { engine: 20 }),
        ),
        (
            ActuatorMapping {
                neutral_wings: &[2, 48],
                ..alia()
            },
            Err(MappingError::WingOutOfRange { wing: 48 }),
        ),
        (
            ActuatorMapping {
                surfaces: const { &[SurfaceLink::new(0, Surface::Aileron, 4, 10.0, -20.0)] },
                ..alia()
            },
            Ok(()),
        ),
        (
            ActuatorMapping {
                surfaces: const { &[SurfaceLink::new(8, Surface::Aileron, 4, 10.0, -20.0)] },
                ..alia()
            },
            Err(MappingError::ServoOutOfRange { servo: 8 }),
        ),
        (
            ActuatorMapping {
                surfaces: const { &[SurfaceLink::new(0, Surface::Aileron, 48, 10.0, -20.0)] },
                ..alia()
            },
            Err(MappingError::WingOutOfRange { wing: 48 }),
        ),
        (
            ActuatorMapping {
                surfaces: const { &[SurfaceLink::new(0, Surface::Aileron, 4, f32::NAN, -20.0)] },
                ..alia()
            },
            Err(MappingError::InvalidDeflection {
                wing: 4,
                surface: Surface::Aileron,
            }),
        ),
        (
            ActuatorMapping {
                surfaces: const {
                    &[SurfaceLink::new(
                        0,
                        Surface::Aileron,
                        4,
                        10.0,
                        f32::INFINITY,
                    )]
                },
                ..alia()
            },
            Err(MappingError::InvalidDeflection {
                wing: 4,
                surface: Surface::Aileron,
            }),
        ),
        (
            ActuatorMapping {
                surfaces: const {
                    &[
                        SurfaceLink::new(0, Surface::Aileron, 4, 10.0, -20.0),
                        SurfaceLink::new(2, Surface::Rudder, 4, 10.0, -10.0),
                        SurfaceLink::new(1, Surface::Aileron, 4, 10.0, -20.0),
                    ]
                },
                ..alia()
            },
            Err(MappingError::DuplicateSurface {
                wing: 4,
                surface: Surface::Aileron,
            }),
        ),
        (
            ActuatorMapping {
                surfaces: const { &[SurfaceLink::new(1, Surface::Elevator, 8, -20.0, 20.0)] },
                ..alia()
            },
            Err(MappingError::DuplicateSurface {
                wing: 8,
                surface: Surface::Elevator,
            }),
        ),
    ];
    for (mapping, expected) in rows {
        check!(mapping.validate() == expected, "{mapping:?}");
    }
}

/// Mapped motors drive their engines, saturated, NaN as off; idle engines get 0.
#[test]
fn writes_throttles_for_mapped_and_idle_engines() {
    let command = ActuatorCommand {
        motors: [0.25, 0.5, f32::NAN, 1.5, 0.9, 0.9, 0.9, 0.9],
        ..Default::default()
    };
    let throttles: Vec<_> = alia().throttles(&command).collect();
    check!(throttles == [(1, 0.25), (0, 0.5), (2, 0.0), (3, 1.0), (4, 0.0)]);
    check!(alia().engines().collect::<Vec<_>>() == [1, 0, 2, 3, 4]);
}

/// A servo deflects its surface linearly on each side of 0, to each end's own travel and
/// sign; out-of-range commands saturate and NaN holds it at 0°.
#[test]
fn deflects_each_side_to_its_own_travel() {
    let aileron = SurfaceLink::new(0, Surface::Aileron, 2, 10.0, -20.0);
    let rudder = SurfaceLink::new(2, Surface::Rudder, 10, -30.0, 30.0);
    // (link, servo, degrees)
    let rows = [
        (aileron, 0.0, 0.0),
        (aileron, 1.0, 10.0),
        (aileron, 0.5, 5.0),
        (aileron, -1.0, -20.0),
        (aileron, -0.25, -5.0),
        (aileron, 2.0, 10.0),
        (aileron, -3.0, -20.0),
        (aileron, f32::NAN, 0.0),
        (rudder, 1.0, -30.0),
        (rudder, -0.5, 15.0),
    ];
    for (link, servo, expected) in rows {
        check!(
            link.deflection_deg(servo) == expected,
            "{link:?} at {servo}"
        );
    }
}

/// The surfaces are taken from X-Plane when any are driven or held neutral.
#[test]
fn overrides_surfaces_when_driving_or_holding_any() {
    let none = ActuatorMapping {
        neutral_wings: &[],
        ..alia()
    };
    let driven = ActuatorMapping {
        surfaces: const { &[SurfaceLink::new(1, Surface::Elevator, 8, -20.0, 20.0)] },
        ..none
    };
    check!(alia().overrides_surfaces());
    check!(driven.overrides_surfaces());
    check!(!none.overrides_surfaces());
}
