use assert2::check;

use super::*;

/// The Alia layout: four lift motors on engines 0–3 in `quad_x` order, the pusher idle.
fn alia() -> ActuatorMapping {
    ActuatorMapping {
        motors: [Some(1), Some(0), Some(2), Some(3), None, None, None, None],
        idle_engines: &[4],
        neutral_wings: &[2, 3, 8, 9],
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
