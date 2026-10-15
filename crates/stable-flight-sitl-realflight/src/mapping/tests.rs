use super::*;
use assert2::check;

fn mapping(motors: &[usize], servos: &[usize]) -> ActuatorMapping {
    let mut mapping = ActuatorMapping::default();
    for (slot, &channel) in mapping.motors.iter_mut().zip(motors) {
        *slot = Some(channel);
    }
    for (slot, &channel) in mapping.servos.iter_mut().zip(servos) {
        *slot = Some(channel);
    }
    mapping
}

#[test]
fn channel_count_matches_realflight() {
    check!(ControlInputs::default().channels.len() == CHANNEL_COUNT);
}

#[test]
fn validate_rejects_missing_and_shared_channels() {
    let cases = [
        (mapping(&[], &[]), Ok(())),
        (mapping(&[2, 0, 1, 3], &[4, 11]), Ok(())),
        (
            mapping(&[12], &[]),
            Err(MappingError::ChannelOutOfRange { channel: 12 }),
        ),
        (
            mapping(&[], &[0, 40]),
            Err(MappingError::ChannelOutOfRange { channel: 40 }),
        ),
        (
            mapping(&[1, 1], &[]),
            Err(MappingError::DuplicateChannel { channel: 1 }),
        ),
        (
            mapping(&[0, 1], &[1]),
            Err(MappingError::DuplicateChannel { channel: 1 }),
        ),
    ];
    for (mapping, expected) in cases {
        check!(mapping.validate() == expected, "{mapping:?}");
    }
}

#[test]
fn motors_pass_through_and_servos_center_on_half() {
    let mapping = mapping(&[0, 1], &[2, 3, 4]);
    let command = ActuatorCommand {
        motors: [0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        servos: [-1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };

    let channels = mapping.channels(&command).channels;

    check!(channels[..5] == [0.5, 1.0, 0.0, 0.5, 1.0]);
    check!(channels[5..].iter().all(|&c| c == 0.0)); // unmapped
}

#[test]
fn reversed_servos_mirror_about_center() {
    let mut mapping = mapping(&[0], &[1, 2]);
    mapping.reversed_servos[1] = true;
    let command = ActuatorCommand {
        motors: [0.6; 8],
        servos: [0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };

    let channels = mapping.channels(&command).channels;

    check!(channels[..3] == [0.6, 0.75, 0.25]); // motors are never reversed
}

/// A trim offsets a servo's neutral in channel terms: it isn't mirrored by a reversal, and the
/// sum clamps to the servo range.
#[test]
fn servo_trims_offset_neutral_after_reversal() {
    let mut mapping = mapping(&[], &[0, 1, 2, 3]);
    mapping.servo_trims[..4].copy_from_slice(&[0.2, 0.2, -0.2, 0.5]);
    mapping.reversed_servos[1] = true;
    // (case, servo commands, expected channels 0–3)
    let cases = [
        ("neutral", [0.0; 4], [0.6, 0.6, 0.4, 0.75]),
        ("half up", [0.5; 4], [0.85, 0.35, 0.65, 1.0]),
        ("clamped", [1.0, -1.0, -1.0, 1.0], [1.0, 1.0, 0.0, 1.0]),
    ];
    for (case, servos, expected) in cases {
        let mut command = ActuatorCommand::default();
        command.servos[..4].copy_from_slice(&servos);
        let channels = mapping.channels(&command).channels;
        for (channel, want) in channels[..4].iter().zip(expected) {
            check!((channel - want).abs() < 1e-6, "{case}: {channels:?}");
        }
    }
}

#[test]
fn out_of_range_channel_is_skipped() {
    let mapping = mapping(&[0, 99], &[99]);
    let command = ActuatorCommand {
        motors: [0.7; 8],
        servos: [1.0; 8],
    };

    let channels = mapping.channels(&command).channels;

    check!(channels[0] == 0.7);
    check!(channels[1..].iter().all(|&c| c == 0.0));
}
