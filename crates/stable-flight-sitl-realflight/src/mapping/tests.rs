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
