use super::*;

#[test]
fn quadcopter_x_maps_each_motor_to_its_own_channel() {
    let airframe = quadcopter_x();

    let mut channels: Vec<usize> = airframe.mapping.motors.iter().flatten().copied().collect();
    channels.sort_unstable();

    assert_eq!(channels, [0, 1, 2, 3]);
}
