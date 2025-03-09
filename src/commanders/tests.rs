
use super::*;
use std::time::Duration;

// A mock Commander implementation for testing
struct MockCommander {
    toggle: bool, // Used to alternate between Manual and Waypoint commands
}

impl MockCommander {
    fn new() -> Self {
        MockCommander { toggle: false }
    }
}

impl Commander for MockCommander {
    fn next(&self, dt: Duration) -> Command {
        // Alternate between Manual and Waypoint commands based on toggle
        let secs = dt.as_secs();
        if secs % 2 == 0 {
            // Manual command
            Command::Manual(ManualCommand {
                throttle: 0.5,
                roll: 0.2,
                pitch: -0.1,
                yaw: 0.0,
                arm: true,
                mode: 1,
                aux0: 0.0,
                aux1: 0.0,
                aux2: 0.0,
                aux3: 0.0,
                ..Default::default()
            })
        } else {
            // Waypoint command
            Command::Waypoint(WaypointCommand {
                position: Vec2 {
                    x: 1000.0,
                    y: 2000.0,
                },
                velocity: Some(Vec2 { x: 15.0, y: 0.0 }),
                altitude: 500.0,
                cruise_speed: 25.0,
                roll: Some(0.05),
            })
        }
    }
}

// Helper function to check if a ManualCommand is within valid ranges
fn assert_manual_command_valid(cmd: &ManualCommand) {
    assert!(
        cmd.throttle >= 0.0 && cmd.throttle <= 1.0,
        "Throttle out of range: {}",
        cmd.throttle
    );
    assert!(
        cmd.roll >= -1.0 && cmd.roll <= 1.0,
        "Roll out of range: {}",
        cmd.roll
    );
    assert!(
        cmd.pitch >= -1.0 && cmd.pitch <= 1.0,
        "Pitch out of range: {}",
        cmd.pitch
    );
    assert!(
        cmd.yaw >= -1.0 && cmd.yaw <= 1.0,
        "Yaw out of range: {}",
        cmd.yaw
    );
    assert!(cmd.mode <= 2, "Mode out of range: {}", cmd.mode);
    assert!(
        cmd.aux0 >= -1.0 && cmd.aux0 <= 1.0,
        "Aux0 out of range: {}",
        cmd.aux0
    );
    assert!(
        cmd.aux1 >= -1.0 && cmd.aux1 <= 1.0,
        "Aux1 out of range: {}",
        cmd.aux1
    );
    assert!(
        cmd.aux2 >= -1.0 && cmd.aux2 <= 1.0,
        "Aux2 out of range: {}",
        cmd.aux2
    );
    assert!(
        cmd.aux3 >= -1.0 && cmd.aux3 <= 1.0,
        "Aux3 out of range: {}",
        cmd.aux3
    );
}

// Helper function to check if a WaypointCommand is within reasonable bounds
fn assert_waypoint_command_valid(cmd: &WaypointCommand) {
    assert!(
        cmd.altitude >= 0.0,
        "Altitude should be non-negative: {}",
        cmd.altitude
    );
    assert!(
        cmd.cruise_speed >= 0.0,
        "Cruise speed should be non-negative: {}",
        cmd.cruise_speed
    );
    if let Some(roll) = cmd.roll {
        assert!(
            roll.abs() <= std::f32::consts::PI,
            "Roll angle too large: {}",
            roll
        );
    }
}

#[test]
fn test_manual_command_generation() {
    let commander = MockCommander::new();
    let dt = Duration::from_secs(0); // Should produce a ManualCommand
    let command = commander.next(dt);

    match command {
        Command::Manual(manual_cmd) => {
            assert_eq!(manual_cmd.throttle, 0.5);
            assert_eq!(manual_cmd.roll, 0.2);
            assert_eq!(manual_cmd.pitch, -0.1);
            assert_eq!(manual_cmd.yaw, 0.0);
            assert_eq!(manual_cmd.arm, true);
            assert_eq!(manual_cmd.mode, 1);
            assert_eq!(manual_cmd.aux0, 0.0);
            assert_eq!(manual_cmd.aux1, 0.0);
            assert_eq!(manual_cmd.aux2, 0.0);
            assert_eq!(manual_cmd.aux3, 0.0);
            assert_manual_command_valid(&manual_cmd);
        }
        _ => panic!("Expected a ManualCommand"),
    }
}

#[test]
fn test_waypoint_command_generation() {
    let commander = MockCommander::new();
    let dt = Duration::from_secs(1); // Should produce a WaypointCommand
    let command = commander.next(dt);

    match command {
        Command::Waypoint(waypoint_cmd) => {
            assert_eq!(waypoint_cmd.position.x, 1000.0);
            assert_eq!(waypoint_cmd.position.y, 2000.0);
            assert_eq!(waypoint_cmd.velocity.unwrap().x, 15.0);
            assert_eq!(waypoint_cmd.velocity.unwrap().y, 0.0);
            assert_eq!(waypoint_cmd.altitude, 500.0);
            assert_eq!(waypoint_cmd.cruise_speed, 25.0);
            assert_eq!(waypoint_cmd.roll.unwrap(), 0.05);
            assert_waypoint_command_valid(&waypoint_cmd);
        }
        _ => panic!("Expected a WaypointCommand"),
    }
}

#[test]
fn test_manual_command_boundary_values() {
    let manual_cmd = ManualCommand {
        throttle: 1.0, // Max throttle
        roll: -1.0,    // Max left roll
        pitch: 1.0,    // Max nose up
        yaw: -1.0,     // Max left yaw
        arm: true,
        mode: 2,    // Max mode
        aux0: 1.0,  // Max aux0
        aux1: -1.0, // Min aux1
        aux2: 0.0,  // Middle aux2
        aux3: 0.0,  // Middle aux3
        ..Default::default()
    };

    assert_manual_command_valid(&manual_cmd);
}

#[test]
fn test_waypoint_command_optional_fields() {
    let waypoint_cmd = WaypointCommand {
        position: Vec2 { x: 0.0, y: 0.0 },
        velocity: None, // No velocity specified
        altitude: 100.0,
        cruise_speed: 10.0,
        roll: None, // No roll specified
    };

    assert_waypoint_command_valid(&waypoint_cmd);
}

#[test]
fn test_command_alternation() {
    let commander = MockCommander::new();

    // Test multiple calls with different dt to ensure alternation
    let dt1 = Duration::from_secs(0); // Should produce ManualCommand
    let dt2 = Duration::from_secs(1); // Should produce WaypointCommand
    let dt3 = Duration::from_secs(2); // Should produce ManualCommand again

    let cmd1 = commander.next(dt1);
    let cmd2 = commander.next(dt2);
    let cmd3 = commander.next(dt3);

    assert!(matches!(cmd1, Command::Manual(_)));
    assert!(matches!(cmd2, Command::Waypoint(_)));
    assert!(matches!(cmd3, Command::Manual(_)));
}

#[test]
fn test_manual_command_default() {
    let default_cmd = ManualCommand::default();
    assert_eq!(default_cmd.throttle, 0.0);
    assert_eq!(default_cmd.roll, 0.0);
    assert_eq!(default_cmd.pitch, 0.0);
    assert_eq!(default_cmd.yaw, 0.0);
    assert_eq!(default_cmd.arm, false);
    assert_eq!(default_cmd.mode, 0);
    assert_eq!(default_cmd.aux0, 0.0);
    assert_eq!(default_cmd.aux1, 0.0);
    assert_eq!(default_cmd.aux2, 0.0);
    assert_eq!(default_cmd.aux3, 0.0);
    assert_manual_command_valid(&default_cmd);
}

#[test]
fn test_waypoint_command_default() {
    let default_cmd = WaypointCommand::default();
    assert_eq!(default_cmd.position.x, 0.0);
    assert_eq!(default_cmd.position.y, 0.0);
    assert_eq!(default_cmd.velocity, None);
    assert_eq!(default_cmd.altitude, 0.0);
    assert_eq!(default_cmd.cruise_speed, 0.0);
    assert_eq!(default_cmd.roll, None);
    assert_waypoint_command_valid(&default_cmd);
}
