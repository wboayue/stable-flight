
use super::*;
use realflight_bridge::{ControlInputs, RealFlightBridge, SimulatorState};
use robomath::{Quaternion, Vec3};
use std::{cell::RefCell, error::Error, time::Duration};
use uom::si::{
    acceleration::meter_per_second_squared,
    angular_velocity,
    f32::{Acceleration, AngularAcceleration, AngularVelocity, Length, Velocity},
    length::meter,
    velocity::meter_per_second,
};

// Mock implementation of RealFlightBridge for testing
struct MockRealFlightBridge {
    should_fail: bool,
    last_input: RefCell<Option<ControlInputs>>,
}

impl MockRealFlightBridge {
    fn new(should_fail: bool) -> Self {
        Self {
            should_fail,
            last_input: RefCell::new(None),
        }
    }
}

impl RealFlightBridge for MockRealFlightBridge {
    fn exchange_data(&self, input: &ControlInputs) -> Result<SimulatorState, Box<dyn Error>> {
        let mut last_input = self.last_input.borrow_mut();
        last_input.replace(input.clone());

        if self.should_fail {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                "Simulated bridge failure",
            )))
        } else {
            Ok(SimulatorState {
                aircraft_position_x: Length::new::<meter>(1.0),
                aircraft_position_y: Length::new::<meter>(2.0),
                altitude_asl: Length::new::<meter>(3.0),
                velocity_world_u: Velocity::new::<meter_per_second>(4.0),
                velocity_world_v: Velocity::new::<meter_per_second>(5.0),
                velocity_world_w: Velocity::new::<meter_per_second>(6.0),
                acceleration_world_ax: Acceleration::new::<meter_per_second_squared>(7.0),
                acceleration_world_ay: Acceleration::new::<meter_per_second_squared>(8.0),
                acceleration_world_az: Acceleration::new::<meter_per_second_squared>(9.0),
                orientation_quaternion_w: 1.0,
                orientation_quaternion_x: 0.0,
                orientation_quaternion_y: 0.0,
                orientation_quaternion_z: 0.0,
                roll_rate: AngularVelocity::new::<angular_velocity::degree_per_second>(90.0),
                pitch_rate: AngularVelocity::new::<angular_velocity::degree_per_second>(45.0),
                yaw_rate: AngularVelocity::new::<angular_velocity::degree_per_second>(30.0),
                ..Default::default()
            })
        }
    }

    fn disable_rc(&self) -> Result<(), Box<dyn Error>> {
        if self.should_fail {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                "RC disable failed",
            )))
        } else {
            Ok(())
        }
    }

    fn enable_rc(&self) -> Result<(), Box<dyn Error>> {
        if self.should_fail {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                "RC enable failed",
            )))
        } else {
            Ok(())
        }
    }

    fn reset_aircraft(&self) -> Result<(), Box<dyn Error>> {
        if self.should_fail {
            Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::Other,
                "Reset failed",
            )))
        } else {
            Ok(())
        }
    }
}

// Helper function to create a test configuration
fn create_test_config(should_fail: bool) -> RealFlightConfiguration<MockRealFlightBridge> {
    let mut mapping = ActuatorMapping::default();
    mapping.motors[0] = Some(0);
    mapping.rudder = Some(1);
    mapping.elevator = Some(2);
    mapping.aileron = Some(3);
    mapping.throttle = Some(4);
    mapping.elevon_left = Some(5);
    mapping.elevon_right = Some(6);

    RealFlightConfiguration {
        mapping,
        bridge: MockRealFlightBridge::new(should_fail),
    }
}

#[test]
fn test_vehicle_creation() {
    let config = create_test_config(false);
    let vehicle = RealFlightVehicle::new(config);

    // Verify initial state
    assert_eq!(vehicle.estimated_state.position, Vec3::default());
    assert_eq!(vehicle.mapping.motors[0], Some(0));
    assert_eq!(vehicle.mapping.rudder, Some(1));
}

#[test]
fn test_vehicle_creation_with_bridge_failure() {
    let config = create_test_config(true);
    let vehicle = RealFlightVehicle::new(config);

    // Should still create successfully despite bridge failures
    assert_eq!(vehicle.estimated_state.position, Vec3::default());
}

#[test]
fn test_channel_mapping() {
    let config = create_test_config(false);
    let vehicle = RealFlightVehicle::new(config);

    let command = ActuatorCommand {
        motors: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        rudder: -0.3,
        elevator: 0.7,
        aileron: -0.1,
        throttle: 1.0,
        elevon_left: 0.4,
        elevon_right: -0.4,
    };

    let inputs = vehicle.map_channels(&command);

    assert_eq!(inputs.channels[0], 0.5); // motor 0
    assert_eq!(inputs.channels[1], -0.3); // rudder
    assert_eq!(inputs.channels[2], 0.7); // elevator
    assert_eq!(inputs.channels[3], -0.1); // aileron
    assert_eq!(inputs.channels[4], 1.0); // throttle
    assert_eq!(inputs.channels[5], 0.4); // elevon_left
    assert_eq!(inputs.channels[6], -0.4); // elevon_right
}

#[test]
fn test_actuate_success() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // Verify state was updated
    assert_eq!(
        vehicle.estimated_state.position.x,
        Length::new::<meter>(1.0)
    );
    assert_eq!(
        vehicle.estimated_state.velocity.y,
        Velocity::new::<meter_per_second>(5.0)
    );
    assert_eq!(
        vehicle.estimated_state.acceleration.z,
        Acceleration::new::<meter_per_second_squared>(9.0)
    );
    assert_eq!(vehicle.estimated_state.attitude.w, 1.0);
    // FIXME
    // assert_eq!(vehicle.estimated_state.angular_velocity.x, to_radians(90.0));
}

#[test]
fn test_actuate_failure() {
    let config = create_test_config(true);
    let mut vehicle = RealFlightVehicle::new(config);
    let initial_state = vehicle.estimated_state.clone();
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // State should remain unchanged on failure
    assert_eq!(vehicle.estimated_state, initial_state);
}

#[test]
fn test_estimate_state() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);

    // Update state through actuation
    vehicle.actuate(&ActuatorCommand::default());
    let state = vehicle.estimate_state(Duration::from_millis(16));

    assert_eq!(state.position.x, Length::new::<meter>(1.0));
    assert_eq!(state.velocity.y, Velocity::new::<meter_per_second>(5.0));
    assert_eq!(
        state.acceleration.z,
        Acceleration::new::<meter_per_second_squared>(9.0)
    );
}

#[test]
fn test_drop_behavior() {
    let config = create_test_config(false);
    {
        let _vehicle = RealFlightVehicle::new(config);
        // Vehicle dropped at end of scope
    }
    // Can't test enable_rc directly, but we can ensure it doesn't panic
}

#[test]
fn test_partial_mapping() {
    let mut mapping = ActuatorMapping::default();
    mapping.motors[0] = Some(0); // Only map motor 0
    let config = RealFlightConfiguration {
        mapping,
        bridge: MockRealFlightBridge::new(false),
    };
    let vehicle = RealFlightVehicle::new(config);

    let command = ActuatorCommand {
        motors: [0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ..Default::default()
    };

    let inputs = vehicle.map_channels(&command);
    assert_eq!(inputs.channels[0], 0.5); // Mapped
    assert_eq!(inputs.channels[1], 0.0); // Unmapped
}
