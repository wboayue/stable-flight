//! Demonstrates basic usage of the Logitech Extreme 3D joystick commander.
//!
//! This example polls the joystick at 2Hz and prints the generated commands
//! for 100 iterations (50 seconds). It shows how to:
//! - Initialize the Extreme3DCommander
//! - Call `next()` with timing and vehicle state
//! - Maintain a fixed update rate using precise sleep timing
//!
//! **Prerequisites**: Logitech Extreme 3D joystick must be connected.

use std::thread;
use std::time::{Duration, Instant};

use stable_flight_sitl::Commander;
use stable_flight_sitl::VehicleState;
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;

pub fn main() {
    let mut commander = Extreme3DCommander::new().expect("Failed to initialize Extreme3D joystick");

    let mut next_time = Instant::now();
    const UPDATE_RATE_HZ: f32 = 2.0;
    const ITERATION_COUNT: usize = 100;
    let dt = Duration::from_secs_f32(1.0 / UPDATE_RATE_HZ);

    // Using default state since this example focuses on commander output
    let vehicle_state = VehicleState::default();

    for _ in 0..ITERATION_COUNT {
        next_time += dt;

        let command = commander.next(dt.as_secs_f32(), &vehicle_state);
        println!("{:?}", command);

        // Sleep to maintain precise timing
        let sleep_time = next_time
            .checked_duration_since(Instant::now())
            .unwrap_or(Duration::ZERO);

        thread::sleep(sleep_time);
    }
}
