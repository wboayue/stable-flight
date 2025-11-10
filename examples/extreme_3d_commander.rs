//! Simple example of using the commander for the LogiTech Extreme 3D joystick.

use std::thread;
use std::time::{Duration, Instant};

use stable_flight_sitl::commanders::extreme_3d::{Extreme3DCommander, Mode};
use stable_flight_sitl::Commander;
use stable_flight_sitl::VehicleState;

pub fn main() {
    let mut commander =
        Extreme3DCommander::new(Mode::Rate).expect("Failed to initialize Extreme3D joystick");

    let mut next_time = Instant::now();
    let dt = Duration::from_secs_f32(1.0 / 2.0);

    let vehicle_state = VehicleState::default();

    for _ in 0..100 {
        next_time += dt;

        let command = commander.next(dt.as_secs_f32(), &vehicle_state);
        println!("{:?}", command);

        // Calculate how much time we need to sleep to hit the target
        let sleep_time = next_time
            .checked_duration_since(Instant::now())
            .unwrap_or(Duration::ZERO);

        thread::sleep(sleep_time);
    }
}
