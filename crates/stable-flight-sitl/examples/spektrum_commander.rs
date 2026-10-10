//! Checks a Spektrum transmitter through the WS3000T dongle: prints the raw dongle axes and
//! buttons next to the [`SpektrumCommander`]'s command at 2 Hz for 50 seconds.
//!
//! Use it to confirm the mapping before flying: centered sticks give roll/pitch/yaw near 0,
//! right aileron, back elevator, and right rudder give `+1`, and the raw values show each stick's
//! real travel (the commander assumes about ±0.7 aileron, ±0.65 elevator/rudder).
//!
//! **Prerequisites**: the WS3000T dongle plugged in and bound to a powered-on transmitter.
//! Move a stick once so the dongle is picked up.

use std::thread;
use std::time::{Duration, Instant};

use gilrs::Gilrs;
use stable_flight::{Commander, VehicleState};
use stable_flight_sitl::commanders::SpektrumCommander;

pub fn main() {
    let mut commander = SpektrumCommander::new().expect("Failed to initialize Spektrum dongle");
    // A second gilrs handle for the raw values; both see the same device.
    let mut raw = Gilrs::new().expect("Failed to initialize gilrs");

    let mut next_time = Instant::now();
    const UPDATE_RATE_HZ: f32 = 2.0;
    const ITERATION_COUNT: usize = 100;
    let dt = Duration::from_secs_f32(1.0 / UPDATE_RATE_HZ);

    // Using default state since this example focuses on commander output
    let vehicle_state = VehicleState::default();

    for _ in 0..ITERATION_COUNT {
        next_time += dt;

        while raw.next_event().is_some() {}
        for (_, gamepad) in raw.gamepads() {
            let mut line = format!("{}:", gamepad.name());
            for (code, data) in gamepad.state().axes() {
                let name = gamepad.axis_or_btn_name(code);
                line += &format!(" {name:?}({code})={:.3}", data.value());
            }
            for (code, data) in gamepad.state().buttons() {
                let name = gamepad.axis_or_btn_name(code);
                line += &format!(" {name:?}({code})={:.3}", data.value());
            }
            println!("{line}");
        }

        let command = commander.next_command(dt.as_secs_f32(), &vehicle_state);
        println!("{command:?}");

        // Sleep to maintain precise timing
        let sleep_time = next_time
            .checked_duration_since(Instant::now())
            .unwrap_or(Duration::ZERO);

        thread::sleep(sleep_time);
    }
}
