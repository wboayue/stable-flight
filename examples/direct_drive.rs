use std::error::Error;
use std::time::{Duration, Instant};

use realflight_bridge::RealFlightRemoteBridge;
use stable_flight_sitl::mixers::QuadCopterMixer;
use stable_flight_sitl::simulators::realflight::{ActuatorMapping, RealFlightConfiguration};
use stable_flight_sitl::simulators::RealFlightVehicle;
use stable_flight_sitl::{ControlOutput, Mixer as _, Vehicle as _};

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();

    let mut vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        bridge: RealFlightRemoteBridge::new("10.211.55.3:8080").expect("Failed to create bridge"),
        mapping: ActuatorMapping {
            //            motors: [Some(0), Some(1), Some(2), Some(3), None, None, None, None],
            //            motors: [Some(3), Some(1), Some(2), Some(0), None, None, None, None],
            motors: [Some(2), Some(0), Some(1), Some(3), None, None, None, None],
            ..Default::default()
        },
    });

    let mixer = QuadCopterMixer::default();

    let mut command = ControlOutput::default();

    let start = Instant::now();
    let loop_duration = Duration::from_secs(20);

    println!("Starting simulation loop for {:?}", loop_duration);

    let mut i = 0;

    while start.elapsed() < loop_duration {
        let c = mixer.mix(&command);
        vehicle.actuate(&c);
        // Simulate a short delay to mimic real-time control loop
        std::thread::sleep(Duration::from_millis(2));

        command.thrust = 1.0;
        command.moments.x = 0.0;
        command.moments.y = 0.0;
        command.moments.z = -0.8;

        i += 1;
    }

    println!("Simulation complete: {} cycles in {:?}", i, start.elapsed());
    println!(
        "Achieved rate: {:.1} Hz",
        i as f32 / start.elapsed().as_secs_f32()
    );

    Ok(())
}
