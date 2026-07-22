//! Reference: https://github.com/ArduPilot/SITL_Models/tree/master/RealFlight/Released_Models/Multicopters/QuadCopterX
//!
//! Flies a quadcopter in RealFlight from a Logitech Extreme 3D joystick, using
//! hand-tuned stabilizer gains. Shared RealFlight wiring lives in `examples/common`.
//!
//! ```text
//! cargo run --example copter-manual-realflight
//! ```

mod common;

use std::f32::consts::PI;

use robomath::{vec2, vec3};
use stable_flight_sitl::commanders::extreme_3d::Extreme3DCommander;
use stable_flight_sitl::stabilizers::Gains;
use stable_flight_sitl::stabilizers::controllers::*;
use stable_flight_sitl::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};

fn main() {
    // Joystick commander drives the vehicle attitude/rate directly.
    let commander = Extreme3DCommander::default();

    let rates_controller = BodyRateController::with_config(&BodyRateControllerConfig {
        gains: Gains {
            p: 7.0,
            i: 0.00,
            d: 0.01,
        },
        max_integral: 100.0,
        max_angular_rates: vec3(3.0, 3.0, 3.0), // Max roll, pitch, yaw rates in rad/s
    });

    let attitude_controller = AttitudeController::with_config(&AttitudeControllerConfig {
        gains: Gains {
            p: 3.0,
            i: 0.0,
            d: 0.1,
        },
        max_integral: 100.0,
        max_angles: vec2(PI / 4.0, PI / 4.0), // 45° for roll and pitch
        max_rate: vec2(3.0, 3.0),             // Max roll, pitch, yaw rates in rad/s
    });

    let stabilizer = CopterStabilizer::new(CopterStabilizerConfig {
        rates_controller,
        attitude_controller,
        ..Default::default()
    });

    common::run_realflight(commander, stabilizer);
}
