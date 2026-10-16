use assert2::check;
use stable_flight::{GRAVITY, Vec3};

use super::*;
use crate::aero;

/// The trim of level flight at `airspeed_mps`: angle of attack (rad, body pitch in level
/// flight), elevator deflection (rad), and the pusher thrust that holds the speed (N). Found by
/// bisection on the angle that balances lift and weight, with the elevator that zeroes the
/// pitch moment at each angle.
fn trim(model: &FixedWingModel, airspeed_mps: f32) -> (f32, f32, f32) {
    let weight_n = model.mass_kg * GRAVITY;
    let at = |alpha: f32| {
        let air = Vec3::new(airspeed_mps * alpha.cos(), 0.0, airspeed_mps * alpha.sin());
        let wrench = |elevator: f32| {
            aero::wrench(
                model,
                air,
                Vec3::zeros(),
                Vec3::new(0.0, elevator, 0.0),
                0.0,
            )
        };
        let (m0, m1) = (wrench(0.0).torque_nm.y, wrench(0.1).torque_nm.y);
        let elevator = -0.1 * m0 / (m1 - m0);
        (elevator, wrench(elevator).force_n)
    };
    // Body z is down: lift is negative `force.z`; level, gravity's body z is `W cos α`.
    let residual = |alpha: f32| at(alpha).1.z + weight_n * alpha.cos();
    let (mut low, mut high) = ((-5.0f32).to_radians(), 16.0f32.to_radians());
    for _ in 0..60 {
        let mid = 0.5 * (low + high);
        if residual(mid) > 0.0 {
            low = mid;
        } else {
            high = mid;
        }
    }
    let alpha = 0.5 * (low + high);
    let (elevator, force) = at(alpha);
    (alpha, elevator, weight_n * alpha.sin() - force.x)
}

/// The pusher command that makes `thrust_n` at `airspeed_mps`.
fn pusher_command(model: &FixedWingModel, airspeed_mps: f32, thrust_n: f32) -> f32 {
    let full_n = model.full_thrust_n(airspeed_mps);
    libm::powf(thrust_n / full_n, 1.0 / model.thrust_exponent)
}

/// Level flight on the model matches X-Plane's level flight (#392: the mean of each speed's
/// cycle-averaged windows from `plane-probe-xplane`): body angle of attack within 2°, elevator
/// command within 0.2, pusher command within 0.12, about the X-Plane windows' own spread at one
/// speed (46.2 m/s: α 4.3–6.9°, elevator 0.08–0.35, pusher 0.75–0.93).
#[test]
fn alia250_trims_as_xplane_flies() {
    let model = alia250().model.wing;
    // (airspeed m/s, X-Plane α deg, elevator command, pusher command)
    let points = [
        (42.0, 9.2, 0.41, 0.68),
        (43.9, 7.3, 0.27, 0.69),
        (45.0, 7.5, 0.35, 0.72),
        (46.2, 6.3, 0.29, 0.82),
        (48.1, 3.5, 0.02, 0.78),
    ];
    for (airspeed, alpha_deg, elevator_cmd, pusher) in points {
        let (alpha, elevator, thrust_n) = trim(&model, airspeed);
        let elevator_cmd_sim = elevator / model.max_deflection_rad.y;
        let pusher_sim = pusher_command(&model, airspeed, thrust_n);
        println!(
            "{airspeed} m/s: α {:.1}° (X-Plane {alpha_deg}), elevator {elevator_cmd_sim:.2} ({elevator_cmd}), pusher {pusher_sim:.2} ({pusher}), thrust {thrust_n:.0} N",
            alpha.to_degrees()
        );
        check!((alpha.to_degrees() - alpha_deg).abs() < 2.0, "{airspeed}");
        check!((elevator_cmd_sim - elevator_cmd).abs() < 0.2, "{airspeed}");
        check!((pusher_sim - pusher).abs() < 0.12, "{airspeed}");
    }
}

/// At full power the model climbs as X-Plane's Alia did: 2.4 m/s at 40.8 m/s (the full-power
/// lap, 409–594 m), within 0.6 m/s. Steady climb: thrust beyond drag, times speed, over weight.
#[test]
fn alia250_climbs_at_full_power_as_xplane() {
    let model = alia250().model.wing;
    let airspeed = 40.8;
    let (_, _, level_thrust_n) = trim(&model, airspeed);
    let climb_mps =
        (model.full_thrust_n(airspeed) - level_thrust_n) * airspeed / (model.mass_kg * GRAVITY);
    println!("full-power climb at {airspeed} m/s: {climb_mps:.2} m/s");
    check!((climb_mps - 2.4).abs() < 0.6);
}

/// The elevator's reach: full up elevator trims the wing at 13–18°. X-Plane held 13–15°
/// (pitch less flight path) at full elevator while sinking with the pusher low, and its trim
/// line (elevator 0.02 at 3.5°, 0.53 at 10.2°) reaches full at 16–17° (#392 probes).
#[test]
fn alia250_full_elevator_reaches_xplanes_angle() {
    let model = alia250().model.wing;
    let c = &model.aero;
    let full = model.max_deflection_rad.y;
    // Zero pitch moment with full elevator and no rate: `Cm0 + Cmα α + Cmδe δ = 0`.
    let alpha_deg = ((c.c_pitch_0 + c.c_pitch_elevator * full) / -c.c_pitch_alpha).to_degrees();
    println!("full elevator trims at α {alpha_deg:.1}°");
    check!((13.0..=18.0).contains(&alpha_deg));
}
