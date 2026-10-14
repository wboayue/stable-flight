//! Stall entry and recovery on `airframes::plane()`: a held pitch-up rate at zero throttle
//! zooms the airplane until it stalls, then guidance flies it back to the line. No external
//! band applies (neither ArduPilot's nor PX4's autotests time a stall recovery), so each
//! band is the measured value with its margin.

mod common;

use assert2::check;
use stable_flight::{AbsoluteRateCommand, ControlInput};
use stable_flight_sim::airframes;

use common::fly;
use common::plane::{Interlude, PlanePlant, plane_command, plane_loop};

/// Seconds on the line before the entry: the prop spins up from rest and the loops settle.
const LEAD_S: f32 = 20.0;

/// Pitch-up rate held through the entry, in rad/s.
const ENTRY_PITCH_RATE_RADPS: f32 = 1.0;

/// Seconds the entry is held: the airplane stalls about 1.8 s in, so it spends the rest stalled.
const ENTRY_S: f32 = 4.0;

/// Seconds flown after the entry.
const RECOVERY_WINDOW_S: f32 = 20.0;

/// Seconds after the entry until the airplane flies again for good (angle of attack under the
/// stall, airspeed at or over the stabilizer's minimum). Measured 0.6 s at most (the tail
/// slide recovers during the entry); 2 s leaves over 3× margin.
const RECOVERY_S: f32 = 2.0;

/// Altitude lost from the entry to the lowest point, in m. Measured 3.7 m at most; 8 m leaves
/// about 2× margin.
const ALTITUDE_LOSS_M: f32 = 8.0;

/// Largest body rate, in rad/s. Measured 1.8 rad/s at most; 3.5 rad/s leaves about 2× margin.
const MAX_BODY_RATE_RADPS: f32 = 3.5;

#[test]
fn stalls_and_recovers_from_a_zoom() {
    // (case, roll rate through the entry rad/s). A pull-up in a held bank only tightens the
    // turn: at line speed the elevator can't hold the angle of attack past about 6° against
    // pitch damping, so a stall needs the airspeed a wings-level zoom bleeds.
    let cases = [("wings level", 0.0), ("rolling", 0.3)];
    for (case, roll_radps) in cases {
        let plant = PlanePlant::clean(airframes::plane());
        let stall_rad = plant.airframe.model.aero.alpha_stall_rad;
        let min_airspeed_mps = plant.airframe.stabilizer.min_airspeed_mps;
        let entry = plane_command(ControlInput::AbsoluteRate(AbsoluteRateCommand {
            roll_radps,
            pitch_radps: ENTRY_PITCH_RATE_RADPS,
            yaw_radps: 0.0,
            thrust: 0.0,
        }));
        let commander = Interlude::new(plant.line(), LEAD_S, LEAD_S + ENTRY_S, entry);
        let mut flight_loop = plane_loop(&plant, commander);
        fly(&mut flight_loop, LEAD_S, |_, _| {});
        let entry_altitude_m = flight_loop.state().position_m.z;

        let (mut alpha_rad, mut body_rate_radps, mut lowest_m) = (0.0_f32, 0.0_f32, f32::MAX);
        let (mut stalled_until_s, mut finite, mut grounded) = (0.0_f32, true, false);
        fly(
            &mut flight_loop,
            ENTRY_S + RECOVERY_WINDOW_S,
            |flight_loop, t| {
                let state = flight_loop.state();
                let vehicle = flight_loop.vehicle();
                let alpha = vehicle.angle_of_attack_rad();
                let airspeed = state.airspeed_mps.unwrap_or(f32::NAN);
                finite &= state.position_m.iter().all(|x| x.is_finite())
                    && state.velocity_mps.iter().all(|x| x.is_finite())
                    && state.angular_velocity_radps.iter().all(|x| x.is_finite())
                    && alpha.is_finite();
                grounded |= vehicle.on_ground();
                alpha_rad = alpha_rad.max(alpha);
                body_rate_radps = body_rate_radps.max(state.angular_velocity_radps.norm());
                lowest_m = lowest_m.min(state.position_m.z);
                // NaN airspeed counts as not flying.
                if alpha >= stall_rad || airspeed.is_nan() || airspeed < min_airspeed_mps {
                    stalled_until_s = t - ENTRY_S;
                }
            },
        );

        let altitude_loss_m = entry_altitude_m - lowest_m;
        check!(finite, "{case}: non-finite state");
        check!(!grounded, "{case}: hit the ground");
        check!(
            alpha_rad > stall_rad,
            "{case}: never stalled (angle of attack peaked at {alpha_rad} rad)"
        );
        check!(
            body_rate_radps <= MAX_BODY_RATE_RADPS,
            "{case}: body rate reached {body_rate_radps} rad/s"
        );
        check!(
            stalled_until_s <= RECOVERY_S,
            "{case}: stalled until {stalled_until_s} s after the entry"
        );
        check!(
            altitude_loss_m <= ALTITUDE_LOSS_M,
            "{case}: lost {altitude_loss_m} m"
        );
    }
}
