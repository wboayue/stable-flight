//! Checks RealFlight captures against the `VehicleState` frame contract (#62, #120), offline.
//!
//! Reads a capture (`capture-state-realflight` or `channel-sweep-realflight`), converts each
//! exchange with the backend's `vehicle_state` (with the model's body rotation, `--pitch-deg`),
//! and reports, binned by |pitch|:
//!
//! - **Body rates:** `angular_velocity_radps` against the body rate implied by the quaternions
//!   around it, and against the Euler-angle rates. Body rates match the first; if the backend
//!   reported Euler rates, they'd match the second, which differs once pitched.
//! - **Gravity at rest:** on the ground and still, RealFlight's body specific force against the
//!   attitude's "up" (`attitude⁻¹ · (0, 0, −g)` NED, in FRD).
//! - **RealFlight's Euler angles** against the quaternion's, both in RealFlight's frame (before
//!   the body rotation) and binned by RealFlight's pitch: they agree away from ±90° pitch and
//!   fall apart near it.
//!
//! ```text
//! cargo run --bin check_frames -- realflight-sweep.csv
//! cargo run --bin check_frames -- --pitch-deg 90 explora.csv  # eXplora: hover-level → nose up
//! ```

use std::f32::consts::PI;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;

use clap::Parser;

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{GRAVITY, Vec3, VehicleState, euler_angles, wrap_angle};
use stable_flight_sitl_realflight::capture::{CapturedExchange, read_capture};
use stable_flight_sitl_realflight::vehicle_state;

/// Longest gap between samples used for a rate, in s: longer ones are stalls.
const MAX_GAP_S: f32 = 0.05;
/// Body rate below which a grounded sample counts as still, in rad/s.
const STILL_RADPS: f32 = 0.05;
/// |pitch| bin edges, in degrees.
const PITCH_BINS_DEG: [f32; 4] = [0.0, 60.0, 80.0, 90.01];

/// Check a RealFlight capture against the frame contract.
#[derive(Parser)]
struct Args {
    /// The capture CSV.
    capture: PathBuf,
    /// The model's body rotation as a pitch, in degrees: the vehicle's pitch when RealFlight
    /// reads level (90 for the eXplora VTOL, whose RealFlight frame is hover-level).
    #[arg(long, default_value_t = 0.0, allow_negative_numbers = true)]
    pitch_deg: f32,
}

/// One converted sample.
struct Sample {
    t_s: f32,
    state: VehicleState,
    /// RealFlight's Euler angles in the frame contract's convention (roll, pitch, heading).
    realflight_euler: Vec3,
    /// The quaternion's Euler angles in RealFlight's body frame (before the body rotation).
    realflight_quaternion_euler: Vec3,
    /// On the ground.
    grounded: bool,
    /// RealFlight's body specific force, in the vehicle's FRD body frame, m/s²: recovered from
    /// the converted state, so it goes through the backend's body rotation.
    specific_force: Vec3,
}

fn sample(exchange: &CapturedExchange, body_rotation: &UnitQuaternion<f32>) -> Sample {
    let s = &exchange.state;
    let state = vehicle_state(s, body_rotation);
    let unrotated = vehicle_state(s, &UnitQuaternion::identity());
    Sample {
        t_s: s.current_physics_time_s,

        realflight_euler: Vec3::new(
            s.roll_deg.to_radians(),
            s.inclination_deg.to_radians(),
            -s.azimuth_deg.to_radians(),
        ),
        realflight_quaternion_euler: Vec3::new(
            unrotated.roll_rad,
            unrotated.pitch_rad,
            unrotated.heading_rad,
        ),
        grounded: s.is_touching_ground,
        specific_force: specific_force(&state),
        state,
    }
}

/// The body specific force (FRD, m/s²) behind `state`'s acceleration: `attitude⁻¹ · (a − g)`,
/// with the acceleration up-positive and gravity NED.
fn specific_force(state: &VehicleState) -> Vec3 {
    let a = state.acceleration_mps2;
    state.attitude.inverse() * Vec3::new(a.x, a.y, -a.z - GRAVITY)
}

/// The body-frame angular velocity that turns `from` into `to` over `dt` seconds (FRD, rad/s):
/// `to = from · exp(ω·dt)`.
fn body_rate(from: &UnitQuaternion<f32>, to: &UnitQuaternion<f32>, dt: f32) -> Vec3 {
    (from.inverse() * to).scaled_axis() / dt
}

/// The rate of the ZYX Euler angles from `from` to `to` over `dt` (roll, pitch, heading), with
/// heading and roll wrapped across ±π.
fn euler_rate(from: &UnitQuaternion<f32>, to: &UnitQuaternion<f32>, dt: f32) -> Vec3 {
    let (r0, p0, h0) = euler_angles(from);
    let (r1, p1, h1) = euler_angles(to);
    Vec3::new(wrap_angle(r1 - r0), p1 - p0, wrap_angle(h1 - h0)) / dt
}

/// The specific force a still vehicle feels, in its body frame (FRD): straight up, `g`.
fn rest_specific_force(attitude: &UnitQuaternion<f32>) -> Vec3 {
    attitude.inverse() * Vec3::new(0.0, 0.0, -GRAVITY)
}

/// Running error statistics for one quantity.
#[derive(Default)]
struct Errors {
    count: usize,
    sum_squares: f32,
    max: f32,
}

impl Errors {
    fn add(&mut self, error: f32) {
        self.count += 1;
        self.sum_squares += error * error;
        self.max = self.max.max(error);
    }

    fn report(&self, unit: &str, scale: f32) -> String {
        if self.count == 0 {
            return "-".to_string();
        }
        let rms = (self.sum_squares / self.count as f32).sqrt();
        format!(
            "rms {:.3} max {:.3} {unit} (n {})",
            rms * scale,
            self.max * scale,
            self.count
        )
    }
}

/// Which |pitch| bin `pitch_rad` falls in.
fn bin(pitch_rad: f32) -> usize {
    let degrees = pitch_rad.abs().to_degrees();
    PITCH_BINS_DEG
        .windows(2)
        .position(|edges| degrees < edges[1])
        .unwrap_or(PITCH_BINS_DEG.len() - 2)
}

/// Errors per |pitch| bin for each check.
#[derive(Default)]
struct Report {
    body: [Errors; 3],
    euler: [Errors; 3],
    gravity: [Errors; 3],
    realflight_euler: [Errors; 3],
}

fn check(samples: &[Sample]) -> Report {
    let mut report = Report::default();
    for window in samples.windows(3) {
        let [before, now, after] = window else {
            continue;
        };
        let dt = after.t_s - before.t_s;
        if dt <= 0.0 || dt > 2.0 * MAX_GAP_S {
            continue;
        }
        let at = bin(now.state.pitch_rad);
        let reported = now.state.angular_velocity_radps;
        let body = body_rate(&before.state.attitude, &after.state.attitude, dt);
        let euler = euler_rate(&before.state.attitude, &after.state.attitude, dt);
        report.body[at].add((reported - body).norm());
        report.euler[at].add((reported - euler).norm());
    }
    for s in samples {
        let at = bin(s.state.pitch_rad);
        if s.grounded && s.state.angular_velocity_radps.norm() < STILL_RADPS {
            let expected = rest_specific_force(&s.state.attitude);
            report.gravity[at].add(expected.angle(&s.specific_force));
        }
        let difference =
            (s.realflight_euler - s.realflight_quaternion_euler).map(|d| wrap_angle(d).abs());
        report.realflight_euler[bin(s.realflight_quaternion_euler.y)].add(difference.max());
    }
    report
}

fn main() {
    let args = Args::parse();
    let file = File::open(&args.capture).expect("failed to open the capture");
    let exchanges = read_capture(BufReader::new(file)).expect("failed to read the capture");
    let body_rotation = UnitQuaternion::from_euler_angles(0.0, args.pitch_deg.to_radians(), 0.0);
    let samples: Vec<Sample> = exchanges
        .iter()
        .map(|exchange| sample(exchange, &body_rotation))
        .collect();
    println!(
        "{}: {} exchanges, body rotation pitch {}°",
        args.capture.display(),
        samples.len(),
        args.pitch_deg
    );

    let report = check(&samples);
    for (at, edges) in PITCH_BINS_DEG.windows(2).enumerate() {
        println!("|pitch| {:.0}–{:.0}°:", edges[0], edges[1].min(90.0));
        println!(
            "  rate vs quaternion body rate: {}",
            report.body[at].report("rad/s", 1.0)
        );
        println!(
            "  rate vs Euler-angle rate:     {}",
            report.euler[at].report("rad/s", 1.0)
        );
        println!(
            "  gravity at rest (angle):      {}",
            report.gravity[at].report("°", 180.0 / PI)
        );
        println!(
            "  RealFlight Euler vs quat (RealFlight |pitch|): {}",
            report.realflight_euler[at].report("°", 180.0 / PI)
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert2::check;

    const DT: f32 = 0.004;

    /// A vehicle turning at `body_radps` (FRD) from `start`, sampled every `DT`.
    fn turning(start: UnitQuaternion<f32>, body_radps: Vec3, n: usize) -> Vec<Sample> {
        let step = UnitQuaternion::from_scaled_axis(body_radps * DT);
        let mut attitude = start;
        (0..n)
            .map(|i| {
                let (roll_rad, pitch_rad, heading_rad) = euler_angles(&attitude);
                let state = VehicleState {
                    attitude,
                    angular_velocity_radps: body_radps,
                    roll_rad,
                    pitch_rad,
                    heading_rad,
                    ..Default::default()
                };
                attitude *= step;
                Sample {
                    t_s: i as f32 * DT,
                    realflight_euler: Vec3::new(roll_rad, pitch_rad, heading_rad),
                    realflight_quaternion_euler: Vec3::new(roll_rad, pitch_rad, heading_rad),
                    grounded: false,
                    specific_force: Vec3::zeros(),
                    state,
                }
            })
            .collect()
    }

    /// Body rates are recovered from the quaternions at any attitude, including nose-up;
    /// pitched, they no longer look like Euler rates.
    #[test]
    fn recovers_body_rates_and_tells_euler_rates_apart() {
        let rate = Vec3::new(0.3, -0.2, 0.5);
        // (roll, pitch, heading) of the start, Euler error expected
        let cases = [((0.0, 0.0, 0.0), false), ((0.1, 1.2, 0.5), true)];
        for ((roll, pitch, heading), euler_differs) in cases {
            let start = UnitQuaternion::from_euler_angles(roll, pitch, heading);
            let report = check(&turning(start, rate, 50));

            let at = bin(pitch);
            check!(report.body[at].max < 1e-3, "{pitch}");
            check!((report.euler[at].max > 0.1) == euler_differs, "{pitch}");
        }
    }

    /// At rest the specific force points up: along FRD −z level, and along +x nose-up.
    #[test]
    fn rest_specific_force_follows_the_attitude() {
        let level = UnitQuaternion::identity();
        let nose_up = UnitQuaternion::from_euler_angles(0.0, std::f32::consts::FRAC_PI_2, 0.0);
        check!((rest_specific_force(&level) - Vec3::new(0.0, 0.0, -GRAVITY)).norm() < 1e-4);
        check!((rest_specific_force(&nose_up) - Vec3::new(GRAVITY, 0.0, 0.0)).norm() < 1e-4);
    }

    /// A hover-level model (the eXplora) at rest on its tail, RealFlight-level: with a 90° body
    /// rotation it reads nose up, gravity along −x, and the gravity check passes; RealFlight's
    /// Euler angles are still compared in RealFlight's frame, where it reads level.
    #[test]
    fn body_rotation_reads_a_hover_level_rest_as_nose_up() {
        use stable_flight_sitl_realflight::realflight_bridge::{Quaternion, SimulatorState};
        let exchange = CapturedExchange {
            state: SimulatorState {
                orientation: Quaternion {
                    w: 1.0,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                acceleration_body_mps2: [0.0, 0.0, -GRAVITY].into(),
                is_touching_ground: true,
                ..Default::default()
            },
            channels: Default::default(),
        };
        let nose_up = UnitQuaternion::from_euler_angles(0.0, std::f32::consts::FRAC_PI_2, 0.0);
        let s = sample(&exchange, &nose_up);
        check!((s.state.pitch_rad.to_degrees() - 90.0).abs() < 0.05);
        check!((s.specific_force - Vec3::new(GRAVITY, 0.0, 0.0)).norm() < 1e-4);

        let report = check(&[s]);
        check!(report.gravity[2].count == 1);
        check!(report.gravity[2].max < 1e-3);
        check!(report.realflight_euler[0].count == 1);
    }

    /// Pitch bins split at 60° and 80°, and ±90° falls in the last.
    #[test]
    fn bins_by_absolute_pitch() {
        let degrees = |d: f32| d.to_radians();
        check!(bin(degrees(10.0)) == 0);
        check!(bin(degrees(-70.0)) == 1);
        check!(bin(degrees(90.0)) == 2);
        check!(bin(degrees(-90.0)) == 2);
    }

    /// Stalls (gaps past `2 · MAX_GAP_S`) are left out of the rate checks.
    #[test]
    fn skips_stalls() {
        let mut samples = turning(UnitQuaternion::identity(), Vec3::new(0.0, 0.0, 1.0), 3);
        samples[2].t_s = 1.0;
        let report = check(&samples);
        check!(report.body[0].count == 0);
    }
}
