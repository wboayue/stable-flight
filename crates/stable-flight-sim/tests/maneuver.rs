//! Scripted maneuvers on `airframes::plane()` and the plane sides of the sim tailsitter and the
//! eXplora: a roll, a loop, and roll and pitch doublets, each flown by a `ManeuverRunner` from
//! level flight up the north line, then the line again.

mod common;

use assert2::check;
use stable_flight::commanders::maneuver::{
    Axis, Maneuver, ManeuverConfig, ManeuverPhase, ManeuverRunner,
};
use stable_flight::{AbsoluteRateCommand, Command, Commander, ControlInput, VehicleState};
use stable_flight_sim::airframes;

use common::fly;
use common::plane::{ALTITUDE_BAND_M, Interlude, PlanePlant, plane_command, plane_loop};
use common::tailsitter::{TailsitterPlant, forward, tailsitter_loop};

/// Seconds of plane flight on the line before the maneuver: settled at trim airspeed. The
/// tailsitter reaches the line in plane flight about 22 s after its forward transition begins.
const PLANE_LEAD_S: f32 = 20.0;
const TAILSITTER_LEAD_S: f32 = 45.0;

/// Seconds flown after the maneuver starts.
const WINDOW_S: f32 = 40.0;

/// Flies `lead`, then from `at_s` (simulated seconds from the first tick) `maneuver` until its
/// runner is done, then `lead` again.
struct ManeuverAt<C> {
    lead: C,
    at_s: f32,
    maneuver: Maneuver,
    runner: ManeuverRunner,
    elapsed_s: f32,
    started: bool,
}

impl<C> ManeuverAt<C> {
    /// Full thrust for aerobatics, `cruise_thrust` for doublets.
    fn new(lead: C, at_s: f32, maneuver: Maneuver, cruise_thrust: f32) -> Self {
        ManeuverAt {
            lead,
            at_s,
            maneuver,
            runner: ManeuverRunner::new(ManeuverConfig {
                cruise_thrust,
                ..ManeuverConfig::default()
            }),
            elapsed_s: 0.0,
            started: false,
        }
    }

    fn phase(&self) -> ManeuverPhase {
        self.runner.phase()
    }
}

impl<C: Commander> Commander for ManeuverAt<C> {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.elapsed_s += dt;
        if !self.started && self.elapsed_s >= self.at_s {
            self.started = true;
            self.runner.start(self.maneuver, state);
        }
        match self.runner.next(dt, state) {
            Some(input) => plane_command(input),
            None => self.lead.next_command(dt, state),
        }
    }
}

/// What a maneuver did, from its start.
#[derive(Debug, Default)]
struct Flown {
    /// Seconds rotating, and recovering.
    rotation_s: f32,
    recovery_s: f32,
    /// The phase at the end of the window.
    done: bool,
    /// Highest and lowest altitude while rotating, in m.
    top_m: f32,
    bottom_m: f32,
    /// Altitude at the start, and lowest over the window, in m.
    entry_m: f32,
    lowest_m: f32,
    /// Peak body rate about the axis in each of the first two segments, each its own way.
    segment_peaks_radps: [f32; 2],
    /// Body rate magnitude on the last rotating tick, in rad/s.
    final_rate_radps: f32,
    /// Largest distance off the line (east) and off the entry altitude over the last 10 s, in m.
    cross_track_m: f32,
    altitude_error_m: f32,
}

/// Observes one tick of a maneuver flown about `axis` in `sign`'s direction.
fn observe(
    flown: &mut Flown,
    phase: ManeuverPhase,
    state: &VehicleState,
    t: f32,
    axis: Axis,
    sign: f32,
) {
    let z = state.position_m.z;
    flown.lowest_m = flown.lowest_m.min(z);
    let rate = state.angular_velocity_radps[axis as usize] * sign;
    match phase {
        ManeuverPhase::Rotation { segment } => {
            if segment < 2 {
                let way = if segment == 0 { 1.0 } else { -1.0 };
                flown.segment_peaks_radps[segment] =
                    flown.segment_peaks_radps[segment].max(rate * way);
            }
            flown.rotation_s += common::TICK.as_secs_f32();
            flown.top_m = flown.top_m.max(z);
            flown.bottom_m = flown.bottom_m.min(z);
            flown.final_rate_radps = state.angular_velocity_radps.norm();
        }
        ManeuverPhase::Recovery => flown.recovery_s += common::TICK.as_secs_f32(),
        _ => {}
    }
    flown.done = phase == ManeuverPhase::Done;
    if t > WINDOW_S - 10.0 {
        flown.cross_track_m = flown.cross_track_m.max(state.position_m.y.abs());
        flown.altitude_error_m = flown.altitude_error_m.max((z - flown.entry_m).abs());
    }
}

/// Starts `flown` at `state`.
fn begun(state: &VehicleState) -> Flown {
    let z = state.position_m.z;
    Flown {
        entry_m: z,
        lowest_m: z,
        top_m: z,
        bottom_m: z,
        ..Flown::default()
    }
}

/// A vehicle's lead commander, boxed so either vehicle's fits one commander type.
struct Lead(Box<dyn Commander>);

impl Commander for Lead {
    fn next_command(&mut self, dt: f32, state: &VehicleState) -> Command {
        self.0.next_command(dt, state)
    }
}

/// The vehicles flown.
#[derive(Clone, Copy, Debug)]
enum Vehicle {
    /// `airframes::plane()`.
    Plane,
    /// `airframes::tailsitter()`'s plane side.
    Tailsitter,
    /// `airframes::explora()`'s plane side.
    Explora,
}

impl Vehicle {
    /// Flies the commander `build` makes from the vehicle's lead (the line), the lead's seconds,
    /// and its cruise thrust: through the lead, then `window_s` more, passing `each` the
    /// commander, the state, and the seconds into the window each tick.
    fn run<C: Commander>(
        self,
        build: impl FnOnce(Lead, f32, f32) -> C,
        window_s: f32,
        mut each: impl FnMut(&C, &VehicleState, f32),
    ) {
        match self {
            Vehicle::Plane => {
                let plant = PlanePlant::clean(airframes::plane());
                let lead = Lead(Box::new(plant.line()));
                let commander = build(lead, PLANE_LEAD_S, plant.airframe.guidance.cruise_thrust);
                let mut flight_loop = plane_loop(&plant, commander);
                fly(&mut flight_loop, PLANE_LEAD_S, |_, _| {});
                fly(&mut flight_loop, window_s, |flight_loop, t| {
                    each(flight_loop.commander(), flight_loop.state(), t);
                });
            }
            Vehicle::Tailsitter | Vehicle::Explora => {
                let airframe = match self {
                    Vehicle::Explora => airframes::explora(),
                    _ => airframes::tailsitter(),
                };
                let plant = TailsitterPlant::clean(airframe);
                let lead = Lead(Box::new(forward(&airframe)));
                let cruise_thrust = airframe.guidance.plane.cruise_thrust;
                let commander = build(lead, TAILSITTER_LEAD_S, cruise_thrust);
                let mut flight_loop = tailsitter_loop(&plant, commander);
                fly(&mut flight_loop, TAILSITTER_LEAD_S, |_, _| {});
                fly(&mut flight_loop, window_s, |flight_loop, t| {
                    each(flight_loop.commander(), flight_loop.state(), t);
                });
            }
        }
    }

    /// `maneuver` flown from the line, observed about `axis` its `sign`'s way.
    fn fly(self, maneuver: Maneuver, axis: Axis, sign: f32) -> Flown {
        let mut flown: Option<Flown> = None;
        self.run(
            |lead, lead_s, cruise_thrust| ManeuverAt::new(lead, lead_s, maneuver, cruise_thrust),
            WINDOW_S,
            |commander, state, t| {
                let flown = flown.get_or_insert_with(|| begun(state));
                observe(flown, commander.phase(), state, t, axis, sign);
            },
        );
        flown.unwrap_or_default()
    }

    /// Peak rate about `axis` over a `radps` step held [`STEP_S`] at cruise thrust from the line.
    fn step_peak_radps(self, axis: Axis, radps: f32) -> f32 {
        let rate = |about| if axis == about { radps } else { 0.0 };
        let mut peak_radps = 0.0_f32;
        self.run(
            |lead, lead_s, cruise_thrust| {
                let step = AbsoluteRateCommand {
                    roll_radps: rate(Axis::Roll),
                    pitch_radps: rate(Axis::Pitch),
                    yaw_radps: rate(Axis::Yaw),
                    thrust: cruise_thrust,
                };
                let command = plane_command(ControlInput::AbsoluteRate(step));
                Interlude::new(lead, lead_s, lead_s + STEP_S, command)
            },
            STEP_S,
            |_, state, _| {
                peak_radps = peak_radps.max(state.angular_velocity_radps[axis as usize]);
            },
        );
        peak_radps
    }
}

/// Seconds a step, and a doublet's half, are held.
const STEP_S: f32 = 1.0;

/// Longest recovery to level, in s. Measured 1.39 s at most; 2 s leaves about 1.4× margin.
const RECOVERY_S: f32 = 2.0;

/// Off the line, sideways, over the window's last 10 s, in m. Measured 0.0 m (every maneuver
/// stays in the vertical plane of the line); 1 m leaves room for the line's capture.
const CROSS_TRACK_M: f32 = 1.0;

/// Checks what every maneuver must do: finish, recover in [`RECOVERY_S`], and fly the line
/// again within [`CROSS_TRACK_M`] and `ALTITUDE_BAND_M` of the entry altitude.
fn check_flies_on(case: &str, flown: &Flown) {
    check!(flown.done, "{case}: {flown:?}");
    check!(flown.recovery_s <= RECOVERY_S, "{case}: {flown:?}");
    check!(flown.cross_track_m <= CROSS_TRACK_M, "{case}: {flown:?}");
    check!(
        flown.altitude_error_m <= ALTITUDE_BAND_M,
        "{case}: {flown:?}"
    );
}

/// A roll at 180°/s turns a full turn before its timeout (twice the 2 s it takes at that
/// rate), both ways, and loses no more height than measured: (vehicle, rate in deg/s, longest
/// rotation in s, most altitude lost in m). Pure roll rate lets the lift tip over, so height
/// goes: measured 3.06 s and 23.4 m on the plane (whose ailerons reach 2.8 rad/s at most),
/// 1.81 s and 12.6 m on the tailsitter, 2.03 s and 6.8 m on the eXplora; each band is about
/// 1.2× that.
#[test]
fn rolls_turn_a_full_turn_and_fly_on() {
    let cases = [
        (Vehicle::Plane, 180.0, 3.6, 28.0),
        (Vehicle::Plane, -180.0, 3.6, 28.0),
        (Vehicle::Tailsitter, 180.0, 2.2, 15.0),
        (Vehicle::Tailsitter, -180.0, 2.2, 15.0),
        (Vehicle::Explora, 180.0, 2.5, 8.0),
        (Vehicle::Explora, -180.0, 2.5, 8.0),
    ];
    for (vehicle, rate_degps, rotation_s, loss_m) in cases {
        let case = format!("{vehicle:?} at {rate_degps}°/s");
        let maneuver = Maneuver::roll(f32::to_radians(rate_degps)).expect("valid");
        let flown = vehicle.fly(maneuver, Axis::Roll, rate_degps.signum());
        check!(flown.rotation_s <= rotation_s, "{case}: {flown:?}");
        check!(
            flown.entry_m - flown.lowest_m <= loss_m,
            "{case}: {flown:?}"
        );
        check_flies_on(&case, &flown);
    }
}

/// A loop flies a circle of its radius: half the height it spans is within 15% of the radius
/// (measured 6–8% short on the plane and tailsitter, where the open-loop rate lags the slowing
/// climb; 8–12% wide on the eXplora, whose rate lags the speed it gains coming down), and it
/// sinks below where it began by no more than measured: (vehicle, radius in m, most altitude
/// lost in m). Measured 2.0 m at most on the plane and tailsitter (3 m band), 4.1 m and 6.4 m
/// on the eXplora's 10 m and 25 m loops (5 m and 8 m). RealFlight's eXplora flew them 15% and
/// 12% wide, losing 6 m and 16 m (#342).
#[test]
fn loops_fly_their_radius_and_fly_on() {
    let cases = [
        (Vehicle::Plane, 10.0, 3.0),
        (Vehicle::Plane, 25.0, 3.0),
        (Vehicle::Tailsitter, 10.0, 3.0),
        (Vehicle::Tailsitter, 25.0, 3.0),
        (Vehicle::Explora, 10.0, 5.0),
        (Vehicle::Explora, 25.0, 8.0),
    ];
    for (vehicle, radius_m, loss_m) in cases {
        let case = format!("{vehicle:?}, {radius_m} m");
        let flown = vehicle.fly(Maneuver::loop_(radius_m).expect("valid"), Axis::Pitch, 1.0);
        let flown_radius_m = (flown.top_m - flown.bottom_m) / 2.0;
        check!(
            (flown_radius_m - radius_m).abs() <= 0.15 * radius_m,
            "{case}: {flown_radius_m} m"
        );
        check!(
            flown.entry_m - flown.lowest_m <= loss_m,
            "{case}: {flown:?}"
        );
        check_flies_on(&case, &flown);
    }
}

/// A doublet's first half peaks as a held step of its rate does, at cruise thrust (measured
/// equal to 0.01 rad/s; 0.05 rad/s of margin), and by the end of its pause the body rate has
/// fallen under 0.7 rad/s (measured 0.48 at most, the plane's pitch): (vehicle, axis).
#[test]
fn doublets_measure_the_step_response() {
    let cases = [
        (Vehicle::Plane, Axis::Roll),
        (Vehicle::Plane, Axis::Pitch),
        (Vehicle::Tailsitter, Axis::Roll),
        (Vehicle::Tailsitter, Axis::Pitch),
        (Vehicle::Explora, Axis::Roll),
        (Vehicle::Explora, Axis::Pitch),
    ];
    let radps = 1.0;
    for (vehicle, axis) in cases {
        let case = format!("{vehicle:?} {axis:?}");
        let doublet = Maneuver::doublet(axis, radps, STEP_S).expect("valid");
        let flown = vehicle.fly(doublet, axis, 1.0);
        let step_peak_radps = vehicle.step_peak_radps(axis, radps);
        check!(
            (flown.segment_peaks_radps[0] - step_peak_radps).abs() <= 0.05,
            "{case}: doublet {} vs step {step_peak_radps}",
            flown.segment_peaks_radps[0]
        );
        check!(flown.final_rate_radps <= 0.7, "{case}: {flown:?}");
        check_flies_on(&case, &flown);
    }
}

/// The eXplora's doublets at 60°/s peak within 1.3× of the command in their first half
/// (measured 1.01 in roll, 1.13 in pitch), where PX4's default feedforward peaked at 3.2× and
/// 3.4× in RealFlight (#342).
#[test]
fn explora_doublets_track_their_command() {
    let radps = 60f32.to_radians();
    for axis in [Axis::Roll, Axis::Pitch] {
        let doublet = Maneuver::doublet(axis, radps, 0.5).expect("valid");
        let flown = Vehicle::Explora.fly(doublet, axis, 1.0);
        let peak = flown.segment_peaks_radps[0] / radps;
        check!((0.8..=1.3).contains(&peak), "{axis:?}: {peak}");
    }
}

/// Centered rate sticks at cruise thrust hold the pitch of level flight: over 10 s the pitch
/// stays within 0.5° of where it began (measured 0.03° at most). The rate loop's integral
/// carries the level-flight moment into rate flight, so this holds without trim too; the
/// eXplora's trim is RealFlight's (#342).
#[test]
fn centered_rate_sticks_hold_pitch() {
    for vehicle in [Vehicle::Plane, Vehicle::Tailsitter, Vehicle::Explora] {
        let mut start_rad = None;
        let mut drift_rad = 0.0_f32;
        vehicle.run(
            |lead, lead_s, cruise_thrust| {
                let centered = AbsoluteRateCommand {
                    roll_radps: 0.0,
                    pitch_radps: 0.0,
                    yaw_radps: 0.0,
                    thrust: cruise_thrust,
                };
                let command = plane_command(ControlInput::AbsoluteRate(centered));
                Interlude::new(lead, lead_s, lead_s + HOLD_S, command)
            },
            HOLD_S,
            |_, state, _| {
                let pitch_rad = stable_flight::euler_angles(&state.attitude).1;
                let start_rad = *start_rad.get_or_insert(pitch_rad);
                drift_rad = drift_rad.max((pitch_rad - start_rad).abs());
            },
        );
        check!(
            drift_rad <= 0.5f32.to_radians(),
            "{vehicle:?}: {drift_rad} rad"
        );
    }
}

/// Seconds centered rate sticks are held.
const HOLD_S: f32 = 10.0;
