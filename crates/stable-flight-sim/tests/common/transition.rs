//! A VTOL transition flight sampled every tick: a [`Trace`] of modes and states, for asking
//! when modes changed and how high or low it flew.

use assert2::check;
use stable_flight::{
    Command, Commander, ControlInput, FlightLoop, FlightMode, Guidance, Mixer, ModeRefusal,
    Stabilizer, TelemetryPublisher, Vec3, Vehicle, VehicleState,
};
use stable_flight_sim::{SimQuadPlane, SimTailsitter};

use super::plane::{Interlude, north_line_command};
use super::tailsitter::view_angles;
use super::{Hover, fly, hover_at, waypoint};

/// Height the transition tests hover at and fly the plane leg at, in m: clear of the
/// transition's `min_height_m` and of PX4's 20 m altitude-loss band (`VT_QC_T_ALT_LOSS`).
pub const TRANSITION_ALTITUDE_M: f32 = 30.0;

/// Seconds of hover, from resting on the ground, before `Plane` is requested: the climb to
/// [`TRANSITION_ALTITUDE_M`] and settling.
pub const FORWARD_AT_S: f32 = 20.0;

/// PX4's forward-transition timeout (`VT_TRANS_TIMEOUT`, PX4 `c4e4ef98e9`,
/// `src/modules/vtol_att_control/vtol_att_control_params.yaml:92-97`): plane flight by then.
pub const FORWARD_TIMEOUT_S: f32 = 15.0;

/// PX4's longest back transition (`VT_B_TRANS_DUR`, `:35-40`): hovering by then.
pub const BACK_DURATION_S: f32 = 10.0;

/// PX4's quad-chute altitude loss (`VT_QC_T_ALT_LOSS`, `:145-160`), in m below the height the
/// transition began, watched until [`ALTITUDE_LOSS_AFTER_S`] after it ends. PX4 watches forward
/// transitions; every transition is held to it here.
pub const ALTITUDE_LOSS_M: f32 = 20.0;

/// Seconds after a transition ends that [`ALTITUDE_LOSS_M`] still applies (PX4, as above).
pub const ALTITUDE_LOSS_AFTER_S: f32 = 5.0;

/// Seconds the plane flies after the forward transition before the hold is judged
/// (`plane_level`'s settling time).
pub const PLANE_SETTLE_S: f32 = 15.0;

/// Seconds a plane hold is judged over (`plane_level`'s window).
pub const PLANE_HOLD_S: f32 = 30.0;

/// Seconds a level hold is judged over.
pub const HOLD_S: f32 = 10.0;

/// Distance off the line in plane flight, in m: no external band applies; the plane's 1 m
/// (`plane_level`).
pub const CROSS_TRACK_M: f32 = 1.0;

/// The tick a flight entered `mode` at or after `from_s`, or a failed check naming `what`.
pub fn entered_s(trace: &Trace, mode: FlightMode, from_s: f32, what: &str) -> f32 {
    let entered = trace.entered(mode, from_s);
    check!(entered.is_some(), "{what}: never flew {mode:?}");
    entered.map_or(f32::INFINITY, |sample| sample.time_s)
}

/// The heights from `from_s`, when a transition was requested, until `ALTITUDE_LOSS_AFTER_S`
/// after it ended at `ended_s`.
pub fn transition_heights(trace: &Trace, from_s: f32, ended_s: f32) -> Heights {
    trace.heights(from_s, ended_s + ALTITUDE_LOSS_AFTER_S)
}

/// Checks the lowest of `heights` is within [`ALTITUDE_LOSS_M`] below their start.
pub fn check_altitude_loss(heights: Heights, what: &str) {
    let fell_m = heights.start_m - heights.lowest_m;
    check!(
        fell_m <= ALTITUDE_LOSS_M,
        "{what}: fell {fell_m} m, {heights:?}"
    );
}

/// Hovers over the origin at [`TRANSITION_ALTITUDE_M`], then from [`FORWARD_AT_S`] flies the
/// line north as a plane.
pub type Forward = Interlude<Hover>;

/// [`Forward`], then from its `back_at_s` hovers over a waypoint.
pub type ForwardAndBack = Interlude<Forward>;

/// [`Forward`], the plane leg at `airspeed_mps`.
pub fn forward_at(airspeed_mps: f32) -> Forward {
    Interlude::new(
        hover_at(TRANSITION_ALTITUDE_M),
        FORWARD_AT_S,
        f32::INFINITY,
        north_line_command(TRANSITION_ALTITUDE_M, airspeed_mps),
    )
}

/// [`forward_at`], then from `back_at_s` a `Copter` request to hover `north_m` up the line at
/// [`TRANSITION_ALTITUDE_M`].
pub fn forward_and_back_at(airspeed_mps: f32, back_at_s: f32, north_m: f32) -> ForwardAndBack {
    let hover = waypoint(north_m, 0.0, TRANSITION_ALTITUDE_M);
    Interlude::new(
        forward_at(airspeed_mps),
        back_at_s,
        f32::INFINITY,
        Command::new(true, ControlInput::Waypoint(hover)),
    )
}

/// One tick of a transition flight.
#[derive(Debug, Clone)]
pub struct Sample {
    /// Simulated seconds since the flight began.
    pub time_s: f32,
    pub mode: FlightMode,
    pub refusal: Option<ModeRefusal>,
    pub state: VehicleState,
}

/// A simulated VTOL's clock.
pub trait SimClock {
    /// Simulated seconds since the flight began.
    fn time_s(&self) -> f64;
}

impl SimClock for SimTailsitter {
    fn time_s(&self) -> f64 {
        SimTailsitter::time_s(self)
    }
}

impl SimClock for SimQuadPlane {
    fn time_s(&self) -> f64 {
        SimQuadPlane::time_s(self)
    }
}

/// Flies `flight_loop` for `seconds`, sampling every tick.
pub fn trace<C, V, G, S, M, T>(
    flight_loop: &mut FlightLoop<C, V, G, S, M, T>,
    seconds: f32,
) -> Trace
where
    C: Commander,
    V: Vehicle + SimClock,
    G: Guidance,
    S: Stabilizer,
    M: Mixer,
    T: TelemetryPublisher,
{
    let start_s = flight_loop.vehicle().time_s() as f32;
    let mut samples = Vec::new();
    fly(flight_loop, seconds, |flight_loop, elapsed_s| {
        let report = flight_loop
            .guidance()
            .flight_mode()
            .expect("a VTOL reports its mode");
        let state = flight_loop.state().clone();
        samples.push(Sample {
            time_s: start_s + elapsed_s,
            mode: report.mode,
            refusal: report.refusal,
            state,
        });
    });
    Trace { samples }
}

/// The ticks of a transition flight, for asking when modes changed and how low it flew.
pub struct Trace {
    pub samples: Vec<Sample>,
}

impl Trace {
    /// The first tick at or after `from_s` that switched into `mode` (a flight's first tick
    /// counts as a switch).
    pub fn entered(&self, mode: FlightMode, from_s: f32) -> Option<&Sample> {
        let mut previous = None;
        self.samples.iter().find(|sample| {
            let switched = sample.mode == mode && previous != Some(mode);
            previous = Some(sample.mode);
            switched && sample.time_s >= from_s
        })
    }

    /// The first tick at or after `time_s`.
    pub fn at(&self, time_s: f32) -> Option<&Sample> {
        self.samples.iter().find(|sample| sample.time_s >= time_s)
    }

    /// The ticks from `from_s` until `until_s`.
    pub fn window(&self, from_s: f32, until_s: f32) -> impl Iterator<Item = &Sample> {
        self.samples
            .iter()
            .filter(move |sample| (from_s..until_s).contains(&sample.time_s))
    }

    /// The height at `from_s` and the lowest and highest from then until `until_s`.
    pub fn heights(&self, from_s: f32, until_s: f32) -> Heights {
        let start_m = self
            .at(from_s)
            .map_or(f32::NAN, |sample| sample.state.position_m.z);
        self.window(from_s, until_s).fold(
            Heights {
                start_m,
                lowest_m: f32::INFINITY,
                highest_m: f32::NEG_INFINITY,
            },
            |heights, sample| {
                let height_m = sample.state.position_m.z;
                Heights {
                    lowest_m: heights.lowest_m.min(height_m),
                    highest_m: heights.highest_m.max(height_m),
                    ..heights
                }
            },
        )
    }

    /// The largest deviations of a level VTOL from `point_m` over the [`HOLD_S`] from `from_s`.
    pub fn level_hold(&self, from_s: f32, point_m: Vec3) -> HoverDeviation {
        let mut worst = HoverDeviation::default();
        for sample in self.window(from_s, from_s + HOLD_S) {
            worst.observe_level(&sample.state, point_m);
        }
        worst
    }
}

/// Heights over a window of a [`Trace`], in m.
#[derive(Debug, Clone, Copy)]
pub struct Heights {
    /// At the window's start.
    pub start_m: f32,
    pub lowest_m: f32,
    pub highest_m: f32,
}

/// Largest deviations from a hover point seen while holding it.
#[derive(Debug, Default)]
pub struct HoverDeviation {
    /// Off the point's altitude, in m.
    pub altitude_m: f32,
    /// Horizontal distance from the point, in m.
    pub horizontal_m: f32,
    /// Roll or pitch, in the hover view for a tailsitter: off level.
    pub tilt_deg: f32,
    /// Heading, in the hover view for a tailsitter.
    pub heading_deg: f32,
    /// Horizontal distance from the point at the last state observed, in m.
    pub final_m: f32,
}

impl HoverDeviation {
    /// Widens the deviations to cover a tailsitter's `state`, hovering nose up over `point_m`
    /// (north, east, up).
    pub fn observe(&mut self, state: &VehicleState, point_m: Vec3) {
        self.observe_angles(state, point_m, view_angles(state));
    }

    /// Widens the deviations to cover a level VTOL's `state`, hovering over `point_m`.
    pub fn observe_level(&mut self, state: &VehicleState, point_m: Vec3) {
        self.observe_angles(state, point_m, stable_flight::euler_angles(&state.attitude));
    }

    /// Widens the deviations to cover `state` at `(roll, pitch, heading)`, in rad.
    fn observe_angles(&mut self, state: &VehicleState, point_m: Vec3, angles: (f32, f32, f32)) {
        let (roll_rad, pitch_rad, heading_rad) = angles;
        let off_m = state.position_m - point_m;
        self.altitude_m = self.altitude_m.max(off_m.z.abs());
        self.final_m = off_m.xy().norm();
        self.horizontal_m = self.horizontal_m.max(self.final_m);
        self.tilt_deg = self
            .tilt_deg
            .max(roll_rad.abs().max(pitch_rad.abs()).to_degrees());
        self.heading_deg = self.heading_deg.max(heading_rad.abs().to_degrees());
    }
}
