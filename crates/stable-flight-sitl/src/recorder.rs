//! Per-tick CSV recording of a flight, for reading a run after it lands.
//!
//! [`CsvRecorder`] is a [`TelemetryPublisher`] that writes one row per tick: `wall_s` (seconds
//! since the recorder was made) followed by a [`Columns`] set's fields. Three sets ship:
//! [`CopterColumns`], [`TailsitterColumns`], and [`QuadPlaneColumns`]. Fan it out with the link as the flight loop's
//! telemetry, `(link, recorder)`.
//!
//! # Examples
//!
//! ```no_run
//! use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
//!
//! // `None` (no `--record` given) records nothing.
//! let path: Option<&str> = Some("flight.csv");
//! let recorder = CsvRecorder::create(path, CopterColumns).expect("failed to create the CSV");
//! ```

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;
use std::time::Instant;

use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::{
    ControlInput, FlightMode, Setpoint, TelemetryPublisher, TelemetryRecord, Vec3, euler_angles,
};

/// The fields a [`CsvRecorder`] writes after `wall_s`.
///
/// # Examples
///
/// A set recording only altitude and thrust:
///
/// ```
/// use std::io::{self, Write};
///
/// use stable_flight::TelemetryRecord;
/// use stable_flight_sitl::recorder::{Columns, CsvRecorder};
///
/// struct Climb;
///
/// impl Columns for Climb {
///     const HEADER: &'static str = "altitude,thrust";
///
///     fn write_row(&mut self, out: &mut impl Write, record: &TelemetryRecord) -> io::Result<()> {
///         write!(out, "{:.3},{:.4}", record.state.position_m.z, record.control.thrust)
///     }
/// }
///
/// let recorder = CsvRecorder::new(Some(Vec::new()), Climb).expect("header written");
/// let csv = recorder.into_writer().expect("a writer");
/// assert_eq!(csv, b"wall_s,altitude,thrust\n");
/// ```
pub trait Columns {
    /// The header's names, comma-separated, matching [`write_row`](Columns::write_row)'s fields.
    const HEADER: &'static str;

    /// Writes one tick's fields, comma-separated, without a leading comma or a line end.
    fn write_row(&mut self, out: &mut impl Write, record: &TelemetryRecord) -> io::Result<()>;
}

/// Writes one CSV row per tick to an optional writer; with none, publishing does nothing.
///
/// A failed write loses diagnostics, not flight: it is ignored, never panics, and later ticks
/// still try to write. Rows are buffered by `W` and flushed when it drops.
///
/// # Examples
///
/// ```
/// use stable_flight::TelemetryPublisher;
/// use stable_flight::testing::RecordedTick;
/// use stable_flight_sitl::recorder::{CopterColumns, CsvRecorder};
/// # use stable_flight::*;
/// # use std::time::Duration;
/// # let tick = RecordedTick {
/// #     dt: Duration::from_millis(4),
/// #     command: Command::default(),
/// #     setpoint: Setpoint::Attitude(AttitudeSetpoint::default()),
/// #     state: VehicleState::default(),
/// #     control: ControlOutput::default(),
/// #     actuators: ActuatorCommand::default(),
/// #     saturation: Saturation::default(),
/// #     failsafe: None,
/// #     flight_mode: FlightModeReport::default(),
/// # };
///
/// let mut recorder = CsvRecorder::new(Some(Vec::new()), CopterColumns).expect("header written");
/// recorder.publish(&tick.as_record());
/// let csv = String::from_utf8(recorder.into_writer().expect("a writer")).expect("UTF-8");
/// assert_eq!(csv.lines().count(), 2); // header and one row
/// ```
pub struct CsvRecorder<C: Columns, W: Write = BufWriter<File>> {
    out: Option<W>,
    start: Instant,
    columns: C,
}

impl<C: Columns> CsvRecorder<C, BufWriter<File>> {
    /// Records to a new file at `path` (truncating one there), or nothing without a path.
    ///
    /// # Errors
    ///
    /// If the file can't be created or the header written.
    pub fn create(path: Option<impl AsRef<Path>>, columns: C) -> io::Result<Self> {
        let out = path
            .map(|path| File::create(path).map(BufWriter::new))
            .transpose()?;
        Self::new(out, columns)
    }
}

impl<C: Columns, W: Write> CsvRecorder<C, W> {
    /// Records to `out`, writing the header now, or nothing without a writer.
    ///
    /// # Errors
    ///
    /// If the header can't be written.
    pub fn new(mut out: Option<W>, columns: C) -> io::Result<Self> {
        if let Some(out) = &mut out {
            writeln!(out, "wall_s,{}", C::HEADER)?;
        }
        Ok(Self {
            out,
            start: Instant::now(),
            columns,
        })
    }

    /// The writer, if any, ending the recording.
    pub fn into_writer(self) -> Option<W> {
        self.out
    }
}

impl<C: Columns, W: Write> TelemetryPublisher for CsvRecorder<C, W> {
    fn publish(&mut self, record: &TelemetryRecord) {
        let Some(out) = &mut self.out else {
            return;
        };
        let wall_s = self.start.elapsed().as_secs_f32();
        // A failed write loses diagnostics, not flight: ignore it.
        let _ = write!(out, "{wall_s:.4},")
            .and_then(|()| self.columns.write_row(out, record))
            .and_then(|()| writeln!(out));
    }
}

/// A copter's attitude loops: body rates (rad/s), Euler angles (rad), altitude (m), moments,
/// thrust (after the stabilizer's thrust curve), the attitude setpoint (rad; NaN for other
/// setpoints), the stick yaw (`yaw` of attitude sticks, `yaw_rate` of rate sticks; NaN for
/// other inputs), and the horizontal position and ground velocity (`north_m`, `east_m`,
/// `vel_north_mps`, `vel_east_mps`; see [`TailsitterColumns`]).
#[derive(Debug, Default, Clone, Copy)]
pub struct CopterColumns;

impl Columns for CopterColumns {
    const HEADER: &'static str = "armed,roll_rate,pitch_rate,yaw_rate,roll,pitch,heading,altitude,moment_x,moment_y,moment_z,thrust,sp_roll,sp_pitch,sp_heading,stick_yaw,north_m,east_m,vel_north_mps,vel_east_mps";

    fn write_row(&mut self, out: &mut impl Write, record: &TelemetryRecord) -> io::Result<()> {
        let state = record.state;
        let rates = state.angular_velocity_radps;
        let (roll, pitch, heading) = euler_angles(&state.attitude);
        let moments = record.control.moments;
        let nan = f32::NAN;
        let (sp_roll, sp_pitch, sp_heading) = match record.setpoint {
            Setpoint::Attitude(s) => (s.roll_rad, s.pitch_rad, s.heading_rad),
            _ => (nan, nan, nan),
        };
        let stick_yaw = match &record.command.input {
            ControlInput::Attitude(c) => c.yaw,
            ControlInput::Rate(c) => c.yaw_rate,
            _ => nan,
        };
        write!(
            out,
            "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3}",
            u8::from(record.command.arm),
            rates.x,
            rates.y,
            rates.z,
            roll,
            pitch,
            heading,
            state.position_m.z,
            moments.x,
            moments.y,
            moments.z,
            record.control.thrust,
            sp_roll,
            sp_pitch,
            sp_heading,
            stick_yaw,
            state.position_m.x,
            state.position_m.y,
            state.velocity_mps.x,
            state.velocity_mps.y,
        )
    }
}

/// A tailsitter's hover, transitions, and plane flight, for reading stick pulses and
/// transitions.
///
/// Rates, moments, and actuators are in the airplane body frame; `view_*` are the attitude in
/// the tailsitter view, where hover is level (the body Euler angles are degenerate there).
/// `mode` is the flight mode (0 copter, 1 plane, 2 to plane, 3 to copter, 9 other) and
/// `refused` whether a requested regime is refused. `sp_kind` names guidance's setpoint;
/// `sp_roll`/`sp_pitch`/`sp_heading` are its angles (in the view for a copter attitude, the
/// airplane frame for a plane one; NaN otherwise) and `sp_p`/`sp_q`/`sp_r` its body rates in
/// rad/s (NaN for attitudes and stick-fraction rates). `sp_thrust` is linear thrust, before the
/// stabilizer's thrust curve; `thrust` is after it. `airspeed_mps` is NaN without air data,
/// `climb_mps` the vertical speed, up positive, and `stick_throttle` the commander's throttle
/// (or absolute thrust), before boost, floor, and sink guard. `motor_*` are motor slots 0 and
/// 1, `elevon_*` servo slots 0 and 1. `north_m`/`east_m` are the position, from the backend's
/// origin (not the arming point), and `vel_north_mps`/`vel_east_mps` the ground velocity, on
/// the frame contract's north and east axes.
#[derive(Debug, Clone, Copy)]
pub struct TailsitterColumns {
    /// The tailsitter view's rotation from the body frame
    /// (`*stable_flight::View::tailsitter(()).rotation()`).
    pub view: UnitQuaternion<f32>,
}

impl Columns for TailsitterColumns {
    const HEADER: &'static str = "armed,mode,refused,roll_rate,pitch_rate,yaw_rate,roll,pitch,heading,view_roll,view_pitch,view_heading,altitude,airspeed_mps,moment_x,moment_y,moment_z,thrust,motor_left,motor_right,elevon_left,elevon_right,sp_kind,sp_roll,sp_pitch,sp_heading,sp_p,sp_q,sp_r,sp_thrust,climb_mps,stick_throttle,north_m,east_m,vel_north_mps,vel_east_mps";

    fn write_row(&mut self, out: &mut impl Write, record: &TelemetryRecord) -> io::Result<()> {
        let state = record.state;
        let rates = state.angular_velocity_radps;
        let (roll, pitch, heading) = euler_angles(&state.attitude);
        let view = state.rotate_body(&self.view);
        let moments = record.control.moments;
        let actuators = record.actuators;
        let nan = f32::NAN;
        let (sp_kind, sp_roll, sp_pitch, sp_heading, sp_p, sp_q, sp_r) =
            setpoint_fields(record.setpoint);
        let mode = mode_code(record.flight_mode.mode);
        let stick_throttle = match &record.command.input {
            ControlInput::Attitude(c) => c.throttle,
            ControlInput::Rate(c) => c.throttle,
            ControlInput::AbsoluteAttitude(c) => c.thrust,
            _ => nan,
        };
        write!(
            out,
            "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.4},{:.3},{:.3},{:.3},{:.3}",
            u8::from(record.command.arm),
            mode,
            u8::from(record.flight_mode.refusal.is_some()),
            rates.x,
            rates.y,
            rates.z,
            roll,
            pitch,
            heading,
            view.roll_rad,
            view.pitch_rad,
            view.heading_rad,
            state.position_m.z,
            state.valid_airspeed_mps().unwrap_or(nan),
            moments.x,
            moments.y,
            moments.z,
            record.control.thrust,
            actuators.motors[0],
            actuators.motors[1],
            actuators.servos[0],
            actuators.servos[1],
            sp_kind,
            sp_roll,
            sp_pitch,
            sp_heading,
            sp_p,
            sp_q,
            sp_r,
            record.setpoint.thrust(),
            state.velocity_mps.z,
            stick_throttle,
            state.position_m.x,
            state.position_m.y,
            state.velocity_mps.x,
            state.velocity_mps.y,
        )
    }
}

#[cfg(test)]
mod tests;

/// A lift+cruise VTOL's (quadplane's) hover, transitions, and plane flight: both sides'
/// outputs, for reading the lift trim, the pusher, and the handovers.
///
/// Rates, angles, and moments are in the body frame. `mode`, `refused`, `sp_kind`,
/// `sp_roll`/`sp_pitch`/`sp_heading`, `sp_p`/`sp_q`/`sp_r`, `sp_thrust`, `airspeed_mps`,
/// `climb_mps`, and the horizontal position and velocity are as in [`TailsitterColumns`]
/// (a blend's angles are its hover side's). `lift_share` is a blend's share of hover lift on
/// the lift rotors (NaN for other setpoints). `moment_*` and `thrust` are the lift rotors'
/// (after the thrust curve); `cruise_thrust` and `cruise_moment_*` are the pusher's and the
/// surfaces' (NaN without a cruise output). `motor_0`–`motor_3` are the lift rotor slots in
/// [`TableMixer::quad_x`](stable_flight::mixers::TableMixer::quad_x)'s order, `pusher` motor
/// slot 4, and `servo_0`–`servo_2` the surfaces (ailerons, then elevator and rudder or the
/// two ruddervators).
#[derive(Debug, Default, Clone, Copy)]
pub struct QuadPlaneColumns;

impl Columns for QuadPlaneColumns {
    const HEADER: &'static str = "armed,mode,refused,roll_rate,pitch_rate,yaw_rate,roll,pitch,heading,altitude,airspeed_mps,moment_x,moment_y,moment_z,thrust,cruise_thrust,cruise_moment_x,cruise_moment_y,cruise_moment_z,motor_0,motor_1,motor_2,motor_3,pusher,servo_0,servo_1,servo_2,sp_kind,sp_roll,sp_pitch,sp_heading,sp_p,sp_q,sp_r,sp_thrust,lift_share,climb_mps,north_m,east_m,vel_north_mps,vel_east_mps";

    fn write_row(&mut self, out: &mut impl Write, record: &TelemetryRecord) -> io::Result<()> {
        let state = record.state;
        let rates = state.angular_velocity_radps;
        let (roll, pitch, heading) = euler_angles(&state.attitude);
        let control = record.control;
        let nan = f32::NAN;
        let (cruise_thrust, cruise_moments) = control
            .cruise
            .map_or((nan, Vec3::repeat(nan)), |c| (c.thrust, c.moments));
        let motors = record.actuators.motors;
        let servos = record.actuators.servos;
        let (sp_kind, sp_roll, sp_pitch, sp_heading, sp_p, sp_q, sp_r) =
            setpoint_fields(record.setpoint);
        let lift_share = match record.setpoint {
            Setpoint::Blend(s) => s.lift_share,
            _ => nan,
        };
        write!(
            out,
            "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{:.3}",
            u8::from(record.command.arm),
            mode_code(record.flight_mode.mode),
            u8::from(record.flight_mode.refusal.is_some()),
            rates.x,
            rates.y,
            rates.z,
            roll,
            pitch,
            heading,
            state.position_m.z,
            state.valid_airspeed_mps().unwrap_or(nan),
            control.moments.x,
            control.moments.y,
            control.moments.z,
            control.thrust,
            cruise_thrust,
            cruise_moments.x,
            cruise_moments.y,
            cruise_moments.z,
            motors[0],
            motors[1],
            motors[2],
            motors[3],
            motors[4],
            servos[0],
            servos[1],
            servos[2],
            sp_kind,
            sp_roll,
            sp_pitch,
            sp_heading,
            sp_p,
            sp_q,
            sp_r,
            record.setpoint.thrust(),
            lift_share,
            state.velocity_mps.z,
            state.position_m.x,
            state.position_m.y,
            state.velocity_mps.x,
            state.velocity_mps.y,
        )
    }
}

/// A setpoint's kind, its angles (rad), and its body rates (rad/s), NaN where it has none: a
/// copter attitude's angles, a plane attitude's roll and pitch, a blend's hover side's angles,
/// body and plane rates' rates.
fn setpoint_fields(setpoint: &Setpoint) -> (&'static str, f32, f32, f32, f32, f32, f32) {
    let nan = f32::NAN;
    match *setpoint {
        Setpoint::Attitude(s) => (
            "attitude",
            s.roll_rad,
            s.pitch_rad,
            s.heading_rad,
            nan,
            nan,
            nan,
        ),
        Setpoint::Plane(s) => ("plane", s.roll_rad, s.pitch_rad, nan, nan, nan, nan),
        Setpoint::BodyRate(s) => (
            "body_rate",
            nan,
            nan,
            nan,
            s.roll_radps,
            s.pitch_radps,
            s.yaw_radps,
        ),
        Setpoint::PlaneRate(s) => (
            "plane_rate",
            nan,
            nan,
            nan,
            s.roll_radps,
            s.pitch_radps,
            s.yaw_radps,
        ),
        Setpoint::Blend(s) => (
            "blend",
            s.hover.roll_rad,
            s.hover.pitch_rad,
            s.hover.heading_rad,
            nan,
            nan,
            nan,
        ),
        _ => ("rate", nan, nan, nan, nan, nan, nan),
    }
}

/// A flight mode as a number: 0 copter, 1 plane, 2 to plane, 3 to copter, 9 other.
fn mode_code(mode: FlightMode) -> u8 {
    match mode {
        FlightMode::Copter => 0,
        FlightMode::Plane => 1,
        FlightMode::ToPlane => 2,
        FlightMode::ToCopter => 3,
        _ => 9,
    }
}
