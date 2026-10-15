//! The simulated lift+cruise VTOL: see [`quadplane`].

use stable_flight::GRAVITY;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::mixers::{MixRow, MixerConfig, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;

use crate::fixed_wing::FixedWingModel;
use crate::quadplane_model::{LiftRotors, QuadPlaneModel};
use crate::start::Start;

use super::plane::{aerosonde_inertia, plane};
use super::{hover_thrust, max_thrust_n, x_moment_arm_m};

/// A simulated quadplane: its physics, the hover mixer that drives its lift rotors and
/// surfaces, the hover guidance and stabilizer configs tuned for it, and a start in cruise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadPlaneAirframe {
    /// Physical parameters.
    pub model: QuadPlaneModel,
    /// The hover mixer: lift rotors on motors 0–3 in [`TableMixer::quad_x`]'s order, the
    /// ailerons, elevator, and rudder on servos 0–2 as on [`TableMixer::plane`]. No row drives
    /// the pusher (motor 4), so it stays stopped.
    pub mixer: TableMixer<7>,
    /// Hover guidance configuration.
    pub guidance: CopterGuidanceConfig,
    /// Hover stabilizer configuration.
    pub stabilizer: CopterStabilizerConfig,
    /// Level at [`plane`]'s 15 m/s trim airspeed, 100 m up over the origin, heading north.
    pub start: Start,
}

/// The pusher's motor slot: after the lift rotors.
const PUSHER_SLOT: u8 = 4;

/// ArduPilot SITL's quadplane (`quadplane`: [`plane`]'s airframe with an X quad,
/// `libraries/SITL/SIM_QuadPlane.cpp:29`, Plane-4.5): 4.5 kg, the quad frame's default 3 kg
/// scaled by 1.5 for the plane's parts (`SIM_Frame.cpp:51,774`, `SIM_QuadPlane.cpp:103-104`),
/// its lift rotors 350 mm apart across the diagonal (`SIM_Frame.cpp:57`). The wing,
/// aerodynamics, surfaces, and parasitic drag are [`plane`]'s: ArduPilot adds no drag for the
/// quad frame in a plane build (`SIM_Frame.cpp:33-37`), so the booms and stopped rotors are in
/// the plane's.
///
/// The rest isn't in ArduPilot's model and is chosen:
/// - Inertia: the whole airframe's, [`plane`]'s Aerosonde scaling at 4.5 kg; ArduPilot's
///   quad frame adds a motor ring's on top of a plane model with none.
/// - Lift rotors: thrust-to-weight 2, 50 ms lag, and yaw torque of
///   [`heavy_quad`](super::heavy_quad), the quad of ArduPilot's default frame; in-plane drag
///   of 1.2 N per m/s at full thrust, so about `heavy_quad`'s 0.6 in a hover.
/// - Pusher: [`plane`]'s static thrust-to-weight of 1.2 at this mass (ArduPilot sizes the
///   prop from the mass too, `SIM_Plane.cpp:54`).
///
/// It hovers on core's copter guidance and stabilizer: roll and pitch rate and angle gains
/// from the sim autotune (`StagePlan::copter_search`), yaw and heading core's defaults, and
/// the hover thrust worked out from the model. The cruise configs come with the quadplane
/// guidance (#391).
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::copter::CopterGuidance;
/// use stable_flight::stabilizers::copter::CopterStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimQuadPlane, airframes};
///
/// let airframe = airframes::quadplane();
/// let vehicle = SimQuadPlane::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     CopterGuidance::new(airframe.guidance),
///     CopterStabilizer::new(airframe.stabilizer),
///     airframe.mixer,
///     RecordingTelemetry::<16>::new(),
/// );
/// ```
pub fn quadplane() -> QuadPlaneAirframe {
    const MASS_KG: f32 = 4.5;
    const DIAGONAL_M: f32 = 0.35;
    let airplane = plane();
    let model = QuadPlaneModel {
        wing: FixedWingModel {
            mass_kg: MASS_KG,
            inertia_kgm2: aerosonde_inertia(MASS_KG, airplane.model.span_m),
            max_thrust_n: 1.2 * MASS_KG * GRAVITY,
            ..airplane.model
        },
        lift: LiftRotors {
            moment_arm_m: x_moment_arm_m(DIAGONAL_M),
            max_thrust_n: max_thrust_n(MASS_KG, 2.0),
            motor_time_constant_s: 0.05,
            yaw_torque_per_thrust_m: 0.022,
            drag_n_per_mps: 1.2,
        },
        pusher_slot: PUSHER_SLOT,
    };
    let mixer = hover_mixer();
    let defaults = CopterStabilizerConfig::default();
    let angle = Gains {
        p: ANGLE_P,
        ..defaults.attitude.roll_gains
    };
    let stabilizer = CopterStabilizerConfig {
        body_rate: BodyRateControllerConfig {
            // Sim autotune, from 3× and 10× core's rate P (from core's defaults the climb out
            // pitches the hover past settling), then rerun from its own result (#390).
            roll_gains: Gains {
                p: 1.50,
                i: 0.0,
                d: 0.0586,
            },
            pitch_gains: Gains {
                p: 1.31,
                i: 0.0,
                d: 0.0671,
            },
            ..defaults.body_rate
        },
        attitude: AttitudeControllerConfig {
            roll_gains: angle,
            pitch_gains: angle,
            ..defaults.attitude
        },
        ..defaults
    };
    let guidance = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            hover_thrust: hover_thrust(
                MASS_KG,
                model.lift.max_thrust_n,
                &mixer,
                &MixerConfig::default(),
                stabilizer.thrust_curve_exponent,
            ),
            ..CopterGuidanceConfig::default().altitude
        },
        ..CopterGuidanceConfig::default()
    };
    QuadPlaneAirframe {
        model,
        mixer,
        guidance,
        stabilizer,
        start: airplane.start,
    }
}

/// Roll and pitch angle P: the sim autotune's (#390).
const ANGLE_P: f32 = 9.0;

/// [`TableMixer::quad_x`]'s motor rows and [`TableMixer::plane`]'s servo rows.
fn hover_mixer() -> TableMixer<7> {
    let [front_left, front_right, back_left, back_right] = *TableMixer::quad_x().rows();
    let rows = [
        front_left,
        front_right,
        back_left,
        back_right,
        MixRow::servo(0, 1.0, 0.0, 0.0),
        MixRow::servo(1, 0.0, 1.0, 0.0),
        MixRow::servo(2, 0.0, 0.0, 1.0),
    ];
    TableMixer::new(rows, MixerConfig::default())
        // Two valid presets' rows: motors 0–3 and servos 0–2, each slot once.
        .unwrap_or_else(|error| unreachable!("quad_x's motors and plane's servos: {error}"))
}
