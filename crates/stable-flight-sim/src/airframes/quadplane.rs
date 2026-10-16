//! The simulated lift+cruise VTOL: see [`quadplane`].

use stable_flight::GRAVITY;
use stable_flight::controllers::{
    AltitudeControllerConfig, AttitudeControllerConfig, BodyRateControllerConfig, Gains,
};
use stable_flight::guidance::copter::CopterGuidanceConfig;
use stable_flight::guidance::plane::PlaneGuidanceConfig;
use stable_flight::guidance::vtol::{
    ForwardThrustConfig, LiftTransitionConfig, QuadPlaneGuidanceConfig,
};
use stable_flight::mixers::{MixerConfig, TableMixer};
use stable_flight::stabilizers::copter::CopterStabilizerConfig;
use stable_flight::stabilizers::vtol::QuadPlaneStabilizerConfig;

use crate::fixed_wing::FixedWingModel;
use crate::quadplane_model::{LiftRotors, QuadPlaneModel};
use crate::start::Start;

use super::plane::{aerosonde_inertia, cruise_thrust, plane};
use super::{hover_thrust, max_thrust_n, x_moment_arm_m};
use crate::motors::QUADRATIC;

/// A simulated quadplane: its physics, the mixer that drives its lift rotors, pusher, and
/// surfaces, the guidance and stabilizer configs tuned for it, and a start in cruise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadPlaneAirframe {
    /// Physical parameters.
    pub model: QuadPlaneModel,
    /// [`TableMixer::quadplane`]: lift rotors on motors 0–3 in [`TableMixer::quad_x`]'s order,
    /// the pusher on motor 4, the ailerons, elevator, and rudder on servos 0–2 as on
    /// [`TableMixer::plane`].
    pub mixer: TableMixer<8>,
    /// Guidance configuration: hover, plane, and the transition between them.
    pub guidance: QuadPlaneGuidanceConfig,
    /// Stabilizer configuration: hover and plane.
    pub stabilizer: QuadPlaneStabilizerConfig,
    /// Level at [`plane`]'s 15 m/s trim airspeed, 100 m up over the origin, heading north.
    pub start: Start,
}

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
/// It flies core's quadplane guidance and stabilizer. The hover side has roll and pitch rate
/// and angle gains from the sim autotune (`StagePlan::copter_search`), yaw and heading core's
/// defaults, and the hover thrust worked out from the model. The plane side is [`plane`]'s
/// gains (the same wing and surfaces), with the cruise thrust worked out for this mass on the
/// pusher. Transitions fly core's defaults; they meet PX4's limits in
/// `tests/quadplane_transition.rs`. Copter flight hands its nose-down pitch past 3° to the
/// pusher ([`ForwardThrustConfig`], gain 1.08), so the wing isn't flown at
/// negative α (`tests/quadplane_waypoints.rs`).
///
/// # Examples
///
/// ```
/// use stable_flight::FlightLoop;
/// use stable_flight::guidance::vtol::QuadPlaneGuidance;
/// use stable_flight::stabilizers::vtol::QuadPlaneStabilizer;
/// use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
/// use stable_flight_sim::{SimQuadPlane, airframes};
///
/// let airframe = airframes::quadplane();
/// let vehicle = SimQuadPlane::new(airframe.model, &airframe.mixer).expect("valid airframe");
/// let flight_loop = FlightLoop::new(
///     ConstantCommander::default(),
///     vehicle,
///     QuadPlaneGuidance::new(airframe.guidance),
///     QuadPlaneStabilizer::new(airframe.stabilizer),
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
            thrust_exponent: QUADRATIC,
            motor_time_constant_s: 0.05,
            yaw_torque_per_thrust_m: 0.022,
            drag_n_per_mps: 1.2,
        },
    };
    let mixer = TableMixer::quadplane();
    let defaults = CopterStabilizerConfig::default();
    let angle = Gains {
        p: ANGLE_P,
        ..defaults.attitude.roll_gains
    };
    let hover_stabilizer = CopterStabilizerConfig {
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
    let mixer_config = MixerConfig::default();
    let hover = CopterGuidanceConfig {
        altitude: AltitudeControllerConfig {
            hover_thrust: hover_thrust(
                MASS_KG,
                model.lift.max_thrust_n,
                &mixer,
                &mixer_config,
                QUADRATIC,
                hover_stabilizer.thrust_curve_exponent,
            ),
            ..CopterGuidanceConfig::default().altitude
        },
        ..CopterGuidanceConfig::default()
    };
    let plane_stabilizer = airplane.stabilizer;
    let plane_guidance = PlaneGuidanceConfig {
        cruise_thrust: cruise_thrust(
            &model.wing,
            &mixer,
            &mixer_config,
            plane_stabilizer.thrust_curve_exponent,
            plane_stabilizer.trim_airspeed_mps,
        ),
        ..airplane.guidance
    };
    QuadPlaneAirframe {
        model,
        mixer,
        guidance: QuadPlaneGuidanceConfig {
            hover,
            plane: plane_guidance,
            // A share of the full pusher, as the transitions' PX4 limits were met with
            // (`tests/quadplane_transition.rs`, #391).
            transition: LiftTransitionConfig {
                forward_thrust: TRANSITION_PUSHER,
                pusher_slew_per_s: TRANSITION_PUSHER
                    * LiftTransitionConfig::default().pusher_slew_per_s,
                ..LiftTransitionConfig::default()
            },
            forward: ForwardThrustConfig {
                gain: FORWARD_THRUST_GAIN,
                ..ForwardThrustConfig::default()
            },
        },
        stabilizer: QuadPlaneStabilizerConfig {
            hover: hover_stabilizer,
            plane: plane_stabilizer,
        },
        start: airplane.start,
    }
}

/// Roll and pitch angle P: the sim autotune's (#390).
const ANGLE_P: f32 = 9.0;

/// The most of the full pusher the forward transition flies, and the share of core's slew it
/// rises at: the pusher commands the transition tests are measured on (#391, #448).
const TRANSITION_PUSHER: f32 = 0.9;

/// Forward thrust gain, in shares of the full pusher per unit of `tan(pitch)`: 1.08 holds a
/// hover and flies upwind at 5 m/s in an 11 m/s headwind and balloons 4 m at most on an
/// uncapped 100 m leg. 0.75 (0.9 of ArduPilot's weight-over-thrust rule, 0.83,
/// `ArduPlane/quadplane.cpp:512`) caps the pusher at 0.38 under the 27° tilt cap and stalls
/// upwind at 0.6 m/s; 1.8 (0.9 of its default 2, `:516`) balloons 16 m (#427, #448).
const FORWARD_THRUST_GAIN: f32 = 1.08;
