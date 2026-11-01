//! The kind of vehicle the bridge reports to the ground station: see [`VehicleKind`].

use std::borrow::Cow;

use mavlink::common::{MavType, MavVtolState};
use stable_flight_link::stable_flight::view::tailsitter_rotation;
use stable_flight_link::stable_flight::{FlightMode, VehicleState};

#[cfg(test)]
mod tests;

/// The airframe behind the flight controller, as the ground station should see it: its
/// `MAV_TYPE`, whether it reports a VTOL state and takes transitions, and the frame its
/// attitude is shown in.
///
/// QGC shows its VTOL UI (state, transition button) for any VTOL `MAV_TYPE`, 19–25, whatever
/// the autopilot (`QGCMAVLink::vehicleClass`, `FlightModeIndicator.qml`'s
/// `vtolTransitionButton`). ArduPilot reports a plain quadplane as `FIXED_WING`
/// (`QuadPlane::get_mav_type`), which hides that UI, so the quadplane reports
/// `VTOL_FIXEDROTOR` instead.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, clap::ValueEnum)]
pub(crate) enum VehicleKind {
    /// A multicopter: `QUADROTOR`, no VTOL state.
    #[default]
    Copter,
    /// A twin-prop tailsitter (the eXplora): `VTOL_TAILSITTER_DUOROTOR`, as ArduPilot reports
    /// one; hover attitude shown in the hover view.
    Tailsitter,
    /// A lift+cruise VTOL (the Titan Cobra, the Alia 250): `VTOL_FIXEDROTOR`.
    Quadplane,
}

impl VehicleKind {
    /// The `MAV_TYPE` its heartbeat reports.
    pub(crate) fn mav_type(self) -> MavType {
        match self {
            VehicleKind::Copter => MavType::MAV_TYPE_QUADROTOR,
            VehicleKind::Tailsitter => MavType::MAV_TYPE_VTOL_TAILSITTER_DUOROTOR,
            VehicleKind::Quadplane => MavType::MAV_TYPE_VTOL_FIXEDROTOR,
        }
    }

    /// Whether it flies both regimes, so it reports a VTOL state and takes transitions.
    pub(crate) fn is_vtol(self) -> bool {
        self != VehicleKind::Copter
    }

    /// `EXTENDED_SYS_STATE.vtol_state` while it flies `mode`; undefined for a copter.
    pub(crate) fn vtol_state(self, mode: FlightMode) -> MavVtolState {
        if self.is_vtol() {
            stable_flight_mavlink::vtol_state(mode)
        } else {
            MavVtolState::MAV_VTOL_STATE_UNDEFINED
        }
    }

    /// `state` as the ground station should show it while the vehicle flies `mode`. A
    /// tailsitter hovering or heading to plane is shown in its hover view
    /// ([`tailsitter_rotation`]): nose-up hover sits at body pitch 90°, where the Euler angles
    /// trade roll against heading and jump. ArduPilot reports a tailsitter's `ATTITUDE` the same
    /// way, in its VTOL modes and while waiting on the forward transition's angle
    /// (`GCS_MAVLINK_Plane::send_attitude`, `Tailsitter_Transition::show_vtol_view`). Anything
    /// else is shown as is.
    pub(crate) fn shown(self, state: &VehicleState, mode: FlightMode) -> Cow<'_, VehicleState> {
        let hover = matches!(mode, FlightMode::Copter | FlightMode::ToPlane);
        if self == VehicleKind::Tailsitter && hover {
            Cow::Owned(state.rotate_body(&tailsitter_rotation()))
        } else {
            Cow::Borrowed(state)
        }
    }
}
