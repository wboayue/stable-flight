use assert2::check;

use super::*;
use crate::test_support::hovering;

#[test]
fn each_kind_reports_its_mav_type_and_vtol_state() {
    use MavVtolState as V;
    // (kind, MAV_TYPE, VTOL, vtol_state in plane)
    let rows = [
        (
            VehicleKind::Copter,
            MavType::MAV_TYPE_QUADROTOR,
            false,
            V::MAV_VTOL_STATE_UNDEFINED,
        ),
        (
            VehicleKind::Tailsitter,
            MavType::MAV_TYPE_VTOL_TAILSITTER_DUOROTOR,
            true,
            V::MAV_VTOL_STATE_FW,
        ),
        (
            VehicleKind::Quadplane,
            MavType::MAV_TYPE_VTOL_FIXEDROTOR,
            true,
            V::MAV_VTOL_STATE_FW,
        ),
    ];
    for (kind, mav_type, vtol, plane) in rows {
        check!(kind.mav_type() == mav_type, "{kind:?}");
        check!(kind.is_vtol() == vtol, "{kind:?}");
        check!(kind.vtol_state(FlightMode::Plane) == plane, "{kind:?}");
    }
}

#[test]
fn a_hovering_tailsitter_shows_a_steady_attitude_and_heading() {
    // Around body pitch 90°, body Euler angles swing roll and heading against each other; the
    // hover view keeps both steady.
    for heading_deg in [0.0_f32, 45.0, 135.0, -100.0] {
        for pitch_deg in [89.0_f32, 89.9, 90.0, 90.1, 91.0] {
            let state = hovering(heading_deg, pitch_deg);
            let shown = VehicleKind::Tailsitter.shown(&state, FlightMode::Copter);
            let case = format!("heading {heading_deg}°, pitch {pitch_deg}°");
            check!(
                shown.roll_rad.abs() < 1e-4,
                "{case}: roll {}",
                shown.roll_rad
            );
            check!(
                (shown.pitch_rad.to_degrees() - (pitch_deg - 90.0)).abs() < 1e-2,
                "{case}: pitch {}",
                shown.pitch_rad.to_degrees()
            );
            let heading_error_deg = stable_flight_link::stable_flight::wrap_angle(
                shown.heading_rad - heading_deg.to_radians(),
            )
            .to_degrees();
            check!(heading_error_deg.abs() < 1e-2, "{case}: heading");
        }
    }
}

#[test]
fn only_a_tailsitter_in_hover_or_heading_to_plane_is_shown_in_the_hover_view() {
    let state = hovering(30.0, 90.0);
    // (kind, mode, shown in the hover view)
    let rows = [
        (VehicleKind::Tailsitter, FlightMode::Copter, true),
        (VehicleKind::Tailsitter, FlightMode::ToPlane, true),
        (VehicleKind::Tailsitter, FlightMode::Plane, false),
        (VehicleKind::Tailsitter, FlightMode::ToCopter, false),
        (VehicleKind::Quadplane, FlightMode::Copter, false),
        (VehicleKind::Copter, FlightMode::Copter, false),
    ];
    let view = state.rotate_body(&tailsitter_rotation());
    for (kind, mode, in_view) in rows {
        let shown = kind.shown(&state, mode);
        let expected = if in_view { &view } else { &state };
        check!(*shown == *expected, "{kind:?} in {mode:?}");
    }
}
