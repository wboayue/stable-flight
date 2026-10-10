//! Builders shared by the bridge's unit tests.

use assert2::check;
use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::stable_flight::{ControlMode, VehicleState};
use stable_flight_link::{Downlink, ParamValue};
use stable_flight_mavlink::{DEFAULT_ORIGIN, FlatEarth};

/// A ground station's header.
pub(crate) const GCS: MavHeader = MavHeader {
    system_id: 255,
    component_id: 190,
    sequence: 0,
};

/// A flight-controller param.
pub(crate) fn param(name: &str, value: f32, index: u16, count: u16) -> ParamValue {
    ParamValue {
        name: name.try_into().unwrap(),
        value,
        index,
        count,
    }
}

/// `PARAM_SET` of `name` to the flight controller.
pub(crate) fn set(name: &str, value: f32, param_type: MavParamType) -> PARAM_SET_DATA {
    PARAM_SET_DATA {
        param_value: value,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
        param_type,
    }
}

/// `PARAM_REQUEST_READ` of `index` from the flight controller.
pub(crate) fn read_index(index: i16) -> PARAM_REQUEST_READ_DATA {
    PARAM_REQUEST_READ_DATA {
        param_index: index,
        target_system: 1,
        target_component: 1,
        param_id: "".into(),
    }
}

/// `PARAM_REQUEST_READ` of `name` from the flight controller.
pub(crate) fn read_name(name: &str) -> PARAM_REQUEST_READ_DATA {
    PARAM_REQUEST_READ_DATA {
        param_index: -1,
        target_system: 1,
        target_component: 1,
        param_id: name.into(),
    }
}

/// `(name, value, index, count)` of a `PARAM_VALUE`, checking it is `REAL32`.
pub(crate) fn param_value_of(msg: &MavMessage) -> Option<(String, f32, u16, u16)> {
    let MavMessage::PARAM_VALUE(data) = msg else {
        return None;
    };
    check!(data.param_type == MavParamType::MAV_PARAM_TYPE_REAL32);
    Some((
        data.param_id.to_str().unwrap().to_owned(),
        data.param_value,
        data.param_index,
        data.param_count,
    ))
}

/// The flight controller's heartbeat, in attitude mode.
pub(crate) fn heartbeat(armed: bool) -> Downlink {
    Downlink::Heartbeat {
        armed,
        mode: ControlMode::Attitude,
        link_ok: true,
    }
}

/// `PARAM_REQUEST_LIST` to the flight controller.
pub(crate) fn request_list() -> MavMessage {
    MavMessage::PARAM_REQUEST_LIST(PARAM_REQUEST_LIST_DATA {
        target_system: 1,
        target_component: 1,
    })
}

/// The projection around the default origin.
pub(crate) fn earth() -> FlatEarth {
    FlatEarth::new(DEFAULT_ORIGIN).expect("valid origin")
}

/// A state at `altitude_m` over the local origin.
pub(crate) fn at_altitude(altitude_m: f32) -> VehicleState {
    VehicleState {
        position_m: [0.0, 0.0, altitude_m].into(),
        ..Default::default()
    }
}

/// The local `z` (NED, down positive) of each `HOME_POSITION` in `msgs`.
pub(crate) fn home_zs(msgs: &[MavMessage]) -> Vec<f32> {
    msgs.iter()
        .filter_map(|m| match m {
            MavMessage::HOME_POSITION(d) => Some(d.z),
            _ => None,
        })
        .collect()
}

/// The throttle of each `VFR_HUD` in `msgs`.
pub(crate) fn throttles(msgs: &[MavMessage]) -> Vec<u16> {
    msgs.iter()
        .filter_map(|m| match m {
            MavMessage::VFR_HUD(d) => Some(d.throttle),
            _ => None,
        })
        .collect()
}

/// The landed state of each `EXTENDED_SYS_STATE` in `msgs`.
pub(crate) fn landed_states(msgs: &[MavMessage]) -> Vec<MavLandedState> {
    msgs.iter()
        .filter_map(|m| match m {
            MavMessage::EXTENDED_SYS_STATE(d) => Some(d.landed_state),
            _ => None,
        })
        .collect()
}
