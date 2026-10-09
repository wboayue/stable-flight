//! Builders shared by the bridge's unit tests.

use assert2::check;
use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::stable_flight::ControlMode;
use stable_flight_link::{Downlink, ParamValue};

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
