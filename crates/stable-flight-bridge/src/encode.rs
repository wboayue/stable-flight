//! Builds the MAVLink payloads only the bridge sends, on the flight controller's behalf. Pure
//! functions: no connection, clock, or state. Each returns a payload (`*_DATA`) for the caller
//! to wrap, as `stable-flight-mavlink`'s builders do.

use mavlink::MavHeader;
use mavlink::common::*;
use stable_flight_link::ParamValue;

use crate::metadata::ComponentMetadata;

#[cfg(test)]
mod tests;

/// `PARAM_VALUE` for one flight-controller param: index and count pass through, and every
/// param is `REAL32`.
pub(crate) fn param_value(param: &ParamValue) -> PARAM_VALUE_DATA {
    PARAM_VALUE_DATA {
        param_value: param.value,
        param_count: param.count,
        param_index: param.index,
        param_id: param.name.as_str().into(),
        param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
    }
}

/// `AUTOPILOT_VERSION`: MAVLink 2 and C-cast `REAL32` params, with this bridge's version as the
/// flight software version.
pub(crate) fn autopilot_version() -> AUTOPILOT_VERSION_DATA {
    AUTOPILOT_VERSION_DATA {
        capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST,
        flight_sw_version: flight_sw_version(
            env!("CARGO_PKG_VERSION_MAJOR"),
            env!("CARGO_PKG_VERSION_MINOR"),
            env!("CARGO_PKG_VERSION_PATCH"),
        ),
        ..Default::default()
    }
}

/// MAVLink's packed version: major, minor, patch, then the release type (`FIRMWARE_VERSION_TYPE`).
/// A component that doesn't parse is 0.
fn flight_sw_version(major: &str, minor: &str, patch: &str) -> u32 {
    let part = |s: &str| s.parse::<u8>().map_or(0, u32::from);
    let release_type = FirmwareVersionType::FIRMWARE_VERSION_TYPE_DEV as u32;
    part(major) << 24 | part(minor) << 16 | part(patch) << 8 | release_type
}

/// `COMPONENT_METADATA` pointing to the general metadata file. `time_boot_ms` is 0: the link
/// carries no flight-controller clock, and the message is a one-off answer.
pub(crate) fn component_metadata(metadata: &ComponentMetadata) -> COMPONENT_METADATA_DATA {
    COMPONENT_METADATA_DATA {
        time_boot_ms: 0,
        file_crc: metadata.crc,
        uri: metadata.uri.into(),
    }
}

/// `MISSION_COUNT` of 0 for `mission_type`: there is no mission, fence, or rally plan.
pub(crate) fn empty_mission_count(
    mission_type: MavMissionType,
    target_system: u8,
    target_component: u8,
) -> MISSION_COUNT_DATA {
    MISSION_COUNT_DATA {
        count: 0,
        target_system,
        target_component,
        mission_type,
        opaque_id: 0,
    }
}

/// `COMMAND_ACK` with `result` for `command`, addressed back to its `sender`.
pub(crate) fn command_ack(
    sender: &MavHeader,
    command: MavCmd,
    result: MavResult,
) -> COMMAND_ACK_DATA {
    COMMAND_ACK_DATA {
        command,
        result,
        target_system: sender.system_id,
        target_component: sender.component_id,
        ..Default::default()
    }
}
