use assert2::check;

use super::*;

#[test]
fn packs_flight_sw_version() {
    // (case, major, minor, patch, expected)
    let cases = [
        ("crate version", "0", "1", "0", 0x0001_0000),
        ("every part", "1", "2", "3", 0x0102_0300),
        ("unparsable part is 0", "1", "x", "3", 0x0100_0300),
    ];
    for (case, major, minor, patch, expected) in cases {
        let dev = FirmwareVersionType::FIRMWARE_VERSION_TYPE_DEV as u32;
        check!(
            flight_sw_version(major, minor, patch) == expected | dev,
            "{case}"
        );
    }
}

#[test]
fn heartbeat_reports_arm_state() {
    // (case, armed, base mode, system status)
    let cases = [
        (
            "armed",
            true,
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED,
            MavState::MAV_STATE_ACTIVE,
        ),
        (
            "disarmed",
            false,
            MavModeFlag::empty(),
            MavState::MAV_STATE_STANDBY,
        ),
    ];
    for (case, armed, base_mode, system_status) in cases {
        let MavMessage::HEARTBEAT(data) = heartbeat(armed, 7) else {
            panic!("{case}: not a heartbeat");
        };
        check!(data.base_mode == base_mode, "{case}");
        check!(data.system_status == system_status, "{case}");
        check!(data.custom_mode == 7, "{case}");
        check!(data.mavtype == MavType::MAV_TYPE_QUADROTOR, "{case}");
        check!(
            data.autopilot == MavAutopilot::MAV_AUTOPILOT_GENERIC,
            "{case}"
        );
    }
}
