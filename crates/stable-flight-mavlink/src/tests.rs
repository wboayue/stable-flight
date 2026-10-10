use assert2::check;

use super::*;

#[test]
fn heartbeat_reports_the_mode_as_standard_flags_and_the_arm_state() {
    use MavModeFlag as F;

    let manual = F::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    let stabilize = F::MAV_MODE_FLAG_STABILIZE_ENABLED;
    let guided = F::MAV_MODE_FLAG_GUIDED_ENABLED;
    let auto = guided | F::MAV_MODE_FLAG_AUTO_ENABLED;
    // (mode, expected mode flags)
    let modes = [
        (ControlMode::Rate, manual),
        (ControlMode::Attitude, manual | stabilize),
        (ControlMode::Waypoint, stabilize | auto),
        (ControlMode::AbsoluteAttitude, stabilize | guided),
    ];
    // (armed, expected arm flag, expected status)
    let arm_states = [
        (false, F::empty(), MavState::MAV_STATE_STANDBY),
        (
            true,
            F::MAV_MODE_FLAG_SAFETY_ARMED,
            MavState::MAV_STATE_ACTIVE,
        ),
    ];

    for (mode, mode_flags) in modes {
        for (armed, arm_flag, status) in arm_states {
            let data = heartbeat(MavType::MAV_TYPE_VTOL_TAILSITTER_QUADROTOR, armed, mode);
            check!(
                data.base_mode == mode_flags | arm_flag,
                "{mode:?} armed={armed}"
            );
            check!(data.system_status == status, "{mode:?} armed={armed}");
            check!(data.custom_mode == 0);
            check!(data.mavtype == MavType::MAV_TYPE_VTOL_TAILSITTER_QUADROTOR);
            check!(data.autopilot == MavAutopilot::MAV_AUTOPILOT_GENERIC);
        }
    }
}
