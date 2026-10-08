//! Builds MAVLink message payloads from a tick's data. Pure functions: no connection, clock,
//! or scheduling, so each mapping is tested on its own.

use mavlink::common::*;

use stable_flight::{ControlOutput, VehicleState};

#[cfg(test)]
mod tests;

pub(super) fn heartbeat() -> HEARTBEAT_DATA {
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    }
}

/// Hard-coded until a geodetic origin exists (#97).
pub(super) fn global_position(time_boot_ms: u32) -> GLOBAL_POSITION_INT_DATA {
    GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: 371234567,      // 37.1234567° N
        lon: -1221234567,    // 122.1234567° W
        alt: 100000,         // 100m above sea level (mm)
        relative_alt: 50000, // 50m above home (mm)
        vx: 100,             // 1 m/s north (cm/s)
        vy: 50,              // 0.5 m/s east (cm/s)
        vz: 10,              // 0.1 m/s down (cm/s)
        hdg: 9000,           // 90° heading (degrees * 100)
    }
}

pub(super) fn local_position_ned(
    state: &VehicleState,
    time_boot_ms: u32,
) -> LOCAL_POSITION_NED_DATA {
    LOCAL_POSITION_NED_DATA {
        time_boot_ms,
        x: state.position_m.x,    // North position (meters)
        y: state.position_m.y,    // East position (meters)
        z: state.position_m.z,    // Down position (meters)
        vx: state.velocity_mps.x, // North velocity (m/s)
        vy: state.velocity_mps.y, // East velocity (m/s)
        vz: state.velocity_mps.z, // Down velocity (m/s)
    }
}

pub(super) fn attitude(state: &VehicleState, time_boot_ms: u32) -> ATTITUDE_DATA {
    ATTITUDE_DATA {
        time_boot_ms,
        roll: state.attitude.i,
        pitch: state.attitude.j,
        yaw: state.attitude.k,
        rollspeed: state.angular_velocity.x,
        pitchspeed: state.angular_velocity.y,
        yawspeed: state.angular_velocity.z,
    }
}

/// Throttle is the stabilizer's commanded thrust, so it is reported for every command kind.
pub(super) fn vfr_hud(control: &ControlOutput, state: &VehicleState) -> VFR_HUD_DATA {
    VFR_HUD_DATA {
        airspeed: state.velocity_mps.x,
        groundspeed: state.velocity_mps.y,
        heading: state.attitude.k as i16,
        throttle: throttle_percent(control.thrust),
        climb: state.velocity_mps.z,
        alt: state.position_m.z, // Altitude (meters)
    }
}

/// Normalized thrust (0.0–1.0) as the 0–100 % throttle VFR_HUD expects; out-of-range clamps.
fn throttle_percent(thrust: f32) -> u16 {
    (thrust.clamp(0.0, 1.0) * 100.0).round() as u16
}
