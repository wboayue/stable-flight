//! Builds MAVLink message payloads from a tick's data. Pure functions: no connection, clock,
//! or scheduling, so each mapping is tested on its own.

use mavlink::common::*;

use stable_flight::{ControlOutput, VehicleState, wrap_angle};

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

/// `VehicleState` translation is north/east/**up**; MAVLink local NED is north/east/down.
pub(super) fn local_position_ned(
    state: &VehicleState,
    time_boot_ms: u32,
) -> LOCAL_POSITION_NED_DATA {
    LOCAL_POSITION_NED_DATA {
        time_boot_ms,
        x: state.position_m.x,
        y: state.position_m.y,
        z: -state.position_m.z,
        vx: state.velocity_mps.x,
        vy: state.velocity_mps.y,
        vz: -state.velocity_mps.z,
    }
}

/// Euler angles in radians (yaw wrapped to [-π, π]) and FRD body rates in rad/s, as the
/// frame contract already reports them.
pub(super) fn attitude(state: &VehicleState, time_boot_ms: u32) -> ATTITUDE_DATA {
    ATTITUDE_DATA {
        time_boot_ms,
        roll: state.roll_rad,
        pitch: state.pitch_rad,
        yaw: wrap_angle(state.heading_rad),
        rollspeed: state.angular_velocity_radps.x,
        pitchspeed: state.angular_velocity_radps.y,
        yawspeed: state.angular_velocity_radps.z,
    }
}

/// Throttle is the stabilizer's commanded thrust, so it is reported for every command kind.
/// There is no air-data source, so airspeed reports horizontal ground speed too. `alt` is
/// the state's altitude above its datum (sea level for RealFlight), as VFR_HUD expects.
pub(super) fn vfr_hud(control: &ControlOutput, state: &VehicleState) -> VFR_HUD_DATA {
    let ground_speed_mps = state.velocity_mps.xy().norm();
    VFR_HUD_DATA {
        airspeed: ground_speed_mps,
        groundspeed: ground_speed_mps,
        heading: heading_deg(state.heading_rad),
        throttle: throttle_percent(control.thrust),
        climb: state.velocity_mps.z,
        alt: state.position_m.z,
    }
}

/// Heading as VFR_HUD's whole degrees in 0..360, clockwise from north; NaN reports 0.
fn heading_deg(heading_rad: f32) -> i16 {
    // `as` saturates and maps NaN to 0; rounding up to 360 wraps to 0.
    (heading_rad.to_degrees().rem_euclid(360.0).round() as i16) % 360
}

/// Normalized thrust (0.0–1.0) as the 0–100 % throttle VFR_HUD expects; out-of-range clamps.
fn throttle_percent(thrust: f32) -> u16 {
    (thrust.clamp(0.0, 1.0) * 100.0).round() as u16
}
