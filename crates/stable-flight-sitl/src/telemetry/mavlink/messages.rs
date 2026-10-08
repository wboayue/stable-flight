//! Builds MAVLink message payloads from a tick's data. Pure functions: no connection, clock,
//! or scheduling, so each mapping is tested on its own.

use mavlink::common::*;

use stable_flight::{Command, ControlInput, ControlOutput, VehicleState, wrap_angle};

use super::GeodeticOrigin;

#[cfg(test)]
mod tests;

/// Earth's mean radius, for a spherical flat-earth projection: its error grows with distance
/// from the origin but stays small at SITL ranges (a few hundred meters).
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// MAVLink's degE7 and mm/cm fixed-point scales.
const DEG_E7: f64 = 1e7;
const MM_PER_M: f64 = 1000.0;
const CM_PER_M: f32 = 100.0;

/// Reports the arm state (`MAV_MODE_FLAG_SAFETY_ARMED`, `MAV_STATE_ACTIVE`/`STANDBY`) and the
/// command kind as standard `base_mode` flags. No custom mode: with a generic autopilot,
/// ground stations name the mode from these flags (QGC shows e.g. "Manual,Stabilize").
pub(super) fn heartbeat(command: &Command) -> HEARTBEAT_DATA {
    let mut base_mode = match command.input {
        ControlInput::Rate(_) => MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        ControlInput::Attitude(_) => {
            MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
        }
        ControlInput::Waypoint(_) => {
            MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED
        }
        // `ControlInput` is non-exhaustive: an unmapped kind reports no mode.
        _ => MavModeFlag::empty(),
    };
    base_mode.set(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED, command.arm);
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode,
        system_status: if command.arm {
            MavState::MAV_STATE_ACTIVE
        } else {
            MavState::MAV_STATE_STANDBY
        },
        mavlink_version: 3,
    }
}

/// Projects local north/east/up positions onto latitude, longitude, and altitude around a
/// [`GeodeticOrigin`], treating the earth as flat near it. Meters-per-degree are computed
/// once here so a projection costs no trig.
#[derive(Debug, Clone, Copy)]
pub(super) struct FlatEarth {
    origin: GeodeticOrigin,
    m_per_deg_lat: f64,
    m_per_deg_lon: f64,
}

impl FlatEarth {
    pub(super) fn new(origin: GeodeticOrigin) -> Self {
        let m_per_deg_lat = EARTH_RADIUS_M.to_radians();
        FlatEarth {
            origin,
            m_per_deg_lat,
            m_per_deg_lon: m_per_deg_lat * origin.latitude_deg.to_radians().cos(),
        }
    }

    /// Latitude and longitude in degrees, and altitude in meters, of local `position_m`.
    fn project(&self, position_m: [f32; 3]) -> (f64, f64, f64) {
        let [north, east, up] = position_m.map(f64::from);
        (
            self.origin.latitude_deg + north / self.m_per_deg_lat,
            self.origin.longitude_deg + east / self.m_per_deg_lon,
            self.origin.altitude_m + up,
        )
    }
}

/// Position from `earth`, with `relative_alt` measured from `home_altitude_m` (local `z`).
/// Velocity is NED (down positive) in cm/s, NaN reporting zero.
pub(super) fn global_position_int(
    earth: &FlatEarth,
    state: &VehicleState,
    home_altitude_m: f32,
    time_boot_ms: u32,
) -> GLOBAL_POSITION_INT_DATA {
    let (lat_deg, lon_deg, alt_m) = earth.project(state.position_m.into());
    let v = state.velocity_mps;
    // `as` saturates and maps NaN to 0.
    let cm_per_s = |mps: f32| (mps * CM_PER_M).round() as i16;
    GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: (lat_deg * DEG_E7).round() as i32,
        lon: (lon_deg * DEG_E7).round() as i32,
        alt: (alt_m * MM_PER_M).round() as i32,
        relative_alt: (f64::from(state.position_m.z - home_altitude_m) * MM_PER_M).round() as i32,
        vx: cm_per_s(v.x),
        vy: cm_per_s(v.y),
        vz: cm_per_s(-v.z),
        hdg: heading_cdeg(state.heading_rad),
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

/// Heading as GLOBAL_POSITION_INT's centidegrees in 0..36000; a non-finite heading reports
/// `u16::MAX` (unknown).
fn heading_cdeg(heading_rad: f32) -> u16 {
    if !heading_rad.is_finite() {
        return u16::MAX;
    }
    // Rounding up to 36000 wraps to 0.
    ((heading_rad.to_degrees().rem_euclid(360.0) * 100.0).round() as u16) % 36000
}

/// Normalized thrust (0.0–1.0) as the 0–100 % throttle VFR_HUD expects; out-of-range clamps.
fn throttle_percent(thrust: f32) -> u16 {
    (thrust.clamp(0.0, 1.0) * 100.0).round() as u16
}
