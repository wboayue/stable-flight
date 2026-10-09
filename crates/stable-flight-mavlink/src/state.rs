//! Builders for the vehicle-state messages: attitude, local and global position, home, VFR_HUD,
//! and the landed state.

use mavlink::common::*;
use stable_flight::{Vec3, VehicleState, wrap_angle};

use crate::geo::FlatEarth;

#[cfg(test)]
mod tests;

/// MAVLink's degE7 and mm/cm fixed-point scales.
const DEG_E7: f64 = 1e7;
const MM_PER_M: f64 = 1000.0;
const CM_PER_M: f32 = 100.0;

/// `EXTENDED_SYS_STATE` with `landed_state` (e.g. from a [`LandDetector`](crate::LandDetector)).
/// No VTOL modes: the vehicle reports as a multicopter.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::extended_sys_state;
/// use stable_flight_mavlink::mavlink::common::MavLandedState;
///
/// let data = extended_sys_state(MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// assert_eq!(data.landed_state, MavLandedState::MAV_LANDED_STATE_IN_AIR);
/// ```
pub fn extended_sys_state(landed_state: MavLandedState) -> EXTENDED_SYS_STATE_DATA {
    EXTENDED_SYS_STATE_DATA {
        vtol_state: MavVtolState::MAV_VTOL_STATE_UNDEFINED,
        landed_state,
    }
}

/// `GLOBAL_POSITION_INT`: position from `earth`, with `relative_alt` measured from
/// `home_altitude_m` (local `z`). Velocity is NED (down positive) in cm/s, NaN reporting zero.
/// `None` when the position isn't finite: latitude and longitude have no "unknown" value, and
/// NaN would report 0°, 0°.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight_mavlink::{DEFAULT_ORIGIN, FlatEarth, global_position_int};
///
/// let earth = FlatEarth::new(DEFAULT_ORIGIN)?;
/// let state = VehicleState { position_m: [0.0, 0.0, 12.0].into(), ..Default::default() };
/// let data = global_position_int(&earth, &state, 2.0, 0).expect("finite position");
/// assert_eq!(data.relative_alt, 10_000); // mm above home
/// # Ok::<(), stable_flight_mavlink::InvalidOrigin>(())
/// ```
pub fn global_position_int(
    earth: &FlatEarth,
    state: &VehicleState,
    home_altitude_m: f32,
    time_boot_ms: u32,
) -> Option<GLOBAL_POSITION_INT_DATA> {
    if !state.position_m.iter().all(|c| c.is_finite()) {
        return None;
    }
    let (lat_deg, lon_deg, alt_m) = earth.project(state.position_m.into());
    let v = state.velocity_mps;
    // `as` saturates and maps NaN to 0.
    let cm_per_s = |mps: f32| (mps * CM_PER_M).round() as i16;
    Some(GLOBAL_POSITION_INT_DATA {
        time_boot_ms,
        lat: (lat_deg * DEG_E7).round() as i32,
        lon: (lon_deg * DEG_E7).round() as i32,
        alt: (alt_m * MM_PER_M).round() as i32,
        relative_alt: (f64::from(state.position_m.z - home_altitude_m) * MM_PER_M).round() as i32,
        vx: cm_per_s(v.x),
        vy: cm_per_s(v.y),
        vz: cm_per_s(-v.z),
        hdg: heading_cdeg(state.heading_rad),
    })
}

/// `HOME_POSITION` at local `home_m` (north/east/up), as both a global position from `earth`
/// and local NED. No surface or heading estimate, so `q` is NaN (unknown); the approach vector
/// ends at home itself.
///
/// # Examples
///
/// ```
/// use stable_flight_mavlink::{DEFAULT_ORIGIN, FlatEarth, home_position};
///
/// let earth = FlatEarth::new(DEFAULT_ORIGIN)?;
/// let data = home_position(&earth, [0.0, 0.0, 3.0].into());
/// assert_eq!(data.z, -3.0); // NED: down positive
/// # Ok::<(), stable_flight_mavlink::InvalidOrigin>(())
/// ```
pub fn home_position(earth: &FlatEarth, home_m: Vec3) -> HOME_POSITION_DATA {
    let (lat_deg, lon_deg, alt_m) = earth.project(home_m.into());
    let [x, y, z] = [home_m.x, home_m.y, -home_m.z];
    HOME_POSITION_DATA {
        latitude: (lat_deg * DEG_E7).round() as i32,
        longitude: (lon_deg * DEG_E7).round() as i32,
        altitude: (alt_m * MM_PER_M).round() as i32,
        x,
        y,
        z,
        q: [f32::NAN; 4],
        approach_x: x,
        approach_y: y,
        approach_z: z,
        // Unknown: 0 per the MAVLink spec.
        time_usec: 0,
    }
}

/// `LOCAL_POSITION_NED`. `VehicleState` translation is north/east/**up**; MAVLink local NED is
/// north/east/down.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight_mavlink::local_position_ned;
///
/// let state = VehicleState { position_m: [1.0, 2.0, 10.0].into(), ..Default::default() };
/// assert_eq!(local_position_ned(&state, 0).z, -10.0);
/// ```
pub fn local_position_ned(state: &VehicleState, time_boot_ms: u32) -> LOCAL_POSITION_NED_DATA {
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

/// `ATTITUDE`: Euler angles in radians (yaw wrapped to [-π, π]) and FRD body rates in rad/s, as
/// the frame contract already reports them.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight_mavlink::attitude;
///
/// let state = VehicleState { roll_rad: 10_f32.to_radians(), ..Default::default() };
/// assert_eq!(attitude(&state, 0).roll, 10_f32.to_radians());
/// ```
pub fn attitude(state: &VehicleState, time_boot_ms: u32) -> ATTITUDE_DATA {
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

/// `VFR_HUD`. Throttle is the normalized commanded `thrust` (0.0–1.0, e.g. the stabilizer's
/// `ControlOutput::thrust`), so it is reported for every command kind; out-of-range clamps and
/// NaN reports 0. There is no air-data source, so airspeed reports horizontal ground speed too.
/// `alt` is the state's altitude above its datum (sea level for RealFlight), as VFR_HUD expects.
///
/// # Examples
///
/// ```
/// use stable_flight::VehicleState;
/// use stable_flight_mavlink::vfr_hud;
///
/// let state = VehicleState { velocity_mps: [3.0, 4.0, 1.0].into(), ..Default::default() };
/// let data = vfr_hud(&state, 0.5);
/// assert_eq!((data.groundspeed, data.throttle), (5.0, 50));
/// ```
pub fn vfr_hud(state: &VehicleState, thrust: f32) -> VFR_HUD_DATA {
    let ground_speed_mps = state.velocity_mps.xy().norm();
    VFR_HUD_DATA {
        airspeed: ground_speed_mps,
        groundspeed: ground_speed_mps,
        heading: heading_deg(state.heading_rad),
        throttle: throttle_percent(thrust),
        climb: state.velocity_mps.z,
        alt: state.position_m.z,
    }
}

/// Heading in degrees in [0, 360), clockwise from north; NaN stays NaN.
fn heading_from_north_deg(heading_rad: f32) -> f32 {
    heading_rad.to_degrees().rem_euclid(360.0)
}

/// Heading as VFR_HUD's whole degrees in 0..360; NaN reports 0 (VFR_HUD has no "unknown").
fn heading_deg(heading_rad: f32) -> i16 {
    // `as` saturates and maps NaN to 0; rounding up to 360 wraps to 0.
    (heading_from_north_deg(heading_rad).round() as i16) % 360
}

/// Heading as GLOBAL_POSITION_INT's centidegrees in 0..36000; a non-finite heading reports
/// `u16::MAX` (unknown).
fn heading_cdeg(heading_rad: f32) -> u16 {
    if !heading_rad.is_finite() {
        return u16::MAX;
    }
    // Rounding up to 36000 wraps to 0.
    ((heading_from_north_deg(heading_rad) * 100.0).round() as u16) % 36000
}

/// Normalized thrust (0.0–1.0) as the 0–100 % throttle VFR_HUD expects; out-of-range clamps.
fn throttle_percent(thrust: f32) -> u16 {
    (thrust.clamp(0.0, 1.0) * 100.0).round() as u16
}
