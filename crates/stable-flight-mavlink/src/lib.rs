//! [![github]](https://github.com/wboayue/stable-flight)&ensp;[![crates-io]](https://crates.io/crates/stable-flight-mavlink)&ensp;[![license]](https://opensource.org/licenses/MIT)
//!
//! [github]: https://img.shields.io/badge/github-8da0cb?style=for-the-badge&labelColor=555555&logo=github
//! [crates-io]: https://img.shields.io/badge/crates.io-fc8d62?style=for-the-badge&labelColor=555555&logo=rust
//! [license]: https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge&labelColor=555555
//!
//! <br>
//!
//! MAVLink message builders for a [Stable Flight](https://github.com/wboayue/stable-flight)
//! flight controller: pure functions from core values to MAVLink payloads (`mavlink` crate,
//! `common` dialect), with no connection, clock, or scheduling. The companion bridge
//! (`stable-flight-bridge`) builds its messages here, so the encoding is tested without I/O.
//!
//! Each builder returns a message's payload (`*_DATA`); the caller wraps it in a `MavMessage`
//! and picks the header. Vehicle identity (e.g. the `MavType`) is a parameter, never assumed.
//!
//! What the state messages need beyond one `VehicleState` lives here too, so both callers derive
//! it the same way: a [`GeodeticOrigin`] and its [`FlatEarth`] projection for global position,
//! [`HomeTracker`] for home, [`LandDetector`] for the landed state, and [`GroundTracker`]
//! combining the two. They hold state across
//! updates but, like the builders, no clock or I/O.
//!
//! # Examples
//!
//! ```
//! use stable_flight::ControlMode;
//! use stable_flight_mavlink::heartbeat;
//! use stable_flight_mavlink::mavlink::common::{MavModeFlag, MavType};
//!
//! let data = heartbeat(MavType::MAV_TYPE_QUADROTOR, true, ControlMode::Attitude);
//! assert!(data.base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED));
//! ```

use mavlink::common::*;
use stable_flight::ControlMode;

/// The `mavlink` version the builders return types from.
pub use mavlink;

mod geo;
mod ground;
mod home;
mod land_detector;
mod saturation;
mod state;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
#[cfg(test)]
mod tests;

pub use geo::{DEFAULT_ORIGIN, FlatEarth, GeodeticOrigin, InvalidOrigin};
pub use ground::{GroundChanges, GroundTracker};
pub use home::HomeTracker;
pub use land_detector::LandDetector;
pub use saturation::{SATURATION_FLAGS, saturation};
pub use state::{
    attitude, extended_sys_state, global_position_int, home_position, local_position_ned, vfr_hud,
    vtol_state,
};

/// `HEARTBEAT` for a `vehicle_type` (e.g. `MAV_TYPE_QUADROTOR`) with a generic autopilot: the arm state
/// (`MAV_MODE_FLAG_SAFETY_ARMED`, `MAV_STATE_ACTIVE`/`STANDBY`) and the mode as standard
/// `base_mode` flags, from which ground stations name it (QGC shows e.g. "Manual,Stabilize").
/// No custom mode.
///
/// | Mode | Flags |
/// |---|---|
/// | [`ControlMode::Rate`] | `MANUAL_INPUT` |
/// | [`ControlMode::Attitude`] | `MANUAL_INPUT \| STABILIZE` |
/// | [`ControlMode::Waypoint`] | `STABILIZE \| GUIDED \| AUTO` |
/// | [`ControlMode::AbsoluteAttitude`] | `STABILIZE \| GUIDED` (offboard, not manual) |
/// | [`ControlMode::Land`] | `STABILIZE \| GUIDED \| AUTO`, as waypoints |
/// | [`ControlMode::AbsoluteRate`] | `GUIDED` (offboard rates: neither manual nor stabilized) |
///
/// A mode added to the core later reports no flags until it is mapped here.
///
/// # Examples
///
/// ```
/// use stable_flight::ControlMode;
/// use stable_flight_mavlink::heartbeat;
/// use stable_flight_mavlink::mavlink::common::{MavModeFlag, MavState, MavType};
///
/// let data = heartbeat(MavType::MAV_TYPE_QUADROTOR, false, ControlMode::Rate);
/// assert_eq!(data.base_mode, MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
/// assert_eq!(data.system_status, MavState::MAV_STATE_STANDBY);
/// ```
pub fn heartbeat(vehicle_type: MavType, armed: bool, mode: ControlMode) -> HEARTBEAT_DATA {
    let mut base_mode = match mode {
        ControlMode::Rate => MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        ControlMode::Attitude => {
            MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
        }
        ControlMode::Waypoint | ControlMode::Land => {
            MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
                | MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED
        }
        ControlMode::AbsoluteAttitude => {
            MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED | MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED
        }
        ControlMode::AbsoluteRate => MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED,
        // `ControlMode` is non-exhaustive: an unmapped mode reports none.
        _ => MavModeFlag::empty(),
    };
    base_mode.set(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED, armed);
    HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: vehicle_type,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode,
        system_status: if armed {
            MavState::MAV_STATE_ACTIVE
        } else {
            MavState::MAV_STATE_STANDBY
        },
        mavlink_version: 3,
    }
}
