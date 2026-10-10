//! Builder for the control loop's saturation flags as named values.

use mavlink::common::NAMED_VALUE_INT_DATA;
use stable_flight::Saturation;

#[cfg(test)]
mod tests;

/// Number of [`Saturation`] flags, one `NAMED_VALUE_INT` each.
pub const SATURATION_FLAGS: usize = 7;

/// One `NAMED_VALUE_INT` per [`Saturation`] flag, `1` if set and `0` if not, in this order:
///
/// | Name | Flag |
/// |---|---|
/// | `SAT_TILT` | `tilt` |
/// | `SAT_THR` | `thrust` |
/// | `SAT_ROLL`, `SAT_PITCH`, `SAT_YAW` | `moments` |
/// | `SAT_MIXTHR` | `mixer_thrust` |
/// | `SAT_MIXMOM` | `mixer_moments` |
///
/// Each name is its own entry in QGC's MAVLink Inspector, so each flag charts as its own trace.
///
/// # Examples
///
/// ```
/// use stable_flight::Saturation;
/// use stable_flight_mavlink::saturation;
///
/// let mut flags = Saturation::default();
/// flags.tilt = true;
/// let data = saturation(&flags, 1000);
/// assert_eq!((data[0].name.to_str(), data[0].value), (Ok("SAT_TILT"), 1));
/// assert_eq!((data[1].name.to_str(), data[1].value), (Ok("SAT_THR"), 0));
/// ```
pub fn saturation(
    flags: &Saturation,
    time_boot_ms: u32,
) -> [NAMED_VALUE_INT_DATA; SATURATION_FLAGS] {
    // `Saturation` is non-exhaustive: a flag added to the core isn't reported until mapped here
    // (the `maps_every_core_flag` test catches a new `bool` flag).
    let Saturation {
        tilt,
        thrust,
        moments: [roll, pitch, yaw],
        mixer_thrust,
        mixer_moments,
        ..
    } = *flags;
    [
        ("SAT_TILT", tilt),
        ("SAT_THR", thrust),
        ("SAT_ROLL", roll),
        ("SAT_PITCH", pitch),
        ("SAT_YAW", yaw),
        ("SAT_MIXTHR", mixer_thrust),
        ("SAT_MIXMOM", mixer_moments),
    ]
    .map(|(name, set)| NAMED_VALUE_INT_DATA {
        time_boot_ms,
        value: i32::from(set),
        name: name.into(),
    })
}
