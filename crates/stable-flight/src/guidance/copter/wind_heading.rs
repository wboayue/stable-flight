//! The heading that turns a wingtip square into the wind, from the hover wind estimate.

use core::f32::consts::FRAC_PI_2;

use crate::estimation::wind::WindEstimate;
use crate::{Vec2, wrap_angle};

/// Most uncertainty in the wind's direction, in radians (15°), that still turns the heading:
/// the RealFlight eXplora held and landed with the wind 81–108° off its belly, and pinned its
/// tilt cap at about 65° (#470, #476).
const MAX_DIRECTION_SD_RAD: f32 = 15.0 * core::f32::consts::PI / 180.0;

/// Least change, in radians (10°), that moves a latched heading: the estimate's direction
/// wanders within 3.3° of RealFlight's wind (#473).
const RETARGET_DEADBAND_RAD: f32 = 10.0 * core::f32::consts::PI / 180.0;

/// Picks a heading that puts the wind square on a wingtip: the hover view's `x` (a
/// tailsitter's belly) 90° off the wind, on the side nearer the heading when first picked,
/// and kept for the life of the value (its owner replaces it to forget them). Allocation-free.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct WindHeading {
    /// `+1` puts the heading 90° clockwise (north toward east) of where the wind blows toward,
    /// `-1` counterclockwise. `None` until picked.
    side: Option<f32>,
    /// The latched heading, in radians from north; `None` until the wind first allows one.
    target_rad: Option<f32>,
}

impl WindHeading {
    /// The heading to turn to, in radians from north: the latched heading, moved when the
    /// wind's wingtip heading differs from it by more than [`RETARGET_DEADBAND_RAD`]. The wind
    /// moves it only when it is at least `min_speed_mps` and its direction known within
    /// [`MAX_DIRECTION_SD_RAD`]; otherwise the latched heading stands. `None` with
    /// `min_speed_mps` 0 (off), or before the wind first allowed a heading.
    pub(crate) fn target_rad(
        &mut self,
        min_speed_mps: f32,
        wind: Option<&WindEstimate>,
        heading_rad: f32,
    ) -> Option<f32> {
        // False for NaN: off.
        let on = min_speed_mps > 0.0;
        if !on {
            return None;
        }
        if let Some(wingtip_rad) =
            wind.and_then(|w| self.wingtip_rad(min_speed_mps, w, heading_rad))
        {
            let moved = self
                .target_rad
                .is_none_or(|t| wrap_angle(wingtip_rad - t).abs() > RETARGET_DEADBAND_RAD);
            if moved {
                self.target_rad = Some(wingtip_rad);
            }
        }
        self.target_rad
    }

    /// The heading 90° off where `wind` blows toward, on the latched side (picked nearer
    /// `heading_rad` the first time); `None` for a wind slower than `min_speed_mps` or of
    /// uncertain direction, or a non-finite one.
    fn wingtip_rad(
        &mut self,
        min_speed_mps: f32,
        wind: &WindEstimate,
        heading_rad: f32,
    ) -> Option<f32> {
        let velocity_mps = wind.velocity_mps;
        let speed_mps = velocity_mps.norm();
        // `>=` and `<=` are false for NaN: a non-finite wind turns nothing.
        let fast = speed_mps >= min_speed_mps;
        if !fast {
            return None;
        }
        let across = Vec2::new(-velocity_mps.y, velocity_mps.x) / speed_mps;
        // Small angles: the spread across the wind over its speed is the direction's spread.
        let known = wind.sd_along_mps(across) <= speed_mps * MAX_DIRECTION_SD_RAD;
        if !known {
            return None;
        }
        let toward_rad = libm::atan2f(velocity_mps.y, velocity_mps.x);
        let side = *self.side.get_or_insert_with(|| {
            let off_rad = wrap_angle(heading_rad - toward_rad);
            // A NaN heading picks +1.
            if off_rad < 0.0 { -1.0 } else { 1.0 }
        });
        Some(wrap_angle(toward_rad + side * FRAC_PI_2))
    }
}

#[cfg(test)]
mod tests;
