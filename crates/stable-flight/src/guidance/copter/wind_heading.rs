//! The heading that turns a wingtip into the wind, from the hover wind estimate.

use core::f32::consts::PI;

use crate::estimation::wind::WindEstimate;
use crate::{Vec2, wrap_angle};

/// Angle, in radians (80°), between the heading and where the wind comes from: a wingtip
/// into the wind, 10° short of square so the wind keeps a little on the belly. Descending
/// 60 → 20 m in RealFlight's 2 m/s wind, the eXplora's pitch swung up to 9.3–12.7° off its
/// setpoint square on, and 4.9–6.0° at 78–80° (#470); at about 65° it pinned its tilt cap.
const FROM_WIND_RAD: f32 = 80.0 * PI / 180.0;

/// Most uncertainty in the wind's direction, in radians (15°), that still turns the heading:
/// the RealFlight eXplora held and landed with the wind 78–108° off its belly, and pinned its
/// tilt cap at about 65° (#470, #476).
const MAX_DIRECTION_SD_RAD: f32 = 15.0 * PI / 180.0;

/// Least change, in radians (10°), that moves a latched heading: the estimate's direction
/// wanders within 3.3° of RealFlight's wind (#473).
const RETARGET_DEADBAND_RAD: f32 = 10.0 * PI / 180.0;

/// Picks a heading that puts a wingtip into the wind: the hover view's `x` (a tailsitter's
/// belly) [`FROM_WIND_RAD`] off where the wind comes from, on the side nearer the heading
/// when first picked, and kept for the life of the value (its owner replaces it to forget
/// them). Allocation-free.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct WindHeading {
    /// `+1` puts the heading clockwise (north toward east) of where the wind blows toward,
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

    /// The heading [`FROM_WIND_RAD`] off where `wind` comes from, on the latched side (picked nearer
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
        Some(wrap_angle(toward_rad + side * (PI - FROM_WIND_RAD)))
    }
}

#[cfg(test)]
mod tests;
