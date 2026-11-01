//! The heading that turns a wingtip into the wind, from the hover wind estimate.

use super::CopterGuidanceConfig;
use crate::estimation::wind::WindEstimate;
use crate::{Vec2, wrap_angle};

/// Picks a heading that puts a wingtip into the wind: the hover view's `x` (a tailsitter's
/// belly) [`wind_heading_from_wind_rad`](CopterGuidanceConfig::wind_heading_from_wind_rad)
/// off where the wind comes from, on the side nearer the heading
/// when first picked, and kept for the life of the value (its owner replaces it to forget
/// them). Allocation-free.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct WindHeading {
    /// `+1` puts the heading clockwise (north toward east) of where the wind blows toward,
    /// `-1` counterclockwise. `None` until picked.
    side: Option<f32>,
    /// The latched heading, in radians from north; `None` until the wind first allows one.
    target_rad: Option<f32>,
    /// The estimate last weighed: it changes every update period (0.2 s), so the ticks in
    /// between reuse the latched heading without its trig.
    weighed: Option<WindEstimate>,
}

impl WindHeading {
    /// The heading to turn to, in radians from north: the latched heading, moved when the
    /// wind's wingtip heading differs from it by more than
    /// [`wind_heading_deadband_rad`](CopterGuidanceConfig::wind_heading_deadband_rad). The
    /// wind moves it only when it is at least
    /// [`wind_heading_min_mps`](CopterGuidanceConfig::wind_heading_min_mps) and its direction
    /// known within [`wind_heading_max_sd_rad`](CopterGuidanceConfig::wind_heading_max_sd_rad);
    /// otherwise the latched heading stands. An estimate already weighed is skipped, so a new
    /// config applies from the next estimate. `None` with `wind_heading_min_mps` 0 (off), or
    /// before the wind first allowed a heading.
    pub(crate) fn target_rad(
        &mut self,
        config: &CopterGuidanceConfig,
        wind: Option<&WindEstimate>,
        heading_rad: f32,
    ) -> Option<f32> {
        // False for NaN: off.
        let on = config.wind_heading_min_mps > 0.0;
        if !on {
            return None;
        }
        let Some(wind) = wind.filter(|w| self.weighed.as_ref() != Some(w)) else {
            return self.target_rad;
        };
        self.weighed = Some(*wind);
        if let Some(wingtip_rad) = self.wingtip_rad(config, wind, heading_rad) {
            let deadband_rad = config.wind_heading_deadband_rad;
            let moved = self
                .target_rad
                .is_none_or(|t| wrap_angle(wingtip_rad - t).abs() > deadband_rad);
            if moved {
                self.target_rad = Some(wingtip_rad);
            }
        }
        self.target_rad
    }

    /// The heading `wind_heading_from_wind_rad` off where `wind` comes from, on the latched
    /// side (picked nearer `heading_rad` the first time); `None` for a wind slower than
    /// `wind_heading_min_mps` or of uncertain direction, or a non-finite one.
    fn wingtip_rad(
        &mut self,
        config: &CopterGuidanceConfig,
        wind: &WindEstimate,
        heading_rad: f32,
    ) -> Option<f32> {
        let velocity_mps = wind.velocity_mps;
        let speed_mps = velocity_mps.norm();
        // `>=` and `<=` are false for NaN: a non-finite wind turns nothing.
        let fast = speed_mps >= config.wind_heading_min_mps;
        if !fast {
            return None;
        }
        let across = Vec2::new(-velocity_mps.y, velocity_mps.x) / speed_mps;
        // Small angles: the spread across the wind over its speed is the direction's spread.
        let known = wind.sd_along_mps(across) <= speed_mps * config.wind_heading_max_sd_rad;
        if !known {
            return None;
        }
        let toward_rad = libm::atan2f(velocity_mps.y, velocity_mps.x);
        let side = *self.side.get_or_insert_with(|| {
            let off_rad = wrap_angle(heading_rad - toward_rad);
            // A NaN heading picks +1.
            if off_rad < 0.0 { -1.0 } else { 1.0 }
        });
        Some(wrap_angle(
            toward_rad + side * (core::f32::consts::PI - config.wind_heading_from_wind_rad),
        ))
    }
}

#[cfg(test)]
mod tests;
