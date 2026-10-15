//! [`ForwardThrust`]: a quadplane's pusher taking the nose-down part of copter flight, so the
//! wing isn't flown at negative α.

use core::f32::consts::FRAC_PI_4;

use crate::Configurable;
use crate::controllers::{InvalidConfig, check, check_limit};
use crate::math::DEG;

mod params;
#[cfg(test)]
mod tests;

/// [`ForwardThrust`] settings: ArduPilot's `Q_FWD_THR_GAIN` and `Q_FWD_PIT_LIM`
/// (`ArduPlane/quadplane.cpp:516`, `:525`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ForwardThrustConfig {
    /// Pusher linear thrust per unit of `tan(pitch)` the hover asks for nose down; `0` (the
    /// default) is off. ArduPilot sets it to the weight over the pusher's most thrust
    /// (`quadplane.cpp:512`). Non-negative and finite.
    pub gain: f32,
    /// The most nose-down pitch copter flight keeps while forward thrust is on, in radians, in
    /// `[0, π/2)`.
    pub pitch_limit_rad: f32,
}

impl ForwardThrustConfig {
    /// `Ok` when the gain is non-negative and finite and the pitch limit in `[0, π/2)`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_limit("gain", self.gain)?;
        check(
            "pitch_limit_rad",
            (0.0..core::f32::consts::FRAC_PI_2).contains(&self.pitch_limit_rad),
            "in [0, π/2) rad",
        )
    }
}

impl Default for ForwardThrustConfig {
    /// Off, as ArduPilot (`Q_FWD_THR_USE`, `quadplane.cpp:532`) and PX4 (`VT_FWD_THRUST_EN`,
    /// `standard_params.yaml:5`) ship it, with ArduPilot's 3° pitch limit.
    fn default() -> Self {
        ForwardThrustConfig {
            gain: 0.0,
            pitch_limit_rad: 3.0 * DEG,
        }
    }
}

/// The pusher's share of copter flight on a quadplane: the hover's pitch held to
/// [`pitch_limit_rad`](ForwardThrustConfig::pitch_limit_rad) nose down, and the pusher's
/// thrust for what the hover asked, `gain · tan(nose-down pitch)` up to 1 with the pitch
/// taken to at most 45° (ArduPilot's `assign_tilt_to_fwd_thr`, `ArduPlane/quadplane.cpp:2942-3058`,
/// ArduPilot `368dc0c428`). Nose up is left alone: the pusher can't push back. A NaN pitch
/// gives no thrust and stays NaN, which the stabilizer flies as level.
///
/// Without it, a quadplane flying as a copter tilts nose down to move ahead or into wind, and
/// at speed the wing at negative α pushes it down (#427).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::vtol::{ForwardThrust, ForwardThrustConfig};
///
/// let forward = ForwardThrust::new(ForwardThrustConfig { gain: 1.0, ..Default::default() });
/// let split = forward.split(-20.0_f32.to_radians());
/// assert!((split.pitch_rad - -3.0_f32.to_radians()).abs() < 1e-6);
/// assert!((split.thrust - 20.0_f32.to_radians().tan()).abs() < 1e-6);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ForwardThrust {
    config: ForwardThrustConfig,
}

/// A hover pitch split between the tilt and the pusher.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ForwardSplit {
    /// The pitch the vehicle flies, in radians.
    pub pitch_rad: f32,
    /// The pusher's linear thrust, `[0, 1]`.
    pub thrust: f32,
}

impl ForwardThrust {
    /// Forward thrust with `config`.
    ///
    /// # Panics
    ///
    /// If `config` is invalid ([`ForwardThrustConfig::validate`]).
    pub fn new(config: ForwardThrustConfig) -> Self {
        if let Err(error) = config.validate() {
            panic!("invalid ForwardThrustConfig: {error}");
        }
        ForwardThrust { config }
    }

    /// On: a positive gain.
    pub fn enabled(&self) -> bool {
        self.config.gain > 0.0
    }

    /// Splits the hover's `pitch_rad` between the tilt and the pusher; off, the pitch passes
    /// through with no thrust.
    pub fn split(&self, pitch_rad: f32) -> ForwardSplit {
        if !self.enabled() || pitch_rad.is_nan() {
            return ForwardSplit {
                pitch_rad,
                thrust: 0.0,
            };
        }
        let nose_down_rad = (-pitch_rad).clamp(0.0, FRAC_PI_4);
        ForwardSplit {
            pitch_rad: pitch_rad.max(-self.config.pitch_limit_rad),
            thrust: (self.config.gain * libm::tanf(nose_down_rad)).min(1.0),
        }
    }
}

impl Configurable for ForwardThrust {
    type Config = ForwardThrustConfig;

    fn config(&self) -> &ForwardThrustConfig {
        &self.config
    }

    /// Validates with [`ForwardThrustConfig::validate`]; applies from the next tick.
    fn set_config(&mut self, config: ForwardThrustConfig) -> Result<(), InvalidConfig> {
        config.validate()?;
        self.config = config;
        Ok(())
    }
}
