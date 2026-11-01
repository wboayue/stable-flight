//! [`PlaneRateController`]: the fixed-wing body-rate loop, PI plus feedforward and trim per
//! axis with airspeed scaling.

use crate::Vec3;

use super::{InvalidConfig, check, check_limit, rescale_integral};
use crate::math::{assert_dt, clamp_symmetric};

#[cfg(test)]
mod tests;

/// Gains for one axis of the [`PlaneRateController`], in normalized moment per rad/s of rate
/// error (`p`), per rad of accumulated error (`i`), and per rad/s of rate setpoint (`ff`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateGains {
    /// Proportional gain.
    pub p: f32,
    /// Integral gain.
    pub i: f32,
    /// Feedforward gain on the rate setpoint: the surface deflection a rate needs, which a
    /// wing holds against its damping.
    pub ff: f32,
}

/// Configures the [`PlaneRateController`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneRateControllerConfig {
    /// Roll-rate gains.
    pub roll: RateGains,
    /// Pitch-rate gains.
    pub pitch: RateGains,
    /// Yaw-rate gains.
    pub yaw: RateGains,
    /// Largest accumulated rate error per axis, in rad.
    pub max_integral: f32,
    /// The moment each axis holds at trim airspeed and zero rate error, roll (`x`), pitch
    /// (`y`), and yaw (`z`), normalized: what the wing needs in steady level flight (a nose-up
    /// pitch moment for a wing that pitches down), so neither the attitude loop's error nor the
    /// rate integral has to hold it.
    pub trim: Vec3,
}

impl Default for PlaneRateControllerConfig {
    /// PX4's fixed-wing rate defaults (`fw_rate_control_params.yaml`: roll P 0.05 I 0.1 FF 0.5,
    /// pitch P 0.08 I 0.1 FF 0.5, yaw P 0.05 FF 0.3, no trim), a starting point for an airframe
    /// to tune.
    fn default() -> Self {
        PlaneRateControllerConfig {
            roll: RateGains {
                p: 0.05,
                i: 0.1,
                ff: 0.5,
            },
            pitch: RateGains {
                p: 0.08,
                i: 0.1,
                ff: 0.5,
            },
            yaw: RateGains {
                p: 0.05,
                i: 0.0,
                ff: 0.3,
            },
            max_integral: 2.0,
            trim: Vec3::zeros(),
        }
    }
}

impl PlaneRateControllerConfig {
    /// `Ok` when every gain and `max_integral` is non-negative and finite, and each trim is in
    /// `[-1, 1]`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    ///
    /// # Examples
    ///
    /// ```
    /// use stable_flight::controllers::PlaneRateControllerConfig;
    ///
    /// assert!(PlaneRateControllerConfig::default().validate().is_ok());
    /// let mut bad = PlaneRateControllerConfig::default();
    /// bad.pitch.ff = -0.1;
    /// assert_eq!(bad.validate().unwrap_err().field, "pitch.ff");
    /// ```
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        let (r, p, y) = (self.roll, self.pitch, self.yaw);
        let fields = [
            ("roll.p", r.p),
            ("roll.i", r.i),
            ("roll.ff", r.ff),
            ("pitch.p", p.p),
            ("pitch.i", p.i),
            ("pitch.ff", p.ff),
            ("yaw.p", y.p),
            ("yaw.i", y.i),
            ("yaw.ff", y.ff),
            ("max_integral", self.max_integral),
        ];
        for (field, value) in fields {
            check_limit(field, value)?;
        }
        let trims = [
            ("trim.x", self.trim.x),
            ("trim.y", self.trim.y),
            ("trim.z", self.trim.z),
        ];
        for (field, value) in trims {
            check(field, (-1.0..=1.0).contains(&value), "in [-1, 1]")?;
        }
        Ok(())
    }

    fn axis_gains(&self) -> [RateGains; 3] {
        [self.roll, self.pitch, self.yaw]
    }
}

/// The fixed-wing body-rate loop: per axis, `s²·(P·e + I·∫e + trim) + s·FF·setpoint`, clamped
/// to `[-1, 1]`, where `s` is the airspeed scale (trim airspeed over current airspeed).
///
/// A surface's moment grows with dynamic pressure (airspeed squared), so the feedback terms
/// and the trim shrink by `s²` as the wing speeds up; the feedforward, a deflection that holds
/// a rate against the wing's damping (itself proportional to airspeed), by `s` (PX4's
/// `FixedwingRateControl::Run`, ArduPilot's `AP_FW_Controller::run_rate_control`). Holds only
/// its integral; the config is passed to every call.
///
/// # Examples
///
/// ```
/// use stable_flight::Vec3;
/// use stable_flight::controllers::{PlaneRateController, PlaneRateControllerConfig};
///
/// let config = PlaneRateControllerConfig::default();
/// let mut controller = PlaneRateController::default();
/// let roll = Vec3::new(0.5, 0.0, 0.0);
/// let at_trim = controller.control(&config, 0.004, roll, Vec3::zeros(), 1.0, true);
/// let mut fast = PlaneRateController::default();
/// let faster = fast.control(&config, 0.004, roll, Vec3::zeros(), 0.5, true);
/// assert!(faster.x < at_trim.x); // less deflection at twice trim airspeed
/// ```
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PlaneRateController {
    integrated_error: Vec3,
}

impl PlaneRateController {
    /// Normalized moments `[-1, 1]` toward `desired_radps` from `current_radps`.
    ///
    /// `scale` is the airspeed scale `s` (1 at trim airspeed). `integrate` is false below the
    /// wing's minimum airspeed, where the surfaces have little authority and the integral
    /// would only wind up: it is held. A non-finite setpoint or rate on an axis gives that
    /// axis only its trim and leaves its integral alone (input policy); a non-finite scale
    /// gives no moment.
    ///
    /// # Panics
    ///
    /// If `dt` is not positive and finite.
    pub fn control(
        &mut self,
        config: &PlaneRateControllerConfig,
        dt: f32,
        desired_radps: Vec3,
        current_radps: Vec3,
        scale: f32,
        integrate: bool,
    ) -> Vec3 {
        assert_dt(dt);
        if !scale.is_finite() {
            return Vec3::zeros();
        }
        let gains = config.axis_gains();
        let scale_squared = scale * scale;
        Vec3::from_fn(|axis, _| {
            let (desired, current) = (desired_radps[axis], current_radps[axis]);
            let error = desired - current;
            let trim = scale_squared * config.trim[axis];
            if !error.is_finite() {
                return clamp_symmetric(trim, 1.0);
            }
            if integrate {
                self.integrated_error[axis] = clamp_symmetric(
                    self.integrated_error[axis] + error * dt,
                    config.max_integral,
                );
            }
            let RateGains { p, i, ff } = gains[axis];
            let feedback = p * error + i * self.integrated_error[axis];
            clamp_symmetric(scale_squared * feedback + trim + scale * ff * desired, 1.0)
        })
    }

    /// Zeroes the integral.
    pub(crate) fn clear_integral(&mut self) {
        self.integrated_error = Vec3::zeros();
    }

    /// Rescales the integral for a change of integral gain from `old` to `new`, so the next
    /// tick's I term picks up where the last left off (`rescale_integral`).
    pub(crate) fn rescale_integral(
        &mut self,
        old: &PlaneRateControllerConfig,
        new: &PlaneRateControllerConfig,
    ) {
        let (old, new) = (old.axis_gains(), new.axis_gains());
        self.integrated_error = Vec3::from_fn(|axis, _| {
            rescale_integral(self.integrated_error[axis], old[axis].i, new[axis].i)
        });
    }
}
