//! Altitude-hold flight ([`ControlInput::AltitudeHold`](crate::ControlInput::AltitudeHold)):
//! the climb stick sets a climb rate, an altitude target moves at it, and the altitude loop
//! flies the target, so a centered stick holds the height. Its settings: [`AltitudeHoldConfig`].

use crate::commanders::TouchdownDetector;
use crate::controllers::{AltitudeTarget, InvalidConfig, check, check_positive};
use crate::math::saturate_unit;
use crate::{AltitudeHoldCommand, AttitudeSetpoint, VehicleState};

use super::CopterGuidance;

#[cfg(test)]
mod tests;

/// Default [`AltitudeHoldConfig::deadzone`]: ArduPilot's `THR_DZ` 100 of the stick's 1000,
/// either side of mid stick (`Copter::get_pilot_desired_climb_rate_ms`): 0.2 of the travel
/// from center to either end.
pub const DEFAULT_CLIMB_DEADZONE: f32 = 0.2;

/// Default [`AltitudeHoldConfig::max_climb_mps`]: ArduPilot's `PILOT_SPD_UP`, 2.5 m/s.
pub const DEFAULT_MAX_CLIMB_MPS: f32 = 2.5;

/// Default [`AltitudeHoldConfig::max_sink_mps`]: PX4's `MPC_Z_VEL_MAX_DN`, 1.5 m/s, slower
/// than ArduPilot's 2.5 (`PILOT_SPD_DN` 0 sinks at `PILOT_SPD_UP`). On the sim's `quad_x`, a
/// full-down stick released sank 0.62 m past where it was let go at 1.5 m/s against 1.54 m at
/// 2.5, and met the ground at 1.5 m/s against 2.5 (#441).
pub const DEFAULT_MAX_SINK_MPS: f32 = 1.5;

/// Default [`AltitudeHoldConfig::accel_mps2`]: ArduPilot's `PILOT_ACC_Z`, 2.5 m/s².
pub const DEFAULT_CLIMB_ACCEL_MPS2: f32 = 2.5;

/// How the climb stick of altitude-hold flight maps to a climb rate, as ArduPilot's AltHold
/// maps its throttle: linear past a deadzone around center, to the top rate at either end
/// (a slower one down).
///
/// # Examples
///
/// ```
/// use stable_flight::guidance::copter::AltitudeHoldConfig;
///
/// let config = AltitudeHoldConfig::default();
/// assert_eq!(config.climb_rate_mps(0.1), 0.0); // inside the deadzone: hold
/// assert_eq!(config.climb_rate_mps(1.0), 2.5); // full up
/// assert_eq!(config.climb_rate_mps(-0.6), -0.75); // halfway from the deadzone down
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AltitudeHoldConfig {
    /// How far the climb stick moves from center, as a fraction of its travel to either end,
    /// before the vehicle climbs or sinks. Higher: a released or loosely centered stick holds
    /// more reliably, less travel left for the rate. In `[0, 1)`. Defaults to
    /// [`DEFAULT_CLIMB_DEADZONE`].
    pub deadzone: f32,
    /// Climb rate at full up stick, in m/s. Higher: a quicker climb, which a vehicle short of
    /// thrust can't reach (its altitude loop saturates). Must be positive and finite. Defaults
    /// to [`DEFAULT_MAX_CLIMB_MPS`].
    pub max_climb_mps: f32,
    /// Sink rate at full down stick, in m/s. Higher: a quicker descent, closer to the vortex
    /// ring state. Must be positive and finite. Defaults to [`DEFAULT_MAX_SINK_MPS`].
    pub max_sink_mps: f32,
    /// How fast the target's climb rate follows the stick, in m/s². Higher: snappier on the
    /// stick, larger thrust steps when it moves. Must be positive and finite. Defaults to
    /// [`DEFAULT_CLIMB_ACCEL_MPS2`].
    pub accel_mps2: f32,
}

impl Default for AltitudeHoldConfig {
    /// ArduPilot's AltHold deadzone, climb, and acceleration (0.2, 2.5 m/s, 2.5 m/s²), and PX4's
    /// 1.5 m/s sink.
    fn default() -> Self {
        Self {
            deadzone: DEFAULT_CLIMB_DEADZONE,
            max_climb_mps: DEFAULT_MAX_CLIMB_MPS,
            max_sink_mps: DEFAULT_MAX_SINK_MPS,
            accel_mps2: DEFAULT_CLIMB_ACCEL_MPS2,
        }
    }
}

impl AltitudeHoldConfig {
    /// Checks that `deadzone` is in `[0, 1)` and the rates and acceleration are positive and
    /// finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check("deadzone", (0.0..1.0).contains(&self.deadzone), "in [0, 1)")?;
        check_positive("max_climb_mps", self.max_climb_mps)?;
        check_positive("max_sink_mps", self.max_sink_mps)?;
        check_positive("accel_mps2", self.accel_mps2)
    }

    /// The climb rate the climb `stick` asks for, in m/s, up positive: 0 inside the deadzone,
    /// then linear to `max_climb_mps` at 1 and `-max_sink_mps` at -1. Saturated past ±1;
    /// NaN is centered.
    pub fn climb_rate_mps(&self, stick: f32) -> f32 {
        let stick = saturate_unit(stick);
        let past_deadzone = (stick.abs() - self.deadzone).max(0.0) / (1.0 - self.deadzone);
        if stick > 0.0 {
            past_deadzone * self.max_climb_mps
        } else {
            -past_deadzone * self.max_sink_mps
        }
    }
}

/// Whether an altitude-hold vehicle is on the ground, where a stick at or below the deadzone
/// idles the motors (ArduPilot's `Mode::get_alt_hold_state_D_ms`).
#[derive(Debug, Default, Clone)]
pub(super) struct Ground {
    /// `None` until altitude-hold flight starts, which reads it from the height above the
    /// ground the guidance was primed on.
    landed: Option<bool>,
    /// Reads a touchdown at the end of a descent on the stick.
    touchdown: TouchdownDetector,
}

impl Ground {
    /// Forgets it, for when altitude-hold flight ends.
    pub(super) fn reset(&mut self) {
        self.landed = None;
        self.touchdown.reset();
    }
}

impl CopterGuidance {
    /// Altitude-hold flight: the roll, pitch, and yaw sticks as attitude flight's, and the
    /// climb stick's rate ([`AltitudeHoldConfig::climb_rate_mps`]) moving an altitude target
    /// that the altitude loop flies, the rate fed forward.
    ///
    /// On the ground, a stick at or below the deadzone idles the motors, and above it the
    /// vehicle takes off. On the ground means: entering altitude-hold flight idle (or just
    /// armed) no higher than
    /// [`airborne_height_m`](super::CopterGuidanceConfig::airborne_height_m) above where
    /// guidance was primed, or a descent on the stick having stopped
    /// ([`TouchdownDetector`]). ArduPilot's AltHold does the same (`_TakeOff::triggered_ms`).
    pub(super) fn altitude_hold(
        &mut self,
        dt: f32,
        command: &AltitudeHoldCommand,
        estimated_state: &VehicleState,
    ) -> AttitudeSetpoint {
        let tilt_rad = self.stick_tilt(
            dt,
            command.roll,
            command.pitch,
            command.yaw,
            estimated_state,
        );
        let rate_mps = self.config.altitude_hold.climb_rate_mps(command.climb);
        let on_ground = self.holds_on_ground(dt, rate_mps, estimated_state);
        let thrust = if on_ground {
            0.0
        } else {
            let target = self.climb_target(
                dt,
                rate_mps,
                estimated_state.position_m.z,
                estimated_state.velocity_mps.z,
            );
            self.altitude_thrust(dt, target, estimated_state)
        };
        AttitudeSetpoint {
            roll_rad: tilt_rad.x,
            pitch_rad: tilt_rad.y,
            heading_rad: self.heading,
            thrust,
            transition: false,
            idle: on_ground,
        }
    }

    /// Whether the vehicle sits on the ground this tick, idling: landed (on entry, from its
    /// height; in flight, once a descent on the stick stops) and the stick at or below the
    /// deadzone. A climb takes off, the altitude target then latching from the vehicle.
    fn holds_on_ground(&mut self, dt: f32, rate_mps: f32, estimated_state: &VehicleState) -> bool {
        let airborne_height_m = self.config.airborne_height_m;
        // Entering on the ground means idling there: the last setpoint idled (or guidance was
        // just primed, as at arming) within `airborne_height_m` of the ground. A hover that
        // low isn't landed: its last setpoint flew.
        let landed_on_entry =
            self.last_idle && self.wind.on_ground(airborne_height_m, estimated_state);
        let landed = *self.ground.landed.get_or_insert(landed_on_entry);
        if landed {
            if rate_mps <= 0.0 {
                return true;
            }
            self.ground.landed = Some(false);
            self.ground.touchdown.reset();
            return false;
        }
        if rate_mps >= 0.0 {
            self.ground.touchdown.reset();
            return false;
        }
        if !self
            .ground
            .touchdown
            .update(dt, estimated_state.velocity_mps.z)
        {
            return false;
        }
        // Down: the next climb starts from the vehicle, not from a target leashed below it.
        self.ground.landed = Some(true);
        self.altitude_target_m = None;
        self.descent_speed_mps = 0.0;
        true
    }

    /// The altitude target for a climb (up positive) of `rate_mps`: the latched target, or the
    /// vehicle's `altitude_m` with its `climb_mps` (as a descent starts,
    /// [`descent_start_m`](Self::descent_start_m)), moved at a rate that follows `rate_mps` at
    /// up to [`AltitudeHoldConfig::accel_mps2`], and leashed
    /// [`altitude_leash_m`](super::CopterGuidanceConfig::altitude_leash_m) above or below the
    /// vehicle, so a vehicle that can't keep up isn't left far behind its target. A NaN
    /// target holds hover (the altitude loop's input policy) until the altitude is finite.
    fn climb_target(
        &mut self,
        dt: f32,
        rate_mps: f32,
        altitude_m: f32,
        climb_mps: f32,
    ) -> AltitudeTarget {
        let Some(start_m) = self.descent_start_m(altitude_m, climb_mps) else {
            return AltitudeTarget {
                altitude_m: f32::NAN,
                velocity_mps: 0.0,
            };
        };
        let step_mps = self.config.altitude_hold.accel_mps2 * dt;
        let last_mps = -self.descent_speed_mps;
        let velocity_mps = last_mps + (rate_mps - last_mps).clamp(-step_mps, step_mps);
        let leash_m = self.config.altitude_leash_m;
        // `max` then `min`, not `clamp`: they pass a NaN altitude's bounds without panicking.
        let target_m = (start_m + velocity_mps * dt)
            .max(altitude_m - leash_m)
            .min(altitude_m + leash_m);
        self.altitude_target_m = Some(target_m).filter(|t| t.is_finite());
        self.descent_speed_mps = -velocity_mps;
        AltitudeTarget {
            altitude_m: target_m,
            velocity_mps,
        }
    }
}
