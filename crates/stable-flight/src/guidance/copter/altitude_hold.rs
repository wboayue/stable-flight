//! Altitude-hold flight ([`ControlInput::AltitudeHold`](crate::ControlInput::AltitudeHold)):
//! the climb stick sets a climb rate, an altitude target moves at it, and the altitude loop
//! flies the target; let go, the vehicle brakes and holds the height it stops at. Its
//! settings: [`AltitudeHoldConfig`].

use crate::commanders::{DEFAULT_STOPPED_MPS, DEFAULT_TOUCHDOWN_HOLD_S};
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

/// Default [`AltitudeHoldConfig::lock_speed_mps`]: PX4's `MPC_HOLD_MAX_Z`, 0.6 m/s
/// (`FlightTaskManualAltitude::_updateAltitudeLock`).
pub const DEFAULT_LOCK_SPEED_MPS: f32 = 0.6;

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
    /// Vertical speed, in m/s, under which a vehicle braking after the stick returns to center
    /// holds the height it is at, as PX4's Altitude mode locks it. Until then the target follows
    /// the vehicle and only the climb rate it brakes along is fed forward, so whatever the
    /// vehicle ran ahead of or behind its target while climbing is dropped: a host whose ticks
    /// run slow in wall time moves the target slower than the rate fed forward, and the
    /// RealFlight QuadCopterX, released from a climb at 180 ticks/s of a nominal 250, sank 0.4 m
    /// back to its target (#441). Higher: locks sooner, stopping further past where the stick
    /// was let go. Must be positive and finite. Defaults to [`DEFAULT_LOCK_SPEED_MPS`].
    pub lock_speed_mps: f32,
}

impl Default for AltitudeHoldConfig {
    /// ArduPilot's AltHold deadzone, climb, and acceleration (0.2, 2.5 m/s, 2.5 m/s²), and PX4's
    /// 1.5 m/s sink and 0.6 m/s lock.
    fn default() -> Self {
        Self {
            deadzone: DEFAULT_CLIMB_DEADZONE,
            max_climb_mps: DEFAULT_MAX_CLIMB_MPS,
            max_sink_mps: DEFAULT_MAX_SINK_MPS,
            accel_mps2: DEFAULT_CLIMB_ACCEL_MPS2,
            lock_speed_mps: DEFAULT_LOCK_SPEED_MPS,
        }
    }
}

impl AltitudeHoldConfig {
    /// Checks that `deadzone` is in `[0, 1)` and the rates, acceleration, and lock speed are
    /// positive and finite.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check("deadzone", (0.0..1.0).contains(&self.deadzone), "in [0, 1)")?;
        check_positive("max_climb_mps", self.max_climb_mps)?;
        check_positive("max_sink_mps", self.max_sink_mps)?;
        check_positive("accel_mps2", self.accel_mps2)?;
        check_positive("lock_speed_mps", self.lock_speed_mps)
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

/// Altitude-hold flight's own state: whether the vehicle is on the ground, where a stick at or
/// below the deadzone idles the motors (ArduPilot's `Mode::get_alt_hold_state_D_ms`), and
/// whether a centered stick has locked the height.
#[derive(Debug, Default, Clone)]
pub(super) struct HoldState {
    /// `None` until altitude-hold flight starts, which reads it from the height above the
    /// ground the guidance was primed on.
    landed: Option<bool>,
    /// How long, in s, the vehicle has held still against a stick down, its target well below
    /// it: the ground holding it up.
    grounded_s: f32,
    /// The stick is centered and the vehicle slowed enough to hold the height it was at.
    locked: bool,
}

impl HoldState {
    /// Forgets it, for when altitude-hold flight ends.
    pub(super) fn reset(&mut self) {
        self.landed = None;
        self.grounded_s = 0.0;
        self.locked = false;
    }
}

/// How far below the vehicle, as a share of
/// [`altitude_leash_m`](super::CopterGuidanceConfig::altitude_leash_m), its target must be
/// for a vehicle held still against a stick down to read as on the ground: a target that far
/// below and a vehicle not following it means the ground is in the way. In the air the vehicle
/// keeps up with a sinking target to within its tracking error, centimeters on the sim.
const GROUNDED_TARGET_LEASHES: f32 = 0.5;

impl CopterGuidance {
    /// Altitude-hold flight: the roll, pitch, and yaw sticks as attitude flight's, and the
    /// climb stick's rate ([`AltitudeHoldConfig::climb_rate_mps`]) moving an altitude target
    /// that the altitude loop flies, the rate fed forward. Centered, it brakes and locks the
    /// height ([`climb_target`](Self::climb_target)).
    ///
    /// On the ground, a stick at or below the deadzone idles the motors, and above it the
    /// vehicle takes off. On the ground means: entering altitude-hold flight idle (or just
    /// armed) no higher than
    /// [`airborne_height_m`](super::CopterGuidanceConfig::airborne_height_m) above where
    /// guidance was primed, or the vehicle held still for [`DEFAULT_TOUCHDOWN_HOLD_S`] against a
    /// stick down, its target at least half the
    /// [`altitude_leash_m`](super::CopterGuidanceConfig::altitude_leash_m) below it
    /// ([`holds_on_ground`](Self::holds_on_ground)). ArduPilot's AltHold idles the same way
    /// (`_TakeOff::triggered_ms`, and its land detector on the throttle held low).
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
    /// height; in flight, once the ground holds it up against a stick down) and the stick at
    /// or below the deadzone. A climb takes off, the altitude target then latching from the
    /// vehicle.
    ///
    /// The ground holds it up when it stays slower than [`DEFAULT_STOPPED_MPS`] for
    /// [`DEFAULT_TOUCHDOWN_HOLD_S`] while its target is at least [`GROUNDED_TARGET_LEASHES`] of
    /// the leash below it. It needs no descent first, so a stick pushed down on a vehicle
    /// already resting on the ground idles it: released less than a second after a touchdown,
    /// the RealFlight QuadCopterX locked its height there and sat at hover thrust, and a later
    /// push couldn't idle it (#441). The target condition keeps a slow sink in the air, which
    /// the vehicle follows, from reading as a touchdown.
    fn holds_on_ground(&mut self, dt: f32, rate_mps: f32, estimated_state: &VehicleState) -> bool {
        let airborne_height_m = self.config.airborne_height_m;
        // Entering on the ground means idling there: the last setpoint idled (or guidance was
        // just primed, as at arming) within `airborne_height_m` of the ground. A hover that
        // low isn't landed: its last setpoint flew.
        let landed_on_entry =
            self.last_idle && self.wind.on_ground(airborne_height_m, estimated_state);
        let landed = *self.hold.landed.get_or_insert(landed_on_entry);
        if landed {
            if rate_mps <= 0.0 {
                return true;
            }
            self.hold.landed = Some(false);
            self.hold.grounded_s = 0.0;
            return false;
        }
        if rate_mps >= 0.0 {
            self.hold.grounded_s = 0.0;
            return false;
        }
        let altitude_m = estimated_state.position_m.z;
        let below_m = self.config.altitude_leash_m * GROUNDED_TARGET_LEASHES;
        let target_below = self
            .altitude_target_m
            .is_some_and(|target_m| target_m <= altitude_m - below_m);
        // `<` is false for a NaN speed: an unknown speed never reads as held still.
        let still = estimated_state.velocity_mps.z.abs() < DEFAULT_STOPPED_MPS;
        self.hold.grounded_s = if still && target_below {
            self.hold.grounded_s + dt
        } else {
            0.0
        };
        if self.hold.grounded_s < DEFAULT_TOUCHDOWN_HOLD_S {
            return false;
        }
        // Down: the next climb starts from the vehicle, not from a target leashed below it.
        self.hold.landed = Some(true);
        self.altitude_target_m = None;
        self.descent_speed_mps = 0.0;
        true
    }

    /// The altitude target for a climb (up positive) of `rate_mps`. Its rate follows `rate_mps`
    /// at up to [`AltitudeHoldConfig::accel_mps2`], starting from the vehicle's `climb_mps` (as
    /// a descent starts, [`descent_start_m`](Self::descent_start_m)), and is fed forward.
    ///
    /// - Climbing or sinking on the stick, the target moves from where it was (the vehicle's
    ///   `altitude_m` when first latched) at that rate, leashed
    ///   [`altitude_leash_m`](super::CopterGuidanceConfig::altitude_leash_m) above or below the
    ///   vehicle, so a vehicle that can't keep up isn't left far behind it.
    /// - Centered, the vehicle brakes: the target sits on the vehicle while its rate falls to 0,
    ///   so only the braking rate is flown, and once the vehicle is slower than
    ///   [`AltitudeHoldConfig::lock_speed_mps`] the height it is at is held, as PX4's Altitude
    ///   mode locks it (`FlightTaskManualAltitude::_updateAltitudeLock`).
    ///
    /// A NaN target holds hover (the altitude loop's input policy) until the altitude is finite.
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
        let config = &self.config.altitude_hold;
        let step_mps = config.accel_mps2 * dt;
        let last_mps = -self.descent_speed_mps;
        let velocity_mps = last_mps + (rate_mps - last_mps).clamp(-step_mps, step_mps);
        let target_m = if rate_mps != 0.0 {
            self.hold.locked = false;
            let leash_m = self.config.altitude_leash_m;
            // `max` then `min`, not `clamp`: they pass a NaN altitude's bounds without panicking.
            (start_m + velocity_mps * dt)
                .max(altitude_m - leash_m)
                .min(altitude_m + leash_m)
        } else if self.hold.locked {
            start_m
        } else {
            // `<` is false for a NaN climb: braking goes on until the climb reads slow.
            self.hold.locked = velocity_mps == 0.0 && climb_mps.abs() < config.lock_speed_mps;
            altitude_m
        };
        self.altitude_target_m = Some(target_m).filter(|t| t.is_finite());
        self.descent_speed_mps = -velocity_mps;
        AltitudeTarget {
            altitude_m: target_m,
            velocity_mps,
        }
    }
}
