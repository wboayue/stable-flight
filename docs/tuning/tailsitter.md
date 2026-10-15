# Tailsitter tuning guide

Tunes a two-motor, two-elevon tailsitter flown with `TailsitterGuidance` and
`TailsitterStabilizer`:
- **Hover:** the copter controllers, run in the tailsitter `View`.
- **Wing-borne flight:** the plane controllers.
- **Between them:** a `Transition`.

The worked example is the RealFlight eXplora VTOL. Its values are quoted below, but
[`airframes::explora_vtol()`](../../crates/stable-flight-sitl-realflight/src/airframes.rs) is
the authority, with the flight behind each value cited beside it.

## What makes a tailsitter different

The elevons are the only pitch control in hover, and they work only in the prop wash. At low
airspeed their authority comes from thrust: in #343 the eXplora lost pitch authority at 3–4 m/s
with thrust 0.6. Most tailsitter-specific knobs exist because of this:

- **Low thrust:** too little thrust in hover or on the back transition, and the elevons can't
  hold the nose.
- **Sinking:** a sink of a few m/s turns the airflow against the prop wash and drops the nose,
  whatever the thrust.
- **Roll in hover:** this is differential thrust, which is weak and needs motor headroom.

When a knob trades climbing against sinking, choose climbing: a climb costs height; a sink can
end in the ground.

## Before you tune

1. **Frame and mapping.** Sweep the channels (`channel-sweep-realflight`), capture a flight,
   and check it against the frame contract: `cargo run --bin check_frames -- capture.csv
   --pitch-deg 90`. Mixer: `TableMixer::tailsitter_2p()`. Body rotation: +90° pitch for a
   model that reads level on its tail. Use the `stable-flight-sitl-realflight` README for the
   setup.
2. **Live tuning.** Run `just bridge`, then `just fly-tailsitter` (joystick) or
   `just fly-tailsitter-auto <script>` (scripted), then QGC.
   - **Params:** change them in QGC's parameter editor; they apply on the next tick. When a
     value works, copy it into the airframe, citing the flight.
   - **Restarts:** the examples start from the airframe's values on each run.
3. **Recording.** Add `--record <csv>` to either example. These columns matter most; the
   [`TailsitterColumns`](../../crates/stable-flight-sitl/src/recorder.rs) rustdoc has the full
   list:

   | Column | Meaning |
   |---|---|
   | `mode` | 0 copter, 1 plane, 2 to plane, 3 to copter |
   | `altitude`, `climb_mps`, `airspeed_mps` | m, m/s up, m/s |
   | `north_m`, `east_m`, `vel_north_mps`, `vel_east_mps` | horizontal position (m from the simulator's origin, not the arming point) and ground velocity (m/s): drift, wind, waypoint arrival |
   | `view_roll`, `view_pitch`, `view_heading` | hover attitude, rad (0 nose up) |
   | `pitch`, `roll` | airplane attitude, rad |
   | `sp_thrust`, `stick_throttle` | commanded linear thrust, the pilot's throttle |
   | `motor_left`, `motor_right`, `elevon_left`, `elevon_right` | actuator outputs; an elevon near ±1 has no authority left |
   | `moment_x`, `_y`, `_z` | stabilizer moments, [-1, 1] |

   Rates, moments, and the motors and elevons are in the airplane body frame, so the hover
   axes are swapped (signs aside):

   | Hover axis | Attitude | Rate | Moment | Actuator |
   |---|---|---|---|---|
   | roll (sideways tilt) | `view_roll` | `yaw_rate` | `moment_z` | differential thrust |
   | pitch (fore-aft tilt) | `view_pitch` | `pitch_rate` | `moment_y` | elevons together |
   | yaw (turn about the vertical) | `view_heading` | `roll_rate` | `moment_x` | elevons apart |

   In plane flight the columns read as named: `roll`, `pitch`, `heading` with `roll_rate`,
   `pitch_rate`, `yaw_rate`.

4. **Repeatable missions.** For transitions, `flights/explora_transition_lap.csv` flies the
   same climb, plane lap, and back transition every run, so heights compare across settings
   (`just fly-tailsitter-auto flights/explora_transition_lap.csv --end land --record <csv>`).
   Its back transition starts where the two-lap `flights/explora_transition.csv` starts its
   own, so runs of either compare.

## Order

Each step assumes the ones before it:

1. Hover thrust
2. Hover rate loops
3. Hover attitude
4. Hover thrust limits
5. Plane cruise and airspeeds
6. Forward transition
7. Back transition

Params in degrees are marked (deg); thrusts are linear (before the thrust curve), in [0, 1].

## 1. Hover thrust

| Param | Field | eXplora |
|---|---|---|
| `GUID_ALT_HOVER` | `hover.altitude.hover_thrust` | 0.84 |
| `STAB_THR_EXPO` | `hover.thrust_curve_exponent` | 2 (default) |

- **What it does:** the altitude loop feeds it forward, so the integral only trims what's left.
- **Measure it:** hold a level hover and read the mean motor command. Through the thrust curve
  and the mixer's thrust factor *k*, hover thrust = (mean motor / *k*)^(1/expo).
  - eXplora: mean motor 0.63, *k* = 0.9, so √(0.63 / 0.9) = 0.84.
- **Symptom when wrong:** waypoint holds settle off target at first, then creep onto it as the
  integral winds.

## 2. Hover rate loops

The axes are the hover view's:
- **Roll:** tilting sideways; differential thrust.
- **Pitch:** tilting fore and aft; the elevons.
- **Yaw:** turning about the vertical; also the elevons.

| Param | Field | eXplora |
|---|---|---|
| `STAB_RATE_RLL_P`, `_I`, `_D` | `hover.body_rate.roll_gains` | 0.4, 0.2, 0 |
| `STAB_RATE_PIT_P`, `_I`, `_D` | `hover.body_rate.pitch_gains` | 0.3, 0.5, 0 |
| `STAB_RATE_YAW_P`, `_I`, `_D` | `hover.body_rate.yaw_gains` | 0.1, 0.05, 0 |
| `STAB_RATE_IMAX` | `hover.body_rate.max_integral` | 1.0 |

- **Method:** stick pulses in each axis. Compare the rates with the setpoint and watch the
  moments.
- **Roll needs high gains.** Differential thrust is weak: the mixer gives 0.25 per unit moment.
  Too low, and a roll can't be stopped.
- **Pitch needs its I term.** When the vehicle sinks, the airflow pushes the nose over;
  I × `STAB_RATE_IMAX` = 0.5 of moment holds it level.

## 3. Hover attitude

| Param | Field | eXplora |
|---|---|---|
| `STAB_ATT_RLL_P`, `STAB_ATT_PIT_P` | `hover.attitude.*_gains.p` | 4, 4 |
| `STAB_ATT_AMAX_X`, `_Y` (deg) | `hover.attitude.max_angles_rad` | 30°, 30° |
| `STAB_ATT_RMAX_X`, `_Y` (deg) | `hover.attitude.max_rate_radps` | 2 rad/s (115°/s) |
| `GUID_STICK_X`, `_Y` (deg) | `hover.max_stick_angles_rad` | 30°, 30° |

- **P:** P 4 asks about 110°/s at 27° off, so a recovery takes about half a second with no
  lingering tail.
- **Angle cap:** keep it modest. The transition ramps fly past it, so it doesn't limit
  transitions.

## 4. Hover thrust limits

| Param | Field | eXplora |
|---|---|---|
| `GUID_THR_MIN` | `hover.thrust.min_thrust` | 0.65 |
| `GUID_SINK_LIM` | `hover.thrust.sink_limit_mps` | 1.0 m/s |
| `GUID_SINK_GAIN` | `hover.thrust.sink_gain` | 0.15 per m/s |
| `GUID_BOOST_MAX` | `hover.thrust.max_boost_thrust` | 0.9 |

- **Floor:** keeps prop wash over the elevons. 0.65 gives 60% of hover thrust force, since
  force goes as thrust²; a throttle at idle still passes through. Keep it below
  `GUID_ALT_HOVER`, or waypoint flight can't descend and a landing never touches down.
- **Sink guard:** adds thrust to piloted flight past the sink limit:
  0.65 + 0.15 × 2 = 0.95 at 3 m/s. Waypoint flight limits its own descent
  (`GUID_DESC_MAX`, 3 m/s), with its altitude target at most 1 m below the vehicle
  (`ALTITUDE_LEASH_M`), so a hover handed over still climbing doesn't sink past it.
- **Boost ceiling:** caps the tilt boost, so the motors keep headroom for differential thrust.
  At full boost the motor pair saturates, and a roll can only be pushed one way.
- **Symptoms:**
  - The nose drops in a descent: raise the floor or tighten the sink guard.
  - A roll in a tilted hover won't stop: lower the boost ceiling.

## 5. Plane cruise and airspeeds

| Param | Field | eXplora |
|---|---|---|
| `PGUID_CRUISE` | `plane.cruise_thrust` | 0.85 |
| `PSTAB_ASPD_TRIM`, `_MIN`, `_MAX` | `plane.trim_airspeed_mps`, … | 14, 9, 25 m/s |
| `PSTAB_THR_EXPO` | `plane.thrust_curve_exponent` | 2, the same as `STAB_THR_EXPO` |

- **Cruise thrust:** the throttle that holds level flight at trim airspeed. The eXplora held
  14–15 m/s with the stick at 0.85–0.89.
- **Thrust curve:** match the hover side's, so a handoff keeps the motor commands continuous.
- **Airspeeds:** still guesses until plane flights measure them.

## 6. Forward transition

| Param | Field | eXplora |
|---|---|---|
| `TRAN_MIN_HGT` | `transition.min_height_m` | 20 m |
| `TRAN_FW_THR` | `transition.forward_thrust` | 0.9 |
| `TRAN_ASPD_MIN` | `transition.min_airspeed_mps` | 9 m/s |
| `TRAN_FW_RATE`, `TRAN_FW_DONE`, `TRAN_FW_MAX` (deg) | `forward_rate_radps`, … | 50°/s, 45°, 80° (defaults) |

- **Ramp thrust:** above hover, because lift drops as the nose tilts over until the wing
  flies. If the hover thrust when the ramp starts is higher, that holds instead.
- **Airspeed gate:** set it to the plane's minimum airspeed. Completing at 45° with 1.5 m/s,
  the wing sank 6 m while it gained speed.
- **Minimum height:** clear the script's hover climb. A script's first plane leg is reached on
  horizontal distance, before the climb ends.
- **Symptoms:**
  - Height lost right after `mode` turns 1: raise `TRAN_ASPD_MIN` or `TRAN_FW_THR`.
  - Times out back to hover: raise `TRAN_FW_THR`, or check that `TRAN_FW_MAX` reaches
    `TRAN_FW_DONE`.

## 7. Back transition

| Param | Field | eXplora |
|---|---|---|
| `TRAN_VT_RATE` (deg) | `transition.back_rate_radps` | 25°/s |
| `TRAN_VT_DONE` (deg) | `transition.back_done_rad` | 20° |
| `TRAN_VT_THR` | `transition.back_thrust` | 0.75 |
| `TRAN_VT_SINK_LIM` | `transition.back_sink_limit_mps` | 1 m/s (default) |
| `TRAN_VT_SINK_GN` | `transition.back_sink_gain` | 0.4 per m/s (default) |

**Where the climb comes from.** The nose-up turns the plane's speed into height: v²/2g, about
11 m from 15 m/s. Ramp thrust decides how much of that speed is bled off rather than climbed.
- **Rate and handover tilt:** a slow ramp that hands over near vertical arrives in hover nearly
  stopped. At 50°/s and 45° the eXplora arrived at 11 m/s and tumbled.
- **Thrust:** below cruise, so the nose-up slows the vehicle more than it climbs.
- **Sink guard:** adds thrust while the ramp sinks past its limit. It's a backstop for a ramp
  that stalls nose-high, not a setting to lean on.

**Method.** Fly the transition mission with `--record`, start below cruise thrust, and step
`TRAN_VT_THR` by 0.05. In the ToCopter ticks (`mode` 3) and at the switch to 0, read:

- `climb_mps` at the handover: below `-TRAN_VT_SINK_LIM` means it handed over sinking.
- `sp_thrust` above `TRAN_VT_THR`: the guard is acting.
- `elevon_*` near ±1, with `pitch` stuck short of its target: the elevons have lost the prop
  wash.
- The peak `altitude` against the height at the back request.

Lower the thrust while the climb is too high and the handover isn't sinking. Raise it if the
guard acts, the nose wallows, or the roll limit (`TRAN_ROLL_MAX`) cuts the ramp short. Keep
the lowest thrust that hands over without sinking.

eXplora (#343), 25°/s to 20°, back request at about 29.5 m and 13.9 m/s:

| `TRAN_VT_THR` | Back transition | Climb rate at handover | Guard | Peak climb | Note |
|---|---|---|---|---|---|
| 0.84 | 5.9 s | | | +18 m | hover thrust: climbs the most |
| 0.6 | 9.1 s, roll-limit cut | −2.2 m/s | repeatedly | +5.0 m | nose stuck 52° up, elevons at full deflection |
| 0.7 | 3.4 s | −1.7 m/s | last 0.2 s | +7.5 m | sinks as the speed runs out |
| **0.75** | 2.9 s | +1.7 m/s | never | +9.3 m | chosen |

The in-repo sim flew 0.6 cleanly where the eXplora couldn't, so tune this on RealFlight or
hardware ([#399](https://github.com/wboayue/stable-flight/issues/399)).

## Transition limits

| Param | Field | Default |
|---|---|---|
| `TRAN_HANDOFF` | `transition.handoff_s` | 0.5 s thrust blend at a handoff (PX4) |
| `TRAN_ROLL_MAX` (deg) | `transition.roll_limit_rad` | 45°: a ramp rolled past it is cut short (ArduPilot) |
| `TRAN_TIMEOUT` | `transition.timeout_s` | 15 s: forward reverts to hover, back completes (PX4) |

## Symptoms

| Symptom | Look at |
|---|---|
| Hover settles off its altitude, then creeps on | `GUID_ALT_HOVER` |
| A roll in hover won't stop | `STAB_RATE_RLL_*`, `GUID_BOOST_MAX` |
| Nose drops in a descent | `GUID_THR_MIN`, `GUID_SINK_LIM`/`_GAIN`, `STAB_RATE_PIT_I` |
| Wing sinks right after the forward transition | `TRAN_ASPD_MIN`, `TRAN_FW_THR` |
| Large climb after the back request | `TRAN_VT_THR` down, `TRAN_VT_RATE` slower |
| Back ramp wallows nose-high, elevons at full deflection, roll-limit cut | `TRAN_VT_THR` up |
| Hover tumbles just after the back handover | arrives too fast: `TRAN_VT_RATE`, `TRAN_VT_DONE` |
