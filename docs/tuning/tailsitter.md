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
  whatever the thrust, so descents are slower than a copter's.
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
5. Hover descent rate
6. Plane cruise and airspeeds
7. Forward transition
8. Back transition
9. Plane rate loop (acro) and maneuvers

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
| `STAB_RATE_RLL_P`, `_I`, `_D` | `hover.body_rate.roll_gains` | 0.4, 0.2, 0.03 |
| `STAB_RATE_PIT_P`, `_I`, `_D` | `hover.body_rate.pitch_gains` | 0.3, 0.5, 0 |
| `STAB_RATE_YAW_P`, `_I`, `_D` | `hover.body_rate.yaw_gains` | 0.1, 0.05, 0 |
| `STAB_RATE_IMAX` | `hover.body_rate.max_integral` | 1.0 |

- **Method:** stick pulses in each axis. Compare the rates with the setpoint and watch the
  moments.
- **Roll needs high gains.** Differential thrust is weak: the mixer gives 0.25 per unit moment.
  Too low, and a roll can't be stopped.
- **Roll needs some D in wind.** Descending with the wind a little behind a wingtip leaves
  the roll loop barely damped: on the eXplora at D 0, roll swung 2.5× its request, ±12–16° on
  a 1.2 s cycle, to touchdown; D 0.03 held it within 4° (#476).
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
  ([step 5](#5-hover-descent-rate)), with its altitude target at most 1 m below the vehicle
  (`ALTITUDE_LEASH_M`), so a hover handed over still climbing doesn't sink past it.
- **Boost ceiling:** caps the tilt boost, so the motors keep headroom for differential thrust.
  At full boost the motor pair saturates, and a roll can only be pushed one way.
- **Symptoms:**
  - The nose drops in a piloted descent: raise the floor or tighten the sink guard. In a
    waypoint descent, lower the descent rate first (step 5).
  - A roll in a tilted hover won't stop: lower the boost ceiling.

## 5. Hover descent rate

| Param | Field | eXplora |
|---|---|---|
| `GUID_DESC_MAX` | `hover.max_descent_mps` | 1.5 m/s (core default 3 m/s) |
| `GUID_DESC_ACC` | `hover.descent_accel_mps2` | 1.0 m/s² (core default) |

The fastest descent waypoint flight asks for, including the climb-down after a back
transition. Sinking tail first, the air meets the prop wash head on, so the elevons lose
authority as the descent speeds up. The pitch loop then lags, and the nose swings even with
thrust near hover and the elevons short of full deflection. A sink guard can't help: thrust is
already there. Landing has its own, slower descent (`GUID_LAND_SPD`, 0.7 m/s).

`GUID_DESC_ACC` is how hard the descent speeds up and brakes onto its waypoint: the target's
rate rises by at most this much per second from hover, and slows so this deceleration stops it
there (ArduPilot's `WP_ACC_Z`, 1 m/s²). A faster start cuts thrust harder: stepping straight
to 1.5 m/s took the eXplora's thrust to its floor in 0.25 s and dropped the nose 10° (#417).
A land speeds up to `GUID_LAND_SPD` the same way: stepping to it cut thrust 0.89 → 0.76 in one
tick (#476).
Lower it if the start drops the nose or the arrival sinks past the waypoint; raise it to reach
the waypoint sooner.

A descent that starts while the vehicle still climbs, as the hover after a back transition
does, first sheds the climb at `GUID_CLIMB_DEC` (step 8).

Start at 1.5 m/s, the default descent rate of ArduPilot (`WP_SPD_DN`) and PX4
(`MPC_Z_V_AUTO_DN`). The copter default of 3 m/s is too fast for a tailsitter until a flight
shows otherwise.

**Method.** Fly a hover-only script that climbs to a waypoint and descends 40 m to another
over the same point (`flights/explora_descent.csv`: 60 m to 20 m over the spawn point), with
`--record`; `--param GUID_DESC_MAX=2` sets each run's rate.
Across the descent, from the peak `altitude` down to the lower waypoint, read:

- `view_pitch` against `sp_pitch`: a swing on a steady setpoint is this limit;
- `elevon_*`: the swing's moment, short of ±1 if it's lag rather than lost authority;
- `climb_mps`: the descent actually flown (screen the tick rate, below).

To descend faster, raise `GUID_DESC_MAX` in 0.5 m/s steps. Keep the fastest rate whose worst
`view_pitch` error stays within about 3°, and drop back a step at any swing. Compare only
seconds that ran at about 245 Hz or more: a slower tick rate slows the descent target too.

eXplora (#414), 60 m to 20 m, calm air:

| `GUID_DESC_MAX` | Sink, mean | `view_pitch` | Worst error | Elevon peak | Note |
|---|---|---|---|---|---|
| **1.5 m/s** | 1.5 m/s | −3.5…−1.5° | 1.5° | 0.16 | chosen |
| 2 m/s | 1.9 m/s | −8…+1° | 6° | 0.31 | past 3°: back to 1.5 |
| 3 m/s | 2.9 m/s | −23…+18°, 2.5 s cycle | 23° | 0.78 | thrust 0.82–0.94 at the nose-down peaks |

Stiffer pitch rate gains (`STAB_RATE_PIT_P` 0.5, `_I` 0.2) at 3 m/s only cut the swing to
about −12…+5°.

- **Symptoms:**
  - The nose swings in a waypoint descent while `sp_pitch` holds: lower `GUID_DESC_MAX`.
  - Descents take too long: raise it in 0.5 m/s steps and check the swing again.

## Hover in wind

A check after step 5, not a step of its own: fly `flights/explora_descent.csv` with
`--record` in a steady wind, and read `north_m`/`east_m` against the waypoint, `view_pitch`
against `sp_pitch`, `sp_thrust`, and `elevon_*`.

- **Which way the wing faces decides whether it holds.** Broadside, the wing catches the wind
  and the hold leans to the tilt cap (`GUID_TILT_MAX`) and drifts. With a wingtip into the
  wind it needs little lean. `GUID_WND_HDG_SPD` turns it there: holding a waypoint or
  landing, a wind at least that fast, whose direction the [wind estimate](#wind-estimate)
  knows within 15°, turns the heading at `GUID_HDG_RATE` to put the wingtip nearer the
  heading into the wind, the belly 80° off where it comes from. A script's `heading` wins,
  and longer legs keep their heading until within `GUID_YAW_MIN_LEG` of the waypoint. Why
  80°: square on (88°), the eXplora's pitch swung 9.3–12.7° off its setpoint early in a
  60 → 20 m descent, and 4.9–6.0° at 78–80°, where a little wind on the belly loads the
  pitch loop; at about 65° it pinned its cap, and with the wind behind the wing (108°) its
  landings rolled on a cycle until `STAB_RATE_RLL_D` damped them (#476).

  | Param | Field | eXplora |
  |---|---|---|
  | `GUID_WND_HDG_SPD` | `hover.wind_heading_min_mps` | 1.0 m/s (core default 0: off) |
- **Don't raise the tilt cap to hold broadside.** More lean takes the elevon margin that the
  start of a descent needs. A 35° cap saturated the eXplora's elevons and threw its nose
  through −25…−66° for 20 s.
- **The descent start** cuts thrust as the target starts down: `GUID_DESC_ACC` sets how fast
  (step 5). If thrust touches `GUID_THR_MIN` and the nose drops, lower it.

eXplora (#417), 3 mph RealFlight wind (stronger at height), 60 m to 20 m:

| Heading | Descent start | Drift max | Worst `view_pitch` error | `sp_thrust` at the start | Elevon peak |
|---|---|---|---|---|---|
| Belly into the wind | stepped to 1.5 m/s | 41–43 m, at the 27° cap | 7.6–8.1° | 0.65 (floor)–0.92 | 0.45–0.50 |
| Belly into the wind, `GUID_TILT_MAX` 35° | stepped | 12.5 m | 97° | 0.65–1.00 | 1.00 |
| About 90° turned | stepped | 3–6 m | 12.0° | 0.65 (floor)–0.99 | 0.72 |
| Belly into the wind | ramped at 1 m/s² | 47 m, at the 27° cap | 4.0° | 0.80–0.87 | 0.43 |
| **About 90° turned** | **ramped at 1 m/s²** | **4.1–4.2 m** | **5.5–5.8°** | **0.80–0.90** | **0.55–0.56** |

- **Symptoms:**
  - The hold sits at `sp_pitch` = the tilt cap and drifts downwind: the wing is broadside;
    turn a wingtip into the wind: `GUID_WND_HDG_SPD` (with the wind estimate's coefficients
    set), or a `heading` on the script's hold waypoints, turned to at `GUID_HDG_RATE`.
  - `sp_thrust` drops to `GUID_THR_MIN` as a descent starts and the nose follows: lower
    `GUID_DESC_ACC`.

### Wind estimate

Hover guidance estimates the wind from the drag push it measures: the specific force across
the belly and along the span, inverted per axis through f = −(k|v| + μ)v to the air's speed,
then filtered (core `estimation::wind`). It reports the estimate in telemetry (`wind_*`
recorder columns), and `GUID_WND_HDG_SPD` turns the heading from it (above). It runs only in copter flight, more than 1 m above
the takeoff height, and tilted less than 45°. Every coefficient 0 switches it off.

| Param | Field | Meaning | eXplora |
|---|---|---|---|
| `GUID_WND_BCOEF_X`, `_Y` | `hover.wind.ballistic_kgpm2` | ballistic coefficient B, k = ½ρ/B (0: no v² term) | 0, 0 |
| `GUID_WND_MCOEF_X`, `_Y` | `hover.wind.momentum_per_s` | momentum drag μ, belly (x) and span (y) | 4.76, 0.24 /s |
| `GUID_WND_F_NSE` | `hover.wind.force_noise_mps2` | specific force noise of a 0.2 s mean | 0.45 m/s² |
| `GUID_WND_P_NSE` | `hover.wind.process_noise_mps_per_rt_s` | how fast the wind may change | 0.1 m/s/√s |

To fit them:

1. Fly `flights/explora_wind_cal.csv` with `--record` in a steady RealFlight wind (5 km/h):
   20 s holds at 40 m at headings up to 45° either side of a wingtip. A heading that pins
   the tilt cap never reaches its hold and stalls the script: stop it, and retune the
   headings to the wind.
2. Per axis, take 0.2 s means of `view_force_*_mps2` against the air's speed along that axis
   (ground velocity minus `true_wind_*`, rotated by the heading), and fit k and μ by least
   squares. `GUID_WND_F_NSE` is the fit's residual.
3. Fly `flights/explora_wind_check.csv` (60 s with a wingtip into the wind, 60 s 20° off it)
   with the fitted values as `--param GUID_WND_MCOEF_X=… --param GUID_WND_MCOEF_Y=…`, and
   compare `wind_*` with `true_wind_*` over seconds at ≥245 Hz. Where the fit varies with
   heading, try candidates the same way, without a rebuild.

eXplora (#473), RealFlight wind 2.1–2.3 m/s at 40 m:

- **Drag is linear in the air's speed** (a wing in its propwash): across the belly μ 4.76/s
  (R² 0.98, residual 0.45 m/s²); a v² term fits a negative k.
- **Along the span it isn't:** f/v is 0.22/s with a wingtip into the wind and 0.26–0.30
  elsewhere. The overall fit, 0.27, read a wingtip wind 30% slow; 0.24 splits the error.

| μ span | Run | Wingtip into the wind (125°) | 20° off (145°) |
|---|---|---|---|
| 0.27 | check 1, 2 | **fails**: speed 0.69–0.70 of the truth, 1/5 and 4/12 s in band | 16/16, 9/9 s; 1.5°, 0.87–0.89 |
| **0.24** | **check 3, 4** | **38/38, 40/40 s**; 3.1°, 0.78–0.85 | **20/20, 37/37 s**; 3.3°, 0.97–1.01 |

Each cell: seconds within 20° and 30% of RealFlight's wind over seconds at ≥245 Hz, worst
direction error, speed ratio.

## 6. Plane cruise and airspeeds

| Param | Field | eXplora |
|---|---|---|
| `PGUID_CRUISE` | `plane.cruise_thrust` | 0.85 |
| `PSTAB_ASPD_TRIM`, `_MIN`, `_MAX` | `plane.trim_airspeed_mps`, … | 14, 9, 25 m/s |
| `PSTAB_THR_EXPO` | `plane.thrust_curve_exponent` | 2, the same as `STAB_THR_EXPO` |

- **Cruise thrust:** the throttle that holds level flight at trim airspeed. The eXplora held
  14–15 m/s with the stick at 0.85–0.89.
- **Thrust curve:** match the hover side's, so a handoff keeps the motor commands continuous.
- **Airspeeds:** still guesses until plane flights measure them. `PGUID_ASPD_MIN` and `_MAX`
  (`plane.tecs`) take the same values: waypoint flight's total energy control holds its speed
  demand between them ([quadplane guide](quadplane.md#plane-altitude-and-airspeed-total-energy-control)).

## 7. Forward transition

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

## 8. Back transition

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

**The hover leg after the handover.** The hover takes over at rest and flies on to its
point. Unshaped, the lateral loop asks for the leg's whole speed in one tick: on the lap's
last leg, which runs along the span, that was 20° of view roll at once. Differential thrust
overshot it to 35°, past the 30° cap, with one motor at 1.00. `GUID_LAT_ACC` starts the
velocity target at the vehicle's velocity and gains speed at that rate, so a leg asks for
`atan(GUID_LAT_ACC / g)` of tilt at first.

| Param | Field | eXplora |
|---|---|---|
| `GUID_LAT_ACC` | `hover.lateral.max_accel_mps2` | 1 m/s² (core default 0: off) |
| `GUID_YAW_SLEW` | `hover.waypoint_yaw_radps` | 0: holds the heading (core default 60°/s): facing a leg flies it belly first |
| `GUID_HDG_RATE` | `hover.heading_rate_radps` | 30°/s (core default 60°/s): turns to a waypoint's `heading`; turned 90° in 3 mph wind and held (#417) |
| `GUID_YAW_YLD_S` | `hover.heading_yield_s` | 0: never yields the heading (core default 1 s); wind along the span is [#387](https://github.com/wboayue/stable-flight/issues/387) |

eXplora (#418), `explora_transition_lap.csv --end land`, runs at 250 Hz around the handover:

| `GUID_LAT_ACC` | Peak view roll | Largest roll setpoint | Arrival at the point |
|---|---|---|---|
| 0 (off) | 35.1°, 29.5°, 34.5° | 22° | 29–33 s |
| **1 m/s²** | 10.8°, 10.7° | 8° | 32–34 s |

It also limits braking: stopping from 2.4 m/s at 1 m/s² takes 2.9 m, so the sim's hover
stops about 2 m past a point straight ahead of the handover. A tilt slew in its place
(15°/s) held the roll to 16°, but in the sim it didn't stop a 3 m/s leg along the span from
overshooting its speed and departing.

**The hover's climb after the handover.** A ramp that hands over climbing leaves the hover
above its point, so it starts a descent at once. A target latched at the vehicle's height and
at rest makes the altitude loop stop the climb with thrust: the eXplora's thrust fell to
`GUID_THR_MIN`, the climb died, and the hover sank at up to 2.75 m/s, past its 1.5 m/s
`GUID_DESC_MAX` (#462). The target now starts rising at the handover's climb and sheds it at
`GUID_CLIMB_DEC`, as ArduPilot starts a tailsitter's vertical controller
(`AC_PosControl.cpp:849-863`, at `Q_PILOT_ACCEL_Z`). A handover that arrives sinking starts
at rest. Shedding a climb v rises about v² / (2 · `GUID_CLIMB_DEC`) further.

| Param | Field | eXplora |
|---|---|---|
| `GUID_CLIMB_DEC` | `hover.climb_decel_mps2` | 2.5 m/s² (core default) |

Sim tailsitter, `GUID_DESC_MAX` 1.5 m/s, worst sink in the 3 s after the handover / rise
above the handover height (#462):

| `TRAN_VT_THR` | Climb at handover | Target at rest | 1 m/s² | **2.5 m/s²** | g |
|---|---|---|---|---|---|
| 0.7 | +3.4 m/s | 1.98 m/s / 0.9 m | climbing / 6.1 m | **1.61 / 2.6 m** | 1.75 / 1.1 m |
| 0.8 | +8.1 m/s | 2.07 / 4.2 m | climbing / 33 m | **climbing / 13.5 m** | 1.94 / 5.0 m |

Lower it if the hover still sinks fast after a climbing handover; raise it if it balloons
too far above the point. A large climb at the handover is the back ramp's to fix (thrust and
handover tilt above), not this one's.

## 9. Plane rate loop (acro) and maneuvers

| Param | Field | eXplora |
|---|---|---|
| `PSTAB_RLL_FF` | `plane.rate.roll.ff` | 0.1 (PX4 default 0.5) |
| `PSTAB_PIT_FF` | `plane.rate.pitch.ff` | 0.05 (0.5) |
| `PSTAB_PIT_I` | `plane.rate.pitch.i` | 0.8 (0.1) |
| `PSTAB_PIT_TRIM` | `plane.rate.trim.y` | 0.09 |

Rate flight (rate sticks, scripted maneuvers) flies the plane rate loop with no attitude loop
over it, so its feedforward and trim show directly; stabilized flight hides them behind the
attitude loop. Tune them from doublets, then fly rolls and loops.

- **Script:** `flights/explora_acro.csv` flies the transition lap's rectangle at 60 m with a
  maneuver at the middle of each long leg: roll and pitch doublets at 60°/s, then a 180°/s
  roll and a 25 m loop (`just fly-tailsitter-auto flights/explora_acro.csv --end land
  --record <csv>`). A script row's eighth field names the maneuver (`roll:<deg/s>`,
  `loop:<m>`, `doublet_roll:<deg/s>`, …; the `ScriptCommander` rustdoc lists them).
- **Maneuver thrust:** `airframes::explora_maneuvers()`. Rolls and loops fly at full thrust;
  doublets fly at cruise thrust (0.85), since the prop wash over the elevons makes the rate a
  moment gives grow with thrust.
- **Feedforward:** read each doublet's first half: its peak rate over the command. Divide the
  feedforward by that ratio, then fly again. At PX4's 0.5 the eXplora's 30°/s doublets peaked
  at 3.2× in roll and 3.4× in pitch: its 0.63 kg wing behind ±45° elevons turns a unit moment
  into far more rate than PX4 expects.
- **Pitch I:** carries the pitch rate through a loop as it speeds up coming down. Too low and
  the rate lags the loop's second half; too high and a doublet's second half overshoots.
- **Path hold:** `ManeuverConfig::path` (`maneuver::PathConfig`; the eXplora flies the
  defaults). A roll holds its line and a loop its circle, steered back with pitch and yaw
  rate: position gain 2/s² and velocity gain 2.8/s (ArduPilot's), and on a roll half the lift
  the wing loses as it rolls, steered against up front. Gains at 0 fly the rates alone,
  as doublets do.
- **Trim:** the mean `moment_y` on level plane legs at trim airspeed, times (airspeed / trim
  airspeed)² off it (the loop scales trim by that factor squared). It holds the nose at zero
  rate without waiting for the I term. The eXplora's: 0.09 at 14–15 m/s.
- **Sim first:** `stable-flight-sim`'s `airframes::explora()` is sized from the eXplora's
  `.RFX` and calibrated to its default-gain doublets, nose-up moment, and level thrust; tune
  feedforward and I on it (`tests/maneuver.rs`), then confirm in RealFlight.
- **eXplora in RealFlight** (the script at 60 m, wind off): doublets peak at 0.91× (roll) and
  1.18× (pitch) in their first half (#342). With the path hold the 180°/s roll turns at
  168°/s and its lowest point is 1–2 m below its entry, either way; the 25 m loop spans
  25.2 m of radius and ends 2 m low, a 10 m loop 10.4 m and 1.3 m low. On rates alone the roll
  lost 20 m and the loops 16 m and 6 m (#481).
- **Symptoms:**
  - Doublet peaks well past its command: feedforward down.
  - Doublet short of its command, rate creeping up through each half: feedforward up.
  - Loop wide and low on its way down: path gains up, or `PSTAB_PIT_I` up if the rate lags.
  - A roll climbs: `line_gravity_share` down; it sinks: up.
  - Nose drops at centered rate sticks: `PSTAB_PIT_TRIM` up.

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
| Roll cycles about 1 s in a descent in wind, larger than its request | `STAB_RATE_RLL_D` up |
| Roll overshoots past the cap as a hover leg starts (after a back handover) | `GUID_LAT_ACC` |
| Nose drops in a descent | `GUID_THR_MIN`, `GUID_SINK_LIM`/`_GAIN`, `STAB_RATE_PIT_I` |
| Nose swings in a waypoint descent, `sp_pitch` steady | `GUID_DESC_MAX` down |
| Nose drops as a waypoint descent starts, `sp_thrust` at the floor | `GUID_DESC_ACC` down |
| Hover in wind pinned at the tilt cap, drifting | heading: a wingtip into the wind, `GUID_WND_HDG_SPD` ([hover in wind](#hover-in-wind)) |
| Wing sinks right after the forward transition | `TRAN_ASPD_MIN`, `TRAN_FW_THR` |
| Large climb after the back request | `TRAN_VT_THR` down, `TRAN_VT_RATE` slower |
| Back ramp wallows nose-high, elevons at full deflection, roll-limit cut | `TRAN_VT_THR` up |
| Hover tumbles just after the back handover | arrives too fast: `TRAN_VT_RATE`, `TRAN_VT_DONE` |
| Rate flight overshoots its commanded rate | `PSTAB_RLL_FF`, `PSTAB_PIT_FF` down ([acro](#9-plane-rate-loop-acro-and-maneuvers)) |
