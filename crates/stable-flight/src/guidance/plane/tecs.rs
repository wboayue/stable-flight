//! Total energy control for [`PlaneGuidance`](super::PlaneGuidance)'s waypoint flight: thrust
//! sets the rate of the aircraft's total specific energy (height and airspeed together), pitch
//! the balance between them. A climb the pitch limit can't make stays in the total energy
//! error, so thrust rises to make it; the speed loop never takes a climb's power (#454).
//!
//! Follows the Total Energy Control System as ArduPilot's `AP_TECS`
//! (`libraries/AP_TECS/AP_TECS.cpp`) and PX4's `TECS` (`src/lib/tecs/TECS.cpp`) fly it,
//! described, not copied:
//!
//! - **Energies:** potential `g·h`, kinetic `½V²`, and their rates `g·ḣ` and `V·V̇`
//!   (`AP_TECS::_update_energies`). `V̇` is the airspeed's own derivative through a low-pass
//!   ([`airspeed_rate_time_constant_s`](TecsConfig::airspeed_rate_time_constant_s)), about the filter ArduPilot puts on its damping term
//!   (`_update_throttle_with_airspeed`), so it doesn't hang on how a backend measures
//!   acceleration; PX4 drops its acceleration term for bias
//!   (`FwLateralLongitudinalControl::tecs_update_pitch_throttle`). The kinetic rate is
//!   `V·(V̇ − lpf(V̇))`, high passed as ArduPilot's (`_update_energies`) at
//!   [`kinetic_rate_high_pass_s`](TecsConfig::kinetic_rate_high_pass_s), so a steady
//!   acceleration isn't a rate error.
//! - **Demands:** the height demand slews toward the target at the climb and sink limits, the
//!   airspeed demand at half the climb's energy rate over `V` (`_update_height_demand`,
//!   `_update_speed_demand`). The slewed height demand then runs through a lag
//!   ([`height_demand_time_constant_s`](TecsConfig::height_demand_time_constant_s),
//!   `_update_height_demand`), whose slope is the height rate's feedforward; the airspeed's
//!   rate demand is its error over the time constant alone, as PX4's over its own
//!   (`FW_T_TAS_TC`, `TECSControl::_calcAirspeedControlOutput`), held to half the climb's energy rate for thrust and unheld for pitch. The lag starts at
//!   the aircraft's own climb rate (within the climb and sink limits), as PX4's reference model
//!   does (`TECSAltitudeReferenceModel::initialize`, from `TECS::initialize`). ArduPilot's
//!   starts level (`_initialise_states`): an aircraft entering plane flight already sinking
//!   then pitches up to stop the sink, falls behind the demand, and sinks past the limit
//!   catching up (#503). Each error over the time constant is a proportional term
//!   (`_update_throttle_with_airspeed`). The airspeed demand stays within the least and most
//!   airspeed, and starts there: seeded at a stalled speed it would hold the stall. The height
//!   demand stays within one time constant's climb or sink of the aircraft, so a climb it
//!   can't make doesn't run the demand away; ArduPilot instead stops advancing the demand
//!   while pitch or thrust is at its limit (`_update_height_demand`'s climb and sink scalers),
//!   which needs the last tick's clipping.
//! - **Thrust:** the total energy rate demand, mapped around cruise thrust so that the climb
//!   limit takes full thrust and the least sink least thrust
//!   (`TECSControl::_calcThrottleControlOutput`), plus damping on the rate error and an
//!   integral of the energy error (`_update_throttle_with_airspeed`), with its own gain as
//!   PX4's (`FW_T_THR_INTEG`).
//! - **Pitch:** the balance `w_p·SPE − w_k·SKE` (`w_k` the speed weight, `w_p = 2 − w_k`, each
//!   capped at 1; `_update_pitch`); its rate demand over `V·g` is a flight path angle, plus
//!   trim pitch, damping on the rate error, and an integral of the rate error that fades while
//!   pinned at a pitch limit (`_update_pitch`), with its own gain as PX4's (`FW_T_I_GAIN_PIT`).
//!   The rate error is taken against the clamped demand, so the integral pulls the nose up as
//!   soon as a sink passes the sink limit. ArduPilot adds a second integral, of the airspeed
//!   error alone (`_update_pitch`); this has none. A forward transition passes its pitch limits
//!   in, so the integral stops where the transition stops the pitch (ArduPlane's
//!   `SLT_Transition::set_FW_roll_pitch`).
//! - **Underspeed:** below [`underspeed_share`](TecsConfig::underspeed_share) of the least
//!   airspeed (`_detect_underspeed`), full thrust and
//!   pitch holding airspeed alone (speed weight 2). It clears back at the least airspeed,
//!   where the wing flies. ArduPilot latches only at 95% throttle and clears at 1.15 of the
//!   least after 3 s or once at the height demand (`_detect_underspeed`); for a heavy airframe
//!   1.15 can sit above trim airspeed (the sim Alia 250: 46 m/s against 45), leaving only the
//!   height to clear it.
//! - **Turns:** thrust for a bank's extra drag, `k·(1/cos²φ − 1)` on the energy rate
//!   (`_update_throttle_with_airspeed`, `TECS_RLL2THR`).

use crate::controllers::{InvalidConfig, check, check_limit, check_positive};
use crate::math::{clamp_symmetric, lagged};
use crate::{GRAVITY, VehicleState};

#[cfg(test)]
mod tests;

/// Default [`TecsConfig::height_demand_time_constant_s`], in s: ArduPilot's
/// `TECS_HDEM_TCONST` default. Without the lag the demand's rate steps by the whole climb limit
/// in one tick where the slew starts and stops, and the eXplora's pitch setpoint jumped 28° in
/// RealFlight (#454).
pub const DEFAULT_HEIGHT_DEMAND_TIME_CONSTANT_S: f32 = 3.0;

/// Default [`TecsConfig::airspeed_rate_time_constant_s`], in s:
/// `AP_TECS::_update_throttle_with_airspeed` filters its damping at about 0.5 s.
pub const DEFAULT_AIRSPEED_RATE_TIME_CONSTANT_S: f32 = 0.5;

/// Below this airspeed, in m/s, `V·g` stops shrinking: the pitch terms stay finite at a stall.
const MIN_GAIN_AIRSPEED_MPS: f32 = 3.0;

/// Default [`TecsConfig::underspeed_share`]: `AP_TECS::_detect_underspeed`'s.
pub const DEFAULT_UNDERSPEED_SHARE: f32 = 0.9;

/// Least `cos²φ` the turn term divides by: 72° of bank
/// (`AP_TECS::_update_throttle_with_airspeed`).
const MIN_COS2_BANK: f32 = 0.1;

/// Default [`TecsConfig::kinetic_rate_high_pass_s`], in s. ArduPilot's is `TECS_TIME_CONST`
/// (`AP_TECS::_update_speed`, `_update_energies`) on an accelerometer's rate; on this filtered
/// one 0.5–2 s passes the sim suite at pitch integral gains 0.1–0.3, and 3 s or more lets the
/// plane's loop and the eXplora's rolls miss their regained altitude (1.0–1.4 m against 1 m)
/// (#505).
pub const DEFAULT_KINETIC_RATE_HIGH_PASS_S: f32 = 1.0;

/// How far past a thrust limit the thrust integral may push, so it can pull back from the
/// limit at once (`AP_TECS::_update_throttle_with_airspeed`).
const THRUST_INTEGRAL_MARGIN: f32 = 0.1;

/// [`PlaneGuidance`](super::PlaneGuidance)'s total energy control settings.
///
/// # Examples
///
/// An airframe sets its climb at full thrust, its sink at least thrust, and its airspeeds; the
/// gains keep ArduPilot's defaults:
///
/// ```
/// use stable_flight::guidance::plane::{PlaneGuidanceConfig, TecsConfig};
///
/// let config = PlaneGuidanceConfig {
///     tecs: TecsConfig {
///         max_climb_mps: 1.8,
///         min_sink_mps: 3.5,
///         min_airspeed_mps: 40.0,
///         max_airspeed_mps: 60.0,
///         ..TecsConfig::default()
///     },
///     ..PlaneGuidanceConfig::default()
/// };
/// assert!(config.validate().is_ok());
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TecsConfig {
    /// How quickly height and airspeed errors close, in s: their energy errors over it are the
    /// rate demands' proportional terms.
    pub time_constant_s: f32,
    /// Climb rate at full thrust and the trim airspeed, in m/s: the most the height demand
    /// climbs, and where full thrust sits on the energy rate.
    pub max_climb_mps: f32,
    /// Sink rate at least thrust and the trim airspeed, in m/s: where least thrust sits on the
    /// energy rate.
    pub min_sink_mps: f32,
    /// The most the height demand sinks, in m/s.
    pub max_sink_mps: f32,
    /// Thrust per unit of energy rate error, as a share of the thrust range over the energy
    /// rate range.
    pub thrust_damping: f32,
    /// Integral gain of the thrust loop: higher closes a steady total energy error (a leg
    /// flown low or slow) sooner.
    pub integrator_gain: f32,
    /// Pitch per unit of energy balance rate error, over `V·g`.
    pub pitch_damping: f32,
    /// Integral gain of the pitch loop, on the balance rate error: higher closes a steady
    /// balance error (height against airspeed) sooner; past 0.3 the sim Alia's attitude steps
    /// overshoot PX4's spec (#505).
    pub pitch_integrator_gain: f32,
    /// How pitch splits height and airspeed, `[0, 2]`: 1 both alike, 0 height only (the speed
    /// left to thrust), 2 airspeed only.
    pub speed_weight: f32,
    /// Energy rate added in a bank, `k·(1/cos²φ − 1)`, in m²/s³: thrust for the turn's
    /// induced drag.
    pub roll_compensation: f32,
    /// Least airspeed the speed demand holds, in m/s; underspeed below
    /// [`underspeed_share`](Self::underspeed_share) of it.
    pub min_airspeed_mps: f32,
    /// Most airspeed the speed demand holds, in m/s; below `min_airspeed_mps` it reads as that.
    pub max_airspeed_mps: f32,
    /// Share of `min_airspeed_mps`, `(0, 1]`, below which underspeed latches: full thrust and
    /// pitch for airspeed alone, until back at `min_airspeed_mps`. Higher: recovers sooner,
    /// trading more height for it.
    pub underspeed_share: f32,
    /// Time constant of the height demand's lag after its slew, in s: the lag keeps the
    /// demand's rate continuous where the slew starts and stops. Higher: gentler pitch at a
    /// climb's start and end, reaching the target later. Past `time_constant_s`, a fast climb
    /// or sink at entry starts the demand slower than the aircraft: the slewed demand stays
    /// within one `time_constant_s` of climb or sink.
    pub height_demand_time_constant_s: f32,
    /// Time constant of the low-pass the airspeed rate is taken against, in s. Higher: a
    /// steadier rate from a noisy airspeed, read later.
    pub airspeed_rate_time_constant_s: f32,
    /// Time constant of the high-pass on the airspeed rate, in s: a steadier acceleration than
    /// it isn't a kinetic energy rate error, only its change is. Higher: damps a slower
    /// acceleration too; 0.5–2 s holds the sim airframes' altitude through their rolls.
    pub kinetic_rate_high_pass_s: f32,
}

impl Default for TecsConfig {
    /// ArduPilot's defaults (`AP_TECS::var_info`: `TECS_TIME_CONST` 5, `CLMB_MAX` 5,
    /// `SINK_MIN` 2, `SINK_MAX` 5, `THR_DAMP` 0.5, `INTEG_GAIN` 0.3 for the thrust integral,
    /// `PTCH_DAMP` 0.3, `SPDWEIGHT` 1, `RLL2THR` 10), with the plane stabilizer's default
    /// airspeeds (10–25 m/s). The pitch integral's 0.2 is the middle of the gains that pass the
    /// sim suite on every airframe (0.1–0.3, #505).
    fn default() -> Self {
        TecsConfig {
            time_constant_s: 5.0,
            max_climb_mps: 5.0,
            min_sink_mps: 2.0,
            max_sink_mps: 5.0,
            thrust_damping: 0.5,
            integrator_gain: 0.3,
            pitch_damping: 0.3,
            pitch_integrator_gain: 0.2,
            speed_weight: 1.0,
            roll_compensation: 10.0,
            min_airspeed_mps: 10.0,
            max_airspeed_mps: 25.0,
            underspeed_share: DEFAULT_UNDERSPEED_SHARE,
            height_demand_time_constant_s: DEFAULT_HEIGHT_DEMAND_TIME_CONSTANT_S,
            airspeed_rate_time_constant_s: DEFAULT_AIRSPEED_RATE_TIME_CONSTANT_S,
            kinetic_rate_high_pass_s: DEFAULT_KINETIC_RATE_HIGH_PASS_S,
        }
    }
}

impl TecsConfig {
    /// The total energy rate at full thrust, `g · max_climb_mps`, in m²/s³: the most the
    /// height's rate demand asks.
    fn max_energy_rate(&self) -> f32 {
        GRAVITY * self.max_climb_mps
    }

    /// The height's rate demand at the largest sink, `−g · max_sink_mps`, in m²/s³.
    fn least_energy_rate(&self) -> f32 {
        -GRAVITY * self.max_sink_mps
    }

    /// `Ok` when the time constants, climb, least sink, and airspeeds are positive, the largest
    /// sink, gains, and roll compensation non-negative, the speed weight in `[0, 2]`, and the
    /// underspeed share in `(0, 1]`.
    ///
    /// # Errors
    ///
    /// [`InvalidConfig`] naming the first field out of range.
    pub fn validate(&self) -> Result<(), InvalidConfig> {
        check_positive("time_constant_s", self.time_constant_s)?;
        check_positive("max_climb_mps", self.max_climb_mps)?;
        check_positive("min_sink_mps", self.min_sink_mps)?;
        check_limit("max_sink_mps", self.max_sink_mps)?;
        check_limit("thrust_damping", self.thrust_damping)?;
        check_limit("integrator_gain", self.integrator_gain)?;
        check_limit("pitch_damping", self.pitch_damping)?;
        check_limit("pitch_integrator_gain", self.pitch_integrator_gain)?;
        check(
            "speed_weight",
            (0.0..=2.0).contains(&self.speed_weight),
            "in [0, 2]",
        )?;
        check_limit("roll_compensation", self.roll_compensation)?;
        check_positive("min_airspeed_mps", self.min_airspeed_mps)?;
        check_positive("max_airspeed_mps", self.max_airspeed_mps)?;
        check(
            "underspeed_share",
            self.underspeed_share > 0.0 && self.underspeed_share <= 1.0,
            "in (0, 1]",
        )?;
        check_positive(
            "height_demand_time_constant_s",
            self.height_demand_time_constant_s,
        )?;
        check_positive(
            "airspeed_rate_time_constant_s",
            self.airspeed_rate_time_constant_s,
        )?;
        check_positive("kinetic_rate_high_pass_s", self.kinetic_rate_high_pass_s)
    }
}

/// The plane guidance's limits TECS flies within.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Limits {
    /// Most pitch up, in rad.
    pub pitch_up_rad: f32,
    /// Most pitch down, in rad, positive.
    pub pitch_down_rad: f32,
    /// Pitch at the cruise's angle of attack, in rad.
    pub trim_pitch_rad: f32,
    /// Least thrust, `[0, 1]`.
    pub min_thrust: f32,
    /// Most thrust, `[0, 1]`, at least `min_thrust`.
    pub max_thrust: f32,
    /// Thrust for level flight at trim airspeed.
    pub cruise_thrust: f32,
}

/// Height and airspeed demands, slewed toward the targets.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Demand {
    /// The height demand slewed at the climb and sink limits, in m.
    slewed_height_m: f32,
    /// The slewed demand through the height demand's lag, in m: the demand flown.
    height_m: f32,
    airspeed_mps: f32,
}

/// Total energy control state: the demands, the two integrals, and the underspeed latch.
/// Seeds its demands at the first step after a [`reset`](Tecs::reset).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Tecs {
    demand: Option<Demand>,
    /// Pitch integral, in rad.
    pitch_integral_rad: f32,
    /// Thrust integral, a thrust share.
    thrust_integral: f32,
    underspeed: bool,
    /// Low-passed airspeed, in m/s: the airspeed rate is the gap to it over the time constant.
    filtered_airspeed_mps: Option<f32>,
    /// The airspeed rate low-passed at the kinetic rate's high-pass, in m/s²: the kinetic
    /// energy rate is the airspeed rate's gap to it.
    steady_accel_mps2: Option<f32>,
}

/// Energy rates and errors of one step, specific (per unit mass): m²/s² and m²/s³.
struct Energies {
    potential_error: f32,
    kinetic_error: f32,
    potential_rate_demand: f32,
    /// The kinetic error over the time constant, unclamped: thrust holds it to half the climb's
    /// energy rate, pitch takes it whole.
    kinetic_rate_demand: f32,
    potential_rate: f32,
    kinetic_rate: f32,
    /// The airspeed the rates divide by, floored at [`MIN_GAIN_AIRSPEED_MPS`], in m/s.
    speed_mps: f32,
}

impl Tecs {
    /// Forgets the demands, integrals, and underspeed: the next step starts from the state.
    pub(crate) fn reset(&mut self) {
        *self = Tecs::default();
    }

    /// Pitch (rad) and thrust toward `altitude_m` at `airspeed_target_mps`, flying at
    /// `airspeed_mps`. A non-finite target, airspeed, height, or climb rate gives trim pitch
    /// and cruise thrust and leaves the state alone.
    pub(crate) fn step(
        &mut self,
        dt: f32,
        config: &TecsConfig,
        limits: &Limits,
        (altitude_m, airspeed_target_mps): (f32, f32),
        airspeed_mps: f32,
        state: &VehicleState,
    ) -> (f32, f32) {
        let height_m = state.position_m.z;
        let climb_mps = state.velocity_mps.z;
        let inputs = [
            altitude_m,
            airspeed_target_mps,
            airspeed_mps,
            height_m,
            climb_mps,
        ];
        if !inputs.iter().all(|x| x.is_finite()) {
            return (limits.trim_pitch_rad, limits.cruise_thrust);
        }
        self.update_underspeed(config, airspeed_mps);
        let speed_mps = airspeed_mps.max(MIN_GAIN_AIRSPEED_MPS);
        let energies = self.energies(
            dt,
            config,
            (altitude_m, airspeed_target_mps),
            (airspeed_mps, speed_mps),
            state,
        );
        let thrust = self.thrust(dt, config, limits, &energies, state.roll_rad);
        let pitch_rad = self.pitch(dt, config, limits, &energies);
        (pitch_rad, thrust)
    }

    /// Slews the demands and forms the energy errors and rates.
    fn energies(
        &mut self,
        dt: f32,
        config: &TecsConfig,
        (altitude_m, airspeed_target_mps): (f32, f32),
        (airspeed_mps, speed_mps): (f32, f32),
        state: &VehicleState,
    ) -> Energies {
        let (height_m, climb_mps) = (state.position_m.z, state.velocity_mps.z);
        let min_mps = config.min_airspeed_mps;
        let max_mps = config.max_airspeed_mps.max(min_mps);
        let tau = config.time_constant_s;
        let height_lag_s = config.height_demand_time_constant_s;
        let previous = *self.demand.get_or_insert_with(|| {
            // The slewed demand starts one height lag of the aircraft's climb away, so
            // the lag's slope starts at the climb rate. Entering plane flight sinking, the
            // demand sinks with it rather than pitching up to stop the sink (#503).
            let entry_climb_mps = climb_mps.clamp(-config.max_sink_mps, config.max_climb_mps);
            Demand {
                slewed_height_m: height_m + entry_climb_mps * height_lag_s,
                height_m,
                // A demand below the least airspeed would hold a stall (AP clamps it the same
                // way).
                airspeed_mps: airspeed_mps.clamp(min_mps, max_mps),
            }
        });

        // Height: slewed at the climb and sink limits, kept within one time constant's climb
        // or sink of the aircraft so an unreachable climb doesn't run away from it, then
        // lagged; the lagged demand's slope is the height rate's feedforward.
        let slewed_m = (previous.slewed_height_m
            + (altitude_m - previous.slewed_height_m)
                .clamp(-config.max_sink_mps * dt, config.max_climb_mps * dt))
        .clamp(
            height_m - tau * config.max_sink_mps,
            height_m + tau * config.max_climb_mps,
        );
        let demand_height_m = lagged(previous.height_m, slewed_m, dt, height_lag_s);
        let height_rate_mps = (demand_height_m - previous.height_m) / dt;

        // Airspeed: slewed at half the climb's energy rate over V
        // (`AP_TECS::_update_speed_demand`). Its rate demand is the error over the time constant
        // alone, as PX4's over its own (`FW_T_TAS_TC`,
        // `TECSControl::_calcAirspeedControlOutput`): the slew's slope would step where it stops.
        let max_accel_mps2 = 0.5 * config.max_energy_rate() / speed_mps;
        let target_mps = airspeed_target_mps.clamp(min_mps, max_mps);
        let demand_airspeed_mps = previous.airspeed_mps
            + (target_mps - previous.airspeed_mps).clamp(-max_accel_mps2 * dt, max_accel_mps2 * dt);
        self.demand = Some(Demand {
            slewed_height_m: slewed_m,
            height_m: demand_height_m,
            airspeed_mps: demand_airspeed_mps,
        });

        // The airspeed's rate: its gap to a low-pass of it over the time constant (a filtered
        // derivative).
        let rate_lag_s = config.airspeed_rate_time_constant_s;
        let filtered_mps = *self.filtered_airspeed_mps.get_or_insert(airspeed_mps);
        let accel_mps2 = (airspeed_mps - filtered_mps) / rate_lag_s;
        self.filtered_airspeed_mps = Some(lagged(filtered_mps, airspeed_mps, dt, rate_lag_s));
        // High-passed: a steady acceleration (the forward transition's lift rotors and pusher)
        // isn't a rate error, only its change is (`AP_TECS::_update_energies`).
        let steady_mps2 = *self.steady_accel_mps2.get_or_insert(accel_mps2);
        self.steady_accel_mps2 = Some(lagged(
            steady_mps2,
            accel_mps2,
            dt,
            config.kinetic_rate_high_pass_s,
        ));
        let changing_accel_mps2 = accel_mps2 - steady_mps2;

        let potential_error = GRAVITY * (demand_height_m - height_m);
        let kinetic_error =
            0.5 * (demand_airspeed_mps * demand_airspeed_mps - airspeed_mps * airspeed_mps);
        Energies {
            potential_error,
            kinetic_error,
            potential_rate_demand: (GRAVITY * height_rate_mps + potential_error / tau)
                .clamp(config.least_energy_rate(), config.max_energy_rate()),
            kinetic_rate_demand: kinetic_error / tau,
            speed_mps,
            potential_rate: GRAVITY * climb_mps,
            kinetic_rate: speed_mps * changing_accel_mps2,
        }
    }

    /// Thrust from the total energy rate demand, its error, and the energy error's integral.
    fn thrust(
        &mut self,
        dt: f32,
        config: &TecsConfig,
        limits: &Limits,
        energies: &Energies,
        roll_rad: f32,
    ) -> f32 {
        let (low, high) = (limits.min_thrust, limits.max_thrust);
        if self.underspeed {
            return high;
        }
        let max_rate = config.max_energy_rate();
        let min_rate = -GRAVITY * config.min_sink_mps;
        // Speeding up takes no more than half the climb's energy rate.
        let kinetic_rate_demand = clamp_symmetric(energies.kinetic_rate_demand, 0.5 * max_rate);
        let rate_demand =
            (energies.potential_rate_demand + kinetic_rate_demand).clamp(min_rate, max_rate);
        let rate = energies.potential_rate + energies.kinetic_rate;

        // A bank's extra drag, on the feedforward only
        // (`AP_TECS::_update_throttle_with_airspeed`).
        let cos_bank = libm::cosf(roll_rad);
        let cos2 = (cos_bank * cos_bank).max(MIN_COS2_BANK);
        let turn = if cos2.is_finite() {
            config.roll_compensation * (1.0 / cos2 - 1.0)
        } else {
            0.0
        };
        let feedforward_rate = rate_demand + turn;
        let cruise = limits.cruise_thrust.clamp(low, high);
        let feedforward = if feedforward_rate >= 0.0 {
            cruise + feedforward_rate / max_rate * (high - cruise)
        } else {
            cruise + feedforward_rate / -min_rate * (cruise - low)
        };
        // Thrust per unit of energy rate, over the whole range.
        let gain = (high - low) / (max_rate - min_rate);
        let proportional = feedforward + gain * config.thrust_damping * (rate_demand - rate);
        let total_error = energies.potential_error + energies.kinetic_error;
        self.thrust_integral = (self.thrust_integral
            + config.integrator_gain * gain * total_error / config.time_constant_s * dt)
            .clamp(
                low - proportional - THRUST_INTEGRAL_MARGIN,
                high - proportional + THRUST_INTEGRAL_MARGIN,
            );
        (proportional + self.thrust_integral).clamp(low, high)
    }

    /// Pitch from the energy balance's rate demand, its error, and the error's integral.
    fn pitch(&mut self, dt: f32, config: &TecsConfig, limits: &Limits, energies: &Energies) -> f32 {
        let (high, low) = (limits.pitch_up_rad, -limits.pitch_down_rad);
        let speed_weight = if self.underspeed {
            2.0
        } else {
            config.speed_weight
        };
        let kinetic_weight = speed_weight.min(1.0);
        let potential_weight = (2.0 - speed_weight).min(1.0);
        // The airspeed's part is unclamped: only the balance as a whole stays within the climb
        // and sink limits (`AP_TECS::_update_pitch`). Held to half the climb's energy rate, as
        // thrust's is, an underspeed transition asks 0.35 m/s² of the sim Alia and pitches up
        // against its faster rotors and pusher (#505).
        let rate_demand = (potential_weight * energies.potential_rate_demand
            - kinetic_weight * energies.kinetic_rate_demand)
            .clamp(config.least_energy_rate(), config.max_energy_rate());
        let rate =
            potential_weight * energies.potential_rate - kinetic_weight * energies.kinetic_rate;
        let rate_error = rate_demand - rate;
        // Energy balance rate over V·g is a flight path angle, in rad.
        let per_rad = energies.speed_mps * GRAVITY;
        let proportional_rad =
            limits.trim_pitch_rad + (rate_demand + config.pitch_damping * rate_error) / per_rad;
        // The integral of the rate error: it closes a steady balance error through the rate
        // demand's error term, and pulls the nose back up as soon as the sink passes the limit
        // (`AP_TECS::_update_pitch`, PX4's `TECSControl::_calcPitchControlUpdate`).
        let integral_step_rad = config.pitch_integrator_gain * rate_error / per_rad * dt;
        let unclamped = proportional_rad + self.pitch_integral_rad + integral_step_rad;
        let pinned = (unclamped > high && integral_step_rad > 0.0)
            || (unclamped < low && integral_step_rad < 0.0);
        let integral_rad = if pinned {
            // Pinned against the limit it pushes toward, the integral fades over the time
            // constant rather than winding (`AP_TECS::_update_pitch`).
            lagged(self.pitch_integral_rad, 0.0, dt, config.time_constant_s)
        } else {
            self.pitch_integral_rad + integral_step_rad
        };
        // Within the pitch range, not within 5° past the limits of the proportional term as
        // ArduPilot's `AP_TECS::_update_pitch`: that bound follows a proportional term far past a
        // limit, which at the forward transition's first few m/s winds the integral 70° up
        // (#505).
        self.pitch_integral_rad = integral_rad.clamp(low - high, high - low);
        (proportional_rad + self.pitch_integral_rad).clamp(low, high)
    }

    /// Latches underspeed below `underspeed_share` of the least airspeed, and clears it back at
    /// the least.
    fn update_underspeed(&mut self, config: &TecsConfig, airspeed_mps: f32) {
        if airspeed_mps < config.underspeed_share * config.min_airspeed_mps {
            self.underspeed = true;
        } else if airspeed_mps >= config.min_airspeed_mps {
            self.underspeed = false;
        }
    }
}
