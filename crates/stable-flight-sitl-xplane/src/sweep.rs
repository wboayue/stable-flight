//! [`EngineSweep`]: the throttle schedule of the `engine-sweep-xplane` example, which measures a
//! multirotor's yaw sign, liftoff throttle, motor lag, and response to each moment, with no
//! flight loop.

#[cfg(test)]
mod tests;

/// Number of lift engines a sweep drives: engines 0–3.
pub const LIFT_ENGINES: usize = 4;

/// The sweep's timing and throttles. [`Default`] suits a large multirotor on a runway.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepConfig {
    /// Rest with every engine at 0 before, between, and after the yaw pulses, s.
    pub rest_s: f32,
    /// The two engine pairs pulsed for yaw, one after the other: each pair is a diagonal, so its
    /// props spin the same way and roll and pitch cancel.
    pub yaw_pairs: [[usize; 2]; 2],
    /// Throttle of a yaw pulse: below liftoff, so the aircraft stays on its wheels.
    pub yaw_throttle: f32,
    /// Length of each yaw pulse, s.
    pub yaw_pulse_s: f32,
    /// How fast the liftoff ramp raises all lift engines, throttle per second.
    pub ramp_per_s: f32,
    /// Throttle added at liftoff for the motor-lag step.
    pub step_throttle: f32,
    /// Length of the step, s.
    pub step_s: f32,
    /// Throttle added to some engines and taken from the others in each airborne moment pulse
    /// (roll, pitch, yaw, in that order, after the step).
    pub moment_throttle: f32,
    /// Length of each moment pulse, s; after the last, every engine cuts.
    pub moment_pulse_s: f32,
    /// Recording after the cut, s.
    pub fall_s: f32,
}

impl Default for SweepConfig {
    fn default() -> Self {
        SweepConfig {
            rest_s: 2.0,
            yaw_pairs: [[0, 2], [1, 3]],
            yaw_throttle: 0.4,
            yaw_pulse_s: 3.0,
            ramp_per_s: 0.05,
            step_throttle: 0.1,
            step_s: 0.5,
            moment_throttle: 0.05,
            moment_pulse_s: 0.3,
            fall_s: 2.0,
        }
    }
}

/// Where the sweep is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Every engine at 0 before a yaw pulse.
    Rest,
    /// Pair `n` of [`SweepConfig::yaw_pairs`] at the yaw throttle.
    YawPulse(usize),
    /// Every engine at 0 before the ramp.
    Settle,
    /// All lift engines rising together until the aircraft leaves the ground.
    Ramp,
    /// The liftoff throttle plus the step.
    Step,
    /// The step's throttle, with a moment about `axis` (0 roll right, 1 pitch up, 2 yaw by
    /// pair 0 of [`SweepConfig::yaw_pairs`]).
    MomentPulse(usize),
    /// Every engine at 0, recording the fall.
    Fall,
    /// Over: every engine at 0.
    Done,
}

impl Phase {
    /// A number for the capture's `phase` column: 0 rest, 1–2 the yaw pulses, 3 settle, 4 ramp,
    /// 5 step, 6–8 the roll, pitch, and yaw moment pulses, 9 fall, 10 done.
    pub fn code(self) -> f32 {
        match self {
            Phase::Rest => 0.0,
            Phase::YawPulse(n) => 1.0 + n as f32,
            Phase::Settle => 3.0,
            Phase::Ramp => 4.0,
            Phase::Step => 5.0,
            Phase::MomentPulse(axis) => 6.0 + axis as f32,
            Phase::Fall => 9.0,
            Phase::Done => 10.0,
        }
    }
}

/// The sweep's state: call [`step`](Self::step) once per tick with the tick's `dt` and whether
/// the aircraft is on the ground, and send the throttles it returns.
///
/// 1. Rest, then each yaw pair alone at the yaw throttle, a rest after each.
/// 2. The ramp: all lift engines from 0 at `ramp_per_s` until the aircraft leaves the ground;
///    that throttle is the [`liftoff_throttle`](Self::liftoff_throttle). At full throttle
///    still on the ground, the sweep skips to the fall.
/// 3. The step: the liftoff throttle plus `step_throttle` for `step_s`.
/// 4. Moment pulses on the step's throttle, `moment_pulse_s` each: roll right (left engines 1
///    and 2 up by `moment_throttle`, right engines 0 and 3 down), pitch up (front 0 and 1 up,
///    rear 2 and 3 down), yaw (the first yaw pair up, the second down); then everything at 0
///    for `fall_s`. A moment's angular acceleration over a known thrust difference gives the
///    inertia; the yaw pulse, the yaw torque.
///
/// # Examples
///
/// ```
/// use stable_flight_sitl_xplane::sweep::{EngineSweep, Phase, SweepConfig};
///
/// let mut sweep = EngineSweep::new(SweepConfig::default());
/// assert_eq!(sweep.step(0.1, true), [0.0; 4]);
/// assert_eq!(sweep.phase(), Phase::Rest);
/// ```
#[derive(Debug, Clone)]
pub struct EngineSweep {
    config: SweepConfig,
    phase: Phase,
    /// Time in the phase, s.
    in_phase_s: f32,
    /// Rests done before the ramp.
    rests: usize,
    ramp_throttle: f32,
    liftoff_throttle: Option<f32>,
}

impl EngineSweep {
    /// A sweep starting at rest.
    pub fn new(config: SweepConfig) -> Self {
        EngineSweep {
            config,
            phase: Phase::Rest,
            in_phase_s: 0.0,
            rests: 0,
            ramp_throttle: 0.0,
            liftoff_throttle: None,
        }
    }

    /// The current phase.
    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// The ramp's throttle when the aircraft left the ground; `None` before, or if it never did.
    pub fn liftoff_throttle(&self) -> Option<f32> {
        self.liftoff_throttle
    }

    /// Advances by `dt_s` and returns the lift engines' throttles (engines 0–3) for this tick.
    pub fn step(&mut self, dt_s: f32, on_ground: bool) -> [f32; LIFT_ENGINES] {
        self.in_phase_s += dt_s;
        let c = self.config;
        match self.phase {
            Phase::Rest if self.in_phase_s >= c.rest_s => {
                self.rests += 1;
                let next = match self.rests {
                    1 => Phase::YawPulse(0),
                    2 => Phase::YawPulse(1),
                    _ => Phase::Settle,
                };
                self.enter(next);
            }
            Phase::YawPulse(_) if self.in_phase_s >= c.yaw_pulse_s => self.enter(Phase::Rest),
            Phase::Settle => self.enter(Phase::Ramp),
            Phase::Ramp => {
                self.ramp_throttle = (self.ramp_throttle + c.ramp_per_s * dt_s).min(1.0);
                if !on_ground {
                    self.liftoff_throttle = Some(self.ramp_throttle);
                    self.enter(Phase::Step);
                } else if self.ramp_throttle >= 1.0 {
                    self.enter(Phase::Fall);
                }
            }
            Phase::Step if self.in_phase_s >= c.step_s => self.enter(Phase::MomentPulse(0)),
            Phase::MomentPulse(axis) if self.in_phase_s >= c.moment_pulse_s => {
                self.enter(if axis < 2 {
                    Phase::MomentPulse(axis + 1)
                } else {
                    Phase::Fall
                });
            }
            Phase::Fall if self.in_phase_s >= c.fall_s => self.enter(Phase::Done),
            _ => {}
        }
        self.throttles()
    }

    /// The step's throttle: the liftoff throttle plus `step_throttle`.
    fn step_throttle(&self) -> f32 {
        (self.liftoff_throttle.unwrap_or(0.0) + self.config.step_throttle).min(1.0)
    }

    fn enter(&mut self, phase: Phase) {
        self.phase = phase;
        self.in_phase_s = 0.0;
    }

    /// The throttles of the current phase.
    fn throttles(&self) -> [f32; LIFT_ENGINES] {
        let c = self.config;
        match self.phase {
            Phase::YawPulse(n) => core::array::from_fn(|engine| {
                if c.yaw_pairs[n].contains(&engine) {
                    c.yaw_throttle
                } else {
                    0.0
                }
            }),
            Phase::Ramp => [self.ramp_throttle; LIFT_ENGINES],
            Phase::Step => [self.step_throttle(); LIFT_ENGINES],
            Phase::MomentPulse(axis) => {
                // Engines whose thrust makes the moment positive: left for roll right, front
                // for pitch up, the first yaw pair for yaw.
                let raised: [usize; 2] = match axis {
                    0 => [1, 2],
                    1 => [0, 1],
                    _ => c.yaw_pairs[0],
                };
                let base = self.step_throttle();
                core::array::from_fn(|engine| {
                    let delta = if raised.contains(&engine) {
                        c.moment_throttle
                    } else {
                        -c.moment_throttle
                    };
                    (base + delta).clamp(0.0, 1.0)
                })
            }
            Phase::Rest | Phase::Settle | Phase::Fall | Phase::Done => [0.0; LIFT_ENGINES],
        }
    }
}
