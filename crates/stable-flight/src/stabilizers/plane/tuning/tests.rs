use assert2::{assert, check};

use super::*;
use crate::controllers::PlaneRateControllerConfig;
use crate::tuning::{Measurement, StepConfig};
use crate::{BodyRateSetpoint, PlaneSetpoint, Setpoint, Stabilizer, Vec2, Vec3, VehicleState};
use nalgebra::UnitQuaternion;

/// The flight loop's tick: 250 Hz.
const DT: f32 = 0.004;

/// A wing's roll and pitch, each a rate driven by its moment against its damping: rate
/// acceleration = control × moment − damping × rate, angles integrated from the rates (small
/// angles). Gentle enough for the default gains to fly.
#[derive(Default)]
struct Wing {
    angles_rad: Vec2,
    rates_radps: Vec2,
}

/// Rate acceleration per unit moment, rad/s².
const CONTROL_RADPS2: f32 = 40.0;
/// Rate damping, 1/s.
const DAMPING_PER_S: f32 = 8.0;

impl Wing {
    fn state(&self) -> VehicleState {
        VehicleState {
            attitude: UnitQuaternion::from_euler_angles(self.angles_rad.x, self.angles_rad.y, 0.0),
            angular_velocity_radps: Vec3::new(self.rates_radps.x, self.rates_radps.y, 0.0),
            ..Default::default()
        }
    }

    fn step(&mut self, moments: Vec3) {
        for axis in 0..2 {
            let accel = CONTROL_RADPS2 * moments[axis] - DAMPING_PER_S * self.rates_radps[axis];
            self.rates_radps[axis] += accel * DT;
            self.angles_rad[axis] += self.rates_radps[axis] * DT;
        }
    }
}

/// Gains that fly the [`Wing`]: feedforward matching its damping, a little P and I.
fn config() -> PlaneStabilizerConfig {
    let rate = RateGains {
        p: 0.1,
        i: 0.1,
        ff: DAMPING_PER_S / CONTROL_RADPS2,
    };
    PlaneStabilizerConfig {
        rate: PlaneRateControllerConfig {
            roll: rate,
            pitch: rate,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn level() -> Setpoint {
    Setpoint::Plane(PlaneSetpoint {
        thrust: 0.6,
        ..Default::default()
    })
}

fn rates() -> Setpoint {
    Setpoint::PlaneRate(BodyRateSetpoint {
        thrust: 0.6,
        ..Default::default()
    })
}

fn tick(stabilizer: &mut PlaneStabilizer, wing: &mut Wing, setpoint: &Setpoint) -> Vec3 {
    let output = stabilizer.stabilize(DT, setpoint, &wing.state());
    wing.step(output.moments);
    output.moments
}

/// A step on `target`: 0.3 rad/s on a rate loop, 0.2 rad on an angle loop, held 2 s.
fn step_for(target: PlaneLoop) -> ProbeConfig {
    let amplitude = if target.is_rate() { 0.3 } else { 0.2 };
    ProbeConfig::Step(StepConfig {
        amplitude,
        duration_s: 2.0,
        settle_band: 0.1,
        filter_hz: 0.0,
    })
}

fn status(stabilizer: &PlaneStabilizer) -> Option<(PlaneLoop, ProbeStatus)> {
    Tunable::<PlaneLoop>::probe(stabilizer)
}

#[test]
fn a_step_replaces_its_loops_setpoint() {
    for target in [
        PlaneLoop::RollRate,
        PlaneLoop::PitchRate,
        PlaneLoop::Roll,
        PlaneLoop::Pitch,
    ] {
        let mut stabilizer = PlaneStabilizer::new(config());
        let mut wing = Wing::default();
        stabilizer.start_probe(target, step_for(target));
        let moments = tick(&mut stabilizer, &mut wing, &level());
        let axis = target.axis();
        check!(moments[axis] > 0.0, "{target:?} pushes its axis");
        check!(moments[1 - axis] == 0.0, "{target:?} leaves the other axis");
    }
}

#[test]
fn each_step_measures_its_loops_response() {
    for target in [
        PlaneLoop::RollRate,
        PlaneLoop::PitchRate,
        PlaneLoop::Roll,
        PlaneLoop::Pitch,
    ] {
        let mut stabilizer = PlaneStabilizer::new(config());
        let mut wing = Wing::default();
        stabilizer.start_probe(target, step_for(target));
        for _ in 0..(2.5 / DT) as usize {
            tick(&mut stabilizer, &mut wing, &level());
        }
        let Some((_, ProbeStatus::Done(Measurement::Step(metrics)))) = status(&stabilizer) else {
            panic!("{target:?}: {:?}", status(&stabilizer));
        };
        check!(
            metrics.rise_time_s.is_some(),
            "{target:?} rises: {metrics:?}"
        );
        check!(metrics.overshoot < 0.3, "{target:?} settles: {metrics:?}");
        check!(
            metrics.saturated_s == 0.0,
            "{target:?} doesn't clip: {metrics:?}"
        );
    }
}

#[test]
fn a_finished_step_hands_its_loop_back() {
    let mut stabilizer = PlaneStabilizer::new(config());
    let mut wing = Wing::default();
    stabilizer.start_probe(PlaneLoop::Roll, step_for(PlaneLoop::Roll));
    for _ in 0..(2.5 / DT) as usize {
        tick(&mut stabilizer, &mut wing, &level());
    }
    // Back on the level setpoint: the bank the step left is flown back out.
    for _ in 0..(3.0 / DT) as usize {
        tick(&mut stabilizer, &mut wing, &level());
    }
    check!(wing.angles_rad.x.abs() < 0.01);
    check!(matches!(
        status(&stabilizer),
        Some((_, ProbeStatus::Done(_)))
    ));
}

#[test]
fn a_step_counts_the_ticks_its_loop_clipped() {
    // A 6 rad/s step: feedforward alone asks for more than full deflection.
    let mut stabilizer = PlaneStabilizer::new(config());
    let mut wing = Wing::default();
    let step = StepConfig {
        amplitude: 6.0,
        duration_s: 1.0,
        settle_band: 0.1,
        filter_hz: 0.0,
    };
    stabilizer.start_probe(PlaneLoop::RollRate, ProbeConfig::Step(step));
    for _ in 0..(1.5 / DT) as usize {
        tick(&mut stabilizer, &mut wing, &level());
    }
    let Some((_, ProbeStatus::Done(Measurement::Step(metrics)))) = status(&stabilizer) else {
        panic!("{:?}", status(&stabilizer));
    };
    check!(metrics.saturated_s > 0.0, "{metrics:?}");
}

#[test]
fn rate_setpoints_interrupt_angle_stages_only() {
    for (target, interrupted) in [
        (PlaneLoop::Roll, true),
        (PlaneLoop::Pitch, true),
        (PlaneLoop::RollRate, false),
        (PlaneLoop::PitchRate, false),
    ] {
        let mut stabilizer = PlaneStabilizer::new(config());
        let mut wing = Wing::default();
        stabilizer.start_probe(target, step_for(target));
        tick(&mut stabilizer, &mut wing, &rates());
        let status = status(&stabilizer).map(|(_, status)| status);
        check!(
            (status == Some(ProbeStatus::Interrupted)) == interrupted,
            "{target:?}: {status:?}"
        );
    }
}

#[test]
fn leaving_the_plane_side_interrupts_any_stage() {
    let transition = Setpoint::Plane(PlaneSetpoint {
        transition: true,
        thrust: 0.6,
        ..Default::default()
    });
    let hover = Setpoint::Attitude(Default::default());
    for (case, setpoint) in [("transition", transition), ("hover", hover)] {
        let mut stabilizer = PlaneStabilizer::new(config());
        let mut wing = Wing::default();
        stabilizer.start_probe(PlaneLoop::RollRate, step_for(PlaneLoop::RollRate));
        tick(&mut stabilizer, &mut wing, &setpoint);
        check!(
            status(&stabilizer) == Some((PlaneLoop::RollRate, ProbeStatus::Interrupted)),
            "{case}"
        );
        // The loop is the attitude loop's again.
        let moments = tick(&mut stabilizer, &mut wing, &level());
        check!(moments.x.abs() < 0.05, "{case}: {moments:?}");
    }
}

#[test]
fn starting_a_stage_ends_the_running_one() {
    let mut stabilizer = PlaneStabilizer::new(config());
    let mut wing = Wing::default();
    stabilizer.start_probe(PlaneLoop::Roll, step_for(PlaneLoop::Roll));
    tick(&mut stabilizer, &mut wing, &level());
    stabilizer.start_probe(PlaneLoop::PitchRate, step_for(PlaneLoop::PitchRate));
    assert!(matches!(
        status(&stabilizer),
        Some((PlaneLoop::PitchRate, ProbeStatus::Running(_)))
    ));
}

#[test]
fn stop_reports_the_last_status_and_ends_the_stage() {
    let mut stabilizer = PlaneStabilizer::new(config());
    let mut wing = Wing::default();
    stabilizer.start_probe(PlaneLoop::Pitch, step_for(PlaneLoop::Pitch));
    tick(&mut stabilizer, &mut wing, &level());
    let stopped = Tunable::<PlaneLoop>::stop_probe(&mut stabilizer);
    check!(matches!(
        stopped,
        Some((PlaneLoop::Pitch, ProbeStatus::Running(_)))
    ));
    check!(status(&stabilizer).is_none());
    check!(Tunable::<PlaneLoop>::stop_probe(&mut stabilizer).is_none());
}

#[test]
fn prime_ends_the_stage() {
    let mut stabilizer = PlaneStabilizer::new(config());
    stabilizer.start_probe(PlaneLoop::Roll, step_for(PlaneLoop::Roll));
    stabilizer.prime(&VehicleState::default());
    check!(status(&stabilizer).is_none());
}

#[test]
fn gains_read_and_set_the_loops_axis() {
    let mut config = config();
    let gains = PlaneAxisGains {
        rate: RateGains {
            p: 0.3,
            i: 0.2,
            ff: 0.4,
        },
        time_constant_s: 0.7,
    };
    PlaneLoop::Roll.set_gains(&mut config, gains);
    check!(config.rate.roll == gains.rate);
    check!(config.time_constant_s.x == 0.7);
    check!(PlaneLoop::RollRate.gains(&config) == gains);
    check!(PlaneLoop::Pitch.gains(&config) != gains);
}
