use assert2::check;

use super::*;

const DT: f32 = 0.01;

/// Runs `seconds` of the sweep with the ground contact `on_ground(t)`, returning each tick's
/// (time, phase, throttles).
fn run(
    sweep: &mut EngineSweep,
    seconds: f32,
    on_ground: impl Fn(f32) -> bool,
) -> Vec<(f32, Phase, [f32; LIFT_ENGINES])> {
    (1..=(seconds / DT).round() as usize)
        .map(|i| {
            let t = i as f32 * DT;
            let throttles = sweep.step(DT, on_ground(t));
            (t, sweep.phase(), throttles)
        })
        .collect()
}

/// The phase at `t`.
fn phase_at(ticks: &[(f32, Phase, [f32; LIFT_ENGINES])], t: f32) -> (Phase, [f32; LIFT_ENGINES]) {
    let &(_, phase, throttles) = ticks
        .iter()
        .find(|(at, _, _)| (at - t).abs() < DT / 2.0)
        .expect("tick at t");
    (phase, throttles)
}

/// The schedule: rest, pair 0+2, rest, pair 1+3, rest, then the ramp until liftoff, the step on
/// the liftoff throttle, the roll, pitch, and yaw pulses on it, and the fall.
#[test]
fn runs_the_schedule() {
    let mut sweep = EngineSweep::new(SweepConfig::default());
    // Leaves the ground 10 s into the ramp (which starts at 12 s), at throttle 0.5.
    let ticks = run(&mut sweep, 26.0, |t| t < 22.0);

    // (t, phase, throttles)
    let rows = [
        (1.0, Phase::Rest, [0.0; 4]),
        (3.0, Phase::YawPulse(0), [0.4, 0.0, 0.4, 0.0]),
        (6.0, Phase::Rest, [0.0; 4]),
        (8.0, Phase::YawPulse(1), [0.0, 0.4, 0.0, 0.4]),
        (11.0, Phase::Rest, [0.0; 4]),
    ];
    for (t, phase, throttles) in rows {
        check!(phase_at(&ticks, t) == (phase, throttles), "{t}");
    }

    let (phase, throttles) = phase_at(&ticks, 17.0);
    check!(phase == Phase::Ramp);
    check!((throttles[0] - 0.25).abs() < 0.01);

    let liftoff = sweep.liftoff_throttle().expect("lifted off");
    check!((liftoff - 0.5).abs() < 0.01);
    let step = liftoff + 0.1;
    let (phase, throttles) = phase_at(&ticks, 22.2);
    check!(phase == Phase::Step);
    check!(throttles == [step; 4]);
    // (t, axis, engines raised)
    let pulses = [(22.65, 0, [1, 2]), (22.95, 1, [0, 1]), (23.25, 2, [0, 2])];
    for (t, axis, raised) in pulses {
        let (phase, throttles) = phase_at(&ticks, t);
        check!(phase == Phase::MomentPulse(axis), "{t}");
        for (engine, throttle) in throttles.iter().enumerate() {
            let expected = if raised.contains(&engine) {
                step + 0.05
            } else {
                step - 0.05
            };
            check!((throttle - expected).abs() < 1e-6, "{t} engine {engine}");
        }
    }
    check!(phase_at(&ticks, 23.5) == (Phase::Fall, [0.0; 4]));
    check!(phase_at(&ticks, 25.6) == (Phase::Done, [0.0; 4]));
}

/// An aircraft that never leaves the ground ends the ramp at full throttle, with no liftoff
/// throttle and no step.
#[test]
fn gives_up_at_full_throttle() {
    let mut sweep = EngineSweep::new(SweepConfig::default());
    let ticks = run(&mut sweep, 40.0, |_| true);
    check!(sweep.liftoff_throttle() == None);
    check!(
        ticks
            .iter()
            .all(|(_, phase, _)| !matches!(phase, Phase::Step | Phase::MomentPulse(_)))
    );
    check!(sweep.phase() == Phase::Done);
}

/// Each phase has its own code for the capture.
#[test]
fn codes_each_phase() {
    let phases = [
        Phase::Rest,
        Phase::YawPulse(0),
        Phase::YawPulse(1),
        Phase::Settle,
        Phase::Ramp,
        Phase::Step,
        Phase::MomentPulse(0),
        Phase::MomentPulse(1),
        Phase::MomentPulse(2),
        Phase::Fall,
        Phase::Done,
    ];
    for (i, phase) in phases.into_iter().enumerate() {
        check!(phase.code() == i as f32);
    }
}

/// The moment pulses follow the configured layout: a frame numbered the other way round
/// raises its own left and front engines.
#[test]
fn pulses_follow_the_engine_layout() {
    let mut sweep = EngineSweep::new(SweepConfig {
        roll_engines: [0, 3],
        pitch_engines: [2, 3],
        ..SweepConfig::default()
    });
    let ticks = run(&mut sweep, 26.0, |t| t < 22.0);
    // (t, engines raised)
    let pulses = [(22.65, [0, 3]), (22.95, [2, 3])];
    for (t, raised) in pulses {
        let (_, throttles) = phase_at(&ticks, t);
        for (engine, throttle) in throttles.iter().enumerate() {
            let up = *throttle > throttles.iter().copied().fold(f32::INFINITY, f32::min);
            check!(
                up == raised.contains(&engine),
                "{t} engine {engine}: {throttles:?}"
            );
        }
    }
}
