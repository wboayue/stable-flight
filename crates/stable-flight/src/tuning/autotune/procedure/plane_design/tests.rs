use assert2::{assert, check};

use super::*;
use crate::controllers::RateGains;
use crate::stabilizers::plane::{PlaneLoop, PlaneStabilizerConfig};
use crate::tuning::{IdentifyAbort, StepAbort, WingModel};

/// A roll axis like the test plant's wing.
const ROLL: WingModel = WingModel {
    stiffness_per_s2: 0.0,
    damping_per_s: 8.0,
    control_radps2: 40.0,
    lag_s: 0.0,
    delay_s: 0.0,
    path_turn_per_s: 0.0,
    trim_moment: 0.0,
};

/// The roll defaults, the design run in one update.
fn plan() -> PlaneDesignPlan {
    PlaneDesignPlan {
        candidates_per_update: usize::MAX,
        ..PlaneDesignPlan::for_plane(PlaneLoop::Roll, &PlaneStabilizerConfig::default())
    }
}

fn fitted(model: WingModel) -> Decide {
    Decide::Measured {
        measurement: Measurement::Wing(model),
        stalled: false,
    }
}

fn stepped(metrics: StepMetrics) -> Decide {
    Decide::Measured {
        measurement: Measurement::Step(metrics),
        stalled: false,
    }
}

/// A step that meets every target at a time constant of `tau_s`.
fn good_step(tau_s: f32) -> StepMetrics {
    StepMetrics {
        overshoot: 0.02,
        rise_time_s: Some(2.0 * tau_s),
        settling_time_s: Some(3.0 * tau_s),
        ..StepMetrics::default()
    }
}

/// A design fitted on `ROLL` and designed: the candidate it holds first.
fn designed(design: &mut PlaneDesign) -> PlaneAxisGains {
    check!(design.next(Decide::Start) == Action::probe(ProbeConfig::IdentifyWing(plan().identify)));
    check!(design.next(fitted(ROLL)) == Action::Compute);
    match design.next(Decide::Computing) {
        Action::Hold { gains } => gains,
        other => panic!("expected a hold on the candidate, got {other:?}"),
    }
}

/// The fit, the design, then each step of a pair held and flown on the candidate; a passing
/// pair finishes on it, tuned.
#[test]
fn fits_designs_and_verifies_a_pair() {
    let mut design = PlaneDesign::new(plan());
    let candidate = designed(&mut design);
    check!(candidate.rate.ff == ROLL.feedforward());
    let step = plan().verify;
    let tau_s = candidate.time_constant_s;

    let probe = |step| Action::Probe {
        probe: ProbeConfig::Step(step),
        gains: Some(candidate),
        commit: None,
    };
    check!(design.next(Decide::Held { settled: true }) == probe(step));
    check!(design.next(stepped(good_step(tau_s))) == Action::Hold { gains: candidate });
    check!(design.next(Decide::Held { settled: true }) == probe(step.reversed()));
    let Action::Finish {
        gains,
        outcome: Outcome::Tuned,
        steps: Some(steps),
        baseline: None,
    } = design.next(stepped(good_step(tau_s)))
    else {
        panic!("no finish");
    };
    check!(gains == candidate);
    check!(steps.count() == 2);
}

/// A design spread over updates computes until it has a candidate.
#[test]
fn computes_over_updates() {
    let mut design = PlaneDesign::new(PlaneDesignPlan {
        candidates_per_update: 1,
        ..plan()
    });
    design.next(Decide::Start);
    check!(design.next(fitted(ROLL)) == Action::Compute);
    let mut updates = 0;
    while design.next(Decide::Computing) == Action::Compute {
        updates += 1;
    }
    check!(updates > 1);
}

/// A pair's second step at a time constant, in s.
type SecondStep = fn(f32) -> StepMetrics;

/// How a miss changes the candidate.
type Nudge = fn(PlaneAxisGains) -> PlaneAxisGains;

/// A pair that misses nudges the candidate for the next: a sag (a step that doesn't settle)
/// or a step that holds past its target (keeps over half its overshoot) raises I by half, a
/// settled step that rises slowly raises P a grid step, an overshoot that swings back or a
/// clip raises the time constant by 15%, and one that swings back but ends past the target
/// raises both.
#[test]
fn a_missed_pair_nudges_the_candidate() {
    // (row, the second step, the candidate change)
    let rows: [(&str, SecondStep, Nudge); 7] = [
        (
            "sags",
            |tau_s| StepMetrics {
                settling_time_s: None,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                rate: RateGains {
                    i: (gains.rate.i * SAG_I_FACTOR).max(MIN_SAG_I),
                    ..gains.rate
                },
                ..gains
            },
        ),
        (
            "rises slowly",
            |tau_s| StepMetrics {
                rise_time_s: Some(3.0 * tau_s),
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                rate: RateGains {
                    p: gains.rate.p * SLOW_P_FACTOR,
                    ..gains.rate
                },
                ..gains
            },
        ),
        (
            "overshoots",
            |tau_s| StepMetrics {
                overshoot: 0.2,
                bounce_back: 0.15,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                time_constant_s: gains.time_constant_s * OVERSHOOT_TAU_FACTOR,
                ..gains
            },
        ),
        (
            "holds past",
            |tau_s| StepMetrics {
                overshoot: 0.12,
                bounce_back: 0.02,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                rate: RateGains {
                    i: (gains.rate.i * SAG_I_FACTOR).max(MIN_SAG_I),
                    ..gains.rate
                },
                ..gains
            },
        ),
        (
            "swings back, ends past",
            |tau_s| StepMetrics {
                overshoot: 0.06,
                bounce_back: 0.03,
                final_value: 1.03,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                rate: RateGains {
                    i: (gains.rate.i * SAG_I_FACTOR).max(MIN_SAG_I),
                    ..gains.rate
                },
                time_constant_s: gains.time_constant_s * OVERSHOOT_TAU_FACTOR,
            },
        ),
        (
            "gives back half",
            |tau_s| StepMetrics {
                overshoot: 0.12,
                bounce_back: 0.06,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                time_constant_s: gains.time_constant_s * OVERSHOOT_TAU_FACTOR,
                ..gains
            },
        ),
        (
            "clips",
            |tau_s| StepMetrics {
                saturated_s: 0.5,
                ..good_step(tau_s)
            },
            |gains| PlaneAxisGains {
                time_constant_s: gains.time_constant_s * OVERSHOOT_TAU_FACTOR,
                ..gains
            },
        ),
    ];
    for (row, second, nudged) in rows {
        let mut design = PlaneDesign::new(plan());
        let candidate = designed(&mut design);
        let tau_s = candidate.time_constant_s;
        design.next(Decide::Held { settled: true });
        design.next(stepped(good_step(tau_s)));
        design.next(Decide::Held { settled: true });
        let next = design.next(stepped(second(tau_s)));
        check!(
            next == Action::Hold {
                gains: nudged(candidate)
            },
            "{row}"
        );
    }
}

/// A candidate that won't settle on its hold, runs away, or leaves the envelope flies its
/// next pair slower.
#[test]
fn a_candidate_that_asks_too_much_slows() {
    let rows = [
        ("unsettled on its hold", Decide::Held { settled: false }),
        (
            "ran away",
            Decide::Aborted {
                abort: ProbeAbort::Step(StepAbort::Runaway),
            },
        ),
        (
            "breached",
            Decide::Breached {
                reason: AutotuneAbort::Tilt,
            },
        ),
    ];
    for (row, on) in rows {
        let mut design = PlaneDesign::new(plan());
        let candidate = designed(&mut design);
        let slower = PlaneAxisGains {
            time_constant_s: candidate.time_constant_s * OVERSHOOT_TAU_FACTOR,
            ..candidate
        };
        check!(design.next(on) == Action::Hold { gains: slower }, "{row}");
    }
}

/// Past the plan's pairs the stage fails unverified.
#[test]
fn fails_after_its_pairs() {
    let mut design = PlaneDesign::new(PlaneDesignPlan {
        max_verify_pairs: 2,
        ..plan()
    });
    designed(&mut design);
    let mut last = Action::Compute;
    for _ in 0..2 {
        last = design.next(Decide::Held { settled: false });
    }
    check!(last == Action::Fail(AutotuneAbort::Unverified));
}

/// Three fits the design can't use fail the stage: an unexcited fit, or a model no gains fly.
#[test]
fn fails_on_unusable_fits() {
    let unexcited = Decide::Aborted {
        abort: ProbeAbort::Identify(IdentifyAbort::Unexcited),
    };
    let mut design = PlaneDesign::new(plan());
    design.next(Decide::Start);
    let refit = Action::probe(ProbeConfig::IdentifyWing(plan().identify));
    check!(design.next(unexcited) == refit);
    check!(design.next(unexcited) == refit);
    check!(
        design.next(unexcited)
            == Action::Fail(AutotuneAbort::Probe(ProbeAbort::Identify(
                IdentifyAbort::Unexcited
            )))
    );

    let reversed = WingModel {
        control_radps2: -40.0,
        ..ROLL
    };
    let mut design = PlaneDesign::new(plan());
    design.next(Decide::Start);
    for _ in 0..2 {
        design.next(fitted(reversed));
        check!(design.next(Decide::Computing) == refit);
    }
    design.next(fitted(reversed));
    check!(
        design.next(Decide::Computing)
            == Action::Fail(AutotuneAbort::Design(DesignError::NoWingDesign))
    );
}

/// A slow host's stops repeat the probe, three in a row fail the stage; a breach or an
/// unsettled vehicle while fitting fails it.
#[test]
fn fails_on_stalls_breaches_and_no_settle() {
    let mut design = PlaneDesign::new(plan());
    design.next(Decide::Start);
    let refit = Action::probe(ProbeConfig::IdentifyWing(plan().identify));
    check!(design.next(Decide::Stalled) == refit);
    check!(design.next(Decide::Stalled) == refit);
    check!(design.next(Decide::Stalled) == Action::Fail(AutotuneAbort::Stalled));

    let rows = [
        (
            Decide::Breached {
                reason: AutotuneAbort::AirspeedLow,
            },
            AutotuneAbort::AirspeedLow,
        ),
        (Decide::Unsettled, AutotuneAbort::NotSettled),
    ];
    for (on, reason) in rows {
        let mut design = PlaneDesign::new(plan());
        design.next(Decide::Start);
        assert!(design.next(on) == Action::Fail(reason));
    }
}
