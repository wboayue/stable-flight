use assert2::check;

use super::*;
use crate::stabilizers::copter::TunedLoop;
use crate::tuning::{ControllerForm, RelayResult, UltimatePoint, gains_from_ultimate};

const CURRENT: Gains = Gains {
    p: 9.0,
    i: 8.0,
    d: 7.0,
};

fn measured(ku: f32, tu_s: f32) -> Input {
    Input::Measured {
        measurement: Measurement::Relay(RelayResult {
            point: UltimatePoint { ku, tu_s },
            relay: 0.2,
            cycle_amplitude: 0.1,
            bias: 0.0,
        }),
        gains: CURRENT,
    }
}

/// A relay stage asks for its relay first, then finishes with the rule's gains for the
/// measured point.
#[test]
fn relay_probes_once_then_finishes_with_the_rules_gains() {
    let plan = RelayPlan::copter(TunedLoop::RollRate);
    let mut procedure = Procedure::new(Method::Relay(plan));

    let first = procedure.next(Input::Start { gains: CURRENT });
    check!(first == Action::Probe(ProbeConfig::Relay(plan.relay)));

    let rule = gains_from_ultimate(&UltimatePoint { ku: 2.0, tu_s: 0.1 }, plan.rule, plan.form);
    let last = procedure.next(measured(2.0, 0.1));
    check!(
        last == Action::Finish(Gains {
            p: rule.p,
            i: CURRENT.i,
            d: rule.d,
        })
    );
}

/// A relay stage sets the rule's gains for the terms its form tunes and keeps the current
/// gains for the rest.
#[test]
fn relay_tunes_only_its_forms_terms() {
    // (form, tunes I, tunes D)
    let cases = [
        (ControllerForm::Pid, true, true),
        (ControllerForm::Pi, true, false),
        (ControllerForm::Pd, false, true),
        (ControllerForm::P, false, false),
    ];
    for (form, tunes_i, tunes_d) in cases {
        let plan = RelayPlan {
            form,
            ..RelayPlan::copter(TunedLoop::RollRate)
        };
        let rule = gains_from_ultimate(&UltimatePoint { ku: 2.0, tu_s: 0.1 }, plan.rule, form);
        let mut procedure = Procedure::new(Method::Relay(plan));

        let Action::Finish(gains) = procedure.next(measured(2.0, 0.1)) else {
            panic!("{form:?}: no gains");
        };

        check!(gains.p == rule.p, "{form:?}");
        check!(
            gains.i == if tunes_i { rule.i } else { CURRENT.i },
            "{form:?}"
        );
        check!(
            gains.d == if tunes_d { rule.d } else { CURRENT.d },
            "{form:?}"
        );
    }
}
