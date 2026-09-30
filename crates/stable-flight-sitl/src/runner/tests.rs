use std::cell::Cell;
use std::rc::Rc;
use std::thread;
use std::time::Duration;

use assert2::{check, let_assert};
use stable_flight::{
    Actuator, ActuatorCommand, AttitudeCommand, Command, Commander, ControlOutput, FlightLoop,
    Mixer, Stabilizer, StateEstimator, TelemetryPublisher, TelemetryRecord, VehicleState,
};

use super::*;

struct MockCommander;
impl Commander for MockCommander {
    fn next_command(&mut self, _: f32, _: &VehicleState) -> Command {
        Command::Attitude(AttitudeCommand::default())
    }
}

/// Counts estimates so tests can tell priming (the first) from ticks.
struct CountingVehicle {
    estimates: Rc<Cell<u32>>,
}
impl StateEstimator for CountingVehicle {
    fn estimate_state(&mut self, _: Duration) -> VehicleState {
        self.estimates.set(self.estimates.get() + 1);
        VehicleState::default()
    }
}
impl Actuator for CountingVehicle {
    fn actuate(&mut self, _: &ActuatorCommand) {}
}

struct MockStabilizer;
impl Stabilizer for MockStabilizer {
    fn stabilize(&mut self, _: f32, _: &Command, _: &VehicleState) -> ControlOutput {
        ControlOutput::default()
    }
    fn prime(&mut self, _: &VehicleState) {}
}

struct MockMixer;
impl Mixer for MockMixer {
    fn mix(&self, _: &ControlOutput) -> ActuatorCommand {
        ActuatorCommand::default()
    }
}

/// Records the `dt` of the last published tick.
struct DtTelemetry {
    last_dt: Rc<Cell<Option<Duration>>>,
}
impl TelemetryPublisher for DtTelemetry {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.last_dt.set(Some(record.dt));
    }
}

type TestLoop = FlightLoop<MockCommander, CountingVehicle, MockStabilizer, MockMixer, DtTelemetry>;

/// A loop plus handles to what its mocks observed.
struct Harness {
    flight_loop: TestLoop,
    estimates: Rc<Cell<u32>>,
    last_dt: Rc<Cell<Option<Duration>>>,
}

fn harness() -> Harness {
    let estimates = Rc::new(Cell::new(0));
    let last_dt = Rc::new(Cell::new(None));
    let flight_loop = FlightLoop::new(
        MockCommander,
        CountingVehicle {
            estimates: estimates.clone(),
        },
        MockStabilizer,
        MockMixer,
        DtTelemetry {
            last_dt: last_dt.clone(),
        },
    );
    Harness {
        flight_loop,
        estimates,
        last_dt,
    }
}

#[test]
fn rejects_non_positive_or_non_finite_frequencies() {
    for hz in [0.0, -50.0, f64::NAN, f64::INFINITY] {
        let_assert!(Err(InvalidFrequency(reported)) = RealTimeRunner::new(hz));
        check!(reported.to_bits() == hz.to_bits(), "frequency {hz}");
    }
}

#[test]
fn period_is_the_reciprocal_of_frequency() {
    // (frequency Hz, expected period)
    let cases = [
        (250.0, Duration::from_millis(4)),
        (1000.0, Duration::from_millis(1)),
        (50.0, Duration::from_millis(20)),
    ];
    for (hz, expected) in cases {
        let_assert!(Ok(runner) = RealTimeRunner::new(hz));
        check!(runner.period() == expected, "frequency {hz}");
    }
}

#[test]
fn already_set_signal_primes_but_runs_no_ticks() {
    let Harness {
        mut flight_loop,
        estimates,
        last_dt,
    } = harness();
    let shutdown = ShutdownSignal::new();
    shutdown.trigger();

    let stats = RealTimeRunner::new(250.0)
        .expect("valid frequency")
        .run(&mut flight_loop, &shutdown);

    check!(stats.ticks == 0);
    check!(stats.average_hz().is_none());
    check!(estimates.get() == 1); // the priming estimate only
    check!(last_dt.get().is_none()); // nothing published
}

#[test]
fn runs_until_signalled_and_steps_with_the_nominal_period() {
    let Harness {
        mut flight_loop,
        estimates,
        last_dt,
    } = harness();
    let shutdown = ShutdownSignal::new();
    let trigger = shutdown.clone();
    let stopper = thread::spawn(move || {
        thread::sleep(Duration::from_millis(30));
        trigger.trigger();
    });

    let stats = RealTimeRunner::new(1000.0)
        .expect("valid frequency")
        .run(&mut flight_loop, &shutdown);
    stopper.join().expect("stopper thread");

    check!(stats.ticks > 0);
    check!(u64::from(estimates.get()) == stats.ticks + 1); // priming + one per tick
    check!(last_dt.get() == Some(Duration::from_millis(1)));
    check!(stats.average_hz().is_some());
}
