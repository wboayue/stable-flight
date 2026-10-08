use std::thread;
use std::time::Duration;

use assert2::check;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::{ConstantCommander, NullVehicle, RecordingTelemetry};
use stable_flight::{AttitudeCommand, Command, ControlInput, FlightLoop};

use super::*;

type TestLoop = FlightLoop<
    ConstantCommander,
    NullVehicle,
    CopterGuidance,
    CopterStabilizer,
    QuadCopterMixer,
    RecordingTelemetry<1>,
>;

/// A hovering quad on a still vehicle: real guidance, stabilizer, and mixer, core test doubles
/// elsewhere.
fn hover_loop() -> TestLoop {
    FlightLoop::new(
        ConstantCommander::new(Command::armed(ControlInput::Attitude(AttitudeCommand {
            throttle: 0.5,
            ..Default::default()
        }))),
        NullVehicle::new(),
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        RecordingTelemetry::<1>::new(),
    )
}

#[test]
fn rejects_non_positive_or_non_finite_frequencies() {
    // Non-positive, non-finite, too slow to fit a Duration, too fast for a non-zero period.
    for hz in [0.0, -50.0, f64::NAN, f64::INFINITY, 1e-30, 1e10] {
        assert2::assert!(let Err(InvalidFrequency(reported)) = RealTimeRunner::new(hz));
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
        (300.0, Duration::from_nanos(3_333_333)),
    ];
    for (hz, expected) in cases {
        assert2::assert!(let Ok(runner) = RealTimeRunner::new(hz));
        check!(runner.period() == expected, "frequency {hz}");
    }
}

#[test]
fn already_set_signal_primes_but_runs_no_ticks() {
    let mut flight_loop = hover_loop();
    let shutdown = ShutdownSignal::new();
    shutdown.trigger();

    let stats = RealTimeRunner::new(250.0)
        .expect("valid frequency")
        .run(&mut flight_loop, &shutdown);

    check!(stats.ticks == 0);
    check!(stats.average_hz().is_none());
    let (_, vehicle, _, _, _, telemetry) = flight_loop.into_parts();
    check!(vehicle.estimates() == 1); // the priming estimate only
    check!(telemetry.total() == 0); // nothing published
}

#[test]
fn runs_until_signalled_and_steps_with_the_nominal_period() {
    let mut flight_loop = hover_loop();
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
    check!(stats.average_hz().is_some());
    let (_, vehicle, _, _, _, telemetry) = flight_loop.into_parts();
    check!(vehicle.estimates() == stats.ticks + 1); // priming + one per tick
    check!(vehicle.actuations() == stats.ticks);
    check!(telemetry.total() == stats.ticks);
    check!(telemetry.last().map(|tick| tick.dt) == Some(Duration::from_millis(1)));
}
