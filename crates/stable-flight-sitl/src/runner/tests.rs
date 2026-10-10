use std::thread;
use std::time::{Duration, Instant};

use assert2::check;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::params::Params;
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

#[test]
fn before_step_runs_once_before_each_tick_and_can_write_params() {
    let mut flight_loop = hover_loop();
    let shutdown = ShutdownSignal::new();
    let trigger = shutdown.clone();
    let mut hooks = 0u64;

    let stats = RealTimeRunner::new(1000.0)
        .expect("valid frequency")
        .run_with(&mut flight_loop, &shutdown, |flight_loop| {
            hooks += 1;
            flight_loop
                .set_param("STAB_RATE_P", 0.01 * hooks as f32)
                .expect("valid value");
            if hooks == 5 {
                trigger.trigger(); // this tick still runs; the next doesn't
            }
        });

    check!(stats.ticks == 5);
    check!(hooks == stats.ticks);
    check!(flight_loop.param("STAB_RATE_P") == Some(0.01 * 5.0));
}

#[test]
fn already_set_signal_never_calls_before_step() {
    let mut flight_loop = hover_loop();
    let shutdown = ShutdownSignal::new();
    shutdown.trigger();
    let mut hooks = 0;

    RealTimeRunner::new(250.0)
        .expect("valid frequency")
        .run_with(&mut flight_loop, &shutdown, |_| hooks += 1);

    check!(hooks == 0);
}

#[test]
fn advance_catches_up_short_lag_and_resyncs_after_a_stall() {
    let period = Duration::from_millis(4);
    // (lag past the advanced deadline, ticks skipped, deadline resynced to now)
    let cases = [
        (Duration::ZERO, 0, false),               // on time
        (Duration::from_millis(2), 0, false),     // half a period: caught up
        (Duration::from_micros(3_999), 0, false), // just under a period
        (Duration::from_millis(4), 1, true),      // exactly one period
        (Duration::from_millis(10), 2, true),     // 2.5 periods
        (Duration::from_secs(60), 15_000, true),  // a long reconnect
    ];
    for (lag, expected_skipped, resynced) in cases {
        let start = Instant::now();
        let mut next_deadline = start;
        let now = start + period + lag;

        let skipped = advance(&mut next_deadline, now, period);

        check!(skipped == expected_skipped, "lag {lag:?}");
        let expected_deadline = if resynced { now } else { start + period };
        check!(next_deadline == expected_deadline, "lag {lag:?}");
    }
}

#[test]
fn advance_never_skips_when_ahead_of_schedule() {
    let period = Duration::from_millis(4);
    let start = Instant::now();
    let mut next_deadline = start;

    check!(advance(&mut next_deadline, start, period) == 0);
    check!(next_deadline == start + period);
}

#[test]
fn resyncs_after_a_stall_instead_of_replaying_missed_ticks() {
    const STALL: Duration = Duration::from_millis(50);
    const TICKS_AFTER_STALL: u64 = 20;
    let mut flight_loop = hover_loop();
    let shutdown = ShutdownSignal::new();
    let trigger = shutdown.clone();
    let mut hooks = 0u64;

    let runner = RealTimeRunner::new(1000.0).expect("valid frequency");
    let stats = runner.run_with(&mut flight_loop, &shutdown, |_| {
        hooks += 1;
        if hooks == 2 {
            thread::sleep(STALL);
        }
        if hooks == 2 + TICKS_AFTER_STALL {
            trigger.trigger();
        }
    });

    check!(stats.ticks == 2 + TICKS_AFTER_STALL);
    check!(stats.overruns >= 1);
    check!(stats.skipped_ticks >= 40);
    // Replayed ticks would run back-to-back right after the stall; resynced ones are paced.
    let paced = runner.period() * (TICKS_AFTER_STALL as u32 - 1);
    check!(stats.elapsed >= STALL + paced);
}
