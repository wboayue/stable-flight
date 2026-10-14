use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::perturbations::Vibration;

#[test]
fn off_draws_nothing() {
    let mut gyro = Gyro::new(GyroNoise {
        seed: 3,
        ..Default::default()
    });
    check!(gyro.error_radps(1.0).is_none());
    check!(gyro.rng == SplitMix64(3));
}

#[test]
fn normal_draws_have_zero_mean_and_unit_variance() {
    const SAMPLES: usize = 100_000;
    let mut rng = SplitMix64(42);
    let draws: Vec<f32> = (0..SAMPLES).map(|_| rng.normal()).collect();
    let mean = draws.iter().sum::<f32>() / SAMPLES as f32;
    let variance = draws.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>() / SAMPLES as f32;
    // Standard error of the mean is 1/√N ≈ 0.003; of the variance √(2/N) ≈ 0.0045.
    check!(mean.abs() < 0.015);
    assert_relative_eq!(variance, 1.0, epsilon = 0.02);
}

#[test]
fn uniform_stays_in_the_half_open_unit_interval() {
    let mut rng = SplitMix64(0);
    for _ in 0..10_000 {
        let u = rng.uniform();
        check!(u > 0.0 && u <= 1.0);
    }
}

#[test]
fn one_seed_repeats_and_another_differs() {
    let noise = |seed| GyroNoise {
        rms_radps: Vec3::repeat(0.1),
        vibration: None,
        seed,
    };
    let run = |seed| {
        let mut gyro = Gyro::new(noise(seed));
        (0..100)
            .map(|i| gyro.error_radps(f64::from(i) * 0.004))
            .collect::<Vec<_>>()
    };
    check!(run(1) == run(1));
    check!(run(1) != run(2));
}

#[test]
fn vibration_follows_its_sine() {
    let mut gyro = Gyro::new(GyroNoise {
        vibration: Some(Vibration {
            amplitude_radps: Vec3::new(0.1, 0.2, 0.0),
            frequency_hz: 50.0,
        }),
        ..Default::default()
    });
    // (time s, fraction of amplitude): 50 Hz has a 20 ms period.
    let cases = [(0.0, 0.0), (0.005, 1.0), (0.015, -1.0), (1000.005, 1.0)];
    for (time_s, fraction) in cases {
        let error_radps = gyro.error_radps(time_s).unwrap_or_default();
        assert_relative_eq!(
            error_radps,
            Vec3::new(0.1, 0.2, 0.0) * fraction,
            epsilon = 1e-5
        );
    }
}
