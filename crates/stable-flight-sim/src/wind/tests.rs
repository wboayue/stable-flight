use approx::assert_relative_eq;

use super::*;

const GUST: Gust = Gust {
    start_s: 2.0,
    duration_s: 4.0,
    peak_mps: Vec3::new(2.0, -4.0, 1.0),
};

#[test]
fn gust_follows_one_minus_cosine() {
    // (time s, fraction of peak)
    let cases = [
        (0.0, 0.0),
        (1.99, 0.0),
        (2.0, 0.0),
        (3.0, 0.5),
        (4.0, 1.0),
        (5.0, 0.5),
        (6.0, 0.0),
        (6.01, 0.0),
    ];
    for (time_s, fraction) in cases {
        let expected = GUST.peak_mps * fraction;
        assert_relative_eq!(GUST.velocity_mps(time_s), expected, epsilon = 1e-5);
    }
}

#[test]
fn steady_wind_adds_to_the_gust() {
    let steady_mps = Vec3::new(0.0, 3.0, 0.0);
    let wind = Wind {
        steady_mps,
        gust: Some(GUST),
    };
    assert_relative_eq!(wind.velocity_mps(0.0), steady_mps);
    assert_relative_eq!(
        wind.velocity_mps(4.0),
        steady_mps + GUST.peak_mps,
        epsilon = 1e-5
    );
}

#[test]
fn degenerate_gust_is_calm() {
    for duration_s in [0.0, -1.0, f32::NAN] {
        let gust = Gust { duration_s, ..GUST };
        assert_relative_eq!(gust.velocity_mps(GUST.start_s), Vec3::zeros());
    }
}

#[test]
fn default_wind_is_calm() {
    assert_relative_eq!(Wind::default().velocity_mps(10.0), Vec3::zeros());
}
