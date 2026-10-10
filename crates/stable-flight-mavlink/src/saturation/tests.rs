use assert2::check;

use super::*;

fn names_and_values(flags: &Saturation) -> Vec<(String, i32)> {
    saturation(flags, 0)
        .iter()
        .map(|data| (data.name.to_str().expect("ASCII").to_owned(), data.value))
        .collect()
}

#[test]
fn reports_each_flag_under_its_own_name() {
    let set = |f: fn(&mut Saturation)| {
        let mut flags = Saturation::default();
        f(&mut flags);
        flags
    };
    // (case, flags, the one name expected at 1)
    let cases: [(&str, Saturation, &str); 7] = [
        ("tilt", set(|s| s.tilt = true), "SAT_TILT"),
        ("thrust", set(|s| s.thrust = true), "SAT_THR"),
        ("roll moment", set(|s| s.moments[0] = true), "SAT_ROLL"),
        ("pitch moment", set(|s| s.moments[1] = true), "SAT_PITCH"),
        ("yaw moment", set(|s| s.moments[2] = true), "SAT_YAW"),
        ("mixer thrust", set(|s| s.mixer_thrust = true), "SAT_MIXTHR"),
        (
            "mixer moments",
            set(|s| s.mixer_moments = true),
            "SAT_MIXMOM",
        ),
    ];
    for (case, flags, name) in cases {
        let set: Vec<_> = names_and_values(&flags)
            .into_iter()
            .filter(|(_, value)| *value == 1)
            .map(|(name, _)| name)
            .collect();
        check!(set == [name], "{case}");
    }
}

#[test]
fn reports_every_flag_clear_when_none_is_set() {
    let values = names_and_values(&Saturation::default());
    check!(values.len() == SATURATION_FLAGS);
    check!(values.iter().all(|(_, value)| *value == 0));
}

#[test]
fn stamps_every_value_with_the_time() {
    check!(
        saturation(&Saturation::default(), 1234)
            .iter()
            .all(|data| data.time_boot_ms == 1234)
    );
}

#[test]
fn maps_every_core_flag() {
    // `Saturation` is non-exhaustive, so the builder can't destructure it exhaustively. Every
    // flag is a one-byte `bool`: a flag added to the core changes the size and fails this until
    // it is mapped (and `SATURATION_FLAGS` bumped).
    check!(size_of::<Saturation>() == SATURATION_FLAGS);
}
