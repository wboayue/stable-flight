use approx::assert_relative_eq;
use assert2::check;

use super::*;

const HEADER: &str =
    "wall_s,armed,mode,altitude,roll,pitch,sp_kind,sp_roll,sp_pitch,north_m,east_m";

/// An altitude in m against time in s.
type Profile = fn(f32) -> f32;

/// Tick spacing of the synthetic laps, in s.
const TICK_S: f32 = 0.1;

/// One tick of a synthetic lap; `sp_*` NaN means no attitude setpoint.
#[derive(Clone, Copy)]
struct Tick {
    armed: bool,
    mode: u8,
    altitude_m: f32,
    roll_rad: f32,
    pitch_rad: f32,
    sp_roll_rad: f32,
    sp_pitch_rad: f32,
    north_m: f32,
    east_m: f32,
}

/// Armed level plane flight at 100 m, on its setpoint, over the start.
const PLANE: Tick = Tick {
    armed: true,
    mode: 1,
    altitude_m: 100.0,
    roll_rad: 0.0,
    pitch_rad: 0.0,
    sp_roll_rad: 0.0,
    sp_pitch_rad: 0.0,
    north_m: 0.0,
    east_m: 0.0,
};

/// A recording of `ticks`, [`TICK_S`] apart.
fn lap(ticks: &[Tick]) -> Recording {
    let mut csv = format!("{HEADER}\n");
    for (i, t) in ticks.iter().enumerate() {
        csv += &format!(
            "{},{},{},{},{},{},plane,{},{},{},{}\n",
            i as f32 * TICK_S,
            u8::from(t.armed),
            t.mode,
            t.altitude_m,
            t.roll_rad,
            t.pitch_rad,
            t.sp_roll_rad,
            t.sp_pitch_rad,
            t.north_m,
            t.east_m
        );
    }
    Recording::parse(&csv).expect("a recording")
}

/// `seconds` of plane ticks with altitude `altitude_m(time_s)`.
fn plane_profile(seconds: f32, altitude_m: impl Fn(f32) -> f32) -> Vec<Tick> {
    (0..=(seconds / TICK_S).round() as usize)
        .map(|i| Tick {
            altitude_m: altitude_m(i as f32 * TICK_S),
            ..PLANE
        })
        .collect()
}

fn stats(ticks: &[Tick]) -> LapStats {
    LapStats::from_recording(&lap(ticks)).expect("lap stats")
}

/// Plane time runs from the first plane row to the last; copter rows around it don't count.
#[test]
fn times_plane_flight() {
    let copter = Tick { mode: 0, ..PLANE };
    let mut ticks = vec![copter; 10];
    ticks.extend(plane_profile(20.0, |_| 100.0));
    ticks.extend([copter; 10]);
    assert_relative_eq!(stats(&ticks).plane_s, 20.0, epsilon = 1e-3);
}

/// Peak sink is the fastest loss over a 0.5 s window, down positive; climbing reads 0.
#[test]
fn measures_peak_sink_over_half_a_second() {
    // (name, altitude profile, expected peak sink m/s)
    let rows: [(&str, Profile, f32); 3] = [
        ("steady 2 m/s descent", |t| 100.0 - 2.0 * t, 2.0),
        ("level", |_| 100.0, 0.0),
        ("climb", |t| 100.0 + 3.0 * t, 0.0),
    ];
    for (name, profile, expected) in rows {
        let sink = stats(&plane_profile(10.0, profile)).peak_sink_mps;
        check!((sink - expected).abs() < 1e-3, "{name}: {sink}");
    }
}

/// A drop between rows further apart than the window is a gap, not a sink.
#[test]
fn ignores_sinks_across_sparse_rows() {
    let recording = Recording::parse(
        "wall_s,armed,mode,altitude,roll,pitch,sp_roll,sp_pitch,north_m,east_m
0.0,1,1,100,0,0,0,0,0,0
0.3,1,1,90,0,0,0,0,0,0
0.6,1,1,80,0,0,0,0,0,0
0.9,1,1,70,0,0,0,0,0,0",
    )
    .expect("a recording");
    let stats = LapStats::from_recording(&recording).expect("lap stats");
    check!(stats.peak_sink_mps == 0.0);
}

/// Sinks 4 m and climbs back, every 8 s.
fn sag_m(time_s: f32) -> f32 {
    100.0 - 4.0 * (time_s * core::f32::consts::FRAC_PI_4).sin()
}

/// A sag the airframe climbs back from within 30 s is a dip; a descent it doesn't climb back
/// from isn't, nor one recovered only after 30 s. Holding the handoff's altitude before the
/// sag doesn't end it.
#[test]
fn measures_the_dip_after_the_handoff() {
    // (name, altitude profile, expected dip m)
    let rows: [(&str, Profile, Option<f32>); 5] = [
        ("sag", sag_m, Some(4.0)),
        (
            "level, then a sag",
            |t| sag_m((t - 1.0).max(0.0)),
            Some(4.0),
        ),
        ("descent", |t| 100.0 - 4.0 * t.min(25.0), None),
        (
            "recovered after 30 s",
            |t| 90.0 + (t - 32.0).abs().min(10.0),
            None,
        ),
        ("climb", |t| 100.0 + t, Some(0.0)),
    ];
    for (name, profile, expected) in rows {
        let dip = stats(&plane_profile(40.0, profile)).handoff_dip_m;
        check!(
            dip.map(|d| (d * 100.0).round() / 100.0) == expected,
            "{name}: {dip:?}"
        );
    }
}

/// Tracking RMS reads the setpoint error in degrees over rows with a setpoint; peak bank is the
/// largest |roll| in plane, either side.
#[test]
fn measures_tracking_and_bank_in_plane() {
    let mut ticks = vec![
        Tick {
            sp_roll_rad: 0.01,
            sp_pitch_rad: -0.02,
            roll_rad: -0.4,
            ..PLANE
        };
        5
    ];
    // No setpoint: left out of the RMS whatever the attitude.
    ticks.push(Tick {
        sp_roll_rad: f32::NAN,
        sp_pitch_rad: f32::NAN,
        pitch_rad: 1.0,
        ..PLANE
    });
    // A copter row's bank isn't plane bank.
    ticks.push(Tick {
        mode: 0,
        roll_rad: 0.9,
        ..PLANE
    });
    let stats = stats(&ticks);
    assert_relative_eq!(stats.roll_rms_deg, 0.41_f32.to_degrees(), epsilon = 1e-3);
    assert_relative_eq!(stats.pitch_rms_deg, 0.02_f32.to_degrees(), epsilon = 1e-3);
    assert_relative_eq!(stats.peak_bank_deg, 0.4_f32.to_degrees(), epsilon = 1e-3);
}

/// Touchdown is the last armed row's distance from the first row, not the last row's.
#[test]
fn measures_touchdown_from_the_start() {
    let ticks = [
        Tick {
            north_m: 10.0,
            east_m: 20.0,
            ..PLANE
        },
        Tick {
            north_m: 13.0,
            east_m: 24.0,
            mode: 0,
            ..PLANE
        },
        Tick {
            north_m: 50.0,
            east_m: 50.0,
            armed: false,
            mode: 0,
            ..PLANE
        },
    ];
    assert_relative_eq!(stats(&ticks).touchdown_m, 5.0, epsilon = 1e-4);
}

/// A copter recording, a lap that never reaches plane, and one never armed have no metrics.
#[test]
fn rejects_laps_without_plane_flight() {
    let copter = Recording::parse(
        "wall_s,armed,roll,pitch,altitude,sp_roll,sp_pitch,north_m,east_m\n0,1,0,0,1,0,0,0,0",
    )
    .expect("a recording");
    check!(let Err(LapError::MissingColumn("mode")) = LapStats::from_recording(&copter));
    let hover = lap(&[Tick { mode: 0, ..PLANE }]);
    check!(let Err(LapError::NoPlaneFlight) = LapStats::from_recording(&hover));
    let disarmed = lap(&[Tick {
        armed: false,
        ..PLANE
    }]);
    check!(let Err(LapError::NeverArmed) = LapStats::from_recording(&disarmed));
}

/// A table row has every metric in the header's order, a missing dip as "—", and the file's
/// stem as its name.
#[test]
fn formats_a_table_row() {
    let stats = LapStats {
        plane_s: 426.2,
        peak_sink_mps: 3.524,
        handoff_dip_m: Some(4.04),
        roll_rms_deg: 1.701,
        pitch_rms_deg: 0.594,
        peak_bank_deg: 22.24,
        touchdown_m: 0.158,
    };
    check!(
        table_row(&lap_name(Path::new("runs/537-tB.csv")), &stats)
            == "| 537-tB | 426 s | 3.52 m/s | 4.0 m | 1.70° / 0.59° | 22.2° | 0.16 m |"
    );
    let descent = LapStats {
        handoff_dip_m: None,
        ..stats
    };
    check!(table_row("d", &descent).contains("| — |"));
}
