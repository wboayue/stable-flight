use core::f32::consts::{FRAC_PI_2, PI};

use approx::assert_relative_eq;
use assert2::check;

use super::*;
use crate::airframes;

fn model() -> FixedWingModel {
    airframes::plane().model
}

fn lift_at(c: &AeroCoefficients, alpha_rad: f32) -> f32 {
    lift_coefficient(
        c,
        alpha_rad,
        stall_blend(c, alpha_rad),
        libm::sinf(alpha_rad),
        libm::cosf(alpha_rad),
    )
}

/// Air at `speed_mps` along body `x`, tilted by `alpha_rad` (from below) and `beta_rad` (from
/// the right).
fn air(speed_mps: f32, alpha_rad: f32, beta_rad: f32) -> Vec3 {
    Vec3::new(
        libm::cosf(alpha_rad) * libm::cosf(beta_rad),
        libm::sinf(beta_rad),
        libm::sinf(alpha_rad) * libm::cosf(beta_rad),
    ) * speed_mps
}

#[test]
fn no_airspeed_no_aero() {
    let still = wrench(
        &model(),
        Vec3::zeros(),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::repeat(0.3),
        Wash::default(),
    );
    check!(still == Aero::default());
}

#[test]
fn flow_angles_follow_the_air() {
    // (alpha rad, beta rad)
    let cases = [
        (0.0, 0.0),
        (0.1, 0.0),
        (-0.2, 0.0),
        (0.0, 0.15),
        (0.05, -0.1),
    ];
    for (alpha_rad, beta_rad) in cases {
        let aero = wrench(
            &model(),
            air(15.0, alpha_rad, beta_rad),
            Vec3::zeros(),
            Vec3::zeros(),
            Wash::default(),
        );
        assert_relative_eq!(aero.alpha_rad, alpha_rad, epsilon = 1e-5);
        assert_relative_eq!(aero.beta_rad, beta_rad, epsilon = 1e-5);
    }
}

/// Head-on air at zero α: lift up (body −z), drag back (body −x), from the coefficients.
#[test]
fn head_on_lift_and_drag() {
    let model = model();
    let c = &model.aero;
    let aero = wrench(
        &model,
        air(15.0, 0.0, 0.0),
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    );
    let dynamic_n = 0.5 * AIR_DENSITY_KGPM3 * 225.0 * model.wing_area_m2;
    let aspect_ratio = model.span_m * model.span_m / model.wing_area_m2;
    let induced = c.c_lift_0 * c.c_lift_0 / (PI * c.oswald * aspect_ratio);
    check!(pressure_area_n(&model, 15.0) == dynamic_n);
    assert_relative_eq!(drag_coefficient(&model, c.c_lift_0), c.c_drag_0 + induced);
    assert_relative_eq!(aero.force_n.z, -dynamic_n * c.c_lift_0, max_relative = 1e-4);
    assert_relative_eq!(
        aero.force_n.x,
        -dynamic_n * (c.c_drag_0 + induced),
        max_relative = 1e-4
    );
    check!(aero.force_n.y == 0.0);
}

/// Sideslip leaves lift, drag, and the pitch moment the share of the dynamic pressure in the
/// plane of symmetry, `cos²β` of it: air along the span (β 90°, a tailsitter's hover in a span
/// wind) lifts and drags nothing. The sideslip terms take the whole airspeed's pressure.
#[test]
fn lift_and_drag_take_the_air_in_the_plane_of_symmetry() {
    let model = model();
    let c = &model.aero;
    let dynamic_n = pressure_area_n(&model, 15.0);
    let head_on = wrench(
        &model,
        air(15.0, 0.0, 0.0),
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    );
    // beta rad
    let cases = [0.0, 0.1, 0.5, 1.2, FRAC_PI_2];
    for beta_rad in cases {
        let aero = wrench(
            &model,
            air(15.0, 0.0, beta_rad),
            Vec3::zeros(),
            Vec3::zeros(),
            Wash::default(),
        );
        let symmetric_share = libm::cosf(beta_rad) * libm::cosf(beta_rad);
        assert_relative_eq!(
            aero.force_n.x,
            symmetric_share * head_on.force_n.x,
            epsilon = 1e-4,
            max_relative = 1e-3
        );
        assert_relative_eq!(
            aero.force_n.z,
            symmetric_share * head_on.force_n.z,
            epsilon = 1e-4,
            max_relative = 1e-3
        );
        assert_relative_eq!(
            aero.torque_nm.y,
            symmetric_share * head_on.torque_nm.y,
            epsilon = 1e-4,
            max_relative = 1e-3
        );
        check!(
            (aero.force_n.y - dynamic_n * c.c_side_beta * beta_rad).abs() < 1e-3 * dynamic_n,
            "at β {beta_rad} rad"
        );
        check!(
            (aero.torque_nm.z - dynamic_n * model.span_m * c.c_yaw_beta * beta_rad).abs()
                < 1e-3 * dynamic_n,
            "at β {beta_rad} rad"
        );
    }
}

/// Each surface, flow angle, and rate pushes its moment the frame contract's way, through the
/// coefficients' signs: the change from head-on air with nothing deflected or turning.
#[test]
fn moments_point_the_contract_way() {
    // (case, alpha rad, beta rad, rates rad/s, deflections rad, moment sign: roll, pitch, yaw;
    // 0 = not checked)
    let cases = [
        (
            "aileron rolls right",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.2, 0.0, 0.0),
            [1.0, 0.0, 0.0],
        ),
        (
            "elevator raises the nose",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.0, 0.2, 0.0),
            [0.0, 1.0, 0.0],
        ),
        (
            "rudder yaws right",
            0.0,
            0.0,
            Vec3::zeros(),
            Vec3::new(0.0, 0.0, 0.2),
            [0.0, 0.0, 1.0],
        ),
        (
            "roll rate damped",
            0.0,
            0.0,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zeros(),
            [-1.0, 0.0, 0.0],
        ),
        (
            "pitch rate damped",
            0.0,
            0.0,
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::zeros(),
            [0.0, -1.0, 0.0],
        ),
        (
            "yaw rate damped",
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::zeros(),
            [0.0, 0.0, -1.0],
        ),
        (
            "nose into the wind, wing away",
            0.0,
            0.2,
            Vec3::zeros(),
            Vec3::zeros(),
            [-1.0, 0.0, 1.0],
        ),
        (
            "alpha pitches the nose down",
            0.2,
            0.0,
            Vec3::zeros(),
            Vec3::zeros(),
            [0.0, -1.0, 0.0],
        ),
    ];
    let head_on = wrench(
        &model(),
        air(15.0, 0.0, 0.0),
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    )
    .torque_nm;
    for (case, alpha_rad, beta_rad, rates_radps, deflection_rad, signs) in cases {
        let aero = wrench(
            &model(),
            air(15.0, alpha_rad, beta_rad),
            rates_radps,
            deflection_rad,
            Wash::default(),
        );
        let change_nm = aero.torque_nm - head_on;
        for (axis, sign) in signs.iter().enumerate() {
            if *sign != 0.0 {
                check!(change_nm[axis] * sign > 0.0, "{case}: {change_nm:?}");
            }
        }
    }
}

/// The pitch moment at zero α and no rates or deflection is the coefficient's, and a rate's
/// damping grows with airspeed (its term is per unit of it).
#[test]
fn moments_scale_from_the_coefficients() {
    let model = model();
    let c = &model.aero;
    let dynamic_n = 0.5 * AIR_DENSITY_KGPM3 * 100.0 * model.wing_area_m2;
    let level = wrench(
        &model,
        air(10.0, 0.0, 0.0),
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    );
    assert_relative_eq!(
        level.torque_nm.y,
        dynamic_n * model.chord_m * c.c_pitch_0,
        max_relative = 1e-4
    );
    let roll_damping = |speed_mps: f32| {
        wrench(
            &model,
            air(speed_mps, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::zeros(),
            Wash::default(),
        )
        .torque_nm
        .x
    };
    assert_relative_eq!(
        roll_damping(20.0),
        2.0 * roll_damping(10.0),
        max_relative = 1e-4
    );
}

/// Lift is the linear slope inside the stall, a flat plate's past it, continuous between, and
/// finite at any angle.
#[test]
fn lift_falls_off_past_the_stall() {
    let c = model().aero;
    let linear = |alpha_rad: f32| c.c_lift_0 + c.c_lift_alpha * alpha_rad;
    let flat_plate = |alpha_rad: f32| {
        2.0 * alpha_rad.signum() * libm::sinf(alpha_rad).powi(2) * libm::cosf(alpha_rad)
    };
    // (alpha rad, expected lift coefficient)
    let cases = [
        (0.0, linear(0.0)),
        (0.2, linear(0.2)),
        (-0.2, linear(-0.2)),
        (0.8, flat_plate(0.8)),
        (-0.8, flat_plate(-0.8)),
        (1.5, flat_plate(1.5)),
    ];
    for (alpha_rad, expected) in cases {
        assert_relative_eq!(lift_at(&c, alpha_rad), expected, epsilon = 1e-3);
    }
    // Past the stall, lift is well below the linear slope's.
    let past = c.alpha_stall_rad + 0.1;
    check!(lift_at(&c, past) < 0.5 * linear(past));
    // Continuous through the blend, and finite all the way round.
    let mut previous = lift_at(&c, -PI);
    for step in 1..=3600 {
        let alpha_rad = -PI + step as f32 * (2.0 * PI / 3600.0);
        let lift = lift_at(&c, alpha_rad);
        check!(lift.is_finite(), "at {alpha_rad} rad");
        check!((lift - previous).abs() < 0.2, "jump at {alpha_rad} rad");
        previous = lift;
    }
}

/// Broadside to the air (α 90°, a tailsitter sinking or climbing belly first), drag is a flat
/// plate's and lift is gone.
#[test]
fn broadside_drag_is_a_flat_plates() {
    let model = model();
    let c = &model.aero;
    let dynamic_n = pressure_area_n(&model, 10.0);
    let aero = wrench(
        &model,
        air(10.0, FRAC_PI_2, 0.0),
        Vec3::zeros(),
        Vec3::zeros(),
        Wash::default(),
    );
    // Drag opposes the air's motion along body +z; lift (along body x here) vanishes.
    assert_relative_eq!(
        aero.force_n.z,
        -dynamic_n * (c.c_drag_0 + 2.0),
        max_relative = 1e-3
    );
    check!(aero.force_n.x.abs() < 1e-3 * dynamic_n);
}

/// All the way round in α, forces and the pitch moment stay within a flat plate's and the
/// coefficients' reach, and change smoothly, through reversed flow at ±180° too, with or without
/// the flat plate's pitch moment past the stall.
#[test]
fn forces_and_moments_stay_bounded_at_any_angle() {
    for plate_share in [0.0, 1.0] {
        let mut model = model();
        model.aero.post_stall_pitch_share = plate_share;
        let c = &model.aero;
        let dynamic_n = pressure_area_n(&model, 10.0);
        let linear_peak = (c.c_lift_0 + c.c_lift_alpha * c.alpha_stall_rad).abs();
        let force_bound_n = dynamic_n * (linear_peak + c.c_drag_0 + 2.0);
        // The plate's normal force, at most 2, at most the three-quarter chord behind the
        // center of gravity.
        let plate_reach = 2.0 * (0.75 - (0.25 + c.c_pitch_alpha / c.c_lift_alpha));
        let pitch_reach = c.c_pitch_0.abs() + c.c_pitch_alpha.abs() + plate_share * plate_reach;
        let pitch_bound_nm = dynamic_n * model.chord_m * pitch_reach;
        let at = |alpha_rad: f32| {
            wrench(
                &model,
                air(10.0, alpha_rad, 0.0),
                Vec3::zeros(),
                Vec3::zeros(),
                Wash::default(),
            )
        };
        let mut previous = at(-PI);
        for step in 1..=3600 {
            let alpha_rad = -PI + step as f32 * (2.0 * PI / 3600.0);
            let aero = at(alpha_rad);
            let what = format!("plate share {plate_share}, at {alpha_rad} rad");
            check!(aero.force_n.norm() <= force_bound_n, "{what}");
            check!(aero.torque_nm.y.abs() <= pitch_bound_nm, "{what}");
            check!(
                (aero.torque_nm.y - previous.torque_nm.y).abs() < 0.05 * pitch_bound_nm,
                "pitch moment jumps, {what}"
            );
            previous = aero;
        }
    }
}

/// Past the stall the pitch moment blends into a flat plate's: its normal force, `2·sin²α`
/// broadside, at a center of pressure from mid-chord broadside to the three-quarter chord
/// reversed, about a center of gravity the static margin ahead of the quarter chord. Below the
/// stall, and with no plate share, it stays `c_pitch_alpha`'s `sin α`.
#[test]
fn post_stall_pitch_follows_a_flat_plate() {
    let mut plate = model().aero;
    plate.post_stall_pitch_share = 1.0;
    let center_of_gravity = 0.25 + plate.c_pitch_alpha / plate.c_lift_alpha;
    let on_sin_alpha =
        |alpha_rad: f32| plate.c_pitch_0 + plate.c_pitch_alpha * libm::sinf(alpha_rad);
    let reversed_rad = PI - 20f32.to_radians();
    let reversed_normal = 2.0
        * libm::sinf(reversed_rad).powi(2)
        * (libm::cosf(reversed_rad).powi(2) + libm::sinf(reversed_rad));
    let reversed_center = 0.25 + 0.5 * reversed_rad / PI;
    // (case, α in rad, coefficient)
    let cases = [
        (
            "attached, 5°",
            5f32.to_radians(),
            on_sin_alpha(5f32.to_radians()),
        ),
        (
            "broadside, nose down at mid-chord",
            FRAC_PI_2,
            plate.c_pitch_0 - 2.0 * (0.5 - center_of_gravity),
        ),
        (
            "reversed, 20° off: nose down, away from the flow",
            reversed_rad,
            plate.c_pitch_0 - reversed_normal * (reversed_center - center_of_gravity),
        ),
        (
            "reversed the other way: nose up",
            -reversed_rad,
            plate.c_pitch_0 + reversed_normal * (reversed_center - center_of_gravity),
        ),
    ];
    for (case, alpha_rad, expected) in cases {
        let blend = stall_blend(&plate, alpha_rad);
        let (sin_alpha, cos_alpha) = (libm::sinf(alpha_rad), libm::cosf(alpha_rad));
        let coefficient = pitch_coefficient(&plate, alpha_rad, blend, sin_alpha, cos_alpha);
        check!(
            (coefficient - expected).abs() < 1e-3,
            "{case}: {coefficient} vs {expected}"
        );
        let without = AeroCoefficients {
            post_stall_pitch_share: 0.0,
            ..plate
        };
        check!(
            pitch_coefficient(&without, alpha_rad, blend, sin_alpha, cos_alpha)
                == on_sin_alpha(alpha_rad),
            "{case}, no plate"
        );
    }
}

/// The wash adds to the free stream's dynamic pressure over the surfaces and damps the turn
/// rates as a flow at its damping speed would, `½ρ·v·S` per unit of airspeed; it doesn't touch
/// lift or drag. In still air the surfaces and that damping act alone.
#[test]
fn wash_works_the_surfaces_and_damps_the_rates() {
    let model = model();
    let c = &model.aero;
    let wash = Wash {
        pressure_pa: 200.0,
        roll_pressure_pa: 200.0,
        damping_mps: 4.0,
        pitch_moment_pa: 0.0,
        stall_pitch_moment_pa: 0.0,
    };
    let wash_scale = 0.5 * AIR_DENSITY_KGPM3 * wash.damping_mps * model.wing_area_m2;
    let elevator = Vec3::new(0.0, 0.2, 0.0);
    let rates_radps = Vec3::new(1.0, 1.0, 1.0);
    let still = wrench(&model, Vec3::zeros(), rates_radps, elevator, wash);
    let (span, chord) = (model.span_m, model.chord_m);
    let damping_nm = Vec3::new(
        wash_scale * span * span / 2.0 * (c.c_roll_p + c.c_roll_r),
        wash_scale * chord * chord / 2.0 * c.c_pitch_q,
        wash_scale * span * span / 2.0 * (c.c_yaw_p + c.c_yaw_r),
    );
    let elevator_nm = wash.pressure_pa * model.wing_area_m2 * chord * c.c_pitch_elevator * 0.2;
    assert_relative_eq!(
        still.torque_nm,
        damping_nm + Vec3::new(0.0, elevator_nm, 0.0),
        max_relative = 1e-5
    );
    check!(still.force_n == Vec3::zeros());

    let flying = |wash: Wash, rates_radps: Vec3, deflection_rad: Vec3| {
        wrench(
            &model,
            air(15.0, 0.05, 0.0),
            rates_radps,
            deflection_rad,
            wash,
        )
    };
    let calm = Wash::default();
    let none = Vec3::zeros();
    // Undeflected surfaces, not turning: the wash changes nothing.
    check!(flying(wash, none, none) == flying(calm, none, none));
    // Deflected and turning: the wash adds exactly what it adds in still air.
    let added = flying(wash, rates_radps, elevator);
    let added_nm = added.torque_nm - flying(calm, rates_radps, elevator).torque_nm;
    assert_relative_eq!(
        added_nm,
        still.torque_nm,
        max_relative = 1e-4,
        epsilon = 1e-6
    );
    check!(added.force_n == flying(calm, rates_radps, elevator).force_n);
}

/// The wash's own pitch moment adds its pressure times wing area and chord, flying or still,
/// and nothing else.
#[test]
fn wash_pitch_moment_adds_its_pressure_times_area_and_chord() {
    let model = model();
    let pitching = Wash {
        pitch_moment_pa: -10.0,
        ..Wash::default()
    };
    let expected_nm = Vec3::new(0.0, -10.0 * model.wing_area_m2 * model.chord_m, 0.0);
    // (case, air m/s)
    let cases = [("still", Vec3::zeros()), ("flying", air(15.0, 0.05, 0.0))];
    for (case, air_mps) in cases {
        let none = Vec3::zeros();
        let with = wrench(&model, air_mps, none, none, pitching);
        let without = wrench(&model, air_mps, none, none, Wash::default());
        assert_relative_eq!(
            with.torque_nm - without.torque_nm,
            expected_nm,
            epsilon = 1e-5
        );
        check!(with.force_n == without.force_n, "{case}");
    }
}

/// The wash's stall pitch moment acts past the stall nose first, as far past the stall, as
/// `cos α`, and as the free stream's share of the pressure over the surfaces: nothing still,
/// attached, broadside, or tail first.
#[test]
fn wash_stall_pitch_moment_acts_past_the_stall_nose_first() {
    let model = model();
    let c = &model.aero;
    let pressure_pa = 50.0;
    let plain = Wash {
        pressure_pa,
        roll_pressure_pa: pressure_pa,
        ..Wash::default()
    };
    let pitching = Wash {
        stall_pitch_moment_pa: -10.0,
        ..plain
    };
    let area_chord_m3 = model.wing_area_m2 * model.chord_m;
    let stalled_rad = 60.0f32.to_radians();
    let stalled_share = {
        let free_pa = pressure_area_n(&model, 10.0) / model.wing_area_m2;
        stall_blend(c, stalled_rad) * libm::cosf(stalled_rad) * free_pa / (free_pa + pressure_pa)
    };
    // (case, air m/s, share of the moment)
    let cases = [
        ("still", Vec3::zeros(), 0.0),
        ("attached", air(15.0, 0.05, 0.0), 0.0),
        (
            "stalled nose first",
            air(10.0, stalled_rad, 0.0),
            stalled_share,
        ),
        ("broadside", air(10.0, FRAC_PI_2, 0.0), 0.0),
        ("tail first", air(10.0, PI - 0.3, 0.0), 0.0),
    ];
    for (case, air_mps, share) in cases {
        let none = Vec3::zeros();
        let with = wrench(&model, air_mps, none, none, pitching);
        let without = wrench(&model, air_mps, none, none, plain);
        let added_nm = with.torque_nm - without.torque_nm;
        check!(
            (added_nm.y - share * -10.0 * area_chord_m3).abs() < 1e-4,
            "{case}: {added_nm:?}"
        );
        check!(added_nm.x == 0.0 && added_nm.z == 0.0, "{case}");
    }
    check!(stalled_share > 0.1, "{stalled_share}");
}

/// The roll pressure works the ailerons alone, the pressure the elevator: in still air, with
/// each deflected, each moment follows its own pressure.
#[test]
fn roll_pressure_works_the_ailerons_alone() {
    let model = model();
    let c = &model.aero;
    let wash = Wash {
        pressure_pa: 200.0,
        roll_pressure_pa: 50.0,
        damping_mps: 0.0,
        pitch_moment_pa: 0.0,
        stall_pitch_moment_pa: 0.0,
    };
    let deflection_rad = Vec3::new(0.1, 0.2, 0.0);
    let still = wrench(&model, Vec3::zeros(), Vec3::zeros(), deflection_rad, wash);
    let area_m2 = model.wing_area_m2;
    let aileron_nm = wash.roll_pressure_pa * area_m2 * model.span_m * c.c_roll_aileron * 0.1;
    let elevator_nm = wash.pressure_pa * area_m2 * model.chord_m * c.c_pitch_elevator * 0.2;
    assert_relative_eq!(
        still.torque_nm,
        Vec3::new(aileron_nm, elevator_nm, 0.0),
        max_relative = 1e-5
    );
}

/// With the surfaces' stall share at 1, past the stall (broadside, tail first) the surfaces
/// work on the wash alone; in attached flow, and with no stall share, on the free stream too.
#[test]
fn stalled_surfaces_work_in_the_wash_alone() {
    let mut stalling = model();
    stalling.aero.surface_stall_share = 1.0;
    let wash = Wash {
        pressure_pa: 50.0,
        roll_pressure_pa: 50.0,
        damping_mps: 0.0,
        pitch_moment_pa: 0.0,
        stall_pitch_moment_pa: 0.0,
    };
    let elevator_rad = 0.2;
    let elevator = Vec3::new(0.0, elevator_rad, 0.0);
    let surface_nm = |model: &FixedWingModel, alpha_rad: f32, wash: Wash| {
        let deflected = wrench(
            model,
            air(5.0, alpha_rad, 0.0),
            Vec3::zeros(),
            elevator,
            wash,
        );
        let neutral = wrench(
            model,
            air(5.0, alpha_rad, 0.0),
            Vec3::zeros(),
            Vec3::zeros(),
            wash,
        );
        deflected.torque_nm.y - neutral.torque_nm.y
    };
    let per_pa_nm =
        stalling.wing_area_m2 * stalling.chord_m * stalling.aero.c_pitch_elevator * elevator_rad;
    let wash_nm = wash.pressure_pa * per_pa_nm;
    let free_nm = pressure_area_n(&stalling, 5.0) / stalling.wing_area_m2 * per_pa_nm;
    // (case, model, α in rad, moment)
    let cases = [
        ("attached", stalling, 5f32.to_radians(), free_nm + wash_nm),
        ("broadside", stalling, FRAC_PI_2, wash_nm),
        ("tail first", stalling, PI, wash_nm),
        (
            "broadside, no stall share",
            model(),
            FRAC_PI_2,
            free_nm + wash_nm,
        ),
    ];
    for (case, model, alpha_rad, expected_nm) in cases {
        let moment_nm = surface_nm(&model, alpha_rad, wash);
        check!(
            (moment_nm - expected_nm).abs() < 0.02 * expected_nm,
            "{case}: {moment_nm} vs {expected_nm}"
        );
    }
}
