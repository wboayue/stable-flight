//! QuadCopterX's yaw on the sim: its yaw trim, its yaw inertia and gains, and the telemetry
//! that watches the yaw moment against the trim.

use std::cell::Cell;
use std::rc::Rc;

use stable_flight::controllers::{Gains, YawControllerConfig};
use stable_flight::{TelemetryPublisher, TelemetryRecord, Vec3};
use stable_flight_sim::airframes::{self, Airframe};

use super::{Hover, Pipeline, Plant};

/// Normalized yaw moment, nose right, the yaw-trimmed rows hover on: QuadCopterX's in
/// RealFlight (#192).
pub const YAW_TRIM: f32 = 0.40;

/// A yaw disturbance torque used to measure how much moment holds it, in N·m: small enough
/// that the default yaw I holds it.
const PROBE_TORQUE_NM: f32 = 0.01;

/// Seconds of settled hover the yaw moment is averaged over.
const TRIM_AVERAGE_S: f32 = 2.0;

/// The last tick's yaw moment, and the ticks the mixer couldn't deliver the moments in full.
#[derive(Clone, Default)]
pub struct YawWatch {
    pub yaw_moment: Rc<Cell<f32>>,
    pub mixer_clipped_ticks: Rc<Cell<usize>>,
}

impl TelemetryPublisher for YawWatch {
    fn publish(&mut self, record: &TelemetryRecord) {
        self.yaw_moment.set(record.control.moments.z);
        if record.saturation.mixer_moments {
            self.mixer_clipped_ticks
                .set(self.mixer_clipped_ticks.get() + 1);
        }
    }
}

/// `plant` hovering on `watch`'s pipeline, settled.
pub fn watched_hover(plant: &Plant, watch: &YawWatch) -> Pipeline<Hover, YawWatch> {
    let mut flight_loop = super::pipeline(plant, super::hover_follower(), watch.clone());
    super::fly(&mut flight_loop, super::HOVER_SETTLE_S, |_, _| {});
    flight_loop
}

/// `airframe` under a yaw torque that takes [`YAW_TRIM`] of moment to hold, measured from the
/// moment a hover holds against [`PROBE_TORQUE_NM`]: a positive torque takes a negative moment.
pub fn yaw_trimmed(airframe: Airframe) -> Plant {
    let mut probe = Plant::clean(airframe);
    probe.perturbations.disturbance_moment_nm = Vec3::new(0.0, 0.0, PROBE_TORQUE_NM);
    let watch = YawWatch::default();
    let mut flight_loop = watched_hover(&probe, &watch);
    let mut sum = 0.0;
    super::fly(&mut flight_loop, TRIM_AVERAGE_S, |_, _| {
        sum += watch.yaw_moment.get()
    });
    let moment_per_nm = sum / super::ticks(TRIM_AVERAGE_S) as f32 / PROBE_TORQUE_NM;
    let mut plant = probe;
    plant.perturbations.disturbance_moment_nm.z = YAW_TRIM / moment_per_nm;
    plant
}

/// QuadCopterX's yaw rate gains (sitl-realflight's `airframes::quadcopter_x`, #558).
pub const QUADCOPTER_X_YAW_RATE: Gains = Gains {
    p: 1.1532,
    i: 0.3460,
    d: 0.0,
};

/// How many times `quad_x`'s yaw inertia QuadCopterX's yaw turns like: with it, a 30° heading
/// turn overshoots and settles as RealFlight's QuadCopterX does on the same gains (yaw rate P
/// 0.6, I 0.6: 28% and 4.1 s against 25% and 4.2 s; P 1.2: 7% and 1.3 s against 7% and 1.45 s;
/// #383).
const QUADCOPTER_X_YAW_INERTIA_SCALE: f32 = 2.0;

/// `quad_x` yawing like QuadCopterX: its yaw inertia scaled by
/// [`QUADCOPTER_X_YAW_INERTIA_SCALE`], and QuadCopterX's heading and yaw rate loops flying
/// `yaw_rate` (core's heading loop, as sitl-realflight's `airframes::quadcopter_x`).
pub fn quadcopter_x_yaw(yaw_rate: Gains) -> Airframe {
    let mut airframe = airframes::quad_x();
    airframe.model.inertia_kgm2.z *= QUADCOPTER_X_YAW_INERTIA_SCALE;
    airframe.stabilizer.yaw = YawControllerConfig::default();
    airframe.stabilizer.body_rate.yaw_gains = yaw_rate;
    airframe
}
