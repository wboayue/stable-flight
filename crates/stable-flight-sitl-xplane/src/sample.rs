//! The raw X-Plane state this backend reads: [`XPlaneSample`], one value per dataref, and
//! [`StateDatarefs`], their subscriptions.

use std::thread;
use std::time::{Duration, Instant};

use log::debug;
use xplane_bridge::datarefs::{position, time};
use xplane_bridge::{Dataref, DatarefSource, Latest, Rate, Sub, Value, XPlaneError};

use crate::ConnectError;

#[cfg(test)]
pub(crate) mod tests;

/// Rate every state dataref is subscribed at. X-Plane sends a dataref at most once per frame
/// (60–120 Hz) and rounds a lower rate down to whole frames, so every frame needs at least
/// twice the frame rate (xplane-bridge `design.md`, Spike findings: Rate).
pub const STATE_RATE: Rate = match Rate::hz(250) {
    Some(rate) => rate,
    None => panic!("nonzero rate"),
};

/// True airspeed, m/s. No xplane-bridge constant.
const TRUE_AIRSPEED: &str = "sim/flightmodel/position/true_airspeed";
/// Any wheel or skid on the ground (`int` boolean). No xplane-bridge constant.
const ON_GROUND: &str = "sim/flightmodel/failures/onground_any";

/// How often [`wait_ready`] resubscribes while a dataref has no value: X-Plane drops `RREF`s
/// that arrive in a burst, and xplane-bridge's `probe` had all 21 of its subscriptions after
/// 1–5 resubscribes at this cadence (`design.md`, Spike findings: Burst loss).
pub(crate) const RESUBSCRIBE_INTERVAL: Duration = Duration::from_millis(250);

/// How long [`wait_ready`] sleeps between polls.
const POLL_INTERVAL: Duration = Duration::from_millis(5);

/// One X-Plane state, in X-Plane's own units and frames, as the datarefs report it.
///
/// Local coordinates are X-Plane's OpenGL frame: x east, y up, z south, from the local origin
/// at (`origin_deg`). [`vehicle_state`](crate::vehicle_state) converts it to the core frame
/// contract.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct XPlaneSample {
    /// Simulator time (`sim/time/total_running_time_sec`), s; frozen while paused.
    pub sim_time_s: f32,
    /// Attitude quaternion `[w, x, y, z]`, the rotation from north-east-down to the body frame
    /// (`position/q`): the same yaw-pitch-roll as `psi`/`theta`/`phi`.
    pub quaternion: [f32; 4],
    /// Body rates `P`, `Q`, `R` (roll right, nose up, nose right positive), deg/s.
    pub body_rates_degps: [f32; 3],
    /// Local position `local_x/y/z` (east, up, south), m.
    pub local_position_m: [f32; 3],
    /// Local velocity `local_vx/vy/vz` (east, up, south), m/s.
    pub local_velocity_mps: [f32; 3],
    /// Local acceleration `local_ax/ay/az` (east, up, south), m/s².
    pub local_acceleration_mps2: [f32; 3],
    /// Latitude, degrees north; `f32` on the wire, so about 0.4 m steps: for anchoring only.
    pub latitude_deg: f32,
    /// Longitude, degrees east; `f32` on the wire, so up to about 0.8 m steps.
    pub longitude_deg: f32,
    /// Elevation above mean sea level, m.
    pub elevation_m: f32,
    /// The local origin's latitude and longitude (`lat_ref`, `lon_ref`), degrees.
    pub origin_deg: [f32; 2],
    /// True airspeed, m/s.
    pub true_airspeed_mps: f32,
    /// Whether any wheel or skid is on the ground.
    pub on_ground: bool,
    /// Whether X-Plane is paused (it keeps streaming frozen values).
    pub paused: bool,
}

/// Number of `f32` datarefs in an [`XPlaneSample`].
const FLOAT_COUNT: usize = 23;

/// The subscriptions behind an [`XPlaneSample`], one handle per dataref.
///
/// Subscribe once with [`subscribe`](Self::subscribe), apply every polled frame to a
/// [`Latest`], and read the sample with [`sample`](Self::sample).
///
/// # Examples
///
/// ```
/// use std::time::Instant;
/// use stable_flight_sitl_xplane::StateDatarefs;
/// use stable_flight_sitl_xplane::xplane_bridge::Latest;
/// use stable_flight_sitl_xplane::xplane_bridge::testing::FakeXPlane;
///
/// let mut fake = FakeXPlane::new();
/// let datarefs = StateDatarefs::subscribe(&mut fake)?;
/// // Nothing has arrived: every dataref is missing.
/// let latest = Latest::default();
/// assert!(datarefs.sample(&latest).is_none());
/// assert_eq!(datarefs.missing(&latest).len(), 25);
/// # Ok::<(), stable_flight_sitl_xplane::xplane_bridge::XPlaneError>(())
/// ```
#[derive(Debug, Clone, Copy)]
pub struct StateDatarefs {
    sim_time: Sub<f32>,
    quaternion: [Sub<f32>; 4],
    body_rates: [Sub<f32>; 3],
    position: [Sub<f32>; 3],
    velocity: [Sub<f32>; 3],
    acceleration: [Sub<f32>; 3],
    latitude: Sub<f32>,
    longitude: Sub<f32>,
    elevation: Sub<f32>,
    origin: [Sub<f32>; 2],
    true_airspeed: Sub<f32>,
    on_ground: Sub<bool>,
    paused: Sub<bool>,
}

impl StateDatarefs {
    /// Subscribes every state dataref at [`STATE_RATE`].
    ///
    /// # Errors
    ///
    /// The bridge's error, when sending a subscription fails.
    pub fn subscribe(source: &mut impl DatarefSource) -> Result<Self, XPlaneError> {
        let quaternion = |i| {
            position::QUATERNION
                .at(i)
                .unwrap_or_else(|| unreachable!("the quaternion has 4 elements"))
        };
        macro_rules! sub {
            ($dataref:expr) => {
                source.subscribe(&$dataref, STATE_RATE)?
            };
        }
        Ok(StateDatarefs {
            sim_time: sub!(time::TOTAL_RUNNING_TIME_SEC),
            quaternion: [
                sub!(quaternion(0)),
                sub!(quaternion(1)),
                sub!(quaternion(2)),
                sub!(quaternion(3)),
            ],
            body_rates: [sub!(position::P), sub!(position::Q), sub!(position::R)],
            position: [
                sub!(position::LOCAL_X),
                sub!(position::LOCAL_Y),
                sub!(position::LOCAL_Z),
            ],
            velocity: [
                sub!(position::LOCAL_VX),
                sub!(position::LOCAL_VY),
                sub!(position::LOCAL_VZ),
            ],
            acceleration: [
                sub!(position::LOCAL_AX),
                sub!(position::LOCAL_AY),
                sub!(position::LOCAL_AZ),
            ],
            latitude: sub!(position::LATITUDE),
            longitude: sub!(position::LONGITUDE),
            elevation: sub!(position::ELEVATION),
            origin: [sub!(position::LAT_REF), sub!(position::LON_REF)],
            true_airspeed: sub!(Dataref::<f32>::new(TRUE_AIRSPEED)?),
            on_ground: sub!(Dataref::<bool>::new(ON_GROUND)?),
            paused: sub!(time::PAUSED),
        })
    }

    /// The sample, once every dataref has a value in `latest`.
    pub fn sample(&self, latest: &Latest) -> Option<XPlaneSample> {
        let get = |sub| value(latest, sub);
        let all = |subs: [Sub<f32>; 3]| -> Option<[f32; 3]> {
            Some([get(subs[0])?, get(subs[1])?, get(subs[2])?])
        };
        let [w, x, y, z] = self.quaternion;
        Some(XPlaneSample {
            sim_time_s: get(self.sim_time)?,
            quaternion: [get(w)?, get(x)?, get(y)?, get(z)?],
            body_rates_degps: all(self.body_rates)?,
            local_position_m: all(self.position)?,
            local_velocity_mps: all(self.velocity)?,
            local_acceleration_mps2: all(self.acceleration)?,
            latitude_deg: get(self.latitude)?,
            longitude_deg: get(self.longitude)?,
            elevation_m: get(self.elevation)?,
            origin_deg: [get(self.origin[0])?, get(self.origin[1])?],
            true_airspeed_mps: get(self.true_airspeed)?,
            on_ground: value(latest, self.on_ground)?,
            paused: value(latest, self.paused)?,
        })
    }

    /// The datarefs with no value in `latest` yet.
    pub fn missing(&self, latest: &Latest) -> Vec<&'static str> {
        self.received(latest)
            .filter(|(_, at)| at.is_none())
            .map(|(name, _)| name)
            .collect()
    }

    /// Whether a dataref is missing, or last arrived more than `max_age` before the newest
    /// one: data flows, but not for every subscription (X-Plane dropped its `RREF`, or forgot
    /// it in a restart and a resubscribe burst lost it again).
    pub(crate) fn incomplete(&self, latest: &Latest, max_age: Duration) -> bool {
        let Some(newest) = self.received(latest).filter_map(|(_, at)| at).max() else {
            return true;
        };
        self.received(latest)
            .any(|(_, at)| at.is_none_or(|at| newest.duration_since(at) > max_age))
    }

    /// Each dataref's name and when its value arrived (`None`: never).
    fn received(&self, latest: &Latest) -> impl Iterator<Item = (&'static str, Option<Instant>)> {
        let floats = self
            .named_floats()
            .map(|(name, sub)| (name, arrived(latest, sub)));
        let bools = self
            .named_bools()
            .map(|(name, sub)| (name, arrived(latest, sub)));
        floats.into_iter().chain(bools)
    }

    /// The `f32` datarefs with their names, in [`XPlaneSample`] field order: the one place
    /// they are named.
    fn named_floats(&self) -> [(&'static str, Sub<f32>); FLOAT_COUNT] {
        let [q0, q1, q2, q3] = self.quaternion;
        let [p, q, r] = self.body_rates;
        let [x, y, z] = self.position;
        let [vx, vy, vz] = self.velocity;
        let [ax, ay, az] = self.acceleration;
        let [lat_ref, lon_ref] = self.origin;
        [
            ("total_running_time_sec", self.sim_time),
            ("q[0]", q0),
            ("q[1]", q1),
            ("q[2]", q2),
            ("q[3]", q3),
            ("P", p),
            ("Q", q),
            ("R", r),
            ("local_x", x),
            ("local_y", y),
            ("local_z", z),
            ("local_vx", vx),
            ("local_vy", vy),
            ("local_vz", vz),
            ("local_ax", ax),
            ("local_ay", ay),
            ("local_az", az),
            ("latitude", self.latitude),
            ("longitude", self.longitude),
            ("elevation", self.elevation),
            ("lat_ref", lat_ref),
            ("lon_ref", lon_ref),
            ("true_airspeed", self.true_airspeed),
        ]
    }

    /// The boolean datarefs with their names, in [`XPlaneSample`] field order.
    fn named_bools(&self) -> [(&'static str, Sub<bool>); 2] {
        [("onground_any", self.on_ground), ("paused", self.paused)]
    }
}

#[cfg(test)]
impl StateDatarefs {
    /// A frame carrying `sample`, received `at`, without the datarefs named in `skip`.
    pub(crate) fn frame(
        &self,
        sample: &XPlaneSample,
        at: Instant,
        skip: &[&str],
    ) -> xplane_bridge::Frame {
        let [q0, q1, q2, q3] = sample.quaternion;
        let [p, q, r] = sample.body_rates_degps;
        let [x, y, z] = sample.local_position_m;
        let [vx, vy, vz] = sample.local_velocity_mps;
        let [ax, ay, az] = sample.local_acceleration_mps2;
        let [lat_ref, lon_ref] = sample.origin_deg;
        // Field order, as `named_floats`; `reads_a_complete_sample_and_names_the_missing`
        // catches a mismatch (every field differs).
        let floats = [
            sample.sim_time_s,
            q0,
            q1,
            q2,
            q3,
            p,
            q,
            r,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            ax,
            ay,
            az,
            sample.latitude_deg,
            sample.longitude_deg,
            sample.elevation_m,
            lat_ref,
            lon_ref,
            sample.true_airspeed_mps,
        ];
        let bools = [sample.on_ground, sample.paused];
        let mut frame = xplane_bridge::Frame::empty(at);
        for ((name, sub), value) in self.named_floats().into_iter().zip(floats) {
            if !skip.contains(&name) {
                frame = frame.with(sub, value);
            }
        }
        for ((name, sub), value) in self.named_bools().into_iter().zip(bools) {
            if !skip.contains(&name) {
                frame = frame.with(sub, value);
            }
        }
        frame
    }
}

fn value<T: Value>(latest: &Latest, sub: Sub<T>) -> Option<T> {
    latest.get(sub).map(|(value, _)| value)
}

fn arrived<T: Value>(latest: &Latest, sub: Sub<T>) -> Option<Instant> {
    latest.get(sub).map(|(_, at)| at)
}

/// Polls `source` into `latest` until every dataref of `datarefs` has a value, resubscribing
/// every 250 ms meanwhile, and returns the first complete sample. Blocks: for start-up, not
/// the tick.
///
/// # Errors
///
/// [`ConnectError::NotReady`] naming the missing datarefs after `timeout` (X-Plane isn't
/// running at that address, or doesn't know a dataref); [`ConnectError::Bridge`] when a poll
/// or resubscribe fails.
pub fn wait_ready(
    source: &mut impl DatarefSource,
    datarefs: &StateDatarefs,
    latest: &mut Latest,
    timeout: Duration,
) -> Result<XPlaneSample, ConnectError> {
    let start = Instant::now();
    let mut resubscribed = start;
    loop {
        while let Some(frame) = source.poll()? {
            latest.apply(&frame);
        }
        if let Some(sample) = datarefs.sample(latest) {
            return Ok(sample);
        }
        let now = Instant::now();
        if now.duration_since(start) >= timeout {
            return Err(ConnectError::NotReady {
                missing: datarefs.missing(latest),
                timeout,
            });
        }
        if now.duration_since(resubscribed) >= RESUBSCRIBE_INTERVAL {
            debug!(
                "Waiting for X-Plane: missing {:?}",
                datarefs.missing(latest)
            );
            source.resubscribe()?;
            resubscribed = now;
        }
        thread::sleep(POLL_INTERVAL);
    }
}
