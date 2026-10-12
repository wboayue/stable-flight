use super::*;
use assert2::check;
use realflight_bridge::{BridgeError, ControlInputs, Quaternion, RealFlightBridge, SimulatorState};
use stable_flight::FlightLoop;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::TableMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::conformance::Scenario;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::{Vec3, wrap_angle};
use std::{
    cell::{Cell, RefCell},
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    time::Duration,
};

// Mock implementation of RealFlightBridge for testing
struct MockRealFlightBridge {
    should_fail: bool,
    last_input: RefCell<Option<ControlInputs>>,
    /// The state returned by every successful exchange.
    state: SimulatorState,
    /// Number of `exchange_data` calls.
    exchanges: Cell<usize>,
    /// Overwrites `aircraft_position_y_m` (north) with the exchange number (1, 2, …) so a test can tell
    /// which exchange a state came from.
    stamp_exchanges: bool,
    /// Fails every exchange and `disable_rc` with a `Connection` error, as an out-of-sync remote
    /// bridge does.
    connection_lost: bool,
    /// Fails `disable_rc` with a SOAP fault, as RealFlight does when the Link already has control.
    disable_rc_fault: bool,
    /// Stands in for the bridge's open connection: a test holding a clone sees it close (the
    /// strong count drop) when the bridge is dropped.
    _connection: Option<Arc<()>>,
}

impl MockRealFlightBridge {
    fn new(should_fail: bool) -> Self {
        Self {
            should_fail,
            last_input: RefCell::new(None),
            state: sample_state(),
            exchanges: Cell::new(0),
            stamp_exchanges: false,
            connection_lost: false,
            disable_rc_fault: false,
            _connection: None,
        }
    }
}

fn sample_state() -> SimulatorState {
    SimulatorState {
        aircraft_position_x_m: 1.0,
        aircraft_position_y_m: 2.0,
        altitude_asl_m: 3.0,
        velocity_world_mps: [4.0, 5.0, 6.0].into(),
        acceleration_body_mps2: [7.0, 8.0, 9.0].into(),
        orientation: Quaternion {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        roll_rate_dps: 90.0,
        pitch_rate_dps: 45.0,
        yaw_rate_dps: 30.0,
        ..Default::default()
    }
}

impl RealFlightBridge for MockRealFlightBridge {
    fn exchange_data(&self, input: &ControlInputs) -> Result<SimulatorState, BridgeError> {
        let mut last_input = self.last_input.borrow_mut();
        last_input.replace(input.clone());
        self.exchanges.set(self.exchanges.get() + 1);

        if self.connection_lost {
            Err(BridgeError::Connection(std::io::Error::other(
                "connection out of sync after failed request; reconnect",
            )))
        } else if self.should_fail {
            Err(BridgeError::Initialization(
                "Simulated bridge failure".to_string(),
            ))
        } else {
            let mut state = self.state.clone();
            if self.stamp_exchanges {
                state.aircraft_position_y_m = self.exchanges.get() as f32;
            }
            Ok(state)
        }
    }

    fn disable_rc(&self) -> Result<(), BridgeError> {
        if self.connection_lost {
            Err(BridgeError::Connection(std::io::Error::other(
                "failed to fill whole buffer",
            )))
        } else if self.disable_rc_fault {
            Err(BridgeError::SoapFault(
                "Preexisting controller reference".to_string(),
            ))
        } else if self.should_fail {
            Err(BridgeError::Initialization("RC disable failed".to_string()))
        } else {
            Ok(())
        }
    }

    fn enable_rc(&self) -> Result<(), BridgeError> {
        if self.should_fail {
            Err(BridgeError::Initialization("RC enable failed".to_string()))
        } else {
            Ok(())
        }
    }

    fn reset_aircraft(&self) -> Result<(), BridgeError> {
        if self.should_fail {
            Err(BridgeError::Initialization("Reset failed".to_string()))
        } else {
            Ok(())
        }
    }
}

// Helper function to create a test configuration
fn create_test_config(should_fail: bool) -> RealFlightConfiguration<MockRealFlightBridge> {
    let mut mapping = ActuatorMapping::default();
    mapping.motors[0] = Some(0);
    mapping.servos[0] = Some(1);

    RealFlightConfiguration {
        mapping,
        body_rotation: UnitQuaternion::identity(),
        bridge: MockRealFlightBridge::new(should_fail),
    }
}

#[test]
fn test_vehicle_creation() {
    let config = create_test_config(false);
    let vehicle = RealFlightVehicle::new(config);

    // Verify initial state
    assert_eq!(vehicle.state.position_m, Vec3::zeros());
    assert_eq!(vehicle.mapping.motors[0], Some(0));
    assert_eq!(vehicle.mapping.servos[0], Some(1));
}

#[test]
fn test_vehicle_creation_with_bridge_failure() {
    let config = create_test_config(true);
    let vehicle = RealFlightVehicle::new(config);

    // Should still create successfully despite bridge failures
    assert_eq!(vehicle.state.position_m, Vec3::zeros());
}

#[test]
fn actuate_sends_mapped_channels() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);

    vehicle.actuate(&ActuatorCommand {
        motors: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        servos: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    });

    let sent = vehicle
        .bridge
        .as_ref()
        .expect("connected")
        .last_input
        .borrow()
        .clone();
    let channels = sent.expect("exchanged").channels;
    assert_eq!(channels[0], 0.5); // motor 0
    assert_eq!(channels[1], 0.0); // servo 0 at full negative
}

#[test]
fn try_new_rejects_invalid_mapping() {
    let mut config = create_test_config(false);
    config.mapping.servos[1] = Some(0); // motor 0's channel

    let result = RealFlightVehicle::try_new(config);

    assert!(matches!(
        result,
        Err(MappingError::DuplicateChannel { channel: 0 })
    ));
}

#[test]
#[should_panic(expected = "invalid ActuatorMapping")]
fn new_panics_on_invalid_mapping() {
    let mut config = create_test_config(false);
    config.mapping.motors[1] = Some(12);
    RealFlightVehicle::new(config);
}

#[test]
fn test_actuate_success() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // Verify state was updated
    assert_eq!(vehicle.state.position_m.x, 2.0);
    assert_eq!(vehicle.state.velocity_mps.z, -6.0);
    assert_eq!(vehicle.state.acceleration_mps2.x, 7.0);
    assert_eq!(vehicle.state.attitude.w, 1.0);
}

#[test]
fn test_actuate_failure() {
    let config = create_test_config(true);
    let mut vehicle = RealFlightVehicle::new(config);
    let initial_state = vehicle.state.clone();
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // State should remain unchanged on failure
    assert_eq!(vehicle.state, initial_state);
}

#[test]
fn returns_cached_state() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);

    // Update state through actuation
    vehicle.actuate(&ActuatorCommand::default());
    let state = vehicle.state(Duration::from_millis(16));

    assert_eq!(state.position_m.x, 2.0);
    assert_eq!(state.velocity_mps.z, -6.0);
    assert_eq!(state.acceleration_mps2.x, 7.0);
}

#[test]
fn test_drop_behavior() {
    let config = create_test_config(false);
    {
        let _vehicle = RealFlightVehicle::new(config);
        // Vehicle dropped at end of scope
    }
    // Can't test enable_rc directly, but we can ensure it doesn't panic
}

#[test]
fn exchanges_once_per_tick_and_reports_state_from_the_previous_exchange() {
    const TICKS: usize = 3;
    let bridge = MockRealFlightBridge {
        stamp_exchanges: true,
        ..MockRealFlightBridge::new(false)
    };
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        mapping: ActuatorMapping::default(),
        body_rotation: UnitQuaternion::identity(),
        bridge,
    });
    let mut flight_loop = FlightLoop::new(
        ConstantCommander::default(),
        vehicle,
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        TableMixer::quad_x(),
        RecordingTelemetry::<TICKS>::new(),
    );

    flight_loop.prime();
    for _ in 0..TICKS {
        flight_loop.step(Duration::from_millis(4));
    }

    let (_, vehicle, _, _, _, telemetry) = flight_loop.into_parts();
    // One exchange to prime, then exactly one per tick (in `actuate`).
    assert_eq!(current_bridge(&vehicle).exchanges.get(), 1 + TICKS);
    // Tick n stabilizes on the state from exchange n: priming's for tick 1, then each previous
    // tick's `actuate`.
    let seen: Vec<f32> = telemetry.iter().map(|t| t.state.position_m.x).collect();
    assert_eq!(seen, [1.0, 2.0, 3.0]);
}

/// RealFlight's native report for a conformance scenario, built with the conventions the live
/// capture established (see `RealFlightVehicle`'s Frames docs): position east/north, velocity NED,
/// acceleration as FRD body specific force, quaternion in `(w, y, x, -z)` order, azimuth and yaw
/// rate counterclockwise-positive, degrees. `captured_frames_agree_across_fields` checks those
/// conventions against real RealFlight frames.
fn native_fixture(scenario: &Scenario) -> SimulatorState {
    let [north, east, up] = scenario.position_neu_m;
    let [v_north, v_east, v_up] = scenario.velocity_neu_mps;
    let [a_north, a_east, a_up] = scenario.acceleration_neu_mps2;
    let [roll_rate, pitch_rate, yaw_rate] = scenario.body_rates_degps;
    let attitude = UnitQuaternion::from_euler_angles(
        scenario.roll_deg.to_radians(),
        scenario.pitch_deg.to_radians(),
        scenario.heading_deg.to_radians(),
    );
    let specific_force = attitude.inverse() * Vec3::new(a_north, a_east, -a_up - GRAVITY);

    SimulatorState {
        aircraft_position_x_m: east,
        aircraft_position_y_m: north,
        altitude_asl_m: up,
        velocity_world_mps: [v_north, v_east, -v_up].into(),
        acceleration_body_mps2: [specific_force.x, specific_force.y, specific_force.z].into(),
        roll_deg: scenario.roll_deg,
        inclination_deg: scenario.pitch_deg,
        azimuth_deg: -scenario.heading_deg,
        roll_rate_dps: roll_rate,
        pitch_rate_dps: pitch_rate,
        yaw_rate_dps: -yaw_rate,
        orientation: Quaternion {
            w: attitude.w,
            x: attitude.j,
            y: attitude.i,
            z: -attitude.k,
        },
        ..Default::default()
    }
}

/// Converts RealFlight's native report for `scenario`.
fn state_for(scenario: &Scenario) -> VehicleState {
    vehicle_state(&native_fixture(scenario), &UnitQuaternion::identity())
}

stable_flight::conformance_tests! {
    state_for;
    conforms_level: LEVEL;
    conforms_roll_right_30: ROLL_RIGHT_30;
    conforms_pitch_up_15: PITCH_UP_15;
    conforms_heading_east: HEADING_EAST;
    conforms_banked_climb_east: BANKED_CLIMB_EAST;
    conforms_climbing: CLIMBING;
    conforms_moving_northeast: MOVING_NORTHEAST;
    conforms_yaw_right_rate: YAW_RIGHT_RATE;
    conforms_combined_rates: COMBINED_RATES;
}

/// The eXplora VTOL's body rotation: its RealFlight frame is hover-level, the vehicle's is the
/// airplane's, so RealFlight's level is nose up (#314).
fn explora_rotation() -> UnitQuaternion<f32> {
    UnitQuaternion::from_euler_angles(0.0, std::f32::consts::FRAC_PI_2, 0.0)
}

/// RealFlight's report for a vehicle whose body frame is `body_rotation` in RealFlight's:
/// `report` (built for the vehicle's frame) with attitude, body rates, and specific force moved
/// into RealFlight's frame, the inverse of `vehicle_state`'s rotation. World-frame fields stay.
fn mounted(report: SimulatorState, body_rotation: &UnitQuaternion<f32>) -> SimulatorState {
    let attitude = realflight_attitude(&report.orientation);
    let rates = (body_rotation * realflight_body_rates_radps(&report)).map(f32::to_degrees);
    let f = report.acceleration_body_mps2;
    let specific_force = body_rotation * Vec3::new(f.x, f.y, f.z);
    SimulatorState {
        orientation: realflight_orientation(&(attitude * body_rotation.inverse())),
        roll_rate_dps: rates.x,
        pitch_rate_dps: rates.y,
        yaw_rate_dps: -rates.z,
        acceleration_body_mps2: [specific_force.x, specific_force.y, specific_force.z].into(),
        ..report
    }
}

/// Every contract field survives the eXplora's rotation: RealFlight reports the scenario in its
/// hover-level frame, and the converted state still meets the contract.
mod explora_rotation_conformance {
    use super::*;

    fn state_for(scenario: &Scenario) -> VehicleState {
        let rotation = explora_rotation();
        vehicle_state(&mounted(native_fixture(scenario), &rotation), &rotation)
    }

    stable_flight::conformance_tests! {
        state_for;
        conforms_level: LEVEL;
        conforms_roll_right_30: ROLL_RIGHT_30;
        conforms_pitch_up_15: PITCH_UP_15;
        conforms_heading_east: HEADING_EAST;
        conforms_banked_climb_east: BANKED_CLIMB_EAST;
        conforms_climbing: CLIMBING;
        conforms_moving_northeast: MOVING_NORTHEAST;
        conforms_yaw_right_rate: YAW_RIGHT_RATE;
        conforms_combined_rates: COMBINED_RATES;
    }
}

/// The eXplora sitting on its tail reads RealFlight-level: with its rotation, that is nose up,
/// gravity along −x, at rest; RealFlight's body axes (x, y, z) become the vehicle's (z, y, −x),
/// for rates and specific force alike.
#[test]
fn explora_rotation_maps_realflight_level_to_nose_up() {
    let rest = SimulatorState {
        orientation: realflight_orientation(&UnitQuaternion::identity()),
        acceleration_body_mps2: [0.0, 0.0, -GRAVITY].into(),
        ..Default::default()
    };
    let state = vehicle_state(&rest, &explora_rotation());
    check!(
        (state.pitch_rad.to_degrees() - 90.0).abs() < 0.05,
        "{}",
        state.pitch_rad.to_degrees()
    );
    check!((state.attitude * Vec3::x() - Vec3::new(0.0, 0.0, -1.0)).norm() < 1e-5);
    check!(state.acceleration_mps2.norm() < 1e-4);

    // RealFlight (roll, pitch, yaw) rate in °/s → the vehicle's body rate (FRD, rad/s). Yaw is
    // RealFlight's counterclockwise-positive field: −1 is a turn about RealFlight's +z.
    let deg = 1.0_f32.to_radians();
    let cases = [
        ((1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, deg)),
        ((0.0, 1.0, 0.0), Vec3::new(0.0, deg, 0.0)),
        ((0.0, 0.0, -1.0), Vec3::new(-deg, 0.0, 0.0)),
    ];
    for ((roll, pitch, yaw), expected) in cases {
        let report = SimulatorState {
            roll_rate_dps: roll,
            pitch_rate_dps: pitch,
            yaw_rate_dps: yaw,
            ..Default::default()
        };
        let rate = vehicle_state(&report, &explora_rotation()).angular_velocity_radps;
        check!((rate - expected).norm() < 1e-6, "{rate:?} vs {expected:?}");
    }
}

/// At identity, the rotation changes nothing: attitude and body rates are RealFlight's, exactly.
#[test]
fn identity_body_rotation_is_a_no_op() {
    for scenario in stable_flight::testing::conformance::SCENARIOS {
        let report = native_fixture(scenario);
        let state = vehicle_state(&report, &UnitQuaternion::identity());
        check!(
            state.attitude == realflight_attitude(&report.orientation),
            "{}",
            scenario.name
        );
        check!(
            state.angular_velocity_radps == realflight_body_rates_radps(&report),
            "{}",
            scenario.name
        );
    }
}

#[test]
fn priming_failure_returns_the_default_state() {
    let mut vehicle = RealFlightVehicle::new(create_test_config(true));

    let state = vehicle.state(Duration::ZERO);

    assert_eq!(state, VehicleState::default());
    assert_eq!(current_bridge(&vehicle).exchanges.get(), 1);
}

/// Real RealFlight frames (QuadCopterX, `capture-state-realflight`): angles in degrees,
/// quaternion `(w, x, y, z)` as reported, world and body velocity in m/s.
struct CapturedFrame {
    roll_deg: f32,
    inclination_deg: f32,
    azimuth_deg: f32,
    orientation: [f32; 4],
    velocity_world: [f32; 3],
    velocity_body: [f32; 3],
}

#[rustfmt::skip]
const CAPTURED_FRAMES: &[CapturedFrame] = &[
    // Banked right.
    CapturedFrame { roll_deg: 66.83714, inclination_deg: 1.5162313, azimuth_deg: -121.246864,
        orientation: [0.41575912, 0.48530817, 0.2605223, -0.7237047],
        velocity_world: [-1.2908397, -1.7935158, 0.011943602],
        velocity_body: [-0.86376965, 0.79000425, -1.8742743] },
    // Banked left, descending.
    CapturedFrame { roll_deg: -69.56736, inclination_deg: 1.9636091, azimuth_deg: -126.60085,
        orientation: [0.36023846, -0.50325423, -0.26885852, -0.7380235],
        velocity_world: [4.0742207, 1.5021355, 5.37459],
        velocity_body: [-1.4067144, -6.4487357, -2.043745] },
    // Nose up.
    CapturedFrame { roll_deg: 0.24343026, inclination_deg: 56.832256, azimuth_deg: -120.71308,
        orientation: [0.43588683, 0.23699024, -0.41266477, -0.76390207],
        velocity_world: [1.8830783, -2.9502623, -0.2262258],
        velocity_body: [-1.7244745, -0.12510146, -3.0515168] },
    // Nose down.
    CapturedFrame { roll_deg: -1.0703444, inclination_deg: -49.82929, azimuth_deg: -123.159,
        orientation: [-0.43508804, 0.2079391, -0.3664479, 0.7957234],
        velocity_world: [-1.5399728, 0.026480667, -0.79843163],
        velocity_body: [-0.052455604, 1.2964513, -1.1515958] },
    // Fast, banked and pitched, heading northwest.
    CapturedFrame { roll_deg: 12.693953, inclination_deg: 10.00485, azimuth_deg: -34.863396,
        orientation: [-0.94750285, -0.11567435, -0.07910883, 0.2874015],
        velocity_world: [-4.1126347, -1.9234412, 5.8557076],
        velocity_body: [-5.423254, 1.8501548, 4.697653] },
];

/// RealFlight reports attitude twice (Euler fields and quaternion) and velocity twice (world and
/// body). In real frames, the converted quaternion must agree with the converted Euler fields,
/// and rotate the converted world velocity onto the body velocity. That pins the quaternion axis
/// order, the azimuth sign, and the velocity frame without trusting any documentation.
#[test]
fn captured_frames_agree_across_fields() {
    for (i, frame) in CAPTURED_FRAMES.iter().enumerate() {
        let [w, x, y, z] = frame.orientation;
        let state = vehicle_state(
            &SimulatorState {
                roll_deg: frame.roll_deg,
                inclination_deg: frame.inclination_deg,
                azimuth_deg: frame.azimuth_deg,
                orientation: Quaternion { w, x, y, z },
                velocity_world_mps: frame.velocity_world.into(),
                ..Default::default()
            },
            &UnitQuaternion::identity(),
        );

        let (roll, pitch, heading) = stable_flight::euler_angles(&state.attitude);
        assert!(
            (roll - state.roll_rad).abs() < 1e-3,
            "frame {i}: roll {roll} vs {}",
            state.roll_rad
        );
        assert!(
            (pitch - state.pitch_rad).abs() < 1e-3,
            "frame {i}: pitch {pitch} vs {}",
            state.pitch_rad
        );
        assert!(
            wrap_angle(heading - state.heading_rad).abs() < 1e-3,
            "frame {i}: heading {heading} vs {}",
            state.heading_rad
        );

        let v = state.velocity_mps;
        let v_ned = Vec3::new(v.x, v.y, -v.z);
        let body = state.attitude.inverse() * v_ned;
        let expected = Vec3::from(frame.velocity_body);
        assert!(
            (body - expected).amax() < 1e-3,
            "frame {i}: body velocity {body:?} vs {expected:?}"
        );
    }
}

/// Tick period for the reconnect tests; thresholds sit a tick or more away from it.
const RECONNECT_DT: Duration = Duration::from_millis(100);

/// A vehicle whose bridge fails every exchange: with `Connection` errors when
/// `connection_lost`, otherwise with a non-connection error.
fn failing_vehicle(connection_lost: bool) -> RealFlightVehicle<MockRealFlightBridge> {
    RealFlightVehicle::new(RealFlightConfiguration {
        mapping: ActuatorMapping::default(),
        body_rotation: UnitQuaternion::identity(),
        bridge: MockRealFlightBridge {
            connection_lost,
            should_fail: !connection_lost,
            ..MockRealFlightBridge::new(false)
        },
    })
}

/// A reconnect hook that counts its calls and builds bridges with `make`.
fn counting_reconnect(
    make: impl Fn() -> Result<MockRealFlightBridge, BridgeError> + Send + 'static,
) -> (
    Arc<AtomicUsize>,
    impl FnMut() -> Result<MockRealFlightBridge, BridgeError> + Send + 'static,
) {
    let calls = Arc::new(AtomicUsize::new(0));
    let counter = Arc::clone(&calls);
    (calls, move || {
        counter.fetch_add(1, Ordering::Relaxed);
        make()
    })
}

/// The vehicle's current bridge; panics if a lost connection dropped it.
fn current_bridge(vehicle: &RealFlightVehicle<MockRealFlightBridge>) -> &MockRealFlightBridge {
    vehicle.bridge.as_ref().expect("has a bridge")
}

/// Makes the vehicle calls of `ticks` flight-loop ticks.
fn tick(vehicle: &mut RealFlightVehicle<MockRealFlightBridge>, ticks: usize) {
    for _ in 0..ticks {
        vehicle.state(RECONNECT_DT);
        vehicle.actuate(&ActuatorCommand::default());
    }
}

#[test]
fn reconnects_after_the_backoff_and_resumes_state_updates() {
    let (calls, reconnect) = counting_reconnect(|| {
        Ok(MockRealFlightBridge {
            stamp_exchanges: true,
            ..MockRealFlightBridge::new(false)
        })
    });
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    // Tick 1 loses the connection and drops the bridge; the next 0.4 s neither reconnects nor
    // exchanges.
    tick(&mut vehicle, 5);
    assert_eq!(calls.load(Ordering::Relaxed), 0);
    assert!(vehicle.bridge.is_none());
    assert_eq!(vehicle.state, VehicleState::default());

    // Past 0.5 s: one attempt, and the new bridge's first exchange updates the state.
    tick(&mut vehicle, 2);
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert!(!current_bridge(&vehicle).connection_lost);
    assert_eq!(vehicle.state.position_m.x, 1.0);

    // Back to one exchange per tick.
    tick(&mut vehicle, 3);
    assert_eq!(current_bridge(&vehicle).exchanges.get(), 4);
    assert_eq!(calls.load(Ordering::Relaxed), 1);
}

#[test]
fn a_failed_reconnect_backs_off_before_the_next_attempt() {
    let (calls, reconnect) = counting_reconnect(|| {
        Err(BridgeError::Connection(std::io::Error::other(
            "connection refused",
        )))
    });
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    // Lost on tick 1, first attempt past 0.5 s, the next one 1 s later.
    tick(&mut vehicle, 7);
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    tick(&mut vehicle, 8);
    assert_eq!(calls.load(Ordering::Relaxed), 1, "retried within 1 s");
    tick(&mut vehicle, 3);
    assert_eq!(calls.load(Ordering::Relaxed), 2);
}

#[test]
fn a_new_bridge_that_loses_its_connection_on_disable_rc_is_not_used() {
    let (calls, reconnect) = counting_reconnect(|| {
        Ok(MockRealFlightBridge {
            connection_lost: true,
            ..MockRealFlightBridge::new(false)
        })
    });
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    tick(&mut vehicle, 7);

    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert!(vehicle.bridge.is_none(), "kept the new bridge");
}

#[test]
fn a_new_bridge_whose_disable_rc_fails_otherwise_is_used() {
    // RealFlight answers "Preexisting controller reference" when the Link already has control.
    let (calls, reconnect) = counting_reconnect(|| {
        Ok(MockRealFlightBridge {
            disable_rc_fault: true,
            ..MockRealFlightBridge::new(false)
        })
    });
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    tick(&mut vehicle, 7);

    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert!(
        !current_bridge(&vehicle).connection_lost,
        "kept the old bridge instead of the new one"
    );
    assert_eq!(vehicle.state.position_m.x, 2.0);
}

#[test]
fn a_non_connection_error_retries_the_exchange_without_reconnecting() {
    let (calls, reconnect) = counting_reconnect(|| Ok(MockRealFlightBridge::new(false)));
    let mut vehicle = failing_vehicle(false).with_reconnect(reconnect);

    tick(&mut vehicle, 20);

    assert_eq!(calls.load(Ordering::Relaxed), 0);
    assert_eq!(current_bridge(&vehicle).exchanges.get(), 20);
}

#[test]
fn without_a_reconnect_hook_a_connection_error_retries_every_tick() {
    let mut vehicle = failing_vehicle(true);

    tick(&mut vehicle, 20);

    assert!(!vehicle.health.is_down());
    assert_eq!(current_bridge(&vehicle).exchanges.get(), 20);
}

#[test]
fn a_failed_priming_exchange_starts_the_backoff() {
    let (calls, reconnect) = counting_reconnect(|| Ok(MockRealFlightBridge::new(false)));
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    vehicle.state(Duration::ZERO);
    assert!(vehicle.health.is_down());

    tick(&mut vehicle, 6);
    assert_eq!(calls.load(Ordering::Relaxed), 1);
    assert_eq!(vehicle.state.position_m.x, 2.0);
}

#[test]
fn a_lost_connection_is_closed_before_the_reconnect() {
    // The RealFlight proxy serves one client at a time: while the old bridge's connection stays
    // open, the new bridge's calls queue behind it and time out.
    let connection = Arc::new(());
    let mut vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        mapping: ActuatorMapping::default(),
        body_rotation: UnitQuaternion::identity(),
        bridge: MockRealFlightBridge {
            connection_lost: true,
            _connection: Some(Arc::clone(&connection)),
            ..MockRealFlightBridge::new(false)
        },
    });
    let open_at_reconnect = Arc::new(AtomicUsize::new(usize::MAX));
    let observed = Arc::clone(&open_at_reconnect);
    let watched = Arc::clone(&connection);
    vehicle = vehicle.with_reconnect(move || {
        // Minus the test's two handles: whether the old bridge still holds it.
        observed.store(Arc::strong_count(&watched) - 2, Ordering::Relaxed);
        Ok(MockRealFlightBridge::new(false))
    });

    tick(&mut vehicle, 1);
    assert_eq!(
        Arc::strong_count(&connection),
        2,
        "still open after the loss"
    );

    tick(&mut vehicle, 6);
    assert_eq!(open_at_reconnect.load(Ordering::Relaxed), 0);
}

#[test]
fn a_vehicle_with_a_reconnect_hook_is_send() {
    fn assert_send<T: Send>(_: &T) {}
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        mapping: ActuatorMapping::default(),
        body_rotation: UnitQuaternion::identity(),
        bridge: MockRealFlightBridge::new(false),
    })
    .with_reconnect(|| Ok(MockRealFlightBridge::new(false)));

    assert_send(&vehicle);
}

/// RealFlight's orientation for an FRD `attitude`: the adapter's (y, x, -z) axis swap inverted.
fn realflight_orientation(attitude: &UnitQuaternion<f32>) -> Quaternion {
    let q = attitude.quaternion();
    Quaternion {
        w: q.w,
        x: q.j,
        y: q.i,
        z: -q.k,
    }
}

/// The Euler fields come from the quaternion, so they agree with `attitude` at any pitch,
/// including nose-up (a tailsitter's hover, #120), where RealFlight's own roll and azimuth are
/// degenerate; RealFlight's Euler angles are ignored.
#[test]
fn euler_fields_follow_the_quaternion_nose_up() {
    // (roll, pitch, heading) in rad
    let cases = [
        (0.0, 0.0, 0.0),
        (0.3, -0.2, 2.0),
        (0.0, 1.5, 0.7),
        (0.2, std::f32::consts::FRAC_PI_2, 1.0),
        (-0.4, -1.55, -2.5),
    ];
    for (roll, pitch, heading) in cases {
        let attitude = UnitQuaternion::from_euler_angles(roll, pitch, heading);
        let report = SimulatorState {
            orientation: realflight_orientation(&attitude),
            // Nonsense Euler angles: they must not reach the state.
            roll_deg: 99.0,
            inclination_deg: -99.0,
            azimuth_deg: 99.0,
            ..Default::default()
        };

        let state = vehicle_state(&report, &UnitQuaternion::identity());

        check!(state.attitude.angle_to(&attitude) < 1e-5, "{pitch}");
        let (r, p, h) = euler_angles(&state.attitude);
        check!(
            (state.roll_rad, state.pitch_rad, state.heading_rad) == (r, p, h),
            "{pitch}"
        );
        // FRD x (the nose) points up when pitched to 90°.
        if pitch == std::f32::consts::FRAC_PI_2 {
            let nose = state.attitude * Vec3::x();
            check!((nose.z + 1.0).abs() < 1e-5, "{nose:?}");
        }
    }
}
