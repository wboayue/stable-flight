use super::*;
use realflight_bridge::{BridgeError, ControlInputs, Quaternion, RealFlightBridge, SimulatorState};
use stable_flight::FlightLoop;
use stable_flight::guidance::copter::{CopterGuidance, CopterGuidanceConfig};
use stable_flight::mixers::QuadCopterMixer;
use stable_flight::nalgebra::UnitQuaternion;
use stable_flight::stabilizers::copter::{CopterStabilizer, CopterStabilizerConfig};
use stable_flight::testing::conformance::Scenario;
use stable_flight::testing::{ConstantCommander, RecordingTelemetry};
use stable_flight::{Vec3, wrap_angle};
use std::{
    cell::{Cell, RefCell},
    rc::Rc,
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
    /// Fails every exchange with a `Connection` error, as an out-of-sync remote bridge does.
    connection_lost: bool,
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
        if self.should_fail {
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
    mapping.rudder = Some(1);
    mapping.elevator = Some(2);
    mapping.aileron = Some(3);
    mapping.throttle = Some(4);
    mapping.elevon_left = Some(5);
    mapping.elevon_right = Some(6);

    RealFlightConfiguration {
        mapping,
        bridge: MockRealFlightBridge::new(should_fail),
    }
}

#[test]
fn test_vehicle_creation() {
    let config = create_test_config(false);
    let vehicle = RealFlightVehicle::new(config);

    // Verify initial state
    assert_eq!(vehicle.estimated_state.position_m, Vec3::zeros());
    assert_eq!(vehicle.mapping.motors[0], Some(0));
    assert_eq!(vehicle.mapping.rudder, Some(1));
}

#[test]
fn test_vehicle_creation_with_bridge_failure() {
    let config = create_test_config(true);
    let vehicle = RealFlightVehicle::new(config);

    // Should still create successfully despite bridge failures
    assert_eq!(vehicle.estimated_state.position_m, Vec3::zeros());
}

#[test]
fn test_channel_mapping() {
    let config = create_test_config(false);
    let vehicle = RealFlightVehicle::new(config);

    let command = ActuatorCommand {
        motors: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        rudder: -0.3,
        elevator: 0.7,
        aileron: -0.1,
        throttle: 1.0,
        elevon_left: 0.4,
        elevon_right: -0.4,
    };

    let inputs = vehicle.map_channels(&command);

    assert_eq!(inputs.channels[0], 0.5); // motor 0
    assert_eq!(inputs.channels[1], -0.3); // rudder
    assert_eq!(inputs.channels[2], 0.7); // elevator
    assert_eq!(inputs.channels[3], -0.1); // aileron
    assert_eq!(inputs.channels[4], 1.0); // throttle
    assert_eq!(inputs.channels[5], 0.4); // elevon_left
    assert_eq!(inputs.channels[6], -0.4); // elevon_right
}

#[test]
fn test_actuate_success() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // Verify state was updated
    assert_eq!(vehicle.estimated_state.position_m.x, 2.0);
    assert_eq!(vehicle.estimated_state.velocity_mps.z, -6.0);
    assert_eq!(vehicle.estimated_state.acceleration_mps2.x, 7.0);
    assert_eq!(vehicle.estimated_state.attitude.w, 1.0);
}

#[test]
fn test_actuate_failure() {
    let config = create_test_config(true);
    let mut vehicle = RealFlightVehicle::new(config);
    let initial_state = vehicle.estimated_state.clone();
    let command = ActuatorCommand::default();

    vehicle.actuate(&command);

    // State should remain unchanged on failure
    assert_eq!(vehicle.estimated_state, initial_state);
}

#[test]
fn test_estimate_state() {
    let config = create_test_config(false);
    let mut vehicle = RealFlightVehicle::new(config);

    // Update state through actuation
    vehicle.actuate(&ActuatorCommand::default());
    let state = vehicle.estimate_state(Duration::from_millis(16));

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
fn test_partial_mapping() {
    let mut mapping = ActuatorMapping::default();
    mapping.motors[0] = Some(0); // Only map motor 0
    let config = RealFlightConfiguration {
        mapping,
        bridge: MockRealFlightBridge::new(false),
    };
    let vehicle = RealFlightVehicle::new(config);

    let command = ActuatorCommand {
        motors: [0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ..Default::default()
    };

    let inputs = vehicle.map_channels(&command);
    assert_eq!(inputs.channels[0], 0.5); // Mapped
    assert_eq!(inputs.channels[1], 0.0); // Unmapped
}

#[test]
fn exchanges_once_per_tick_and_estimates_from_the_previous_exchange() {
    const TICKS: usize = 3;
    let bridge = MockRealFlightBridge {
        stamp_exchanges: true,
        ..MockRealFlightBridge::new(false)
    };
    let vehicle = RealFlightVehicle::new(RealFlightConfiguration {
        mapping: ActuatorMapping::default(),
        bridge,
    });
    let mut flight_loop = FlightLoop::new(
        ConstantCommander::default(),
        vehicle,
        CopterGuidance::new(CopterGuidanceConfig::default()),
        CopterStabilizer::new(CopterStabilizerConfig::default()),
        QuadCopterMixer::default(),
        RecordingTelemetry::<TICKS>::new(),
    );

    flight_loop.prime();
    for _ in 0..TICKS {
        flight_loop.step(Duration::from_millis(4));
    }

    let (_, vehicle, _, _, _, telemetry) = flight_loop.into_parts();
    // One exchange to prime, then exactly one per tick (in `actuate`).
    assert_eq!(vehicle.bridge.exchanges.get(), 1 + TICKS);
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
    vehicle_state(&native_fixture(scenario))
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

#[test]
fn priming_failure_returns_the_default_state() {
    let mut vehicle = RealFlightVehicle::new(create_test_config(true));

    let state = vehicle.estimate_state(Duration::ZERO);

    assert_eq!(state, VehicleState::default());
    assert_eq!(vehicle.bridge.exchanges.get(), 1);
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
        let state = vehicle_state(&SimulatorState {
            roll_deg: frame.roll_deg,
            inclination_deg: frame.inclination_deg,
            azimuth_deg: frame.azimuth_deg,
            orientation: Quaternion { w, x, y, z },
            velocity_world_mps: frame.velocity_world.into(),
            ..Default::default()
        });

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
        bridge: MockRealFlightBridge {
            connection_lost,
            should_fail: !connection_lost,
            ..MockRealFlightBridge::new(false)
        },
    })
}

/// A reconnect hook that counts its calls and builds bridges with `make`.
fn counting_reconnect(
    make: impl Fn() -> Result<MockRealFlightBridge, BridgeError> + 'static,
) -> (
    Rc<Cell<usize>>,
    impl FnMut() -> Result<MockRealFlightBridge, BridgeError> + 'static,
) {
    let calls = Rc::new(Cell::new(0));
    let counter = Rc::clone(&calls);
    (calls, move || {
        counter.set(counter.get() + 1);
        make()
    })
}

/// Makes the vehicle calls of `ticks` flight-loop ticks.
fn tick(vehicle: &mut RealFlightVehicle<MockRealFlightBridge>, ticks: usize) {
    for _ in 0..ticks {
        vehicle.estimate_state(RECONNECT_DT);
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

    // Tick 1 loses the connection; the next 0.4 s neither reconnects nor exchanges.
    tick(&mut vehicle, 5);
    assert_eq!(calls.get(), 0);
    assert_eq!(vehicle.bridge.exchanges.get(), 1);
    assert_eq!(vehicle.estimated_state, VehicleState::default());

    // Past 0.5 s: one attempt, and the new bridge's first exchange updates the state.
    tick(&mut vehicle, 2);
    assert_eq!(calls.get(), 1);
    assert!(!vehicle.bridge.connection_lost);
    assert_eq!(vehicle.estimated_state.position_m.x, 1.0);

    // Back to one exchange per tick.
    tick(&mut vehicle, 3);
    assert_eq!(vehicle.bridge.exchanges.get(), 4);
    assert_eq!(calls.get(), 1);
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
    assert_eq!(calls.get(), 1);
    tick(&mut vehicle, 8);
    assert_eq!(calls.get(), 1, "retried within 1 s");
    tick(&mut vehicle, 3);
    assert_eq!(calls.get(), 2);
}

#[test]
fn a_new_bridge_that_rejects_disable_rc_is_not_used() {
    // `MockRealFlightBridge::new(true)` fails `disable_rc`.
    let (calls, reconnect) = counting_reconnect(|| Ok(MockRealFlightBridge::new(true)));
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    tick(&mut vehicle, 7);

    assert_eq!(calls.get(), 1);
    assert!(vehicle.bridge.connection_lost, "kept the old bridge");
    assert_eq!(vehicle.bridge.exchanges.get(), 1, "exchanged while down");
}

#[test]
fn a_non_connection_error_retries_the_exchange_without_reconnecting() {
    let (calls, reconnect) = counting_reconnect(|| Ok(MockRealFlightBridge::new(false)));
    let mut vehicle = failing_vehicle(false).with_reconnect(reconnect);

    tick(&mut vehicle, 20);

    assert_eq!(calls.get(), 0);
    assert_eq!(vehicle.bridge.exchanges.get(), 20);
}

#[test]
fn without_a_reconnect_hook_a_connection_error_retries_every_tick() {
    let mut vehicle = failing_vehicle(true);

    tick(&mut vehicle, 20);

    assert!(!vehicle.health.is_down());
    assert_eq!(vehicle.bridge.exchanges.get(), 20);
}

#[test]
fn a_failed_priming_exchange_starts_the_backoff() {
    let (calls, reconnect) = counting_reconnect(|| Ok(MockRealFlightBridge::new(false)));
    let mut vehicle = failing_vehicle(true).with_reconnect(reconnect);

    vehicle.estimate_state(Duration::ZERO);
    assert!(vehicle.health.is_down());

    tick(&mut vehicle, 6);
    assert_eq!(calls.get(), 1);
    assert_eq!(vehicle.estimated_state.position_m.x, 2.0);
}
