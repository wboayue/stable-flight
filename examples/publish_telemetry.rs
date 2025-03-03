use mavlink::{MavlinkVersion, MavHeader, MavConnection};
use mavlink::common::*; // Import common MAVLink messages
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Establish MAVLink connection (UDP example)
    let mut connection = mavlink::connect("udpout:127.0.0.1:14550")?;
    
    // Set protocol version
    connection.set_protocol_version(MavlinkVersion::V2);
    
    // System and component IDs
    let header = MavHeader {
        system_id: 1,    // Your system ID
        component_id: 1, // Your component ID
        sequence: 0,
    };

    // Simulated flight loop
    let mut sequence = 0;
    let mut last_heartbeat = 0;
    let heartbeat_interval = 1000; // 1 Hz

    let mut x = 10.5;

    loop {
        // Update sequence number
        sequence = (sequence + 1) % 255;

        let current_time = get_time_ms();

        // Send heartbeat at 1 Hz
        if current_time - last_heartbeat >= heartbeat_interval {
            let heartbeat = HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            };

            connection.send(&header, &MavMessage::HEARTBEAT(heartbeat))?;

            last_heartbeat = current_time;
        }
        
        x += 0.1;
        // 1. Publish Estimated Position and Velocity (LOCAL_POSITION_NED)
        let local_position_msg = LOCAL_POSITION_NED_DATA {
            time_boot_ms: get_time_ms(),
            x,    // North position (meters)
            y: 5.2,     // East position (meters)
            z: -2.1,    // Down position (meters)
            vx: 1.0,    // North velocity (m/s)
            vy: 0.5,    // East velocity (m/s)
            vz: -0.1,   // Down velocity (m/s)
        };
        
        connection.send(&MavHeader {
            sequence,
            ..header
        }, &mavlink::common::MavMessage::LOCAL_POSITION_NED(local_position_msg))?;

        // 2. Publish Desired Position (SET_POSITION_TARGET_LOCAL_NED)
        let set_position_msg = SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: get_time_ms(),
            target_system: 1,
            target_component: 1,
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED,
            type_mask: PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
                      | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE 
                      | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE,
            x: 15.0,    // Desired North position
            y: 7.0,     // Desired East position
            z: -1.5,    // Desired Down position
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw: 0.0,
            yaw_rate: 0.0,
        };
        
        connection.send(&MavHeader {
            sequence,
            ..header
        }, &mavlink::common::MavMessage::SET_POSITION_TARGET_LOCAL_NED(set_position_msg))?;

        // 3. Publish Attitude (ATTITUDE)
        let attitude_msg = ATTITUDE_DATA {
            time_boot_ms: get_time_ms(),
            roll: 0.1,      // Roll angle (radians)
            pitch: 0.05,    // Pitch angle (radians)
            yaw: 0.2,       // Yaw angle (radians)
            rollspeed: 0.01, // Roll angular speed (rad/s)
            pitchspeed: 0.005, // Pitch angular speed (rad/s)
            yawspeed: 0.02,   // Yaw angular speed (rad/s)
        };
        
        connection.send(&MavHeader {
            sequence,
            ..header
        }, &mavlink::common::MavMessage::ATTITUDE(attitude_msg))?;

        // Sleep for 100ms (10Hz update rate)
        thread::sleep(Duration::from_millis(100));
    }
}

// Helper function to get system time in milliseconds
fn get_time_ms() -> u32 {
    use std::time::SystemTime;
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u32
}

// fn send_heartbeat<T : mavlink::Message>(
//     conn: &mut Box<dyn MavConnection<T>>,
//     header: MavHeader
// ) -> Result<(), Box<dyn std::error::Error>> {
//     let heartbeat = HEARTBEAT_DATA {
//         custom_mode: 0,
//         mavtype: MavType::MAV_TYPE_QUADROTOR,
//         autopilot: MavAutopilot::MAV_AUTOPILOT_PX4,
//         base_mode: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
//         system_status: MavState::MAV_STATE_ACTIVE,
//         mavlink_version: 3,
//     };
//     conn.send(&header, &MavMessage::HEARTBEAT(heartbeat))?;
//     Ok(())
// }

// Add these dependencies to your Cargo.toml:
// [dependencies]
// mavlink = "0.9"
// Add this if you get build errors:
// [build-dependencies]
// mavlink = "0.9"