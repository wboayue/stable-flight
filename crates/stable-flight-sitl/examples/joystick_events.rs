//! Prints every gamepad event gilrs delivers, one CSV row each (`ms,gamepad,event`), to
//! explain what a commander saw: a press it missed, or one it read that nobody made.
//!
//! As each gamepad connects it prints its name, mapping source, and the native code
//! behind every standard axis and button (the codes `ButtonChanged` and `AxisChanged` carry).
//! `--raw` turns off gilrs's default filters (`axis_dpad_to_button`, jitter, dead zone), as
//! the commanders don't: the dump then shows the axis events a hat arrives as on macOS, before
//! they become D-pad buttons. Runs until Ctrl-C, or for `--seconds`.
//!
//! ```text
//! cargo run -p stable-flight-sitl --example joystick_events > dump.csv
//! cargo run -p stable-flight-sitl --example joystick_events -- --raw --seconds 20 > raw.csv
//! ```

use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use gilrs::{Axis, Button, EventType, Gamepad, GamepadId, GilrsBuilder};

/// Dumps gilrs gamepad events as CSV.
#[derive(Parser)]
struct Args {
    /// Turn off gilrs's default event filters.
    #[arg(long)]
    raw: bool,
    /// Stop after this many seconds (default: run until Ctrl-C).
    #[arg(long)]
    seconds: Option<f64>,
}

const AXES: [Axis; 8] = [
    Axis::LeftStickX,
    Axis::LeftStickY,
    Axis::LeftZ,
    Axis::RightStickX,
    Axis::RightStickY,
    Axis::RightZ,
    Axis::DPadX,
    Axis::DPadY,
];

const BUTTONS: [Button; 19] = [
    Button::South,
    Button::East,
    Button::North,
    Button::West,
    Button::C,
    Button::Z,
    Button::LeftTrigger,
    Button::LeftTrigger2,
    Button::RightTrigger,
    Button::RightTrigger2,
    Button::Select,
    Button::Start,
    Button::Mode,
    Button::LeftThumb,
    Button::RightThumb,
    Button::DPadUp,
    Button::DPadDown,
    Button::DPadLeft,
    Button::DPadRight,
];

/// Polling period: well under a press, light on the CPU.
const POLL_PERIOD: Duration = Duration::from_millis(1);

fn main() {
    let args = Args::parse();
    let mut gilrs = GilrsBuilder::new()
        .with_default_filters(!args.raw)
        .build()
        .expect("failed to open gilrs");

    let start = Instant::now();
    let limit = args.seconds.map(Duration::from_secs_f64);
    println!("ms,gamepad,event");
    while limit.is_none_or(|limit| start.elapsed() < limit) {
        while let Some(event) = gilrs.next_event() {
            if event.event == EventType::Connected {
                describe(event.id, &gilrs.gamepad(event.id));
            }
            println!(
                "{:.1},{},{:?}",
                start.elapsed().as_secs_f64() * 1e3,
                event.id,
                event.event
            );
        }
        gilrs.inc();
        thread::sleep(POLL_PERIOD);
    }
}

/// Prints a gamepad's identity and the native code of each mapped axis and button to stderr,
/// keeping stdout the event CSV.
fn describe(id: GamepadId, gamepad: &Gamepad<'_>) {
    eprintln!(
        "gamepad {id}: {} (os name {}), uuid {:02x?}, mapping {:?}",
        gamepad.name(),
        gamepad.os_name(),
        gamepad.uuid(),
        gamepad.mapping_source()
    );
    for axis in AXES {
        if let Some(code) = gamepad.axis_code(axis) {
            eprintln!("  axis {axis:?}: code {} ({code})", code.into_u32());
        }
    }
    for button in BUTTONS {
        if let Some(code) = gamepad.button_code(button) {
            eprintln!("  button {button:?}: code {} ({code})", code.into_u32());
        }
    }
}
