# ark-fpv

Stable Flight firmware for the [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight
controller (STM32H743, Cortex-M7), on [RTIC 2](https://rtic.rs/2/book/en/). Board setup (clocks,
LEDs, memory map, flash flow) comes from
[ark-fpv-discovery](https://github.com/wboayue/ark-fpv-discovery).

**Status: stub.** It runs the core `FlightLoop` (`CopterGuidance` → `CopterStabilizer` →
`TableMixer::quad_x`) at 250 Hz on the SysTick monotonic (I-cache on), with placeholder components
in `src/stub.rs`:

| Component | Stub | Replaced by ([`ark-fpv` milestone](https://github.com/wboayue/stable-flight/milestone/3)) |
|---|---|---|
| Commander | `HoldCommander`: zero throttle, level, disarmed (armed with `bench-armed`) | receiver input |
| State source | `HoldState`: level, at rest | IMU/baro/mag fusion from ark-fpv-discovery |
| Actuator | `NullActuator`: discards motor commands | ESC output (PWM/DShot) |
| Telemetry | `()` | USB telemetry |

No sensors are read and no motors are driven.

## Build

This is a nested workspace, excluded from the repo root. Build it from this directory so
`.cargo/config.toml` (target `thumbv7em-none-eabihf`, linker script) applies. Running with
`--manifest-path` from the root would target the host instead. From the repo root, the `just`
recipes handle this:

```bash
just fw-build   # release build
just fw-size    # flash/RAM usage (cargo install cargo-binutils; rustup component add llvm-tools)
```

`just fw-build bench-armed` (or `cargo build --release --features bench-armed` here) arms
`HoldCommander`, so each tick runs guidance, the stabilizer, and the mixer instead of the
disarmed short path; use it to check loop timing on the board (blue LED). Flash is the same
either way. Never ship it with real ESC output.

The release profile is size-tuned (`opt-level = "s"`, LTO). CI builds the firmware and prints
its size, so a core change that breaks the link fails the PR and flash growth shows in the log.

## Flash

The stub has no USB, so enter the ROM bootloader by hand: hold **BOOT0**, tap **NRST**, release
**BOOT0**, then:

```bash
just fw-flash   # objcopy → dfu-util to 0x08000000 (needs dfu-util); fails unless the download completes
```

Press **NRST** afterwards to boot cleanly.

## LEDs

| LED | Meaning |
|---|---|
| Green | Toggles every second: firmware alive |
| Blue | Last second: the loop ran 250 ± 1 ticks, none overran its 4 ms slot |
| Red (green toggling) | Last second: the loop rate was off or a tick overran |
| Red alone, green stopped | Panic: interrupts off, firmware halted |
