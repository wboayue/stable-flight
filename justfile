# ARK FPV firmware: a nested workspace, built from its own dir so its .cargo/config.toml applies
fw := "firmware/ark-fpv"
fw_bin := "ark-fpv.bin"

# Builds every workspace crate
build:
    cargo build --workspace

# Runs tests
test:
    cargo test --workspace --all-features

# Builds the no_std core for a bare-metal target, with and without `testing`, and the firmware (as CI does)
embedded: && fw-build
    rustup target add thumbv7em-none-eabihf
    cargo build -p stable-flight --target thumbv7em-none-eabihf
    cargo build -p stable-flight --features testing --target thumbv7em-none-eabihf

# Release build of the ARK FPV firmware
fw-build:
    cd {{fw}} && cargo build --release

# Flash/RAM usage of the firmware (needs cargo-binutils + llvm-tools)
fw-size: fw-build
    cd {{fw}} && cargo size --release

# Firmware as a raw binary for DFU
fw-bin: fw-build
    cd {{fw}} && cargo objcopy --release -- -O binary {{fw_bin}}

# Flash over USB DFU. Put the board in DFU first: hold BOOT0, tap NRST, release BOOT0 (the stub has no USB reboot)
fw-flash: fw-bin
    #!/usr/bin/env bash
    set -euo pipefail
    if ! dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
        echo "no DFU device (0483:df11). Hold BOOT0, tap NRST, release BOOT0 first."; exit 1
    fi
    # dfu-util exits non-zero on the flaky :leave even after a good write, so judge success
    # by the download line instead of the exit status.
    log=$(mktemp)
    dfu-util -a 0 -s 0x08000000:leave -D {{fw}}/{{fw_bin}} 2>&1 | tee "$log" || true
    if ! grep -qE "Download done|File downloaded successfully" "$log"; then
        echo; echo ">>> FLASH FAILED (dfu-util never reported a completed download)."; exit 1
    fi
    echo
    echo ">>> Flash done. PRESS NRST to boot cleanly (:leave auto-run is unreliable)."
    echo ">>> 'get_status' / 'Invalid DFU suffix' warnings above are benign."

# Generate and save coverage report using tarpaulin
cover:
    cargo tarpaulin --workspace -o html
    echo "coverage report saved to tarpaulin-report.html"

# Joystick-flown quadcopter in RealFlight, e.g. `just fly-manual --host 192.168.1.20:8080`
fly-manual *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- {{ARGS}}

# Scripted figure-eight in RealFlight; args: [script.csv] [--host host:port]
fly-auto *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- {{ARGS}}

# Renders a waypoint script as a 3D flight path, e.g. `just plot flights/figure_eight.csv out.svg`
plot *ARGS:
    cargo run --bin plot_flight -- {{ARGS}}

# (Re)writes flights/figure_eight.csv; flags such as --half-width resize the path
gen-figure-eight *ARGS:
    cargo run --bin generate_figure_eight -- {{ARGS}}

# Tags a crate release, e.g. `just tag stable-flight-sitl 0.1.0` -> stable-flight-sitl-v0.1.0
tag CRATE VERSION:
    echo "Tagging repo with {{CRATE}}-v{{VERSION}}"
    git tag {{CRATE}}-v{{VERSION}} -m "{{CRATE}} {{VERSION}}"
    git push origin {{CRATE}}-v{{VERSION}}

# Lists all available versions
versions:
    @git tag
