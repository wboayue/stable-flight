# ARK FPV firmware: a nested workspace, built from its own dir so its .cargo/config.toml applies
fw := "firmware/ark-fpv"
fw_bin := "ark-fpv.bin"

# Builds every workspace crate
build:
    cargo build --workspace

# Runs tests
test:
    cargo test --workspace --all-features

# Clippy and rustdoc with warnings as errors (as CI does)
lint:
    cargo clippy --workspace --all-targets --all-features -- -D warnings
    RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps

# Builds the no_std core (plain, `testing`, `serde`, `param-metadata`) and link crates for a bare-metal target, and the firmware (as CI does)
embedded: && fw-build
    rustup target add thumbv7em-none-eabihf
    cargo build -p stable-flight --target thumbv7em-none-eabihf
    cargo build -p stable-flight --features testing --target thumbv7em-none-eabihf
    cargo build -p stable-flight --features serde --target thumbv7em-none-eabihf
    cargo build -p stable-flight --features param-metadata --target thumbv7em-none-eabihf
    cargo build -p stable-flight-link --target thumbv7em-none-eabihf
    cargo build -p stable-flight-link --features testing --target thumbv7em-none-eabihf

# Release build of the ARK FPV firmware; `just fw-build bench-armed` arms the stub for timing checks
fw-build features="":
    cd {{fw}} && cargo build --release {{ if features != "" { "--features " + features } else { "" } }}

# Flash/RAM usage of the firmware (needs cargo-binutils + llvm-tools)
fw-size: fw-build
    cd {{fw}} && cargo size --release

# Firmware as a raw binary for DFU
fw-bin features="": (fw-build features)
    cd {{fw}} && cargo objcopy --release -- -O binary {{fw_bin}}

# Flash over USB DFU. Put the board in DFU first: hold BOOT0, tap NRST, release BOOT0 (the stub has no USB reboot)
fw-flash features="": (fw-bin features)
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

# Lists TODO(#n) markers in Rust sources whose issue is closed (needs gh); fails if any
todo-check:
    #!/usr/bin/env bash
    set -euo pipefail
    open=$(gh issue list --state open --limit 1000 --json number -q '.[].number')
    stale=0
    while IFS= read -r hit; do
        n=$(sed -E 's/.*TODO\(#([0-9]+)\)$/\1/' <<<"$hit")
        if ! grep -qx "$n" <<<"$open"; then echo "closed: $hit"; stale=1; fi
    done < <(grep -rnoE --include='*.rs' --exclude-dir=target 'TODO\(#[0-9]+\)' crates firmware || true)
    exit $stale

# Generate and save coverage report using tarpaulin
cover:
    cargo tarpaulin -o html
    echo "coverage report saved to tarpaulin-report.html"

# Joystick-flown quadcopter in RealFlight, params and telemetry over the native link for `just bridge`, e.g. `just fly-manual --host 192.168.1.20:8080`
fly-manual *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-manual-realflight -- {{ARGS}}

# Joystick-flown eXplora VTOL tailsitter hover in RealFlight, e.g. `just fly-tailsitter --record hover.csv`
fly-tailsitter *ARGS:
    cargo run -p stable-flight-sitl-realflight --example tailsitter-manual-realflight -- {{ARGS}}

# Scripted eXplora VTOL tailsitter hover flight in RealFlight; args: [script.csv] [--host host:port] [--end hold|loop|land]
fly-tailsitter-auto *ARGS:
    cargo run -p stable-flight-sitl-realflight --example tailsitter-auto-realflight -- {{ARGS}}

# Quadcopter in RealFlight flown from the native link, holding when the link goes quiet; drive it with `just link-pilot`
fly-companion *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-companion-realflight -- {{ARGS}}

# Autotune of the rate and angle loops in RealFlight: climbs to the hover point, tunes, prints the gains, lands; e.g. `just fly-autotune --step-test`
fly-autotune *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-autotune-realflight -- {{ARGS}}

# Plays the companion for `just fly-companion`: heartbeats, arm, a scripted waypoint flight; Ctrl-C to go quiet
link-pilot *ARGS:
    cargo run --bin link_pilot -- {{ARGS}}

# Native link ⇄ MAVLink bridge for QGC, e.g. `just bridge --gcs udpout:192.168.1.20:14550`
bridge *ARGS:
    cargo run -p stable-flight-bridge -- {{ARGS}}

# Scripted figure-eight in RealFlight; args: [script.csv] [--host host:port] [--end hold|loop|land]
fly-auto *ARGS:
    cargo run -p stable-flight-sitl-realflight --example copter-auto-realflight -- {{ARGS}}

# Joystick flight (as fly-manual) recording raw RealFlight state to CSV, e.g. `just capture --output roll.csv`
capture *ARGS:
    cargo run -p stable-flight-sitl-realflight --example capture-state-realflight -- {{ARGS}}

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
