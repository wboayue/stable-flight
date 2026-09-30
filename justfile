# Builds every workspace crate
build:
    cargo build --workspace

# Runs tests
test:
    cargo test --workspace --all-features

# Builds the no_std core for a bare-metal target, with and without `testing` (as CI does)
embedded:
    rustup target add thumbv7em-none-eabihf
    cargo build -p stable-flight --target thumbv7em-none-eabihf
    cargo build -p stable-flight --features testing --target thumbv7em-none-eabihf

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
