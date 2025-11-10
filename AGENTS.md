# Repository Guidelines

## Project Structure & Module Organization
The core library lives in `src/lib.rs`, with domain-specific modules under `src/commanders`, `src/mixers`, `src/orchestrator`, `src/simulators`, `src/stabilizers`, and `src/telemetry`. Each module encapsulates one stage of the flight control loop so features stay focused. Runtime entrypoints and reference setups are provided in `examples/*.rs`; use these as templates when wiring new hardware or scripted missions. Generated artifacts such as `target/` and `tarpaulin-report.html` remain untracked, while `architecture.md` offers deeper context for system design decisions.

## Build, Test, and Development Commands
Run `cargo build` (or `just build`) to compile everything with the Rust 2021 toolchain. Execute `cargo run --example copter-manual-realflight` to interact with RealFlight in manual mode, and swap the example name for other mission profiles. Use `cargo test` (or `just test`) before every PR to exercise library and example tests. Generate coverage with `just cover`, which produces `tarpaulin-report.html` for review.

## Coding Style & Naming Conventions
Follow standard Rust formatting via `cargo fmt --all`; CI assumes four-space indentation and trailing commas where idiomatic. Keep modules and files snake_case (`src/commanders/extreme_3d.rs`), types in PascalCase (`FlightStabilizer`), and constants in SCREAMING_SNAKE_CASE. Prefer expressive logging with `log` + `env_logger`, and document non-obvious control logic with concise comments.

## Testing Guidelines
Place unit tests alongside the code they verify using `#[cfg(test)]` modules; the crate already uses `approx` and `assert2` for tolerant comparisons. Name tests after the behaviour they protect (e.g., `maintains_pitch_hold`). Run `cargo test --all-targets` locally; integration-style checks can live in `examples/` when full simulator interaction is required. After significant control-loop changes, refresh coverage via `just cover` and inspect `tarpaulin-report.html` for regressions.

## Commit & Pull Request Guidelines
Commits should be small, imperative, and descriptive, mirroring existing history such as `Use f32 instead of Duration... (#18)`. Reference related issues or PRs with `(#id)` when applicable, and include simulator context for flight behaviour adjustments. Pull requests need a summary of the scenario tested, reproduction or validation steps (commands, configurations), and screenshots or telemetry snippets when visual feedback changes. Update `architecture.md` or other docs whenever the control flow or module boundaries shift.
