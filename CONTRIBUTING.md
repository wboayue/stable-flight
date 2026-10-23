# Contributing

Thanks for your interest in Stable Flight. Bug reports, questions, and pull requests are welcome.

## Discussions and issues

- **Questions, ideas, and flights to show:** start a thread in
  [Discussions](https://github.com/wboayue/stable-flight/discussions) (Q&A, Ideas, Show and
  tell). Issues are for actionable work; a discussion that turns into one gets converted.
- **Bugs:** what you ran (command, simulator, airframe), what you expected, what happened, and
  logs or telemetry if you have them.
- **Features and larger changes:** open an issue first to agree on the approach before writing
  code. Outstanding work is tracked in [issues](https://github.com/wboayue/stable-flight/issues),
  grouped by milestone.
- **Security problems:** don't open an issue; see [SECURITY.md](SECURITY.md).

Participation is covered by the [Code of Conduct](CODE_OF_CONDUCT.md).

## Pull requests

[AGENTS.md](AGENTS.md) holds the rules for humans and AI assistants alike: commands, coding and
testing style, docs and changelog upkeep, and the pre-PR checklist. In short:

1. Branch from `main` and keep the PR to one change.
2. Add tests next to the code; a test that guards a fix must fail without it.
3. Run the checks CI runs:
   ```bash
   cargo fmt --all
   just lint                              # clippy + rustdoc, -D warnings
   cargo test --workspace --all-features
   just embedded                          # no_std core and link, firmware build
   ```
4. Update the docs that own what you changed, and the crate's `CHANGELOG.md`
   ([rules](AGENTS.md#commits-prs-and-releases)).
5. Describe what you changed and how you checked it. Changes to flight behavior, the link, or
   the bridge need a RealFlight check ([Flight checks](AGENTS.md#simulators-and-flight-checks)): say what you
   flew, or that you couldn't and the maintainer will fly it.

The `test`, `lint`, and `embedded` checks must pass before a PR can merge. Most PRs are
squash-merged, so the PR title becomes the commit subject: make it a short imperative summary.

Architecture and design rationale live in [architecture.md](architecture.md).

## License

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
