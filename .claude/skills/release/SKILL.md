---
name: release
description: Release stable-flight's published crates (changelogs, release PR, tags, GitHub releases, crates.io). Use only when the maintainer asks for a release.
disable-model-invocation: true
---

# Release

Changelog and new-crate rules are in AGENTS.md (**Commits, PRs, and releases**); this is the
release itself.

1. `cargo publish --workspace --dry-run --locked`.
2. A `Release x.y.z` PR: each changelog's `[Unreleased]` becomes `## [x.y.z] - YYYY-MM-DD`
   under a fresh `[Unreleased]`, with links (`compare/<crate>-vx.y.z...HEAD`,
   `releases/tag/<crate>-vx.y.z`).
3. After it merges, annotated `<crate>-vx.y.z` tags on the merge commit, pushed
   (`just tag <crate> <version>`; `just versions` lists them).
4. One GitHub release per tag, notes from the changelog section; `stable-flight` is latest
   (`--latest=false` for the rest).
5. `cargo publish --workspace --locked` (dependency order, waits for indexing). On a 429 for
   a new crate name, publish the rest with `-p <crate>` after the given wait.
