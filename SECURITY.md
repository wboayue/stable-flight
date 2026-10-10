# Security Policy

## Supported versions

Stable Flight is pre-release (0.x). Only `main` gets fixes; there are no maintained release
branches yet.

Stable Flight is experimental and flies in simulation; the ARK FPV firmware is not flight-ready.
It is not certified or flight-proven, and it is not meant to fly a real aircraft near people or
property.

## Reporting a vulnerability

Report privately through GitHub: the repo's **Security** tab → **Report a vulnerability**
([direct link](https://github.com/wboayue/stable-flight/security/advisories/new)). Please don't
open a public issue for a vulnerability.

Include the affected crate and commit, what an attacker can do, and steps to reproduce.
Examples in scope:

- a malformed frame on the native link (`stable-flight-link`) or a MAVLink message to the bridge
  (`stable-flight-bridge`) that panics, hangs, or corrupts the flight controller or bridge;
- a ground station or link peer that can command the vehicle (arm, setpoints, params) without
  going through the bridge's addressing and validation;
- param writes that get past `validate` and reach the flight loop.

Expect an acknowledgment within a week. Fixes land on `main` and, once crates are published, in
a patch release with a GitHub security advisory.
