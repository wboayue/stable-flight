# Findings

Measured facts about the simulators and flight behavior that shape how runs are flown and read.
Each finding is one file: what was measured, where, the result, and what follows from it, with
the issue holding the data. The rules they back live in the
[flight-check skill](../../.claude/skills/flight-check/SKILL.md) and `AGENTS.md`; tuned values
cite their evidence in code. RealFlight's tick rate is measured in the sitl-realflight README's
[**Timing**](../../crates/stable-flight-sitl-realflight/README.md#timing).

A finding states what was measured, not what the code does now: when a simulator update or a
change makes one wrong, re-measure and update it, or delete it.

| Finding | Measured | Issue |
|---|---|---|
| [A low tick rate slows every `dt` clock](low-tick-rate-slows-clocks.md) | 2026-10-15 | #405 |
| [Back-transition handover varies run to run](back-transition-handover-varies.md) | 2026-10-15 | #404 |
| [Heading in the wind decides the eXplora's roll cycle](heading-decides-roll-cycle.md) | 2026-10-21 | #476 |
| [RealFlight wind: direction, speed, sign](realflight-wind.md) | 2026-10-20–21 | #473, #476 |
| [The eXplora's elevon trim falls with airspeed as V^-0.74](explora-trim-vs-airspeed.md) | 2026-11-01 | #648 |
| [The eXplora's hover descent swings on a soft pitch tune, not from a vortex ring](explora-descent-swing.md) | 2026-11-03 | #647 |
| [The eXplora's autotuned hover descends at 2 m/s as cleanly as at 1.5; at 2.5 its arrival lags](explora-descent-limit.md) | 2026-11-03 | #653 |

## Format

```markdown
# <the fact, as a sentence or short title>

**Measured:** <date>, <simulator, model, field, script>. **Data:** #n.

<What was flown and what came out, with numbers.>

**So:** <its consequence in a phrase>, linking the rule it backs (skill or AGENTS.md section).

The rule itself lives where it is applied; a finding doesn't restate it.
```
