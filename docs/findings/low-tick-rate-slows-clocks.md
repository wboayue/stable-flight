# A low tick rate slows every `dt`-based clock

**Measured:** 2026-10-15, RealFlight, eXplora waypoint descent. **Data:** #405.

Each tick advances every `dt`-based clock (descent targets, ramps, timeouts) by the nominal
4 ms, whatever the wall-clock time between ticks. At 170 Hz a 3 m/s descent target moved at
2 m/s, and the eXplora's descent looked like a guidance bug until the trace was replayed.

**So:** screen every run's tick rate
([flight-check](../../.claude/skills/flight-check/comparable-runs.md#tick-rate)).
