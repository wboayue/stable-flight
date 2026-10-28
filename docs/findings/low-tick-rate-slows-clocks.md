# A low tick rate slows every `dt`-based clock

**Measured:** 2026-10-15, RealFlight, eXplora waypoint descent. **Data:** #405.

Each tick advances every `dt`-based clock (descent targets, ramps, timeouts) by the nominal
4 ms, whatever the wall-clock time between ticks. At 170 Hz a 3 m/s descent target moved at
2 m/s, and the eXplora's descent looked like a guidance bug until the trace was replayed. The
runner also drops time lost to a stall over 100 ms, so heavy `ticks skipped` has the same
effect.

**So:** drop a run below about 245 Hz around the event it measures; count `wall_s` rows per
wall-clock second there, and `grep "host slow"` its log.
