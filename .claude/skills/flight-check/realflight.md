# RealFlight runs

- **Field:** Eli Field for checks: near sea level, and its tick rate holds. Flight School
  switches camera views, for watching a flight
  ([finding](../../../docs/findings/realflight-tick-rate.md)).
- **Minimize RealFlight** for scripted runs (restore it only to watch): minimized it has
  nothing to render and holds 240–250 Hz. Visible or covered, it flips to the Mac display's
  refresh rate (about 120 Hz on ProMotion) on its own, for either build and any model
  (#447, #438; sitl-realflight README **Timing**). Alternate the builds being compared and
  screen every run before blaming one.
- **Wind:** reset to the setting the script assumes (the wind scripts expect 5 km/h from about
  37°); [finding](../../../docs/findings/realflight-wind.md).
- **Not connected:** "Failed to create bridge: connection timed out" means RealFlight isn't
  running or its link isn't enabled: ask the person at the simulator.
- **Stopping:** Ctrl-C or SIGTERM (`ShutdownSignal`), so `RealFlightVehicle` hands RC back to
  RealFlight. A kill leaves "Preexisting controller reference" on the next run and truncates
  capture CSVs.
- **Disconnects:** flight examples reconnect on their own and log "Reconnected to RealFlight"
  (`capture-state-realflight` doesn't). The runner drops time lost to a stall over 100 ms, so
  heavy skipping (`ticks skipped` in its exit log) slows every `dt`-based clock
  ([tick rate](comparable-runs.md#tick-rate)).
- **Runtime:** a 10-minute background limit stops a loop of several long runs mid-run; run one
  per background command, or a few short (~2 min) runs in one foreground call with a long
  timeout.
