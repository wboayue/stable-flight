# RealFlight runs

- **Field:** Eli Field for checks: near sea level, and its tick rate holds (an eXplora lap held
  245 Hz for 101 of 137 s there, 48 of 153 s at Flight School). Flight School switches camera
  views, for watching a flight.
- **Minimize RealFlight** for scripted runs (restore it only to watch), alternate the builds
  being compared, and screen every run before blaming one: visible, it locks to the display's
  refresh rate on its own (sitl-realflight README
  [**Timing**](../../../crates/stable-flight-sitl-realflight/README.md#timing)).
- **Wind:** reset to the setting the script assumes (the wind scripts expect 5 km/h from about
  37°; [finding](../../../docs/findings/realflight-wind.md)).
- **Not connected:** "Failed to create bridge: connection timed out" means RealFlight isn't
  running or its link isn't enabled: ask the person at the simulator.
- **Stopping:** a kill instead of SIGTERM leaves RC with the example, so the next run fails on
  "Preexisting controller reference", and truncates capture CSVs.
- **Disconnects:** flight examples reconnect on their own and log "Reconnected to RealFlight"
  (`capture-state-realflight` doesn't). Heavy `ticks skipped` in the exit log makes a run
  incomparable ([tick rate](comparable-runs.md#tick-rate)).
- **Runtime:** a 10-minute background limit stops a loop of several long runs mid-run; run one
  per background command, or a few short (~2 min) runs in one foreground call with a long
  timeout.
