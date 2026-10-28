# Link setups

For flights through the companion link (QGC, `link_pilot`, offboard MAVLink).

- `just fly-companion` + `just bridge` + QGC, or `just fly-companion` + `just link-pilot`
  (instead of the bridge). Offboard MAVLink scripts use the bridge's UDP 14550 (close QGC
  first) and run with `uv run --no-project --with pymavlink python script.py`.
- **Start order:** bridge and example first, then QGC. If the vehicle disappears and returns
  under an open QGC, its virtual joystick can go silent while Arm still works: restart QGC.
- **Signals:** the companion example logs each failsafe handover (failsafe timeout 1 s). QGC's
  throttle and `SAT_*` displays: the bridge README's `VFR_HUD` and `NAMED_VALUE_INT` rows.
  QGC → bridge traffic: `nettop -p <bridge pid> -J bytes_in` (heartbeat only ≈ 21 B/s, joystick
  ≈ 600 B/s).
