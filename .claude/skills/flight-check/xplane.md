# X-Plane runs

- **Reset before every run:** a `CMND` of `sim/operation/reset_flight` to UDP 49000 puts the
  aircraft back at its flight start, where scripts are anchored. A run left on its last
  landing has drained batteries and leftover state. Wait about 10 s after the reset:

  ```bash
  python3 -I -c "import socket; socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b'CMND\x00sim/operation/reset_flight\x00', ('127.0.0.1', 49000))"
  ```

- **Where is it:** `capture-state-xplane` reads position and attitude without taking control.
- **Fly plane legs over water:** `flights/alia_*.csv` start at Oceano (L52), runway 11, so
  plane legs fly offshore.
- **Off-altitude loiter:** a plane leg that passes its waypoint off altitude loiters on it
  until at its altitude, which can add minutes to a lap (the Alia's 136 m short climb waited
  193 s, #451): give a watchdog's timeout that margin. A sinking one falls back to a hover
  below the Alia's `TRAN_FW_MIN_HGT` (20 m above arming).
- **Recorder altitude is MSL** (spawn reads about 4.6 m at L52): offset altitude floors, and
  detect landing by `armed == 0` after a back transition.
- **Copter legs turn the nose along the leg** as they fly it (`GUID_YAW_SLEW`), so start a
  route with a climb over the spawn, which turns nothing on the ground.
- **Manual runs:** unbind the Extreme 3D's hat in X-Plane's joystick settings (bound to view
  panning, a pan-back Left asks for copter, #446). `joystick_events --raw` dumps what gilrs
  sees.
