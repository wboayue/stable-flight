# RealFlight wind: direction, speed, and sign

**Measured:** 2026-10-20 and 2026-10-21, RealFlight, eXplora, `tailsitter-auto-realflight
--record` (`true_wind_*` columns), `flights/explora_heading_box.csv` and the wind scripts.
**Data:** #473, #476.

- RealFlight's `wind_mps` is the air's velocity (where it blows toward), not where it comes
  from: a fit gives R² 0.98 against 0.82 flipped.
- The 5 km/h setting blows from about 37°, 2.1–2.3 m/s at 40 m, about 1.7 m/s lower down.
  3 mph behaves about the same.
- eXplora hover headings (the belly's direction): belly into the wind (about 38°) pins the
  27° tilt cap and drifts; a wingtip into the wind (about 125° or 305°) holds; ±45° off a
  wingtip is the edge at 40 m, narrower at 60 m. Belly-on climbs can drift past the 3 m
  acceptance before reaching altitude and stall a script's first hold.
- Descending with the wind behind the wing cycles roll
  ([heading-decides-roll-cycle.md](heading-decides-roll-cycle.md)).

**So:** wind flights script climbs and holds at wingtip headings
([flight-check](../../.claude/skills/flight-check/SKILL.md#rules)).
