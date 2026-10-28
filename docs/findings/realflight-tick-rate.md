# RealFlight's tick rate follows its window and field

**Measured:** 2026-10-15 to 2026-10-28, RealFlight in a Parallels VM on a ProMotion Mac,
QuadCopterX, Titan Cobra, and eXplora. **Data:** #438, #447, #512.

- RealFlight flips between 250 Hz and the Mac display's refresh rate (about 120 Hz on
  ProMotion) on its own, for debug and release builds and any model, and stays at the
  refresh rate while its window is covered. Lowering the resolution (800×600) made it worse.
- Minimized, it has nothing to render and holds 240–250 Hz. An autotune design search then
  finishes in 5–7 min.
- The field matters: an eXplora transition lap held 245 Hz for 101 of 137 s at Eli Field,
  against 48 of 153 s at Flight School.

**So:** minimize RealFlight for scripted runs and fly checks at Eli Field. Screen every run's
tick rate before comparing it (see [low-tick-rate-slows-clocks.md](low-tick-rate-slows-clocks.md)).
