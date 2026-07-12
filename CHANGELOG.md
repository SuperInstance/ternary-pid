# Changelog

## [Unreleased] - 2026-07-11 — Production hardening pass

### Fixed
- Deadband no longer leaves the derivative term in a stale or
  uninitialized state. Previously, while `|error| < deadband`,
  `update()` returned early without updating `prev_error` or
  `initialized`, so the first non-deadband sample either used a
  stale error (bogus derivative spike) or skipped the derivative
  entirely (`initialized == false`). Both paths are now covered.
- `cargo fmt --check` now passes (CI was red on `master`).

### Changed
- `update()` and `update_raw()` now share a single private
  `compute_pid()` helper. The two public methods no longer carry
  duplicated PID math, eliminating the risk of them drifting apart.

### Tests
- Replaced the tautological `test_cascade_controller` (which asserted
  `output ∈ {-1, 0, +1}`, true by construction for any `i8` return)
  with a real assertion that the cascade output actually responds to
  the sign of the error.
- Strengthened `test_integral_builds` to verify the ternary output
  flips under sustained error, not just that the integral state is
  nonzero.
- Added coverage: deadband→derivative handoff, `reset()` equivalence
  to a fresh controller, feedforward symmetry, anti-windup output
  bound, and `update_raw()` ignoring the deadband.

### Documentation
- README "Architecture Notes" now marks the γ+η=C framing as a
  metaphor rather than a rigorous control-theory identity, and notes
  the fixed-unit-sample-time assumption of the discrete PID math.
