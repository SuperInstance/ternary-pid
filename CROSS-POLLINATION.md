# CROSS-POLLINATION.md — ternary-pid

> **Conservation Law Connection:** PID controller drives γ/η toward equilibrium

## Role in the Conservation Law

`ternary-pid` provides the **feedback control** that keeps fleet operations near
the conservation law optimum. The controller output is ternary {-1, 0, +1}:

- **+1:** Increase γ allocation (push harder, more agents, more compute)
- **0:** Hold steady (system at equilibrium, γ/C ≈ 1 − δ(n))
- **−1:** Decrease γ allocation (pull back, reduce η-producing activity)

The PID terms map to conservation law diagnostics:
- **P (Proportional):** Current drift |γ_actual − γ_predicted| — how far from optimal
- **I (Integral):** Accumulated drift over time — systematic bias detection
- **D (Derivative):** Rate of change of drift — early warning for divergence

## delta-clt Verification Results

The delta-clt correlated fleet simulation shows drift increases with shared bias.
A PID controller with anti-windup would:

1. Detect rising η (I-term accumulation) from correlation bias
2. Apply −1 correction (reduce fleet coupling)
3. Drive η back toward δ(n) theoretical floor

The adversarial fleet test (10% adversarial agents) showed the law holds within
bounds. PID control with bang-bang ternary output can detect and counter adversarial
influence by switching to −1 when η exceeds the adversarial threshold.

## Cross-Repo Connections

### → ternary-hamiltonian
`ternary-hamiltonian` defines the system dynamics; `ternary-pid` provides the controller.
Together they form a complete control-theoretic framework: Hamiltonian = plant model,
PID = controller, conservation law = stability criterion.

**Shared:** Both operate on ternary state spaces. Both assume conservation (C constant).
**Different:** Hamiltonian describes what happens; PID makes specific things happen.

### → ternary-rhythm
`ternary-rhythm` identifies oscillation modes; `ternary-pid` damps them. If rhythm
detects a dangerous oscillation (η spiking periodically), PID parameters can be
tuned to add damping at that frequency.

**Shared:** Both analyze temporal behavior of ternary signals.
**Different:** Rhythm is diagnostic (observes patterns); PID is therapeutic (corrects them).

### → ternary-fleet
Fleet sub-crates use PID controllers for rate limiting, load balancing, and
resource allocation. The ternary output maps naturally to scale-up/hold/scale-down.

**Shared:** PID is a fleet-level control primitive.
**Different:** `fleet` is the system being controlled; `ternary-pid` is the controller.

## Fleet Position

```
┌────────────────────────────────────────────────┐
│  ternary-pid — THE CONTROL LAYER                │
│                                                 │
│  Measurement: γ_actual, η_actual                │
│  Setpoint:    γ_target = (1 − δ(n)) × C         │
│  Error:       e = γ_target − γ_actual            │
│  Output:      Trit {−1, 0, +1}                   │
│                                                 │
│  P = e           (current drift)                │
│  I = Σ e dt      (accumulated bias)             │
│  D = de/dt       (drift velocity)               │
│                                                 │
│  Anti-windup: prevent I-term from masking       │
│  correlation-driven η (cf. delta-clt Section 3) │
│                                                 │
│  Plant model: ternary-hamiltonian               │
│  Oscillation detector: ternary-rhythm           │
│  Application: ternary-fleet rate control        │
└────────────────────────────────────────────────┘
```

