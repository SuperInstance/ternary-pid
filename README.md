# ternary-pid

Ternary PID controller with anti-windup, derivative filtering, and bang-bang ternary output {-1, 0, +1}. Includes cascade (multi-loop) and feedforward architectures for industrial control of ternary actuated systems.

## Why It Matters

Classic PID controllers produce continuous control signals. But many real actuators are ternary: they can push forward (+1), pull backward (-1), or do nothing (0) — think of thrusters, relays, or three-way valves. The ternary PID bridges continuous control theory and ternary actuation:

- **Preserves PID intuition**: tune $K_p$, $K_i$, $K_d$ as usual
- **Anti-windup**: integral term clamped to prevent saturation
- **Derivative filtering**: low-pass filter suppresses noise amplification
- **Deadband**: errors within a band produce zero output (prevents chatter)
- **Cascade architecture**: outer loop generates setpoints for inner loop
- **Feedforward**: compensate for known disturbances before feedback

## How It Works

### PID Control Law

The continuous-time PID equation:

$$u(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$

where $e(t) = \text{setpoint} - \text{measurement}$ is the error signal.

### Ternary Quantization

The PID output is quantized to a ternary command:

$$u_{\text{ternary}} = \begin{cases} +1 & \text{if } u > 0 \\ 0 & \text{if } u = 0 \\ -1 & \text{if } u < 0 \end{cases}$$

This is a sign function — equivalent to bang-bang control with a dead zone.

### Anti-Windup

The integral term is clamped to prevent windup during sustained errors:

$$I_{\text{acc}}(t) = \text{clamp}\!\left(I_{\text{acc}}(t-1) + e(t),\;-I_{\lim},\;+I_{\lim}\right)$$

### Derivative Filtering

Raw derivative is noisy. A first-order low-pass filter smooths it:

$$\dot{e}_{\text{filt}}(t) = \alpha \cdot (e(t) - e(t-1)) + (1 - \alpha) \cdot \dot{e}_{\text{filt}}(t-1)$$

where $\alpha \in [0, 1]$ is the filter coefficient (smaller = smoother).

### Deadband

When $|e(t)| < e_{\text{dead}}$, output is forced to 0 and the integral slowly bleeds:

$$I_{\text{acc}} \leftarrow 0.95 \cdot I_{\text{acc}}$$

This prevents limit-cycle oscillation around the setpoint.

**Complexity:** O(1) per update — fixed work regardless of history length.

### Cascade Control

Outer loop produces a setpoint for the inner loop:

$$\text{SP}_{\text{inner}} = u_{\text{outer, raw}} + \text{SP}_{\text{outer}}$$

The outer loop uses raw (unquantized) output to provide a smooth reference, while the inner loop makes the ternary decision.

### Feedforward

Compensate for known disturbances before feedback:

$$\text{SP}_{\text{adjusted}} = \text{SP} + K_{ff} \cdot d + b_{\text{dist}}$$

where $d$ is the measured disturbance, $K_{ff}$ is the feedforward gain, and $b_{\text{dist}}$ is a bias term.

## Quick Start

```rust
use ternary_pid::*;

// Basic ternary PID
let mut pid = TernaryPid::new(1.0, 0.1, 0.5);
let output = pid.update(100.0, 50.0); // setpoint=100, meas=50
assert_eq!(output, 1); // positive error → push forward

// With deadband
let mut pid = TernaryPid::new(1.0, 0.0, 0.0);
pid.deadband = 1.0;
assert_eq!(pid.update(10.0, 9.5), 0); // error=0.5 within deadband
assert_eq!(pid.update(10.0, 8.0), 1); // error=2.0 outside

// Anti-windup
let mut pid = TernaryPid::new(0.0, 1.0, 0.0);
pid.integral_limit = 10.0;
for _ in 0..1000 { pid.update(100.0, 0.0); }
assert!(pid.integral <= 10.0); // integral never exceeds limit

// Cascade controller
let outer = TernaryPid::new(1.0, 0.1, 0.0);
let inner = TernaryPid::new(2.0, 0.0, 0.5);
let mut cascade = CascadePid::new(outer, inner);
let cmd = cascade.update(100.0, 50.0, 45.0);

// Feedforward
let pid = TernaryPid::new(1.0, 0.0, 0.0);
let mut ff = FeedforwardPid::new(pid, 1.0, 0.0);
let cmd = ff.update(10.0, 10.0, 5.0); // disturbance=5.0
assert_eq!(cmd, 1); // feedforward adjusts setpoint up
```

## API

| Type | Description |
|---|---|
| `TernaryPid::new(kp, ki, kd)` | PID controller with ternary output |
| `.update(setpoint, meas) → i8` | Compute ternary command |
| `.update_raw(setpoint, meas) → f64` | Raw (float) PID output |
| `.reset()` | Clear integral, derivative, history |
| `.deadband`, `.integral_limit`, `.derivative_filter` | Tunable parameters |
| `CascadePid::new(outer, inner)` | Two-loop cascade controller |
| `.update(sp, outer_meas, inner_meas) → i8` | Cascade update |
| `FeedforwardPid::new(pid, ff_gain, bias)` | Feedforward + feedback |
| `.update(sp, meas, disturbance) → i8` | Compensated update |

### Discrete-time assumption

The controller is implemented in discrete time with an implicit fixed sample
period of Δt = 1. The integral accumulates raw error samples
($\sum e$, no Δt factor) and the derivative uses raw sample-to-sample
differences ($e(t) - e(t-1)$, no division by Δt). As a result the gains
carry implicit units: $K_p$ is dimensionless, $K_i$ is per-sample, and $K_d$
is in samples. Callers running the loop at a different sample rate must scale
$K_i$ and $K_d$ accordingly. The textbook continuous-time gains recovered at
any particular sample rate are $K_i^{\text{ct}} = K_i / \Delta t$ and
$K_d^{\text{ct}} = K_d \cdot \Delta t$.

## Architecture Notes

> **Status:** This section is **interpretive framing, not a formal
> control-theory result.** The discrete PID math itself (P/I/D terms,
> anti-windup clamp, derivative low-pass, ternary quantization) is
> rigorously implemented and verified by `test_discrete_pid_math_matches_textbook_formula`.
> The γ+η=C mapping below is a metaphor borrowed from a sibling
> conservation-law project; no quantity is mathematically conserved by
> the algorithm in this crate. Treat it as a design mnemonic.

The ternary PID can be read through the **γ + η = C** identity as a
*metaphor* for actuator authority: the +1 command (constructive, γ) drives
the system toward the setpoint, the -1 command (inhibitory, η) drives it
away from overshoot, and the 0 command (neutral) spends no actuator budget.
The "conserved quantity" $C$ is a *physical* actuator budget — thermal
limits, fuel, battery life — enforced by the hardware, not by this code.

Anti-windup is best understood through standard control theory (preventing
integrator saturation during sustained error) rather than as a
budget-conservation law. The deadband similarly has a standard
interpretation: only spend control authority when the error exceeds the
noise floor.

The cascade architecture is a standard two-loop structure; the outer loop's
raw (unquantized) output becomes the inner loop's setpoint. There is no
formal sense in which $C = C_{\text{outer}} + C_{\text{inner}}$ is conserved
by the algorithm — each loop spends its own actuator authority independently.

## References

- Åström, K. J. & Hägglund, T. (2006). *Advanced PID Control.* ISA Press.
- Ogata, K. (2010). *Modern Control Engineering.* 5th ed. Pearson.
- Levine, W. S. (Ed.) (2018). *The Control Handbook.* 3rd ed. CRC Press.
- Visioli, A. (2006). *Practical PID Control.* Springer.

## License

MIT
