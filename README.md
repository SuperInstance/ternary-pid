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

## Architecture Notes

The ternary PID embodies the **γ + η = C** identity in control theory. The +1 command (constructive, γ) drives the system toward the setpoint, the -1 command (inhibitory, η) drives it away from overshoot, and the 0 command (neutral) conserves actuator energy. The conserved quantity $C$ is the actuator duty cycle — bounded by physical constraints (thermal limits, fuel, battery life).

Anti-windup ensures that the *budget* of corrective action doesn't accumulate beyond $C$. The deadband prevents the controller from spending $\gamma$ and $\eta$ on noise that averages to zero — a direct application of conservation: only spend control authority when the error exceeds the noise floor.

The cascade architecture splits $C$ into two sub-budgets: the outer loop's $C_{\text{outer}}$ determines the setpoint trajectory, and the inner loop's $C_{\text{inner}}$ tracks it. The total $C = C_{\text{outer}} + C_{\text{inner}}$ is conserved.

## References

- Åström, K. J. & Hägglund, T. (2006). *Advanced PID Control.* ISA Press.
- Ogata, K. (2010). *Modern Control Engineering.* 5th ed. Pearson.
- Levine, W. S. (Ed.) (2018). *The Control Handbook.* 3rd ed. CRC Press.
- Visioli, A. (2006). *Practical PID Control.* Springer.

## License

MIT
