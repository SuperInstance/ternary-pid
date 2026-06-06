# ternary-pid

A PID controller that speaks ternary: continuous computation, three-state output.

## Why This Exists

PID controllers are ubiquitous in control systems — temperature regulation, motor control, process automation. But most actuators aren't continuous. Heating elements turn on or off. Valves open or close. Motors run forward, stop, or reverse. The standard approach is to compute a continuous PID output, then quantize it with a threshold — but that quantization is an afterthought, not a first-class design decision.

This crate makes ternary output the primary interface. The PID computes a continuous correction internally (proportional + integral + derivative), then quantizes to `{−1, 0, +1}`. The deadband parameter controls when the output is zero — errors within the deadband produce no action, preventing chattering around the setpoint.

The crate also provides **cascade control** (two PID loops in series — outer loop produces setpoint for inner loop) and **feedforward control** (compensate for known disturbances before they hit the feedback loop). Both produce ternary output.

## Architecture

```
Setpoint + Measurement
         │
         ▼
    ┌─────────────────────────────┐
    │       TernaryPid            │
    │                             │
    │  error = setpoint - meas    │
    │  P = kp × error             │
    │  I = ki × Σ(error) [clamped]│
    │  D = kd × d(error)/dt       │
    │       [low-pass filtered]   │
    │                             │
    │  output = P + I + D         │
    │  ternary = sign(output)     │
    │    (±deadband → 0)          │
    └─────────────────────────────┘
         │
         ▼
      i8 {-1, 0, +1}

CascadePid: outer loop → inner_setpoint → inner loop → ternary output
FeedforwardPid: disturbance → ff_correction + setpoint → PID → ternary output
```

**Key types:**

- **`TernaryPid`** — full PID with anti-windup (integral clamping), derivative low-pass filtering, and configurable deadband. `update()` returns `i8`. `update_raw()` returns the continuous `f64` before quantization.
- **`CascadePid`** — two PIDs in series. Outer loop tracks a slow variable and produces a setpoint for the inner loop, which tracks a fast variable.
- **`FeedforwardPid`** — PID + known disturbance model. Adjusts the setpoint before the feedback loop sees it, so the system reacts to predictable disturbances immediately rather than waiting for feedback.

## Usage

```rust
use ternary_pid::{TernaryPid, CascadePid, FeedforwardPid};

// Basic proportional control
let mut pid = TernaryPid::new(1.0, 0.0, 0.0);
assert_eq!(pid.update(10.0, 5.0), 1);   // error = +5 → heat/forward/increase
assert_eq!(pid.update(5.0, 10.0), -1);  // error = -5 → cool/reverse/decrease

// With deadband: errors within ±1.0 produce no output
let mut pid = TernaryPid::new(1.0, 0.0, 0.0);
pid.deadband = 1.0;
assert_eq!(pid.update(10.0, 9.5), 0);   // within deadband → idle
assert_eq!(pid.update(10.0, 8.0), 1);   // outside deadband → actuate

// Integral builds up over time
let mut pid = TernaryPid::new(0.0, 0.1, 0.0);
for _ in 0..100 {
    pid.update(10.0, 9.0); // persistent 1.0 error
}
assert!(pid.integral > 0.0); // integral accumulated

// Anti-windup prevents integral from exceeding limit
let mut pid = TernaryPid::new(0.0, 1.0, 0.0);
pid.integral_limit = 10.0;
for _ in 0..1000 {
    pid.update(100.0, 0.0); // massive persistent error
}
assert!(pid.integral <= 10.0); // clamped

// Derivative resists sudden changes
let mut pid = TernaryPid::new(0.0, 0.0, 10.0);
pid.update(10.0, 10.0); // first call, no derivative
let out = pid.update(10.0, 5.0); // sudden drop → derivative kicks in
assert_eq!(out, 1);

// Reset controller state
pid.reset();

// Cascade control: temperature (outer) → heater power (inner)
let outer = TernaryPid::new(1.0, 0.1, 0.0);  // slow: temperature tracking
let inner = TernaryPid::new(2.0, 0.0, 0.5);  // fast: heater power
let mut cascade = CascadePid::new(outer, inner);
let action = cascade.update(100.0, 50.0, 45.0); // setpoint=100, temp=50, heater=45

// Feedforward control: compensate for known disturbance
let pid = TernaryPid::new(1.0, 0.0, 0.0);
let mut ff = FeedforwardPid::new(pid, 1.0, 0.0); // ff_gain=1.0
let action = ff.update(10.0, 10.0, 5.0);
// disturbance=5.0, ff adjusts setpoint to 15.0, error=5 → +1
```

## API Reference

### `TernaryPid`

| Method | Description |
|--------|-------------|
| `TernaryPid::new(kp, ki, kd)` | Create PID with given gains |
| `.update(setpoint, measurement)` | Compute PID, return ternary `{-1, 0, +1}` |
| `.update_raw(setpoint, measurement)` | Compute PID, return continuous `f64` |
| `.reset()` | Zero integral, derivative, and initialization flag |

Configurable fields: `deadband: f64` (default 0.0), `integral_limit: f64` (default 100.0), `derivative_filter: f64` (default 0.1)

### `CascadePid`

| Method | Description |
|--------|-------------|
| `CascadePid::new(outer, inner)` | Create cascade from two `TernaryPid` instances |
| `.update(outer_setpoint, outer_measurement, inner_measurement)` | Outer loop computes inner setpoint, inner loop produces ternary output |

### `FeedforwardPid`

| Method | Description |
|--------|-------------|
| `FeedforwardPid::new(pid, ff_gain, disturbance_bias)` | Create with PID, feedforward gain, and linear bias |
| `.update(setpoint, measurement, disturbance)` | Apply feedforward correction, then PID |

Fields: `pid: TernaryPid`, `ff_gain: f64`, `disturbance_bias: f64`

## The Deeper Idea

The ternary PID is a specific case of **quantized control** — continuous control law, discrete output. This is the reality for most physical systems: valves are on/off, heaters are on/off, motors have discrete direction states. The classical approach of computing continuous output then thresholding works, but the deadband and quantization are usually bolted on.

Making ternary the primary interface forces you to think about the deadband explicitly. In a continuous PID, a small error produces a small output — which is fine for analog actuators but meaningless for a relay. The deadband says: "errors this small aren't worth the wear on the hardware." This is an engineering judgment, not a mathematical optimization, and the ternary PID makes it a first-class parameter.

**Anti-windup** is critical for ternary systems. When the output is saturated (stuck at +1 or -1), the integral term keeps accumulating. Without clamping, when the error finally reverses, the integral takes a long time to discharge — causing overshoot. The `integral_limit` parameter bounds this, and the deadband slowly bleeds the integral when the system is near setpoint.

**Derivative filtering** prevents noise from causing spurious ternary transitions. Raw derivative is `Δerror/Δt`, which amplifies high-frequency noise. The low-pass filter smooths this: `filtered = α × raw + (1-α) × previous`. This is standard practice in industrial PID controllers, but often omitted in software implementations.

**Cascade control** handles systems with multiple timescales. Temperature control is the classic example: the outer loop tracks room temperature (slow), the inner loop controls heater power (fast). The outer loop's output becomes the inner loop's setpoint, ensuring the heater responds to temperature deviations without oscillating.

## Related Crates

- **`ternary-thermostat`** — climate control built on top of this PID controller
- **`ternary-scheduler`** — task scheduling with ternary priority, analogous to control
- **`ternary-route`** — ternary routing with health-aware load balancing
