# ternary-pid

Ternary PID controller with anti-windup and derivative filtering

## Overview

Ternary PID controller: continuous PID with ternary output {-1, 0, +1}

## Stats

- **Tests**: 9
- **LOC**: 230
- **License**: MIT

## Part of the Oxide Stack

This crate is part of the [Flux→PTX](https://github.com/SuperInstance/cuda-oxide/blob/main/FLUX_TO_PTX.md) experimental suite, testing synergies between the five layers of the distributed GPU runtime:

1. **open-parallel** — async runtime (tokio fork)
2. **pincher** — "Vector DB as runtime, LLM as compiler"
3. **flux-core** — bytecode VM + A2A agent protocol
4. **cuda-oxide** — Flux→MIR→Pliron→NVVM→PTX compiler
5. **cudaclaw** — persistent GPU kernels, warp-level consensus, SmartCRDT

## Usage

```rust
use ternary_pid::*;
// See tests in src/lib.rs for examples
```

## License

MIT
