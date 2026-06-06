# PLUG_AND_PLAY — Pid

> Ternary PID controller with anti-windup

## 🚀 Quick Start

Add to your `Cargo.toml`:

```toml
[dependencies]
ternary-pid = { git = "https://github.com/SuperInstance/ternary-pid" }
```

Use in your code:

```rust
use ternary_pid::TernaryPid;

let mut pid = TernaryPid::new(1.0, 0.1, 0.05);
let output = pid.update(0.0, 1.0, 0.01);
```

## 📚 Available Documentation

| Document | Description |
|----------|-------------|
| `docs/FROM_BINARY.md` | Understanding ternary concepts as a binary programmer |
| `docs/MIGRATION.md` | Version migration guide |
| `docs/FUTURE-INTEGRATION.md` | Planned features and roadmap |

## 🔗 Integration

This crate is part of the [SuperInstance ternary fleet](https://github.com/SuperInstance). It uses the canonical `Ternary` type from `ternary-types` for cross-crate compatibility.

## 📄 License

MIT
