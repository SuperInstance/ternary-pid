## Migrating from Binary

If you're used to binary PID (on/off hysteresis), ternary PID adds a **deadband** — the $0$ state where no action is needed.

| Binary | Ternary |
|--------|---------|
| Heat on ($1$) | Heat ($+1$) |
| Heat off ($0$) | Idle ($0$) |
| | Cool ($-1$) |

The $0$ state prevents oscillation around setpoint. In binary PID, the system constantly toggles between on and off near target. Ternary PID's neutral zone absorbs small deviations without actuator wear.

See **[From Binary to Ternary](https://github.com/SuperInstance/ternary-cookbook/blob/master/guides/FROM_BINARY.md)** for the full migration guide.
