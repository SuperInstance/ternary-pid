//! Ternary PID controller: continuous PID with ternary output {-1, 0, +1}

/// Ternary PID controller with anti-windup and derivative filtering
#[derive(Clone, Debug)]
pub struct TernaryPid {
    /// Proportional gain
    pub kp: f64,
    /// Integral gain
    pub ki: f64,
    /// Derivative gain
    pub kd: f64,
    /// Deadband: error within this range produces 0 output
    pub deadband: f64,
    /// Integral windup limit
    pub integral_limit: f64,
    /// Derivative low-pass filter coefficient (0-1)
    pub derivative_filter: f64,
    /// Internal state
    integral: f64,
    prev_error: f64,
    filtered_derivative: f64,
    initialized: bool,
}

impl TernaryPid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            deadband: 0.0,
            integral_limit: 100.0,
            derivative_filter: 0.1,
            integral: 0.0,
            prev_error: 0.0,
            filtered_derivative: 0.0,
            initialized: false,
        }
    }

    /// Compute ternary output for given setpoint and measurement.
    ///
    /// The discrete PID assumes a fixed unit sample time (Δt = 1). Gains
    /// therefore carry implicit units: `ki` is per-sample and `kd` is in
    /// samples. Callers sampling at a different rate must scale `ki` and
    /// `kd` accordingly.
    pub fn update(&mut self, setpoint: f64, measurement: f64) -> i8 {
        let error = setpoint - measurement;

        // Deadband check
        if error.abs() < self.deadband {
            self.integral *= 0.95; // Slowly bleed integral in deadband
                                   // Keep derivative state consistent so the next non-deadband
                                   // sample does not compute a derivative against a stale error
                                   // (huge bogus spike) or skip the derivative entirely
                                   // (treated as uninitialized). See test_deadband_does_not_break_derivative.
            self.prev_error = error;
            self.initialized = true;
            return 0;
        }

        let output = self.compute_pid(error);
        // Ternary decision
        if output > 0.0 {
            1
        } else if output < 0.0 {
            -1
        } else {
            0
        }
    }

    /// Get raw PID output before ternary quantization.
    ///
    /// Note: `update_raw` deliberately does NOT apply the deadband or the
    /// in-deadband integral bleed — it is intended for cascade outer loops
    /// and other callers that need the continuous (unquantized) signal.
    /// Anti-windup clamping of the integral still applies. The ternary
    /// `update()` is the only entry point that quantizes.
    pub fn update_raw(&mut self, setpoint: f64, measurement: f64) -> f64 {
        let error = setpoint - measurement;
        self.compute_pid(error)
    }

    /// Shared discrete PID core. Does not apply the deadband; callers
    /// decide whether to short-circuit on `|error| < deadband`.
    fn compute_pid(&mut self, error: f64) -> f64 {
        // Proportional
        let p = self.kp * error;

        // Integral with anti-windup
        self.integral += error;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        // Derivative with first-order low-pass filtering
        let derivative = if self.initialized {
            let raw_d = error - self.prev_error;
            self.filtered_derivative = self.derivative_filter * raw_d
                + (1.0 - self.derivative_filter) * self.filtered_derivative;
            self.filtered_derivative
        } else {
            0.0
        };
        let d = self.kd * derivative;

        self.prev_error = error;
        self.initialized = true;

        p + i + d
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.filtered_derivative = 0.0;
        self.initialized = false;
    }
}

/// Multi-loop cascade controller
pub struct CascadePid {
    pub outer: TernaryPid,
    pub inner: TernaryPid,
}

impl CascadePid {
    pub fn new(outer: TernaryPid, inner: TernaryPid) -> Self {
        Self { outer, inner }
    }

    /// Outer loop produces setpoint for inner loop
    pub fn update(
        &mut self,
        outer_setpoint: f64,
        outer_measurement: f64,
        inner_measurement: f64,
    ) -> i8 {
        let inner_setpoint =
            self.outer.update_raw(outer_setpoint, outer_measurement) + outer_setpoint;
        self.inner.update(inner_setpoint, inner_measurement)
    }
}

/// Feedforward + feedback combined controller
pub struct FeedforwardPid {
    pub pid: TernaryPid,
    /// Feedforward gain
    pub ff_gain: f64,
    /// Known disturbance model (linear)
    pub disturbance_bias: f64,
}

impl FeedforwardPid {
    pub fn new(pid: TernaryPid, ff_gain: f64, disturbance_bias: f64) -> Self {
        Self {
            pid,
            ff_gain,
            disturbance_bias,
        }
    }

    pub fn update(&mut self, setpoint: f64, measurement: f64, disturbance: f64) -> i8 {
        let ff_correction = self.ff_gain * disturbance + self.disturbance_bias;
        let adjusted_setpoint = setpoint + ff_correction;
        self.pid.update(adjusted_setpoint, measurement)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_proportional_only() {
        let mut pid = TernaryPid::new(1.0, 0.0, 0.0);
        assert_eq!(pid.update(10.0, 5.0), 1); // positive error -> +1
        assert_eq!(pid.update(5.0, 10.0), -1); // negative error -> -1
    }

    #[test]
    fn test_deadband() {
        let mut pid = TernaryPid::new(1.0, 0.0, 0.0);
        pid.deadband = 1.0;
        assert_eq!(pid.update(10.0, 9.5), 0); // within deadband
        assert_eq!(pid.update(10.0, 8.0), 1); // outside deadband
    }

    #[test]
    fn test_integral_memory_persists_output_after_error_clears() {
        // Strengthened from the original test_integral_builds, which only
        // checked that the integral state was nonzero after sustained error
        // (trivially true). Here we verify the *behavior* the integral is
        // for: holding the output after the error has gone to zero.
        let mut pid = TernaryPid::new(0.0, 0.1, 0.0);
        pid.deadband = 0.0;
        // Sustain positive error to charge the integral.
        for _ in 0..10 {
            pid.update(10.0, 9.0); // error = +1
        }
        assert!(pid.integral > 0.0);
        // Error drops to zero. A pure P or D controller would now output 0;
        // the integral memory must keep driving +1.
        let out = pid.update(10.0, 10.0);
        assert_eq!(out, 1);
    }

    #[test]
    fn test_integral_memory_sign_tracks_history() {
        let mut pid = TernaryPid::new(0.0, 0.1, 0.0);
        pid.deadband = 0.0;
        for _ in 0..10 {
            pid.update(9.0, 10.0); // error = -1
        }
        // Zero current error, but the integral remembers the negative history.
        let out = pid.update(10.0, 10.0);
        assert_eq!(out, -1);
    }

    #[test]
    fn test_anti_windup() {
        let mut pid = TernaryPid::new(0.0, 1.0, 0.0);
        pid.integral_limit = 10.0;
        for _ in 0..1000 {
            pid.update(100.0, 0.0);
        }
        assert!(pid.integral <= 10.0);
    }

    #[test]
    fn test_anti_windup_bounds_raw_output() {
        // The existing test_anti_windup only checks the internal integral
        // state stays <= limit. This test verifies the user-visible
        // *output* also stays bounded: i = ki * clamp(integral, ±I_lim).
        let mut pid = TernaryPid::new(0.0, 1.0, 0.0);
        pid.integral_limit = 10.0;
        let mut last_raw = 0.0;
        for _ in 0..10_000 {
            last_raw = pid.update_raw(100.0, 0.0);
        }
        assert!(
            (last_raw - 10.0).abs() < 1e-9,
            "raw output should saturate at ki * I_lim = 10.0, got {last_raw}"
        );
        assert!(pid.integral <= 10.0);
    }

    #[test]
    fn test_derivative_resists_change() {
        let mut pid = TernaryPid::new(0.0, 0.0, 10.0);
        // Sudden change should trigger derivative action
        let out = pid.update(10.0, 10.0); // first call, no derivative
        assert_eq!(out, 0);
        let out2 = pid.update(10.0, 5.0); // sudden drop
        assert_eq!(out2, 1); // derivative kicks in
    }

    #[test]
    fn test_reset() {
        let mut pid = TernaryPid::new(1.0, 1.0, 1.0);
        pid.update(10.0, 5.0);
        pid.reset();
        assert_eq!(pid.integral, 0.0);
        assert!(!pid.initialized);
    }

    #[test]
    fn test_cascade_output_tracks_outer_error_sign() {
        // Replaces the original test_cascade_controller, which asserted
        // `output == 1 || output == -1 || output == 0` — true for ANY i8
        // return, so it could not fail. These assertions check the cascade
        // actually responds to the sign of the outer-loop error.
        let make = || {
            let outer = TernaryPid::new(1.0, 0.1, 0.0);
            let inner = TernaryPid::new(2.0, 0.0, 0.5);
            CascadePid::new(outer, inner)
        };
        // Positive outer error -> inner setpoint rises above inner meas -> +1.
        let mut c = make();
        assert_eq!(c.update(100.0, 50.0, 45.0), 1);
        // Negative outer error -> inner setpoint falls below inner meas -> -1.
        let mut c = make();
        assert_eq!(c.update(0.0, 50.0, 55.0), -1);
    }

    #[test]
    fn test_feedforward() {
        let pid = TernaryPid::new(1.0, 0.0, 0.0);
        let mut ff = FeedforwardPid::new(pid, 1.0, 0.0);
        // Known disturbance of 5.0, setpoint 10.0, measurement 10.0
        // FF adjusts setpoint to 15.0, error = 15-10 = 5 -> +1
        let out = ff.update(10.0, 10.0, 5.0);
        assert_eq!(out, 1);
    }

    #[test]
    fn test_settling_to_zero() {
        let mut pid = TernaryPid::new(1.0, 0.1, 0.5);
        pid.deadband = 0.5;
        // Simulate approaching setpoint
        let mut measurement = 0.0;
        for _ in 0..200 {
            let action = pid.update(10.0, measurement);
            measurement += action as f64 * 0.1;
        }
        assert!((measurement - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_deadband_does_not_break_derivative() {
        // Regression test for the bug where update() returned early from
        // the deadband branch without updating prev_error / initialized.
        // Before the fix, the third call below saw initialized == false
        // (because no prior non-deadband sample had set it) and therefore
        // computed derivative == 0, giving output 0 instead of +1.
        let mut pid = TernaryPid::new(0.0, 0.0, 10.0);
        pid.deadband = 1.0;
        let _ = pid.update(10.0, 10.0); // error = 0, in deadband
        let _ = pid.update(10.0, 9.5); // error = 0.5, in deadband
        let out = pid.update(10.0, 5.0); // error = 5, leaves deadband
        assert_eq!(out, 1); // derivative action must now fire
    }

    #[test]
    fn test_discrete_pid_math_matches_textbook_formula() {
        // Verifies the core control-loop math:
        //   u[n] = Kp*e[n] + Ki*sum(e[0..=n]) + Kd*(e[n] - e[n-1])
        // under the library's fixed-unit-sample-time (Δt = 1) assumption.
        let kp = 2.0;
        let ki = 0.5;
        let kd = 1.0;
        let mut pid = TernaryPid::new(kp, ki, kd);
        pid.derivative_filter = 1.0; // disable low-pass: filtered_d == raw_d
        pid.integral_limit = f64::INFINITY; // disable anti-windup clamp

        let sp = 10.0;
        let measurements = [0.0, 3.0, 7.0, 9.0, 11.0];
        let mut manual_integral = 0.0;
        let mut prev_error = 0.0;
        let mut initialized = false;
        for (n, &m) in measurements.iter().enumerate() {
            let e = sp - m;
            manual_integral += e;
            let raw = pid.update_raw(sp, m);
            let d_term = if initialized {
                kd * (e - prev_error)
            } else {
                0.0
            };
            let expected = kp * e + ki * manual_integral + d_term;
            assert!(
                (raw - expected).abs() < 1e-9,
                "sample {n}: raw={raw}, expected={expected}"
            );
            prev_error = e;
            initialized = true;
        }
    }

    #[test]
    fn test_reset_makes_controller_behave_like_new() {
        let mut a = TernaryPid::new(1.0, 0.5, 0.5);
        let mut b = TernaryPid::new(1.0, 0.5, 0.5);
        // Accumulate state in `a`.
        for i in 0..20 {
            a.update(10.0, i as f64);
        }
        a.reset();
        // After reset, `a` and a fresh `b` must produce identical outputs.
        for m in [5.0_f64, 8.0, 10.0, 12.0, 9.5] {
            let oa = a.update(10.0, m);
            let ob = b.update(10.0, m);
            assert_eq!(oa, ob, "after reset, output mismatch at measurement {m}");
        }
    }

    #[test]
    fn test_update_raw_ignores_deadband() {
        // Documents & verifies the contract: update_raw skips the deadband
        // short-circuit (so cascade outer loops always get a signal).
        let mut ternary = TernaryPid::new(1.0, 0.0, 0.0);
        ternary.deadband = 5.0;
        assert_eq!(ternary.update(10.0, 9.0), 0); // |error| = 1 < deadband

        let mut raw = TernaryPid::new(1.0, 0.0, 0.0);
        raw.deadband = 5.0;
        // Same input, but update_raw ignores deadband: p = kp * error = 1.
        let r = raw.update_raw(10.0, 9.0);
        assert!(
            (r - 1.0).abs() < 1e-9,
            "update_raw should ignore deadband, got {r}"
        );
    }

    #[test]
    fn test_feedforward_negative_disturbance_flips_output() {
        // Symmetry counterpart to test_feedforward (which uses d = +5).
        let pid = TernaryPid::new(1.0, 0.0, 0.0);
        let mut ff = FeedforwardPid::new(pid, 1.0, 0.0);
        // disturbance = -5 -> adjusted_setpoint = 10 - 5 = 5,
        // error = 5 - 10 = -5 -> -1.
        let out = ff.update(10.0, 10.0, -5.0);
        assert_eq!(out, -1);
    }
}
