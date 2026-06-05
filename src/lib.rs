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
            kp, ki, kd,
            deadband: 0.0,
            integral_limit: 100.0,
            derivative_filter: 0.1,
            integral: 0.0,
            prev_error: 0.0,
            filtered_derivative: 0.0,
            initialized: false,
        }
    }

    /// Compute ternary output for given setpoint and measurement
    pub fn update(&mut self, setpoint: f64, measurement: f64) -> i8 {
        let error = setpoint - measurement;

        // Deadband check
        if error.abs() < self.deadband {
            self.integral *= 0.95; // Slowly bleed integral in deadband
            return 0;
        }

        // Proportional
        let p = self.kp * error;

        // Integral with anti-windup
        self.integral += error;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        // Derivative with filtering
        let derivative = if self.initialized {
            let raw_d = error - self.prev_error;
            self.filtered_derivative = self.derivative_filter * raw_d + (1.0 - self.derivative_filter) * self.filtered_derivative;
            self.filtered_derivative
        } else {
            0.0
        };
        let d = self.kd * derivative;

        self.prev_error = error;
        self.initialized = true;

        let output = p + i + d;
        // Ternary decision
        if output > 0.0 { 1 } else if output < 0.0 { -1 } else { 0 }
    }

    /// Get raw PID output before ternary quantization
    pub fn update_raw(&mut self, setpoint: f64, measurement: f64) -> f64 {
        let error = setpoint - measurement;
        let p = self.kp * error;
        self.integral += error;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;
        let derivative = if self.initialized {
            let raw_d = error - self.prev_error;
            self.filtered_derivative = self.derivative_filter * raw_d + (1.0 - self.derivative_filter) * self.filtered_derivative;
            self.filtered_derivative
        } else { 0.0 };
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
    pub fn update(&mut self, outer_setpoint: f64, outer_measurement: f64, inner_measurement: f64) -> i8 {
        let inner_setpoint = self.outer.update_raw(outer_setpoint, outer_measurement) + outer_setpoint;
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
        Self { pid, ff_gain, disturbance_bias }
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
        assert_eq!(pid.update(10.0, 5.0), 1);  // positive error -> +1
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
    fn test_integral_builds() {
        let mut pid = TernaryPid::new(0.0, 0.1, 0.0);
        pid.deadband = 0.0;
        // Small persistent error should eventually produce output via integral
        for _ in 0..100 {
            pid.update(10.0, 9.0);
        }
        assert!(pid.integral > 0.0);
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
    fn test_cascade_controller() {
        let outer = TernaryPid::new(1.0, 0.1, 0.0);
        let inner = TernaryPid::new(2.0, 0.0, 0.5);
        let mut cascade = CascadePid::new(outer, inner);
        let output = cascade.update(100.0, 50.0, 45.0);
        assert!(output == 1 || output == -1 || output == 0);
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
}
