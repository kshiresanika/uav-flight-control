/// Tuning and limits of one PID loop.
/// `integral_limit` and `output_limit` must be positive.
#[derive(Debug, Clone, Copy)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Maximum magnitude of the integral term (in output units).
    pub integral_limit: f32,
    /// Maximum magnitude of the total output.
    pub output_limit: f32,
    /// Time constant of the low-pass filter on the derivative input (s).
    pub d_filter_tau: f32,
}

/// PID controller with a filtered derivative and anti-windup.
#[derive(Debug, Clone)]
pub struct Pid {
    config: PidConfig,
    /// Integral term, already multiplied by ki (so it is in output units).
    integral: f32,
    d_filtered: f32,
}

impl Pid {
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            integral: 0.0,
            d_filtered: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.d_filtered = 0.0;
    }

    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// One controller step.
    /// `error`      = setpoint - measurement
    /// `error_rate` = derivative of the error (for a constant setpoint: -gyro rate)
    pub fn update(&mut self, error: f32, error_rate: f32, dt: f32) -> f32 {
        // Never let NaN / Inf enter the controller state.
        if !error.is_finite() || !error_rate.is_finite() {
            return 0.0;
        }
        let c = self.config;

        // Low-pass filter on the derivative input.
        let a = dt / (c.d_filter_tau + dt);
        self.d_filtered += a * (error_rate - self.d_filtered);

        let p = c.kp * error;
        let d = c.kd * self.d_filtered;

        // Anti-windup: conditional integration + clamp.
        let candidate =
            (self.integral + c.ki * error * dt).clamp(-c.integral_limit, c.integral_limit);
        let unsaturated = p + candidate + d;
        let pushing_into_saturation =
            unsaturated.abs() > c.output_limit && unsaturated.signum() == error.signum();
        if !pushing_into_saturation {
            self.integral = candidate;
        }

        (p + self.integral + d).clamp(-c.output_limit, c.output_limit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(kp: f32, ki: f32, kd: f32) -> PidConfig {
        PidConfig {
            kp,
            ki,
            kd,
            integral_limit: 10.0,
            output_limit: 10.0,
            d_filter_tau: 0.0,
        }
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn proportional_term() {
        let mut pid = Pid::new(cfg(2.0, 0.0, 0.0));
        assert!(close(pid.update(0.5, 0.0, 0.001), 1.0, 1e-6));
    }

    #[test]
    fn output_is_clamped() {
        let mut pid = Pid::new(PidConfig {
            output_limit: 2.0,
            ..cfg(100.0, 0.0, 0.0)
        });
        assert_eq!(pid.update(1.0, 0.0, 0.001), 2.0);
        assert_eq!(pid.update(-1.0, 0.0, 0.001), -2.0);
    }

    #[test]
    fn integral_accumulates() {
        let mut pid = Pid::new(cfg(0.0, 1.0, 0.0));
        let mut out = 0.0;
        for _ in 0..100 {
            out = pid.update(1.0, 0.0, 0.01);
        }
        assert!(close(out, 1.0, 1e-3));
    }

    #[test]
    fn integral_is_clamped() {
        let mut pid = Pid::new(PidConfig {
            integral_limit: 0.2,
            ..cfg(0.0, 1.0, 0.0)
        });
        for _ in 0..1000 {
            pid.update(1.0, 0.0, 0.01);
        }
        assert!(close(pid.integral(), 0.2, 1e-6));
    }

    #[test]
    fn derivative_passes_through_without_filter() {
        let mut pid = Pid::new(cfg(0.0, 0.0, 1.0));
        assert!(close(pid.update(0.0, 3.0, 0.001), 3.0, 1e-5));
    }

    #[test]
    fn derivative_filter_smooths_a_sudden_change() {
        let mut pid = Pid::new(PidConfig {
            d_filter_tau: 0.1,
            ..cfg(0.0, 0.0, 1.0)
        });
        let out = pid.update(0.0, 10.0, 0.001);
        assert!(out > 0.0 && out < 0.2);
    }

    #[test]
    fn anti_windup_stops_integration_while_saturated() {
        let mut pid = Pid::new(PidConfig {
            integral_limit: 100.0,
            output_limit: 1.0,
            ..cfg(1.0, 10.0, 0.0)
        });
        for _ in 0..1000 {
            pid.update(5.0, 0.0, 0.001);
        }
        assert!(pid.integral().abs() < 1e-6);
        // Error flips sign: the output reacts immediately, no windup to unwind.
        let out = pid.update(-0.1, 0.0, 0.001);
        assert!(out < 0.0 && out > -0.2);
    }

    #[test]
    fn non_finite_input_is_rejected() {
        let mut pid = Pid::new(cfg(1.0, 1.0, 1.0));
        assert_eq!(pid.update(f32::NAN, 0.0, 0.001), 0.0);
        assert_eq!(pid.update(0.0, f32::INFINITY, 0.001), 0.0);
        assert!(pid.update(1.0, 0.0, 0.001).is_finite());
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = Pid::new(cfg(0.0, 1.0, 0.0));
        for _ in 0..100 {
            pid.update(1.0, 0.0, 0.01);
        }
        pid.reset();
        assert_eq!(pid.integral(), 0.0);
    }
}
