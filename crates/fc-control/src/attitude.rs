use crate::pid::{Pid, PidConfig};
use fc_core::math::{Attitude, Vec3, wrap_pi};

/// One PID configuration per axis.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeGains {
    pub roll: PidConfig,
    pub pitch: PidConfig,
    pub yaw: PidConfig,
}

impl Default for AttitudeGains {
    /// Derived from the plant: wn = 20 rad/s, zeta = 0.8 for roll/pitch
    /// (kp = I*wn^2, kd = 2*zeta*wn*I); wn = 10 rad/s for yaw.
    fn default() -> Self {
        let roll_pitch = PidConfig {
            kp: 3.0,
            ki: 1.0,
            kd: 0.24,
            integral_limit: 0.2,
            output_limit: 0.5,
            d_filter_tau: 0.005,
        };
        let yaw = PidConfig {
            kp: 1.3,
            ki: 0.3,
            kd: 0.208,
            integral_limit: 0.1,
            output_limit: 0.2,
            d_filter_tau: 0.005,
        };
        Self {
            roll: roll_pitch,
            pitch: roll_pitch,
            yaw,
        }
    }
}

/// Three independent PID loops: attitude error -> body torque (N·m).
#[derive(Debug, Clone)]
pub struct AttitudeController {
    roll: Pid,
    pitch: Pid,
    yaw: Pid,
}

impl AttitudeController {
    pub fn new(gains: AttitudeGains) -> Self {
        Self {
            roll: Pid::new(gains.roll),
            pitch: Pid::new(gains.pitch),
            yaw: Pid::new(gains.yaw),
        }
    }

    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }

    /// Compute the body torque command.
    /// `gyro` is the measured body rate: the derivative term acts on it directly.
    pub fn update(&mut self, setpoint: Attitude, estimate: Attitude, gyro: Vec3, dt: f32) -> Vec3 {
        let roll_error = setpoint.roll - estimate.roll;
        let pitch_error = setpoint.pitch - estimate.pitch;
        // Yaw wraps: always take the short way round.
        let yaw_error = wrap_pi(setpoint.yaw - estimate.yaw);

        Vec3::new(
            self.roll.update(roll_error, -gyro.x, dt),
            self.pitch.update(pitch_error, -gyro.y, dt),
            self.yaw.update(yaw_error, -gyro.z, dt),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_core::config::{CONTROL_RATE_HZ, DT};
    use fc_estimation::{AttitudeEstimator, EstimatorConfig};
    use fc_sensors::imu::{Imu, ImuConfig};
    use fc_sim::{QuadParams, QuadSim};

    /// Full closed loop: sim -> IMU -> estimator -> controller -> sim.
    /// Returns the final simulator state and the largest roll angle reached.
    fn simulate(
        initial: Attitude,
        setpoint: Attitude,
        imu_config: ImuConfig,
        gains: AttitudeGains,
        disturbance: Vec3,
        seconds: f32,
    ) -> (QuadSim, f32) {
        let mut sim = QuadSim::new(QuadParams::default());
        sim.attitude = initial;
        let mut imu = Imu::new(imu_config, 42);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        let mut ctrl = AttitudeController::new(gains);

        let first = imu.measure(sim.attitude, sim.rates);
        est.initialize_from_accel(first.accel);

        let steps = (seconds * CONTROL_RATE_HZ) as usize;
        let mut max_roll = f32::MIN;
        for _ in 0..steps {
            let sample = imu.measure(sim.attitude, sim.rates);
            let att = est.update(&sample, DT);
            let torque = ctrl.update(setpoint, att, sample.gyro, DT);
            sim.step(torque + disturbance, DT);
            max_roll = max_roll.max(sim.attitude.roll);
        }
        (sim, max_roll)
    }

    #[test]
    fn yaw_error_takes_the_short_way() {
        let mut ctrl = AttitudeController::new(AttitudeGains::default());
        let setpoint = Attitude {
            yaw: 3.0,
            ..Attitude::default()
        };
        let estimate = Attitude {
            yaw: -3.0,
            ..Attitude::default()
        };
        // 3.0 - (-3.0) = 6.0 rad, which wraps to -0.28 rad: turn negative.
        let torque = ctrl.update(setpoint, estimate, Vec3::ZERO, DT);
        assert!(torque.z < 0.0);
    }

    #[test]
    fn recovers_level_flight_from_a_large_tilt() {
        let initial = Attitude {
            roll: 0.3,
            pitch: -0.2,
            yaw: 0.0,
        };
        let (sim, _) = simulate(
            initial,
            Attitude::default(),
            ImuConfig::ideal(),
            AttitudeGains::default(),
            Vec3::ZERO,
            3.0,
        );
        assert!(sim.attitude.roll.abs() < 0.01);
        assert!(sim.attitude.pitch.abs() < 0.01);
    }

    #[test]
    fn roll_step_response_has_small_overshoot_and_settles() {
        let target = 0.1745; // 10 degrees
        let setpoint = Attitude {
            roll: target,
            ..Attitude::default()
        };
        let (sim, max_roll) = simulate(
            Attitude::default(),
            setpoint,
            ImuConfig::ideal(),
            AttitudeGains::default(),
            Vec3::ZERO,
            4.0,
        );
        assert!(max_roll < target * 1.10);
        assert!((sim.attitude.roll - target).abs() < 0.005);
    }

    #[test]
    fn integral_term_rejects_a_constant_disturbance() {
        let disturbance = Vec3::new(0.02, 0.0, 0.0); // constant torque, N·m

        // Without the I term a constant torque leaves an offset of about 0.02 / kp.
        let mut p_only = AttitudeGains::default();
        p_only.roll.ki = 0.0;
        let (without_i, _) = simulate(
            Attitude::default(),
            Attitude::default(),
            ImuConfig::ideal(),
            p_only,
            disturbance,
            12.0,
        );
        assert!(without_i.attitude.roll.abs() > 0.005);

        // With the I term the offset is removed.
        let (with_i, _) = simulate(
            Attitude::default(),
            Attitude::default(),
            ImuConfig::ideal(),
            AttitudeGains::default(),
            disturbance,
            12.0,
        );
        assert!(with_i.attitude.roll.abs() < 0.002);
    }

    #[test]
    fn yaw_step_converges() {
        let target = 0.3;
        let setpoint = Attitude {
            yaw: target,
            ..Attitude::default()
        };
        let (sim, _) = simulate(
            Attitude::default(),
            setpoint,
            ImuConfig::ideal(),
            AttitudeGains::default(),
            Vec3::ZERO,
            6.0,
        );
        assert!((sim.attitude.yaw - target).abs() < 0.02);
    }

    #[test]
    fn holds_level_with_noisy_biased_sensors() {
        let (sim, _) = simulate(
            Attitude::default(),
            Attitude::default(),
            ImuConfig::default(),
            AttitudeGains::default(),
            Vec3::ZERO,
            5.0,
        );
        assert!(sim.attitude.roll.abs() < 0.02);
        assert!(sim.attitude.pitch.abs() < 0.02);
    }
}
