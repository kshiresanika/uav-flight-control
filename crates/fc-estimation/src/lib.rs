use fc_core::config::GRAVITY;
use fc_core::math::{Attitude, Vec3, wrap_pi};
use fc_sensors::imu::ImuSample;

/// Pitch is kept just below ±90° to avoid the gimbal-lock singularity.
const MAX_PITCH: f32 = core::f32::consts::FRAC_PI_2 - 0.01;

#[derive(Debug, Clone, Copy)]
pub struct EstimatorConfig {
    /// Filter time constant in seconds. Below it the gyro dominates,
    /// above it the accelerometer dominates.
    pub tau: f32,
    /// Accelerometer is ignored if | |accel| - g | exceeds this (m/s²).
    pub accel_gate: f32,
}

impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            tau: 0.2,
            accel_gate: 3.0,
        }
    }
}

/// Complementary-filter attitude estimator (roll, pitch, yaw).
#[derive(Debug, Clone)]
pub struct AttitudeEstimator {
    config: EstimatorConfig,
    attitude: Attitude,
}

impl AttitudeEstimator {
    pub fn new(config: EstimatorConfig) -> Self {
        Self {
            config,
            attitude: Attitude::default(),
        }
    }

    pub fn attitude(&self) -> Attitude {
        self.attitude
    }

    /// Set roll and pitch directly from one accelerometer reading (yaw = 0).
    /// Useful at start-up so the filter does not have to converge from zero.
    pub fn initialize_from_accel(&mut self, accel: Vec3) {
        let (roll, pitch) = tilt_from_accel(accel);
        self.attitude = Attitude {
            roll,
            pitch: pitch.clamp(-MAX_PITCH, MAX_PITCH),
            yaw: 0.0,
        };
    }

    /// One filter step: predict with the gyro, correct with the accelerometer.
    pub fn update(&mut self, sample: &ImuSample, dt: f32) -> Attitude {
        let alpha = self.config.tau / (self.config.tau + dt);
        let (p, q, r) = (sample.gyro.x, sample.gyro.y, sample.gyro.z);

        // 1. Predict: body rates -> Euler angle rates (using the ESTIMATED angles).
        let (sin_roll, cos_roll) = self.attitude.roll.sin_cos();
        let (tan_pitch, cos_pitch) = (self.attitude.pitch.tan(), self.attitude.pitch.cos());

        let roll_dot = p + (q * sin_roll + r * cos_roll) * tan_pitch;
        let pitch_dot = q * cos_roll - r * sin_roll;
        let yaw_dot = (q * sin_roll + r * cos_roll) / cos_pitch;

        let mut roll = self.attitude.roll + roll_dot * dt;
        let mut pitch = self.attitude.pitch + pitch_dot * dt;
        let yaw = self.attitude.yaw + yaw_dot * dt;

        // 2.-4. Measure, gate, correct.
        // A NaN accelerometer reading makes this comparison false, so it is
        // ignored too (full sensor validation comes in a later phase).
        let norm = sample.accel.norm();
        if (norm - GRAVITY).abs() <= self.config.accel_gate {
            let (roll_acc, pitch_acc) = tilt_from_accel(sample.accel);
            roll += (1.0 - alpha) * wrap_pi(roll_acc - roll);
            pitch += (1.0 - alpha) * (pitch_acc - pitch);
        }

        self.attitude = Attitude {
            roll: wrap_pi(roll),
            pitch: pitch.clamp(-MAX_PITCH, MAX_PITCH),
            yaw: wrap_pi(yaw),
        };
        self.attitude
    }
}

/// Roll and pitch implied by the direction of gravity in the body frame.
fn tilt_from_accel(accel: Vec3) -> (f32, f32) {
    let roll = accel.y.atan2(accel.z);
    let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    (roll, pitch)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_core::config::DT;
    use fc_sensors::imu::{Imu, ImuConfig};
    use fc_sim::{QuadParams, QuadSim};

    #[test]
    fn level_and_still_stays_level() {
        let mut imu = Imu::new(ImuConfig::ideal(), 1);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        for _ in 0..1000 {
            let s = imu.measure(Attitude::default(), Vec3::ZERO);
            est.update(&s, DT);
        }
        let a = est.attitude();
        assert!(a.roll.abs() < 1e-5);
        assert!(a.pitch.abs() < 1e-5);
        assert!(a.yaw.abs() < 1e-5);
    }

    #[test]
    fn initialize_from_accel_matches_tilt() {
        let truth = Attitude {
            roll: 0.3,
            pitch: -0.2,
            yaw: 0.0,
        };
        let s = Imu::new(ImuConfig::ideal(), 1).measure(truth, Vec3::ZERO);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        est.initialize_from_accel(s.accel);
        assert!((est.attitude().roll - 0.3).abs() < 1e-4);
        assert!((est.attitude().pitch + 0.2).abs() < 1e-4);
    }

    #[test]
    fn converges_to_a_static_tilt() {
        let truth = Attitude {
            roll: 0.3,
            pitch: -0.2,
            yaw: 0.0,
        };
        let mut imu = Imu::new(ImuConfig::ideal(), 1);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        // Starts at zero, so the accelerometer must pull it to the truth.
        for _ in 0..3000 {
            let s = imu.measure(truth, Vec3::ZERO);
            est.update(&s, DT);
        }
        assert!((est.attitude().roll - 0.3).abs() < 0.01);
        assert!((est.attitude().pitch + 0.2).abs() < 0.01);
    }

    #[test]
    fn filter_bounds_gyro_bias_drift() {
        let mut imu = Imu::new(ImuConfig::default(), 11);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        let first = imu.measure(Attitude::default(), Vec3::ZERO);
        est.initialize_from_accel(first.accel);

        let mut gyro_only_roll = 0.0_f32;
        for _ in 0..10_000 {
            let s = imu.measure(Attitude::default(), Vec3::ZERO);
            est.update(&s, DT);
            gyro_only_roll += s.gyro.x * DT;
        }
        // Pure integration drifts ~0.005 rad/s * 10 s = 0.05 rad.
        assert!(gyro_only_roll.abs() > 0.04);
        // The filter stays within 0.01 rad (~0.6 deg).
        assert!(est.attitude().roll.abs() < 0.01);
    }

    #[test]
    fn tracks_a_dynamic_roll_kick() {
        let mut sim = QuadSim::new(QuadParams::default());
        let mut imu = Imu::new(ImuConfig::ideal(), 1);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());

        let mut max_err = 0.0_f32;
        for step in 0..2000 {
            let t = step as f32 * DT;
            let torque = if t < 0.1 {
                Vec3::new(0.01, 0.0, 0.0)
            } else {
                Vec3::ZERO
            };
            sim.step(torque, DT);
            let s = imu.measure(sim.attitude, sim.rates);
            let att = est.update(&s, DT);
            max_err = max_err.max((att.roll - sim.attitude.roll).abs());
        }
        assert!(max_err < 0.005);
    }

    #[test]
    fn yaw_drifts_because_nothing_corrects_it() {
        let mut imu = Imu::new(ImuConfig::default(), 3);
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());
        for _ in 0..10_000 {
            let s = imu.measure(Attitude::default(), Vec3::ZERO);
            est.update(&s, DT);
        }
        // Gyro z bias = 0.002 rad/s over 10 s -> about 0.02 rad.
        let yaw = est.attitude().yaw;
        assert!(yaw > 0.015 && yaw < 0.025);
    }

    #[test]
    fn accel_gate_ignores_bad_samples() {
        let mut est = AttitudeEstimator::new(EstimatorConfig::default());

        let hard_acceleration = ImuSample {
            gyro: Vec3::ZERO,
            accel: Vec3::new(0.0, 20.0, 20.0),
        };
        est.update(&hard_acceleration, DT);
        assert_eq!(est.attitude().roll, 0.0);

        let broken = ImuSample {
            gyro: Vec3::ZERO,
            accel: Vec3::new(f32::NAN, 0.0, 0.0),
        };
        est.update(&broken, DT);
        assert!(est.attitude().roll.is_finite());
        assert!(est.attitude().pitch.is_finite());
    }
}
