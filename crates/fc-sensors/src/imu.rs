use crate::rng::Rng;
use fc_core::config::GRAVITY;
use fc_core::math::{Attitude, Vec3};

/// Error model of the IMU.
#[derive(Debug, Clone, Copy)]
pub struct ImuConfig {
    /// Gyro white-noise standard deviation per sample (rad/s).
    pub gyro_noise_std: f32,
    /// Constant gyro bias (rad/s).
    pub gyro_bias: Vec3,
    /// Accelerometer white-noise standard deviation per sample (m/s²).
    pub accel_noise_std: f32,
    /// Constant accelerometer bias (m/s²).
    pub accel_bias: Vec3,
}

impl ImuConfig {
    /// A perfect sensor: no noise, no bias. Useful for testing.
    pub fn ideal() -> Self {
        Self {
            gyro_noise_std: 0.0,
            gyro_bias: Vec3::ZERO,
            accel_noise_std: 0.0,
            accel_bias: Vec3::ZERO,
        }
    }
}

impl Default for ImuConfig {
    /// Roughly a consumer-grade MEMS IMU.
    fn default() -> Self {
        Self {
            gyro_noise_std: 0.002,                      // ~0.11 deg/s
            gyro_bias: Vec3::new(0.005, -0.003, 0.002), // ~0.3 deg/s
            accel_noise_std: 0.05,                      // m/s²
            accel_bias: Vec3::new(0.02, -0.02, 0.03),
        }
    }
}

/// One IMU reading, in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    /// Angular rate in rad/s.
    pub gyro: Vec3,
    /// Specific force in m/s².
    pub accel: Vec3,
}

/// Simulated IMU: turns true state into noisy measurements.
#[derive(Debug, Clone)]
pub struct Imu {
    config: ImuConfig,
    rng: Rng,
}

impl Imu {
    pub fn new(config: ImuConfig, seed: u64) -> Self {
        Self {
            config,
            rng: Rng::new(seed),
        }
    }

    fn noise(&mut self, std: f32) -> Vec3 {
        Vec3::new(
            self.rng.gaussian() * std,
            self.rng.gaussian() * std,
            self.rng.gaussian() * std,
        )
    }

    /// Measure the true attitude and body rates.
    /// Assumes a hovering drone: no linear acceleration, only gravity.
    pub fn measure(&mut self, attitude: Attitude, rates: Vec3) -> ImuSample {
        let (sin_roll, cos_roll) = attitude.roll.sin_cos();
        let (sin_pitch, cos_pitch) = attitude.pitch.sin_cos();

        // Gravity (specific force at hover) expressed in the body frame.
        let specific_force = Vec3::new(
            -GRAVITY * sin_pitch,
            GRAVITY * sin_roll * cos_pitch,
            GRAVITY * cos_roll * cos_pitch,
        );

        let gyro_std = self.config.gyro_noise_std;
        let accel_std = self.config.accel_noise_std;

        let gyro = rates + self.config.gyro_bias + self.noise(gyro_std);
        let accel = specific_force + self.config.accel_bias + self.noise(accel_std);

        ImuSample { gyro, accel }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    fn ideal_imu() -> Imu {
        Imu::new(ImuConfig::ideal(), 1)
    }

    #[test]
    fn level_drone_reads_one_g_on_z() {
        let s = ideal_imu().measure(Attitude::default(), Vec3::ZERO);
        assert!(close(s.accel.x, 0.0, 1e-4));
        assert!(close(s.accel.y, 0.0, 1e-4));
        assert!(close(s.accel.z, GRAVITY, 1e-4));
    }

    #[test]
    fn roll_30_degrees_splits_gravity_between_y_and_z() {
        let att = Attitude {
            roll: PI / 6.0,
            ..Attitude::default()
        };
        let s = ideal_imu().measure(att, Vec3::ZERO);
        assert!(close(s.accel.y, 4.9033, 1e-3));
        assert!(close(s.accel.z, 8.4932, 1e-3));
    }

    #[test]
    fn pitch_30_degrees_shows_up_on_x() {
        let att = Attitude {
            pitch: PI / 6.0,
            ..Attitude::default()
        };
        let s = ideal_imu().measure(att, Vec3::ZERO);
        assert!(close(s.accel.x, -4.9033, 1e-3));
    }

    #[test]
    fn accel_magnitude_is_always_one_g() {
        let attitudes = [(0.3, -0.4, 1.0), (-1.0, 0.5, 2.0), (0.0, 1.2, -3.0)];
        for (roll, pitch, yaw) in attitudes {
            let att = Attitude { roll, pitch, yaw };
            let s = ideal_imu().measure(att, Vec3::ZERO);
            assert!(close(s.accel.norm(), GRAVITY, 1e-3));
        }
    }

    #[test]
    fn gyro_bias_appears_at_rest() {
        let cfg = ImuConfig {
            gyro_bias: Vec3::new(0.01, -0.02, 0.03),
            ..ImuConfig::ideal()
        };
        let s = Imu::new(cfg, 1).measure(Attitude::default(), Vec3::ZERO);
        assert_eq!(s.gyro, Vec3::new(0.01, -0.02, 0.03));
    }

    #[test]
    fn measured_gyro_noise_matches_configured_std() {
        let cfg = ImuConfig {
            gyro_noise_std: 0.01,
            ..ImuConfig::ideal()
        };
        let mut imu = Imu::new(cfg, 99);
        let n = 20_000_usize;
        let mut sum = 0.0_f64;
        let mut sum_sq = 0.0_f64;
        for _ in 0..n {
            let x = imu.measure(Attitude::default(), Vec3::ZERO).gyro.x as f64;
            sum += x;
            sum_sq += x * x;
        }
        let mean = sum / n as f64;
        let std = (sum_sq / n as f64 - mean * mean).sqrt();
        assert!((std - 0.01).abs() < 0.0005);
    }

    #[test]
    fn same_seed_gives_identical_measurements() {
        let mut a = Imu::new(ImuConfig::default(), 5);
        let mut b = Imu::new(ImuConfig::default(), 5);
        let att = Attitude {
            roll: 0.1,
            pitch: -0.2,
            yaw: 0.3,
        };
        for _ in 0..1000 {
            let r = Vec3::new(0.1, 0.2, 0.3);
            assert_eq!(a.measure(att, r), b.measure(att, r));
        }
    }
}
