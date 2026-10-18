use crate::imu::ImuSample;
use fc_core::math::Vec3;

/// Why a sample was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFault {
    /// NaN or infinity in the data.
    NonFinite,
    /// Value outside the physical measurement range of the sensor.
    OutOfRange,
    /// Change since the last good sample is larger than physically possible.
    Jump,
    /// The exact same value repeated for too long (frozen sensor).
    Stuck,
}

/// Plausibility limits for one sensor (all three axes share them).
#[derive(Debug, Clone, Copy)]
pub struct ChannelLimits {
    /// Largest plausible absolute value per axis.
    pub max_abs: f32,
    /// Largest plausible change per axis between two consecutive samples.
    pub max_step: f32,
    /// Number of identical repeats after which the sensor counts as stuck.
    pub stuck_samples: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct ImuValidatorConfig {
    pub gyro: ChannelLimits,
    pub accel: ChannelLimits,
}

impl Default for ImuValidatorConfig {
    fn default() -> Self {
        Self {
            // Gyro: +-35 rad/s (about 2000 deg/s). The airframe can change its
            // rate by only ~0.07 rad/s per 1 ms, so 1.0 rad/s is a clear glitch.
            gyro: ChannelLimits {
                max_abs: 35.0,
                max_step: 1.0,
                stuck_samples: 50,
            },
            // Accelerometer: +-16 g.
            accel: ChannelLimits {
                max_abs: 157.0,
                max_step: 5.0,
                stuck_samples: 50,
            },
        }
    }
}

/// Result of validating one IMU sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuValidation {
    pub gyro: Option<SensorFault>,
    pub accel: Option<SensorFault>,
}

impl ImuValidation {
    pub fn is_ok(&self) -> bool {
        self.gyro.is_none() && self.accel.is_none()
    }
}

/// Validator for one 3-axis sensor.
#[derive(Debug, Clone)]
struct ChannelValidator {
    limits: ChannelLimits,
    /// Last sample that passed every check (reference for the jump check).
    last_good: Option<Vec3>,
    /// Last finite sample, good or not (reference for the stuck check).
    last_raw: Option<Vec3>,
    identical_count: u32,
}

impl ChannelValidator {
    fn new(limits: ChannelLimits) -> Self {
        Self {
            limits,
            last_good: None,
            last_raw: None,
            identical_count: 0,
        }
    }

    fn check(&mut self, v: Vec3) -> Option<SensorFault> {
        // NaN/Inf must never touch the internal state.
        if !v.is_finite() {
            return Some(SensorFault::NonFinite);
        }

        // Stuck tracking: count exact repeats of the previous sample.
        if self.last_raw == Some(v) {
            self.identical_count = self.identical_count.saturating_add(1);
        } else {
            self.identical_count = 0;
        }
        self.last_raw = Some(v);

        let fault = if any_exceeds(v, self.limits.max_abs) {
            Some(SensorFault::OutOfRange)
        } else if self
            .last_good
            .is_some_and(|good| any_exceeds(v - good, self.limits.max_step))
        {
            Some(SensorFault::Jump)
        } else if self.identical_count >= self.limits.stuck_samples {
            Some(SensorFault::Stuck)
        } else {
            None
        };

        if fault.is_none() {
            self.last_good = Some(v);
        }
        fault
    }
}

fn any_exceeds(v: Vec3, limit: f32) -> bool {
    v.x.abs() > limit || v.y.abs() > limit || v.z.abs() > limit
}

/// Validates gyro and accelerometer of every IMU sample.
#[derive(Debug, Clone)]
pub struct ImuValidator {
    gyro: ChannelValidator,
    accel: ChannelValidator,
}

impl ImuValidator {
    pub fn new(config: ImuValidatorConfig) -> Self {
        Self {
            gyro: ChannelValidator::new(config.gyro),
            accel: ChannelValidator::new(config.accel),
        }
    }

    pub fn check(&mut self, sample: &ImuSample) -> ImuValidation {
        ImuValidation {
            gyro: self.gyro.check(sample.gyro),
            accel: self.accel.check(sample.accel),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn validator() -> ImuValidator {
        ImuValidator::new(ImuValidatorConfig::default())
    }

    fn sample(gyro: Vec3, accel: Vec3) -> ImuSample {
        ImuSample { gyro, accel }
    }

    /// A slowly changing healthy signal: never identical twice, never jumps.
    fn healthy(i: usize) -> ImuSample {
        let k = 0.001 * i as f32;
        sample(Vec3::new(k, 0.0, 0.0), Vec3::new(0.0, 0.0, 9.8 + k))
    }

    #[test]
    fn healthy_sample_passes() {
        let s = sample(Vec3::new(0.01, -0.02, 0.0), Vec3::new(0.0, 0.1, 9.8));
        assert!(validator().check(&s).is_ok());
    }

    #[test]
    fn detects_nan_and_infinity() {
        let mut v = validator();
        let bad_gyro = sample(Vec3::new(f32::NAN, 0.0, 0.0), Vec3::new(0.0, 0.0, 9.8));
        let r = v.check(&bad_gyro);
        assert_eq!(r.gyro, Some(SensorFault::NonFinite));
        assert_eq!(r.accel, None);

        let bad_accel = sample(Vec3::ZERO, Vec3::new(0.0, f32::INFINITY, 9.8));
        let r = v.check(&bad_accel);
        assert_eq!(r.gyro, None);
        assert_eq!(r.accel, Some(SensorFault::NonFinite));
    }

    #[test]
    fn detects_out_of_range_values() {
        let mut v = validator();
        let r = v.check(&sample(Vec3::new(50.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 9.8)));
        assert_eq!(r.gyro, Some(SensorFault::OutOfRange));

        let r = v.check(&sample(Vec3::ZERO, Vec3::new(0.0, 0.0, -200.0)));
        assert_eq!(r.accel, Some(SensorFault::OutOfRange));
    }

    #[test]
    fn detects_a_spike_and_recovers_afterwards() {
        let mut v = validator();
        for i in 0..20 {
            assert!(v.check(&healthy(i)).is_ok());
        }
        // One-sample spike: in range, but physically impossible.
        let mut spike = healthy(20);
        spike.gyro.x = 3.0;
        assert_eq!(v.check(&spike).gyro, Some(SensorFault::Jump));
        // The spike was not accepted as reference, so the next sample is fine.
        assert!(v.check(&healthy(21)).is_ok());
    }

    #[test]
    fn a_persistent_offset_stays_faulty() {
        let mut v = validator();
        for i in 0..20 {
            assert!(v.check(&healthy(i)).is_ok());
        }
        for i in 20..30 {
            let mut s = healthy(i);
            s.accel.z += 50.0; // permanent offset
            assert_eq!(v.check(&s).accel, Some(SensorFault::Jump));
        }
    }

    #[test]
    fn detects_a_stuck_sensor() {
        let config = ImuValidatorConfig {
            gyro: ChannelLimits {
                stuck_samples: 5,
                ..ImuValidatorConfig::default().gyro
            },
            ..ImuValidatorConfig::default()
        };
        let mut v = ImuValidator::new(config);
        let frozen = Vec3::new(0.1, 0.2, 0.3);

        for i in 0..5 {
            // Gyro frozen, accelerometer alive.
            let s = sample(frozen, healthy(i).accel);
            assert_eq!(v.check(&s).gyro, None, "sample {i} should still pass");
        }
        let s = sample(frozen, healthy(5).accel);
        let r = v.check(&s);
        assert_eq!(r.gyro, Some(SensorFault::Stuck));
        assert_eq!(r.accel, None);

        // As soon as the value moves again, the fault clears.
        let s = sample(Vec3::new(0.1, 0.2, 0.301), healthy(6).accel);
        assert_eq!(v.check(&s).gyro, None);
    }

    #[test]
    fn nan_does_not_corrupt_validator_state() {
        let mut v = validator();
        assert!(v.check(&healthy(0)).is_ok());
        let nan = sample(Vec3::new(f32::NAN, f32::NAN, f32::NAN), Vec3::ZERO);
        assert!(!v.check(&nan).is_ok());
        assert!(v.check(&healthy(1)).is_ok());
    }

    #[test]
    fn faults_in_gyro_and_accel_are_reported_independently() {
        let mut v = validator();
        let s = sample(Vec3::new(0.0, f32::NAN, 0.0), Vec3::new(0.0, 0.0, 300.0));
        let r = v.check(&s);
        assert_eq!(r.gyro, Some(SensorFault::NonFinite));
        assert_eq!(r.accel, Some(SensorFault::OutOfRange));
    }
}
