use fc_core::math::Vec3;

/// Hard limits on what the controller may ask of the airframe.
/// All limits must be positive.
#[derive(Debug, Clone, Copy)]
pub struct LimiterConfig {
    pub max_roll_torque: f32,
    pub max_pitch_torque: f32,
    pub max_yaw_torque: f32,
}

impl Default for LimiterConfig {
    /// Yaw authority of a quadcopter is much smaller than roll/pitch,
    /// because it only comes from propeller reaction torque.
    fn default() -> Self {
        Self {
            max_roll_torque: 0.5,
            max_pitch_torque: 0.5,
            max_yaw_torque: 0.1,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Limiter {
    config: LimiterConfig,
}

impl Limiter {
    pub fn new(config: LimiterConfig) -> Self {
        assert!(
            config.max_roll_torque > 0.0
                && config.max_pitch_torque > 0.0
                && config.max_yaw_torque > 0.0,
            "limiter limits must be positive"
        );
        Self { config }
    }

    /// Limit throttle to [0, 1] and every torque axis to its maximum.
    /// Non-finite values are replaced by 0 (the safe value).
    pub fn apply(&self, throttle: f32, torque: Vec3) -> (f32, Vec3) {
        let c = self.config;
        let throttle = if throttle.is_finite() {
            throttle.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let torque = Vec3::new(
            limit(torque.x, c.max_roll_torque),
            limit(torque.y, c.max_pitch_torque),
            limit(torque.z, c.max_yaw_torque),
        );
        (throttle, torque)
    }
}

fn limit(value: f32, max: f32) -> f32 {
    if value.is_finite() {
        value.clamp(-max, max)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn passes_values_within_limits_unchanged() {
        let limiter = Limiter::new(LimiterConfig::default());
        let torque = Vec3::new(0.1, -0.2, 0.05);
        assert_eq!(limiter.apply(0.5, torque), (0.5, torque));
    }

    #[test]
    fn clamps_torque_per_axis_and_throttle() {
        let limiter = Limiter::new(LimiterConfig::default());
        let (throttle, torque) = limiter.apply(1.5, Vec3::new(1.0, -1.0, 1.0));
        assert_eq!(throttle, 1.0);
        assert_eq!(torque, Vec3::new(0.5, -0.5, 0.1));
        assert_eq!(limiter.apply(-0.3, Vec3::ZERO).0, 0.0);
    }

    #[test]
    fn replaces_non_finite_values_with_safe_defaults() {
        let limiter = Limiter::new(LimiterConfig::default());
        let (throttle, torque) = limiter.apply(f32::NAN, Vec3::new(f32::NAN, f32::INFINITY, 0.01));
        assert_eq!(throttle, 0.0);
        assert_eq!(torque, Vec3::new(0.0, 0.0, 0.01));
    }
}
