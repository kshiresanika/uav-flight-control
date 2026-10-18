use core::f32::consts::PI;
use core::ops::{Add, Mul, Sub};

/// 3D vector, used for gyro (rad/s), accel (m/s²), torques, etc.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn norm(self) -> f32 {
        self.dot(self).sqrt()
    }

    /// True only if every component is a finite number (no NaN / Inf).
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Cross product: a × b.
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
}

impl Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, k: f32) -> Vec3 {
        Vec3::new(self.x * k, self.y * k, self.z * k)
    }
}

/// Attitude as Euler angles in radians (roll about X, pitch about Y, yaw about Z).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// Wrap an angle into [-π, π).
pub fn wrap_pi(angle: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let mut r = (angle + PI) % two_pi;
    if r < 0.0 {
        r += two_pi;
    }
    r - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn vec3_add_sub_scale() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
        assert_eq!(b - a, Vec3::new(3.0, 3.0, 3.0));
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
    }

    #[test]
    fn vec3_norm() {
        assert!(close(Vec3::new(3.0, 4.0, 0.0).norm(), 5.0));
    }

    #[test]
    fn vec3_detects_nan_and_inf() {
        assert!(Vec3::new(1.0, 2.0, 3.0).is_finite());
        assert!(!Vec3::new(f32::NAN, 0.0, 0.0).is_finite());
        assert!(!Vec3::new(0.0, f32::INFINITY, 0.0).is_finite());
    }

    #[test]
    fn vec3_cross_product() {
        let x = Vec3::new(1.0, 0.0, 0.0);
        let y = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(x.cross(y), Vec3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn wrap_pi_keeps_small_angles() {
        assert!(close(wrap_pi(0.5), 0.5));
        assert!(close(wrap_pi(-0.5), -0.5));
    }

    #[test]
    fn wrap_pi_wraps_large_angles() {
        assert!(close(wrap_pi(3.0 * PI / 2.0), -PI / 2.0));
        assert!(close(wrap_pi(-3.0 * PI / 2.0), PI / 2.0));
        assert!(close(wrap_pi(4.0 * PI), 0.0));
    }
}
