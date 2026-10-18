use fc_core::math::{Attitude, Vec3, wrap_pi};

/// Physical parameters of the quadcopter (rotational part only).
#[derive(Debug, Clone, Copy)]
pub struct QuadParams {
    /// Moments of inertia about body X, Y, Z in kg·m².
    pub inertia: Vec3,
    /// Aerodynamic damping coefficient in N·m per (rad/s).
    pub angular_drag: f32,
}

impl Default for QuadParams {
    /// Roughly a 450 mm class quadcopter.
    fn default() -> Self {
        Self {
            inertia: Vec3::new(0.0075, 0.0075, 0.013),
            angular_drag: 0.002,
        }
    }
}

/// Rotational dynamics of a quadcopter: the "true" state of the world.
#[derive(Debug, Clone, Copy)]
pub struct QuadSim {
    pub params: QuadParams,
    /// True orientation (roll, pitch, yaw) in radians.
    pub attitude: Attitude,
    /// True body angular rates (p, q, r) in rad/s.
    pub rates: Vec3,
}

/// Pitch is kept just below ±90° to avoid the gimbal-lock singularity.
const MAX_PITCH: f32 = core::f32::consts::FRAC_PI_2 - 0.01;

impl QuadSim {
    pub fn new(params: QuadParams) -> Self {
        Self {
            params,
            attitude: Attitude::default(),
            rates: Vec3::ZERO,
        }
    }

    /// Advance the simulation by `dt` seconds under the given body torque (N·m).
    pub fn step(&mut self, torque: Vec3, dt: f32) {
        let i = self.params.inertia;
        let w = self.rates;

        // Euler's rotation equation: I·dω/dt = τ − ω × (I·ω) − drag
        let i_w = Vec3::new(i.x * w.x, i.y * w.y, i.z * w.z);
        let gyroscopic = w.cross(i_w);
        let drag = w * self.params.angular_drag;
        let net = torque - gyroscopic - drag;
        let alpha = Vec3::new(net.x / i.x, net.y / i.y, net.z / i.z);

        // Semi-implicit Euler: update rate first, then use the NEW rate for the angle.
        self.rates = self.rates + alpha * dt;
        let (p, q, r) = (self.rates.x, self.rates.y, self.rates.z);

        // Body rates -> Euler angle rates (ZYX convention).
        let (sin_roll, cos_roll) = self.attitude.roll.sin_cos();
        let (tan_pitch, cos_pitch) = (self.attitude.pitch.tan(), self.attitude.pitch.cos());

        let roll_dot = p + (q * sin_roll + r * cos_roll) * tan_pitch;
        let pitch_dot = q * cos_roll - r * sin_roll;
        let yaw_dot = (q * sin_roll + r * cos_roll) / cos_pitch;

        self.attitude.roll = wrap_pi(self.attitude.roll + roll_dot * dt);
        self.attitude.pitch = (self.attitude.pitch + pitch_dot * dt).clamp(-MAX_PITCH, MAX_PITCH);
        self.attitude.yaw = wrap_pi(self.attitude.yaw + yaw_dot * dt);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_core::config::DT;

    fn no_drag() -> QuadParams {
        QuadParams {
            angular_drag: 0.0,
            ..QuadParams::default()
        }
    }

    #[test]
    fn stays_at_rest_without_torque() {
        let mut sim = QuadSim::new(QuadParams::default());
        for _ in 0..1000 {
            sim.step(Vec3::ZERO, DT);
        }
        assert_eq!(sim.rates, Vec3::ZERO);
        assert_eq!(sim.attitude, Attitude::default());
    }

    #[test]
    fn constant_torque_gives_constant_acceleration() {
        // alpha = 0.01 / 0.0075 = 1.3333 rad/s². After 0.1 s: rate = 0.1333 rad/s.
        let mut sim = QuadSim::new(no_drag());
        for _ in 0..100 {
            sim.step(Vec3::new(0.01, 0.0, 0.0), DT);
        }
        assert!((sim.rates.x - 0.1333).abs() < 1e-3);
        // angle ≈ ½·alpha·t² = 0.00667 rad
        assert!((sim.attitude.roll - 0.00667).abs() < 2e-4);
    }

    #[test]
    fn drag_slows_a_spinning_body() {
        let mut sim = QuadSim::new(QuadParams::default());
        sim.rates = Vec3::new(1.0, 0.0, 0.0);
        for _ in 0..1000 {
            sim.step(Vec3::ZERO, DT);
        }
        assert!(sim.rates.x < 1.0);
        assert!(sim.rates.x > 0.0);
    }

    #[test]
    fn yaw_angle_stays_wrapped() {
        let mut sim = QuadSim::new(no_drag());
        sim.rates = Vec3::new(0.0, 0.0, 10.0); // fast yaw spin
        for _ in 0..3000 {
            sim.step(Vec3::ZERO, DT);
            assert!(sim.attitude.yaw.abs() <= core::f32::consts::PI);
        }
    }

    #[test]
    fn simulation_is_deterministic() {
        let run = || {
            let mut sim = QuadSim::new(QuadParams::default());
            for k in 0..2000 {
                let t = if k < 100 { 0.01 } else { 0.0 };
                sim.step(Vec3::new(t, -t, 0.5 * t), DT);
            }
            sim
        };
        let (a, b) = (run(), run());
        assert_eq!(a.attitude, b.attitude);
        assert_eq!(a.rates, b.rates);
    }
    #[test]

    fn pitch_is_clamped_below_gimbal_lock() {
        let mut sim = QuadSim::new(no_drag());
        sim.rates = Vec3::new(0.0, 10.0, 0.0); // fast nose-up rotation
        for _ in 0..1000 {
            sim.step(Vec3::ZERO, DT);
        }
        assert!((sim.attitude.pitch - MAX_PITCH).abs() < 1e-6);
        assert!(sim.attitude.roll.is_finite());
        assert!(sim.attitude.yaw.is_finite());
    }
}
