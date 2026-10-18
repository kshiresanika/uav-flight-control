use core::f32::consts::FRAC_1_SQRT_2;
use fc_core::math::Vec3;

pub const NUM_MOTORS: usize = 4;

/// Normalised motor commands in [0, 1], order M1..M4.
pub type MotorCommands = [f32; NUM_MOTORS];

// Motor layout (top view, x forward, y left, z up):
//
//    M2 (CW)    M1 (CCW)      front
//    M3 (CCW)   M4 (CW)       rear
//
// Sign of each motor's contribution to a positive torque about each axis.
const ROLL: [f32; NUM_MOTORS] = [-1.0, 1.0, 1.0, -1.0]; // left motors up
const PITCH: [f32; NUM_MOTORS] = [-1.0, -1.0, 1.0, 1.0]; // rear motors up
const YAW: [f32; NUM_MOTORS] = [-1.0, 1.0, -1.0, 1.0]; // CW motors speed up

#[derive(Debug, Clone, Copy)]
pub struct MixerConfig {
    /// Distance from the centre to each motor (m).
    pub arm_length: f32,
    /// Thrust of one motor at command 1.0 (N).
    pub max_thrust: f32,
    /// Yaw reaction torque per unit of thrust (m).
    pub yaw_coeff: f32,
}

impl Default for MixerConfig {
    /// Roughly a 450 mm class quadcopter of about 1.2 kg.
    fn default() -> Self {
        Self {
            arm_length: 0.225,
            max_thrust: 6.0,
            yaw_coeff: 0.016,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Mixer {
    config: MixerConfig,
}

impl Mixer {
    pub fn new(config: MixerConfig) -> Self {
        assert!(
            config.arm_length > 0.0 && config.max_thrust > 0.0 && config.yaw_coeff > 0.0,
            "mixer parameters must be positive"
        );
        Self { config }
    }

    /// Each motor sits at (±d, ±d).
    fn motor_offset(&self) -> f32 {
        self.config.arm_length * FRAC_1_SQRT_2
    }

    /// Convert a throttle (mean motor command, 0..1) and a body torque (N·m)
    /// into four motor commands in [0, 1].
    ///
    /// If the request does not fit, the torques are scaled down together and
    /// the collective throttle is shifted: attitude authority is kept at the
    /// expense of total thrust.
    pub fn mix(&self, throttle: f32, torque: Vec3) -> MotorCommands {
        if !throttle.is_finite() || !torque.is_finite() {
            return [0.0; NUM_MOTORS];
        }
        let c = self.config;
        let d = self.motor_offset();

        // Differential command of each motor, relative to the collective.
        let delta: [f32; NUM_MOTORS] = core::array::from_fn(|i| {
            let thrust_delta = ROLL[i] * torque.x / (4.0 * d)
                + PITCH[i] * torque.y / (4.0 * d)
                + YAW[i] * torque.z / (4.0 * c.yaw_coeff);
            thrust_delta / c.max_thrust
        });
        if delta.iter().any(|x| !x.is_finite()) {
            return [0.0; NUM_MOTORS];
        }

        // 1. If the spread is larger than the whole range, scale all torques down.
        let max = delta.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        let min = delta.iter().copied().fold(f32::INFINITY, f32::min);
        let span = max - min;
        let scale = if span > 1.0 { 1.0 / span } else { 1.0 };

        // 2. Shift the collective so that every motor fits into [0, 1].
        let lowest = -min * scale;
        let highest = 1.0 - max * scale;
        let collective = throttle.max(lowest).min(highest);

        core::array::from_fn(|i| (collective + delta[i] * scale).clamp(0.0, 1.0))
    }

    /// Forward model: motor commands -> (total thrust in N, body torque in N·m).
    /// Assumes instantaneous motors with thrust proportional to the command.
    pub fn forward(&self, commands: &MotorCommands) -> (f32, Vec3) {
        let c = self.config;
        let d = self.motor_offset();
        let thrust: [f32; NUM_MOTORS] = core::array::from_fn(|i| commands[i] * c.max_thrust);

        let total: f32 = thrust.iter().sum();
        let torque = Vec3::new(
            d * dot(&ROLL, &thrust),
            d * dot(&PITCH, &thrust),
            c.yaw_coeff * dot(&YAW, &thrust),
        );
        (total, torque)
    }
}

fn dot(a: &[f32; NUM_MOTORS], b: &[f32; NUM_MOTORS]) -> f32 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mixer() -> Mixer {
        Mixer::new(MixerConfig::default())
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn pure_throttle_drives_all_motors_equally() {
        let u = mixer().mix(0.6, Vec3::ZERO);
        for m in u {
            assert!(close(m, 0.6, 1e-6));
        }
    }

    #[test]
    fn positive_roll_torque_speeds_up_left_motors() {
        let u = mixer().mix(0.5, Vec3::new(0.2, 0.0, 0.0));
        // M2 and M3 are on the left, M1 and M4 on the right.
        assert!(u[1] > 0.5 && u[2] > 0.5);
        assert!(u[0] < 0.5 && u[3] < 0.5);
        assert!(close(u.iter().sum::<f32>(), 2.0, 1e-5)); // collective unchanged
    }

    #[test]
    fn positive_pitch_torque_speeds_up_rear_motors() {
        let u = mixer().mix(0.5, Vec3::new(0.0, 0.2, 0.0));
        assert!(u[2] > 0.5 && u[3] > 0.5);
        assert!(u[0] < 0.5 && u[1] < 0.5);
    }

    #[test]
    fn positive_yaw_torque_speeds_up_cw_motors() {
        let u = mixer().mix(0.5, Vec3::new(0.0, 0.0, 0.02));
        // M2 and M4 spin clockwise.
        assert!(u[1] > 0.5 && u[3] > 0.5);
        assert!(u[0] < 0.5 && u[2] < 0.5);
    }

    #[test]
    fn mix_then_forward_returns_the_command() {
        let m = mixer();
        let requested = Vec3::new(0.1, -0.05, 0.02);
        let u = m.mix(0.5, requested);
        let (thrust, torque) = m.forward(&u);
        assert!(close(thrust, 4.0 * 0.5 * 6.0, 1e-3));
        assert!(close(torque.x, requested.x, 1e-4));
        assert!(close(torque.y, requested.y, 1e-4));
        assert!(close(torque.z, requested.z, 1e-4));
    }

    #[test]
    fn outputs_stay_in_range_for_extreme_inputs() {
        let m = mixer();
        let values = [-1000.0, -5.0, -0.3, 0.0, 0.3, 5.0, 1000.0];
        for &throttle in &[-1.0, 0.0, 0.5, 1.0, 2.0] {
            for &x in &values {
                for &y in &values {
                    for &z in &values {
                        let u = m.mix(throttle, Vec3::new(x, y, z));
                        assert!(u.iter().all(|&c| (0.0..=1.0).contains(&c)));
                    }
                }
            }
        }
    }

    #[test]
    fn saturation_sacrifices_thrust_not_torque() {
        let m = mixer();

        // Near full throttle there is no room above: the collective is lowered
        // so that the requested torques are still delivered exactly.
        let requested = Vec3::new(1.0, 0.5, 0.0);
        let u = m.mix(0.98, requested);
        let (thrust, torque) = m.forward(&u);
        assert!(close(torque.x, requested.x, 1e-3));
        assert!(close(torque.y, requested.y, 1e-3));
        assert!(thrust < 4.0 * 0.98 * 6.0 - 1.0);

        // A request larger than the motors can ever deliver is scaled down,
        // but keeps its direction: roll stays roll.
        let (_, torque) = m.forward(&m.mix(0.9, Vec3::new(2.0, 0.0, 0.0)));
        assert!(torque.x > 1.8 && torque.x < 2.0);
        assert!(torque.y.abs() < 1e-3 && torque.z.abs() < 1e-3);
    }

    #[test]
    fn non_finite_inputs_stop_the_motors() {
        let m = mixer();
        assert_eq!(m.mix(f32::NAN, Vec3::ZERO), [0.0; NUM_MOTORS]);
        assert_eq!(m.mix(0.5, Vec3::new(f32::NAN, 0.0, 0.0)), [0.0; NUM_MOTORS]);
        assert_eq!(
            m.mix(0.5, Vec3::new(0.0, f32::INFINITY, 0.0)),
            [0.0; NUM_MOTORS]
        );
    }
}
