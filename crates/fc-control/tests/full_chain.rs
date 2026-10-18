use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_control::limiter::{Limiter, LimiterConfig};
use fc_control::mixer::{Mixer, MixerConfig};
use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Attitude;
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_sensors::imu::{Imu, ImuConfig};
use fc_sim::{QuadParams, QuadSim};

/// Whole chain: sim -> IMU -> estimator -> PID -> limiter -> mixer -> sim.
fn run_full_chain(
    initial: Attitude,
    setpoint: Attitude,
    imu_config: ImuConfig,
    throttle: f32,
    seconds: f32,
) -> QuadSim {
    let mut sim = QuadSim::new(QuadParams::default());
    sim.attitude = initial;
    let mut imu = Imu::new(imu_config, 42);
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());
    let limiter = Limiter::new(LimiterConfig::default());
    let mixer = Mixer::new(MixerConfig::default());

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);

    for _ in 0..(seconds * CONTROL_RATE_HZ) as usize {
        let sample = imu.measure(sim.attitude, sim.rates);
        let estimate = est.update(&sample, DT);
        let torque_cmd = ctrl.update(setpoint, estimate, sample.gyro, DT);
        let (thr, torque) = limiter.apply(throttle, torque_cmd);
        let motors = mixer.mix(thr, torque);
        assert!(motors.iter().all(|&m| (0.0..=1.0).contains(&m)));
        let (_thrust, applied) = mixer.forward(&motors);
        sim.step(applied, DT);
    }
    sim
}

#[test]
fn stabilizes_from_a_tilt_with_noisy_biased_sensors() {
    let initial = Attitude {
        roll: 0.3,
        pitch: -0.2,
        yaw: 0.0,
    };
    let sim = run_full_chain(initial, Attitude::default(), ImuConfig::default(), 0.5, 4.0);
    assert!(sim.attitude.roll.abs() < 0.03);
    assert!(sim.attitude.pitch.abs() < 0.03);
}

#[test]
fn stabilizes_even_when_motors_saturate() {
    // High throttle plus a large tilt: motors hit their limits during recovery.
    let initial = Attitude {
        roll: 0.5,
        pitch: 0.3,
        yaw: 0.0,
    };
    let sim = run_full_chain(initial, Attitude::default(), ImuConfig::ideal(), 0.9, 5.0);
    assert!(sim.attitude.roll.abs() < 0.02);
    assert!(sim.attitude.pitch.abs() < 0.02);
}

#[test]
fn follows_a_yaw_command_through_the_reaction_torque_limit() {
    let setpoint = Attitude {
        yaw: 0.3,
        ..Attitude::default()
    };
    let sim = run_full_chain(Attitude::default(), setpoint, ImuConfig::ideal(), 0.5, 6.0);
    assert!((sim.attitude.yaw - 0.3).abs() < 0.02);
}
