use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_control::limiter::{Limiter, LimiterConfig};
use fc_control::mixer::{Mixer, MixerConfig, NUM_MOTORS};
use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::{Attitude, Vec3};
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_safety::{FailsafeReason, FlightMode, SafetyConfig, SafetySupervisor};
use fc_sensors::imu::{Imu, ImuConfig, ImuSample};
use fc_sensors::validation::{ImuValidator, ImuValidatorConfig};
use fc_sim::{QuadParams, QuadSim};

struct Outcome {
    sim: QuadSim,
    mode: FlightMode,
    reason: Option<FailsafeReason>,
    failsafe_step: Option<usize>,
    disarm_step: Option<usize>,
    last_motors: [f32; NUM_MOTORS],
}

/// Whole chain with safety supervision. `inject` may corrupt each raw sample.
fn run(seconds: f32, inject: impl Fn(usize, &mut ImuSample)) -> Outcome {
    let mut sim = QuadSim::new(QuadParams::default());
    sim.attitude = Attitude {
        roll: 0.2,
        pitch: -0.1,
        yaw: 0.0,
    };
    let mut imu = Imu::new(ImuConfig::default(), 42);
    let mut validator = ImuValidator::new(ImuValidatorConfig::default());
    let mut supervisor = SafetySupervisor::new(SafetyConfig::default());
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());
    let limiter = Limiter::new(LimiterConfig::default());
    let mixer = Mixer::new(MixerConfig::default());

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);

    let mut failsafe_step = None;
    let mut disarm_step = None;
    let mut motors = [0.0; NUM_MOTORS];

    for step in 0..(seconds * CONTROL_RATE_HZ) as usize {
        let mut raw = imu.measure(sim.attitude, sim.rates);
        inject(step, &mut raw);

        let validation = validator.check(&raw);
        let clean = supervisor.process_sample(&raw, &validation);
        let att = if supervisor.accel_trusted() {
            est.update(&clean, DT)
        } else {
            est.predict(clean.gyro, DT)
        };
        supervisor.check_attitude(att);

        let cmd = supervisor.command(Attitude::default(), 0.5);
        if !cmd.motors_enabled {
            ctrl.reset();
        }
        let torque_cmd = ctrl.update(cmd.setpoint, att, clean.gyro, DT);
        let (thr, torque) = limiter.apply(cmd.throttle, torque_cmd);
        motors = if cmd.motors_enabled {
            mixer.mix(thr, torque)
        } else {
            [0.0; NUM_MOTORS]
        };
        assert!(
            motors
                .iter()
                .all(|m| m.is_finite() && (0.0..=1.0).contains(m))
        );

        let (_thrust, applied) = mixer.forward(&motors);
        sim.step(applied, DT);

        if supervisor.mode() == FlightMode::Failsafe && failsafe_step.is_none() {
            failsafe_step = Some(step);
        }
        if supervisor.mode() == FlightMode::Disarmed && disarm_step.is_none() {
            disarm_step = Some(step);
        }
    }

    Outcome {
        sim,
        mode: supervisor.mode(),
        reason: supervisor.failsafe_reason(),
        failsafe_step,
        disarm_step,
        last_motors: motors,
    }
}

#[test]
fn short_sensor_glitch_is_survived() {
    let out = run(4.0, |step, raw| {
        if (2000..=2002).contains(&step) {
            raw.gyro.x = f32::NAN;
        }
    });
    assert_eq!(out.mode, FlightMode::Normal);
    assert_eq!(out.failsafe_step, None);
    assert!(out.sim.attitude.roll.abs() < 0.02);
    assert!(out.sim.attitude.pitch.abs() < 0.02);
}

#[test]
fn dead_gyro_ends_in_failsafe_then_disarm() {
    let out = run(5.0, |step, raw| {
        if step >= 1000 {
            raw.gyro = Vec3::new(f32::NAN, f32::NAN, f32::NAN);
        }
    });
    // 50 faulty samples starting at step 1000 -> failsafe at step 1049,
    // then 3000 ticks later the motors are switched off.
    assert_eq!(out.failsafe_step, Some(1049));
    assert_eq!(out.reason, Some(FailsafeReason::GyroTimeout));
    assert_eq!(out.disarm_step, Some(4049));
    assert_eq!(out.mode, FlightMode::Disarmed);
    assert_eq!(out.last_motors, [0.0; NUM_MOTORS]);
}

#[test]
fn dead_accelerometer_is_tolerated_for_a_while() {
    let out = run(3.0, |step, raw| {
        if step >= 1000 {
            raw.accel = Vec3::new(f32::NAN, f32::NAN, f32::NAN);
        }
    });
    // 500 faulty samples starting at step 1000 -> failsafe at step 1499.
    assert_eq!(out.failsafe_step, Some(1499));
    assert_eq!(out.reason, Some(FailsafeReason::AccelTimeout));
    assert_eq!(out.mode, FlightMode::Failsafe);
    // The gyro still works, so the drone stays stable.
    assert!(out.sim.attitude.roll.abs() < 0.05);
    assert!(out.sim.attitude.pitch.abs() < 0.05);
}
