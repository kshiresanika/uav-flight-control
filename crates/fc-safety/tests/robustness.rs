use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_control::limiter::{Limiter, LimiterConfig};
use fc_control::mixer::{Mixer, MixerConfig, NUM_MOTORS};
use fc_core::config::{DT, GRAVITY};
use fc_core::math::{Attitude, Vec3};
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_safety::{FlightMode, SafetyConfig, SafetySupervisor};
use fc_sensors::imu::ImuSample;
use fc_sensors::rng::Rng;
use fc_sensors::validation::{ImuValidator, ImuValidatorConfig};

struct Stats {
    final_mode: FlightMode,
    last_motors: [f32; NUM_MOTORS],
    /// Hash of every motor command ever produced, to compare whole runs.
    checksum: u32,
}

/// Sensor -> validator -> supervisor -> estimator -> PID -> limiter -> mixer.
/// Asserts the safety invariants on every single tick.
fn fly(ticks: usize, mut sensor: impl FnMut(usize) -> ImuSample) -> Stats {
    let mut validator = ImuValidator::new(ImuValidatorConfig::default());
    let mut supervisor = SafetySupervisor::new(SafetyConfig::default());
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());
    let limiter = Limiter::new(LimiterConfig::default());
    let mixer = Mixer::new(MixerConfig::default());

    let mut motors = [0.0; NUM_MOTORS];
    let mut checksum = 0_u32;

    for tick in 0..ticks {
        let raw = sensor(tick);
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
        let (throttle, torque) = limiter.apply(cmd.throttle, torque_cmd);
        motors = if cmd.motors_enabled {
            mixer.mix(throttle, torque)
        } else {
            [0.0; NUM_MOTORS]
        };

        assert!(
            att.roll.is_finite() && att.pitch.is_finite() && att.yaw.is_finite(),
            "tick {tick}: attitude estimate is not finite"
        );
        assert!(
            motors
                .iter()
                .all(|m| m.is_finite() && (0.0..=1.0).contains(m)),
            "tick {tick}: invalid motor commands {motors:?}"
        );
        for m in motors {
            checksum = checksum.wrapping_mul(31).wrapping_add(m.to_bits());
        }
    }

    Stats {
        final_mode: supervisor.mode(),
        last_motors: motors,
        checksum,
    }
}

/// Mostly noise, but 2% NaN, 2% infinity and 2% huge values.
fn noisy(rng: &mut Rng, std: f32) -> f32 {
    let u = rng.uniform();
    if u < 0.02 {
        f32::NAN
    } else if u < 0.04 {
        f32::INFINITY
    } else if u < 0.06 {
        1.0e6 * rng.gaussian()
    } else {
        std * rng.gaussian()
    }
}

fn garbage_sample(rng: &mut Rng) -> ImuSample {
    ImuSample {
        gyro: Vec3::new(noisy(rng, 0.5), noisy(rng, 0.5), noisy(rng, 0.5)),
        accel: Vec3::new(noisy(rng, 2.0), noisy(rng, 2.0), GRAVITY + noisy(rng, 1.0)),
    }
}

#[test]
fn garbage_sensors_never_produce_invalid_motor_commands() {
    let mut rng = Rng::new(2024);
    let stats = fly(20_000, |_| garbage_sample(&mut rng));
    // The per-tick assertions inside `fly` are the real test; this is a sanity check.
    assert!(stats.last_motors.iter().all(|m| (0.0..=1.0).contains(m)));
}

#[test]
fn garbage_run_is_deterministic() {
    let run = || {
        let mut rng = Rng::new(7);
        fly(5000, |_| garbage_sample(&mut rng)).checksum
    };
    assert_eq!(run(), run());
}

#[test]
fn total_sensor_loss_ends_disarmed_with_motors_off() {
    let nan = Vec3::new(f32::NAN, f32::NAN, f32::NAN);
    // Every sample is NaN from the very first tick:
    // failsafe at tick 49, disarmed 3000 ticks later (tick 3049).
    let stats = fly(3100, |_| ImuSample {
        gyro: nan,
        accel: nan,
    });
    assert_eq!(stats.final_mode, FlightMode::Disarmed);
    assert_eq!(stats.last_motors, [0.0; NUM_MOTORS]);
}
