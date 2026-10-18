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
use fc_telemetry::{TelemetryLogger, TelemetryRecord};
use std::io::{self, Write};

/// A reproducible test flight, optionally with injected sensor faults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    /// Start tilted, recover to level, then follow a 10 degree roll command.
    Recovery,
    /// 3 ms of NaN on the gyro at t = 2 s.
    GyroGlitch,
    /// Gyro dead from t = 1 s: Degraded -> Failsafe -> Disarmed.
    GyroDead,
    /// Accelerometer dead from t = 1 s: Degraded -> Failsafe.
    AccelDead,
}

impl Scenario {
    pub const ALL: [Scenario; 4] = [
        Scenario::Recovery,
        Scenario::GyroGlitch,
        Scenario::GyroDead,
        Scenario::AccelDead,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Scenario::Recovery => "recovery",
            Scenario::GyroGlitch => "gyro-glitch",
            Scenario::GyroDead => "gyro-dead",
            Scenario::AccelDead => "accel-dead",
        }
    }

    pub fn from_name(name: &str) -> Option<Scenario> {
        Scenario::ALL.into_iter().find(|s| s.name() == name)
    }

    fn duration_s(self) -> f32 {
        match self {
            Scenario::Recovery => 8.0,
            Scenario::GyroGlitch => 5.0,
            Scenario::GyroDead => 6.0,
            Scenario::AccelDead => 3.0,
        }
    }

    fn initial_attitude(self) -> Attitude {
        match self {
            Scenario::Recovery => Attitude {
                roll: 0.35,
                pitch: -0.20,
                yaw: 0.0,
            },
            _ => Attitude {
                roll: 0.2,
                pitch: -0.1,
                yaw: 0.0,
            },
        }
    }

    /// What the pilot (or mission) asks for at time `t`.
    fn setpoint(self, t: f32) -> Attitude {
        match self {
            Scenario::Recovery if t >= 4.0 => Attitude {
                roll: 0.1745, // 10 degrees
                ..Attitude::default()
            },
            _ => Attitude::default(),
        }
    }

    /// Corrupt the raw sensor sample to simulate a hardware fault.
    fn inject(self, step: usize, raw: &mut ImuSample) {
        let nan = Vec3::new(f32::NAN, f32::NAN, f32::NAN);
        match self {
            Scenario::Recovery => {}
            Scenario::GyroGlitch => {
                if (2000..=2002).contains(&step) {
                    raw.gyro = nan;
                }
            }
            Scenario::GyroDead => {
                if step >= 1000 {
                    raw.gyro = nan;
                }
            }
            Scenario::AccelDead => {
                if step >= 1000 {
                    raw.accel = nan;
                }
            }
        }
    }
}

/// How a scenario ended.
#[derive(Debug, Clone, Copy)]
pub struct Summary {
    /// True attitude at the end (ground truth from the simulator).
    pub final_attitude: Attitude,
    pub final_mode: FlightMode,
    pub failsafe_reason: Option<FailsafeReason>,
}

fn mode_label(mode: FlightMode) -> &'static str {
    match mode {
        FlightMode::Normal => "Normal",
        FlightMode::Degraded => "Degraded",
        FlightMode::Failsafe => "Failsafe",
        FlightMode::Disarmed => "Disarmed",
    }
}

/// Fly one scenario with the complete chain and log every tick.
pub fn run<W: Write>(scenario: Scenario, logger: &mut TelemetryLogger<W>) -> io::Result<Summary> {
    let mut sim = QuadSim::new(QuadParams::default());
    sim.attitude = scenario.initial_attitude();

    let mut imu = Imu::new(ImuConfig::default(), 42);
    let mut validator = ImuValidator::new(ImuValidatorConfig::default());
    let mut supervisor = SafetySupervisor::new(SafetyConfig::default());
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());
    let limiter = Limiter::new(LimiterConfig::default());
    let mixer = Mixer::new(MixerConfig::default());

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);

    let steps = (scenario.duration_s() * CONTROL_RATE_HZ) as usize;
    for step in 0..steps {
        let t = step as f32 * DT;

        // Sense: ground truth -> IMU -> (fault injection) -> validation.
        let truth = sim.attitude;
        let mut raw = imu.measure(sim.attitude, sim.rates);
        scenario.inject(step, &mut raw);
        let validation = validator.check(&raw);

        // Supervise and estimate. Untrusted accelerometer data is not used.
        let clean = supervisor.process_sample(&raw, &validation);
        let att = if supervisor.accel_trusted() {
            est.update(&clean, DT)
        } else {
            est.predict(clean.gyro, DT)
        };
        supervisor.check_attitude(att);

        // Control: setpoint -> PID -> limiter -> mixer.
        let cmd = supervisor.command(scenario.setpoint(t), 0.5);
        if !cmd.motors_enabled {
            ctrl.reset();
        }
        let torque_cmd = ctrl.update(cmd.setpoint, att, clean.gyro, DT);
        let (throttle, torque) = limiter.apply(cmd.throttle, torque_cmd);
        let motors = if cmd.motors_enabled {
            mixer.mix(throttle, torque)
        } else {
            [0.0; NUM_MOTORS]
        };

        // Record what happened this tick, then advance the world.
        logger.log(&TelemetryRecord {
            time_s: t,
            true_attitude: truth,
            estimated_attitude: att,
            setpoint: cmd.setpoint,
            gyro: raw.gyro,
            mode: mode_label(supervisor.mode()),
            gyro_fault: validation.gyro.is_some(),
            accel_fault: validation.accel.is_some(),
            throttle,
            motors,
        })?;

        let (_thrust, applied) = mixer.forward(&motors);
        sim.step(applied, DT);
    }

    Ok(Summary {
        final_attitude: sim.attitude,
        final_mode: supervisor.mode(),
        failsafe_reason: supervisor.failsafe_reason(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_telemetry::CSV_HEADER;

    fn run_to_string(scenario: Scenario, decimation: u32) -> (Summary, String) {
        let mut logger = TelemetryLogger::new(Vec::new(), decimation).unwrap();
        let summary = run(scenario, &mut logger).unwrap();
        let text = String::from_utf8(logger.into_inner()).unwrap();
        (summary, text)
    }

    #[test]
    fn recovery_follows_the_roll_command_in_normal_mode() {
        let (s, _) = run_to_string(Scenario::Recovery, 10);
        assert_eq!(s.final_mode, FlightMode::Normal);
        assert!((s.final_attitude.roll - 0.1745).abs() < 0.02);
        assert!(s.final_attitude.pitch.abs() < 0.02);
    }

    #[test]
    fn short_gyro_glitch_is_survived() {
        let (s, _) = run_to_string(Scenario::GyroGlitch, 10);
        assert_eq!(s.final_mode, FlightMode::Normal);
        assert_eq!(s.failsafe_reason, None);
        assert!(s.final_attitude.roll.abs() < 0.03);
        assert!(s.final_attitude.pitch.abs() < 0.03);
    }

    #[test]
    fn dead_gyro_ends_disarmed_and_the_log_shows_every_mode() {
        let (s, text) = run_to_string(Scenario::GyroDead, 10);
        assert_eq!(s.final_mode, FlightMode::Disarmed);
        assert_eq!(s.failsafe_reason, Some(FailsafeReason::GyroTimeout));
        for mode in [",Normal,", ",Degraded,", ",Failsafe,", ",Disarmed,"] {
            assert!(text.contains(mode), "log never shows {mode}");
        }
    }

    #[test]
    fn dead_accelerometer_ends_in_failsafe() {
        let (s, _) = run_to_string(Scenario::AccelDead, 10);
        assert_eq!(s.final_mode, FlightMode::Failsafe);
        assert_eq!(s.failsafe_reason, Some(FailsafeReason::AccelTimeout));
    }

    #[test]
    fn telemetry_is_complete_and_deterministic() {
        let (_, a) = run_to_string(Scenario::GyroGlitch, 1);
        let (_, b) = run_to_string(Scenario::GyroGlitch, 1);
        assert_eq!(a, b);

        // 5 s at 1 kHz, every tick logged: 5000 rows + header.
        assert_eq!(a.lines().count(), 5001);
        let columns = CSV_HEADER.split(',').count();
        for line in a.lines() {
            assert_eq!(line.split(',').count(), columns);
        }
        // The injected NaN burst is visible in the raw gyro column.
        assert!(a.contains("NaN"));
    }
}
