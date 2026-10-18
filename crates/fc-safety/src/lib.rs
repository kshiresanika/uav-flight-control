use fc_core::config::{CONTROL_RATE_HZ, GRAVITY};
use fc_core::math::{Attitude, Vec3};
use fc_sensors::imu::ImuSample;
use fc_sensors::validation::ImuValidation;

/// Why the supervisor entered failsafe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeReason {
    /// Gyro faulty for longer than its timeout.
    GyroTimeout,
    /// Accelerometer faulty for longer than its timeout.
    AccelTimeout,
    /// Roll or pitch beyond the maximum tilt.
    ExcessiveTilt,
    /// The attitude estimate itself is NaN or infinite.
    InvalidAttitude,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    Normal,
    Degraded,
    Failsafe,
    Disarmed,
}

#[derive(Debug, Clone, Copy)]
pub struct SafetyConfig {
    /// Gyro may be faulty this long before failsafe (s).
    pub gyro_timeout_s: f32,
    /// Accelerometer may be faulty this long before failsafe (s).
    pub accel_timeout_s: f32,
    /// Consecutive healthy time needed to return from Degraded to Normal (s).
    pub recovery_s: f32,
    /// Time in Failsafe before the motors are switched off (s).
    pub disarm_after_s: f32,
    /// Maximum roll/pitch magnitude before failsafe (rad).
    pub max_tilt: f32,
    /// Throttle limit in Failsafe (0..1), below hover for a gentle descent.
    pub failsafe_throttle: f32,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            gyro_timeout_s: 0.05,
            accel_timeout_s: 0.5,
            recovery_s: 0.1,
            disarm_after_s: 3.0,
            max_tilt: core::f32::consts::FRAC_PI_3, // 60 degrees
            failsafe_throttle: 0.42,
        }
    }
}

/// What the rest of the flight code must do this tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightCommand {
    pub setpoint: Attitude,
    pub throttle: f32,
    pub motors_enabled: bool,
}

/// Safety supervisor: sensor-fault handling and the flight-mode state machine.
#[derive(Debug, Clone)]
pub struct SafetySupervisor {
    config: SafetyConfig,
    gyro_timeout: u32,
    accel_timeout: u32,
    recovery: u32,
    disarm_after: u32,

    mode: FlightMode,
    reason: Option<FailsafeReason>,
    last_good: ImuSample,
    /// Consecutive faulty samples per sensor.
    gyro_bad: u32,
    accel_bad: u32,
    /// Consecutive healthy ticks while Degraded.
    good_run: u32,
    failsafe_ticks: u32,
}

/// Seconds -> control ticks (at least 1).
fn to_ticks(seconds: f32) -> u32 {
    ((seconds * CONTROL_RATE_HZ).round() as u32).max(1)
}

/// Safe starting value if the very first sample is already bad:
/// at rest, level (gravity on +z).
fn rest_sample() -> ImuSample {
    ImuSample {
        gyro: Vec3::ZERO,
        accel: Vec3::new(0.0, 0.0, GRAVITY),
    }
}

impl SafetySupervisor {
    pub fn new(config: SafetyConfig) -> Self {
        assert!(
            config.gyro_timeout_s > 0.0
                && config.accel_timeout_s > 0.0
                && config.recovery_s > 0.0
                && config.disarm_after_s > 0.0,
            "safety timeouts must be positive"
        );
        assert!(
            config.max_tilt > 0.0 && config.max_tilt < core::f32::consts::PI,
            "max_tilt must be in (0, pi)"
        );
        assert!(
            (0.0..=1.0).contains(&config.failsafe_throttle),
            "failsafe_throttle must be in [0, 1]"
        );
        Self {
            config,
            gyro_timeout: to_ticks(config.gyro_timeout_s),
            accel_timeout: to_ticks(config.accel_timeout_s),
            recovery: to_ticks(config.recovery_s),
            disarm_after: to_ticks(config.disarm_after_s),
            mode: FlightMode::Normal,
            reason: None,
            last_good: rest_sample(),
            gyro_bad: 0,
            accel_bad: 0,
            good_run: 0,
            failsafe_ticks: 0,
        }
    }

    pub fn mode(&self) -> FlightMode {
        self.mode
    }

    /// Why failsafe was entered (stays set in Disarmed, cleared by `rearm`).
    pub fn failsafe_reason(&self) -> Option<FailsafeReason> {
        self.reason
    }

    fn enter_failsafe(&mut self, reason: FailsafeReason) {
        self.mode = FlightMode::Failsafe;
        self.reason = Some(reason);
        self.failsafe_ticks = 0;
    }

    /// Step 1 of every tick: handle one validated IMU sample.
    /// Returns the sample the estimator should use: bad channels are replaced
    /// by the last good value. The result is always finite.
    pub fn process_sample(&mut self, raw: &ImuSample, validation: &ImuValidation) -> ImuSample {
        // Defence in depth: non-finite data is a fault even if the validator
        // did not flag it.
        let gyro_ok = validation.gyro.is_none() && raw.gyro.is_finite();
        let accel_ok = validation.accel.is_none() && raw.accel.is_finite();

        self.gyro_bad = if gyro_ok {
            0
        } else {
            self.gyro_bad.saturating_add(1)
        };
        self.accel_bad = if accel_ok {
            0
        } else {
            self.accel_bad.saturating_add(1)
        };

        if gyro_ok {
            self.last_good.gyro = raw.gyro;
        }
        if accel_ok {
            self.last_good.accel = raw.accel;
        }
        let clean = self.last_good;

        match self.mode {
            FlightMode::Normal | FlightMode::Degraded => {
                if self.gyro_bad >= self.gyro_timeout {
                    self.enter_failsafe(FailsafeReason::GyroTimeout);
                } else if self.accel_bad >= self.accel_timeout {
                    self.enter_failsafe(FailsafeReason::AccelTimeout);
                } else if !gyro_ok || !accel_ok {
                    self.mode = FlightMode::Degraded;
                    self.good_run = 0;
                } else if self.mode == FlightMode::Degraded {
                    self.good_run += 1;
                    if self.good_run >= self.recovery {
                        self.mode = FlightMode::Normal;
                    }
                }
            }
            FlightMode::Failsafe => {
                self.failsafe_ticks += 1;
                if self.failsafe_ticks >= self.disarm_after {
                    self.mode = FlightMode::Disarmed;
                }
            }
            FlightMode::Disarmed => {}
        }
        clean
    }

    /// Step 2 of every tick: check the estimated attitude.
    pub fn check_attitude(&mut self, attitude: Attitude) {
        if !matches!(self.mode, FlightMode::Normal | FlightMode::Degraded) {
            return;
        }
        let finite =
            attitude.roll.is_finite() && attitude.pitch.is_finite() && attitude.yaw.is_finite();
        if !finite {
            self.enter_failsafe(FailsafeReason::InvalidAttitude);
        } else if attitude.roll.abs() > self.config.max_tilt
            || attitude.pitch.abs() > self.config.max_tilt
        {
            self.enter_failsafe(FailsafeReason::ExcessiveTilt);
        }
    }

    /// Step 3 of every tick: apply the mode to the pilot/mission request.
    pub fn command(&self, setpoint: Attitude, throttle: f32) -> FlightCommand {
        match self.mode {
            FlightMode::Normal | FlightMode::Degraded => FlightCommand {
                setpoint,
                throttle,
                motors_enabled: true,
            },
            FlightMode::Failsafe => FlightCommand {
                // Level out, keep the requested heading, descend gently.
                setpoint: Attitude {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: setpoint.yaw,
                },
                throttle: throttle.min(self.config.failsafe_throttle),
                motors_enabled: true,
            },
            FlightMode::Disarmed => FlightCommand {
                setpoint: Attitude::default(),
                throttle: 0.0,
                motors_enabled: false,
            },
        }
    }

    /// Leave Disarmed. Only allowed from Disarmed; returns whether it happened.
    /// A real system would also require the vehicle to be on the ground
    /// and an explicit pilot action.
    pub fn rearm(&mut self) -> bool {
        if self.mode != FlightMode::Disarmed {
            return false;
        }
        self.mode = FlightMode::Normal;
        self.reason = None;
        self.last_good = rest_sample();
        self.gyro_bad = 0;
        self.accel_bad = 0;
        self.good_run = 0;
        self.failsafe_ticks = 0;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fc_sensors::validation::SensorFault;

    fn supervisor() -> SafetySupervisor {
        SafetySupervisor::new(SafetyConfig::default())
    }

    fn sample(gyro_x: f32) -> ImuSample {
        ImuSample {
            gyro: Vec3::new(gyro_x, 0.0, 0.0),
            accel: Vec3::new(0.0, 0.0, GRAVITY),
        }
    }

    fn ok() -> ImuValidation {
        ImuValidation {
            gyro: None,
            accel: None,
        }
    }

    fn gyro_fault() -> ImuValidation {
        ImuValidation {
            gyro: Some(SensorFault::NonFinite),
            accel: None,
        }
    }

    fn accel_fault() -> ImuValidation {
        ImuValidation {
            gyro: None,
            accel: Some(SensorFault::Jump),
        }
    }

    /// Drive the supervisor into Failsafe through a dead gyro.
    fn enter_failsafe_by_gyro(s: &mut SafetySupervisor) {
        for _ in 0..50 {
            s.process_sample(&sample(f32::NAN), &gyro_fault());
        }
        assert_eq!(s.mode(), FlightMode::Failsafe);
    }

    #[test]
    fn normal_passes_samples_and_commands_through() {
        let mut s = supervisor();
        for i in 0..200 {
            let raw = sample(0.001 * i as f32);
            assert_eq!(s.process_sample(&raw, &ok()), raw);
        }
        assert_eq!(s.mode(), FlightMode::Normal);

        let setpoint = Attitude {
            roll: 0.1,
            pitch: -0.1,
            yaw: 0.5,
        };
        let cmd = s.command(setpoint, 0.6);
        assert_eq!(cmd.setpoint, setpoint);
        assert_eq!(cmd.throttle, 0.6);
        assert!(cmd.motors_enabled);
    }

    #[test]
    fn fault_degrades_and_holds_last_good_value() {
        let mut s = supervisor();
        let good = sample(0.5);
        s.process_sample(&good, &ok());

        let clean = s.process_sample(&sample(f32::NAN), &gyro_fault());
        assert_eq!(clean.gyro, good.gyro);
        assert_eq!(s.mode(), FlightMode::Degraded);
    }

    #[test]
    fn recovery_needs_a_run_of_healthy_samples() {
        let mut s = supervisor();
        s.process_sample(&sample(f32::NAN), &gyro_fault());
        assert_eq!(s.mode(), FlightMode::Degraded);

        for _ in 0..99 {
            s.process_sample(&sample(0.0), &ok());
            assert_eq!(s.mode(), FlightMode::Degraded);
        }
        s.process_sample(&sample(0.0), &ok());
        assert_eq!(s.mode(), FlightMode::Normal);
    }

    #[test]
    fn a_new_fault_restarts_the_recovery_count() {
        let mut s = supervisor();
        s.process_sample(&sample(f32::NAN), &gyro_fault());
        for _ in 0..99 {
            s.process_sample(&sample(0.0), &ok());
        }
        // One sample short of recovery: another fault arrives.
        s.process_sample(&sample(f32::NAN), &gyro_fault());
        for _ in 0..99 {
            s.process_sample(&sample(0.0), &ok());
            assert_eq!(s.mode(), FlightMode::Degraded);
        }
        s.process_sample(&sample(0.0), &ok());
        assert_eq!(s.mode(), FlightMode::Normal);
    }

    #[test]
    fn persistent_gyro_fault_triggers_failsafe_at_timeout() {
        let mut s = supervisor();
        for _ in 0..49 {
            s.process_sample(&sample(f32::NAN), &gyro_fault());
            assert_eq!(s.mode(), FlightMode::Degraded);
        }
        s.process_sample(&sample(f32::NAN), &gyro_fault());
        assert_eq!(s.mode(), FlightMode::Failsafe);
        assert_eq!(s.failsafe_reason(), Some(FailsafeReason::GyroTimeout));
    }

    #[test]
    fn accel_is_tolerated_longer_than_gyro() {
        let mut s = supervisor();
        for _ in 0..499 {
            s.process_sample(&sample(0.0), &accel_fault());
            assert_eq!(s.mode(), FlightMode::Degraded);
        }
        s.process_sample(&sample(0.0), &accel_fault());
        assert_eq!(s.mode(), FlightMode::Failsafe);
        assert_eq!(s.failsafe_reason(), Some(FailsafeReason::AccelTimeout));
    }

    #[test]
    fn failsafe_is_latched() {
        let mut s = supervisor();
        enter_failsafe_by_gyro(&mut s);
        for _ in 0..200 {
            s.process_sample(&sample(0.0), &ok());
            assert_eq!(s.mode(), FlightMode::Failsafe);
        }
        assert_eq!(s.failsafe_reason(), Some(FailsafeReason::GyroTimeout));
    }

    #[test]
    fn failsafe_ends_in_disarmed() {
        let mut s = supervisor();
        enter_failsafe_by_gyro(&mut s);
        for _ in 0..2999 {
            s.process_sample(&sample(0.0), &ok());
            assert_eq!(s.mode(), FlightMode::Failsafe);
        }
        s.process_sample(&sample(0.0), &ok());
        assert_eq!(s.mode(), FlightMode::Disarmed);
    }

    #[test]
    fn excessive_tilt_triggers_failsafe() {
        let mut s = supervisor();
        s.check_attitude(Attitude {
            roll: 0.5,
            ..Attitude::default()
        });
        assert_eq!(s.mode(), FlightMode::Normal);

        s.check_attitude(Attitude {
            roll: 1.2, // about 69 degrees
            ..Attitude::default()
        });
        assert_eq!(s.mode(), FlightMode::Failsafe);
        assert_eq!(s.failsafe_reason(), Some(FailsafeReason::ExcessiveTilt));
    }

    #[test]
    fn invalid_attitude_triggers_failsafe() {
        let mut s = supervisor();
        s.check_attitude(Attitude {
            roll: f32::NAN,
            ..Attitude::default()
        });
        assert_eq!(s.mode(), FlightMode::Failsafe);
        assert_eq!(s.failsafe_reason(), Some(FailsafeReason::InvalidAttitude));
    }

    #[test]
    fn failsafe_command_levels_and_limits_throttle() {
        let mut s = supervisor();
        enter_failsafe_by_gyro(&mut s);

        let setpoint = Attitude {
            roll: 0.3,
            pitch: 0.2,
            yaw: 0.4,
        };
        let cmd = s.command(setpoint, 0.8);
        assert_eq!(cmd.setpoint.roll, 0.0);
        assert_eq!(cmd.setpoint.pitch, 0.0);
        assert_eq!(cmd.setpoint.yaw, 0.4);
        assert_eq!(cmd.throttle, SafetyConfig::default().failsafe_throttle);
        assert!(cmd.motors_enabled);

        // A request that is already lower is kept.
        assert_eq!(s.command(setpoint, 0.2).throttle, 0.2);
    }

    #[test]
    fn disarmed_command_stops_motors() {
        let mut s = supervisor();
        enter_failsafe_by_gyro(&mut s);
        for _ in 0..3000 {
            s.process_sample(&sample(0.0), &ok());
        }
        assert_eq!(s.mode(), FlightMode::Disarmed);

        let cmd = s.command(Attitude::default(), 0.9);
        assert_eq!(cmd.throttle, 0.0);
        assert!(!cmd.motors_enabled);
    }

    #[test]
    fn rearm_only_works_when_disarmed() {
        let mut s = supervisor();
        assert!(!s.rearm()); // Normal
        enter_failsafe_by_gyro(&mut s);
        assert!(!s.rearm()); // Failsafe

        for _ in 0..3000 {
            s.process_sample(&sample(0.0), &ok());
        }
        assert_eq!(s.mode(), FlightMode::Disarmed);
        assert!(s.rearm());
        assert_eq!(s.mode(), FlightMode::Normal);
        assert_eq!(s.failsafe_reason(), None);
        assert!(s.command(Attitude::default(), 0.5).motors_enabled);
    }

    #[test]
    fn substituted_samples_are_always_finite() {
        // First sample already bad: falls back to the safe rest value.
        let mut s = supervisor();
        let clean = s.process_sample(&sample(f32::NAN), &gyro_fault());
        assert!(clean.gyro.is_finite() && clean.accel.is_finite());

        // Even if the validator wrongly says "ok", NaN never gets through.
        let mut s = supervisor();
        let clean = s.process_sample(&sample(f32::NAN), &ok());
        assert!(clean.gyro.is_finite());
        assert_eq!(s.mode(), FlightMode::Degraded);
    }
}
