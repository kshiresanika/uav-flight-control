use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_control::limiter::{Limiter, LimiterConfig};
use fc_control::mixer::{Mixer, MixerConfig, NUM_MOTORS};
use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Attitude;
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_safety::{FlightMode, SafetyConfig, SafetySupervisor};
use fc_sensors::imu::{Imu, ImuConfig};
use fc_sensors::validation::{ImuValidator, ImuValidatorConfig};
use fc_sim::{QuadParams, QuadSim};

fn main() {
    let mut sim = QuadSim::new(QuadParams::default());
    sim.attitude = Attitude {
        roll: 0.35,
        pitch: -0.20,
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

    let mut previous_mode = FlightMode::Normal;
    let total_steps = (8.0 * CONTROL_RATE_HZ) as u32;
    println!("   t  | mode      | roll true |   M1    M2    M3    M4");

    for step in 0..total_steps {
        let t = step as f32 * DT;
        let mut raw = imu.measure(sim.attitude, sim.rates);

        // Fault injection: a 3 ms gyro glitch at 2.0 s,
        // then the gyro dies for good at 4.0 s.
        match step {
            2000..=2002 => raw.gyro.x = f32::NAN,
            4000.. => raw.gyro.x = f32::NAN,
            _ => {}
        }

        let validation = validator.check(&raw);
        let clean = supervisor.process_sample(&raw, &validation);
        let att = est.update(&clean, DT);
        supervisor.check_attitude(att);

        let mode = supervisor.mode();
        if mode != previous_mode {
            println!(
                "t = {t:.3} s  {previous_mode:?} -> {mode:?}   reason: {:?}",
                supervisor.failsafe_reason()
            );
            previous_mode = mode;
        }

        let cmd = supervisor.command(Attitude::default(), 0.5);
        if !cmd.motors_enabled {
            ctrl.reset();
        }
        let torque_cmd = ctrl.update(cmd.setpoint, att, clean.gyro, DT);
        let (thr, torque) = limiter.apply(cmd.throttle, torque_cmd);
        let motors = if cmd.motors_enabled {
            mixer.mix(thr, torque)
        } else {
            [0.0; NUM_MOTORS]
        };
        let (_thrust, applied) = mixer.forward(&motors);
        sim.step(applied, DT);

        if step % 1000 == 0 {
            println!(
                "{:4.1} s | {:9} | {:9.2} | {:5.2} {:5.2} {:5.2} {:5.2}",
                t,
                format!("{mode:?}"),
                sim.attitude.roll.to_degrees(),
                motors[0],
                motors[1],
                motors[2],
                motors[3]
            );
        }
    }
}
