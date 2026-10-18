use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_control::limiter::{Limiter, LimiterConfig};
use fc_control::mixer::{Mixer, MixerConfig};
use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Attitude;
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
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
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());
    let limiter = Limiter::new(LimiterConfig::default());
    let mixer = Mixer::new(MixerConfig::default());
    let throttle = 0.5;

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);
    let mut last_good = first;
    let mut fault_count = 0_u32;

    let total_steps = (8.0 * CONTROL_RATE_HZ) as u32;
    println!("   t  | roll true | roll est | pitch true");

    for step in 0..total_steps {
        let t = step as f32 * DT;
        let mut sample = imu.measure(sim.attitude, sim.rates);

        // Fault injection: a 3 ms NaN burst on the gyro, then one accel spike.
        match step {
            2000..=2002 => sample.gyro.x = f32::NAN,
            5000 => sample.accel.z += 100.0,
            _ => {}
        }

        // Validate; replace rejected data by the last good value.
        let check = validator.check(&sample);
        if let Some(fault) = check.gyro {
            println!("t = {t:.3} s  GYRO  fault: {fault:?}");
            fault_count += 1;
            sample.gyro = last_good.gyro;
        } else {
            last_good.gyro = sample.gyro;
        }
        if let Some(fault) = check.accel {
            println!("t = {t:.3} s  ACCEL fault: {fault:?}");
            fault_count += 1;
            sample.accel = last_good.accel;
        } else {
            last_good.accel = sample.accel;
        }

        let att = est.update(&sample, DT);
        let torque_cmd = ctrl.update(Attitude::default(), att, sample.gyro, DT);
        let (thr, torque) = limiter.apply(throttle, torque_cmd);
        let motors = mixer.mix(thr, torque);
        let (_thrust, applied) = mixer.forward(&motors);
        sim.step(applied, DT);

        if step % 1000 == 0 {
            println!(
                "{:4.1} s | {:8.2} | {:8.2} | {:9.2}",
                t,
                sim.attitude.roll.to_degrees(),
                att.roll.to_degrees(),
                sim.attitude.pitch.to_degrees()
            );
        }
    }
    println!("faults detected: {fault_count} (injected: 4)");
}
