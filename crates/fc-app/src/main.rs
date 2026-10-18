use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Vec3;
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_sensors::imu::{Imu, ImuConfig};
use fc_sim::{QuadParams, QuadSim};

fn main() {
    let mut sim = QuadSim::new(QuadParams::default());
    let mut imu = Imu::new(ImuConfig::default(), 42);
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);

    let mut gyro_only_roll = 0.0_f32;
    let total_steps = (10.0 * CONTROL_RATE_HZ) as u32;

    println!("   t   true roll | filter roll (err) | gyro-only roll (err) | yaw est");
    for step in 0..total_steps {
        let t = step as f32 * DT;

        // A 0.1 s torque pulse on the roll axis: a "kick" to the drone.
        let torque = if t < 0.1 {
            Vec3::new(0.01, 0.0, 0.0)
        } else {
            Vec3::ZERO
        };

        sim.step(torque, DT);
        let sample = imu.measure(sim.attitude, sim.rates);
        let att = est.update(&sample, DT);
        gyro_only_roll += sample.gyro.x * DT;

        if step % 1000 == 0 {
            let truth = sim.attitude.roll.to_degrees();
            let filt = att.roll.to_degrees();
            let gyro = gyro_only_roll.to_degrees();
            println!(
                "{:4.1} s {:8.2} deg | {:7.2} ({:5.2}) | {:7.2} ({:5.2}) | {:6.2} deg",
                t,
                truth,
                filt,
                filt - truth,
                gyro,
                gyro - truth,
                att.yaw.to_degrees()
            );
        }
    }
}
