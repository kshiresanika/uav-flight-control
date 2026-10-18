use fc_control::attitude::{AttitudeController, AttitudeGains};
use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Attitude;
use fc_estimation::{AttitudeEstimator, EstimatorConfig};
use fc_sensors::imu::{Imu, ImuConfig};
use fc_sim::{QuadParams, QuadSim};

fn main() {
    let mut sim = QuadSim::new(QuadParams::default());
    // Start badly tilted: 20 deg roll, -11 deg pitch.
    sim.attitude = Attitude {
        roll: 0.35,
        pitch: -0.20,
        yaw: 0.0,
    };

    let mut imu = Imu::new(ImuConfig::default(), 42);
    let mut est = AttitudeEstimator::new(EstimatorConfig::default());
    let mut ctrl = AttitudeController::new(AttitudeGains::default());

    let first = imu.measure(sim.attitude, sim.rates);
    est.initialize_from_accel(first.accel);

    let total_steps = (8.0 * CONTROL_RATE_HZ) as u32;
    println!("   t  | roll set | roll true | pitch true | yaw true | torque x");

    for step in 0..total_steps {
        let t = step as f32 * DT;

        // Hold level for 4 s, then command a 10 degree roll.
        let setpoint = Attitude {
            roll: if t >= 4.0 { 0.1745 } else { 0.0 },
            ..Attitude::default()
        };

        let sample = imu.measure(sim.attitude, sim.rates);
        let att = est.update(&sample, DT);
        let torque = ctrl.update(setpoint, att, sample.gyro, DT);
        sim.step(torque, DT);

        if step % 500 == 0 {
            println!(
                "{:4.1} s | {:6.1} | {:8.2} | {:9.2} | {:8.2} | {:8.4}",
                t,
                setpoint.roll.to_degrees(),
                sim.attitude.roll.to_degrees(),
                sim.attitude.pitch.to_degrees(),
                sim.attitude.yaw.to_degrees(),
                torque.x
            );
        }
    }
}
