use fc_core::config::{CONTROL_RATE_HZ, DT};
use fc_core::math::Vec3;
use fc_sim::{QuadParams, QuadSim};

fn main() {
    let mut sim = QuadSim::new(QuadParams::default());
    let total_steps = (2.0 * CONTROL_RATE_HZ) as u32;

    for step in 0..total_steps {
        let t = step as f32 * DT;

        // A 0.1 s torque pulse on the roll axis: a "kick" to the drone.
        let torque = if t < 0.1 {
            Vec3::new(0.01, 0.0, 0.0)
        } else {
            Vec3::ZERO
        };

        sim.step(torque, DT);

        if step % 200 == 0 {
            println!(
                "t = {:.2} s   roll = {:6.2} deg   roll rate = {:6.2} deg/s",
                t,
                sim.attitude.roll.to_degrees(),
                sim.rates.x.to_degrees()
            );
        }
    }
}
