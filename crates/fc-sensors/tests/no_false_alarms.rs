use fc_core::config::DT;
use fc_core::math::Vec3;
use fc_sensors::imu::{Imu, ImuConfig};
use fc_sensors::validation::{ImuValidator, ImuValidatorConfig};
use fc_sim::{QuadParams, QuadSim};

#[test]
fn healthy_dynamic_flight_raises_no_faults() {
    let mut sim = QuadSim::new(QuadParams::default());
    let mut imu = Imu::new(ImuConfig::default(), 42);
    let mut validator = ImuValidator::new(ImuValidatorConfig::default());

    for step in 0..5000 {
        let t = step as f32 * DT;
        // A strong 0.1 s kick on all three axes, then free rotation.
        let torque = if t < 0.1 {
            Vec3::new(0.05, -0.03, 0.01)
        } else {
            Vec3::ZERO
        };
        sim.step(torque, DT);

        let sample = imu.measure(sim.attitude, sim.rates);
        assert!(
            validator.check(&sample).is_ok(),
            "false alarm at step {step}"
        );
    }
}
