use fc_core::config::{CONTROL_RATE_HZ, DT};

fn main() {
    let duration_s = 5.0_f32;
    let total_steps = (duration_s * CONTROL_RATE_HZ) as u32;

    for step in 0..total_steps {
        // Time comes from an integer step counter, not from accumulating
        // floats: no drift, fully reproducible.
        let t = step as f32 * DT;

        // Later: sensors -> validate -> estimate -> PID -> mix -> sim
        if step % 1000 == 0 {
            println!("step {step:5}  t = {t:.3} s");
        }
    }
}
