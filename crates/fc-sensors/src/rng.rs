use core::f32::consts::PI;

/// Small deterministic pseudo-random generator (SplitMix64).
/// Same seed -> same sequence, on every machine, forever.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform random number in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        // Keep 24 bits: exactly representable in f32.
        (self.next_u64() >> 40) as f32 / 16_777_216.0
    }

    /// Standard normal (mean 0, std 1) via the Box-Muller transform.
    pub fn gaussian(&mut self) -> f32 {
        let u1 = 1.0 - self.uniform(); // in (0, 1], so ln() is finite
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_gives_same_sequence() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_differ() {
        let mut a = Rng::new(1);
        let mut b = Rng::new(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn gaussian_has_zero_mean_and_unit_std() {
        let mut rng = Rng::new(7);
        let n = 100_000_usize;
        let mut sum = 0.0_f64;
        let mut sum_sq = 0.0_f64;
        for _ in 0..n {
            let x = rng.gaussian() as f64;
            sum += x;
            sum_sq += x * x;
        }
        let mean = sum / n as f64;
        let var = sum_sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.02);
        assert!((var.sqrt() - 1.0).abs() < 0.02);
    }
}
