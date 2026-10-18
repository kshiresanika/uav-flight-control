/// Control loop frequency. Fixed, never derived from the wall clock.
pub const CONTROL_RATE_HZ: f32 = 1000.0;

/// Fixed time step in seconds (1 ms).
pub const DT: f32 = 1.0 / CONTROL_RATE_HZ;
