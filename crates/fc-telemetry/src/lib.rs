use fc_core::math::{Attitude, Vec3};
use std::io::{self, Write};

/// Column names, in the exact order of the written rows (21 columns).
pub const CSV_HEADER: &str = "time_s,roll_true,pitch_true,yaw_true,\
roll_est,pitch_est,yaw_est,roll_set,pitch_set,yaw_set,\
gyro_x,gyro_y,gyro_z,mode,gyro_fault,accel_fault,throttle,m1,m2,m3,m4";

/// Everything worth recording about one control tick.
/// Angles in radians, rates in rad/s.
#[derive(Debug, Clone, Copy)]
pub struct TelemetryRecord {
    pub time_s: f32,
    /// Ground truth from the simulator (not available on a real vehicle).
    pub true_attitude: Attitude,
    pub estimated_attitude: Attitude,
    /// Setpoint after the safety supervisor (e.g. level in failsafe).
    pub setpoint: Attitude,
    /// Raw gyro as delivered by the sensor, may contain NaN.
    pub gyro: Vec3,
    /// Flight mode label. Must not contain commas.
    pub mode: &'static str,
    pub gyro_fault: bool,
    pub accel_fault: bool,
    pub throttle: f32,
    pub motors: [f32; 4],
}

/// Writes telemetry records as CSV to any `Write` target.
#[derive(Debug)]
pub struct TelemetryLogger<W: Write> {
    writer: W,
    decimation: u32,
    /// Ticks to skip before the next row is written (0 = write now).
    countdown: u32,
    rows_written: u64,
}

impl<W: Write> TelemetryLogger<W> {
    /// Writes the CSV header immediately. Only every `decimation`-th record
    /// is written (0 is treated as 1 = every record).
    pub fn new(mut writer: W, decimation: u32) -> io::Result<Self> {
        writeln!(writer, "{CSV_HEADER}")?;
        Ok(Self {
            writer,
            decimation: decimation.max(1),
            countdown: 0,
            rows_written: 0,
        })
    }

    /// Call once per control tick.
    pub fn log(&mut self, r: &TelemetryRecord) -> io::Result<()> {
        let write_this = self.countdown == 0;
        self.countdown = if write_this {
            self.decimation - 1
        } else {
            self.countdown - 1
        };
        if !write_this {
            return Ok(());
        }

        writeln!(
            self.writer,
            "{:.4},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4}",
            r.time_s,
            r.true_attitude.roll,
            r.true_attitude.pitch,
            r.true_attitude.yaw,
            r.estimated_attitude.roll,
            r.estimated_attitude.pitch,
            r.estimated_attitude.yaw,
            r.setpoint.roll,
            r.setpoint.pitch,
            r.setpoint.yaw,
            r.gyro.x,
            r.gyro.y,
            r.gyro.z,
            r.mode,
            u8::from(r.gyro_fault),
            u8::from(r.accel_fault),
            r.throttle,
            r.motors[0],
            r.motors[1],
            r.motors[2],
            r.motors[3],
        )?;
        self.rows_written += 1;
        Ok(())
    }

    pub fn rows_written(&self) -> u64 {
        self.rows_written
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }

    /// Give back the underlying writer (used by tests to read the output).
    pub fn into_inner(self) -> W {
        self.writer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(t: f32) -> TelemetryRecord {
        TelemetryRecord {
            time_s: t,
            true_attitude: Attitude {
                roll: 0.1,
                pitch: -0.2,
                yaw: 0.3,
            },
            estimated_attitude: Attitude {
                roll: 0.11,
                pitch: -0.21,
                yaw: 0.31,
            },
            setpoint: Attitude::default(),
            gyro: Vec3::new(0.01, 0.02, 0.03),
            mode: "Normal",
            gyro_fault: false,
            accel_fault: true,
            throttle: 0.5,
            motors: [0.1, 0.2, 0.3, 0.4],
        }
    }

    fn logger(decimation: u32) -> TelemetryLogger<Vec<u8>> {
        TelemetryLogger::new(Vec::new(), decimation).unwrap()
    }

    fn text(logger: TelemetryLogger<Vec<u8>>) -> String {
        String::from_utf8(logger.into_inner()).unwrap()
    }

    fn field_count(line: &str) -> usize {
        line.split(',').count()
    }

    #[test]
    fn header_is_written_on_creation() {
        let out = text(logger(1));
        assert_eq!(out.lines().count(), 1);
        assert_eq!(out.lines().next().unwrap(), CSV_HEADER);
        assert_eq!(field_count(CSV_HEADER), 21);
    }

    #[test]
    fn every_row_has_as_many_fields_as_the_header() {
        let mut l = logger(1);
        l.log(&record(0.0)).unwrap();
        l.log(&record(0.001)).unwrap();
        let out = text(l);
        assert_eq!(out.lines().count(), 3);
        for line in out.lines() {
            assert_eq!(field_count(line), 21);
        }
    }

    #[test]
    fn values_round_trip_through_the_row() {
        let mut l = logger(1);
        l.log(&record(0.123)).unwrap();
        let out = text(l);
        let row: Vec<&str> = out.lines().nth(1).unwrap().split(',').collect();
        let num = |i: usize| row[i].parse::<f32>().unwrap();

        assert!((num(0) - 0.123).abs() < 1e-4); // time
        assert!((num(1) - 0.1).abs() < 1e-5); // roll_true
        assert!((num(5) + 0.21).abs() < 1e-5); // pitch_est
        assert!((num(12) - 0.03).abs() < 1e-5); // gyro_z
        assert_eq!(row[13], "Normal");
        assert_eq!(row[14], "0"); // gyro_fault
        assert_eq!(row[15], "1"); // accel_fault
        assert!((num(16) - 0.5).abs() < 1e-4); // throttle
        assert!((num(17) - 0.1).abs() < 1e-4); // m1
        assert!((num(20) - 0.4).abs() < 1e-4); // m4
    }

    #[test]
    fn non_finite_values_stay_visible_and_the_row_stays_intact() {
        let mut r = record(0.0);
        r.gyro = Vec3::new(f32::NAN, 0.0, f32::INFINITY);
        let mut l = logger(1);
        l.log(&r).unwrap();
        let out = text(l);
        let line = out.lines().nth(1).unwrap();
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 21);
        assert_eq!(row[10], "NaN");
        assert_eq!(row[12], "inf");
    }

    #[test]
    fn decimation_keeps_every_nth_record() {
        let mut l = logger(10);
        for i in 0..100 {
            l.log(&record(i as f32 * 0.001)).unwrap();
        }
        assert_eq!(l.rows_written(), 10);
        let out = text(l);
        assert_eq!(out.lines().count(), 11);

        // First record is kept, then the 11th, ...
        let time_of = |n: usize| -> f32 {
            out.lines()
                .nth(n)
                .unwrap()
                .split(',')
                .next()
                .unwrap()
                .parse()
                .unwrap()
        };
        assert!(time_of(1).abs() < 1e-6);
        assert!((time_of(2) - 0.010).abs() < 1e-4);
    }

    #[test]
    fn zero_decimation_is_treated_as_one() {
        let mut l = logger(0);
        for i in 0..3 {
            l.log(&record(i as f32)).unwrap();
        }
        assert_eq!(l.rows_written(), 3);
    }
}
