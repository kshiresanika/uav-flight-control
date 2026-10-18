mod scenario;

use fc_telemetry::TelemetryLogger;
use scenario::Scenario;
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::Path;
use std::process::ExitCode;

/// Log every 10th tick: 1 kHz loop -> 100 Hz telemetry.
const LOG_DECIMATION: u32 = 10;

fn print_usage() {
    eprintln!("usage: fc-app <scenario> [output.csv]");
    eprintln!("scenarios:");
    for s in Scenario::ALL {
        eprintln!("  {}", s.name());
    }
}

fn execute(scenario: Scenario, path: &str) -> io::Result<()> {
    // Create the output folder if needed (e.g. docs/).
    let parent = Path::new(path)
        .parent()
        .filter(|p| !p.as_os_str().is_empty());
    if let Some(parent) = parent {
        fs::create_dir_all(parent)?;
    }

    let file = File::create(path)?;
    let mut logger = TelemetryLogger::new(BufWriter::new(file), LOG_DECIMATION)?;
    let summary = scenario::run(scenario, &mut logger)?;
    logger.flush()?;

    println!("scenario:   {}", scenario.name());
    println!(
        "final true: roll {:.2} deg, pitch {:.2} deg",
        summary.final_attitude.roll.to_degrees(),
        summary.final_attitude.pitch.to_degrees()
    );
    println!("final mode: {:?}", summary.final_mode);
    if let Some(reason) = summary.failsafe_reason {
        println!("failsafe:   {reason:?}");
    }
    println!("telemetry:  {} rows -> {path}", logger.rows_written());
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    let Some(name) = args.first() else {
        print_usage();
        return ExitCode::FAILURE;
    };
    let Some(scenario) = Scenario::from_name(name) else {
        eprintln!("unknown scenario '{name}'");
        print_usage();
        return ExitCode::FAILURE;
    };
    let path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| format!("telemetry_{}.csv", scenario.name()));

    match execute(scenario, &path) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
