//! Command-line front end (same binary as the GUI; used when arguments are given).

use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

use anyhow::{anyhow, bail, Context};
use clap::Parser;

use crate::dji::{self, Clip};
use crate::export::{self, ExportOptions, Format, APP_NAME, APP_VERSION};
use crate::process::{self, AccelUnit, AxisMap, ExportRate, GravityMode, ProcessOptions, Telemetry};
use crate::{mp4, protobuf};

#[derive(Parser, Debug)]
#[command(
    name = APP_NAME,
    version = APP_VERSION,
    about = "Extract telemetry from DJI Osmo Action MP4 files and export it for OVRLEY",
    long_about = "Reads the `djmd` metadata track of a DJI Osmo Action 4/5/6 clip (accelerometer, \
attitude, exposure and — with the AC004 GPS remote — position/velocity) and writes CSV, SRT, VBO, \
GPX, FIT or IGC files. Run without arguments to open the graphical interface."
)]
pub struct Args {
    /// DJI MP4 file
    pub video: PathBuf,

    /// Comma-separated formats: csv,srt,vbo,gpx,fit,igc or "all"
    #[arg(short, long, default_value = "csv")]
    pub formats: String,

    /// Output directory (default: next to the video)
    #[arg(short, long)]
    pub out: Option<PathBuf>,

    /// Gravity removal: quat (attitude quaternion), highpass, none
    #[arg(long, default_value = "quat")]
    pub gravity: String,

    /// Moving-average window in seconds for --gravity highpass
    #[arg(long, default_value_t = 1.0)]
    pub highpass_window: f64,

    /// Body axis mapping: "auto", "xyz" (lateral,longitudinal,vertical) or lateral=x,longitudinal=y,vertical=z
    #[arg(long, default_value = "auto")]
    pub axes: String,

    /// Invert the sign of these vehicle axes: lateral,longitudinal,vertical
    #[arg(long, default_value = "")]
    pub invert: String,

    /// Export rate: native, or a value in Hz (e.g. 10). Per-format defaults apply to "native".
    #[arg(long, default_value = "native")]
    pub rate: String,

    /// Accelerometer unit override: g, ms2, mg (default: auto-detect)
    #[arg(long)]
    pub unit: Option<String>,

    /// Also write raw body-frame X/Y/Z columns (CSV)
    #[arg(long)]
    pub raw_axes: bool,

    /// Omit camera roll/pitch/yaw columns (CSV)
    #[arg(long)]
    pub no_orientation: bool,

    /// Omit ISO / shutter / colour temperature columns (CSV)
    #[arg(long)]
    pub no_exposure: bool,

    /// Write camera roll as an OVRLEY "Lean angle" column (CSV; OVRLEY then back-fills lateral G from it)
    #[arg(long)]
    pub lean_angle: bool,

    /// Print the clip summary and exit without exporting
    #[arg(long)]
    pub info: bool,

    /// Dump the protobuf field tree of the first N metadata samples and exit
    #[arg(long, value_name = "N", num_args = 0..=1, default_missing_value = "2")]
    pub inspect: Option<usize>,

    /// Print the clip summary as JSON (with --info)
    #[arg(long)]
    pub json: bool,
}

pub fn parse_formats(spec: &str) -> anyhow::Result<Vec<Format>> {
    if spec.trim().eq_ignore_ascii_case("all") {
        return Ok(Format::ALL.to_vec());
    }
    let mut out = Vec::new();
    for part in spec.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let f = Format::parse(part)
            .ok_or_else(|| anyhow!("unknown format '{part}' (expected csv, srt, vbo, gpx, fit, igc)"))?;
        if !out.contains(&f) {
            out.push(f);
        }
    }
    if out.is_empty() {
        bail!("no export format selected");
    }
    Ok(out)
}

pub fn open_clip(path: &Path, mut progress: impl FnMut(usize, usize)) -> anyhow::Result<Clip> {
    // No BufReader on purpose: samples are ~150 bytes scattered between video chunks, and a
    // buffered reader would refill its whole buffer after every seek (gigabytes of extra I/O).
    let mut file = File::open(path).with_context(|| format!("cannot open {}", path.display()))?;
    let name = path.file_name().and_then(|n| n.to_str());
    let clip = dji::read_clip(&mut file, name, &mut progress)?;
    Ok(clip)
}

pub fn build_process_options(args: &Args, clip: &Clip) -> anyhow::Result<ProcessOptions> {
    let gravity =
        GravityMode::parse(&args.gravity).ok_or_else(|| anyhow!("unknown --gravity '{}'", args.gravity))?;
    let unit_override = match args.unit.as_deref().map(|s| s.to_ascii_lowercase()) {
        None => None,
        Some(u) if u == "g" => Some(AccelUnit::G),
        Some(u) if u == "ms2" || u == "m/s2" || u == "m/s^2" => Some(AccelUnit::MetersPerSecondSquared),
        Some(u) if u == "mg" => Some(AccelUnit::MilliG),
        Some(u) => bail!("unknown --unit '{u}'"),
    };
    let scale = unit_override
        .unwrap_or_else(|| process::detect_accel_unit(&clip.samples).0)
        .scale_to_g();
    let mut axes = if args.axes.trim().eq_ignore_ascii_case("auto") {
        process::auto_axis_map(process::axis_medians(&clip.samples, scale))
    } else {
        AxisMap::parse(&args.axes).map_err(|e| anyhow!("--axes: {e}"))?
    };
    for part in args.invert.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        match part.to_ascii_lowercase().as_str() {
            "lateral" | "lat" => axes.invert_lateral = !axes.invert_lateral,
            "longitudinal" | "lon" | "long" => axes.invert_longitudinal = !axes.invert_longitudinal,
            "vertical" | "vert" | "ver" => axes.invert_vertical = !axes.invert_vertical,
            other => bail!("--invert: unknown axis '{other}'"),
        }
    }
    Ok(ProcessOptions {
        gravity,
        highpass_window_s: args.highpass_window,
        axes,
        rate: ExportRate::Native,
        unit_override,
    })
}

pub fn build_export_options(args: &Args) -> anyhow::Result<ExportOptions> {
    let rate = ExportRate::parse(&args.rate)
        .ok_or_else(|| anyhow!("--rate must be 'native' or a positive number of Hz"))?;
    Ok(ExportOptions {
        include_raw_axes: args.raw_axes,
        include_orientation: !args.no_orientation,
        include_exposure: !args.no_exposure,
        lean_angle_from_roll: args.lean_angle,
        rate,
    })
}

/// Human-readable clip summary shared by the CLI and the GUI.
pub fn summary_lines(telemetry: &Telemetry) -> Vec<(String, String)> {
    let info = &telemetry.info;
    let mut lines = Vec::new();
    let mut push = |k: &str, v: String| lines.push((k.to_string(), v));
    push("Camera", info.model.clone().unwrap_or_else(|| "unknown".into()));
    if let Some(s) = &info.serial {
        push("Serial", s.clone());
    }
    if let Some(f) = &info.firmware {
        push("Firmware", f.clone());
    }
    if let (Some(p), Some(v)) = (&info.proto_name, &info.proto_version) {
        push("Schema", format!("{p} v{v}"));
    }
    if let (Some(w), Some(h)) = (info.width, info.height) {
        push(
            "Video",
            format!(
                "{w}×{h} @ {}",
                info.fps.map(|f| format!("{f:.2} fps")).unwrap_or_default()
            ),
        );
    }
    push("Metadata track", info.metadata_track.clone().unwrap_or_default());
    let rate = if info.duration_s > 0.0 {
        (info.sample_count as f64 - 1.0) / info.duration_s
    } else {
        0.0
    };
    push(
        "Samples",
        format!(
            "{} over {:.1} s (≈{rate:.2} Hz)",
            info.sample_count, info.duration_s
        ),
    );
    push(
        "Accelerometer",
        format!(
            "unit {} (median |a| = {:.3}), body-axis medians X {:+.3} Y {:+.3} Z {:+.3} g",
            telemetry.unit,
            telemetry.unit_median_magnitude,
            telemetry.axis_medians[0],
            telemetry.axis_medians[1],
            telemetry.axis_medians[2]
        ),
    );
    let a = telemetry.options.axes;
    push(
        "Axis mapping",
        format!(
            "lateral = {}{}, longitudinal = {}{}, vertical = {}{}",
            if a.invert_lateral { "-" } else { "" },
            a.lateral.label(),
            if a.invert_longitudinal { "-" } else { "" },
            a.longitudinal.label(),
            if a.invert_vertical { "-" } else { "" },
            a.vertical.label()
        ),
    );
    push(
        "Gravity removal",
        match telemetry.gravity_world {
            Some(g) => format!(
                "{} — world gravity ({:+.3}, {:+.3}, {:+.3}) g",
                telemetry.gravity_used.cli_name(),
                g[0],
                g[1],
                g[2]
            ),
            None => telemetry.gravity_used.cli_name().to_string(),
        },
    );
    let gps = match (&info.remote_name, info.gps_fix_count) {
        (None, _) => "no GPS remote connected".to_string(),
        (Some(r), 0) => format!(
            "{r} connected, no fix (status invalid in {} of {} samples)",
            info.gps_status_counts[1], info.sample_count
        ),
        (Some(r), n) => format!(
            "{r}: {n} fixes in {} samples ({} RTK)",
            info.sample_count, info.gps_status_counts[2]
        ),
    };
    push("GPS", gps);
    push(
        "Start time",
        match info.start_local {
            Some(dt) => format!(
                "{} camera clock ({}){}",
                dt.format("%Y-%m-%d %H:%M:%S"),
                info.start_time_source.label(),
                info.utc_offset_minutes
                    .map(|m| format!(", UTC{:+03}:{:02}", m / 60, (m % 60).abs()))
                    .unwrap_or_default()
            ),
            None => "unknown".to_string(),
        },
    );
    lines
}

fn inspect(path: &Path, count: usize) -> anyhow::Result<()> {
    let mut reader = File::open(path)?;
    let movie = mp4::read_movie(&mut reader)?;
    println!("Top-level tracks:");
    for t in &movie.tracks {
        println!(
            "  #{} {:<5} {:<18} samples={:<7} timescale={}",
            t.track_id,
            t.fourcc,
            format!("{:?}", t.handler_name),
            t.sample_count(),
            t.timescale
        );
    }
    let track = dji::find_metadata_track(&movie).ok_or(dji::DjiError::NoMetadataTrack)?;
    for i in 0..count.min(track.sample_count()) {
        let bytes = track.read_sample(&mut reader, i)?;
        println!(
            "\n--- {} sample #{i} ({} bytes, t={:.3}s) ---",
            track.fourcc,
            bytes.len(),
            track.times[i]
        );
        let mut out = String::new();
        protobuf::dump_tree(&bytes, 0, 6, &mut out);
        print!("{out}");
    }
    Ok(())
}

/// Runs the CLI; returns the process exit code.
pub fn run(args: Args) -> anyhow::Result<i32> {
    if let Some(n) = args.inspect {
        inspect(&args.video, n.max(1))?;
        return Ok(0);
    }

    let formats = parse_formats(&args.formats)?;
    let export_options = build_export_options(&args)?;

    let stderr = std::io::stderr();
    let mut last_pct = usize::MAX;
    let clip = open_clip(&args.video, |done, total| {
        let pct = (done * 100).checked_div(total).unwrap_or(0);
        if pct != last_pct && pct % 10 == 0 {
            last_pct = pct;
            let _ = write!(stderr.lock(), "\rReading metadata… {pct:3}%");
        }
    })?;
    let _ = writeln!(
        stderr.lock(),
        "\rReading metadata… done ({} samples)",
        clip.samples.len()
    );

    let process_options = build_process_options(&args, &clip)?;
    let telemetry = process::process(&clip, &process_options);

    if args.info || args.json {
        if args.json {
            println!("{}", summary_json(&telemetry));
        } else {
            for (k, v) in summary_lines(&telemetry) {
                println!("{k:<16} {v}");
            }
            println!();
            for f in Format::ALL {
                match export::availability(f, &telemetry) {
                    Ok(()) => println!("{:<4} available", f.label()),
                    Err(reason) => println!("{:<4} unavailable: {reason}", f.label()),
                }
            }
        }
        return Ok(0);
    }

    let out_dir = match &args.out {
        Some(d) => d.clone(),
        None => args
            .video
            .parent()
            .map(Path::to_path_buf)
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| PathBuf::from(".")),
    };
    std::fs::create_dir_all(&out_dir).with_context(|| format!("cannot create {}", out_dir.display()))?;
    let stem = args
        .video
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("telemetry")
        .to_string();
    let source_name = args
        .video
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or("video")
        .to_string();

    for (k, v) in summary_lines(&telemetry) {
        eprintln!("{k:<16} {v}");
    }

    let mut exit = 0;
    for format in formats {
        match export::export_file(format, &telemetry, &export_options, &out_dir, &stem, &source_name) {
            Ok(w) => println!(
                "{:<4} {} ({} rows, {} bytes)",
                w.format.label(),
                w.path.display(),
                w.rows,
                w.bytes
            ),
            Err(export::ExportError::Unavailable(reason)) => {
                eprintln!("{:<4} skipped: {reason}", format.label());
                exit = 2;
            }
            Err(e) => return Err(e.into()),
        }
    }
    Ok(exit)
}

fn summary_json(telemetry: &Telemetry) -> String {
    fn esc(s: &str) -> String {
        s.replace('\\', "\\\\").replace('"', "\\\"")
    }
    let info = &telemetry.info;
    let mut fields = vec![
        format!("\"model\": \"{}\"", esc(info.model.as_deref().unwrap_or(""))),
        format!("\"serial\": \"{}\"", esc(info.serial.as_deref().unwrap_or(""))),
        format!(
            "\"firmware\": \"{}\"",
            esc(info.firmware.as_deref().unwrap_or(""))
        ),
        format!("\"proto\": \"{}\"", esc(info.proto_name.as_deref().unwrap_or(""))),
        format!("\"samples\": {}", info.sample_count),
        format!("\"duration_s\": {:.3}", info.duration_s),
        format!("\"gps_fixes\": {}", info.gps_fix_count),
        format!(
            "\"remote\": \"{}\"",
            esc(info.remote_name.as_deref().unwrap_or(""))
        ),
        format!("\"accel_unit\": \"{}\"", telemetry.unit),
        format!(
            "\"axis_medians_g\": [{:.4}, {:.4}, {:.4}]",
            telemetry.axis_medians[0], telemetry.axis_medians[1], telemetry.axis_medians[2]
        ),
        format!(
            "\"start_local\": \"{}\"",
            info.start_local
                .map(|d| d.format("%Y-%m-%dT%H:%M:%S").to_string())
                .unwrap_or_default()
        ),
        format!(
            "\"utc_offset_minutes\": {}",
            info.utc_offset_minutes
                .map(|m| m.to_string())
                .unwrap_or_else(|| "null".into())
        ),
    ];
    let avail: Vec<String> = Format::ALL
        .iter()
        .map(|f| {
            format!(
                "\"{}\": {}",
                f.extension(),
                match export::availability(*f, telemetry) {
                    Ok(()) => "true".to_string(),
                    Err(r) => format!("\"{}\"", esc(&r)),
                }
            )
        })
        .collect();
    fields.push(format!("\"formats\": {{{}}}", avail.join(", ")));
    format!("{{{}}}", fields.join(", "))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_list_parsing() {
        assert_eq!(parse_formats("csv, gpx").unwrap(), vec![Format::Csv, Format::Gpx]);
        assert_eq!(parse_formats("all").unwrap().len(), 6);
        assert!(parse_formats("kml").is_err());
        assert!(parse_formats("").is_err());
    }

    #[test]
    fn args_parse_defaults() {
        let a = Args::try_parse_from(["x", "clip.mp4"]).unwrap();
        assert_eq!(a.formats, "csv");
        assert_eq!(a.axes, "auto");
        assert!(a.inspect.is_none());
        let a = Args::try_parse_from(["x", "clip.mp4", "--inspect"]).unwrap();
        assert_eq!(a.inspect, Some(2));
        let a = Args::try_parse_from(["x", "clip.mp4", "--inspect", "5", "-f", "all"]).unwrap();
        assert_eq!(a.inspect, Some(5));
    }
}
