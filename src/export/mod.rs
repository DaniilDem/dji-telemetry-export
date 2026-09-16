//! Output formats and the rules that decide when each one makes sense.

use std::fmt;
use std::path::{Path, PathBuf};

use crate::process::{decimate, ExportRate, Telemetry};

pub mod csv;
pub mod fit;
pub mod gpx;
pub mod igc;
pub mod srt;
pub mod vbo;

pub const APP_NAME: &str = "dji-telemetry-export";
pub const APP_VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    Csv,
    Srt,
    Vbo,
    Gpx,
    Fit,
    Igc,
}

impl Format {
    pub const ALL: [Format; 6] = [
        Format::Csv,
        Format::Srt,
        Format::Vbo,
        Format::Gpx,
        Format::Fit,
        Format::Igc,
    ];

    pub fn extension(self) -> &'static str {
        match self {
            Format::Csv => "csv",
            Format::Srt => "srt",
            Format::Vbo => "vbo",
            Format::Gpx => "gpx",
            Format::Fit => "fit",
            Format::Igc => "igc",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Format::Csv => "CSV",
            Format::Srt => "SRT",
            Format::Vbo => "VBO",
            Format::Gpx => "GPX",
            Format::Fit => "FIT",
            Format::Igc => "IGC",
        }
    }

    pub fn description(self) -> &'static str {
        match self {
            Format::Csv => "OVRLEY-compatible CSV: elapsed time, lateral / longitudinal / vertical G, orientation, exposure and GPS columns",
            Format::Srt => "DJI-style subtitle telemetry (exposure + GPS per frame)",
            Format::Vbo => "Racelogic VBO text (RaceBox layout): time, GPS, G channels",
            Format::Gpx => "GPX 1.1 track with speed / heading / g-force extensions",
            Format::Fit => "Garmin FIT activity (record messages at 1 Hz — FIT timestamps are whole seconds)",
            Format::Igc => "IGC flight log (B records at 1 Hz with GSP / TRT extensions)",
        }
    }

    /// Formats that carry nothing useful without a GPS position.
    pub fn requires_gps(self) -> bool {
        matches!(self, Format::Gpx | Format::Fit | Format::Igc)
    }

    /// Rate used when the user leaves the export rate on "native".
    pub fn default_rate(self) -> ExportRate {
        match self {
            Format::Csv | Format::Srt | Format::Vbo => ExportRate::Native,
            Format::Gpx => ExportRate::Hz(10.0),
            Format::Fit | Format::Igc => ExportRate::Hz(1.0),
        }
    }

    /// Hard upper bound imposed by the format itself.
    pub fn max_rate(self) -> Option<f64> {
        match self {
            Format::Fit | Format::Igc => Some(1.0),
            _ => None,
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "csv" => Some(Format::Csv),
            "srt" => Some(Format::Srt),
            "vbo" => Some(Format::Vbo),
            "gpx" => Some(Format::Gpx),
            "fit" => Some(Format::Fit),
            "igc" => Some(Format::Igc),
            _ => None,
        }
    }
}

impl fmt::Display for Format {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Per-export toggles that do not change the processed telemetry itself.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportOptions {
    /// Write raw body-frame X/Y/Z (gravity included) columns to CSV.
    pub include_raw_axes: bool,
    /// Write camera roll / pitch / yaw columns to CSV.
    pub include_orientation: bool,
    /// Write ISO / shutter / colour temperature columns to CSV.
    pub include_exposure: bool,
    /// Duplicate camera roll into an OVRLEY `Lean angle (deg)` column.
    /// Off by default: OVRLEY back-fills lateral G from lean angle.
    pub lean_angle_from_roll: bool,
    /// Rate override; `Native` means "use each format's default".
    pub rate: ExportRate,
}

impl Default for ExportOptions {
    fn default() -> Self {
        ExportOptions {
            include_raw_axes: false,
            include_orientation: true,
            include_exposure: true,
            lean_angle_from_roll: false,
            rate: ExportRate::Native,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("{0}")]
    Unavailable(String),
    #[error("I/O error writing {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
}

/// Explains why a format cannot be produced from this telemetry, if it cannot.
pub fn availability(format: Format, telemetry: &Telemetry) -> Result<(), String> {
    let info = &telemetry.info;
    if format.requires_gps() && !telemetry.has_gps() {
        let total = info.sample_count;
        let invalid = info.gps_status_counts[1];
        let reason = if info.remote_name.is_none() {
            format!(
                "No GPS data in this clip — no GPS remote (AC004) was connected. {} needs at least two GPS fixes.",
                format.label()
            )
        } else if invalid == total && total > 0 {
            format!(
                "No GPS fix in this clip — {} reported GPS_INVALID in {}/{} samples. {} needs at least two GPS fixes.",
                info.remote_name.as_deref().unwrap_or("the remote"),
                invalid,
                total,
                format.label()
            )
        } else {
            format!(
                "Only {} GPS fix(es) in {} samples. {} needs at least two GPS fixes.",
                info.gps_fix_count,
                total,
                format.label()
            )
        };
        return Err(reason);
    }
    match format {
        Format::Srt => {
            let has_exposure = telemetry
                .rows
                .iter()
                .any(|r| r.iso.is_some() || r.shutter.is_some() || r.color_temperature.is_some());
            if !has_exposure && !telemetry.has_gps() {
                return Err("SRT carries exposure and GPS data only, and this clip has neither.".to_string());
            }
        }
        Format::Csv | Format::Vbo if telemetry.rows.len() < 2 => {
            return Err("Fewer than two telemetry samples.".to_string());
        }
        _ => {}
    }
    Ok(())
}

/// Effective rate for a format given the user's choice.
pub fn effective_rate(format: Format, requested: ExportRate) -> ExportRate {
    let rate = match requested {
        ExportRate::Native => format.default_rate(),
        other => other,
    };
    match (rate, format.max_rate()) {
        (ExportRate::Native, Some(max)) => ExportRate::Hz(max),
        (ExportRate::Hz(hz), Some(max)) if hz > max => ExportRate::Hz(max),
        (r, _) => r,
    }
}

/// Telemetry resampled for a specific format.
pub fn resampled<'a>(
    telemetry: &'a Telemetry,
    format: Format,
    options: &ExportOptions,
) -> std::borrow::Cow<'a, Telemetry> {
    match effective_rate(format, options.rate) {
        ExportRate::Native => std::borrow::Cow::Borrowed(telemetry),
        ExportRate::Hz(hz) => {
            let mut t = telemetry.clone();
            t.rows = decimate(&telemetry.rows, hz);
            t.gps_rows = t.rows.iter().filter(|r| r.has_gps()).count();
            std::borrow::Cow::Owned(t)
        }
    }
}

/// Renders one format to bytes (all formats but FIT are UTF-8 text).
pub fn render(
    format: Format,
    telemetry: &Telemetry,
    options: &ExportOptions,
    source_name: &str,
) -> Result<Vec<u8>, ExportError> {
    availability(format, telemetry).map_err(ExportError::Unavailable)?;
    let t = resampled(telemetry, format, options);
    Ok(match format {
        Format::Csv => csv::render(&t, options).into_bytes(),
        Format::Srt => srt::render(&t).into_bytes(),
        Format::Vbo => vbo::render(&t, source_name).into_bytes(),
        Format::Gpx => gpx::render(&t, source_name).into_bytes(),
        Format::Fit => fit::render(&t),
        Format::Igc => igc::render(&t, source_name).into_bytes(),
    })
}

/// Summary of one written file.
#[derive(Debug, Clone)]
pub struct Written {
    pub format: Format,
    pub path: PathBuf,
    pub rows: usize,
    pub bytes: usize,
}

/// Writes `<out_dir>/<stem>.<ext>`.
pub fn export_file(
    format: Format,
    telemetry: &Telemetry,
    options: &ExportOptions,
    out_dir: &Path,
    stem: &str,
    source_name: &str,
) -> Result<Written, ExportError> {
    let bytes = render(format, telemetry, options, source_name)?;
    let rows = resampled(telemetry, format, options).rows.len();
    let path = out_dir.join(format!("{stem}.{}", format.extension()));
    std::fs::write(&path, &bytes).map_err(|source| ExportError::Io {
        path: path.clone(),
        source,
    })?;
    Ok(Written {
        format,
        path,
        rows,
        bytes: bytes.len(),
    })
}

/// Formats a float without exponent notation and with a fixed number of decimals.
pub(crate) fn num(v: f64, decimals: usize) -> String {
    if v.is_finite() {
        format!("{v:.decimals$}")
    } else {
        String::new()
    }
}

pub(crate) fn opt(v: Option<f64>, decimals: usize) -> String {
    v.map(|v| num(v, decimals)).unwrap_or_default()
}

pub(crate) fn xml_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

#[cfg(test)]
pub(crate) mod testutil {
    use crate::dji::{Clip, ClipInfo, GpsFix, Sample, StartTimeSource};
    use crate::process::{process, ProcessOptions, Telemetry};
    use chrono::NaiveDate;

    /// 5 seconds at 30 Hz, level camera, small lateral burst; optional GPS at every sample.
    pub fn telemetry(with_gps: bool) -> Telemetry {
        let mut samples = Vec::new();
        for i in 0..150 {
            let t = i as f64 / 30.0;
            let lateral = if (60..90).contains(&i) { 0.8 } else { 0.0 };
            let mut s = Sample {
                t,
                frame: i as u64,
                acc: Some([lateral, 0.1, 1.0]),
                quat: Some([1.0, 0.0, 0.0, 0.0]),
                iso: Some(100.0),
                shutter: Some((1, 2000)),
                color_temperature: Some(5500),
                gps: None,
            };
            if with_gps {
                s.gps = Some(GpsFix {
                    latitude: 55.75 + 0.00002 * i as f64,
                    longitude: 37.62 + 0.00001 * i as f64,
                    altitude_m: Some(150.0 + i as f64 * 0.01),
                    status: 0,
                    velocity: Some([2.0, 1.5, 0.0]),
                    time: None,
                });
            }
            samples.push(s);
        }
        let info = ClipInfo {
            model: Some("DJI OsmoAction5 Pro".into()),
            serial: Some("SERIAL01".into()),
            firmware: Some("10.00.16.13".into()),
            fps: Some(29.97),
            remote_name: Some("DJI AC004".into()),
            start_local: NaiveDate::from_ymd_opt(2026, 9, 16)
                .unwrap()
                .and_hms_opt(9, 59, 53),
            start_time_source: StartTimeSource::FileName,
            utc_offset_minutes: Some(120),
            sample_count: 150,
            duration_s: 149.0 / 30.0,
            gps_fix_count: if with_gps { 150 } else { 0 },
            gps_status_counts: if with_gps { [150, 0, 0] } else { [0, 150, 0] },
            ..ClipInfo::default()
        };
        process(&Clip { info, samples }, &ProcessOptions::default())
    }
}

#[cfg(test)]
mod tests {
    use super::testutil::telemetry;
    use super::*;

    #[test]
    fn gps_formats_are_unavailable_without_fix() {
        let t = telemetry(false);
        for f in [Format::Gpx, Format::Fit, Format::Igc] {
            let err = availability(f, &t).unwrap_err();
            assert!(err.contains("GPS_INVALID"), "{err}");
        }
        for f in [Format::Csv, Format::Srt, Format::Vbo] {
            availability(f, &t).unwrap();
        }
    }

    #[test]
    fn everything_available_with_gps() {
        let t = telemetry(true);
        for f in Format::ALL {
            availability(f, &t).unwrap();
        }
    }

    #[test]
    fn effective_rates() {
        assert_eq!(
            effective_rate(Format::Csv, ExportRate::Native),
            ExportRate::Native
        );
        assert_eq!(
            effective_rate(Format::Gpx, ExportRate::Native),
            ExportRate::Hz(10.0)
        );
        assert_eq!(
            effective_rate(Format::Igc, ExportRate::Hz(10.0)),
            ExportRate::Hz(1.0)
        );
        assert_eq!(
            effective_rate(Format::Csv, ExportRate::Hz(5.0)),
            ExportRate::Hz(5.0)
        );
    }
}
