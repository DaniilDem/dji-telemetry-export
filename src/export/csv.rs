//! OVRLEY-compatible CSV.
//!
//! Header names are taken verbatim from OVRLEY's alias registry
//! (`src-tauri/ovrley_core/src/activity/csv/parser.rs`): `elapsed time`,
//! `lateral acceleration`, `longitudinal acceleration`, `vertical acceleration`,
//! `combined acceleration`, `latitude`, `longitude`, `altitude`, `speed`,
//! `heading`, `utc time`, `lean angle`. Parenthesised units are parsed by
//! OVRLEY; unknown columns (roll/pitch/yaw, ISO…) are ignored by it.

use super::{num, opt, ExportOptions};
use crate::process::Telemetry;

pub fn header(telemetry: &Telemetry, options: &ExportOptions) -> Vec<&'static str> {
    let mut h = vec![
        "Elapsed time (s)",
        "Lateral acceleration (g)",
        "Longitudinal acceleration (g)",
        "Vertical acceleration (g)",
        "Combined acceleration (g)",
    ];
    if options.include_raw_axes {
        h.extend(["Accel X (g)", "Accel Y (g)", "Accel Z (g)"]);
    }
    if options.include_orientation {
        h.extend(["Roll (deg)", "Pitch (deg)", "Yaw (deg)"]);
    }
    if options.lean_angle_from_roll {
        h.push("Lean angle (deg)");
    }
    if options.include_exposure {
        h.extend(["ISO", "Shutter (s)", "Color temperature (K)"]);
    }
    if telemetry.has_gps() {
        h.extend([
            "Latitude",
            "Longitude",
            "Altitude (m)",
            "Speed (m/s)",
            "Heading (deg)",
            "Distance (m)",
        ]);
        if telemetry.info.start_utc().is_some() {
            h.push("UTC time");
        }
    }
    h
}

pub fn render(telemetry: &Telemetry, options: &ExportOptions) -> String {
    let header = header(telemetry, options);
    let mut out = String::with_capacity(telemetry.rows.len() * 96);
    out.push_str(&header.join(","));
    out.push('\n');
    let with_gps = telemetry.has_gps();
    let with_utc = with_gps && telemetry.info.start_utc().is_some();

    for r in &telemetry.rows {
        let mut cols: Vec<String> = vec![
            num(r.t, 6),
            opt(r.lateral, 5),
            opt(r.longitudinal, 5),
            opt(r.vertical, 5),
            opt(r.combined, 5),
        ];
        if options.include_raw_axes {
            match r.raw {
                Some(a) => cols.extend([num(a[0], 5), num(a[1], 5), num(a[2], 5)]),
                None => cols.extend([String::new(), String::new(), String::new()]),
            }
        }
        if options.include_orientation {
            cols.extend([opt(r.roll, 2), opt(r.pitch, 2), opt(r.yaw, 2)]);
        }
        if options.lean_angle_from_roll {
            cols.push(opt(r.roll, 2));
        }
        if options.include_exposure {
            cols.push(r.iso.map(|v| format!("{v:.0}")).unwrap_or_default());
            cols.push(r.shutter_seconds().map(|v| format!("{v:.6}")).unwrap_or_default());
            cols.push(r.color_temperature.map(|v| v.to_string()).unwrap_or_default());
        }
        if with_gps {
            cols.extend([
                opt(r.latitude, 7),
                opt(r.longitude, 7),
                opt(r.altitude_m, 2),
                opt(r.speed_ms, 3),
                opt(r.heading_deg, 2),
                opt(r.distance_m, 2),
            ]);
            if with_utc {
                cols.push(
                    telemetry
                        .utc_time(r.t)
                        .map(|dt| dt.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string())
                        .unwrap_or_default(),
                );
            }
        }
        out.push_str(&cols.join(","));
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::testutil::telemetry;
    use super::*;

    /// Mirrors OVRLEY's `normalize_syntax` + `split_header` just enough to prove our
    /// headers resolve to the aliases in its registry.
    fn ovrley_semantic(header: &str) -> String {
        let normalized: String = header
            .trim()
            .chars()
            .map(|c| match c {
                '_' | '-' => ' ',
                o => o.to_ascii_lowercase(),
            })
            .collect::<String>()
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        match normalized.rfind('(') {
            Some(open) if normalized.ends_with(')') => normalized[..open].trim().to_string(),
            _ => normalized,
        }
    }

    const OVRLEY_ALIASES: &[&str] = &[
        "elapsed time",
        "lateral acceleration",
        "longitudinal acceleration",
        "vertical acceleration",
        "combined acceleration",
        "accel x",
        "accel y",
        "accel z",
        "lean angle",
        "latitude",
        "longitude",
        "altitude",
        "speed",
        "heading",
        "distance",
        "utc time",
    ];

    #[test]
    fn ovrley_relevant_headers_resolve_to_known_aliases() {
        let t = telemetry(true);
        let opts = ExportOptions {
            include_raw_axes: true,
            lean_angle_from_roll: true,
            ..ExportOptions::default()
        };
        let known: Vec<String> = header(&t, &opts)
            .into_iter()
            .map(ovrley_semantic)
            .filter(|s| OVRLEY_ALIASES.contains(&s.as_str()))
            .collect();
        for alias in OVRLEY_ALIASES {
            assert!(known.iter().any(|k| k == alias), "missing OVRLEY alias {alias}");
        }
    }

    #[test]
    fn csv_without_gps_has_no_position_columns_and_monotonic_time() {
        let t = telemetry(false);
        let text = render(&t, &ExportOptions::default());
        let mut lines = text.lines();
        let head = lines.next().unwrap();
        assert!(head.starts_with("Elapsed time (s),Lateral acceleration (g),Longitudinal acceleration (g)"));
        assert!(!head.contains("Latitude"));
        assert!(!head.contains("Lean angle"));
        let mut last = -1.0;
        for line in lines {
            let t: f64 = line.split(',').next().unwrap().parse().unwrap();
            assert!(t >= last);
            last = t;
        }
        assert_eq!(text.lines().count(), 151);
    }

    #[test]
    fn csv_with_gps_has_utc_column() {
        let t = telemetry(true);
        let text = render(&t, &ExportOptions::default());
        let head = text.lines().next().unwrap();
        assert!(
            head.ends_with("Latitude,Longitude,Altitude (m),Speed (m/s),Heading (deg),Distance (m),UTC time")
        );
        let row = text.lines().nth(1).unwrap();
        assert!(row.ends_with("2026-09-16T07:59:53.000Z"), "{row}");
        assert!(row.contains(",55.7500000,37.6200000,"));
    }
}
