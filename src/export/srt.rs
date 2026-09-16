//! DJI-style SRT telemetry subtitles ("Format A" in OVRLEY's `srt-parser.js`).
//!
//! ```text
//! 1
//! 00:00:00,000 --> 00:00:00,033
//! <font size="28">FrameCnt: 1, DiffTime: 33ms
//! 2026-09-16 09:59:53.000
//! [iso: 252] [shutter: 1/3111.0] [ct: 5219] [latitude: 55.75] [longitude: 37.62] [rel_alt: 1.200 abs_alt: 151.200] </font>
//! ```
//!
//! Timestamps are the camera's local clock, like DJI's own SRT files; OVRLEY
//! resolves the timezone itself. Only fields the camera actually recorded are
//! written — no fabricated f-number or focal length.

use std::fmt::Write as _;

use crate::process::{Row, Telemetry};

fn srt_time(seconds: f64) -> String {
    let ms = (seconds * 1000.0).round().max(0.0) as u64;
    format!(
        "{:02}:{:02}:{:02},{:03}",
        ms / 3_600_000,
        (ms / 60_000) % 60,
        (ms / 1000) % 60,
        ms % 1000
    )
}

fn shutter_text(row: &Row) -> Option<String> {
    let (num, den) = row.shutter?;
    if num == 0 || den == 0 {
        return None;
    }
    let seconds = num as f64 / den as f64;
    if seconds >= 1.0 {
        Some(format!("{seconds:.1}"))
    } else {
        Some(format!("1/{:.1}", den as f64 / num as f64))
    }
}

pub fn render(telemetry: &Telemetry) -> String {
    let rows = &telemetry.rows;
    let mut out = String::with_capacity(rows.len() * 160);
    let frame_step = telemetry
        .info
        .fps
        .filter(|f| *f > 0.0)
        .map(|f| 1.0 / f64::from(f))
        .unwrap_or(1.0 / 30.0);
    let first_alt = rows.iter().find_map(|r| r.altitude_m);

    for (i, r) in rows.iter().enumerate() {
        let end = rows.get(i + 1).map(|n| n.t).unwrap_or(r.t + frame_step);
        let diff_ms = ((end - r.t) * 1000.0).round().max(0.0) as u64;
        let _ = writeln!(out, "{}", i + 1);
        let _ = writeln!(out, "{} --> {}", srt_time(r.t), srt_time(end));
        let _ = writeln!(
            out,
            "<font size=\"28\">FrameCnt: {}, DiffTime: {diff_ms}ms",
            i + 1
        );
        if let Some(local) = telemetry.local_time(r.t) {
            let _ = writeln!(out, "{}", local.format("%Y-%m-%d %H:%M:%S%.3f"));
        }
        let mut fields: Vec<String> = Vec::new();
        if let Some(iso) = r.iso {
            fields.push(format!("[iso: {iso:.0}]"));
        }
        if let Some(sh) = shutter_text(r) {
            fields.push(format!("[shutter: {sh}]"));
        }
        if let Some(ct) = r.color_temperature {
            fields.push(format!("[ct: {ct}]"));
        }
        if let (Some(lat), Some(lon)) = (r.latitude, r.longitude) {
            fields.push(format!("[latitude: {lat:.6}]"));
            fields.push(format!("[longitude: {lon:.6}]"));
            if let Some(alt) = r.altitude_m {
                let rel = alt - first_alt.unwrap_or(alt);
                fields.push(format!("[rel_alt: {rel:.3} abs_alt: {alt:.3}]"));
            }
        }
        let _ = writeln!(out, "{} </font>", fields.join(" "));
        out.push('\n');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::testutil::telemetry;
    use super::*;

    #[test]
    fn cue_layout_matches_dji_format_a() {
        let t = telemetry(true);
        let text = render(&t);
        let first: Vec<&str> = text.lines().take(5).collect();
        assert_eq!(first[0], "1");
        assert_eq!(first[1], "00:00:00,000 --> 00:00:00,033");
        assert_eq!(first[2], "<font size=\"28\">FrameCnt: 1, DiffTime: 33ms");
        assert_eq!(first[3], "2026-09-16 09:59:53.000");
        assert!(first[4].starts_with("[iso: 100] [shutter: 1/2000.0] [ct: 5500] [latitude: 55.750000] [longitude: 37.620000] [rel_alt: 0.000 abs_alt: 150.000] </font>"), "{}", first[4]);
        // OVRLEY's Format A detector: a `[key:` within the first 3000 chars
        assert!(text[..3000.min(text.len())].contains("[iso:"));
    }

    #[test]
    fn no_gps_means_no_position_fields() {
        let t = telemetry(false);
        let text = render(&t);
        assert!(!text.contains("latitude"));
        assert!(text.contains("[shutter: 1/2000.0]"));
        assert_eq!(text.matches("--> ").count(), 150);
    }

    #[test]
    fn srt_time_formatting() {
        assert_eq!(srt_time(3661.5), "01:01:01,500");
        assert_eq!(srt_time(0.0334), "00:00:00,033");
    }
}
