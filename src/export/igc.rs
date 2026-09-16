//! IGC flight log. One `B` record per second (IGC time resolution) with the
//! `GSP` (ground speed, 0.01 km/h) and `TRT` (true track, degrees) extensions
//! that OVRLEY's IGC importer understands.

use std::fmt::Write as _;

use chrono::{Datelike, NaiveDateTime, Timelike};

use super::{APP_NAME, APP_VERSION};
use crate::process::Telemetry;

fn time_for(telemetry: &Telemetry, t: f64) -> Option<NaiveDateTime> {
    telemetry.utc_time(t).or_else(|| telemetry.local_time(t))
}

fn lat_field(lat: f64) -> String {
    let hemi = if lat >= 0.0 { 'N' } else { 'S' };
    let lat = lat.abs();
    let deg = lat.floor();
    let thousandths = ((lat - deg) * 60.0 * 1000.0).round() as u32;
    let (deg, thousandths) = if thousandths >= 60_000 {
        (deg + 1.0, 0)
    } else {
        (deg, thousandths)
    };
    format!("{:02}{:05}{hemi}", deg as u32, thousandths)
}

fn lon_field(lon: f64) -> String {
    let hemi = if lon >= 0.0 { 'E' } else { 'W' };
    let lon = lon.abs();
    let deg = lon.floor();
    let thousandths = ((lon - deg) * 60.0 * 1000.0).round() as u32;
    let (deg, thousandths) = if thousandths >= 60_000 {
        (deg + 1.0, 0)
    } else {
        (deg, thousandths)
    };
    format!("{:03}{:05}{hemi}", deg as u32, thousandths)
}

fn alt_field(alt: f64) -> String {
    let v = alt.round().clamp(-9999.0, 99999.0) as i64;
    if v < 0 {
        format!("-{:04}", -v)
    } else {
        format!("{v:05}")
    }
}

pub fn render(telemetry: &Telemetry, source_name: &str) -> String {
    let info = &telemetry.info;
    let mut out = String::with_capacity(telemetry.rows.len() * 48 + 512);
    let base = source_name.rsplit(['/', '\\']).next().unwrap_or(source_name);

    let first_fix_t = telemetry
        .rows
        .iter()
        .find(|r| r.has_gps())
        .map(|r| r.t)
        .unwrap_or(0.0);
    let start = time_for(telemetry, first_fix_t);

    let _ = writeln!(out, "AXXX{}", APP_NAME.to_ascii_uppercase());
    if let Some(s) = start {
        let _ = writeln!(out, "HFDTE{:02}{:02}{:02}", s.day(), s.month(), s.year() % 100);
    }
    out.push_str("HFFXA010\n");
    out.push_str("HFPLTPILOTINCHARGE:\n");
    let _ = writeln!(
        out,
        "HFGTYGLIDERTYPE:{}",
        info.model.as_deref().unwrap_or("DJI camera")
    );
    let _ = writeln!(out, "HFGIDGLIDERID:{}", info.serial.as_deref().unwrap_or(""));
    out.push_str("HFDTMGPSDATUM:WGS84\n");
    if let Some(fw) = &info.firmware {
        let _ = writeln!(out, "HFRFWFIRMWAREVERSION:{fw}");
    }
    if let Some(remote) = &info.remote_name {
        let _ = writeln!(out, "HFRHWHARDWAREVERSION:{remote}");
        let _ = writeln!(out, "HFGPSRECEIVER:{remote}");
    }
    let _ = writeln!(out, "HFFTYFRTYPE:{APP_NAME},{APP_VERSION}");
    out.push_str("HFPRSPRESSALTSENSOR:NONE\n");
    let _ = writeln!(out, "HFCIDCOMPETITIONID:{base}");
    if telemetry.utc_time(0.0).is_none() {
        out.push_str("LXXXTIMES ARE CAMERA LOCAL TIME (UTC OFFSET UNKNOWN)\n");
    }
    // Extensions: bytes 36-40 GSP (5 digits), 41-43 TRT (3 digits)
    out.push_str("I023640GSP4143TRT\n");

    let mut last_second: Option<i64> = None;
    for r in telemetry.rows.iter().filter(|r| r.has_gps()) {
        let Some(when) = time_for(telemetry, r.t) else {
            continue;
        };
        let second = when.and_utc().timestamp();
        if last_second == Some(second) {
            continue;
        }
        last_second = Some(second);
        let alt = r.altitude_m.unwrap_or(0.0);
        let gsp = (r.speed_ms.unwrap_or(0.0) * 3.6 * 100.0)
            .round()
            .clamp(0.0, 99999.0) as u32;
        let trt = (r.heading_deg.unwrap_or(0.0).round() as i64).rem_euclid(360) as u32;
        let _ = writeln!(
            out,
            "B{:02}{:02}{:02}{}{}A{}{}{gsp:05}{trt:03}",
            when.hour(),
            when.minute(),
            when.second(),
            lat_field(r.latitude.unwrap()),
            lon_field(r.longitude.unwrap()),
            alt_field(alt),
            alt_field(alt),
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::super::testutil::telemetry;
    use super::super::{resampled, ExportOptions, Format};
    use super::*;

    #[test]
    fn coordinate_fields() {
        assert_eq!(lat_field(55.75), "5545000N");
        assert_eq!(lat_field(-33.8688), "3352128S");
        assert_eq!(lon_field(37.62), "03737200E");
        assert_eq!(lon_field(-1.209534), "00112572W");
        assert_eq!(alt_field(150.4), "00150");
        assert_eq!(alt_field(-44.8), "-0045");
    }

    #[test]
    fn b_records_are_one_per_second_and_fixed_width() {
        let t = telemetry(true);
        let t = resampled(&t, Format::Igc, &ExportOptions::default());
        let text = render(&t, "clip.MP4");
        assert!(text.starts_with("AXXXDJI-TELEMETRY-EXPORT\nHFDTE160926\n"));
        assert!(text.contains("I023640GSP4143TRT\n"));
        let b: Vec<&str> = text.lines().filter(|l| l.starts_with('B')).collect();
        assert_eq!(b.len(), 5);
        for line in &b {
            assert_eq!(line.len(), 43, "{line}");
        }
        // 07:59:53 UTC, 55.75N 37.62E, alt 150, speed 2.5 m/s = 9 km/h => GSP 00900, heading 53°
        assert_eq!(b[0], "B0759535545000N03737200EA001500015000900053");
        assert!(b[1].starts_with("B075954"));
    }
}
