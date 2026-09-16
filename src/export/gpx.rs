//! GPX 1.1 track. OVRLEY reads `<trkpt lat lon>`, `<ele>`, `<time>` and any leaf
//! element under `<extensions>` by its local name (`speed`, `heading`, `g_force`, …).

use std::fmt::Write as _;

use super::{num, xml_escape, APP_NAME, APP_VERSION};
use crate::process::Telemetry;

fn iso_time(telemetry: &Telemetry, t: f64) -> Option<String> {
    // Prefer true UTC; fall back to the camera clock labelled as UTC (documented caveat).
    telemetry
        .utc_time(t)
        .or_else(|| telemetry.local_time(t))
        .map(|dt| dt.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string())
}

pub fn render(telemetry: &Telemetry, source_name: &str) -> String {
    let mut out = String::with_capacity(telemetry.rows.len() * 220 + 512);
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    let _ = writeln!(
        out,
        "<gpx version=\"1.1\" creator=\"{APP_NAME} {APP_VERSION}\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">"
    );
    let name = xml_escape(source_name.rsplit(['/', '\\']).next().unwrap_or(source_name));
    out.push_str("  <metadata>\n");
    let _ = writeln!(out, "    <name>{name}</name>");
    if let Some(t) = iso_time(telemetry, 0.0) {
        let _ = writeln!(out, "    <time>{t}</time>");
    }
    out.push_str("  </metadata>\n  <trk>\n");
    let _ = writeln!(out, "    <name>{name}</name>");
    if let Some(model) = &telemetry.info.model {
        let _ = writeln!(out, "    <src>{}</src>", xml_escape(model));
    }
    out.push_str("    <trkseg>\n");

    for r in telemetry.rows.iter().filter(|r| r.has_gps()) {
        let _ = writeln!(
            out,
            "      <trkpt lat=\"{}\" lon=\"{}\">",
            num(r.latitude.unwrap(), 7),
            num(r.longitude.unwrap(), 7)
        );
        if let Some(alt) = r.altitude_m {
            let _ = writeln!(out, "        <ele>{}</ele>", num(alt, 2));
        }
        if let Some(t) = iso_time(telemetry, r.t) {
            let _ = writeln!(out, "        <time>{t}</time>");
        }
        out.push_str("        <extensions>\n");
        let _ = writeln!(out, "          <elapsed_time>{}</elapsed_time>", num(r.t, 3));
        if let Some(v) = r.speed_ms {
            let _ = writeln!(out, "          <speed>{}</speed>", num(v, 3));
        }
        if let Some(v) = r.heading_deg {
            let _ = writeln!(out, "          <heading>{}</heading>", num(v, 2));
        }
        if let Some(v) = r.distance_m {
            let _ = writeln!(out, "          <distance>{}</distance>", num(v, 2));
        }
        if let Some(v) = r.combined {
            let _ = writeln!(out, "          <g_force>{}</g_force>", num(v, 4));
        }
        if let Some(v) = r.lateral {
            let _ = writeln!(out, "          <lateral_g>{}</lateral_g>", num(v, 4));
        }
        if let Some(v) = r.longitudinal {
            let _ = writeln!(out, "          <longitudinal_g>{}</longitudinal_g>", num(v, 4));
        }
        if let Some(v) = r.vertical {
            let _ = writeln!(out, "          <vertical_g>{}</vertical_g>", num(v, 4));
        }
        out.push_str("        </extensions>\n      </trkpt>\n");
    }
    out.push_str("    </trkseg>\n  </trk>\n</gpx>\n");
    out
}

#[cfg(test)]
mod tests {
    use super::super::testutil::telemetry;
    use super::super::{resampled, ExportOptions, Format};
    use super::*;

    #[test]
    fn produces_well_formed_gpx_with_extensions() {
        let t = telemetry(true);
        let t = resampled(&t, Format::Gpx, &ExportOptions::default());
        let text = render(&t, "D:\\clips\\DJI_0001.MP4");
        let doc = roxmltree::Document::parse(&text).expect("valid XML");
        let pts: Vec<_> = doc.descendants().filter(|n| n.has_tag_name("trkpt")).collect();
        assert_eq!(pts.len(), 50, "10 Hz over 5 s");
        let first = pts[0];
        assert_eq!(first.attribute("lat"), Some("55.7500000"));
        let time = first
            .descendants()
            .find(|n| n.has_tag_name("time"))
            .unwrap()
            .text()
            .unwrap();
        assert_eq!(time, "2026-09-16T07:59:53.000Z");
        let speed = first
            .descendants()
            .find(|n| n.has_tag_name("speed"))
            .unwrap()
            .text()
            .unwrap();
        assert_eq!(speed, "2.500");
        assert!(first.descendants().any(|n| n.has_tag_name("g_force")));
        assert!(doc
            .descendants()
            .any(|n| n.has_tag_name("name") && n.text() == Some("DJI_0001.MP4")));
    }
}
