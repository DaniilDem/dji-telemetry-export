//! End-to-end test on 300 real `djmd` samples cut from a DJI Osmo Action 5 Pro clip
//! (`tools/extract_fixture.py`), wrapped into a synthetic MP4 container.

use std::io::Cursor;

use dji_telemetry_export::dji::{read_clip, StartTimeSource};
use dji_telemetry_export::export::{self, ExportOptions, Format};
use dji_telemetry_export::mp4::builder::{build, TrackSpec};
use dji_telemetry_export::process::{self, AccelUnit, Axis, GravityMode, ProcessOptions};

fn fixture_samples() -> Vec<Vec<u8>> {
    let bytes = include_bytes!("fixtures/ac204_djmd_first300.bin");
    let mut out = Vec::new();
    let mut pos = 0;
    while pos + 4 <= bytes.len() {
        let len = u32::from_be_bytes([bytes[pos], bytes[pos + 1], bytes[pos + 2], bytes[pos + 3]]) as usize;
        pos += 4;
        out.push(bytes[pos..pos + len].to_vec());
        pos += len;
    }
    out
}

fn fixture_movie() -> Vec<u8> {
    let samples = fixture_samples();
    // A dummy video track first, like the real file, so the metadata track is not track 1.
    let video: Vec<Vec<u8>> = samples.iter().map(|_| vec![0u8; 16]).collect();
    build(
        &[
            TrackSpec {
                fourcc: b"hvc1",
                handler_type: b"vide",
                handler_name: "VideoHandler",
                timescale: 30000,
                sample_delta: 1001,
                samples: &video,
                chunk_size: 5,
            },
            TrackSpec {
                fourcc: b"djmd",
                handler_type: b"meta",
                handler_name: "DJI meta",
                timescale: 30000,
                sample_delta: 1001,
                samples: &samples,
                chunk_size: 3,
            },
        ],
        2_082_844_800 + 1_789_545_593, // 2026-09-16T07:59:53Z
    )
}

#[test]
fn real_osmo_action_5_pro_samples_decode() {
    let clip = read_clip(
        &mut Cursor::new(fixture_movie()),
        Some("DJI_20260916095953_0098_D.MP4"),
        |_, _| {},
    )
    .unwrap();
    let info = &clip.info;
    assert_eq!(clip.samples.len(), 300);
    assert_eq!(info.model.as_deref(), Some("DJI OsmoAction5 Pro"));
    assert_eq!(info.proto_name.as_deref(), Some("dvtm_ac204.proto"));
    assert_eq!(info.proto_version.as_deref(), Some("2.0.1"));
    assert_eq!(info.firmware.as_deref(), Some("10.00.16.13"));
    assert_eq!((info.width, info.height), (Some(3840), Some(2160)));
    assert_eq!(info.remote_name.as_deref(), Some("DJI AC004"));
    assert_eq!(info.gps_fix_count, 0);
    assert_eq!(info.gps_status_counts, [0, 300, 0]);
    assert_eq!(info.start_time_source, StartTimeSource::FileName);
    assert_eq!(info.utc_offset_minutes, Some(120));

    let s0 = &clip.samples[0];
    assert_eq!(s0.t, 0.0);
    assert_eq!(s0.iso, Some(252.0));
    assert_eq!(s0.shutter, Some((1, 3111)));
    assert_eq!(s0.color_temperature, Some(5219));
    let acc = s0.acc.unwrap();
    assert!(
        (acc[0] - -1.011326).abs() < 1e-5
            && (acc[1] - 0.04635983).abs() < 1e-6
            && (acc[2] - -0.21425033).abs() < 1e-6
    );
    let q = s0.quat.unwrap();
    assert!((q[0] - 0.80715305).abs() < 1e-6);

    // 29.97 fps -> 33.366 ms between frame timestamps
    let dt = clip.samples[1].t - clip.samples[0].t;
    assert!((dt - 0.033365).abs() < 1e-5, "{dt}");
    assert!((clip.samples[299].t - 299.0 * 0.0333655).abs() < 1e-3);

    let (unit, median) = process::detect_accel_unit(&clip.samples);
    assert_eq!(unit, AccelUnit::G);
    assert!(median > 0.9 && median < 1.3, "{median}");
}

#[test]
fn real_samples_process_and_export_without_gps() {
    let clip = read_clip(
        &mut Cursor::new(fixture_movie()),
        Some("DJI_20260916095953_0098_D.MP4"),
        |_, _| {},
    )
    .unwrap();
    let medians = process::axis_medians(&clip.samples, 1.0);
    let axes = process::auto_axis_map(medians);
    // At the start of this clip the camera lies with X pointing down.
    assert_eq!(axes.vertical, Axis::X);
    assert!(axes.invert_vertical);

    let telemetry = process::process(
        &clip,
        &ProcessOptions {
            axes,
            ..ProcessOptions::default()
        },
    );
    assert_eq!(telemetry.gravity_used, GravityMode::Quaternion);
    let g = telemetry.gravity_world.unwrap();
    let norm = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
    assert!((norm - 1.0).abs() < 0.05, "world gravity magnitude {norm}");
    // After removal the vertical channel should hover around zero, not around ±1 g.
    let mean_vertical: f64 =
        telemetry.rows.iter().filter_map(|r| r.vertical).sum::<f64>() / telemetry.rows.len() as f64;
    assert!(mean_vertical.abs() < 0.1, "{mean_vertical}");

    let opts = ExportOptions::default();
    for f in [Format::Csv, Format::Srt, Format::Vbo] {
        let bytes = export::render(f, &telemetry, &opts, "DJI_20260916095953_0098_D.MP4").unwrap();
        assert!(!bytes.is_empty());
    }
    for f in [Format::Gpx, Format::Fit, Format::Igc] {
        let err = export::render(f, &telemetry, &opts, "x").unwrap_err();
        assert!(err.to_string().contains("GPS_INVALID in 300/300"), "{err}");
    }
    let csv = String::from_utf8(export::render(Format::Csv, &telemetry, &opts, "x").unwrap()).unwrap();
    assert_eq!(csv.lines().count(), 301);
    assert!(csv.starts_with("Elapsed time (s),Lateral acceleration (g),"));
}
