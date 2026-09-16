//! Interpretation of DJI `dvtm_ac20x.proto` (Osmo Action 4/5/6) metadata samples.
//!
//! Field numbers follow `dvtm_library.proto` / `dvtm_oq101.proto` published in
//! AdrianEddy/telemetry-parser and the ExifTool `DJI.pm` Protobuf table:
//!
//! ```text
//! ProductMeta
//!   1 ClipMeta
//!     1 ClipMetaHeader   1 proto name, 2 pb lib version, 3 pb version,
//!                        5 serial, 6 firmware, 10 model
//!   2 StreamMeta
//!     3 VideoStreamMeta  1 width, 2 height, 3 fps (f32)
//!   3 FrameMeta
//!     1 FrameMetaHeader  1 frame_seq_num, 2 frame_timestamp (µs since power-up)
//!     2 FrameMetaOfCamera
//!        3 ISO {1 f32}   4 ExposureTime {1 rational}   6 WhiteBalanceCCT {1 uint}
//!        9 Quaternion {1 w, 2 x, 3 y, 4 z}   10 Accelerometer {2 x, 3 y, 4 z}
//!     4 FrameMetaOfGimbal (AC004 GPS remote)
//!        1 MetaHeaderOfDevice {4 name, 5 f32 rate}
//!        2 GpsBasic {1 PositionCoord {1 unit(0 rad,1 deg), 2 lat f64, 3 lon f64},
//!                    2 altitude mm, 3 status (0 ok, 1 invalid, 2 RTK),
//!                    5 has_time, 6 GpsTime {1 "YYYY-MM-DD HH-MM-SS"}}
//!        3 Velocity {1 vx, 2 vy, 3 vz} m/s
//! ```

use std::io::{Read, Seek};

use chrono::{NaiveDate, NaiveDateTime};

use crate::mp4::{self, Movie, Track};
use crate::protobuf::{get_f32, get_f64, get_i64, get_rational, get_string, get_u64, sub};

/// Clip-level facts collected from the stream.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ClipInfo {
    pub proto_name: Option<String>,
    pub proto_version: Option<String>,
    pub model: Option<String>,
    pub serial: Option<String>,
    pub firmware: Option<String>,
    pub width: Option<u32>,
    pub height: Option<u32>,
    pub fps: Option<f32>,
    /// Name of the GPS remote / gimbal device, e.g. `DJI AC004`.
    pub remote_name: Option<String>,
    /// Advertised remote sample rate in Hz.
    pub remote_rate_hz: Option<f32>,
    /// Camera clock at recording start (naive local time), see [`start_time_source`].
    pub start_local: Option<NaiveDateTime>,
    pub start_time_source: StartTimeSource,
    /// `mvhd` creation time (UTC) when present.
    pub creation_time_utc: Option<NaiveDateTime>,
    /// Camera clock offset from UTC, derived from `start_local - creation_time_utc`
    /// and rounded to 15 minutes. `None` when either side is unknown.
    pub utc_offset_minutes: Option<i32>,
    pub metadata_track: Option<String>,
    pub sample_count: usize,
    pub duration_s: f64,
    pub gps_fix_count: usize,
    /// Distribution of `GpsBasic.gps_status` values (index = status; 0 ok, 1 invalid, 2 RTK).
    pub gps_status_counts: [usize; 3],
    /// Number of samples that carried a GPS time string.
    pub gps_time_count: usize,
}

impl ClipInfo {
    /// Recording start in UTC when the camera's UTC offset is known.
    pub fn start_utc(&self) -> Option<NaiveDateTime> {
        let start = self.start_local?;
        let offset = self.utc_offset_minutes?;
        Some(start - chrono::Duration::minutes(i64::from(offset)))
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum StartTimeSource {
    #[default]
    Unknown,
    /// First GPS time string in the clip, shifted back by that sample's elapsed time.
    GpsTime,
    /// `DJI_YYYYMMDDHHMMSS_*` file name (camera local clock).
    FileName,
    /// `mvhd` creation time.
    Container,
}

impl StartTimeSource {
    pub fn label(self) -> &'static str {
        match self {
            StartTimeSource::Unknown => "unknown",
            StartTimeSource::GpsTime => "GPS time",
            StartTimeSource::FileName => "file name (camera clock)",
            StartTimeSource::Container => "MP4 creation time",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct GpsFix {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: Option<f64>,
    /// 0 normal, 2 RTK (status 1 = invalid never produces a fix).
    pub status: u64,
    /// Velocity x/y/z in m/s when the remote reports it.
    pub velocity: Option<[f64; 3]>,
    /// GPS time string as written by the camera.
    pub time: Option<NaiveDateTime>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Sample {
    /// Seconds since the first sample (from `frame_timestamp`, else the container timeline).
    pub t: f64,
    pub frame: u64,
    /// Raw accelerometer in device units (g on Osmo Action 5 Pro).
    pub acc: Option<[f64; 3]>,
    /// Attitude quaternion `[w, x, y, z]`.
    pub quat: Option<[f64; 4]>,
    pub iso: Option<f32>,
    pub shutter: Option<(u64, u64)>,
    pub color_temperature: Option<u32>,
    pub gps: Option<GpsFix>,
}

#[derive(Debug, Clone, Default)]
pub struct Clip {
    pub info: ClipInfo,
    pub samples: Vec<Sample>,
}

#[derive(Debug, thiserror::Error)]
pub enum DjiError {
    #[error(transparent)]
    Mp4(#[from] mp4::Mp4Error),
    #[error("I/O error while reading samples: {0}")]
    Io(#[from] std::io::Error),
    #[error("no DJI metadata track (`djmd` / handler `DJI meta`) found in this file")]
    NoMetadataTrack,
    #[error("the metadata track has no usable samples")]
    NoSamples,
}

/// Locates the DJI timed-metadata track.
pub fn find_metadata_track(movie: &Movie) -> Option<&Track> {
    movie.tracks.iter().find(|t| t.fourcc == "djmd").or_else(|| {
        movie
            .tracks
            .iter()
            .find(|t| t.handler_name.contains("DJI meta") || t.handler_name.contains("CAM meta"))
    })
}

/// Parses the `DJI_YYYYMMDDHHMMSS_NNNN_X.MP4` naming scheme.
pub fn parse_filename_time(name: &str) -> Option<NaiveDateTime> {
    let stem = name.rsplit(['/', '\\']).next()?;
    let digits = stem.strip_prefix("DJI_")?.get(0..14)?;
    if !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    NaiveDateTime::parse_from_str(digits, "%Y%m%d%H%M%S").ok()
}

/// DJI writes GPS time as `YYYY-MM-DD HH-MM-SS` (sometimes with `:`).
pub fn parse_gps_time(text: &str) -> Option<NaiveDateTime> {
    let text = text.trim();
    let normalized: String = text
        .chars()
        .enumerate()
        .map(|(i, c)| if i >= 11 && c == '-' { ':' } else { c })
        .collect();
    for fmt in [
        "%Y-%m-%d %H:%M:%S%.f",
        "%Y-%m-%d %H:%M:%S",
        "%Y-%m-%dT%H:%M:%S%.f",
    ] {
        if let Ok(dt) = NaiveDateTime::parse_from_str(&normalized, fmt) {
            return Some(dt);
        }
    }
    if let Ok(d) = NaiveDate::parse_from_str(&normalized, "%Y-%m-%d") {
        return d.and_hms_opt(0, 0, 0);
    }
    None
}

/// Reads and interprets every sample of the DJI metadata track.
///
/// `progress(done, total)` is invoked periodically so a UI can show a bar.
pub fn read_clip<R: Read + Seek>(
    reader: &mut R,
    file_name: Option<&str>,
    mut progress: impl FnMut(usize, usize),
) -> Result<Clip, DjiError> {
    let movie = mp4::read_movie(reader)?;
    let track = find_metadata_track(&movie).ok_or(DjiError::NoMetadataTrack)?;
    let total = track.sample_count();
    if total == 0 {
        return Err(DjiError::NoSamples);
    }

    let mut info = ClipInfo {
        creation_time_utc: movie.creation_time,
        metadata_track: Some(format!(
            "{} ({})",
            track.fourcc,
            if track.handler_name.is_empty() {
                track.handler_type.clone()
            } else {
                track.handler_name.clone()
            }
        )),
        ..ClipInfo::default()
    };
    let mut samples = Vec::with_capacity(total);
    let mut base_ts: Option<u64> = None;
    let mut first_gps_time: Option<(f64, NaiveDateTime)> = None;

    for index in 0..total {
        if index % 512 == 0 {
            progress(index, total);
        }
        let payload = track.read_sample(reader, index)?;
        collect_clip_meta(&payload, &mut info);

        let Some(frame) = sub(&payload, &[3]) else {
            continue;
        };
        let header = sub(frame, &[1]);
        let ts_us = header.and_then(|h| get_u64(h, 2));
        let t = match ts_us {
            Some(ts) => {
                let base = *base_ts.get_or_insert(ts);
                (ts as f64 - base as f64) / 1e6
            }
            None => track.times.get(index).copied().unwrap_or(index as f64),
        };
        let frame_no = header.and_then(|h| get_u64(h, 1)).unwrap_or(index as u64);

        let mut sample = Sample {
            t,
            frame: frame_no,
            ..Sample::default()
        };

        if let Some(cam) = sub(frame, &[2]) {
            if let Some(acc) = sub(cam, &[10]) {
                if let (Some(x), Some(y), Some(z)) = (get_f32(acc, 2), get_f32(acc, 3), get_f32(acc, 4)) {
                    sample.acc = Some([f64::from(x), f64::from(y), f64::from(z)]);
                }
            }
            if let Some(q) = sub(cam, &[9]) {
                if let (Some(w), Some(x), Some(y), Some(z)) =
                    (get_f32(q, 1), get_f32(q, 2), get_f32(q, 3), get_f32(q, 4))
                {
                    sample.quat = Some([f64::from(w), f64::from(x), f64::from(y), f64::from(z)]);
                }
            }
            sample.iso = sub(cam, &[3]).and_then(|m| get_f32(m, 1));
            sample.shutter = sub(cam, &[4])
                .and_then(|m| get_rational(m, 1))
                .filter(|&(_, d)| d != 0);
            sample.color_temperature = sub(cam, &[6])
                .and_then(|m| get_u64(m, 1))
                .and_then(|v| u32::try_from(v).ok());
        }

        if let Some(gimbal) = sub(frame, &[4]) {
            if info.remote_name.is_none() {
                if let Some(dev) = sub(gimbal, &[1]) {
                    info.remote_name = get_string(dev, 4);
                    info.remote_rate_hz = get_f32(dev, 5);
                }
            }
            if let Some(gps) = sub(gimbal, &[2]) {
                let status = get_u64(gps, 3).unwrap_or(0);
                if let Some(slot) = info.gps_status_counts.get_mut(status.min(2) as usize) {
                    *slot += 1;
                }
                let time = sub(gps, &[6])
                    .and_then(|m| get_string(m, 1))
                    .and_then(|s| parse_gps_time(&s));
                if time.is_some() {
                    info.gps_time_count += 1;
                }
                if status != 1 {
                    if let Some(coords) = sub(gps, &[1]) {
                        let unit = get_u64(coords, 1).unwrap_or(0);
                        if let (Some(mut lat), Some(mut lon)) = (get_f64(coords, 2), get_f64(coords, 3)) {
                            if unit == 0 {
                                lat = lat.to_degrees();
                                lon = lon.to_degrees();
                            }
                            let valid = lat.is_finite()
                                && lon.is_finite()
                                && !(lat == 0.0 && lon == 0.0)
                                && (-90.0..=90.0).contains(&lat)
                                && (-180.0..=180.0).contains(&lon);
                            if valid {
                                let altitude_m = get_i64(gps, 2).map(|mm| mm as f64 / 1000.0);
                                let velocity = sub(gimbal, &[3]).map(|v| {
                                    [
                                        f64::from(get_f32(v, 1).unwrap_or(0.0)),
                                        f64::from(get_f32(v, 2).unwrap_or(0.0)),
                                        f64::from(get_f32(v, 3).unwrap_or(0.0)),
                                    ]
                                });
                                if let (None, Some(gt)) = (first_gps_time, time) {
                                    first_gps_time = Some((t, gt));
                                }
                                sample.gps = Some(GpsFix {
                                    latitude: lat,
                                    longitude: lon,
                                    altitude_m,
                                    status,
                                    velocity,
                                    time,
                                });
                                info.gps_fix_count += 1;
                            }
                        }
                    }
                }
            }
        }

        samples.push(sample);
    }
    progress(total, total);

    if samples.is_empty() {
        return Err(DjiError::NoSamples);
    }

    info.sample_count = samples.len();
    info.duration_s = samples.last().map(|s| s.t).unwrap_or(0.0);

    // Absolute start time: GPS time > file name > container.
    if let Some((t, gt)) = first_gps_time {
        info.start_local = gt.checked_sub_signed(chrono::Duration::milliseconds((t * 1000.0) as i64));
        info.start_time_source = StartTimeSource::GpsTime;
    } else if let Some(dt) = file_name.and_then(parse_filename_time) {
        info.start_local = Some(dt);
        info.start_time_source = StartTimeSource::FileName;
    } else if let Some(dt) = movie.creation_time {
        info.start_local = Some(dt);
        info.start_time_source = StartTimeSource::Container;
    }
    if let (Some(local), Some(utc)) = (info.start_local, movie.creation_time) {
        let minutes = (local - utc).num_minutes();
        let rounded = ((minutes as f64) / 15.0).round() as i64 * 15;
        if rounded.abs() <= 14 * 60 {
            info.utc_offset_minutes = Some(rounded as i32);
        }
    }

    Ok(Clip { info, samples })
}

/// Pulls clip/stream metadata out of a sample; DJI only writes it into the first one.
fn collect_clip_meta(payload: &[u8], info: &mut ClipInfo) {
    if let Some(header) = sub(payload, &[1, 1]) {
        info.proto_name = info.proto_name.take().or_else(|| get_string(header, 1));
        info.proto_version = info.proto_version.take().or_else(|| get_string(header, 3));
        info.serial = info.serial.take().or_else(|| get_string(header, 5));
        info.firmware = info.firmware.take().or_else(|| get_string(header, 6));
        info.model = info.model.take().or_else(|| get_string(header, 10));
    }
    if let Some(video) = sub(payload, &[2, 3]) {
        if info.width.is_none() {
            info.width = get_u64(video, 1).and_then(|v| u32::try_from(v).ok());
            info.height = get_u64(video, 2).and_then(|v| u32::try_from(v).ok());
            info.fps = get_f32(video, 3);
        }
    }
}

/// Test/fixture helpers that build synthetic `ProductMeta` samples.
pub mod synth {
    use crate::protobuf::encode::*;

    /// Parameters for one synthetic frame.
    #[derive(Debug, Clone, Default)]
    pub struct Frame {
        pub seq: u64,
        pub timestamp_us: u64,
        pub acc: Option<[f32; 3]>,
        pub quat: Option<[f32; 4]>,
        pub iso: Option<f32>,
        pub shutter: Option<(u64, u64)>,
        pub cct: Option<u64>,
        /// `(status, lat, lon, alt_mm, velocity, time)`
        pub gps: Option<Gps>,
        pub with_remote_header: bool,
    }

    #[derive(Debug, Clone)]
    pub struct Gps {
        pub status: u64,
        pub unit_deg: bool,
        pub lat: f64,
        pub lon: f64,
        pub alt_mm: i64,
        pub velocity: Option<[f32; 3]>,
        pub time: Option<String>,
    }

    pub fn clip_header(model: &str, serial: &str, fps: f32) -> Vec<u8> {
        let mut hdr = Vec::new();
        field_str(1, "dvtm_ac204.proto", &mut hdr);
        field_str(2, "02.01.01", &mut hdr);
        field_str(3, "2.0.1", &mut hdr);
        field_str(5, serial, &mut hdr);
        field_str(6, "10.00.16.13", &mut hdr);
        field_str(10, model, &mut hdr);
        let mut clip = Vec::new();
        field_bytes(1, &hdr, &mut clip);

        let mut video = Vec::new();
        field_varint(1, 3840, &mut video);
        field_varint(2, 2160, &mut video);
        field_f32(3, fps, &mut video);
        let mut stream = Vec::new();
        field_bytes(3, &video, &mut stream);

        let mut out = Vec::new();
        field_bytes(1, &clip, &mut out);
        field_bytes(2, &stream, &mut out);
        out
    }

    pub fn frame(f: &Frame) -> Vec<u8> {
        let mut header = Vec::new();
        if f.seq != 0 {
            field_varint(1, f.seq, &mut header);
        }
        field_varint(2, f.timestamp_us, &mut header);

        let mut cam = Vec::new();
        if let Some(iso) = f.iso {
            let mut m = Vec::new();
            field_f32(1, iso, &mut m);
            field_bytes(3, &m, &mut cam);
        }
        if let Some((n, d)) = f.shutter {
            let mut m = Vec::new();
            field_rational(1, n, d, &mut m);
            field_bytes(4, &m, &mut cam);
        }
        if let Some(cct) = f.cct {
            let mut m = Vec::new();
            field_varint(1, cct, &mut m);
            field_bytes(6, &m, &mut cam);
        }
        if let Some(q) = f.quat {
            let mut m = Vec::new();
            for (i, v) in q.iter().enumerate() {
                field_f32(i as u32 + 1, *v, &mut m);
            }
            field_bytes(9, &m, &mut cam);
        }
        if let Some(a) = f.acc {
            let mut m = Vec::new();
            for (i, v) in a.iter().enumerate() {
                field_f32(i as u32 + 2, *v, &mut m);
            }
            field_bytes(10, &m, &mut cam);
        }

        let mut gimbal = Vec::new();
        if f.with_remote_header {
            let mut dev = Vec::new();
            field_varint(1, 1, &mut dev);
            field_varint(2, 1, &mut dev);
            field_str(4, "DJI AC004", &mut dev);
            field_f32(5, 29.97, &mut dev);
            field_bytes(1, &dev, &mut gimbal);
        }
        if let Some(g) = &f.gps {
            let mut basic = Vec::new();
            if g.status != 1 {
                let mut coords = Vec::new();
                field_varint(1, u64::from(g.unit_deg), &mut coords);
                field_f64(2, g.lat, &mut coords);
                field_f64(3, g.lon, &mut coords);
                field_bytes(1, &coords, &mut basic);
                field_varint(2, g.alt_mm as u64, &mut basic);
            }
            field_varint(3, g.status, &mut basic);
            if let Some(t) = &g.time {
                field_varint(5, 1, &mut basic);
                let mut tm = Vec::new();
                field_str(1, t, &mut tm);
                field_bytes(6, &tm, &mut basic);
            }
            field_bytes(2, &basic, &mut gimbal);
            if let Some(v) = g.velocity {
                let mut vel = Vec::new();
                for (i, x) in v.iter().enumerate() {
                    field_f32(i as u32 + 1, *x, &mut vel);
                }
                field_bytes(3, &vel, &mut gimbal);
            }
        }

        let mut frame = Vec::new();
        field_bytes(1, &header, &mut frame);
        field_bytes(2, &cam, &mut frame);
        if !gimbal.is_empty() {
            field_bytes(4, &gimbal, &mut frame);
        }
        let mut out = Vec::new();
        field_bytes(3, &frame, &mut out);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::synth::{clip_header, frame, Frame, Gps};
    use super::*;
    use crate::mp4::builder::{build, TrackSpec};
    use std::io::Cursor;

    fn make_clip(frames: &[Frame]) -> Vec<Vec<u8>> {
        frames
            .iter()
            .enumerate()
            .map(|(i, f)| {
                let mut s = if i == 0 {
                    clip_header("DJI OsmoAction5 Pro", "SERIAL", 29.97)
                } else {
                    Vec::new()
                };
                s.extend(frame(f));
                s
            })
            .collect()
    }

    fn movie_bytes(samples: &[Vec<u8>]) -> Vec<u8> {
        build(
            &[TrackSpec {
                fourcc: b"djmd",
                handler_type: b"meta",
                handler_name: "DJI meta",
                timescale: 30000,
                sample_delta: 1001,
                samples,
                chunk_size: 4,
            }],
            0,
        )
    }

    #[test]
    fn reads_accelerometer_only_clip() {
        let frames: Vec<Frame> = (0..10)
            .map(|i| Frame {
                seq: i,
                timestamp_us: 2_000_000_000 + i * 33_366,
                acc: Some([-1.0, 0.05, -0.2]),
                quat: Some([0.8, 0.0, -0.58, 0.05]),
                iso: Some(252.0),
                shutter: Some((1, 3111)),
                cct: Some(5219),
                gps: Some(Gps {
                    status: 1,
                    unit_deg: true,
                    lat: 0.0,
                    lon: 0.0,
                    alt_mm: 0,
                    velocity: None,
                    time: None,
                }),
                with_remote_header: true,
            })
            .collect();
        let bytes = movie_bytes(&make_clip(&frames));
        let clip = read_clip(
            &mut Cursor::new(bytes),
            Some("DJI_20260916095953_0098_D.MP4"),
            |_, _| {},
        )
        .unwrap();
        assert_eq!(clip.samples.len(), 10);
        assert_eq!(clip.info.model.as_deref(), Some("DJI OsmoAction5 Pro"));
        assert_eq!(clip.info.proto_name.as_deref(), Some("dvtm_ac204.proto"));
        assert_eq!(clip.info.remote_name.as_deref(), Some("DJI AC004"));
        assert_eq!(clip.info.width, Some(3840));
        assert_eq!(clip.info.gps_fix_count, 0);
        assert_eq!(clip.info.gps_status_counts, [0, 10, 0]);
        assert_eq!(clip.info.start_time_source, StartTimeSource::FileName);
        assert_eq!(
            clip.info
                .start_local
                .unwrap()
                .format("%Y-%m-%d %H:%M:%S")
                .to_string(),
            "2026-09-16 09:59:53"
        );
        let s = &clip.samples[3];
        assert!((s.t - 3.0 * 0.033366).abs() < 1e-9);
        assert_eq!(s.acc, Some([-1.0, 0.05000000074505806, -0.20000000298023224]));
        assert_eq!(s.shutter, Some((1, 3111)));
        assert_eq!(s.color_temperature, Some(5219));
        assert_eq!(s.iso, Some(252.0));
        assert!(s.gps.is_none());
    }

    #[test]
    fn reads_gps_fixes_and_derives_start_from_gps_time() {
        let frames: Vec<Frame> = (0..6)
            .map(|i| Frame {
                seq: i,
                timestamp_us: 1_000_000 + i * 100_000,
                acc: Some([0.0, 0.0, 1.0]),
                gps: Some(Gps {
                    status: if i < 2 { 1 } else { 0 },
                    unit_deg: i % 2 == 0,
                    lat: if i % 2 == 0 { 55.75 } else { 55.75f64.to_radians() },
                    lon: if i % 2 == 0 { 37.62 } else { 37.62f64.to_radians() },
                    alt_mm: 150_500,
                    velocity: Some([3.0, 4.0, 0.0]),
                    time: Some("2026-09-16 07-00-10".to_string()),
                }),
                with_remote_header: true,
                ..Frame::default()
            })
            .collect();
        let bytes = movie_bytes(&make_clip(&frames));
        let clip = read_clip(&mut Cursor::new(bytes), Some("other.mp4"), |_, _| {}).unwrap();
        assert_eq!(clip.info.gps_fix_count, 4);
        assert_eq!(clip.info.gps_status_counts, [4, 2, 0]);
        for s in &clip.samples[2..] {
            let g = s.gps.as_ref().unwrap();
            assert!((g.latitude - 55.75).abs() < 1e-9, "{}", g.latitude);
            assert!((g.longitude - 37.62).abs() < 1e-9);
            assert_eq!(g.altitude_m, Some(150.5));
            assert_eq!(g.velocity, Some([3.0, 4.0, 0.0]));
        }
        assert_eq!(clip.info.start_time_source, StartTimeSource::GpsTime);
        // first fix at t = 0.2 s with GPS time 07:00:10 -> start 07:00:09.800
        assert_eq!(
            clip.info.start_local.unwrap().format("%H:%M:%S%.3f").to_string(),
            "07:00:09.800"
        );
    }

    #[test]
    fn filename_time_parsing() {
        assert_eq!(
            parse_filename_time("D:\\clips\\DJI_20260916095953_0098_D.MP4")
                .unwrap()
                .to_string(),
            "2026-09-16 09:59:53"
        );
        assert!(parse_filename_time("GX010001.MP4").is_none());
        assert!(parse_filename_time("DJI_2026.MP4").is_none());
    }

    #[test]
    fn gps_time_parsing() {
        assert_eq!(
            parse_gps_time("2026-09-16 07-00-10").unwrap().to_string(),
            "2026-09-16 07:00:10"
        );
        assert_eq!(
            parse_gps_time("2026-09-16 07:00:10").unwrap().to_string(),
            "2026-09-16 07:00:10"
        );
        assert!(parse_gps_time("").is_none());
    }
}
