//! Minimal Garmin FIT activity writer (header, `file_id`, `record`, `session`,
//! `activity`, CRC). Record timestamps are whole seconds, so the caller resamples
//! to 1 Hz. Field numbers and scales follow the public FIT profile.

use crate::process::Telemetry;

const FIT_EPOCH_OFFSET: i64 = 631_065_600; // 1989-12-31T00:00:00Z in Unix seconds

const BASE_ENUM: u8 = 0x00;
const BASE_UINT16: u8 = 0x84;
const BASE_UINT32: u8 = 0x86;
const BASE_SINT32: u8 = 0x85;
const BASE_UINT32Z: u8 = 0x8C;

const MSG_FILE_ID: u16 = 0;
const MSG_SESSION: u16 = 18;
const MSG_RECORD: u16 = 20;
const MSG_ACTIVITY: u16 = 34;

/// CRC-16 as specified by the FIT SDK.
pub fn crc16(data: &[u8]) -> u16 {
    const TABLE: [u16; 16] = [
        0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401, 0xA001, 0x6C00, 0x7800, 0xB401,
        0x5000, 0x9C01, 0x8801, 0x4400,
    ];
    let mut crc: u16 = 0;
    for &byte in data {
        let tmp = TABLE[(crc & 0xF) as usize];
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ TABLE[(byte & 0xF) as usize];
        let tmp = TABLE[(crc & 0xF) as usize];
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ TABLE[((byte >> 4) & 0xF) as usize];
    }
    crc
}

struct FieldDef {
    number: u8,
    size: u8,
    base_type: u8,
}

fn definition(local: u8, global: u16, fields: &[FieldDef], out: &mut Vec<u8>) {
    out.push(0x40 | (local & 0x0F));
    out.push(0); // reserved
    out.push(0); // little endian
    out.extend_from_slice(&global.to_le_bytes());
    out.push(fields.len() as u8);
    for f in fields {
        out.push(f.number);
        out.push(f.size);
        out.push(f.base_type);
    }
}

fn fit_timestamp(dt: chrono::NaiveDateTime) -> u32 {
    (dt.and_utc().timestamp() - FIT_EPOCH_OFFSET).max(0) as u32
}

fn semicircles(deg: f64) -> i32 {
    (deg / 180.0 * 2_147_483_648.0)
        .round()
        .clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

pub fn render(telemetry: &Telemetry) -> Vec<u8> {
    let mut body: Vec<u8> = Vec::with_capacity(telemetry.rows.len() * 32 + 128);

    let start = telemetry
        .utc_time(0.0)
        .or_else(|| telemetry.local_time(0.0))
        .unwrap_or_else(|| {
            chrono::DateTime::from_timestamp(FIT_EPOCH_OFFSET, 0)
                .unwrap()
                .naive_utc()
        });
    let start_ts = fit_timestamp(start);
    let time_at = |t: f64| start_ts.saturating_add(t.floor() as u32);

    // file_id
    definition(
        0,
        MSG_FILE_ID,
        &[
            FieldDef {
                number: 0,
                size: 1,
                base_type: BASE_ENUM,
            }, // type
            FieldDef {
                number: 1,
                size: 2,
                base_type: BASE_UINT16,
            }, // manufacturer
            FieldDef {
                number: 2,
                size: 2,
                base_type: BASE_UINT16,
            }, // product
            FieldDef {
                number: 3,
                size: 4,
                base_type: BASE_UINT32Z,
            }, // serial_number
            FieldDef {
                number: 4,
                size: 4,
                base_type: BASE_UINT32,
            }, // time_created
        ],
        &mut body,
    );
    body.push(0x00);
    body.push(4); // activity
    body.extend_from_slice(&255u16.to_le_bytes()); // manufacturer: development
    body.extend_from_slice(&1u16.to_le_bytes());
    let serial: u32 = telemetry
        .info
        .serial
        .as_deref()
        .map(|s| {
            s.bytes()
                .fold(0u32, |acc, b| acc.wrapping_mul(31).wrapping_add(u32::from(b)))
        })
        .filter(|v| *v != 0)
        .unwrap_or(1);
    body.extend_from_slice(&serial.to_le_bytes());
    body.extend_from_slice(&start_ts.to_le_bytes());

    // record
    definition(
        1,
        MSG_RECORD,
        &[
            FieldDef {
                number: 253,
                size: 4,
                base_type: BASE_UINT32,
            }, // timestamp
            FieldDef {
                number: 0,
                size: 4,
                base_type: BASE_SINT32,
            }, // position_lat
            FieldDef {
                number: 1,
                size: 4,
                base_type: BASE_SINT32,
            }, // position_long
            FieldDef {
                number: 2,
                size: 2,
                base_type: BASE_UINT16,
            }, // altitude (scale 5, offset 500)
            FieldDef {
                number: 5,
                size: 4,
                base_type: BASE_UINT32,
            }, // distance (scale 100)
            FieldDef {
                number: 6,
                size: 2,
                base_type: BASE_UINT16,
            }, // speed (scale 1000)
            FieldDef {
                number: 73,
                size: 4,
                base_type: BASE_UINT32,
            }, // enhanced_speed (scale 1000)
            FieldDef {
                number: 78,
                size: 4,
                base_type: BASE_UINT32,
            }, // enhanced_altitude (scale 5, offset 500)
        ],
        &mut body,
    );
    let gps_rows: Vec<_> = telemetry.rows.iter().filter(|r| r.has_gps()).collect();
    let mut last_ts: Option<u32> = None;
    let mut total_distance = 0.0;
    for r in &gps_rows {
        let ts = time_at(r.t);
        if last_ts == Some(ts) {
            continue;
        }
        last_ts = Some(ts);
        body.push(0x01);
        body.extend_from_slice(&ts.to_le_bytes());
        body.extend_from_slice(&semicircles(r.latitude.unwrap()).to_le_bytes());
        body.extend_from_slice(&semicircles(r.longitude.unwrap()).to_le_bytes());
        let alt = r
            .altitude_m
            .map(|a| ((a + 500.0) * 5.0).round().clamp(0.0, 65534.0) as u16)
            .unwrap_or(0xFFFF);
        body.extend_from_slice(&alt.to_le_bytes());
        let dist = r
            .distance_m
            .map(|d| (d * 100.0).round() as u32)
            .unwrap_or(0xFFFF_FFFF);
        if let Some(d) = r.distance_m {
            total_distance = d;
        }
        body.extend_from_slice(&dist.to_le_bytes());
        let speed = r
            .speed_ms
            .map(|v| (v * 1000.0).round().clamp(0.0, 65534.0) as u16)
            .unwrap_or(0xFFFF);
        body.extend_from_slice(&speed.to_le_bytes());
        let espeed = r
            .speed_ms
            .map(|v| (v * 1000.0).round() as u32)
            .unwrap_or(0xFFFF_FFFF);
        body.extend_from_slice(&espeed.to_le_bytes());
        let ealt = r
            .altitude_m
            .map(|a| ((a + 500.0) * 5.0).round().max(0.0) as u32)
            .unwrap_or(0xFFFF_FFFF);
        body.extend_from_slice(&ealt.to_le_bytes());
    }

    let end_ts = last_ts.unwrap_or(start_ts);
    let elapsed_ms = (u64::from(end_ts - start_ts) * 1000) as u32;

    // session
    definition(
        2,
        MSG_SESSION,
        &[
            FieldDef {
                number: 253,
                size: 4,
                base_type: BASE_UINT32,
            }, // timestamp
            FieldDef {
                number: 2,
                size: 4,
                base_type: BASE_UINT32,
            }, // start_time
            FieldDef {
                number: 7,
                size: 4,
                base_type: BASE_UINT32,
            }, // total_elapsed_time (scale 1000)
            FieldDef {
                number: 8,
                size: 4,
                base_type: BASE_UINT32,
            }, // total_timer_time
            FieldDef {
                number: 9,
                size: 4,
                base_type: BASE_UINT32,
            }, // total_distance (scale 100)
            FieldDef {
                number: 5,
                size: 1,
                base_type: BASE_ENUM,
            }, // sport
            FieldDef {
                number: 6,
                size: 1,
                base_type: BASE_ENUM,
            }, // sub_sport
            FieldDef {
                number: 0,
                size: 1,
                base_type: BASE_ENUM,
            }, // event
            FieldDef {
                number: 1,
                size: 1,
                base_type: BASE_ENUM,
            }, // event_type
        ],
        &mut body,
    );
    body.push(0x02);
    body.extend_from_slice(&end_ts.to_le_bytes());
    body.extend_from_slice(&start_ts.to_le_bytes());
    body.extend_from_slice(&elapsed_ms.to_le_bytes());
    body.extend_from_slice(&elapsed_ms.to_le_bytes());
    body.extend_from_slice(&((total_distance * 100.0).round() as u32).to_le_bytes());
    body.push(10); // sport: motorcycling (closest generic motorsport value)
    body.push(0); // sub_sport: generic
    body.push(8); // event: session
    body.push(1); // event_type: stop

    // activity
    definition(
        3,
        MSG_ACTIVITY,
        &[
            FieldDef {
                number: 253,
                size: 4,
                base_type: BASE_UINT32,
            }, // timestamp
            FieldDef {
                number: 0,
                size: 4,
                base_type: BASE_UINT32,
            }, // total_timer_time
            FieldDef {
                number: 1,
                size: 2,
                base_type: BASE_UINT16,
            }, // num_sessions
            FieldDef {
                number: 2,
                size: 1,
                base_type: BASE_ENUM,
            }, // type
            FieldDef {
                number: 3,
                size: 1,
                base_type: BASE_ENUM,
            }, // event
            FieldDef {
                number: 4,
                size: 1,
                base_type: BASE_ENUM,
            }, // event_type
            FieldDef {
                number: 5,
                size: 4,
                base_type: BASE_UINT32,
            }, // local_timestamp
        ],
        &mut body,
    );
    body.push(0x03);
    body.extend_from_slice(&end_ts.to_le_bytes());
    body.extend_from_slice(&elapsed_ms.to_le_bytes());
    body.extend_from_slice(&1u16.to_le_bytes());
    body.push(0); // manual
    body.push(26); // activity
    body.push(1); // stop
    let local_ts = telemetry
        .local_time(0.0)
        .map(fit_timestamp)
        .unwrap_or(start_ts)
        .saturating_add(end_ts - start_ts);
    body.extend_from_slice(&local_ts.to_le_bytes());

    // header
    let mut file = Vec::with_capacity(body.len() + 16);
    file.push(14);
    file.push(0x20); // protocol 2.0
    file.extend_from_slice(&2132u16.to_le_bytes()); // profile 21.32
    file.extend_from_slice(&(body.len() as u32).to_le_bytes());
    file.extend_from_slice(b".FIT");
    let header_crc = crc16(&file[..12]);
    file.extend_from_slice(&header_crc.to_le_bytes());
    file.extend_from_slice(&body);
    let crc = crc16(&file);
    file.extend_from_slice(&crc.to_le_bytes());
    file
}

#[cfg(test)]
mod tests {
    use super::super::testutil::telemetry;
    use super::super::{resampled, ExportOptions, Format};
    use super::*;

    #[test]
    fn crc_reference_vector() {
        // From the FIT SDK: CRC of a 14-byte header with these fields is verifiable by re-check.
        let data = b"123456789";
        assert_eq!(crc16(data), 0xBB3D);
    }

    #[test]
    fn fit_round_trips_through_fitparser() {
        let t = telemetry(true);
        let t = resampled(&t, Format::Fit, &ExportOptions::default());
        let bytes = render(&t);
        assert_eq!(&bytes[8..12], b".FIT");
        let records = fitparser::from_bytes(&bytes).expect("fitparser accepts the file");
        let recs: Vec<_> = records
            .iter()
            .filter(|r| r.kind() == fitparser::profile::MesgNum::Record)
            .collect();
        assert_eq!(recs.len(), 5);
        let lat = recs[0]
            .fields()
            .iter()
            .find(|f| f.name() == "position_lat")
            .map(|f| f.value().clone())
            .unwrap();
        let lat_deg: f64 = match lat {
            fitparser::Value::Float64(v) => v,
            fitparser::Value::SInt32(v) => f64::from(v) * 180.0 / 2_147_483_648.0,
            other => panic!("unexpected {other:?}"),
        };
        assert!((lat_deg - 55.75).abs() < 1e-4, "{lat_deg}");
        let speed = recs[0]
            .fields()
            .iter()
            .find(|f| f.name() == "speed" || f.name() == "enhanced_speed")
            .unwrap();
        match speed.value() {
            fitparser::Value::Float64(v) => assert!((v - 2.5).abs() < 1e-3),
            other => panic!("unexpected speed {other:?}"),
        }
        assert!(records
            .iter()
            .any(|r| r.kind() == fitparser::profile::MesgNum::Session));
        assert!(records
            .iter()
            .any(|r| r.kind() == fitparser::profile::MesgNum::FileId));
    }
}
