//! Just enough ISO BMFF (MP4) parsing to locate the samples of a metadata track.
//!
//! Only box headers and the `moov` atom are read into memory; individual samples
//! are fetched later with seek+read, so a 7 GB clip costs a few megabytes of I/O.

use std::io::{self, Read, Seek, SeekFrom};

use chrono::NaiveDateTime;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Mp4Error {
    #[error("I/O error: {0}")]
    Io(#[from] io::Error),
    #[error("not an MP4/MOV file: no `moov` box found")]
    NoMoov,
    #[error("malformed MP4: {0}")]
    Malformed(&'static str),
}

pub type Result<T> = std::result::Result<T, Mp4Error>;

/// Top-level box location.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoxHeader {
    pub kind: [u8; 4],
    /// File offset of the box start (its size field).
    pub offset: u64,
    /// 8 or 16 bytes.
    pub header_len: u64,
    /// Total box size including the header.
    pub size: u64,
}

impl BoxHeader {
    pub fn kind_str(&self) -> String {
        String::from_utf8_lossy(&self.kind).into_owned()
    }
    pub fn payload_offset(&self) -> u64 {
        self.offset + self.header_len
    }
    pub fn payload_len(&self) -> u64 {
        self.size - self.header_len
    }
}

/// One track's sample table, already flattened to per-sample offsets and times.
#[derive(Debug, Clone, Default)]
pub struct Track {
    pub track_id: u32,
    /// `hdlr` handler type, e.g. `vide`, `soun`, `meta`.
    pub handler_type: String,
    /// `hdlr` human-readable name, e.g. `DJI meta`.
    pub handler_name: String,
    /// First `stsd` sample-entry format, e.g. `hvc1`, `djmd`.
    pub fourcc: String,
    pub timescale: u32,
    pub offsets: Vec<u64>,
    pub sizes: Vec<u32>,
    /// Decode time of each sample in seconds on the media timeline.
    pub times: Vec<f64>,
}

impl Track {
    pub fn sample_count(&self) -> usize {
        self.offsets.len()
    }

    pub fn duration_s(&self) -> f64 {
        self.times.last().copied().unwrap_or(0.0)
    }

    /// Reads one sample's raw bytes.
    pub fn read_sample<R: Read + Seek>(&self, reader: &mut R, index: usize) -> io::Result<Vec<u8>> {
        let offset = self.offsets[index];
        let size = self.sizes[index] as usize;
        reader.seek(SeekFrom::Start(offset))?;
        let mut buf = vec![0u8; size];
        reader.read_exact(&mut buf)?;
        Ok(buf)
    }
}

#[derive(Debug, Clone, Default)]
pub struct Movie {
    /// `mvhd` creation time (UTC, as written by the camera).
    pub creation_time: Option<NaiveDateTime>,
    pub duration_s: f64,
    pub tracks: Vec<Track>,
    /// Raw `udta` payload for optional vendor lookups.
    pub udta: Vec<u8>,
}

impl Movie {
    pub fn find_track(&self, fourcc: &str) -> Option<&Track> {
        self.tracks.iter().find(|t| t.fourcc == fourcc)
    }
}

/// Lists the top-level boxes of the file by reading only their headers.
pub fn top_level_boxes<R: Read + Seek>(reader: &mut R) -> Result<Vec<BoxHeader>> {
    let file_len = reader.seek(SeekFrom::End(0))?;
    let mut boxes = Vec::new();
    let mut pos = 0u64;
    while pos + 8 <= file_len {
        reader.seek(SeekFrom::Start(pos))?;
        let mut hdr = [0u8; 8];
        reader.read_exact(&mut hdr)?;
        let mut size = u64::from(u32::from_be_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]));
        let kind = [hdr[4], hdr[5], hdr[6], hdr[7]];
        let mut header_len = 8u64;
        if size == 1 {
            let mut large = [0u8; 8];
            reader.read_exact(&mut large)?;
            size = u64::from_be_bytes(large);
            header_len = 16;
        } else if size == 0 {
            size = file_len - pos;
        }
        if size < header_len || pos + size > file_len {
            // Tolerate a damaged tail (e.g. a clip that was still being written).
            break;
        }
        boxes.push(BoxHeader {
            kind,
            offset: pos,
            header_len,
            size,
        });
        pos += size;
    }
    Ok(boxes)
}

/// Reads and parses the `moov` box.
pub fn read_movie<R: Read + Seek>(reader: &mut R) -> Result<Movie> {
    let boxes = top_level_boxes(reader)?;
    let moov = boxes
        .iter()
        .find(|b| &b.kind == b"moov")
        .ok_or(Mp4Error::NoMoov)?;
    if moov.payload_len() > 512 * 1024 * 1024 {
        return Err(Mp4Error::Malformed("moov box is unreasonably large"));
    }
    reader.seek(SeekFrom::Start(moov.payload_offset()))?;
    let mut buf = vec![0u8; moov.payload_len() as usize];
    reader.read_exact(&mut buf)?;
    parse_moov(&buf)
}

/// Iterates `(kind, payload_start, payload_end)` over the boxes in `buf[start..end]`.
fn iter_boxes(buf: &[u8], start: usize, end: usize) -> Vec<([u8; 4], usize, usize)> {
    let mut out = Vec::new();
    let mut pos = start;
    while pos + 8 <= end {
        let mut size = u64::from(be_u32(buf, pos));
        let kind = [buf[pos + 4], buf[pos + 5], buf[pos + 6], buf[pos + 7]];
        let mut header = 8usize;
        if size == 1 {
            if pos + 16 > end {
                break;
            }
            size = be_u64(buf, pos + 8);
            header = 16;
        } else if size == 0 {
            size = (end - pos) as u64;
        }
        let size = match usize::try_from(size) {
            Ok(s) => s,
            Err(_) => break,
        };
        if size < header || pos + size > end {
            break;
        }
        out.push((kind, pos + header, pos + size));
        pos += size;
    }
    out
}

fn find_boxes(buf: &[u8], path: &[&[u8; 4]], start: usize, end: usize) -> Vec<(usize, usize)> {
    let (head, rest) = match path.split_first() {
        Some(x) => x,
        None => return vec![(start, end)],
    };
    let mut out = Vec::new();
    for (kind, s, e) in iter_boxes(buf, start, end) {
        if &kind != *head {
            continue;
        }
        if rest.is_empty() {
            out.push((s, e));
        } else {
            out.extend(find_boxes(buf, rest, s, e));
        }
    }
    out
}

fn be_u32(buf: &[u8], pos: usize) -> u32 {
    u32::from_be_bytes([buf[pos], buf[pos + 1], buf[pos + 2], buf[pos + 3]])
}

fn be_u64(buf: &[u8], pos: usize) -> u64 {
    let mut b = [0u8; 8];
    b.copy_from_slice(&buf[pos..pos + 8]);
    u64::from_be_bytes(b)
}

/// Seconds between 1904-01-01 (MP4 epoch) and 1970-01-01.
const MP4_EPOCH_OFFSET: i64 = 2_082_844_800;

fn mp4_time(seconds: u64) -> Option<NaiveDateTime> {
    if seconds == 0 {
        return None;
    }
    chrono::DateTime::from_timestamp(seconds as i64 - MP4_EPOCH_OFFSET, 0).map(|dt| dt.naive_utc())
}

fn parse_moov(buf: &[u8]) -> Result<Movie> {
    let mut movie = Movie::default();
    let end = buf.len();

    if let Some(&(s, e)) = find_boxes(buf, &[b"mvhd"], 0, end).first() {
        if e - s >= 20 {
            let version = buf[s];
            let (creation, timescale, duration) = if version == 1 && e - s >= 32 {
                (be_u64(buf, s + 4), be_u32(buf, s + 20), be_u64(buf, s + 24))
            } else {
                (
                    u64::from(be_u32(buf, s + 4)),
                    be_u32(buf, s + 12),
                    u64::from(be_u32(buf, s + 16)),
                )
            };
            movie.creation_time = mp4_time(creation);
            if timescale > 0 {
                movie.duration_s = duration as f64 / f64::from(timescale);
            }
        }
    }

    if let Some(&(s, e)) = find_boxes(buf, &[b"udta"], 0, end).first() {
        movie.udta = buf[s..e].to_vec();
    }

    for (trak_s, trak_e) in find_boxes(buf, &[b"trak"], 0, end) {
        let mut track = Track::default();

        if let Some(&(s, e)) = find_boxes(buf, &[b"tkhd"], trak_s, trak_e).first() {
            if e - s >= 16 {
                let version = buf[s];
                track.track_id = if version == 1 {
                    be_u32(buf, s + 20)
                } else {
                    be_u32(buf, s + 12)
                };
            }
        }

        if let Some(&(s, e)) = find_boxes(buf, &[b"mdia", b"mdhd"], trak_s, trak_e).first() {
            if e - s >= 20 {
                let version = buf[s];
                track.timescale = if version == 1 {
                    be_u32(buf, s + 20)
                } else {
                    be_u32(buf, s + 12)
                };
            }
        }

        if let Some(&(s, e)) = find_boxes(buf, &[b"mdia", b"hdlr"], trak_s, trak_e).first() {
            if e - s >= 24 {
                track.handler_type = String::from_utf8_lossy(&buf[s + 8..s + 12]).into_owned();
                let name = &buf[s + 24..e];
                let name = name.split(|&c| c == 0).next().unwrap_or(&[]);
                // QuickTime writes a Pascal string (length prefix); ISO writes a C string.
                let name = if !name.is_empty() && usize::from(name[0]) == name.len() - 1 {
                    &name[1..]
                } else {
                    name
                };
                track.handler_name = String::from_utf8_lossy(name).trim().to_string();
            }
        }

        if let Some(&(s, e)) = find_boxes(buf, &[b"mdia", b"minf", b"stbl"], trak_s, trak_e).first() {
            parse_stbl(buf, s, e, &mut track)?;
        }

        movie.tracks.push(track);
    }

    Ok(movie)
}

fn parse_stbl(buf: &[u8], start: usize, end: usize, track: &mut Track) -> Result<()> {
    let mut sizes: Vec<u32> = Vec::new();
    let mut chunk_offsets: Vec<u64> = Vec::new();
    let mut stsc: Vec<(u32, u32)> = Vec::new(); // (first_chunk, samples_per_chunk)
    let mut stts: Vec<(u32, u32)> = Vec::new(); // (count, delta)

    for (kind, s, e) in iter_boxes(buf, start, end) {
        match &kind {
            b"stsd" => {
                if e - s >= 16 {
                    track.fourcc = String::from_utf8_lossy(&buf[s + 12..s + 16]).into_owned();
                }
            }
            b"stsz" => {
                if e - s < 12 {
                    return Err(Mp4Error::Malformed("short stsz"));
                }
                let sample_size = be_u32(buf, s + 4);
                let count = be_u32(buf, s + 8) as usize;
                if sample_size != 0 {
                    sizes = vec![sample_size; count];
                } else {
                    if e - s < 12 + count * 4 {
                        return Err(Mp4Error::Malformed("short stsz table"));
                    }
                    sizes = (0..count).map(|i| be_u32(buf, s + 12 + i * 4)).collect();
                }
            }
            b"stco" => {
                let count = be_u32(buf, s + 4) as usize;
                if e - s < 8 + count * 4 {
                    return Err(Mp4Error::Malformed("short stco"));
                }
                chunk_offsets = (0..count)
                    .map(|i| u64::from(be_u32(buf, s + 8 + i * 4)))
                    .collect();
            }
            b"co64" => {
                let count = be_u32(buf, s + 4) as usize;
                if e - s < 8 + count * 8 {
                    return Err(Mp4Error::Malformed("short co64"));
                }
                chunk_offsets = (0..count).map(|i| be_u64(buf, s + 8 + i * 8)).collect();
            }
            b"stsc" => {
                let count = be_u32(buf, s + 4) as usize;
                if e - s < 8 + count * 12 {
                    return Err(Mp4Error::Malformed("short stsc"));
                }
                stsc = (0..count)
                    .map(|i| {
                        let p = s + 8 + i * 12;
                        (be_u32(buf, p), be_u32(buf, p + 4))
                    })
                    .collect();
            }
            b"stts" => {
                let count = be_u32(buf, s + 4) as usize;
                if e - s < 8 + count * 8 {
                    return Err(Mp4Error::Malformed("short stts"));
                }
                stts = (0..count)
                    .map(|i| {
                        let p = s + 8 + i * 8;
                        (be_u32(buf, p), be_u32(buf, p + 4))
                    })
                    .collect();
            }
            _ => {}
        }
    }

    // sample -> file offset through the chunk table
    let mut offsets = Vec::with_capacity(sizes.len());
    if !chunk_offsets.is_empty() && !stsc.is_empty() && !sizes.is_empty() {
        let mut sample_index = 0usize;
        'outer: for (entry_index, &(first_chunk, spc)) in stsc.iter().enumerate() {
            let last_chunk = stsc
                .get(entry_index + 1)
                .map(|next| next.0 - 1)
                .unwrap_or(chunk_offsets.len() as u32);
            for chunk in first_chunk..=last_chunk {
                let Some(&chunk_offset) = chunk_offsets.get((chunk as usize).wrapping_sub(1)) else {
                    break 'outer;
                };
                let mut pos = chunk_offset;
                for _ in 0..spc {
                    if sample_index >= sizes.len() {
                        break 'outer;
                    }
                    offsets.push(pos);
                    pos += u64::from(sizes[sample_index]);
                    sample_index += 1;
                }
            }
        }
    }
    let n = offsets.len();
    sizes.truncate(n);

    // sample -> decode time
    let ts = if track.timescale == 0 {
        1.0
    } else {
        f64::from(track.timescale)
    };
    let mut times = Vec::with_capacity(n);
    let mut t: u64 = 0;
    'stts: for &(count, delta) in &stts {
        for _ in 0..count {
            if times.len() >= n {
                break 'stts;
            }
            times.push(t as f64 / ts);
            t += u64::from(delta);
        }
    }
    if times.len() < n {
        let step = stts.last().map(|&(_, d)| f64::from(d) / ts).unwrap_or(0.0);
        let mut last = times.last().copied().unwrap_or(0.0);
        while times.len() < n {
            last += step;
            times.push(last);
        }
    }

    track.offsets = offsets;
    track.sizes = sizes;
    track.times = times;
    Ok(())
}

/// Builder for synthetic MP4 files used by tests and fixtures.
pub mod builder {
    fn boxed(kind: &[u8; 4], payload: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(payload.len() + 8);
        out.extend_from_slice(&((payload.len() + 8) as u32).to_be_bytes());
        out.extend_from_slice(kind);
        out.extend_from_slice(payload);
        out
    }

    fn full(kind: &[u8; 4], version: u8, body: &[u8]) -> Vec<u8> {
        let mut payload = vec![version, 0, 0, 0];
        payload.extend_from_slice(body);
        boxed(kind, &payload)
    }

    /// A track description for [`build`].
    pub struct TrackSpec<'a> {
        pub fourcc: &'a [u8; 4],
        pub handler_type: &'a [u8; 4],
        pub handler_name: &'a str,
        pub timescale: u32,
        pub sample_delta: u32,
        pub samples: &'a [Vec<u8>],
        /// Samples per chunk (the mdat interleaving granularity).
        pub chunk_size: usize,
    }

    /// Builds `ftyp | mdat | moov` with `co64` chunk offsets, mdat before moov like DJI cameras.
    pub fn build(tracks: &[TrackSpec<'_>], creation_time_1904: u64) -> Vec<u8> {
        let ftyp = boxed(b"ftyp", b"isom\0\0\x02\0isommp42");
        // Lay out mdat: track after track, chunk after chunk.
        let mut mdat_payload = Vec::new();
        let mut chunk_offsets_per_track: Vec<Vec<u64>> = Vec::new();
        let mdat_header_len = 16u64; // use 64-bit size header
        let mdat_start = ftyp.len() as u64;
        for track in tracks {
            let mut chunk_offsets = Vec::new();
            for chunk in track.samples.chunks(track.chunk_size.max(1)) {
                chunk_offsets.push(mdat_start + mdat_header_len + mdat_payload.len() as u64);
                for s in chunk {
                    mdat_payload.extend_from_slice(s);
                }
            }
            chunk_offsets_per_track.push(chunk_offsets);
        }
        let mut mdat = Vec::new();
        mdat.extend_from_slice(&1u32.to_be_bytes());
        mdat.extend_from_slice(b"mdat");
        mdat.extend_from_slice(&((mdat_payload.len() as u64) + mdat_header_len).to_be_bytes());
        mdat.extend_from_slice(&mdat_payload);

        let mut moov_payload = Vec::new();
        let mut mvhd = Vec::new();
        mvhd.extend_from_slice(&(creation_time_1904 as u32).to_be_bytes());
        mvhd.extend_from_slice(&(creation_time_1904 as u32).to_be_bytes());
        mvhd.extend_from_slice(&1000u32.to_be_bytes());
        let max_dur = tracks
            .iter()
            .map(|t| t.samples.len() as u64 * u64::from(t.sample_delta) * 1000 / u64::from(t.timescale))
            .max()
            .unwrap_or(0);
        mvhd.extend_from_slice(&(max_dur as u32).to_be_bytes());
        mvhd.extend_from_slice(&[0u8; 80]);
        moov_payload.extend_from_slice(&full(b"mvhd", 0, &mvhd));

        for (i, track) in tracks.iter().enumerate() {
            let mut tkhd = Vec::new();
            tkhd.extend_from_slice(&[0u8; 8]);
            tkhd.extend_from_slice(&((i + 1) as u32).to_be_bytes());
            tkhd.extend_from_slice(&[0u8; 68]);
            let tkhd = full(b"tkhd", 0, &tkhd);

            let mut mdhd = Vec::new();
            mdhd.extend_from_slice(&[0u8; 8]);
            mdhd.extend_from_slice(&track.timescale.to_be_bytes());
            mdhd.extend_from_slice(&((track.samples.len() as u32) * track.sample_delta).to_be_bytes());
            mdhd.extend_from_slice(&[0u8; 4]);
            let mdhd = full(b"mdhd", 0, &mdhd);

            let mut hdlr = Vec::new();
            hdlr.extend_from_slice(&[0u8; 4]);
            hdlr.extend_from_slice(track.handler_type);
            hdlr.extend_from_slice(&[0u8; 12]);
            hdlr.extend_from_slice(track.handler_name.as_bytes());
            hdlr.push(0);
            let hdlr = full(b"hdlr", 0, &hdlr);

            let mut stsd = Vec::new();
            stsd.extend_from_slice(&1u32.to_be_bytes());
            let mut entry = Vec::new();
            entry.extend_from_slice(&[0u8; 8]);
            let entry = boxed(track.fourcc, &entry);
            stsd.extend_from_slice(&entry);
            let stsd = full(b"stsd", 0, &stsd);

            let mut stts = Vec::new();
            stts.extend_from_slice(&1u32.to_be_bytes());
            stts.extend_from_slice(&(track.samples.len() as u32).to_be_bytes());
            stts.extend_from_slice(&track.sample_delta.to_be_bytes());
            let stts = full(b"stts", 0, &stts);

            let mut stsz = Vec::new();
            stsz.extend_from_slice(&0u32.to_be_bytes());
            stsz.extend_from_slice(&(track.samples.len() as u32).to_be_bytes());
            for s in track.samples {
                stsz.extend_from_slice(&(s.len() as u32).to_be_bytes());
            }
            let stsz = full(b"stsz", 0, &stsz);

            let chunks = &chunk_offsets_per_track[i];
            let full_chunks = track.samples.len() / track.chunk_size.max(1);
            let rem = track.samples.len() % track.chunk_size.max(1);
            let mut stsc_entries: Vec<(u32, u32)> = Vec::new();
            if full_chunks > 0 {
                stsc_entries.push((1, track.chunk_size as u32));
            }
            if rem > 0 {
                stsc_entries.push((full_chunks as u32 + 1, rem as u32));
            }
            let mut stsc = Vec::new();
            stsc.extend_from_slice(&(stsc_entries.len() as u32).to_be_bytes());
            for (first, spc) in stsc_entries {
                stsc.extend_from_slice(&first.to_be_bytes());
                stsc.extend_from_slice(&spc.to_be_bytes());
                stsc.extend_from_slice(&1u32.to_be_bytes());
            }
            let stsc = full(b"stsc", 0, &stsc);

            let mut co64 = Vec::new();
            co64.extend_from_slice(&(chunks.len() as u32).to_be_bytes());
            for &c in chunks {
                co64.extend_from_slice(&c.to_be_bytes());
            }
            let co64 = full(b"co64", 0, &co64);

            let mut stbl = Vec::new();
            for b in [stsd, stts, stsc, stsz, co64] {
                stbl.extend_from_slice(&b);
            }
            let stbl = boxed(b"stbl", &stbl);
            let minf = boxed(b"minf", &stbl);
            let mut mdia = Vec::new();
            mdia.extend_from_slice(&mdhd);
            mdia.extend_from_slice(&hdlr);
            mdia.extend_from_slice(&minf);
            let mdia = boxed(b"mdia", &mdia);
            let mut trak = Vec::new();
            trak.extend_from_slice(&tkhd);
            trak.extend_from_slice(&mdia);
            moov_payload.extend_from_slice(&boxed(b"trak", &trak));
        }
        let moov = boxed(b"moov", &moov_payload);

        let mut file = ftyp;
        file.extend_from_slice(&mdat);
        file.extend_from_slice(&moov);
        file
    }
}

#[cfg(test)]
mod tests {
    use super::builder::{build, TrackSpec};
    use super::*;
    use std::io::Cursor;

    #[test]
    fn parses_synthetic_file_with_interleaved_chunks() {
        let video: Vec<Vec<u8>> = (0..7).map(|i| vec![i as u8; 100 + i]).collect();
        let meta: Vec<Vec<u8>> = (0..7).map(|i| vec![0xA0 + i as u8; 10 + i]).collect();
        let file = build(
            &[
                TrackSpec {
                    fourcc: b"hvc1",
                    handler_type: b"vide",
                    handler_name: "VideoHandler",
                    timescale: 30000,
                    sample_delta: 1001,
                    samples: &video,
                    chunk_size: 3,
                },
                TrackSpec {
                    fourcc: b"djmd",
                    handler_type: b"meta",
                    handler_name: "DJI meta",
                    timescale: 30000,
                    sample_delta: 1001,
                    samples: &meta,
                    chunk_size: 2,
                },
            ],
            MP4_EPOCH_OFFSET as u64 + 1_789_000_000,
        );
        let mut cursor = Cursor::new(file);
        let movie = read_movie(&mut cursor).unwrap();
        assert_eq!(movie.tracks.len(), 2);
        assert_eq!(movie.creation_time.unwrap().and_utc().timestamp(), 1_789_000_000);
        let track = movie.find_track("djmd").unwrap();
        assert_eq!(track.handler_name, "DJI meta");
        assert_eq!(track.handler_type, "meta");
        assert_eq!(track.sample_count(), 7);
        assert!((track.times[1] - 1001.0 / 30000.0).abs() < 1e-9);
        for (i, expected) in meta.iter().enumerate() {
            assert_eq!(&track.read_sample(&mut cursor, i).unwrap(), expected);
        }
        let video_track = movie.find_track("hvc1").unwrap();
        for (i, expected) in video.iter().enumerate() {
            assert_eq!(&video_track.read_sample(&mut cursor, i).unwrap(), expected);
        }
    }

    #[test]
    fn missing_moov_is_reported() {
        let mut cursor = Cursor::new(b"\0\0\0\x08free".to_vec());
        assert!(matches!(read_movie(&mut cursor), Err(Mp4Error::NoMoov)));
    }
}
