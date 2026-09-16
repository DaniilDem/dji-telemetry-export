//! Minimal schema-less protobuf wire decoder.
//!
//! DJI's `djmd` samples are ordinary protobuf messages. We only need a handful
//! of fields at known paths, so instead of generating code from the `.proto`
//! files we walk the wire format directly: every field is `(number, wire type,
//! raw value)` and nested messages are just length-delimited byte slices.

use std::fmt;

/// One decoded field value, still in wire representation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value<'a> {
    /// Wire type 0.
    Varint(u64),
    /// Wire type 1 (little-endian).
    Fixed64([u8; 8]),
    /// Wire type 2: nested message, string, bytes or packed repeated field.
    Bytes(&'a [u8]),
    /// Wire type 5 (little-endian).
    Fixed32([u8; 4]),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodeError(pub &'static str);

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "protobuf decode error: {}", self.0)
    }
}

impl std::error::Error for DecodeError {}

/// Reads one base-128 varint starting at `pos`.
pub fn read_varint(data: &[u8], mut pos: usize) -> Result<(u64, usize), DecodeError> {
    let mut result: u64 = 0;
    let mut shift = 0u32;
    while pos < data.len() {
        let byte = data[pos];
        pos += 1;
        if shift < 64 {
            result |= u64::from(byte & 0x7f) << shift;
        }
        if byte & 0x80 == 0 {
            return Ok((result, pos));
        }
        shift += 7;
        if shift > 70 {
            return Err(DecodeError("varint longer than 10 bytes"));
        }
    }
    Err(DecodeError("truncated varint"))
}

/// Iterator over the top-level fields of one message.
pub struct Fields<'a> {
    data: &'a [u8],
    pos: usize,
    failed: bool,
}

impl<'a> Fields<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Fields {
            data,
            pos: 0,
            failed: false,
        }
    }

    fn next_field(&mut self) -> Result<Option<(u32, Value<'a>)>, DecodeError> {
        if self.pos >= self.data.len() {
            return Ok(None);
        }
        let (key, pos) = read_varint(self.data, self.pos)?;
        let field = (key >> 3) as u32;
        let wire = key & 0x7;
        if field == 0 {
            return Err(DecodeError("field number 0"));
        }
        let (value, next) = match wire {
            0 => {
                let (v, p) = read_varint(self.data, pos)?;
                (Value::Varint(v), p)
            }
            1 => {
                let end = pos + 8;
                if end > self.data.len() {
                    return Err(DecodeError("truncated fixed64"));
                }
                let mut b = [0u8; 8];
                b.copy_from_slice(&self.data[pos..end]);
                (Value::Fixed64(b), end)
            }
            2 => {
                let (len, p) = read_varint(self.data, pos)?;
                let len = usize::try_from(len).map_err(|_| DecodeError("length overflow"))?;
                let end = p.checked_add(len).ok_or(DecodeError("length overflow"))?;
                if end > self.data.len() {
                    return Err(DecodeError("truncated length-delimited field"));
                }
                (Value::Bytes(&self.data[p..end]), end)
            }
            5 => {
                let end = pos + 4;
                if end > self.data.len() {
                    return Err(DecodeError("truncated fixed32"));
                }
                let mut b = [0u8; 4];
                b.copy_from_slice(&self.data[pos..end]);
                (Value::Fixed32(b), end)
            }
            _ => return Err(DecodeError("unsupported wire type (group)")),
        };
        self.pos = next;
        Ok(Some((field, value)))
    }
}

impl<'a> Iterator for Fields<'a> {
    type Item = Result<(u32, Value<'a>), DecodeError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.failed {
            return None;
        }
        match self.next_field() {
            Ok(Some(item)) => Some(Ok(item)),
            Ok(None) => None,
            Err(e) => {
                self.failed = true;
                Some(Err(e))
            }
        }
    }
}

/// Returns true when `data` parses cleanly as a sequence of fields.
pub fn looks_like_message(data: &[u8]) -> bool {
    !data.is_empty() && Fields::new(data).all(|f| f.is_ok())
}

/// Returns the first occurrence of `field` in `data`, or `None` if absent or malformed.
pub fn get(data: &[u8], field: u32) -> Option<Value<'_>> {
    for item in Fields::new(data) {
        match item {
            Ok((f, v)) if f == field => return Some(v),
            Ok(_) => {}
            Err(_) => return None,
        }
    }
    None
}

/// Walks a chain of nested length-delimited fields.
pub fn sub<'a>(data: &'a [u8], path: &[u32]) -> Option<&'a [u8]> {
    let mut cur = data;
    for &field in path {
        match get(cur, field)? {
            Value::Bytes(b) => cur = b,
            _ => return None,
        }
    }
    Some(cur)
}

pub fn get_f32(data: &[u8], field: u32) -> Option<f32> {
    match get(data, field)? {
        Value::Fixed32(b) => Some(f32::from_le_bytes(b)),
        _ => None,
    }
}

pub fn get_f64(data: &[u8], field: u32) -> Option<f64> {
    match get(data, field)? {
        Value::Fixed64(b) => Some(f64::from_le_bytes(b)),
        _ => None,
    }
}

pub fn get_u64(data: &[u8], field: u32) -> Option<u64> {
    match get(data, field)? {
        Value::Varint(v) => Some(v),
        _ => None,
    }
}

/// Reads a varint as a two's-complement `int32`/`int64` (protobuf sign-extends negatives to 64 bits).
pub fn get_i64(data: &[u8], field: u32) -> Option<i64> {
    get_u64(data, field).map(|v| v as i64)
}

pub fn get_string(data: &[u8], field: u32) -> Option<String> {
    match get(data, field)? {
        Value::Bytes(b) => std::str::from_utf8(b)
            .ok()
            .map(|s| s.trim_end_matches('\0').to_string()),
        _ => None,
    }
}

/// DJI encodes rationals (shutter speed) as a length-delimited pair of varints `num, den`.
pub fn get_rational(data: &[u8], field: u32) -> Option<(u64, u64)> {
    match get(data, field)? {
        Value::Bytes(b) => {
            let (num, p) = read_varint(b, 0).ok()?;
            let (den, p) = read_varint(b, p).ok()?;
            if p != b.len() {
                return None;
            }
            Some((num, den))
        }
        _ => None,
    }
}

/// Renders a message as an indented field tree (used by `--inspect`).
pub fn dump_tree(data: &[u8], indent: usize, max_depth: usize, out: &mut String) {
    use std::fmt::Write as _;
    let pad = "  ".repeat(indent);
    let fields: Result<Vec<_>, _> = Fields::new(data).collect();
    let fields = match fields {
        Ok(f) => f,
        Err(_) => {
            let _ = writeln!(out, "{pad}<{} raw bytes>", data.len());
            return;
        }
    };
    for (field, value) in fields {
        match value {
            Value::Varint(v) => {
                let _ = writeln!(out, "{pad}f{field} varint = {v}");
            }
            Value::Fixed32(b) => {
                let _ = writeln!(out, "{pad}f{field} f32 = {}", f32::from_le_bytes(b));
            }
            Value::Fixed64(b) => {
                let _ = writeln!(out, "{pad}f{field} f64 = {}", f64::from_le_bytes(b));
            }
            Value::Bytes(b) => {
                if indent < max_depth && b.len() > 1 && looks_like_message(b) {
                    let _ = writeln!(out, "{pad}f{field} msg ({} B)", b.len());
                    dump_tree(b, indent + 1, max_depth, out);
                } else if !b.is_empty() && b.iter().all(|c| (0x20..0x7f).contains(c)) {
                    let _ = writeln!(
                        out,
                        "{pad}f{field} str ({} B) = {:?}",
                        b.len(),
                        String::from_utf8_lossy(b)
                    );
                } else {
                    let hex: String = b.iter().take(24).map(|c| format!("{c:02x}")).collect();
                    let _ = writeln!(out, "{pad}f{field} bytes ({} B) = {hex}", b.len());
                }
            }
        }
    }
}

/// Tiny encoder used by tests and fixtures to build synthetic messages.
pub mod encode {
    pub fn varint(mut v: u64, out: &mut Vec<u8>) {
        loop {
            let byte = (v & 0x7f) as u8;
            v >>= 7;
            if v == 0 {
                out.push(byte);
                return;
            }
            out.push(byte | 0x80);
        }
    }

    fn key(field: u32, wire: u8, out: &mut Vec<u8>) {
        varint((u64::from(field) << 3) | u64::from(wire), out);
    }

    pub fn field_varint(field: u32, v: u64, out: &mut Vec<u8>) {
        key(field, 0, out);
        varint(v, out);
    }

    pub fn field_f32(field: u32, v: f32, out: &mut Vec<u8>) {
        key(field, 5, out);
        out.extend_from_slice(&v.to_le_bytes());
    }

    pub fn field_f64(field: u32, v: f64, out: &mut Vec<u8>) {
        key(field, 1, out);
        out.extend_from_slice(&v.to_le_bytes());
    }

    pub fn field_bytes(field: u32, v: &[u8], out: &mut Vec<u8>) {
        key(field, 2, out);
        varint(v.len() as u64, out);
        out.extend_from_slice(v);
    }

    pub fn field_str(field: u32, v: &str, out: &mut Vec<u8>) {
        field_bytes(field, v.as_bytes(), out);
    }

    pub fn field_rational(field: u32, num: u64, den: u64, out: &mut Vec<u8>) {
        let mut inner = Vec::new();
        varint(num, &mut inner);
        varint(den, &mut inner);
        field_bytes(field, &inner, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn varint_roundtrip() {
        for v in [0u64, 1, 127, 128, 300, 2000746133, u64::MAX] {
            let mut buf = Vec::new();
            encode::varint(v, &mut buf);
            assert_eq!(read_varint(&buf, 0).unwrap(), (v, buf.len()));
        }
    }

    #[test]
    fn nested_lookup() {
        let mut inner = Vec::new();
        encode::field_f32(2, -1.5, &mut inner);
        encode::field_f32(3, 0.25, &mut inner);
        let mut mid = Vec::new();
        encode::field_bytes(10, &inner, &mut mid);
        let mut outer = Vec::new();
        encode::field_bytes(2, &mid, &mut outer);
        encode::field_varint(7, 42, &mut outer);
        encode::field_str(9, "DJI", &mut outer);

        let acc = sub(&outer, &[2, 10]).unwrap();
        assert_eq!(get_f32(acc, 2), Some(-1.5));
        assert_eq!(get_f32(acc, 3), Some(0.25));
        assert_eq!(get_f32(acc, 4), None);
        assert_eq!(get_u64(&outer, 7), Some(42));
        assert_eq!(get_string(&outer, 9).as_deref(), Some("DJI"));
        assert!(sub(&outer, &[7]).is_none());
    }

    #[test]
    fn rational_matches_dji_shutter_encoding() {
        // Observed on Osmo Action 5 Pro: bytes 01 a7 18 => 1/3111 s
        let mut msg = Vec::new();
        encode::field_bytes(1, &[0x01, 0xa7, 0x18], &mut msg);
        assert_eq!(get_rational(&msg, 1), Some((1, 3111)));
    }

    #[test]
    fn negative_int32_is_sign_extended() {
        let mut msg = Vec::new();
        encode::field_varint(2, (-1500i64) as u64, &mut msg);
        assert_eq!(get_i64(&msg, 2), Some(-1500));
    }

    #[test]
    fn truncated_message_is_an_error() {
        let mut msg = Vec::new();
        encode::field_bytes(1, &[1, 2, 3], &mut msg);
        msg.pop();
        assert!(!looks_like_message(&msg));
        assert!(get(&msg, 1).is_none());
    }
}
