//! Turns raw camera samples into vehicle-frame telemetry rows.
//!
//! * accelerometer unit detection (g / m·s⁻² / mg) from the resting magnitude
//! * gravity removal — attitude-quaternion method (keeps sustained cornering
//!   acceleration) with a zero-phase moving-average high-pass as fallback
//! * body-axis → lateral / longitudinal / vertical mapping with sign flips
//! * camera roll / pitch / yaw from the quaternion
//! * GPS-derived speed, heading and cumulative distance
//! * optional decimation to a lower export rate

use std::fmt;

use chrono::{Duration, NaiveDateTime};

use crate::dji::{Clip, ClipInfo, Sample};

pub const G_SI: f64 = 9.80665;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccelUnit {
    #[default]
    G,
    MetersPerSecondSquared,
    MilliG,
    Unknown,
}

impl AccelUnit {
    pub fn scale_to_g(self) -> f64 {
        match self {
            AccelUnit::G | AccelUnit::Unknown => 1.0,
            AccelUnit::MetersPerSecondSquared => 1.0 / G_SI,
            AccelUnit::MilliG => 1.0 / 1000.0,
        }
    }
}

impl fmt::Display for AccelUnit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            AccelUnit::G => "g",
            AccelUnit::MetersPerSecondSquared => "m/s²",
            AccelUnit::MilliG => "mg",
            AccelUnit::Unknown => "unknown (assuming g)",
        })
    }
}

fn median(values: &mut [f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(|a, b| a.total_cmp(b));
    Some(values[values.len() / 2])
}

/// Infers the accelerometer unit from the median magnitude (≈1 g at rest, ≈1.5 g in a
/// vibrating kart — still unambiguous against 9.8 m/s² or 1000 mg).
pub fn detect_accel_unit(samples: &[Sample]) -> (AccelUnit, f64) {
    let mut mags: Vec<f64> = samples
        .iter()
        .filter_map(|s| s.acc)
        .map(|a| (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
        .collect();
    let Some(m) = median(&mut mags) else {
        return (AccelUnit::Unknown, 0.0);
    };
    let unit = if (0.5..=3.0).contains(&m) {
        AccelUnit::G
    } else if (5.0..=30.0).contains(&m) {
        AccelUnit::MetersPerSecondSquared
    } else if (500.0..=3000.0).contains(&m) {
        AccelUnit::MilliG
    } else {
        AccelUnit::Unknown
    };
    (unit, m)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GravityMode {
    /// Rotate into the world frame with the attitude quaternion, subtract the median
    /// (which is gravity), rotate back. Keeps sustained acceleration.
    #[default]
    Quaternion,
    /// Subtract a centred moving average (window `highpass_window_s`).
    HighPass,
    /// Leave gravity in.
    None,
}

impl GravityMode {
    pub const ALL: [GravityMode; 3] = [GravityMode::Quaternion, GravityMode::HighPass, GravityMode::None];

    pub fn label(self) -> &'static str {
        match self {
            GravityMode::Quaternion => "Attitude quaternion (recommended)",
            GravityMode::HighPass => "High-pass (moving average)",
            GravityMode::None => "Keep gravity",
        }
    }

    pub fn cli_name(self) -> &'static str {
        match self {
            GravityMode::Quaternion => "quat",
            GravityMode::HighPass => "highpass",
            GravityMode::None => "none",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "quat" | "quaternion" => Some(GravityMode::Quaternion),
            "highpass" | "high-pass" | "hp" => Some(GravityMode::HighPass),
            "none" | "keep" => Some(GravityMode::None),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Axis {
    #[default]
    X,
    Y,
    Z,
}

impl Axis {
    pub const ALL: [Axis; 3] = [Axis::X, Axis::Y, Axis::Z];

    pub fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Axis::X => "X",
            Axis::Y => "Y",
            Axis::Z => "Z",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "x" => Some(Axis::X),
            "y" => Some(Axis::Y),
            "z" => Some(Axis::Z),
            _ => None,
        }
    }
}

/// Which body axis feeds each vehicle axis, plus sign flips.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AxisMap {
    pub lateral: Axis,
    pub longitudinal: Axis,
    pub vertical: Axis,
    pub invert_lateral: bool,
    pub invert_longitudinal: bool,
    pub invert_vertical: bool,
}

impl Default for AxisMap {
    fn default() -> Self {
        // ExifTool: X = left/right, Y = front/back, Z = up/down.
        AxisMap {
            lateral: Axis::X,
            longitudinal: Axis::Y,
            vertical: Axis::Z,
            invert_lateral: false,
            invert_longitudinal: false,
            invert_vertical: false,
        }
    }
}

impl AxisMap {
    pub fn is_permutation(&self) -> bool {
        let mut seen = [false; 3];
        for a in [self.lateral, self.longitudinal, self.vertical] {
            seen[a.index()] = true;
        }
        seen.iter().all(|&s| s)
    }

    /// Parses `lateral=x,longitudinal=y,vertical=z` (any subset; missing keep defaults)
    /// or the compact `xyz` form (lateral, longitudinal, vertical).
    pub fn parse(s: &str) -> Result<Self, String> {
        let mut map = AxisMap::default();
        let s = s.trim();
        if s.len() == 3 && !s.contains('=') {
            let mut chars = s.chars();
            let lat = Axis::parse(&chars.next().unwrap().to_string());
            let lon = Axis::parse(&chars.next().unwrap().to_string());
            let ver = Axis::parse(&chars.next().unwrap().to_string());
            match (lat, lon, ver) {
                (Some(a), Some(b), Some(c)) => {
                    map.lateral = a;
                    map.longitudinal = b;
                    map.vertical = c;
                }
                _ => return Err(format!("invalid axis spec '{s}'")),
            }
        } else {
            for part in s.split(',').filter(|p| !p.trim().is_empty()) {
                let (key, value) = part
                    .split_once('=')
                    .ok_or_else(|| format!("invalid axis spec '{part}', expected key=axis"))?;
                let axis = Axis::parse(value).ok_or_else(|| format!("unknown axis '{value}'"))?;
                match key.trim().to_ascii_lowercase().as_str() {
                    "lateral" | "lat" => map.lateral = axis,
                    "longitudinal" | "lon" | "long" => map.longitudinal = axis,
                    "vertical" | "vert" | "ver" => map.vertical = axis,
                    other => return Err(format!("unknown vehicle axis '{other}'")),
                }
            }
        }
        if !map.is_permutation() {
            return Err("axis mapping must use each of X, Y, Z exactly once".to_string());
        }
        Ok(map)
    }
}

/// Median of each raw body axis (in g) — shows where gravity sits at rest.
pub fn axis_medians(samples: &[Sample], scale: f64) -> [f64; 3] {
    let mut out = [0.0; 3];
    for (i, slot) in out.iter_mut().enumerate() {
        let mut v: Vec<f64> = samples
            .iter()
            .filter_map(|s| s.acc)
            .map(|a| a[i] * scale)
            .collect();
        *slot = median(&mut v).unwrap_or(0.0);
    }
    out
}

/// Guesses the mapping: the axis holding gravity is vertical. Of the remaining two, X — the
/// camera's optical axis, which points along the vehicle on a normal forward-facing mount — is
/// longitudinal and the other is lateral. When X itself holds gravity (lens pointing up or
/// down) there is nothing to go on, so Y is taken as lateral and Z as longitudinal.
///
/// Signs: the Osmo Action body frame is X forward, Y right, Z down and the reported vector is
/// the proper acceleration (checked against video on a forward-facing kart clip: a left-hand
/// corner reads negative on Y, braking negative on X). Longitudinal is taken as is (forward
/// positive, so braking is negative). Lateral is negated so that a left-hand corner reads
/// positive: on OVRLEY's G-force gauge that moves the dot the way the driver is thrown
/// (right in a left-hander, up under braking), which is what people expect to see. The
/// vertical axis is flipped so that +1 g at rest reads as "up".
pub fn auto_axis_map(medians: [f64; 3]) -> AxisMap {
    let mut order: Vec<usize> = (0..3).collect();
    order.sort_by(|&a, &b| medians[b].abs().total_cmp(&medians[a].abs()));
    let vertical_idx = order[0];
    let vertical = Axis::ALL[vertical_idx];
    let rest: Vec<Axis> = Axis::ALL
        .iter()
        .copied()
        .filter(|a| a.index() != vertical_idx)
        .collect();
    let (lateral, longitudinal) = if vertical == Axis::X {
        (rest[0], rest[1])
    } else {
        (
            rest.into_iter().find(|a| *a != Axis::X).unwrap_or(Axis::Y),
            Axis::X,
        )
    };
    AxisMap {
        lateral,
        longitudinal,
        vertical,
        invert_lateral: true,
        invert_longitudinal: false,
        invert_vertical: medians[vertical_idx] < 0.0,
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExportRate {
    Native,
    Hz(f64),
}

impl ExportRate {
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim().to_ascii_lowercase();
        if s == "native" || s == "full" || s == "0" {
            return Some(ExportRate::Native);
        }
        s.trim_end_matches("hz")
            .trim()
            .parse::<f64>()
            .ok()
            .filter(|v| *v > 0.0)
            .map(ExportRate::Hz)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ProcessOptions {
    pub gravity: GravityMode,
    pub highpass_window_s: f64,
    pub axes: AxisMap,
    /// Rotate the body frame so that gravity lies exactly on the vertical axis before the
    /// lateral / longitudinal / vertical split. Compensates a camera that is pitched or rolled
    /// relative to the vehicle; per sample when the attitude quaternion is available, otherwise
    /// with the clip's median gravity direction.
    pub level: bool,
    pub rate: ExportRate,
    /// Override the detected accelerometer unit.
    pub unit_override: Option<AccelUnit>,
}

impl Default for ProcessOptions {
    fn default() -> Self {
        ProcessOptions {
            gravity: GravityMode::Quaternion,
            highpass_window_s: 1.0,
            axes: AxisMap::default(),
            level: true,
            rate: ExportRate::Native,
            unit_override: None,
        }
    }
}

/// One exported telemetry row (all accelerations in g, angles in degrees, SI otherwise).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Row {
    pub t: f64,
    pub frame: u64,
    pub lateral: Option<f64>,
    pub longitudinal: Option<f64>,
    pub vertical: Option<f64>,
    /// `hypot(lateral, longitudinal)` — the motorsport "G" scalar.
    pub combined: Option<f64>,
    /// Raw body axes (in g, gravity included).
    pub raw: Option<[f64; 3]>,
    pub roll: Option<f64>,
    pub pitch: Option<f64>,
    pub yaw: Option<f64>,
    pub iso: Option<f32>,
    pub shutter: Option<(u64, u64)>,
    pub color_temperature: Option<u32>,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
    pub altitude_m: Option<f64>,
    pub speed_ms: Option<f64>,
    pub heading_deg: Option<f64>,
    pub distance_m: Option<f64>,
}

impl Row {
    pub fn shutter_seconds(&self) -> Option<f64> {
        self.shutter.map(|(n, d)| n as f64 / d as f64)
    }
    pub fn has_gps(&self) -> bool {
        self.latitude.is_some() && self.longitude.is_some()
    }
}

#[derive(Debug, Clone)]
pub struct Telemetry {
    pub info: ClipInfo,
    pub options: ProcessOptions,
    pub unit: AccelUnit,
    pub unit_median_magnitude: f64,
    /// Method actually used (quaternion falls back to high-pass when attitude is missing).
    pub gravity_used: GravityMode,
    /// World-frame gravity vector estimated by the quaternion method.
    pub gravity_world: Option<[f64; 3]>,
    pub axis_medians: [f64; 3],
    /// Median angle between the measured gravity direction and the chosen vertical axis —
    /// how far the camera is pitched / rolled off the vehicle's vertical. `None` when it
    /// could not be estimated.
    pub tilt_deg: Option<f64>,
    /// Whether the exported axes were levelled (see [`ProcessOptions::level`]).
    pub levelled: bool,
    pub rows: Vec<Row>,
    pub gps_rows: usize,
}

impl Telemetry {
    pub fn has_gps(&self) -> bool {
        self.gps_rows >= 2
    }

    /// Camera-local wall clock for a row.
    pub fn local_time(&self, t: f64) -> Option<NaiveDateTime> {
        self.info
            .start_local
            .map(|s| s + Duration::microseconds((t * 1e6).round() as i64))
    }

    /// UTC wall clock for a row, when the camera's UTC offset is known.
    pub fn utc_time(&self, t: f64) -> Option<NaiveDateTime> {
        self.info
            .start_utc()
            .map(|s| s + Duration::microseconds((t * 1e6).round() as i64))
    }

    pub fn duration_s(&self) -> f64 {
        self.rows.last().map(|r| r.t).unwrap_or(0.0)
    }
}

fn rotate(q: [f64; 4], v: [f64; 3], inverse: bool) -> [f64; 3] {
    let [w, mut x, mut y, mut z] = q;
    if inverse {
        x = -x;
        y = -y;
        z = -z;
    }
    let [vx, vy, vz] = v;
    let tx = 2.0 * (y * vz - z * vy);
    let ty = 2.0 * (z * vx - x * vz);
    let tz = 2.0 * (x * vy - y * vx);
    [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]
}

fn normalized(q: [f64; 4]) -> Option<[f64; 4]> {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-6 || !n.is_finite() {
        return None;
    }
    Some([q[0] / n, q[1] / n, q[2] / n, q[3] / n])
}

fn unit3(v: [f64; 3]) -> Option<[f64; 3]> {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if n < 1e-6 || !n.is_finite() {
        return None;
    }
    Some([v[0] / n, v[1] / n, v[2] / n])
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Applies to `v` the smallest rotation that takes unit vector `from` onto unit vector `to`
/// (Rodrigues' formula).
fn rotate_onto(from: [f64; 3], to: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let c = dot3(from, to).clamp(-1.0, 1.0);
    let axis = cross3(from, to);
    let s = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if s < 1e-9 {
        if c > 0.0 {
            return v;
        }
        // Exactly opposite: rotate 180° about any axis perpendicular to `from`.
        let helper = if from[0].abs() < 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let k = unit3(cross3(from, helper)).unwrap_or([0.0, 0.0, 1.0]);
        let kv = dot3(k, v);
        return [
            2.0 * k[0] * kv - v[0],
            2.0 * k[1] * kv - v[1],
            2.0 * k[2] * kv - v[2],
        ];
    }
    let k = [axis[0] / s, axis[1] / s, axis[2] / s];
    let kxv = cross3(k, v);
    let kv = dot3(k, v) * (1.0 - c);
    [
        v[0] * c + kxv[0] * s + k[0] * kv,
        v[1] * c + kxv[1] * s + k[1] * kv,
        v[2] * c + kxv[2] * s + k[2] * kv,
    ]
}

/// Unit vector along the vehicle's vertical axis in the body frame, pointing the way an
/// accelerometer reads gravity at rest (+1 g "up").
fn body_up(axes: &AxisMap) -> [f64; 3] {
    let mut up = [0.0; 3];
    up[axes.vertical.index()] = if axes.invert_vertical { -1.0 } else { 1.0 };
    up
}

/// Body-frame dynamic acceleration per sample plus the estimated world-frame gravity vector.
type GravityRemoval = (Vec<Option<[f64; 3]>>, [f64; 3]);

/// Quaternion-based gravity removal; returns body-frame dynamic acceleration per sample.
fn remove_gravity_quat(acc: &[Option<[f64; 3]>], quats: &[Option<[f64; 4]>]) -> Option<GravityRemoval> {
    let mut world = Vec::with_capacity(acc.len());
    let mut idx = Vec::with_capacity(acc.len());
    for (i, (a, q)) in acc.iter().zip(quats).enumerate() {
        if let (Some(a), Some(q)) = (a, q.and_then(normalized)) {
            world.push(rotate(q, *a, false));
            idx.push((i, q));
        }
    }
    if world.len() < 10 {
        return None;
    }
    let mut gravity = [0.0; 3];
    for (k, g) in gravity.iter_mut().enumerate() {
        let mut comp: Vec<f64> = world.iter().map(|w| w[k]).collect();
        *g = median(&mut comp).unwrap_or(0.0);
    }
    let mut out = vec![None; acc.len()];
    for ((i, q), w) in idx.into_iter().zip(world) {
        let residual = [w[0] - gravity[0], w[1] - gravity[1], w[2] - gravity[2]];
        out[i] = Some(rotate(q, residual, true));
    }
    Some((out, gravity))
}

/// Centred moving-average high-pass on each axis.
fn remove_gravity_highpass(acc: &[Option<[f64; 3]>], times: &[f64], window_s: f64) -> Vec<Option<[f64; 3]>> {
    let n = acc.len();
    if n == 0 || window_s <= 0.0 {
        return acc.to_vec();
    }
    let duration = times[n - 1] - times[0];
    let rate = if duration > 0.0 {
        (n as f64 - 1.0) / duration
    } else {
        0.0
    };
    let half = if rate > 0.0 {
        ((rate * window_s / 2.0).round() as usize).max(1)
    } else {
        1
    };

    let mut out = vec![None; n];
    for axis in 0..3 {
        let present: Vec<f64> = acc.iter().filter_map(|a| a.map(|a| a[axis])).collect();
        if present.is_empty() {
            continue;
        }
        let fallback = present.iter().sum::<f64>() / present.len() as f64;
        let mut prefix = Vec::with_capacity(n + 1);
        prefix.push(0.0);
        for a in acc {
            let v = a.map(|a| a[axis]).unwrap_or(fallback);
            prefix.push(prefix.last().unwrap() + v);
        }
        for i in 0..n {
            let Some(a) = acc[i] else { continue };
            let lo = i.saturating_sub(half);
            let hi = (i + half + 1).min(n);
            let mean = (prefix[hi] - prefix[lo]) / (hi - lo) as f64;
            let slot = out[i].get_or_insert([0.0; 3]);
            slot[axis] = a[axis] - mean;
        }
    }
    out
}

/// Aerospace (ZYX) Euler angles from a `[w, x, y, z]` quaternion, in degrees.
pub fn quat_to_euler_deg(q: [f64; 4]) -> Option<[f64; 3]> {
    let [w, x, y, z] = normalized(q)?;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    Some([roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()])
}

/// Great-circle distance in metres.
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let r = 6_371_008.8;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * r * a.sqrt().asin()
}

/// Initial bearing in degrees clockwise from north.
pub fn bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dl = (lon2 - lon1).to_radians();
    let y = dl.sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// Runs the full processing chain.
pub fn process(clip: &Clip, options: &ProcessOptions) -> Telemetry {
    let samples = &clip.samples;
    let (detected_unit, magnitude) = detect_accel_unit(samples);
    let unit = options.unit_override.unwrap_or(detected_unit);
    let scale = unit.scale_to_g();

    let times: Vec<f64> = samples.iter().map(|s| s.t).collect();
    let acc_g: Vec<Option<[f64; 3]>> = samples
        .iter()
        .map(|s| s.acc.map(|a| [a[0] * scale, a[1] * scale, a[2] * scale]))
        .collect();
    let quats: Vec<Option<[f64; 4]>> = samples.iter().map(|s| s.quat).collect();
    let medians = axis_medians(samples, scale);

    // The quaternion estimate is computed whenever attitude is available: even when another
    // gravity mode is selected it still provides the per-sample gravity direction for levelling.
    let quat_estimate = remove_gravity_quat(&acc_g, &quats);
    let gravity_world = quat_estimate.as_ref().map(|(_, g)| *g);
    let mut gravity_used = options.gravity;
    let dynamic: Vec<Option<[f64; 3]>> = match options.gravity {
        GravityMode::None => acc_g.clone(),
        GravityMode::Quaternion => match quat_estimate {
            Some((d, _)) => d,
            None => {
                gravity_used = GravityMode::HighPass;
                remove_gravity_highpass(&acc_g, &times, options.highpass_window_s)
            }
        },
        GravityMode::HighPass => remove_gravity_highpass(&acc_g, &times, options.highpass_window_s),
    };

    let axes = options.axes;
    let up = body_up(&axes);
    // Direction gravity reads in the body frame: per sample from the attitude quaternion when
    // possible, otherwise the clip-wide median. `None` when there is nothing to level with.
    let world_up = gravity_world.and_then(unit3);
    let static_up = unit3(medians).filter(|_| medians.iter().map(|m| m * m).sum::<f64>().sqrt() > 0.5);
    let gravity_dir = |i: usize| -> Option<[f64; 3]> {
        match (world_up, quats[i].and_then(normalized)) {
            (Some(g), Some(q)) => unit3(rotate(q, g, true)),
            _ => static_up,
        }
    };
    let mut tilts: Vec<f64> = Vec::with_capacity(samples.len());
    let dynamic: Vec<Option<[f64; 3]>> = dynamic
        .iter()
        .enumerate()
        .map(|(i, d)| {
            let d = (*d)?;
            let Some(g) = gravity_dir(i) else { return Some(d) };
            tilts.push(dot3(g, up).clamp(-1.0, 1.0).acos().to_degrees());
            Some(if options.level { rotate_onto(g, up, d) } else { d })
        })
        .collect();
    let tilt_deg = median(&mut tilts);
    let levelled = options.level && tilt_deg.is_some();

    let pick = |v: &[f64; 3], axis: Axis, invert: bool| {
        let x = v[axis.index()];
        if invert {
            -x
        } else {
            x
        }
    };

    // GPS-derived series on the full-rate timeline.
    let mut distance = 0.0;
    let mut last_fix: Option<(f64, f64, f64)> = None;
    let mut rows: Vec<Row> = Vec::with_capacity(samples.len());
    for (i, s) in samples.iter().enumerate() {
        let mut row = Row {
            t: s.t,
            frame: s.frame,
            raw: acc_g[i],
            iso: s.iso,
            shutter: s.shutter,
            color_temperature: s.color_temperature,
            ..Row::default()
        };
        if let Some(d) = dynamic[i] {
            let lateral = pick(&d, axes.lateral, axes.invert_lateral);
            let longitudinal = pick(&d, axes.longitudinal, axes.invert_longitudinal);
            let vertical = pick(&d, axes.vertical, axes.invert_vertical);
            row.lateral = Some(lateral);
            row.longitudinal = Some(longitudinal);
            row.vertical = Some(vertical);
            row.combined = Some(lateral.hypot(longitudinal));
        }
        if let Some(e) = s.quat.and_then(quat_to_euler_deg) {
            row.roll = Some(e[0]);
            row.pitch = Some(e[1]);
            row.yaw = Some(e[2]);
        }
        if let Some(g) = &s.gps {
            row.latitude = Some(g.latitude);
            row.longitude = Some(g.longitude);
            row.altitude_m = g.altitude_m;
            if let Some((t0, lat0, lon0)) = last_fix {
                let d = haversine_m(lat0, lon0, g.latitude, g.longitude);
                distance += d;
                let dt = s.t - t0;
                if g.velocity.is_none() && dt > 0.0 {
                    row.speed_ms = Some(d / dt);
                    if d > 0.05 {
                        row.heading_deg = Some(bearing_deg(lat0, lon0, g.latitude, g.longitude));
                    }
                }
            }
            if let Some(v) = g.velocity {
                // Same convention as OVRLEY's DJI parser: speed = hypot(vx, vy), heading = atan2(vx, vy).
                row.speed_ms = Some(v[0].hypot(v[1]));
                if v[0] != 0.0 || v[1] != 0.0 {
                    row.heading_deg = Some((v[0].atan2(v[1]).to_degrees() + 360.0) % 360.0);
                }
            }
            row.distance_m = Some(distance);
            last_fix = Some((s.t, g.latitude, g.longitude));
        }
        rows.push(row);
    }

    // Guarantee a non-decreasing timeline (OVRLEY rejects backwards time).
    let mut last_t = f64::NEG_INFINITY;
    rows.retain(|r| {
        if r.t >= last_t {
            last_t = r.t;
            true
        } else {
            false
        }
    });

    let rows = match options.rate {
        ExportRate::Native => rows,
        ExportRate::Hz(hz) => decimate(&rows, hz),
    };
    let gps_rows = rows.iter().filter(|r| r.has_gps()).count();

    Telemetry {
        info: clip.info.clone(),
        options: options.clone(),
        unit,
        unit_median_magnitude: magnitude,
        gravity_used,
        gravity_world,
        axis_medians: medians,
        tilt_deg,
        levelled,
        rows,
        gps_rows,
    }
}

fn mean_opt(values: impl Iterator<Item = Option<f64>>) -> Option<f64> {
    let (sum, n) = values.flatten().fold((0.0, 0usize), |(s, n), v| (s + v, n + 1));
    (n > 0).then(|| sum / n as f64)
}

/// Bins rows into `1/hz` slots; accelerations are averaged (anti-aliasing), everything
/// else is taken from the first row of the bin (whose timestamp the output keeps), with
/// position fields coming from the first GPS-bearing row of the bin.
pub fn decimate(rows: &[Row], hz: f64) -> Vec<Row> {
    if rows.is_empty() || hz <= 0.0 {
        return rows.to_vec();
    }
    let mut out = Vec::new();
    let mut bin_start = 0usize;
    let bin_of = |t: f64| (t * hz + 1e-9).floor() as i64;
    let mut current_bin = bin_of(rows[0].t);
    for i in 1..=rows.len() {
        let flush = i == rows.len() || bin_of(rows[i].t) != current_bin;
        if flush {
            let bin = &rows[bin_start..i];
            let mut row = bin[0].clone();
            row.lateral = mean_opt(bin.iter().map(|r| r.lateral));
            row.longitudinal = mean_opt(bin.iter().map(|r| r.longitudinal));
            row.vertical = mean_opt(bin.iter().map(|r| r.vertical));
            row.combined = match (row.lateral, row.longitudinal) {
                (Some(a), Some(b)) => Some(a.hypot(b)),
                _ => None,
            };
            row.raw = {
                let x = mean_opt(bin.iter().map(|r| r.raw.map(|a| a[0])));
                let y = mean_opt(bin.iter().map(|r| r.raw.map(|a| a[1])));
                let z = mean_opt(bin.iter().map(|r| r.raw.map(|a| a[2])));
                match (x, y, z) {
                    (Some(x), Some(y), Some(z)) => Some([x, y, z]),
                    _ => None,
                }
            };
            // Position from the first GPS-bearing row of the bin (closest to the kept timestamp).
            if let Some(g) = bin.iter().find(|r| r.has_gps()) {
                row.latitude = g.latitude;
                row.longitude = g.longitude;
                row.altitude_m = g.altitude_m;
                row.speed_ms = g.speed_ms;
                row.heading_deg = g.heading_deg;
                row.distance_m = g.distance_m;
            }
            out.push(row);
            bin_start = i;
            if i < rows.len() {
                current_bin = bin_of(rows[i].t);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dji::GpsFix;

    fn sample(t: f64, acc: [f64; 3], quat: Option<[f64; 4]>) -> Sample {
        Sample {
            t,
            frame: (t * 30.0) as u64,
            acc: Some(acc),
            quat,
            ..Sample::default()
        }
    }

    #[test]
    fn unit_detection() {
        let s: Vec<Sample> = (0..20)
            .map(|i| sample(i as f64 / 30.0, [0.02, 0.01, 1.02], None))
            .collect();
        assert_eq!(detect_accel_unit(&s).0, AccelUnit::G);
        let s: Vec<Sample> = (0..20)
            .map(|i| sample(i as f64 / 30.0, [0.2, 0.1, 9.75], None))
            .collect();
        assert_eq!(detect_accel_unit(&s).0, AccelUnit::MetersPerSecondSquared);
        let s: Vec<Sample> = (0..20)
            .map(|i| sample(i as f64 / 30.0, [20.0, 10.0, 995.0], None))
            .collect();
        assert_eq!(detect_accel_unit(&s).0, AccelUnit::MilliG);
    }

    #[test]
    fn auto_axis_map_picks_gravity_axis() {
        let m = auto_axis_map([-1.01, 0.05, -0.2]);
        assert_eq!(m.vertical, Axis::X);
        assert_eq!(m.lateral, Axis::Y);
        assert_eq!(m.longitudinal, Axis::Z);
        assert!(m.invert_vertical);
        assert!(m.invert_lateral && !m.invert_longitudinal);
        // Upright camera: the optical axis X runs along the vehicle.
        let m = auto_axis_map([0.01, 0.02, 0.99]);
        assert_eq!(
            (m.lateral, m.longitudinal, m.vertical),
            (Axis::Y, Axis::X, Axis::Z)
        );
        assert!(!m.invert_vertical);
        // Portrait mount: gravity on Y, X still longitudinal.
        let m = auto_axis_map([0.02, -0.98, 0.1]);
        assert_eq!(
            (m.lateral, m.longitudinal, m.vertical),
            (Axis::Z, Axis::X, Axis::Y)
        );
        assert!(m.invert_vertical);
    }

    #[test]
    fn axis_map_parsing() {
        let m = AxisMap::parse("yzx").unwrap();
        assert_eq!(
            (m.lateral, m.longitudinal, m.vertical),
            (Axis::Y, Axis::Z, Axis::X)
        );
        let m = AxisMap::parse("lateral=y,vertical=x,longitudinal=z").unwrap();
        assert_eq!(
            (m.lateral, m.longitudinal, m.vertical),
            (Axis::Y, Axis::Z, Axis::X)
        );
        assert!(AxisMap::parse("xxz").is_err());
        assert!(AxisMap::parse("lateral=q").is_err());
    }

    #[test]
    fn quaternion_gravity_removal_keeps_sustained_acceleration() {
        // Camera level (identity attitude), gravity on +Z, sustained 0.5 g lateral on X.
        let clip = Clip {
            info: ClipInfo::default(),
            samples: (0..60)
                .map(|i| sample(i as f64 / 30.0, [0.5, 0.0, 1.0], Some([1.0, 0.0, 0.0, 0.0])))
                .collect(),
        };
        let mut opts = ProcessOptions::default();
        // gravity is on Z; the quaternion method subtracts the *median* — which includes the
        // sustained 0.5 g on X — so use a mostly-quiet clip with a burst instead:
        let mut samples = clip.samples.clone();
        for s in samples.iter_mut().take(50) {
            s.acc = Some([0.0, 0.0, 1.0]);
        }
        let clip = Clip {
            info: ClipInfo::default(),
            samples,
        };
        opts.gravity = GravityMode::Quaternion;
        let t = process(&clip, &opts);
        assert_eq!(t.gravity_used, GravityMode::Quaternion);
        let g = t.gravity_world.unwrap();
        assert!((g[2] - 1.0).abs() < 1e-9 && g[0].abs() < 1e-9);
        assert!(t.rows[10].lateral.unwrap().abs() < 1e-9);
        assert!((t.rows[55].lateral.unwrap() - 0.5).abs() < 1e-9);
        assert!(t.rows[55].vertical.unwrap().abs() < 1e-9);
        assert!((t.rows[55].combined.unwrap() - 0.5).abs() < 1e-9);
    }

    #[test]
    fn quaternion_method_uses_attitude_when_camera_is_rotated() {
        // Camera rolled 90° about Y: body X points down, so gravity reads on -X.
        // q for 90° about Y: [cos45, 0, sin45, 0]
        let q = [
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ];
        let samples: Vec<Sample> = (0..60)
            .map(|i| {
                let a = if i >= 50 {
                    [-1.0, 0.3, 0.0]
                } else {
                    [-1.0, 0.0, 0.0]
                };
                sample(i as f64 / 30.0, a, Some(q))
            })
            .collect();
        let clip = Clip {
            info: ClipInfo::default(),
            samples,
        };
        let medians = axis_medians(&clip.samples, 1.0);
        let axes = auto_axis_map(medians);
        assert_eq!(axes.vertical, Axis::X);
        let opts = ProcessOptions {
            axes,
            ..ProcessOptions::default()
        };
        let t = process(&clip, &opts);
        // lateral = -Y in this mapping, burst of 0.3 g at the end, vertical ~0 after removal
        assert!(t.rows[10].lateral.unwrap().abs() < 1e-9);
        assert!((t.rows[55].lateral.unwrap() + 0.3).abs() < 1e-9);
        assert!(t.rows[55].vertical.unwrap().abs() < 1e-9);
    }

    #[test]
    fn rotate_onto_moves_vector_and_is_identity_for_aligned_axes() {
        let v = rotate_onto([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
        assert!((v[0]).abs() < 1e-12 && (v[1] - 1.0).abs() < 1e-12 && v[2].abs() < 1e-12);
        let v = rotate_onto([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.3, -0.2, 0.9]);
        assert_eq!(v, [0.3, -0.2, 0.9]);
        // Antipodal: length preserved and `from` lands on `to`.
        let v = rotate_onto([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 0.0, 1.0]);
        assert!((v[2] + 1.0).abs() < 1e-12);
    }

    #[test]
    fn levelling_removes_camera_pitch_from_the_vehicle_axes() {
        // Camera pitched 30° nose-down about body Y; world Z reads +1 g at rest. A burst of
        // 0.5 g straight ahead (world X) must show up entirely as longitudinal once levelled.
        let phi = 30f64.to_radians();
        let q = [(phi / 2.0).cos(), 0.0, (phi / 2.0).sin(), 0.0];
        let samples: Vec<Sample> = (0..60)
            .map(|i| {
                let world = if i >= 50 { [0.5, 0.0, 1.0] } else { [0.0, 0.0, 1.0] };
                sample(i as f64 / 30.0, rotate(q, world, true), Some(q))
            })
            .collect();
        let clip = Clip {
            info: ClipInfo::default(),
            samples,
        };
        let medians = axis_medians(&clip.samples, 1.0);
        assert!((medians[2].abs() - phi.cos()).abs() < 1e-9, "{medians:?}");
        let axes = auto_axis_map(medians);
        assert_eq!(
            (axes.lateral, axes.longitudinal, axes.vertical),
            (Axis::Y, Axis::X, Axis::Z)
        );

        let levelled = process(
            &clip,
            &ProcessOptions {
                axes,
                ..ProcessOptions::default()
            },
        );
        assert!(levelled.levelled);
        assert!((levelled.tilt_deg.unwrap() - 30.0).abs() < 1e-6);
        let r = &levelled.rows[55];
        assert!((r.longitudinal.unwrap().abs() - 0.5).abs() < 1e-9, "{r:?}");
        assert!(r.vertical.unwrap().abs() < 1e-9, "{r:?}");
        assert!(r.lateral.unwrap().abs() < 1e-9, "{r:?}");

        let raw = process(
            &clip,
            &ProcessOptions {
                axes,
                level: false,
                ..ProcessOptions::default()
            },
        );
        assert!(!raw.levelled);
        let r = &raw.rows[55];
        // Without levelling the burst is split by the camera pitch.
        assert!(
            (r.longitudinal.unwrap().abs() - 0.5 * phi.cos()).abs() < 1e-9,
            "{r:?}"
        );
        assert!(
            (r.vertical.unwrap().abs() - 0.5 * phi.sin()).abs() < 1e-9,
            "{r:?}"
        );
    }

    #[test]
    fn levelling_uses_the_median_gravity_direction_without_attitude() {
        // Same geometry, no quaternion: the static tilt still gets levelled out.
        let phi = 30f64.to_radians();
        let q = [(phi / 2.0).cos(), 0.0, (phi / 2.0).sin(), 0.0];
        let samples: Vec<Sample> = (0..90)
            .map(|i| {
                let world = if (50..53).contains(&i) {
                    [0.5, 0.0, 1.0]
                } else {
                    [0.0, 0.0, 1.0]
                };
                sample(i as f64 / 30.0, rotate(q, world, true), None)
            })
            .collect();
        let clip = Clip {
            info: ClipInfo::default(),
            samples,
        };
        let axes = auto_axis_map(axis_medians(&clip.samples, 1.0));
        let t = process(
            &clip,
            &ProcessOptions {
                axes,
                gravity: GravityMode::HighPass,
                ..ProcessOptions::default()
            },
        );
        assert!(t.levelled);
        assert!((t.tilt_deg.unwrap() - 30.0).abs() < 1e-6);
        let r = &t.rows[51];
        // High-pass leaves a little of the burst in the baseline; the split must still be clean.
        assert!(r.vertical.unwrap().abs() < 0.02, "{r:?}");
        assert!(r.longitudinal.unwrap().abs() > 0.4, "{r:?}");
    }

    #[test]
    fn falls_back_to_highpass_without_quaternion() {
        let clip = Clip {
            info: ClipInfo::default(),
            samples: (0..90)
                .map(|i| sample(i as f64 / 30.0, [0.0, 0.0, 1.0], None))
                .collect(),
        };
        let t = process(&clip, &ProcessOptions::default());
        assert_eq!(t.gravity_used, GravityMode::HighPass);
        assert!(t.rows[45].vertical.unwrap().abs() < 1e-9);
    }

    #[test]
    fn euler_angles() {
        let e = quat_to_euler_deg([1.0, 0.0, 0.0, 0.0]).unwrap();
        assert!(e.iter().all(|v| v.abs() < 1e-9));
        let half = 30f64.to_radians() / 2.0;
        let e = quat_to_euler_deg([half.cos(), half.sin(), 0.0, 0.0]).unwrap();
        assert!((e[0] - 30.0).abs() < 1e-9);
    }

    #[test]
    fn gps_derived_speed_distance_and_decimation() {
        let mut samples: Vec<Sample> = Vec::new();
        for i in 0..30 {
            let t = i as f64 / 10.0;
            let mut s = sample(t, [0.0, 0.0, 1.0], Some([1.0, 0.0, 0.0, 0.0]));
            s.gps = Some(GpsFix {
                latitude: 55.0 + 0.0001 * i as f64, // ~11.1 m per step, due north
                longitude: 37.0,
                altitude_m: Some(100.0),
                status: 0,
                velocity: None,
                time: None,
            });
            samples.push(s);
        }
        let clip = Clip {
            info: ClipInfo::default(),
            samples,
        };
        let t = process(&clip, &ProcessOptions::default());
        let r = &t.rows[10];
        assert!((r.speed_ms.unwrap() - 111.2).abs() < 1.0, "{:?}", r.speed_ms);
        assert!((r.heading_deg.unwrap()).abs() < 0.01);
        assert!((t.rows[29].distance_m.unwrap() - 29.0 * 11.12).abs() < 2.0);
        assert!(t.has_gps());

        let dec = decimate(&t.rows, 1.0);
        assert_eq!(dec.len(), 3);
        assert!((dec[1].t - 1.0).abs() < 1e-9);
        assert!(dec[1].has_gps());
        assert_eq!(dec[1].latitude, t.rows[10].latitude);
    }

    #[test]
    fn export_rate_parsing() {
        assert_eq!(ExportRate::parse("native"), Some(ExportRate::Native));
        assert_eq!(ExportRate::parse("10"), Some(ExportRate::Hz(10.0)));
        assert_eq!(ExportRate::parse("1hz"), Some(ExportRate::Hz(1.0)));
        assert_eq!(ExportRate::parse("-3"), None);
    }
}
