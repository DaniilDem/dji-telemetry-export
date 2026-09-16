# DJI Telemetry Export

[Русская версия](README.ru.md)

Pulls the telemetry that DJI Osmo Action cameras embed in their MP4 files and writes it as
**CSV, SRT, VBO, GPX, FIT or IGC** — ready to import into [OVRLEY](https://github.com/sstoychev/OVRLEY)
(or anything else that reads those formats).

> **Visualising the data:** use [OVRLEY](https://www.ovrley.cc/) — a free, offline desktop app that turns
> telemetry files into customisable video overlays (G-force meters, speed, maps, gauges). Export a file
> with this tool, import it into OVRLEY together with the video, and render the overlay. See
> [Importing into OVRLEY](#importing-into-ovrley) below.

One self-contained executable for Windows and macOS, with a small GUI and a full command line.

![GUI screenshot](docs/gui.png)

## Why

DJI Osmo Action 4/5/6 write a `djmd` timed-metadata track: one protobuf message per video frame with the
accelerometer, the attitude quaternion, ISO / shutter / colour temperature and — when the **AC004 GPS
remote** is paired — position, altitude, velocity and GPS time.

OVRLEY already parses that track, but it keeps only samples with a valid GPS fix. If the remote had no
fix (or was not connected) the clip shows up as "no telemetry" even though the accelerometer data is
all there. This tool reads the same track directly, removes gravity, maps the camera axes onto the
vehicle, and exports what the clip actually contains.

## Download

Grab the latest build from the **Releases** page:

| Platform | File | Notes |
|---|---|---|
| Windows 10/11 x64 | `dji-telemetry-export-<version>-windows-x64.zip` | Unzip, run `dji-telemetry-export.exe`. SmartScreen may ask once ("More info" → "Run anyway"). |
| macOS 11+ (Apple Silicon + Intel) | `dji-telemetry-export-<version>-macos-universal.dmg` | Not notarised: right-click → **Open** the first time, or `xattr -cr "/Applications/DJI Telemetry Export.app"`. |

No installer, no runtime, no network access.

## Using the GUI

1. **Open video…** (or drop the MP4 onto the window / onto the executable).
2. Check the **Clip** panel: camera, sample rate, accelerometer unit, GPS status, start time.
3. **Processing**
   * *Gravity removal* — the default uses the attitude quaternion (keeps sustained cornering G);
     a moving-average high-pass and "keep gravity" are also available.
   * *Body axis to vehicle axis* — the axis whose median reads ±1 g holds gravity and is pre-selected
     as **Vertical**. The camera's X axis is its optical axis, so on a normal forward-facing mount
     it is pre-selected as **Longitudinal** (along the vehicle) and the remaining axis as
     **Lateral** (across it). Swap them if the camera looks sideways. Signs follow what a G-force
     gauge is expected to show — the direction the driver is thrown: braking reads negative
     longitudinal, a **left**-hand corner reads **positive** lateral. Use *invert* if your setup
     comes out the other way round. On the Osmo Action 5 Pro (X forward, Y right) the auto
     mapping is `lateral = -Y, longitudinal = X`.
   * *G-force gauge* — two switches, *Mirror left / right* and *Swap braking / acceleration*, that
     flip a channel relative to the default above (same effect as the per-axis *invert* boxes, in
     gauge terms).
   * *Level to the vehicle* — on by default. Cameras are rarely mounted dead level; a camera pitched
     30° down would otherwise leak half of every bump into the longitudinal channel and report
     only 87 % of the real braking G. The readings are rotated per sample (using the attitude
     quaternion) so that gravity sits exactly on the vertical axis before the split. The detected
     tilt is shown next to the checkbox and in `--info`.
   * *Export rate* — native (one row per frame) or 10 / 5 / 1 Hz.
4. **Export** — tick the formats and press *Export*. Files are written next to the video as
   `<clip name>.<ext>` unless you choose another folder.

Formats that need a GPS position are greyed out with the reason when the clip has no fix.

### Importing into OVRLEY

Import the video, then import the exported file as the activity. Row 0 of every export is video
frame 0, so the sync offset is **0**. For a G-force gauge use the CSV: OVRLEY reads the
`Lateral acceleration (g)`, `Longitudinal acceleration (g)`, `Vertical acceleration (g)` and
`Combined acceleration (g)` columns directly (and latitude / longitude / altitude / speed / heading
when GPS is present). OVRLEY's gauge draws positive lateral to the right and positive longitudinal
*downwards* (screen coordinates); with the default signs the dot therefore moves up under braking
and right in a left-hand corner — the way the driver is thrown.

## Command line

```
dji-telemetry-export <VIDEO> [OPTIONS]

  -f, --formats <LIST>     csv,srt,vbo,gpx,fit,igc or "all"        [default: csv]
  -o, --out <DIR>          output directory                        [default: next to the video]
      --gravity <MODE>     quat | highpass | none                  [default: quat]
      --highpass-window <S>  window for --gravity highpass         [default: 1.0]
      --axes <SPEC>        auto | xyz | lateral=x,longitudinal=y,vertical=z   [default: auto]
      --invert <LIST>      lateral,longitudinal,vertical
      --no-level           keep the camera's own pitch / roll instead of levelling to the vehicle
      --rate <RATE>        native | <Hz>                           [default: native]
      --unit <g|ms2|mg>    override accelerometer unit detection
      --raw-axes           add raw X/Y/Z columns (CSV)
      --no-orientation     drop roll/pitch/yaw columns (CSV)
      --no-exposure        drop ISO/shutter/colour-temperature columns (CSV)
      --lean-angle         write camera roll as "Lean angle" (CSV)
      --info [--json]      print the clip summary and format availability, then exit
      --inspect [N]        dump the protobuf tree of the first N metadata samples
```

Examples:

```sh
dji-telemetry-export DJI_20260916095953_0098_D.MP4 -f csv,vbo
dji-telemetry-export DJI_0001.MP4 -f all -o ./telemetry --rate 10
dji-telemetry-export DJI_0001.MP4 --info
dji-telemetry-export DJI_0001.MP4 --inspect 3
```

Exit code `2` means at least one requested format was skipped (e.g. GPX without GPS); the other
formats are still written. Starting the program with a single file path and no options opens the GUI
with that clip loaded (that is what dragging a file onto the executable does).

## What is exported

| Format | Needs GPS | Contents |
|---|---|---|
| **CSV** | no | `Elapsed time (s)`, lateral / longitudinal / vertical / combined acceleration (g), roll / pitch / yaw, ISO, shutter, colour temperature; with GPS: latitude, longitude, altitude, speed, heading, distance, UTC time. Header names match OVRLEY's CSV aliases. |
| **SRT** | no | DJI-style subtitle telemetry, one cue per frame: `[iso: …] [shutter: 1/…] [ct: …]` plus `[latitude: …] [longitude: …] [rel_alt: … abs_alt: …]` when GPS is present. Timestamps are the camera clock, like DJI's own SRT files. |
| **VBO** | no | Racelogic VBO text in the RaceBox layout: `time`, `LongAcc`, `LatAcc`, `VertAcc`, and with GPS `lat`, `long` (minutes, west positive), `velocity kmh`, `heading`, `height`, `sats`. |
| **GPX** | yes | GPX 1.1 track points with `<ele>`, `<time>` and extensions `speed`, `heading`, `distance`, `g_force`, `lateral_g`, `longitudinal_g`, `vertical_g`. Default 10 Hz. |
| **FIT** | yes | Garmin FIT activity (`file_id`, `record`, `session`, `activity`). FIT record timestamps are whole seconds, so records are written at 1 Hz. |
| **IGC** | yes | IGC flight log with one `B` record per second and `GSP` (ground speed) / `TRT` (track) extensions. |

Absolute time comes from the GPS time string when there is a fix, otherwise from the
`DJI_YYYYMMDDHHMMSS` file name (camera clock). The camera's UTC offset is derived from the MP4
creation time and used for the UTC column, GPX, FIT and IGC.

### A note on "Lean angle"

The CSV can duplicate camera roll into an OVRLEY `Lean angle (deg)` column (`--lean-angle`). It is off
by default because OVRLEY back-fills lateral G from a lean-angle column, which would overwrite the
real accelerometer data.

## How it works

* `src/mp4.rs` — walks the ISO-BMFF boxes, reads only the `moov` atom and the sample table of the
  `djmd` track, then fetches each sample with seek + read. A 7.5 GB clip takes well under a second.
* `src/protobuf.rs` — a schema-less protobuf wire decoder.
* `src/dji.rs` — interprets the `dvtm_ac20x.proto` field numbers (documented at the top of the file;
  sources: DJI's `dvtm_library.proto` as published in
  [telemetry-parser](https://github.com/AdrianEddy/telemetry-parser) and the
  [ExifTool DJI tag table](https://exiftool.org/TagNames/DJI.html)).
* `src/process.rs` — unit detection, gravity removal, axis mapping, Euler angles, GPS-derived
  speed / heading / distance, decimation.
* `src/export/` — one module per format.
* `src/gui.rs` (egui) and `src/cli.rs` share the same pipeline; `src/main.rs` picks one.

Tested on **DJI Osmo Action 5 Pro** (`dvtm_ac204.proto` 2.0.1, firmware 10.00.16.13) with the AC004
remote connected but without a GPS fix. Osmo Action 4 (`ac203`) and 6 (`ac206`) use the same field
layout according to ExifTool. The GPS path is covered by synthetic tests only until a clip with a fix
is available — please open an issue with a short sample if something looks off.

## Building

```sh
cargo build --release          # binary in target/release/
cargo test                     # unit + integration tests
cargo run -- clip.MP4 --info
```

Requires a stable Rust toolchain (1.80+). `tools/extract_fixture.py <clip> <out.bin> [N]` cuts the
first N metadata samples out of a clip to create a small test fixture.

Tagging `vX.Y.Z` triggers `.github/workflows/release.yml`, which builds the Windows zip and the
macOS universal `.app`/`.dmg` and attaches them to the GitHub Release.

## Licence

MIT.
