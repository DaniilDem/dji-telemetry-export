//! egui front end: open a clip, review what it contains, tweak processing, export.

use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};
use std::sync::Arc;
use std::thread;

use eframe::egui::{self, Color32, RichText};

use crate::cli::{open_clip, summary_lines};
use crate::dji::Clip;
use crate::export::{self, ExportOptions, Format, APP_NAME, APP_VERSION};
use crate::process::{self, Axis, AxisMap, ExportRate, GravityMode, ProcessOptions, Telemetry};

const WINDOW_TITLE: &str = "DJI Telemetry Export";

enum Msg {
    Progress(usize, usize),
    Done(Box<Result<Clip, String>>),
}

enum LoadState {
    Idle,
    Loading { done: usize, total: usize },
    Loaded(Arc<Clip>),
    Failed(String),
}

#[derive(Clone, Copy, PartialEq)]
enum RateChoice {
    Native,
    Hz10,
    Hz5,
    Hz1,
}

impl RateChoice {
    const ALL: [RateChoice; 4] = [
        RateChoice::Native,
        RateChoice::Hz10,
        RateChoice::Hz5,
        RateChoice::Hz1,
    ];
    fn label(self) -> &'static str {
        match self {
            RateChoice::Native => "Native (format defaults: GPX 10 Hz, FIT/IGC 1 Hz)",
            RateChoice::Hz10 => "10 Hz",
            RateChoice::Hz5 => "5 Hz",
            RateChoice::Hz1 => "1 Hz",
        }
    }
    fn rate(self) -> ExportRate {
        match self {
            RateChoice::Native => ExportRate::Native,
            RateChoice::Hz10 => ExportRate::Hz(10.0),
            RateChoice::Hz5 => ExportRate::Hz(5.0),
            RateChoice::Hz1 => ExportRate::Hz(1.0),
        }
    }
}

pub struct App {
    state: LoadState,
    rx: Option<Receiver<Msg>>,
    video_path: Option<PathBuf>,
    process_options: ProcessOptions,
    auto_axes: AxisMap,
    export_options: ExportOptions,
    rate_choice: RateChoice,
    selected: [bool; 6],
    out_dir: Option<PathBuf>,
    telemetry: Option<Telemetry>,
    dirty: bool,
    log: Vec<(bool, String)>,
}

impl Default for App {
    fn default() -> Self {
        App {
            state: LoadState::Idle,
            rx: None,
            video_path: None,
            process_options: ProcessOptions::default(),
            auto_axes: AxisMap::default(),
            export_options: ExportOptions::default(),
            rate_choice: RateChoice::Native,
            selected: [true, false, false, false, false, false],
            out_dir: None,
            telemetry: None,
            dirty: false,
            log: Vec::new(),
        }
    }
}

pub fn run(initial: Option<PathBuf>) -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(WINDOW_TITLE)
            .with_inner_size([780.0, 900.0])
            .with_min_inner_size([560.0, 480.0])
            .with_drag_and_drop(true)
            .with_icon(icon()),
        centered: true,
        ..Default::default()
    };
    eframe::run_native(
        WINDOW_TITLE,
        options,
        Box::new(move |cc| {
            let mut app = App::default();
            if let Some(p) = initial {
                app.start_loading(&cc.egui_ctx, p);
            }
            Ok(Box::new(app))
        }),
    )
}

/// A simple procedural icon: dark rounded square with a green "G" arc.
fn icon() -> egui::IconData {
    let size = 64u32;
    let mut rgba = vec![0u8; (size * size * 4) as usize];
    let c = (size as f32 - 1.0) / 2.0;
    for y in 0..size {
        for x in 0..size {
            let dx = x as f32 - c;
            let dy = y as f32 - c;
            let r = (dx * dx + dy * dy).sqrt();
            let i = ((y * size + x) * 4) as usize;
            if r <= c {
                let ring = (r - c * 0.62).abs() < c * 0.12;
                let angle = dy.atan2(dx);
                let gap = angle > -0.5 && angle < 0.5;
                let dot = (dx * dx + dy * dy).sqrt() < c * 0.18;
                let (cr, cg, cb) = if (ring && !gap) || dot {
                    (76, 217, 100)
                } else {
                    (28, 30, 36)
                };
                rgba[i] = cr;
                rgba[i + 1] = cg;
                rgba[i + 2] = cb;
                rgba[i + 3] = 255;
            }
        }
    }
    egui::IconData {
        rgba,
        width: size,
        height: size,
    }
}

impl App {
    fn start_loading(&mut self, ctx: &egui::Context, path: PathBuf) {
        let (tx, rx) = mpsc::channel();
        self.rx = Some(rx);
        self.state = LoadState::Loading { done: 0, total: 0 };
        self.telemetry = None;
        self.log.clear();
        self.video_path = Some(path.clone());
        if self.out_dir.is_none() {
            self.out_dir = path.parent().map(Path::to_path_buf);
        }
        let ctx = ctx.clone();
        thread::spawn(move || {
            let tx_progress = tx.clone();
            let ctx_progress = ctx.clone();
            let result = open_clip(&path, |done, total| {
                let _ = tx_progress.send(Msg::Progress(done, total));
                ctx_progress.request_repaint();
            })
            .map_err(|e| format!("{e:#}"));
            let _ = tx.send(Msg::Done(Box::new(result)));
            ctx.request_repaint();
        });
    }

    fn poll(&mut self) {
        let Some(rx) = &self.rx else { return };
        let mut finished = false;
        while let Ok(msg) = rx.try_recv() {
            match msg {
                Msg::Progress(done, total) => {
                    if let LoadState::Loading { done: d, total: t } = &mut self.state {
                        *d = done;
                        *t = total;
                    }
                }
                Msg::Done(result) => {
                    finished = true;
                    match *result {
                        Ok(clip) => {
                            let (unit, _) = process::detect_accel_unit(&clip.samples);
                            let medians = process::axis_medians(&clip.samples, unit.scale_to_g());
                            self.auto_axes = process::auto_axis_map(medians);
                            self.process_options.axes = self.auto_axes;
                            self.state = LoadState::Loaded(Arc::new(clip));
                            self.dirty = true;
                        }
                        Err(e) => self.state = LoadState::Failed(e),
                    }
                }
            }
        }
        if finished {
            self.rx = None;
        }
    }

    fn reprocess(&mut self) {
        if !self.dirty {
            return;
        }
        if let LoadState::Loaded(clip) = &self.state {
            let telemetry = process::process(clip, &self.process_options);
            // Default the format selection: CSV always; add GPX when GPS is present.
            if self.telemetry.is_none() {
                self.selected = [true, false, false, telemetry.has_gps(), false, false];
            }
            self.telemetry = Some(telemetry);
        }
        self.dirty = false;
    }

    fn pick_video(&mut self, ctx: &egui::Context) {
        let mut dialog = rfd::FileDialog::new().add_filter("DJI video", &["mp4", "MP4", "mov", "MOV"]);
        if let Some(dir) = self.video_path.as_ref().and_then(|p| p.parent()) {
            dialog = dialog.set_directory(dir);
        }
        if let Some(path) = dialog.pick_file() {
            self.start_loading(ctx, path);
        }
    }

    fn do_export(&mut self) {
        let Some(telemetry) = &self.telemetry else {
            return;
        };
        let Some(video) = &self.video_path else {
            return;
        };
        let out_dir = self
            .out_dir
            .clone()
            .or_else(|| video.parent().map(Path::to_path_buf))
            .unwrap_or_else(|| PathBuf::from("."));
        if let Err(e) = std::fs::create_dir_all(&out_dir) {
            self.log
                .push((false, format!("Cannot create {}: {e}", out_dir.display())));
            return;
        }
        let stem = video
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("telemetry")
            .to_string();
        let source = video
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("video")
            .to_string();
        self.export_options.rate = self.rate_choice.rate();
        let mut log = Vec::new();
        for (i, format) in Format::ALL.iter().enumerate() {
            if !self.selected[i] {
                continue;
            }
            match export::export_file(*format, telemetry, &self.export_options, &out_dir, &stem, &source) {
                Ok(w) => log.push((
                    true,
                    format!(
                        "{} → {} ({} rows, {} KB)",
                        w.format.label(),
                        w.path.display(),
                        w.rows,
                        w.bytes.div_ceil(1024)
                    ),
                )),
                Err(e) => log.push((false, format!("{}: {e}", format.label()))),
            }
        }
        if log.is_empty() {
            log.push((false, "No format selected.".to_string()));
        }
        self.log = log;
    }

    fn ui_header(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        ui.horizontal(|ui| {
            ui.heading(WINDOW_TITLE);
            ui.label(RichText::new(format!("v{APP_VERSION}")).weak());
        });
        ui.label("Extracts accelerometer, attitude, exposure and GPS telemetry from DJI Osmo Action clips for OVRLEY.");
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            let loading = matches!(self.state, LoadState::Loading { .. });
            if ui
                .add_enabled(!loading, egui::Button::new("Open video…"))
                .clicked()
            {
                self.pick_video(ctx);
            }
            match &self.video_path {
                Some(p) => {
                    ui.label(RichText::new(p.display().to_string()).monospace());
                }
                None => {
                    ui.label(RichText::new("…or drop an MP4 file onto this window").weak());
                }
            }
        });
    }

    fn ui_state(&mut self, ui: &mut egui::Ui) {
        match &self.state {
            LoadState::Idle => {}
            LoadState::Loading { done, total } => {
                let frac = if *total > 0 {
                    *done as f32 / *total as f32
                } else {
                    0.0
                };
                ui.add(
                    egui::ProgressBar::new(frac)
                        .show_percentage()
                        .text("Reading metadata track…"),
                );
            }
            LoadState::Failed(e) => {
                ui.colored_label(
                    Color32::from_rgb(220, 80, 80),
                    format!("Could not read this file: {e}"),
                );
            }
            LoadState::Loaded(_) => {}
        }
    }

    fn ui_info(&self, ui: &mut egui::Ui) {
        let Some(t) = &self.telemetry else { return };
        egui::CollapsingHeader::new("Clip").default_open(true).show(ui, |ui| {
            egui::Grid::new("info").num_columns(2).spacing([16.0, 4.0]).striped(true).show(ui, |ui| {
                for (k, v) in summary_lines(t) {
                    ui.label(RichText::new(k).strong());
                    ui.label(v);
                    ui.end_row();
                }
            });
            if !t.has_gps() {
                ui.add_space(4.0);
                ui.label(
                    RichText::new(
                        "No GPS fix in this clip: only accelerometer / attitude / exposure data can be exported. \
                         GPX, FIT and IGC are disabled below.",
                    )
                    .color(Color32::from_rgb(230, 170, 60)),
                );
            }
        });
    }

    /// Checkbox that flips a vehicle axis relative to the auto-detected sign. It is the same
    /// `invert` flag as in the axis rows, presented in gauge terms: "on" means "the opposite of
    /// the default", whatever the raw axis sign happens to be.
    fn flip_toggle(&mut self, ui: &mut egui::Ui, label: &str, which: usize) {
        let (invert, auto) = match which {
            0 => (
                &mut self.process_options.axes.invert_lateral,
                self.auto_axes.invert_lateral,
            ),
            _ => (
                &mut self.process_options.axes.invert_longitudinal,
                self.auto_axes.invert_longitudinal,
            ),
        };
        let mut flipped = *invert != auto;
        if ui.checkbox(&mut flipped, label).changed() {
            *invert = auto != flipped;
            self.dirty = true;
        }
    }

    fn axis_row(&mut self, ui: &mut egui::Ui, name: &str, which: usize) {
        let (axis, invert) = match which {
            0 => (
                &mut self.process_options.axes.lateral,
                &mut self.process_options.axes.invert_lateral,
            ),
            1 => (
                &mut self.process_options.axes.longitudinal,
                &mut self.process_options.axes.invert_longitudinal,
            ),
            _ => (
                &mut self.process_options.axes.vertical,
                &mut self.process_options.axes.invert_vertical,
            ),
        };
        ui.label(name);
        let before = *axis;
        egui::ComboBox::from_id_salt(("axis", which))
            .selected_text(axis.label())
            .width(60.0)
            .show_ui(ui, |ui| {
                for a in Axis::ALL {
                    ui.selectable_value(axis, a, a.label());
                }
            });
        if ui.checkbox(invert, "invert").changed() {
            self.dirty = true;
        }
        let median = self.telemetry.as_ref().map(|t| t.axis_medians[axis.index()]);
        ui.label(
            RichText::new(match median {
                Some(m) => format!("median {m:+.3} g"),
                None => String::new(),
            })
            .weak(),
        );
        ui.end_row();
        if before != *axis {
            self.dirty = true;
        }
    }

    fn ui_options(&mut self, ui: &mut egui::Ui) {
        if self.telemetry.is_none() {
            return;
        }
        egui::CollapsingHeader::new("Processing").default_open(true).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label("Gravity removal");
                let mut g = self.process_options.gravity;
                egui::ComboBox::from_id_salt("gravity")
                    .selected_text(g.label())
                    .show_ui(ui, |ui| {
                        for m in GravityMode::ALL {
                            ui.selectable_value(&mut g, m, m.label());
                        }
                    });
                if g != self.process_options.gravity {
                    self.process_options.gravity = g;
                    self.dirty = true;
                }
                if g == GravityMode::HighPass {
                    ui.label("window");
                    if ui
                        .add(egui::DragValue::new(&mut self.process_options.highpass_window_s).range(0.1..=10.0).speed(0.1).suffix(" s"))
                        .changed()
                    {
                        self.dirty = true;
                    }
                }
            });
            if let Some(t) = &self.telemetry {
                if self.process_options.gravity == GravityMode::Quaternion && t.gravity_used != GravityMode::Quaternion {
                    ui.colored_label(Color32::from_rgb(230, 170, 60), "No usable attitude quaternion — fell back to the high-pass method.");
                }
            }
            ui.add_space(4.0);
            ui.label(RichText::new("Body axis to vehicle axis").strong());
            ui.label(
                RichText::new(
                    "The axis whose median is close to ±1 g holds gravity and should be Vertical. \
                     Pick which of the remaining axes points across the vehicle (lateral) and along it (longitudinal); \
                     use \"invert\" if right/forward come out negative.",
                )
                .weak(),
            );
            egui::Grid::new("axes").num_columns(4).spacing([10.0, 4.0]).show(ui, |ui| {
                self.axis_row(ui, "Lateral", 0);
                self.axis_row(ui, "Longitudinal", 1);
                self.axis_row(ui, "Vertical", 2);
            });
            ui.horizontal(|ui| {
                if ui.button("Reset to auto-detected").clicked() {
                    self.process_options.axes = self.auto_axes;
                    self.dirty = true;
                }
                if !self.process_options.axes.is_permutation() {
                    ui.colored_label(Color32::from_rgb(220, 80, 80), "Each of X, Y, Z must be used exactly once.");
                }
            });
            ui.add_space(4.0);
            ui.label(RichText::new("G-force gauge").strong());
            ui.label(
                RichText::new(
                    "Defaults move the dot the way the driver is thrown: up under braking, right in a left-hand corner.                      Flip either direction here if your gauge shows the opposite.",
                )
                .weak(),
            );
            ui.horizontal(|ui| {
                self.flip_toggle(ui, "Mirror left / right", 0);
                self.flip_toggle(ui, "Swap braking / acceleration", 1);
            });
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui
                    .checkbox(&mut self.process_options.level, "Level to the vehicle (compensate camera tilt)")
                    .on_hover_text(
                        "Rotates the readings so that gravity sits exactly on the Vertical axis before the split.                          A camera pitched down 30° would otherwise leak half of every bump into the longitudinal channel.",
                    )
                    .changed()
                {
                    self.dirty = true;
                }
                if let Some(tilt) = self.telemetry.as_ref().and_then(|t| t.tilt_deg) {
                    ui.label(RichText::new(format!("camera tilt {tilt:.1}°")).weak());
                }
            });
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Export rate");
                egui::ComboBox::from_id_salt("rate")
                    .selected_text(self.rate_choice.label())
                    .show_ui(ui, |ui| {
                        for r in RateChoice::ALL {
                            ui.selectable_value(&mut self.rate_choice, r, r.label());
                        }
                    });
            });
            ui.horizontal_wrapped(|ui| {
                ui.checkbox(&mut self.export_options.include_orientation, "Roll / pitch / yaw columns");
                ui.checkbox(&mut self.export_options.include_exposure, "ISO / shutter / colour temperature");
                ui.checkbox(&mut self.export_options.include_raw_axes, "Raw X / Y / Z (with gravity)");
            });
            ui.checkbox(&mut self.export_options.lean_angle_from_roll, "Write camera roll as \"Lean angle\" (CSV)")
                .on_hover_text("Off by default: OVRLEY back-fills lateral G from a lean-angle column and would overwrite the accelerometer data.");
        });
    }

    fn ui_formats(&mut self, ui: &mut egui::Ui) {
        let Some(t) = self.telemetry.clone() else {
            return;
        };
        egui::CollapsingHeader::new("Export")
            .default_open(true)
            .show(ui, |ui| {
                for (i, format) in Format::ALL.iter().enumerate() {
                    let avail = export::availability(*format, &t);
                    ui.horizontal(|ui| {
                        let enabled = avail.is_ok();
                        if !enabled {
                            self.selected[i] = false;
                        }
                        let resp = ui.add_enabled(
                            enabled,
                            egui::Checkbox::new(&mut self.selected[i], format.label()),
                        );
                        let resp = resp.on_hover_text(format.description());
                        match &avail {
                            Ok(()) => {
                                ui.label(RichText::new(format.description()).weak());
                            }
                            Err(reason) => {
                                resp.on_disabled_hover_text(reason.clone());
                                ui.label(RichText::new(reason).color(Color32::from_rgb(230, 170, 60)));
                            }
                        }
                    });
                }
                ui.add_space(6.0);
                ui.horizontal(|ui| {
                    ui.label("Output folder");
                    if ui.button("Choose…").clicked() {
                        let mut dialog = rfd::FileDialog::new();
                        if let Some(d) = &self.out_dir {
                            dialog = dialog.set_directory(d);
                        }
                        if let Some(dir) = dialog.pick_folder() {
                            self.out_dir = Some(dir);
                        }
                    }
                    ui.label(
                        RichText::new(
                            self.out_dir
                                .as_ref()
                                .map(|p| p.display().to_string())
                                .unwrap_or_default(),
                        )
                        .monospace(),
                    );
                });
                ui.add_space(6.0);
                let any = self.selected.iter().any(|s| *s) && self.process_options.axes.is_permutation();
                if ui
                    .add_enabled(
                        any,
                        egui::Button::new(RichText::new("Export").strong()).min_size([120.0, 28.0].into()),
                    )
                    .clicked()
                {
                    self.do_export();
                }
                for (ok, line) in &self.log {
                    let color = if *ok {
                        Color32::from_rgb(76, 217, 100)
                    } else {
                        Color32::from_rgb(220, 80, 80)
                    };
                    ui.colored_label(color, line);
                }
                if self.log.iter().any(|(ok, _)| *ok) {
                    ui.label(
                        RichText::new(
                            "In OVRLEY: import the video, then import the exported file as the activity; \
                         row 0 is video frame 0, so the sync offset is 0.",
                        )
                        .weak(),
                    );
                }
            });
    }
}

impl eframe::App for App {
    fn ui(&mut self, root: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = &root.ctx().clone();
        self.poll();
        self.reprocess();

        // Drag & drop
        let dropped: Vec<PathBuf> = ctx.input(|i| {
            i.raw
                .dropped_files
                .iter()
                .map(|f| f.path().to_path_buf())
                .collect()
        });
        if let Some(path) = dropped.into_iter().find(|p| p.is_file()) {
            self.start_loading(ctx, path);
        }
        let hovering = ctx.input(|i| !i.raw.hovered_files.is_empty());

        egui::CentralPanel::default().show(root, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    self.ui_header(ui, ctx);
                    ui.separator();
                    self.ui_state(ui);
                    self.ui_info(ui);
                    self.ui_options(ui);
                    self.ui_formats(ui);
                    ui.add_space(12.0);
                    ui.label(
                        RichText::new(format!("{APP_NAME} {APP_VERSION} — MIT licence"))
                            .weak()
                            .small(),
                    );
                });
        });

        if hovering {
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Foreground,
                egui::Id::new("drop_overlay"),
            ));
            let rect = ctx.content_rect();
            painter.rect_filled(rect, 0.0, Color32::from_black_alpha(120));
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Drop the MP4 to open it",
                egui::FontId::proportional(28.0),
                Color32::WHITE,
            );
        }
    }
}
