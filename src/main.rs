#![cfg_attr(all(windows, not(debug_assertions)), windows_subsystem = "windows")]

use clap::Parser;
use dji_telemetry_export::{cli, gui};

/// On Windows the release binary is a GUI-subsystem executable (no console window when
/// double-clicked). When it is started from a terminal with arguments we attach to that
/// terminal so CLI output is visible.
#[cfg(windows)]
fn attach_parent_console() {
    use windows_sys::Win32::System::Console::{AttachConsole, ATTACH_PARENT_PROCESS};
    // SAFETY: plain Win32 call with a documented constant; failure (no parent console) is harmless.
    unsafe {
        AttachConsole(ATTACH_PARENT_PROCESS);
    }
}

#[cfg(not(windows))]
fn attach_parent_console() {}

fn main() {
    let args: Vec<std::ffi::OsString> = std::env::args_os().collect();
    let cli_mode = args.len() > 1
        && !(args.len() == 2 && std::path::Path::new(&args[1]).is_file() && !looks_like_flag(&args[1]));

    if cli_mode {
        attach_parent_console();
        let parsed = match cli::Args::try_parse() {
            Ok(a) => a,
            Err(e) => {
                // clap prints help/version/errors itself
                let _ = e.print();
                std::process::exit(if e.use_stderr() { 2 } else { 0 });
            }
        };
        match cli::run(parsed) {
            Ok(code) => std::process::exit(code),
            Err(e) => {
                eprintln!("error: {e:#}");
                std::process::exit(1);
            }
        }
    }

    // GUI mode: no args, or exactly one existing file (drag a clip onto the executable).
    let initial = if args.len() == 2 {
        Some(std::path::PathBuf::from(&args[1]))
    } else {
        None
    };
    if let Err(e) = gui::run(initial) {
        attach_parent_console();
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn looks_like_flag(arg: &std::ffi::OsString) -> bool {
    arg.to_string_lossy().starts_with('-')
}
