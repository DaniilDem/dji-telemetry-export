//! DJI Osmo Action telemetry extraction and export.
//!
//! Pipeline: [`mp4`] locates the `djmd` track → [`dji`] decodes the protobuf
//! samples → [`process`] turns raw sensor data into vehicle-frame telemetry →
//! [`export`] writes CSV / SRT / VBO / GPX / FIT / IGC files for OVRLEY.

pub mod cli;
pub mod dji;
pub mod export;
pub mod gui;
pub mod mp4;
pub mod process;
pub mod protobuf;
