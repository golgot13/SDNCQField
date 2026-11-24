//! `sdnc-gnss` — Adaptation en Rust pur de la bibliothèque **RTKLIB** (RTK & PPK).
//!
//! Modules inclus :
//! - [`ephemeris`] : Orbites Kepler, correction relativiste, Sagnac, Saastamoinen, Klobuchar (`ephemeris.c` / `rtkcmn.c`)
//! - [`lambda`] : Résolution d'ambiguïtés entières LAMBDA / MLAMBDA + Ratio Test (`lambda.c`)
//! - [`rtk_engine`] : Positionnement SPP + Filtre de Kalman Double-Différences RTK Float/Fix (`pntpos.c` / `rtkpos.c`)
//! - [`ppk_solver`] : Post-traitement PPK Forward / Backward / Combined (`postpos.c`)
//! - [`rinex`] : Lecture & Enregistrement temps réel RINEX 3.x OBS & NAV
//! - [`rtcm3`], [`ubx`], [`nmea`], [`ntrip`] : Flux temps réel et corrections différentielles
//! - [`controller`] : Pilotage multi-sources thread-safe pour l'interface terrain

pub mod controller;
pub mod ephemeris;
pub mod lambda;
pub mod nmea;
pub mod ntrip;
pub mod ppk_solver;
pub mod receivers;
pub mod rinex;
pub mod rtcm3;
pub mod rtk_engine;
pub mod types;
pub mod ubx;

pub use controller::{GnssController, GnssLiveTelemetry};
pub use ppk_solver::{run_ppk_files, run_ppk_in_memory, PpkJobConfig, PpkReport};
pub use receivers::{ReceiverCapabilities, ReceiverProfile};
pub use rtk_engine::RtkEngine;
pub use types::*;
