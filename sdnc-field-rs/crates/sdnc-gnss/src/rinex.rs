//! Parseur & Enregistreur RINEX 3.x OBS & NAV (pour le workflow PPK complet).
//!
//! Permet :
//! - D'enregistrer sur le terrain les observations brutes du rover au format RINEX 3.04 (`.obs`)
//! - De lire les fichiers RINEX 3 OBS (Rover & Base + `APPROX POSITION XYZ`) et NAV (`.nav`/`.rnx`)
//!   pour le post-traitement PPK (`ppk_solver.rs`).

use crate::types::{BroadcastEphemeris, Constellation, GTime, NavData, ObsEpoch, ObsRecord, SatId};
use anyhow::{Context, Result};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::Path;

#[derive(Debug, Clone, Default)]
pub struct RinexObsFile {
    pub approx_position_ecef: Option<[f64; 3]>,
    pub antenna_delta_h_m: f64,
    pub marker_name: String,
    pub epochs: Vec<ObsEpoch>,
}

/// Lit un fichier RINEX 3.x OBS (`.obs`, `.26o`, `.rnx`) multi-fréquences L1/L2/L5
/// (compatible Emlid Reach, Leica, Trimble, Sokkia/Topcon, u-blox ZED-F9P et stations RGP IGN).
pub fn read_rinex3_obs<P: AsRef<Path>>(path: P) -> Result<RinexObsFile> {
    let file = File::open(path.as_ref())
        .with_context(|| format!("Impossible d'ouvrir le fichier RINEX OBS {:?}", path.as_ref()))?;
    let reader = BufReader::new(file);

    let mut out = RinexObsFile::default();
    let mut sys_obs_types: HashMap<char, Vec<String>> = HashMap::new();
    let mut last_sys_char: Option<char> = None;
    let mut in_header = true;
    let mut current_epoch: Option<ObsEpoch> = None;

    for line_res in reader.lines() {
        let line = line_res?;
        if in_header {
            let label = if line.len() >= 60 { &line[60..] } else { "" };
            if label.contains("END OF HEADER") {
                in_header = false;
                continue;
            }
            if label.contains("MARKER NAME") {
                out.marker_name = line[..line.len().min(60)].trim().to_string();
            } else if label.contains("APPROX POSITION XYZ") {
                let parts: Vec<f64> = line[..line.len().min(60)]
                    .split_whitespace()
                    .filter_map(|s| s.parse().ok())
                    .collect();
                if parts.len() >= 3 {
                    out.approx_position_ecef = Some([parts[0], parts[1], parts[2]]);
                }
            } else if label.contains("ANTENNA: DELTA H/E/N") {
                if let Some(h) = line[..line.len().min(60)]
                    .split_whitespace()
                    .next()
                    .and_then(|s| s.parse::<f64>().ok())
                {
                    out.antenna_delta_h_m = h;
                }
            } else if label.contains("SYS / # / OBS TYPES") {
                let content = &line[..line.len().min(60)];
                let first_c = content.chars().next().unwrap_or(' ');
                let sys_c = if first_c != ' ' {
                    last_sys_char = Some(first_c);
                    Some(first_c)
                } else {
                    last_sys_char
                };
                if let Some(sc) = sys_c {
                    let tokens: Vec<String> = content
                        .get(6..)
                        .unwrap_or("")
                        .split_whitespace()
                        .map(|s| s.to_string())
                        .collect();
                    sys_obs_types.entry(sc).or_default().extend(tokens);
                }
            }
            continue;
        }

        if line.starts_with('>') {
            if let Some(ep) = current_epoch.take() {
                if !ep.records.is_empty() {
                    out.epochs.push(ep);
                }
            }
            let parts: Vec<&str> = line[1..].split_whitespace().collect();
            if parts.len() >= 6 {
                let y: i32 = parts[0].parse().unwrap_or(2026);
                let m: u32 = parts[1].parse().unwrap_or(1);
                let d: u32 = parts[2].parse().unwrap_or(1);
                let hh: u32 = parts[3].parse().unwrap_or(0);
                let mm: u32 = parts[4].parse().unwrap_or(0);
                let ss: f64 = parts[5].parse().unwrap_or(0.0);
                current_epoch = Some(ObsEpoch {
                    time: GTime::from_ymdhms(y, m, d, hh, mm, ss),
                    records: Vec::new(),
                });
            }
        } else if let Some(ep) = current_epoch.as_mut() {
            if line.len() < 4 {
                continue;
            }
            let Some(sat) = SatId::parse(&line[0..3]) else {
                continue;
            };
            let types = sys_obs_types.get(&sat.constellation.prefix());
            let mut rec = ObsRecord::new(sat);

            if let Some(obs_list) = types {
                for (idx, code) in obs_list.iter().enumerate() {
                    let start = 3 + idx * 16;
                    if start >= line.len() {
                        break;
                    }
                    let end = (start + 14).min(line.len());
                    let val: f64 = line[start..end].trim().parse().unwrap_or(0.0);
                    let lli_val: u8 = line
                        .get(start + 14..start + 15)
                        .and_then(|s| s.trim().parse().ok())
                        .unwrap_or(0);
                    if val == 0.0 {
                        continue;
                    }

                    match code.as_str() {
                        // Bande 0 : L1 / E1 / B1
                        "C1C" | "C1W" | "C1X" | "C1P" | "C1S" | "C1L" | "C2I" => {
                            if rec.pseudorange[0] == 0.0 {
                                rec.pseudorange[0] = val;
                            }
                        }
                        "L1C" | "L1W" | "L1X" | "L1P" | "L1S" | "L1L" | "L2I" => {
                            if rec.carrier_phase[0] == 0.0 {
                                rec.carrier_phase[0] = val;
                                rec.lli[0] = lli_val;
                            }
                        }
                        "D1C" | "D1W" | "D1X" | "D2I" => rec.doppler[0] = val as f32,
                        "S1C" | "S1W" | "S1X" | "S2I" => rec.snr[0] = val as f32,

                        // Bande 1 : L2 / E5b (7Q/7X) / B2 (7I/7X)
                        "C2W" | "C2L" | "C2S" | "C2X" | "C2C" | "C2P" | "C7Q" | "C7I" | "C7X" => {
                            if rec.pseudorange[1] == 0.0 {
                                rec.pseudorange[1] = val;
                            }
                        }
                        "L2W" | "L2L" | "L2S" | "L2X" | "L2C" | "L2P" | "L7Q" | "L7I" | "L7X" => {
                            if rec.carrier_phase[1] == 0.0 {
                                rec.carrier_phase[1] = val;
                                rec.lli[1] = lli_val;
                            }
                        }
                        "D2W" | "D2L" | "D2S" | "D2X" | "D2C" | "D7Q" | "D7I" => {
                            rec.doppler[1] = val as f32
                        }
                        "S2W" | "S2L" | "S2S" | "S2X" | "S2C" | "S7Q" | "S7I" => {
                            rec.snr[1] = val as f32
                        }

                        // Bande 2 : L5 / E5a
                        "C5Q" | "C5X" | "C5I" => rec.pseudorange[2] = val,
                        "L5Q" | "L5X" | "L5I" => {
                            rec.carrier_phase[2] = val;
                            rec.lli[2] = lli_val;
                        }
                        "D5Q" | "D5X" => rec.doppler[2] = val as f32,
                        "S5Q" | "S5X" => rec.snr[2] = val as f32,
                        _ => {}
                    }
                }
            }
            ep.records.push(rec);
        }
    }

    if let Some(ep) = current_epoch {
        if !ep.records.is_empty() {
            out.epochs.push(ep);
        }
    }

    Ok(out)
}

fn parse_rinex_f64(slice: &str) -> f64 {
    slice.trim().replace(['D', 'd'], "E").parse().unwrap_or(0.0)
}

/// Lit un fichier RINEX 3.x NAV (éphémérides GPS/Galileo/BeiDou).
pub fn read_rinex3_nav<P: AsRef<Path>>(path: P) -> Result<NavData> {
    let file = File::open(path.as_ref())
        .with_context(|| format!("Impossible d'ouvrir le fichier RINEX NAV {:?}", path.as_ref()))?;
    let lines: Vec<String> = BufReader::new(file).lines().collect::<std::io::Result<_>>()?;

    let mut nav = NavData::default();
    let mut i = 0usize;
    while i < lines.len() {
        let l = &lines[i];
        if l.len() >= 60 && l[60..].contains("END OF HEADER") {
            i += 1;
            break;
        }
        i += 1;
    }

    while i + 7 < lines.len() {
        let l0 = &lines[i];
        let Some(sat) = l0.get(0..3).and_then(SatId::parse) else {
            i += 1;
            continue;
        };
        if !matches!(
            sat.constellation,
            Constellation::Gps | Constellation::Galileo | Constellation::BeiDou | Constellation::Qzss
        ) {
            i += 4; // GLONASS / SBAS ont 4 lignes
            continue;
        }

        let header_nums: Vec<&str> = l0.get(3..23).unwrap_or("").split_whitespace().collect();
        let toc = if header_nums.len() >= 6 {
            GTime::from_ymdhms(
                header_nums[0].parse().unwrap_or(2026),
                header_nums[1].parse().unwrap_or(1),
                header_nums[2].parse().unwrap_or(1),
                header_nums[3].parse().unwrap_or(0),
                header_nums[4].parse().unwrap_or(0),
                header_nums[5].parse().unwrap_or(0.0),
            )
        } else {
            GTime::default()
        };

        let f0 = parse_rinex_f64(l0.get(23..42).unwrap_or(""));
        let f1 = parse_rinex_f64(l0.get(42..61).unwrap_or(""));
        let f2 = parse_rinex_f64(l0.get(61..80).unwrap_or(""));

        let get_field = |row: usize, col: usize| -> f64 {
            let line = &lines[i + row];
            let start = 4 + col * 19;
            let end = (start + 19).min(line.len());
            if start < line.len() {
                parse_rinex_f64(&line[start..end])
            } else {
                0.0
            }
        };

        let iode = get_field(1, 0) as i32;
        let crs = get_field(1, 1);
        let deln = get_field(1, 2);
        let m0 = get_field(1, 3);

        let cuc = get_field(2, 0);
        let e = get_field(2, 1);
        let cus = get_field(2, 2);
        let sqrt_a = get_field(2, 3);

        let toe_s = get_field(3, 0);
        let cic = get_field(3, 1);
        let omg0 = get_field(3, 2);
        let cis = get_field(3, 3);

        let i0 = get_field(4, 0);
        let crc = get_field(4, 1);
        let omg = get_field(4, 2);
        let omgd = get_field(4, 3);

        let idot = get_field(5, 0);
        let week = get_field(5, 2) as i32;
        let sv_health = get_field(6, 1) as u8;
        let tgd = get_field(6, 2);

        let toe = if week > 0 {
            GTime::from_gps_week_tow(week, toe_s)
        } else {
            toc
        };

        nav.add_ephemeris(BroadcastEphemeris {
            sat,
            iode,
            week,
            toe_s,
            toc,
            toe,
            sqrt_a,
            e,
            i0,
            omg0,
            omg,
            m0,
            deln,
            idot,
            omgd,
            cuc,
            cus,
            crc,
            crs,
            cic,
            cis,
            f0,
            f1,
            f2,
            tgd,
            sv_health,
        });

        i += 8;
    }

    Ok(nav)
}

/// Enregistreur temps réel RINEX 3.04 OBS pour l'acquisition terrain PPK.
pub struct Rinex3ObsWriter {
    writer: BufWriter<File>,
    pub epochs_written: usize,
}

impl Rinex3ObsWriter {
    pub fn create<P: AsRef<Path>>(path: P, approx_ecef: [f64; 3], marker_name: &str) -> Result<Self> {
        if let Some(parent) = path.as_ref().parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        let file = File::create(path.as_ref())?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "     3.04           OBSERVATION DATA    M (Mixed)           RINEX VERSION / TYPE"
        )?;
        writeln!(
            writer,
            "SDNCQField-RS       RTKLIB-Rust                             PGM / RUN BY / DATE"
        )?;
        writeln!(writer, "{:<60}MARKER NAME", marker_name)?;
        writeln!(
            writer,
            "{:14.4}{:14.4}{:14.4}                  APPROX POSITION XYZ",
            approx_ecef[0], approx_ecef[1], approx_ecef[2]
        )?;
        for sys in ['G', 'E', 'R', 'C'] {
            writeln!(
                writer,
                "{}    8 C1C L1C D1C S1C C2W L2W D2W S2W                      SYS / # / OBS TYPES",
                sys
            )?;
        }
        writeln!(
            writer,
            "                                                            END OF HEADER"
        )?;
        writer.flush()?;
        Ok(Self {
            writer,
            epochs_written: 0,
        })
    }

    pub fn write_epoch(&mut self, epoch: &ObsEpoch) -> Result<()> {
        let dt = chrono::DateTime::from_timestamp(epoch.time.sec, 0)
            .unwrap_or_default()
            .naive_utc();
        writeln!(
            self.writer,
            "> {:04} {:02} {:02} {:02} {:02} {:10.7}  0 {:2}",
            dt.format("%Y"),
            dt.format("%m"),
            dt.format("%d"),
            dt.format("%H"),
            dt.format("%M"),
            dt.format("%S").to_string().parse::<f64>().unwrap_or(0.0) + epoch.time.frac,
            epoch.records.len()
        )?;
        for r in &epoch.records {
            let lli0 = if r.lli[0] > 0 {
                r.lli[0].to_string()
            } else {
                " ".to_string()
            };
            let lli1 = if r.lli[1] > 0 {
                r.lli[1].to_string()
            } else {
                " ".to_string()
            };
            writeln!(
                self.writer,
                "{}{:14.3}  {:14.3}{} {:14.3}  {:14.3}  {:14.3}  {:14.3}{} {:14.3}  {:14.3}",
                r.sat.code(),
                r.pseudorange[0],
                r.carrier_phase[0],
                lli0,
                r.doppler[0],
                r.snr[0],
                r.pseudorange[1],
                r.carrier_phase[1],
                lli1,
                r.doppler[1],
                r.snr[1]
            )?;
        }
        self.epochs_written += 1;
        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rinex3_obs_write_and_read_roundtrip() {
        let tmp = std::env::temp_dir().join("sdnc_test_rover.obs");
        let mut w =
            Rinex3ObsWriter::create(&tmp, [4580000.0, 440000.0, 4370000.0], "ROVER_01").unwrap();
        let mut rec = ObsRecord::new(SatId::new(Constellation::Gps, 5));
        rec.pseudorange[0] = 21_543_210.123;
        rec.carrier_phase[0] = 113_210_456.789;
        rec.snr[0] = 44.5;
        rec.pseudorange[1] = 21_543_214.456;
        rec.carrier_phase[1] = 88_216_543.210;
        rec.snr[1] = 41.0;
        w.write_epoch(&ObsEpoch {
            time: GTime::from_ymdhms(2026, 11, 21, 10, 15, 30.0),
            records: vec![rec],
        })
        .unwrap();
        w.flush().unwrap();

        let parsed = read_rinex3_obs(&tmp).unwrap();
        assert_eq!(parsed.marker_name, "ROVER_01");
        assert_eq!(parsed.epochs.len(), 1);
        assert_eq!(parsed.epochs[0].records.len(), 1);
        assert!((parsed.epochs[0].records[0].pseudorange[0] - 21_543_210.123).abs() < 1e-3);
        assert!((parsed.epochs[0].records[0].pseudorange[1] - 21_543_214.456).abs() < 1e-3);
        assert!((parsed.epochs[0].records[0].carrier_phase[1] - 88_216_543.210).abs() < 1e-3);
        let _ = std::fs::remove_file(&tmp);
    }
}
