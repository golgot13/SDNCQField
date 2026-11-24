//! Décodeur RTCM 3.3 complet bit-à-bit (CRC-24Q + Station ARP 1005/1006 + MSM4 & MSM7).
//!
//! Portage fidèle de `rtcm3.c` de RTKLIB (standard RTCM 10403.3) :
//! - Vérification CRC-24Q (`0x1864CFB`)
//! - Messages 1005 / 1006 : Position ARP de la station de base en ECEF (résolution 0.1 mm)
//! - Messages **MSM4** (`1074`, `1084`, `1094`, `1114`, `1124`) et **MSM7** (`1077`, `1087`, `1097`, `1117`, `1127`) :
//!   - Décodage complet de l'en-tête 169 bits (masque satellites 64 bits, masque signaux 32 bits, masque cellules)
//!   - Décodage des données satellites (rough range ms entier + mod 1 ms + extended info + phase range rate)
//!   - Décodage des données signaux (fine pseudorange 15/20 bits, fine phase range 22/24 bits, lock time 4/10 bits,
//!     half-cycle ambiguity, CNR 6/10 bits, fine Doppler 15 bits)
//!   - Reconstitution d'un [`ObsEpoch`] prêt à alimenter directement [`crate::RtkEngine`] !

use crate::types::{
    BroadcastEphemeris, Constellation, GTime, NavData, ObsEpoch, ObsRecord, SatId, CLIGHT, LAMBDA1,
    LAMBDA2, LAMBDA5,
};
use std::collections::HashMap;
use std::f64::consts::PI;

const RANGE_MS: f64 = CLIGHT * 0.001; // ~299792.458 m / ms
const P2_5: f64 = 1.0 / 32.0;
const P2_10: f64 = 1.0 / 1024.0;
const P2_19: f64 = 1.0 / 524_288.0;
const P2_24: f64 = 1.0 / 16_777_216.0;
const P2_29: f64 = 1.0 / 536_870_912.0;
const P2_31: f64 = 1.0 / 2_147_483_648.0;
const P2_32: f64 = 1.0 / 4_294_967_296.0;
const P2_33: f64 = 1.0 / 8_589_934_592.0;
const P2_34: f64 = 1.0 / 17_179_869_184.0;
const P2_43: f64 = 1.0 / 8_796_093_022_208.0;
const P2_46: f64 = 1.0 / 70_368_744_177_664.0;
const P2_55: f64 = 1.0 / 36_028_797_018_963_968.0;
const P2_59: f64 = 1.0 / 576_460_752_303_423_488.0;

#[derive(Debug, Clone)]
pub struct MsmDecodedData {
    pub msg_type: u16,
    pub constellation: Constellation,
    pub is_msm7: bool,
    pub station_id: u16,
    pub epoch_ms: u32,
    pub sync_flag: bool,
    pub iods: u8,
    pub records: Vec<ObsRecord>,
}

#[derive(Debug, Clone)]
pub enum Rtcm3Message {
    /// Message 1005 / 1006 : Position ARP de la station de base en ECEF (mètres)
    StationArp {
        station_id: u16,
        ecef: [f64; 3],
        antenna_height_m: f64,
    },
    /// Message 1019 (GPS) ou 1046 (Galileo I/NAV) : Éphéméride radiodiffusée Kepler
    Ephemeris(BroadcastEphemeris),
    /// Message MSM4 / MSM7 complètement décodé bit-à-bit en observations brutes (code, phase, Doppler, SNR, LLI)
    MsmObservations(MsmDecodedData),
    Other {
        msg_type: u16,
        payload_len: usize,
    },
}

/// Calcule le CRC-24Q standard RTCM 3.x (polynôme `0x1864CFB`).
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc ^= (b as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if (crc & 0x100_0000) != 0 {
                crc ^= 0x186_4CFB;
            }
        }
    }
    crc & 0xFF_FFFF
}

/// Lecteur de bits MSB-first pour les trames binaires RTCM 3.x.
struct BitReader<'a> {
    buf: &'a [u8],
    bit_pos: usize,
}

impl<'a> BitReader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, bit_pos: 0 }
    }

    fn remaining_bits(&self) -> usize {
        (self.buf.len() * 8).saturating_sub(self.bit_pos)
    }

    fn read_u64(&mut self, len: usize) -> Option<u64> {
        if len == 0 {
            return Some(0);
        }
        if len > 64 || self.remaining_bits() < len {
            return None;
        }
        let mut val = 0u64;
        for _ in 0..len {
            let byte_idx = self.bit_pos / 8;
            let bit_idx = 7 - (self.bit_pos % 8);
            val = (val << 1) | (((self.buf[byte_idx] >> bit_idx) & 1) as u64);
            self.bit_pos += 1;
        }
        Some(val)
    }

    fn read_i64(&mut self, len: usize) -> Option<i64> {
        let u = self.read_u64(len)?;
        if len == 0 || len >= 64 {
            return Some(u as i64);
        }
        let sign_bit = 1u64 << (len - 1);
        if (u & sign_bit) != 0 {
            Some((u | (!0u64 << len)) as i64)
        } else {
            Some(u as i64)
        }
    }
}

/// Associe un numéro de signal RTCM MSM (1..=32) à `(bande 0=L1/1=L2/2=L5, longueur d'onde m)`.
fn msm_signal_band_and_wavelength(constellation: Constellation, sig_id: u8) -> Option<(usize, f64)> {
    match constellation {
        Constellation::Gps => match sig_id {
            2..=4 => Some((0, LAMBDA1)),            // L1 C/A, P, W
            8..=10 | 15..=17 => Some((1, LAMBDA2)), // L2 P, W, C(M/L)
            22..=24 => Some((2, LAMBDA5)),          // L5 I/Q/I+Q
            30..=32 => Some((0, LAMBDA1)),          // L1C
            _ => None,
        },
        Constellation::Galileo => match sig_id {
            2..=6 => Some((0, LAMBDA1)),   // E1 C/A/B/B+C
            14..=16 => Some((1, LAMBDA2)), // E6 (slot secondaire)
            18..=20 => Some((1, CLIGHT / 1.207_14e9)), // E5b
            22..=24 => Some((2, LAMBDA5)), // E5a
            _ => None,
        },
        Constellation::BeiDou => match sig_id {
            2..=4 => Some((0, CLIGHT / 1.561_098e9)), // B1I
            30..=32 => Some((0, LAMBDA1)),            // B1C
            14..=16 => Some((1, CLIGHT / 1.207_14e9)), // B2I / B2b
            22..=24 => Some((2, LAMBDA5)),            // B2a
            _ => None,
        },
        Constellation::Glonass => match sig_id {
            2..=3 => Some((0, CLIGHT / 1.602e9)), // G1 nominal
            8..=9 => Some((1, CLIGHT / 1.246e9)), // G2 nominal
            _ => None,
        },
        Constellation::Qzss => match sig_id {
            2 | 30..=32 => Some((0, LAMBDA1)),
            15..=17 => Some((1, LAMBDA2)),
            22..=24 => Some((2, LAMBDA5)),
            _ => None,
        },
    }
}

/// Décodeur incrémental de flux binaire RTCM 3.3 avec assemblage d'époques multi-constellations
/// et collecte automatique des éphémérides radiodiffusées (`1019` GPS, `1046` Galileo).
#[derive(Default)]
pub struct Rtcm3StreamDecoder {
    buf: Vec<u8>,
    pub valid_frames: u64,
    pub crc_errors: u64,
    pub msm_frames_decoded: u64,
    pub eph_frames_decoded: u64,
    pub last_base_ecef: Option<[f64; 3]>,
    pub last_station_id: Option<u16>,
    /// Éphémérides accumulées depuis les trames RTCM 1019/1046 pour alimenter `RtkEngine`
    pub nav_data: NavData,
    /// Accumulateur courant des observations multi-messages (jusqu'à `sync_flag == false`)
    pending_epoch_records: Vec<ObsRecord>,
    /// Dernière époque complète assemblée prête pour le moteur RTK
    pub latest_base_epoch: Option<ObsEpoch>,
}

impl Rtcm3StreamDecoder {
    pub fn push_bytes(&mut self, data: &[u8]) -> Vec<Rtcm3Message> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();

        loop {
            let Some(sync_pos) = self.buf.iter().position(|&b| b == 0xD3) else {
                self.buf.clear();
                break;
            };
            if sync_pos > 0 {
                self.buf.drain(0..sync_pos);
            }
            if self.buf.len() < 6 {
                break;
            }
            let payload_len = (((self.buf[1] & 0x03) as usize) << 8) | (self.buf[2] as usize);
            let frame_len = 3 + payload_len + 3;
            if self.buf.len() < frame_len {
                break;
            }
            let calc_crc = crc24q(&self.buf[0..3 + payload_len]);
            let frame_crc = ((self.buf[3 + payload_len] as u32) << 16)
                | ((self.buf[4 + payload_len] as u32) << 8)
                | (self.buf[5 + payload_len] as u32);

            if calc_crc == frame_crc {
                self.valid_frames += 1;
                let payload = self.buf[3..3 + payload_len].to_vec();
                self.buf.drain(0..frame_len);

                if let Some(msg) = parse_rtcm3_payload(&payload) {
                    match &msg {
                        Rtcm3Message::StationArp {
                            station_id, ecef, ..
                        } => {
                            self.last_station_id = Some(*station_id);
                            self.last_base_ecef = Some(*ecef);
                        }
                        Rtcm3Message::Ephemeris(eph) => {
                            self.eph_frames_decoded += 1;
                            self.nav_data.add_ephemeris(eph.clone());
                        }
                        Rtcm3Message::MsmObservations(msm) => {
                            self.msm_frames_decoded += 1;
                            self.last_station_id = Some(msm.station_id);
                            self.pending_epoch_records.extend(msm.records.clone());
                            // Quand sync_flag == false, c'est le dernier message MSM de l'époque
                            if !msm.sync_flag {
                                let (week, _) = GTime::from_unix_f64(
                                    chrono::Utc::now().timestamp() as f64,
                                )
                                .to_gps_week_tow();
                                let epoch_time = GTime::from_gps_week_tow(
                                    week,
                                    (msm.epoch_ms as f64) * 0.001,
                                );
                                let records = std::mem::take(&mut self.pending_epoch_records);
                                self.latest_base_epoch = Some(ObsEpoch {
                                    time: epoch_time,
                                    records,
                                });
                            }
                        }
                        Rtcm3Message::Other { .. } => {}
                    }
                    out.push(msg);
                }
            } else {
                self.crc_errors += 1;
                self.buf.drain(0..1);
            }
        }

        out
    }
}

fn parse_rtcm3_payload(payload: &[u8]) -> Option<Rtcm3Message> {
    let mut br = BitReader::new(payload);
    let msg_type = br.read_u64(12)? as u16;

    match msg_type {
        1005 | 1006 => {
            let station_id = br.read_u64(12)? as u16;
            let _itrf_year = br.read_u64(6)?;
            let _flags = br.read_u64(4)?;
            let x_raw = br.read_i64(38)?;
            let _osc = br.read_u64(2)?;
            let y_raw = br.read_i64(38)?;
            let _qc = br.read_u64(2)?;
            let z_raw = br.read_i64(38)?;
            let ant_h = if msg_type == 1006 {
                br.read_u64(16)? as f64 * 0.0001
            } else {
                0.0
            };
            Some(Rtcm3Message::StationArp {
                station_id,
                ecef: [
                    x_raw as f64 * 0.0001,
                    y_raw as f64 * 0.0001,
                    z_raw as f64 * 0.0001,
                ],
                antenna_height_m: ant_h,
            })
        }
        1019 => decode_rtcm_1019(&mut br).map(Rtcm3Message::Ephemeris),
        1046 => decode_rtcm_1046(&mut br).map(Rtcm3Message::Ephemeris),
        1074 | 1077 | 1084 | 1087 | 1094 | 1097 | 1114 | 1117 | 1124 | 1127 => {
            let (constellation, is_msm7) = match msg_type {
                1074 => (Constellation::Gps, false),
                1077 => (Constellation::Gps, true),
                1084 => (Constellation::Glonass, false),
                1087 => (Constellation::Glonass, true),
                1094 => (Constellation::Galileo, false),
                1097 => (Constellation::Galileo, true),
                1114 => (Constellation::Qzss, false),
                1117 => (Constellation::Qzss, true),
                1124 => (Constellation::BeiDou, false),
                1127 => (Constellation::BeiDou, true),
                _ => return None,
            };
            decode_msm_body(&mut br, msg_type, constellation, is_msm7)
                .map(Rtcm3Message::MsmObservations)
        }
        _ => Some(Rtcm3Message::Other {
            msg_type,
            payload_len: payload.len(),
        }),
    }
}

/// Décodeur RTCM 3.3 Message 1019 : Éphéméride GPS radiodiffusée (488 bits).
fn decode_rtcm_1019(br: &mut BitReader<'_>) -> Option<BroadcastEphemeris> {
    let prn = br.read_u64(6)? as u8;
    let week_10 = br.read_u64(10)? as i32;
    let _sv_acc = br.read_u64(4)?;
    let _code_l2 = br.read_u64(2)?;
    let idot = br.read_i64(14)? as f64 * P2_43 * PI;
    let iode = br.read_u64(8)? as i32;
    let toc_s = br.read_u64(16)? as f64 * 16.0;
    let f2 = br.read_i64(8)? as f64 * P2_55;
    let f1 = br.read_i64(16)? as f64 * P2_43;
    let f0 = br.read_i64(22)? as f64 * P2_31;
    let _iodc = br.read_u64(10)?;
    let crs = br.read_i64(16)? as f64 * P2_5;
    let deln = br.read_i64(16)? as f64 * P2_43 * PI;
    let m0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let cuc = br.read_i64(16)? as f64 * P2_29;
    let e = br.read_u64(32)? as f64 * P2_33;
    let cus = br.read_i64(16)? as f64 * P2_29;
    let sqrt_a = br.read_u64(32)? as f64 * P2_19;
    let toe_s = br.read_u64(16)? as f64 * 16.0;
    let cic = br.read_i64(16)? as f64 * P2_29;
    let omg0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let cis = br.read_i64(16)? as f64 * P2_29;
    let i0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let crc = br.read_i64(16)? as f64 * P2_5;
    let omg = br.read_i64(32)? as f64 * P2_31 * PI;
    let omgd = br.read_i64(24)? as f64 * P2_43 * PI;
    let tgd = br.read_i64(8)? as f64 * P2_31;
    let sv_health = br.read_u64(6)? as u8;
    let _l2_p = br.read_u64(1)?;
    let _fit = br.read_u64(1)?;

    // Résolution du rollover 1024 semaines GPS (cycle courant >= 2048)
    let week = week_10 + 2048;
    let toe = GTime::from_gps_week_tow(week, toe_s);
    let toc = GTime::from_gps_week_tow(week, toc_s);

    Some(BroadcastEphemeris {
        sat: SatId::new(Constellation::Gps, prn),
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
    })
}

/// Décodeur RTCM 3.3 Message 1046 : Éphéméride Galileo I/NAV radiodiffusée (504 bits).
fn decode_rtcm_1046(br: &mut BitReader<'_>) -> Option<BroadcastEphemeris> {
    let prn = br.read_u64(6)? as u8;
    let gst_week = br.read_u64(12)? as i32;
    let iode = br.read_u64(10)? as i32;
    let _sisa = br.read_u64(8)?;
    let idot = br.read_i64(14)? as f64 * P2_43 * PI;
    let toc_s = br.read_u64(14)? as f64 * 60.0;
    let f2 = br.read_i64(6)? as f64 * P2_59;
    let f1 = br.read_i64(21)? as f64 * P2_46;
    let f0 = br.read_i64(31)? as f64 * P2_34;
    let crs = br.read_i64(16)? as f64 * P2_5;
    let deln = br.read_i64(16)? as f64 * P2_43 * PI;
    let m0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let cuc = br.read_i64(16)? as f64 * P2_29;
    let e = br.read_u64(32)? as f64 * P2_33;
    let cus = br.read_i64(16)? as f64 * P2_29;
    let sqrt_a = br.read_u64(32)? as f64 * P2_19;
    let toe_s = br.read_u64(14)? as f64 * 60.0;
    let cic = br.read_i64(16)? as f64 * P2_29;
    let omg0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let cis = br.read_i64(16)? as f64 * P2_29;
    let i0 = br.read_i64(32)? as f64 * P2_31 * PI;
    let crc = br.read_i64(16)? as f64 * P2_5;
    let omg = br.read_i64(32)? as f64 * P2_31 * PI;
    let omgd = br.read_i64(24)? as f64 * P2_43 * PI;
    let tgd = br.read_i64(10)? as f64 * P2_32;
    let _bgd_e5b = br.read_i64(10)?;
    let e5b_hs = br.read_u64(2)? as u8;
    let _e5b_dvs = br.read_u64(1)?;
    let e1b_hs = br.read_u64(2)? as u8;
    let _e1b_dvs = br.read_u64(1)?;
    let _reserved = br.read_u64(2)?;

    let week = gst_week + 1024;
    let toe = GTime::from_gps_week_tow(week, toe_s);
    let toc = GTime::from_gps_week_tow(week, toc_s);

    Some(BroadcastEphemeris {
        sat: SatId::new(Constellation::Galileo, prn),
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
        sv_health: e1b_hs | (e5b_hs << 2),
    })
}

/// Décodeur complet bit-à-bit RTCM 3.3 MSM4 et MSM7 (équivalent `decode_msm4` / `decode_msm7` RTKLIB).
fn decode_msm_body(
    br: &mut BitReader<'_>,
    msg_type: u16,
    constellation: Constellation,
    is_msm7: bool,
) -> Option<MsmDecodedData> {
    // ── 1. En-tête MSM (169 bits au total incluant msg_type) ──────────────────
    let station_id = br.read_u64(12)? as u16;
    let epoch_ms = br.read_u64(30)? as u32;
    let sync_flag = br.read_u64(1)? == 1;
    let iods = br.read_u64(3)? as u8;
    let _reserved = br.read_u64(7)?;
    let _clk_steer = br.read_u64(2)?;
    let _ext_clk = br.read_u64(2)?;
    let _smooth = br.read_u64(1)?;
    let _smooth_int = br.read_u64(3)?;

    let sat_mask = br.read_u64(64)?;
    let sig_mask = br.read_u64(32)? as u32;

    let mut sat_prns = Vec::new();
    for i in 0..64u8 {
        if ((sat_mask >> (63 - i)) & 1) != 0 {
            sat_prns.push(i + 1);
        }
    }
    let mut sig_ids = Vec::new();
    for j in 0..32u8 {
        if ((sig_mask >> (31 - j)) & 1) != 0 {
            sig_ids.push(j + 1);
        }
    }

    let nsat = sat_prns.len();
    let nsig = sig_ids.len();
    if nsat == 0 || nsig == 0 || nsat * nsig > 64 {
        return Some(MsmDecodedData {
            msg_type,
            constellation,
            is_msm7,
            station_id,
            epoch_ms,
            sync_flag,
            iods,
            records: Vec::new(),
        });
    }

    // Masque cellules (nsat * nsig bits)
    let mut cells: Vec<(usize, usize)> = Vec::new();
    for s_idx in 0..nsat {
        for sig_idx in 0..nsig {
            if br.read_u64(1)? == 1 {
                cells.push((s_idx, sig_idx));
            }
        }
    }
    let ncell = cells.len();

    // ── 2. Données Satellites ─────────────────────────────────────────────────
    let mut rough_ms = Vec::with_capacity(nsat);
    for _ in 0..nsat {
        rough_ms.push(br.read_u64(8)? as u8);
    }
    let mut _ext_sat_info = Vec::with_capacity(nsat);
    if is_msm7 {
        for _ in 0..nsat {
            _ext_sat_info.push(br.read_u64(4)? as u8);
        }
    }
    let mut rough_mod = Vec::with_capacity(nsat);
    for _ in 0..nsat {
        rough_mod.push(br.read_u64(10)? as u16);
    }
    let mut rough_prr = vec![0.0f64; nsat];
    if is_msm7 {
        for val in &mut rough_prr {
            let raw = br.read_i64(14)?;
            *val = if raw == -8192 { 0.0 } else { raw as f64 };
        }
    }

    // ── 3. Données Signaux (ncell entrées) ────────────────────────────────────
    let mut fine_pr_m = vec![f64::NAN; ncell];
    let mut fine_cp_m = vec![f64::NAN; ncell];
    let mut lock_ind = vec![0u16; ncell];
    let mut half_cyc = vec![false; ncell];
    let mut cnr_dbhz = vec![0.0f32; ncell];
    let mut fine_prr_ms = vec![0.0f64; ncell];

    if is_msm7 {
        for v in &mut fine_pr_m {
            let raw = br.read_i64(20)?;
            if raw != -524_288 {
                *v = (raw as f64) * P2_29 * RANGE_MS;
            }
        }
        for v in &mut fine_cp_m {
            let raw = br.read_i64(24)?;
            if raw != -8_388_608 {
                *v = (raw as f64) * P2_31 * RANGE_MS;
            }
        }
        for l in &mut lock_ind {
            *l = br.read_u64(10)? as u16;
        }
        for h in &mut half_cyc {
            *h = br.read_u64(1)? == 1;
        }
        for c in &mut cnr_dbhz {
            *c = (br.read_u64(10)? as f32) * 0.0625; // 2^-4 dB-Hz
        }
        for d in &mut fine_prr_ms {
            let raw = br.read_i64(15)?;
            if raw != -16_384 {
                *d = (raw as f64) * 0.0001; // m/s
            }
        }
    } else {
        // MSM4
        for v in &mut fine_pr_m {
            let raw = br.read_i64(15)?;
            if raw != -16_384 {
                *v = (raw as f64) * P2_24 * RANGE_MS;
            }
        }
        for v in &mut fine_cp_m {
            let raw = br.read_i64(22)?;
            if raw != -2_097_152 {
                *v = (raw as f64) * P2_29 * RANGE_MS;
            }
        }
        for l in &mut lock_ind {
            *l = br.read_u64(4)? as u16;
        }
        for h in &mut half_cyc {
            *h = br.read_u64(1)? == 1;
        }
        for c in &mut cnr_dbhz {
            *c = br.read_u64(6)? as f32; // 1.0 dB-Hz
        }
    }

    // ── 4. Reconstruction des ObsRecord par satellite ─────────────────────────
    let mut by_sat: HashMap<SatId, ObsRecord> = HashMap::new();
    for (cell_i, &(s_idx, sig_idx)) in cells.iter().enumerate() {
        let rms_int = rough_ms[s_idx];
        if rms_int == 255 {
            continue; // Rough range invalide
        }
        let sat = SatId::new(constellation, sat_prns[s_idx]);
        let sig_id = sig_ids[sig_idx];
        let Some((band, wavelength)) = msm_signal_band_and_wavelength(constellation, sig_id) else {
            continue;
        };

        let rough_range_m = ((rms_int as f64) + (rough_mod[s_idx] as f64) * P2_10) * RANGE_MS;
        let rec = by_sat.entry(sat).or_insert_with(|| ObsRecord::new(sat));

        if fine_pr_m[cell_i].is_finite() {
            rec.pseudorange[band] = rough_range_m + fine_pr_m[cell_i];
        }
        if fine_cp_m[cell_i].is_finite() {
            rec.carrier_phase[band] = (rough_range_m + fine_cp_m[cell_i]) / wavelength;
        }
        if is_msm7 {
            let total_prr = rough_prr[s_idx] + fine_prr_ms[cell_i];
            rec.doppler[band] = (-total_prr / wavelength) as f32;
        }
        rec.snr[band] = cnr_dbhz[cell_i];
        let mut lli = 0u8;
        if lock_ind[cell_i] == 0 {
            lli |= 0x01; // Loss of lock / cycle slip
        }
        if half_cyc[cell_i] {
            lli |= 0x02; // Half-cycle ambiguity
        }
        rec.lli[band] = lli;
    }

    let mut records: Vec<ObsRecord> = by_sat.into_values().collect();
    records.sort_by_key(|r| r.sat);

    Some(MsmDecodedData {
        msg_type,
        constellation,
        is_msm7,
        station_id,
        epoch_ms,
        sync_flag,
        iods,
        records,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encodeur de bits MSB-first pour générer des trames RTCM 3.3 MSM7 conformes dans les tests.
    struct BitWriter {
        bytes: Vec<u8>,
        cur_byte: u8,
        bit_count: u8,
    }

    impl BitWriter {
        fn new() -> Self {
            Self {
                bytes: Vec::new(),
                cur_byte: 0,
                bit_count: 0,
            }
        }

        fn write_u64(&mut self, val: u64, len: usize) {
            for i in (0..len).rev() {
                let bit = ((val >> i) & 1) as u8;
                self.cur_byte = (self.cur_byte << 1) | bit;
                self.bit_count += 1;
                if self.bit_count == 8 {
                    self.bytes.push(self.cur_byte);
                    self.cur_byte = 0;
                    self.bit_count = 0;
                }
            }
        }

        fn write_i64(&mut self, val: i64, len: usize) {
            let mask = if len >= 64 { u64::MAX } else { (1u64 << len) - 1 };
            self.write_u64((val as u64) & mask, len);
        }

        fn finish(mut self) -> Vec<u8> {
            if self.bit_count > 0 {
                self.cur_byte <<= 8 - self.bit_count;
                self.bytes.push(self.cur_byte);
            }
            self.bytes
        }
    }

    fn wrap_rtcm3_frame(payload: &[u8]) -> Vec<u8> {
        let len = payload.len();
        let mut frame = vec![0xD3, ((len >> 8) & 0x03) as u8, (len & 0xFF) as u8];
        frame.extend_from_slice(payload);
        let crc = crc24q(&frame);
        frame.push(((crc >> 16) & 0xFF) as u8);
        frame.push(((crc >> 8) & 0xFF) as u8);
        frame.push((crc & 0xFF) as u8);
        frame
    }

    #[test]
    fn test_rtcm3_msm7_full_bit_decode_roundtrip() {
        // Génère un message RTCM 1077 (GPS MSM7) complet pour 2 satellites (G05, G12) sur L1 (sig 2) + L2 (sig 15)
        let pr_g05_l1 = 21_450_123.456_f64;
        let cp_g05_l1_m = 21_450_123.512_f64;
        let pr_g12_l1 = 22_890_654.321_f64;
        let cp_g12_l1_m = 22_890_654.298_f64;

        let mut bw = BitWriter::new();
        bw.write_u64(1077, 12); // GPS MSM7
        bw.write_u64(42, 12); // Station ID 42
        bw.write_u64(123_456_000, 30); // Epoch TOW ms
        bw.write_u64(0, 1); // sync_flag = false (fin d'époque)
        bw.write_u64(1, 3); // IODS
        bw.write_u64(0, 7); // reserved
        bw.write_u64(0, 2); // clock steering
        bw.write_u64(0, 2); // ext clock
        bw.write_u64(0, 1); // smooth
        bw.write_u64(0, 3); // smooth int

        // Sat mask: PRN 5 (bit 63-4) & PRN 12 (bit 63-11)
        let sat_mask = (1u64 << (63 - 4)) | (1u64 << (63 - 11));
        bw.write_u64(sat_mask, 64);
        // Signal mask: Sig 2 (L1C/A -> bit 31-1) & Sig 15 (L2C -> bit 31-14)
        let sig_mask = (1u64 << (31 - 1)) | (1u64 << (31 - 14));
        bw.write_u64(sig_mask, 32);

        // Cell mask (2 sats * 2 sigs = 4 bits): G05(L1,L2), G12(L1 uniquement) -> 1, 1, 1, 0
        bw.write_u64(0b1110, 4);

        // Satellite rough ranges
        let r0_ms_f = pr_g05_l1 / RANGE_MS;
        let r0_int = r0_ms_f.floor() as u64;
        let r0_mod = ((r0_ms_f - r0_int as f64) * 1024.0).floor() as u64;
        let r0_base_m = (r0_int as f64 + r0_mod as f64 * P2_10) * RANGE_MS;

        let r1_ms_f = pr_g12_l1 / RANGE_MS;
        let r1_int = r1_ms_f.floor() as u64;
        let r1_mod = ((r1_ms_f - r1_int as f64) * 1024.0).floor() as u64;
        let r1_base_m = (r1_int as f64 + r1_mod as f64 * P2_10) * RANGE_MS;

        bw.write_u64(r0_int, 8);
        bw.write_u64(r1_int, 8);
        // Extended sat info (4 bits * 2)
        bw.write_u64(0, 4);
        bw.write_u64(0, 4);
        // Rough mod 1 ms (10 bits * 2)
        bw.write_u64(r0_mod, 10);
        bw.write_u64(r1_mod, 10);
        // Rough phase range rate (14 bits signed * 2) : +250 m/s et -180 m/s
        bw.write_i64(250, 14);
        bw.write_i64(-180, 14);

        // Fine pseudoranges (20 bits signed * 3 cells)
        let fp0 = ((pr_g05_l1 - r0_base_m) / (P2_29 * RANGE_MS)).round() as i64;
        let fp1 = ((pr_g05_l1 + 1.25 - r0_base_m) / (P2_29 * RANGE_MS)).round() as i64;
        let fp2 = ((pr_g12_l1 - r1_base_m) / (P2_29 * RANGE_MS)).round() as i64;
        bw.write_i64(fp0, 20);
        bw.write_i64(fp1, 20);
        bw.write_i64(fp2, 20);

        // Fine phase ranges (24 bits signed * 3 cells)
        let fc0 = ((cp_g05_l1_m - r0_base_m) / (P2_31 * RANGE_MS)).round() as i64;
        let fc1 = ((cp_g05_l1_m + 1.25 - r0_base_m) / (P2_31 * RANGE_MS)).round() as i64;
        let fc2 = ((cp_g12_l1_m - r1_base_m) / (P2_31 * RANGE_MS)).round() as i64;
        bw.write_i64(fc0, 24);
        bw.write_i64(fc1, 24);
        bw.write_i64(fc2, 24);

        // Lock time indicators (10 bits * 3)
        bw.write_u64(120, 10);
        bw.write_u64(120, 10);
        bw.write_u64(0, 10); // Cycle slip sur G12!

        // Half-cycle ambiguity (1 bit * 3)
        bw.write_u64(0, 1);
        bw.write_u64(0, 1);
        bw.write_u64(1, 1);

        // CNR (10 bits * 3, pas 0.0625 dB-Hz) -> 46.5, 43.25, 39.0 dB-Hz
        bw.write_u64((46.5 * 16.0) as u64, 10);
        bw.write_u64((43.25 * 16.0) as u64, 10);
        bw.write_u64((39.0 * 16.0) as u64, 10);

        // Fine phase range rate (15 bits signed * 3, pas 0.0001 m/s) -> +0.25, +0.25, -0.50 m/s
        bw.write_i64(2500, 15);
        bw.write_i64(2500, 15);
        bw.write_i64(-5000, 15);

        let payload = bw.finish();
        let frame = wrap_rtcm3_frame(&payload);

        let mut dec = Rtcm3StreamDecoder::default();
        let msgs = dec.push_bytes(&frame);
        assert_eq!(dec.valid_frames, 1);
        assert_eq!(dec.msm_frames_decoded, 1);
        assert_eq!(msgs.len(), 1);
        assert!(dec.latest_base_epoch.is_some());

        if let Rtcm3Message::MsmObservations(msm) = &msgs[0] {
            assert_eq!(msm.msg_type, 1077);
            assert_eq!(msm.station_id, 42);
            assert!(msm.is_msm7);
            assert_eq!(msm.records.len(), 2);

            let g05 = &msm.records[0];
            assert_eq!(g05.sat, SatId::new(Constellation::Gps, 5));
            assert!((g05.pseudorange[0] - pr_g05_l1).abs() < 0.002);
            assert!((g05.carrier_phase[0] * LAMBDA1 - cp_g05_l1_m).abs() < 0.001);
            assert!((g05.pseudorange[1] - (pr_g05_l1 + 1.25)).abs() < 0.002);
            assert!((g05.snr[0] - 46.5).abs() < 0.01);
            assert!((g05.snr[1] - 43.25).abs() < 0.01);
            assert_eq!(g05.lli[0], 0);
            assert!((g05.doppler[0] as f64 - (-250.25 / LAMBDA1)).abs() < 0.05);

            let g12 = &msm.records[1];
            assert_eq!(g12.sat, SatId::new(Constellation::Gps, 12));
            assert!((g12.pseudorange[0] - pr_g12_l1).abs() < 0.002);
            assert_eq!(g12.lli[0], 0x03); // lock==0 (bit 0) + half_cycle (bit 1)
            assert!((g12.snr[0] - 39.0).abs() < 0.01);
        } else {
            panic!("MsmObservations attendu");
        }
    }

    #[test]
    fn test_rtcm3_1019_gps_ephemeris_decode() {
        let mut bw = BitWriter::new();
        bw.write_u64(1019, 12); // GPS Broadcast Ephemeris
        bw.write_u64(17, 6); // PRN G17
        bw.write_u64(252, 10); // Week 2048 + 252 = 2300
        bw.write_u64(0, 4); // sv_acc
        bw.write_u64(1, 2); // code_l2
        bw.write_i64(12, 14); // idot
        bw.write_u64(45, 8); // iode
        bw.write_u64(6250, 16); // toc = 100000 s
        bw.write_i64(0, 8); // f2
        bw.write_i64(100, 16); // f1
        bw.write_i64(42000, 22); // f0
        bw.write_u64(45, 10); // iodc
        bw.write_i64(-320, 16); // crs
        bw.write_i64(12000, 16); // deln
        bw.write_i64(500_000_000, 32); // m0
        bw.write_i64(100, 16); // cuc
        bw.write_u64(40_000_000, 32); // e (~0.00465)
        bw.write_i64(-100, 16); // cus
        bw.write_u64((5153.6_f64 / P2_19).round() as u64, 32); // sqrt_a (~5153.6)
        bw.write_u64(6250, 16); // toe = 100000 s
        bw.write_i64(10, 16); // cic
        bw.write_i64(300_000_000, 32); // omg0
        bw.write_i64(-10, 16); // cis
        bw.write_i64(650_000_000, 32); // i0
        bw.write_i64(6400, 16); // crc
        bw.write_i64(-200_000_000, 32); // omg
        bw.write_i64(-22000, 24); // omgd
        bw.write_i64(-5, 8); // tgd
        bw.write_u64(0, 6); // sv_health
        bw.write_u64(0, 1); // l2_p
        bw.write_u64(0, 1); // fit

        let frame = wrap_rtcm3_frame(&bw.finish());
        let mut dec = Rtcm3StreamDecoder::default();
        let msgs = dec.push_bytes(&frame);
        assert_eq!(dec.eph_frames_decoded, 1);
        assert_eq!(msgs.len(), 1);
        if let Rtcm3Message::Ephemeris(eph) = &msgs[0] {
            assert_eq!(eph.sat, SatId::new(Constellation::Gps, 17));
            assert_eq!(eph.week, 2300);
            assert_eq!(eph.iode, 45);
            assert!((eph.sqrt_a - 5153.6).abs() < 1e-4);
            assert!((eph.toe_s - 100_000.0).abs() < 1e-6);
            assert!(dec.nav_data.select_eph(eph.sat, eph.toe).is_some());
        } else {
            panic!("Ephemeris 1019 attendu");
        }
    }
}
