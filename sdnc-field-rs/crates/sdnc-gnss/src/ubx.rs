//! Décodeur & Encodeur binaire UBX u-blox (`NAV-PVT`, `RXM-RAWX` L1/L2/L5, `CFG-VALSET`, `CFG-MSG`, `CFG-RATE`).
//!
//! Supporte :
//! - **u-blox ZED-F9P** (SparkFun RTK Facet / Express / Torch, ArduSimple simpleRTK2B, Emlid Reach M2/RS2+) :
//!   décodage bi-fréquence L1/L2/L5 via `sig_id` + génération de trames `UBX-CFG-VALSET`
//! - **u-blox M8T** (Emlid Reach M+ / RS+ L1) : décodage `RXM-RAWX` L1 + configuration `UBX-CFG-MSG` / `UBX-CFG-RATE`
//! - **Post-traitement PPK direct sur fichiers `.ubx`** (`read_ubx_obs_file`)

use crate::types::{Constellation, FixQuality, GTime, ObsEpoch, ObsRecord, SatId};
use anyhow::{Context, Result};
use sdnc_geo::{geodetic_to_ecef, WGS84};
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug, Clone)]
pub enum UbxMessage {
    NavPvt {
        lat_deg: f64,
        lon_deg: f64,
        alt_ellipsoid_m: f64,
        alt_msl_m: f64,
        h_acc_m: f32,
        v_acc_m: f32,
        num_sv: u8,
        quality: FixQuality,
    },
    RxmRawx(ObsEpoch),
}

#[derive(Default)]
pub struct UbxStreamDecoder {
    buf: Vec<u8>,
    pub valid_frames: u64,
}

impl UbxStreamDecoder {
    pub fn push_bytes(&mut self, data: &[u8]) -> Vec<UbxMessage> {
        self.buf.extend_from_slice(data);
        let mut out = Vec::new();

        loop {
            let sync = self.buf.windows(2).position(|w| w == [0xB5, 0x62]);
            let Some(pos) = sync else {
                self.buf.clear();
                break;
            };
            if pos > 0 {
                self.buf.drain(0..pos);
            }
            if self.buf.len() < 8 {
                break;
            }
            let cls = self.buf[2];
            let id = self.buf[3];
            let len = u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;
            let total = 6 + len + 2;
            if self.buf.len() < total {
                break;
            }

            let (ck_a, ck_b) = ubx_checksum(&self.buf[2..6 + len]);
            if ck_a == self.buf[6 + len] && ck_b == self.buf[7 + len] {
                self.valid_frames += 1;
                let payload = &self.buf[6..6 + len];
                if let Some(m) = parse_ubx_payload(cls, id, payload) {
                    out.push(m);
                }
                self.buf.drain(0..total);
            } else {
                self.buf.drain(0..2);
            }
        }
        out
    }
}

/// Checksum Fletcher-8 standard du protocole u-blox UBX.
pub fn ubx_checksum(body: &[u8]) -> (u8, u8) {
    let (mut ck_a, mut ck_b) = (0u8, 0u8);
    for &b in body {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Encapsule une charge utile dans une trame binaire UBX complète (`0xB5 0x62 cls id len payload ck_a ck_b`).
pub fn encode_ubx_frame(cls: u8, id: u8, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(8 + payload.len());
    frame.push(0xB5);
    frame.push(0x62);
    frame.push(cls);
    frame.push(id);
    let len = payload.len() as u16;
    frame.extend_from_slice(&len.to_le_bytes());
    frame.extend_from_slice(payload);
    let (ck_a, ck_b) = ubx_checksum(&frame[2..]);
    frame.push(ck_a);
    frame.push(ck_b);
    frame
}

/// Construit une commande `UBX-CFG-VALSET` (RAM + BBR) pour u-blox ZED-F9P (SparkFun, ArduSimple, Reach M2).
pub fn encode_ubx_cfg_valset(items: &[(u32, &[u8])]) -> Vec<u8> {
    // version=0, layers=0x03 (RAM + BBR), reserved=[0, 0]
    let mut payload = vec![0x00, 0x03, 0x00, 0x00];
    for &(key_id, val_bytes) in items {
        payload.extend_from_slice(&key_id.to_le_bytes());
        payload.extend_from_slice(val_bytes);
    }
    encode_ubx_frame(0x06, 0x8A, &payload)
}

/// Séquence complète d'initialisation RTK L1/L2 + PPK RAWX pour puce **u-blox ZED-F9P**
/// (SparkFun RTK Facet/Express/Torch, ArduSimple simpleRTK2B, Emlid Reach M2).
pub fn build_ubx_f9p_rtk_ppk_init(rate_hz: u8, enable_raw_ppk: bool) -> Vec<Vec<u8>> {
    let meas_ms: u16 = (1000u16 / (rate_hz.clamp(1, 10) as u16)).max(100);
    let meas_bytes = meas_ms.to_le_bytes();
    let raw_rate: [u8; 1] = [if enable_raw_ppk { 1 } else { 0 }];

    // 1. Fréquence de mesure + activation bandes L1+L2 (GPS L2C, Galileo E5b, GLONASS L2, BeiDou B2)
    let cfg_signals = encode_ubx_cfg_valset(&[
        (0x3021_0001, &meas_bytes), // CFG-RATE-MEAS (ms)
        (0x3021_0002, &1u16.to_le_bytes()), // CFG-RATE-NAV = 1
        (0x1031_0001, &[1]),        // CFG-SIGNAL-GPS_L1CA_ENA
        (0x1031_0003, &[1]),        // CFG-SIGNAL-GPS_L2C_ENA
        (0x1031_0007, &[1]),        // CFG-SIGNAL-GAL_E1_ENA
        (0x1031_000A, &[1]),        // CFG-SIGNAL-GAL_E5B_ENA
        (0x1031_0018, &[1]),        // CFG-SIGNAL-GLO_L1_ENA
        (0x1031_001A, &[1]),        // CFG-SIGNAL-GLO_L2_ENA
        (0x1031_000D, &[1]),        // CFG-SIGNAL-BDS_B1_ENA
        (0x1031_000E, &[1]),        // CFG-SIGNAL-BDS_B2_ENA
    ]);

    // 2. Sorties NAV-PVT, NMEA GGA/GST/GSV, RAWX/SFRBX (PPK) & entrée RTCM3 sur USB + UART1 + UART2
    let cfg_io = encode_ubx_cfg_valset(&[
        (0x1077_0004, &[1]), // CFG-USBINPROT-RTCM3X
        (0x1073_0004, &[1]), // CFG-UART1INPROT-RTCM3X
        (0x1075_0004, &[1]), // CFG-UART2INPROT-RTCM3X
        (0x2091_0009, &[1]), // CFG-MSGOUT-UBX_NAV_PVT_USB
        (0x2091_0007, &[1]), // CFG-MSGOUT-UBX_NAV_PVT_UART1
        (0x2091_00BE, &[1]), // CFG-MSGOUT-NMEA_ID_GGA_USB
        (0x2091_00D6, &[1]), // CFG-MSGOUT-NMEA_ID_GST_USB
        (0x2091_02A7, &raw_rate), // CFG-MSGOUT-UBX_RXM_RAWX_USB
        (0x2091_02A5, &raw_rate), // CFG-MSGOUT-UBX_RXM_RAWX_UART1
        (0x2091_0234, &raw_rate), // CFG-MSGOUT-UBX_RXM_SFRBX_USB
    ]);

    vec![cfg_signals, cfg_io]
}

/// Séquence d'initialisation UBX classique (`CFG-RATE` + `CFG-MSG`) pour **u-blox M8T / Emlid Reach L1 (M+/RS+)**.
pub fn build_ubx_m8t_raw_init(rate_hz: u8, enable_raw_ppk: bool) -> Vec<Vec<u8>> {
    let meas_ms: u16 = (1000u16 / (rate_hz.clamp(1, 10) as u16)).max(100);
    let mut rate_payload = Vec::with_capacity(6);
    rate_payload.extend_from_slice(&meas_ms.to_le_bytes());
    rate_payload.extend_from_slice(&1u16.to_le_bytes()); // navRate = 1
    rate_payload.extend_from_slice(&1u16.to_le_bytes()); // timeRef = GPS

    let mut cmds = vec![
        encode_ubx_frame(0x06, 0x08, &rate_payload),        // UBX-CFG-RATE
        encode_ubx_frame(0x06, 0x01, &[0x01, 0x07, 1]),     // UBX-NAV-PVT
    ];
    let raw_rate = if enable_raw_ppk { 1u8 } else { 0u8 };
    cmds.push(encode_ubx_frame(0x06, 0x01, &[0x02, 0x15, raw_rate])); // UBX-RXM-RAWX (L1)
    cmds.push(encode_ubx_frame(0x06, 0x01, &[0x02, 0x13, raw_rate])); // UBX-RXM-SFRBX
    cmds
}

/// Associe `(gnss_id, sig_id)` d'un message `UBX-RXM-RAWX` (F9P / M8T) à l'index de bande `[0=L1, 1=L2, 2=L5]`.
fn ubx_signal_band_index(gnss_id: u8, sig_id: u8) -> usize {
    match gnss_id {
        // GPS : 0 = L1C/A, 3 = L2CL, 4 = L2CM, 6/7 = L5
        0 => match sig_id {
            3 | 4 => 1,
            6 | 7 => 2,
            _ => 0,
        },
        // Galileo : 0/1 = E1, 5/6 = E5b (bande 2), 3/4 = E5a (L5)
        2 => match sig_id {
            5 | 6 => 1,
            3 | 4 => 2,
            _ => 0,
        },
        // BeiDou : 0/1 = B1I/B1C, 2/3/5/6 = B2I/B2a
        3 => match sig_id {
            2 | 3 | 5 | 6 => 1,
            _ => 0,
        },
        // QZSS : 0/1 = L1, 4/5 = L2C
        5 => match sig_id {
            4 | 5 => 1,
            _ => 0,
        },
        // GLONASS : 0 = L1OF, 2 = L2OF
        6 => match sig_id {
            2 => 1,
            _ => 0,
        },
        _ => 0,
    }
}

fn parse_ubx_payload(cls: u8, id: u8, p: &[u8]) -> Option<UbxMessage> {
    match (cls, id) {
        // UBX-NAV-PVT (92 octets)
        (0x01, 0x07) if p.len() >= 92 => {
            let fix_type = p[20];
            let flags = p[21];
            let num_sv = p[23];
            let lon = i32::from_le_bytes(p[24..28].try_into().ok()?) as f64 * 1e-7;
            let lat = i32::from_le_bytes(p[28..32].try_into().ok()?) as f64 * 1e-7;
            let height = i32::from_le_bytes(p[32..36].try_into().ok()?) as f64 * 1e-3;
            let h_msl = i32::from_le_bytes(p[36..40].try_into().ok()?) as f64 * 1e-3;
            let h_acc = u32::from_le_bytes(p[40..44].try_into().ok()?) as f32 * 1e-3;
            let v_acc = u32::from_le_bytes(p[44..48].try_into().ok()?) as f32 * 1e-3;

            let carr_soln = (flags >> 6) & 0x03;
            let quality = match (fix_type, carr_soln) {
                (_, 2) => FixQuality::RtkFix,
                (_, 1) => FixQuality::RtkFloat,
                (3 | 4, _) => FixQuality::Single,
                _ => FixQuality::NoFix,
            };
            Some(UbxMessage::NavPvt {
                lat_deg: lat,
                lon_deg: lon,
                alt_ellipsoid_m: height,
                alt_msl_m: h_msl,
                h_acc_m: h_acc,
                v_acc_m: v_acc,
                num_sv,
                quality,
            })
        }
        // UBX-RXM-RAWX (16 + 32 * numMeas octets) — fusion L1 / L2 / L5 par satellite via sig_id
        (0x02, 0x15) if p.len() >= 16 => {
            let rcv_tow = f64::from_le_bytes(p[0..8].try_into().ok()?);
            let week = u16::from_le_bytes(p[8..10].try_into().ok()?) as i32;
            let num_meas = p[11] as usize;
            let time = GTime::from_gps_week_tow(week, rcv_tow);

            let mut by_sat: HashMap<SatId, ObsRecord> = HashMap::new();
            let mut order: Vec<SatId> = Vec::with_capacity(num_meas);

            for m in 0..num_meas {
                let off = 16 + m * 32;
                if off + 32 > p.len() {
                    break;
                }
                let pr = f64::from_le_bytes(p[off..off + 8].try_into().ok()?);
                let cp = f64::from_le_bytes(p[off + 8..off + 16].try_into().ok()?);
                let dop = f32::from_le_bytes(p[off + 16..off + 20].try_into().ok()?);
                let gnss_id = p[off + 20];
                let sv_id = p[off + 21];
                let sig_id = p[off + 22];
                let cno = p[off + 26] as f32;
                let trk_stat = p[off + 30];

                let constellation = match gnss_id {
                    0 => Constellation::Gps,
                    2 => Constellation::Galileo,
                    3 => Constellation::BeiDou,
                    5 => Constellation::Qzss,
                    6 => Constellation::Glonass,
                    _ => continue,
                };
                let sat = SatId::new(constellation, sv_id);
                let band = ubx_signal_band_index(gnss_id, sig_id);

                let rec = by_sat.entry(sat).or_insert_with(|| {
                    order.push(sat);
                    ObsRecord::new(sat)
                });
                if (trk_stat & 0x01) != 0 && pr.is_finite() {
                    rec.pseudorange[band] = pr;
                }
                if (trk_stat & 0x02) != 0 && cp.is_finite() {
                    rec.carrier_phase[band] = cp;
                }
                rec.doppler[band] = dop;
                rec.snr[band] = cno;
                // Bit 2 = half-cycle non résolu -> marquer LLI
                if (trk_stat & 0x04) == 0 && (trk_stat & 0x02) != 0 {
                    rec.lli[band] |= 1;
                }
            }

            let records = order
                .into_iter()
                .filter_map(|sat| by_sat.remove(&sat))
                .collect();
            Some(UbxMessage::RxmRawx(ObsEpoch { time, records }))
        }
        _ => None,
    }
}

/// Lit un fichier binaire brut `.ubx` (Emlid Reach L1/M2/RS2+/RS3/RS4 Pro ou SparkFun/u-blox ZED-F9P)
/// et extrait toutes les époques `UBX-RXM-RAWX` (L1/L2/L5) + position approximative ECEF pour PPK.
pub fn read_ubx_obs_file<P: AsRef<Path>>(path: P) -> Result<(Option<[f64; 3]>, Vec<ObsEpoch>)> {
    let bytes = std::fs::read(path.as_ref())
        .with_context(|| format!("Impossible de lire le fichier UBX {:?}", path.as_ref()))?;
    let mut dec = UbxStreamDecoder::default();
    let mut epochs = Vec::new();
    let mut approx_ecef = None;

    for msg in dec.push_bytes(&bytes) {
        match msg {
            UbxMessage::RxmRawx(ep) => {
                if !ep.records.is_empty() {
                    epochs.push(ep);
                }
            }
            UbxMessage::NavPvt {
                lat_deg,
                lon_deg,
                alt_ellipsoid_m,
                quality,
                ..
            } => {
                if approx_ecef.is_none() && quality != FixQuality::NoFix {
                    let (ex, ey, ez) = geodetic_to_ecef(
                        lat_deg.to_radians(),
                        lon_deg.to_radians(),
                        alt_ellipsoid_m,
                        &WGS84,
                    );
                    approx_ecef = Some([ex, ey, ez]);
                }
            }
        }
    }
    Ok((approx_ecef, epochs))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ubx_f9p_dual_freq_rawx_merge_l1_l2() {
        // Construit une trame UBX-RXM-RAWX contenant G05 en L1 (sig_id=0) + L2C (sig_id=3)
        let mut payload = vec![0u8; 16 + 32 * 2];
        payload[0..8].copy_from_slice(&200_000.0f64.to_le_bytes());
        payload[8..10].copy_from_slice(&2300u16.to_le_bytes());
        payload[11] = 2; // numMeas = 2

        // Mesure 0 : GPS PRN 5, sig_id=0 (L1C/A)
        let o0 = 16;
        payload[o0..o0 + 8].copy_from_slice(&21_500_000.5f64.to_le_bytes());
        payload[o0 + 8..o0 + 16].copy_from_slice(&113_000_000.25f64.to_le_bytes());
        payload[o0 + 20] = 0; // GPS
        payload[o0 + 21] = 5; // PRN 5
        payload[o0 + 22] = 0; // L1C/A
        payload[o0 + 26] = 46; // C/N0
        payload[o0 + 30] = 0x07; // pr + cp + half-cycle valid

        // Mesure 1 : GPS PRN 5, sig_id=3 (L2CL)
        let o1 = 16 + 32;
        payload[o1..o1 + 8].copy_from_slice(&21_500_004.2f64.to_le_bytes());
        payload[o1 + 8..o1 + 16].copy_from_slice(&88_050_000.75f64.to_le_bytes());
        payload[o1 + 20] = 0; // GPS
        payload[o1 + 21] = 5; // PRN 5
        payload[o1 + 22] = 3; // L2CL
        payload[o1 + 26] = 43; // C/N0
        payload[o1 + 30] = 0x07;

        let frame = encode_ubx_frame(0x02, 0x15, &payload);
        let mut dec = UbxStreamDecoder::default();
        let msgs = dec.push_bytes(&frame);
        assert_eq!(msgs.len(), 1);
        match &msgs[0] {
            UbxMessage::RxmRawx(ep) => {
                assert_eq!(ep.records.len(), 1);
                let r = &ep.records[0];
                assert_eq!(r.sat, SatId::new(Constellation::Gps, 5));
                assert!((r.pseudorange[0] - 21_500_000.5).abs() < 1e-6);
                assert!((r.pseudorange[1] - 21_500_004.2).abs() < 1e-6);
                assert!((r.carrier_phase[0] - 113_000_000.25).abs() < 1e-6);
                assert!((r.carrier_phase[1] - 88_050_000.75).abs() < 1e-6);
            }
            _ => panic!("Attendu UbxMessage::RxmRawx"),
        }
    }

    #[test]
    fn test_ubx_cfg_valset_checksum() {
        let cmds = build_ubx_f9p_rtk_ppk_init(5, true);
        assert_eq!(cmds.len(), 2);
        for frame in cmds {
            let n = frame.len();
            let (ca, cb) = ubx_checksum(&frame[2..n - 2]);
            assert_eq!(ca, frame[n - 2]);
            assert_eq!(cb, frame[n - 1]);
        }
    }
}
