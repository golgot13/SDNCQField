//! Décodeur binaire UBX u-blox (`NAV-PVT` & `RXM-RAWX`) pour récepteurs ZED-F9P / M8T.
//!
//! Permet d'alimenter directement le moteur RTK (`RtkEngine`) et l'enregistreur RINEX PPK
//! depuis un récepteur u-blox raccordé en TCP/UDP/Série.

use crate::types::{Constellation, FixQuality, GTime, ObsEpoch, ObsRecord, SatId};

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

            // Checksum Fletcher-8 sur [cls..payload_end]
            let (mut ck_a, mut ck_b) = (0u8, 0u8);
            for &b in &self.buf[2..6 + len] {
                ck_a = ck_a.wrapping_add(b);
                ck_b = ck_b.wrapping_add(ck_a);
            }
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
        // UBX-RXM-RAWX (16 + 32 * numMeas octets)
        (0x02, 0x15) if p.len() >= 16 => {
            let rcv_tow = f64::from_le_bytes(p[0..8].try_into().ok()?);
            let week = u16::from_le_bytes(p[8..10].try_into().ok()?) as i32;
            let num_meas = p[11] as usize;
            let time = GTime::from_gps_week_tow(week, rcv_tow);
            let mut records = Vec::with_capacity(num_meas);

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
                let mut rec = ObsRecord::new(SatId::new(constellation, sv_id));
                if (trk_stat & 0x01) != 0 {
                    rec.pseudorange[0] = pr;
                }
                if (trk_stat & 0x02) != 0 {
                    rec.carrier_phase[0] = cp;
                }
                rec.doppler[0] = dop;
                rec.snr[0] = cno;
                records.push(rec);
            }
            Some(UbxMessage::RxmRawx(ObsEpoch { time, records }))
        }
        _ => None,
    }
}
