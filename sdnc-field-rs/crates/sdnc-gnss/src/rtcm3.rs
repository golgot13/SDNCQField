//! Décodeur RTCM 3.x (CRC-24Q + Station ARP 1005/1006 + entêtes MSM4/MSM7).
//!
//! Inspiré de `rtcm3.c` de RTKLIB : permet d'extraire la position de la base RTK
//! et le flux de messages de correction différentielle reçu via NTRIP / radio.

#[derive(Debug, Clone)]
pub enum Rtcm3Message {
    /// Message 1005 / 1006 : Position ARP de la station de base en ECEF (mètres)
    StationArp {
        station_id: u16,
        ecef: [f64; 3],
        antenna_height_m: f64,
    },
    /// Message MSM4/MSM7 (1074/1077 GPS, 1084/1087 GLO, 1094/1097 GAL, 1124/1127 BDS)
    MsmHeader {
        msg_type: u16,
        station_id: u16,
        epoch_ms: u32,
        sync_flag: bool,
    },
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

fn get_bits_u64(buf: &[u8], bit_pos: usize, len: usize) -> u64 {
    let mut val = 0u64;
    for i in 0..len {
        let pos = bit_pos + i;
        let byte_idx = pos / 8;
        let bit_idx = 7 - (pos % 8);
        if byte_idx < buf.len() {
            val = (val << 1) | (((buf[byte_idx] >> bit_idx) & 1) as u64);
        }
    }
    val
}

fn get_bits_i64(buf: &[u8], bit_pos: usize, len: usize) -> i64 {
    let u = get_bits_u64(buf, bit_pos, len);
    if len == 0 || len >= 64 {
        return u as i64;
    }
    let sign_bit = 1u64 << (len - 1);
    if (u & sign_bit) != 0 {
        (u | (!0u64 << len)) as i64
    } else {
        u as i64
    }
}

/// Décodeur incrémental de flux binaire RTCM 3.x.
#[derive(Default)]
pub struct Rtcm3StreamDecoder {
    buf: Vec<u8>,
    pub valid_frames: u64,
    pub crc_errors: u64,
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
                let payload = &self.buf[3..3 + payload_len];
                if let Some(msg) = parse_rtcm3_payload(payload) {
                    out.push(msg);
                }
                self.buf.drain(0..frame_len);
            } else {
                self.crc_errors += 1;
                self.buf.drain(0..1);
            }
        }

        out
    }
}

fn parse_rtcm3_payload(payload: &[u8]) -> Option<Rtcm3Message> {
    if payload.len() < 2 {
        return None;
    }
    let msg_type = get_bits_u64(payload, 0, 12) as u16;
    match msg_type {
        1005 | 1006 if payload.len() >= 19 => {
            let station_id = get_bits_u64(payload, 12, 12) as u16;
            let x_raw = get_bits_i64(payload, 34, 38);
            let y_raw = get_bits_i64(payload, 74, 38);
            let z_raw = get_bits_i64(payload, 114, 38);
            let ant_h = if msg_type == 1006 && payload.len() >= 21 {
                get_bits_u64(payload, 152, 16) as f64 * 0.0001
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
        1074 | 1077 | 1084 | 1087 | 1094 | 1097 | 1124 | 1127 if payload.len() >= 8 => {
            let station_id = get_bits_u64(payload, 12, 12) as u16;
            let epoch_ms = get_bits_u64(payload, 24, 30) as u32;
            let sync_flag = get_bits_u64(payload, 54, 1) == 1;
            Some(Rtcm3Message::MsmHeader {
                msg_type,
                station_id,
                epoch_ms,
                sync_flag,
            })
        }
        _ => Some(Rtcm3Message::Other {
            msg_type,
            payload_len: payload.len(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rtcm3_crc24q_frame_decode() {
        // Construit une trame RTCM3 valide de type 1077 (8 octets payload)
        let mut payload = vec![0u8; 8];
        // msg_type 1077 = 0x435 -> bits 0..12
        payload[0] = 0x43;
        payload[1] = 0x50;
        let mut frame = vec![0xD3, 0x00, payload.len() as u8];
        frame.extend_from_slice(&payload);
        let crc = crc24q(&frame);
        frame.push(((crc >> 16) & 0xFF) as u8);
        frame.push(((crc >> 8) & 0xFF) as u8);
        frame.push((crc & 0xFF) as u8);

        let mut dec = Rtcm3StreamDecoder::default();
        let msgs = dec.push_bytes(&frame);
        assert_eq!(dec.valid_frames, 1);
        assert_eq!(msgs.len(), 1);
        assert!(matches!(msgs[0], Rtcm3Message::MsmHeader { msg_type: 1077, .. }));
    }
}
