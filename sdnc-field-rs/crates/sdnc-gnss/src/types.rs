//! Types fondamentaux GNSS / RTK / PPK inspirés de RTKLIB (`rtklib.h`).

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

pub const CLIGHT: f64 = 299_792_458.0; // Vitesse de la lumière (m/s)
pub const OMGE: f64 = 7.292_115_146_7e-5; // Vitesse angulaire Terre WGS84 (rad/s)
pub const MU_GPS: f64 = 3.986_005e14; // Constante gravitationnelle GPS (m^3/s^2)
pub const FREQ1: f64 = 1.575_42e9; // L1 / E1 / B1C (Hz)
pub const FREQ2: f64 = 1.227_60e9; // L2 (Hz)
pub const FREQ5: f64 = 1.176_45e9; // L5 / E5a (Hz)
pub const LAMBDA1: f64 = CLIGHT / FREQ1; // ~0.19029367 m
pub const LAMBDA2: f64 = CLIGHT / FREQ2; // ~0.24421021 m
pub const LAMBDA5: f64 = CLIGHT / FREQ5; // ~0.25482805 m
pub const RE_WGS84: f64 = 6_378_137.0;
pub const FE_WGS84: f64 = 1.0 / 298.257_223_563;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Constellation {
    Gps,     // G
    Glonass, // R
    Galileo, // E
    BeiDou,  // C
    Qzss,    // J
}

impl Constellation {
    pub fn prefix(&self) -> char {
        match self {
            Self::Gps => 'G',
            Self::Glonass => 'R',
            Self::Galileo => 'E',
            Self::BeiDou => 'C',
            Self::Qzss => 'J',
        }
    }

    pub fn from_prefix(c: char) -> Option<Self> {
        match c.to_ascii_uppercase() {
            'G' => Some(Self::Gps),
            'R' => Some(Self::Glonass),
            'E' => Some(Self::Galileo),
            'C' => Some(Self::BeiDou),
            'J' => Some(Self::Qzss),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SatId {
    pub constellation: Constellation,
    pub prn: u8,
}

impl SatId {
    pub fn new(constellation: Constellation, prn: u8) -> Self {
        Self { constellation, prn }
    }

    pub fn code(&self) -> String {
        format!("{}{:02}", self.constellation.prefix(), self.prn)
    }

    pub fn parse(s: &str) -> Option<Self> {
        let s = s.trim();
        let mut chars = s.chars();
        let c = chars.next()?;
        let constellation = Constellation::from_prefix(c)?;
        let prn: u8 = chars.as_str().trim().parse().ok()?;
        Some(Self { constellation, prn })
    }
}

/// Temps GNSS haute précision (équivalent `gtime_t` RTKLIB : secondes Unix + fraction).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct GTime {
    pub sec: i64,
    pub frac: f64,
}

impl GTime {
    pub fn new(sec: i64, frac: f64) -> Self {
        let extra = frac.floor() as i64;
        Self {
            sec: sec + extra,
            frac: frac - extra as f64,
        }
    }

    pub fn from_unix_f64(t: f64) -> Self {
        let sec = t.floor() as i64;
        Self {
            sec,
            frac: t - sec as f64,
        }
    }

    pub fn to_unix_f64(&self) -> f64 {
        self.sec as f64 + self.frac
    }

    /// Différence `self - other` en secondes.
    pub fn diff(&self, other: GTime) -> f64 {
        (self.sec - other.sec) as f64 + (self.frac - other.frac)
    }

    pub fn add(&self, dt: f64) -> Self {
        Self::new(self.sec, self.frac + dt)
    }

    /// Conversion Semaine GPS + TOW -> GTime (époque GPS 1980-01-06 00:00:00 UTC = 315964800).
    pub fn from_gps_week_tow(week: i32, tow: f64) -> Self {
        const GPS_EPOCH_UNIX: i64 = 315_964_800;
        let sec = GPS_EPOCH_UNIX + (week as i64) * 604_800 + tow.floor() as i64;
        Self::new(sec, tow - tow.floor())
    }

    /// Conversion GTime -> `(week, tow)` GPS.
    pub fn to_gps_week_tow(&self) -> (i32, f64) {
        const GPS_EPOCH_UNIX: i64 = 315_964_800;
        let dt = self.sec - GPS_EPOCH_UNIX;
        let week = (dt / 604_800) as i32;
        let tow = (dt - (week as i64) * 604_800) as f64 + self.frac;
        (week, tow)
    }

    /// Conversion date calendaire UTC/GPST -> GTime.
    pub fn from_ymdhms(year: i32, month: u32, day: u32, hour: u32, min: u32, sec: f64) -> Self {
        let sec_int = sec.floor() as u32;
        let frac = sec - sec_int as f64;
        if let Some(date) = chrono::NaiveDate::from_ymd_opt(year, month, day) {
            if let Some(dt) = date.and_hms_opt(hour, min, sec_int) {
                return Self::new(dt.and_utc().timestamp(), frac);
            }
        }
        Self::default()
    }
}

/// Observation brute d'un satellite à une époque (code + phase porteuse + Doppler + SNR).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObsRecord {
    pub sat: SatId,
    /// Pseudodistances code [L1, L2, L5] en mètres (0.0 si absent)
    pub pseudorange: [f64; 3],
    /// Phase porteuse [L1, L2, L5] en cycles (0.0 si absent)
    pub carrier_phase: [f64; 3],
    /// Doppler [L1, L2, L5] en Hz
    pub doppler: [f32; 3],
    /// Rapport signal/bruit C/N0 [L1, L2, L5] en dB-Hz
    pub snr: [f32; 3],
    /// Loss-of-Lock Indicator [L1, L2, L5] (bit 0 = cycle slip)
    pub lli: [u8; 3],
}

impl ObsRecord {
    pub fn new(sat: SatId) -> Self {
        Self {
            sat,
            pseudorange: [0.0; 3],
            carrier_phase: [0.0; 3],
            doppler: [0.0; 3],
            snr: [0.0; 3],
            lli: [0; 3],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ObsEpoch {
    pub time: GTime,
    pub records: Vec<ObsRecord>,
}

/// Éphéméride radiodiffusée Kepler (équivalent `eph_t` RTKLIB pour GPS/Galileo/BeiDou/QZSS).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BroadcastEphemeris {
    pub sat: SatId,
    pub iode: i32,
    pub week: i32,
    pub toe_s: f64,
    pub toc: GTime,
    pub toe: GTime,
    pub sqrt_a: f64,
    pub e: f64,
    pub i0: f64,
    pub omg0: f64,
    pub omg: f64,
    pub m0: f64,
    pub deln: f64,
    pub idot: f64,
    pub omgd: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
    pub f0: f64,
    pub f1: f64,
    pub f2: f64,
    pub tgd: f64,
    pub sv_health: u8,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NavData {
    pub ephemerides: HashMap<SatId, Vec<BroadcastEphemeris>>,
    pub iono_gps: [f64; 8], // alpha[0..4], beta[4..8]
}

impl NavData {
    pub fn add_ephemeris(&mut self, eph: BroadcastEphemeris) {
        self.ephemerides.entry(eph.sat).or_default().push(eph);
    }

    /// Sélectionne l'éphéméride la plus proche de `t` (tolérance 4h).
    pub fn select_eph(&self, sat: SatId, t: GTime) -> Option<&BroadcastEphemeris> {
        let list = self.ephemerides.get(&sat)?;
        let mut best: Option<(&BroadcastEphemeris, f64)> = None;
        for eph in list {
            let dt = t.diff(eph.toe).abs();
            if dt <= 14_400.0 && best.map_or(true, |(_, bd)| dt < bd) {
                best = Some((eph, dt));
            }
        }
        best.map(|(e, _)| e)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FixQuality {
    NoFix,
    Single,
    Dgnss,
    RtkFloat,
    RtkFix,
    PpkFloat,
    PpkFix,
}

impl FixQuality {
    pub fn label(&self) -> &'static str {
        match self {
            Self::NoFix => "AUCUN FIX",
            Self::Single => "SPP (Autonome)",
            Self::Dgnss => "DGNSS (Code)",
            Self::RtkFloat => "RTK FLOAT",
            Self::RtkFix => "RTK FIX",
            Self::PpkFloat => "PPK FLOAT",
            Self::PpkFix => "PPK FIX",
        }
    }

    pub fn color_rgb(&self) -> [u8; 3] {
        match self {
            Self::NoFix => [220, 60, 60],
            Self::Single => [235, 160, 40],
            Self::Dgnss => [120, 200, 255],
            Self::RtkFloat | Self::PpkFloat => [255, 220, 40],
            Self::RtkFix | Self::PpkFix => [40, 235, 110],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SatSkyInfo {
    pub sat: SatId,
    pub azimuth_deg: f32,
    pub elevation_deg: f32,
    pub snr_dbhz: f32,
    pub used_in_fix: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GnssSolution {
    pub time: GTime,
    pub quality: FixQuality,
    pub ecef: [f64; 3],
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_ellipsoid_m: f64,
    pub alt_orthometric_m: f64,
    pub std_enu_m: [f32; 3],
    pub hdop: f32,
    pub pdop: f32,
    pub num_sats: u8,
    pub age_diff_s: f32,
    pub lambda_ratio: f32,
    pub baseline_m: f64,
    pub satellites: Vec<SatSkyInfo>,
}

impl Default for GnssSolution {
    fn default() -> Self {
        Self {
            time: GTime::default(),
            quality: FixQuality::NoFix,
            ecef: [0.0; 3],
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_ellipsoid_m: 0.0,
            alt_orthometric_m: 0.0,
            std_enu_m: [99.0; 3],
            hdop: 99.0,
            pdop: 99.0,
            num_sats: 0,
            age_diff_s: 0.0,
            lambda_ratio: 0.0,
            baseline_m: 0.0,
            satellites: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssMode {
    Single,
    RtkKinematic,
    RtkStatic,
    PpkForward,
    PpkCombined,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReceiverSource {
    SimulatedRtk,
    TcpStream,
    UdpStream,
    SerialPort,
    BluetoothExternal,
    AndroidInternal,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NtripConfig {
    pub enabled: bool,
    pub host: String,
    pub port: u16,
    pub mountpoint: String,
    pub username: String,
    pub password: String,
    pub send_gga: bool,
}

impl Default for NtripConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            host: "caster.centipede.fr".to_string(),
            port: 2101,
            mountpoint: "NEAR".to_string(),
            username: "centipede".to_string(),
            password: "centipede".to_string(),
            send_gga: true,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GnssConfig {
    pub mode: GnssMode,
    pub source: ReceiverSource,
    pub tcp_host: String,
    pub tcp_port: u16,
    pub udp_port: u16,
    /// Port série USB/COM pour u-blox ZED-F9P / Emlid Reach (ex. `/dev/ttyACM0` ou `COM3`)
    pub serial_port_path: String,
    pub serial_baud_rate: u32,
    /// Récepteur Bluetooth externe Android SPP/BLE (ex. `Reach RS2+`, `u-blox ZED-F9P`)
    pub bluetooth_device_name: String,
    pub bluetooth_device_mac: String,
    pub elevation_mask_deg: f64,
    pub lambda_ratio_thresh: f64,
    pub code_sigma_m: f64,
    pub phase_sigma_m: f64,
    pub antenna_height_m: f64,
    pub ntrip: NtripConfig,
    pub base_ecef_override: Option<[f64; 3]>,
    pub log_raw_ppk: bool,
    pub raw_ppk_output_path: String,
}

impl Default for GnssConfig {
    fn default() -> Self {
        Self {
            mode: GnssMode::RtkKinematic,
            source: ReceiverSource::SimulatedRtk,
            tcp_host: "127.0.0.1".to_string(),
            tcp_port: 5000,
            udp_port: 5001,
            serial_port_path: if cfg!(target_os = "windows") {
                "COM3".to_string()
            } else {
                "/dev/ttyACM0".to_string()
            },
            serial_baud_rate: 115_200,
            bluetooth_device_name: "Emlid Reach / u-blox ZED-F9P".to_string(),
            bluetooth_device_mac: String::new(),
            elevation_mask_deg: 15.0,
            lambda_ratio_thresh: 3.0,
            code_sigma_m: 0.3,
            phase_sigma_m: 0.003,
            antenna_height_m: 2.0,
            ntrip: NtripConfig::default(),
            base_ecef_override: None,
            log_raw_ppk: false,
            raw_ppk_output_path: "gnss_logs/rover_raw.obs".to_string(),
        }
    }
}
