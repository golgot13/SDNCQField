//! Profils matériels et pilotes de commande pour récepteurs GNSS RTK / PPK professionnels.
//!
//! Couvre les principales marques terrain :
//! - **Leica Geosystems** (GS18 T / GS16 / GS15 / GS07 / Zeno FLX100 Plus — L1/L2/L5)
//! - **Trimble** (R12i / R12 / R10 / R8s / R2 / Catalyst DA2 — L1/L2/L5)
//! - **Sokkia / Topcon** (GCX3 / GRX3 / GSX2 / HiPer VR — L1/L2/L5, commandes GREIS/TPS)
//! - **SparkFun & modules u-blox ZED-F9P** (RTK Facet / Express / Torch, GPS-RTK2, ArduSimple simpleRTK2B — L1/L2, UBX-CFG-VALSET)
//! - **Emlid Reach** (gamme complète : Reach M+/RS+ **L1**, Reach M2 **L1/L2**, RS2/RS2+ **L1/L2**, RS3 **L1/L2+Tilt**, **RS4 Pro L1/L2/L5+Tilt**)

use crate::types::{FrequencyBandMode, ReceiverSource};
use crate::ubx::{build_ubx_f9p_rtk_ppk_init, build_ubx_m8t_raw_init};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum ReceiverProfile {
    /// u-blox ZED-F9P générique / ArduSimple simpleRTK2B (L1/L2)
    #[default]
    UbloxZedF9p,
    /// SparkFun RTK Facet / RTK Express / RTK Torch (ZED-F9P L1/L2)
    SparkFunRtkF9p,
    /// Emlid Reach M+ / RS+ (puce u-blox M8T — Mono-fréquence L1)
    EmlidReachL1,
    /// Emlid Reach M2 (puce u-blox ZED-F9P — L1/L2)
    EmlidReachM2,
    /// Emlid Reach RS2 / RS2+ (Multi-bande L1/L2)
    EmlidReachRs2Plus,
    /// Emlid Reach RS3 (L1/L2 + compensation inclinaison IMU)
    EmlidReachRs3,
    /// Emlid Reach RS4 Pro (Triple-bande L1/L2/L5 + IMU Tilt + AR)
    EmlidReachRs4Pro,
    /// Leica Geosystems GS18 T / GS16 / GS15 / GS07 / FLX100 (L1/L2/L5)
    LeicaGsSeries,
    /// Trimble R12i / R12 / R10 / R8s / R2 / DA2 (L1/L2/L5)
    TrimbleRSeries,
    /// Sokkia GCX3 / GRX3 / GSX2 & Topcon HiPer VR / HR (L1/L2/L5)
    SokkiaTopconSeries,
    /// Récepteur RTK générique NMEA / RTCM / UBX (Septentrio, CHCNav, Stonex, Tersus...)
    GenericRtkL1L2,
}

#[derive(Debug, Clone)]
pub struct ReceiverCapabilities {
    pub brand: &'static str,
    pub model_label: &'static str,
    pub bands_label: &'static str,
    pub default_freq_mode: FrequencyBandMode,
    pub default_source: ReceiverSource,
    pub default_baud: u32,
    pub default_tcp_port: u16,
    /// Décalage interne constructeur ARP -> APC (mètres) à ajouter à la hauteur de canne
    pub default_arp_offset_m: f64,
    pub supports_raw_ppk: bool,
    pub supports_imu_tilt: bool,
    pub protocols_summary: &'static str,
    pub bt_name_hint: &'static str,
}

impl ReceiverProfile {
    pub const ALL: &'static [ReceiverProfile] = &[
        ReceiverProfile::UbloxZedF9p,
        ReceiverProfile::SparkFunRtkF9p,
        ReceiverProfile::EmlidReachL1,
        ReceiverProfile::EmlidReachM2,
        ReceiverProfile::EmlidReachRs2Plus,
        ReceiverProfile::EmlidReachRs3,
        ReceiverProfile::EmlidReachRs4Pro,
        ReceiverProfile::LeicaGsSeries,
        ReceiverProfile::TrimbleRSeries,
        ReceiverProfile::SokkiaTopconSeries,
        ReceiverProfile::GenericRtkL1L2,
    ];

    pub fn capabilities(&self) -> ReceiverCapabilities {
        match self {
            Self::UbloxZedF9p => ReceiverCapabilities {
                brand: "u-blox",
                model_label: "u-blox ZED-F9P / ArduSimple (L1/L2)",
                bands_label: "L1 + L2 (GPS/GAL/GLO/BDS)",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::SerialPort,
                default_baud: 115_200,
                default_tcp_port: 5000,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "UBX (NAV-PVT + RXM-RAWX L1/L2 + CFG-VALSET) + NMEA + RTCM3",
                bt_name_hint: "ZED-F9P / ArduSimple",
            },
            Self::SparkFunRtkF9p => ReceiverCapabilities {
                brand: "SparkFun",
                model_label: "SparkFun RTK Facet / Express / Torch (F9P L1/L2)",
                bands_label: "L1 + L2 (ZED-F9P)",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 2947,
                default_arp_offset_m: 0.061,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "UBX (NAV-PVT + RXM-RAWX L1/L2) + NMEA GGA/GST + RTCM3",
                bt_name_hint: "SparkFun RTK Facet",
            },
            Self::EmlidReachL1 => ReceiverCapabilities {
                brand: "Emlid",
                model_label: "Emlid Reach M+ / RS+ (L1)",
                bands_label: "L1 seul (u-blox M8T)",
                default_freq_mode: FrequencyBandMode::L1Only,
                default_source: ReceiverSource::TcpStream,
                default_baud: 115_200,
                default_tcp_port: 9000,
                default_arp_offset_m: 0.065,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "NMEA GGA/GST + UBX RXM-RAWX L1 + RINEX/UBX PPK",
                bt_name_hint: "Reach RS+ / Reach M+",
            },
            Self::EmlidReachM2 => ReceiverCapabilities {
                brand: "Emlid",
                model_label: "Emlid Reach M2 (L1/L2 F9P)",
                bands_label: "L1 + L2 (ZED-F9P)",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::TcpStream,
                default_baud: 115_200,
                default_tcp_port: 9000,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "NMEA GGA/GST + UBX RXM-RAWX L1/L2 + RINEX/UBX PPK",
                bt_name_hint: "Reach M2",
            },
            Self::EmlidReachRs2Plus => ReceiverCapabilities {
                brand: "Emlid",
                model_label: "Emlid Reach RS2 / RS2+ (L1/L2)",
                bands_label: "L1 + L2 (Multi-constellation)",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 9000,
                default_arp_offset_m: 0.134,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "NMEA GGA/GST/GSV + UBX RAWX L1/L2 + RTCM3 + PPK RINEX",
                bt_name_hint: "Reach RS2+",
            },
            Self::EmlidReachRs3 => ReceiverCapabilities {
                brand: "Emlid",
                model_label: "Emlid Reach RS3 (L1/L2 + Tilt)",
                bands_label: "L1 + L2 + IMU Tilt 60°",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 9000,
                default_arp_offset_m: 0.134,
                supports_raw_ppk: true,
                supports_imu_tilt: true,
                protocols_summary: "NMEA GGA/GST/GSV + IMU Tilt + RTCM3 + PPK RINEX/UBX",
                bt_name_hint: "Reach RS3",
            },
            Self::EmlidReachRs4Pro => ReceiverCapabilities {
                brand: "Emlid",
                model_label: "Emlid Reach RS4 Pro (L1/L2/L5 + Tilt)",
                bands_label: "L1 + L2 + L5 + IMU Tilt",
                default_freq_mode: FrequencyBandMode::L1L2L5,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 9000,
                default_arp_offset_m: 0.134,
                supports_raw_ppk: true,
                supports_imu_tilt: true,
                protocols_summary: "NMEA GGA/GST/GSV + UBX/RINEX L1/L2/L5 + RTCM3 MSM7 + PPK",
                bt_name_hint: "Reach RS4 Pro",
            },
            Self::LeicaGsSeries => ReceiverCapabilities {
                brand: "Leica",
                model_label: "Leica GS18 T / GS16 / GS15 / GS07 (L1/L2/L5)",
                bands_label: "L1 + L2 + L5 (SmartTrack+)",
                default_freq_mode: FrequencyBandMode::L1L2L5,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 5000,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: true,
                protocols_summary: "NMEA GGA/GST/GSV + Leica $GPLLQ + RTCM 3.3 + RINEX PPK",
                bt_name_hint: "Leica GS18 / GS16",
            },
            Self::TrimbleRSeries => ReceiverCapabilities {
                brand: "Trimble",
                model_label: "Trimble R12i / R12 / R10 / R8s / DA2 (L1/L2/L5)",
                bands_label: "L1 + L2 + L5 (ProPoint / Maxwell)",
                default_freq_mode: FrequencyBandMode::L1L2L5,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 5017,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: true,
                protocols_summary: "NMEA GGA/GST/GSV + Trimble $PTNL,GGK/AVR + RTCM3 + RINEX PPK",
                bt_name_hint: "Trimble R12i / R10",
            },
            Self::SokkiaTopconSeries => ReceiverCapabilities {
                brand: "Sokkia / Topcon",
                model_label: "Sokkia GCX3 / GRX3 / Topcon HiPer VR (L1/L2/L5)",
                bands_label: "L1 + L2 + L5 (Vanguard)",
                default_freq_mode: FrequencyBandMode::L1L2L5,
                default_source: ReceiverSource::BluetoothExternal,
                default_baud: 115_200,
                default_tcp_port: 8002,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: true,
                protocols_summary: "NMEA GGA/GST/GSV + Commandes GREIS/TPS + RTCM3 + RINEX PPK",
                bt_name_hint: "Sokkia GRX3 / Topcon HiPer",
            },
            Self::GenericRtkL1L2 => ReceiverCapabilities {
                brand: "Générique",
                model_label: "Récepteur RTK Générique L1/L2 (Septentrio, CHC, Stonex...)",
                bands_label: "L1 + L2 (Standard)",
                default_freq_mode: FrequencyBandMode::L1L2,
                default_source: ReceiverSource::SerialPort,
                default_baud: 115_200,
                default_tcp_port: 5000,
                default_arp_offset_m: 0.0,
                supports_raw_ppk: true,
                supports_imu_tilt: false,
                protocols_summary: "NMEA-0183 + UBX + RTCM 3.3 MSM4/MSM7 + RINEX 3 PPK",
                bt_name_hint: "GNSS RTK",
            },
        }
    }

    pub fn label(&self) -> &'static str {
        self.capabilities().model_label
    }

    /// Génère les trames binaires ou ASCII d'initialisation à envoyer au récepteur
    /// pour activer la sortie RTK (NMEA GGA/GST/GSV ou UBX NAV-PVT), les observations brutes PPK
    /// (`UBX-RXM-RAWX` L1/L2 ou GREIS) et la réception des corrections RTCM3.
    pub fn build_init_commands(&self, rate_hz: u8, enable_raw_ppk: bool) -> Vec<Vec<u8>> {
        let hz = rate_hz.clamp(1, 10);
        match self {
            Self::UbloxZedF9p | Self::SparkFunRtkF9p | Self::EmlidReachM2 => {
                build_ubx_f9p_rtk_ppk_init(hz, enable_raw_ppk)
            }
            Self::EmlidReachL1 => build_ubx_m8t_raw_init(hz, enable_raw_ppk),
            Self::SokkiaTopconSeries => {
                let period_s = 1.0 / (hz as f32);
                let mut cmds = vec![
                    format!("em,,/msg/nmea/GGA:{:.2}\r\n", period_s).into_bytes(),
                    format!("em,,/msg/nmea/GST:{:.2}\r\n", period_s).into_bytes(),
                    b"em,,/msg/nmea/GSA:1.0\r\n".to_vec(),
                    b"em,,/msg/nmea/GSV:1.0\r\n".to_vec(),
                ];
                if enable_raw_ppk {
                    cmds.push(format!("em,,/msg/jps/RC:{:.2}\r\n", period_s).into_bytes());
                    cmds.push(format!("em,,/msg/jps/CP:{:.2}\r\n", period_s).into_bytes());
                }
                cmds
            }
            Self::LeicaGsSeries => {
                // Leica conserve généralement la sortie configurée dans Captivate/WebUI,
                // tout en acceptant une interrogation NMEA douce sans perturber la session RTK.
                vec![b"$PASHQ,RID\r\n".to_vec()]
            }
            Self::TrimbleRSeries
            | Self::EmlidReachRs2Plus
            | Self::EmlidReachRs3
            | Self::EmlidReachRs4Pro
            | Self::GenericRtkL1L2 => {
                // Emlid Reach RS2+/RS3/RS4 Pro (configurés via Emlid Flow) et Trimble R-Series
                // diffusent nativement NMEA GGA/GST/GSV + PTNL,GGK / UBX et acceptent directement RTCM3 en entrée.
                Vec::new()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_all_receiver_profiles_metadata_and_init_commands() {
        assert_eq!(ReceiverProfile::ALL.len(), 11);

        // SparkFun / ZED-F9P génère bien des trames UBX-CFG-VALSET valides (sync 0xB5 0x62)
        let f9p_cmds = ReceiverProfile::SparkFunRtkF9p.build_init_commands(5, true);
        assert!(!f9p_cmds.is_empty());
        for cmd in &f9p_cmds {
            assert!(cmd.len() >= 8);
            assert_eq!(cmd[0], 0xB5);
            assert_eq!(cmd[1], 0x62);
        }

        // Emlid Reach L1 est mono-fréquence L1 et génère des commandes UBX M8T
        assert_eq!(
            ReceiverProfile::EmlidReachL1.capabilities().default_freq_mode,
            FrequencyBandMode::L1Only
        );
        assert!(!ReceiverProfile::EmlidReachL1.build_init_commands(1, true).is_empty());

        // Emlid Reach RS4 Pro est triple-fréquence L1/L2/L5 avec IMU Tilt
        let rs4 = ReceiverProfile::EmlidReachRs4Pro.capabilities();
        assert_eq!(rs4.default_freq_mode, FrequencyBandMode::L1L2L5);
        assert!(rs4.supports_imu_tilt);

        // Sokkia / Topcon génère des commandes GREIS/TPS valides
        let sokkia_cmds = ReceiverProfile::SokkiaTopconSeries.build_init_commands(5, true);
        assert!(sokkia_cmds.iter().any(|c| String::from_utf8_lossy(c).contains("/msg/nmea/GGA:0.20")));
        assert!(sokkia_cmds.iter().any(|c| String::from_utf8_lossy(c).contains("/msg/jps/CP:0.20")));
    }
}
