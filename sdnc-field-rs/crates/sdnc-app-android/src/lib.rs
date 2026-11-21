//! Point d'entrée natif Android (`libsdnc_field_android.so`) + Ponts JNI GNSS interne & Bluetooth/USB-Série.
//!
//! Supporte :
//! - GPS interne Android (`LocationManager` -> `nativeOnLocationUpdate`)
//! - Récepteurs externes **u-blox ZED-F9P / Emlid Reach** via Bluetooth RFCOMM SPP ou USB-OTG Série
//!   (`nativeOnExternalGnssBytes` -> décodage UBX `NAV-PVT`/`RXM-RAWX` + NMEA + enregistrement RINEX PPK)
//! - Flux corrections différentielles **RTCM 3.3 MSM4/MSM7** (`nativeOnRtcmBytes`)
//!
//! Compilation avec `cargo-ndk` :
//! `cargo ndk -t arm64-v8a -o android/app/src/main/jniLibs build --release -p sdnc-app-android`

use sdnc_gnss::GnssController;
use std::sync::{Arc, OnceLock};

static ANDROID_GNSS_BRIDGE: OnceLock<Arc<GnssController>> = OnceLock::new();

/// Enregistre ou récupère le contrôleur GNSS partagé entre l'UI et les ponts JNI Android.
pub fn shared_gnss_bridge() -> &'static Arc<GnssController> {
    ANDROID_GNSS_BRIDGE.get_or_init(|| Arc::new(GnssController::default()))
}

/// Injecte une mesure GPS interne Android dans le moteur Rust.
pub fn push_android_location_sample(
    lat_deg: f64,
    lon_deg: f64,
    alt_ellipsoid_m: f64,
    accuracy_m: f32,
    num_sats: u8,
) {
    shared_gnss_bridge().inject_android_location(
        lat_deg,
        lon_deg,
        alt_ellipsoid_m,
        accuracy_m,
        num_sats,
    );
}

/// Injecte des octets bruts UBX / NMEA reçus par Bluetooth SPP ou USB-Série Android (u-blox / Emlid Reach).
pub fn push_android_external_gnss_bytes(bytes: &[u8], transport_label: &str) {
    shared_gnss_bridge().inject_external_raw_bytes(bytes, transport_label);
}

/// Injecte des octets RTCM 3.3 (NTRIP / radio / base Bluetooth) dans le décodeur MSM4/MSM7.
pub fn push_android_rtcm_bytes(bytes: &[u8]) {
    shared_gnss_bridge().inject_rtcm_bytes(bytes);
}

#[cfg(target_os = "android")]
#[no_mangle]
pub fn android_main(app: winit::platform::android::activity::AndroidApp) {
    use sdnc_ui::SdncFieldApp;

    android_logger::init_once(
        android_logger::Config::default().with_max_level(log::LevelFilter::Info),
    );
    log::info!("Démarrage SDNCQField-RS Android (Vulkan / WGPU)");

    let data_dir = app
        .internal_data_path()
        .unwrap_or_else(|| std::path::PathBuf::from("/data/local/tmp/sdnc_field"));

    let mut options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    options.android_app = Some(app);

    let _ = eframe::run_native(
        "SDNCQField-RS",
        options,
        Box::new(move |cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let target_fmt = cc.wgpu_render_state.as_ref().map(|rs| rs.target_format);
            Ok(Box::new(SdncFieldApp::new_with_wgpu_and_gnss(
                data_dir,
                target_fmt,
                Arc::clone(shared_gnss_bridge()),
            )))
        }),
    );
}

/// Pont JNI appelé par `MainActivity.kt` (`LocationManager`).
#[cfg(target_os = "android")]
#[no_mangle]
pub extern "system" fn Java_fr_sdnc_qfield_MainActivity_nativeOnLocationUpdate(
    _env: jni::JNIEnv,
    _class: jni::objects::JClass,
    lat: jni::sys::jdouble,
    lon: jni::sys::jdouble,
    alt_ellipsoid: jni::sys::jdouble,
    accuracy_m: jni::sys::jfloat,
    num_sats: jni::sys::jint,
) {
    push_android_location_sample(
        lat,
        lon,
        alt_ellipsoid,
        accuracy_m,
        num_sats.clamp(0, 255) as u8,
    );
}

/// Pont JNI appelé par `MainActivity.kt` lorsqu'un récepteur externe Bluetooth SPP ou USB-Série
/// (u-blox ZED-F9P, Emlid Reach RS2+/RS3) envoie un paquet brut UBX/NMEA.
#[cfg(target_os = "android")]
#[no_mangle]
pub extern "system" fn Java_fr_sdnc_qfield_MainActivity_nativeOnExternalGnssBytes(
    env: jni::JNIEnv,
    _class: jni::objects::JClass,
    data: jni::objects::JByteArray,
) {
    if let Ok(bytes) = env.convert_byte_array(data) {
        push_android_external_gnss_bytes(&bytes, "Android Bluetooth/USB GNSS");
    }
}

/// Pont JNI pour injecter des corrections RTCM 3.3 brutes depuis une liaison radio/Bluetooth Android.
#[cfg(target_os = "android")]
#[no_mangle]
pub extern "system" fn Java_fr_sdnc_qfield_MainActivity_nativeOnRtcmBytes(
    env: jni::JNIEnv,
    _class: jni::objects::JClass,
    data: jni::objects::JByteArray,
) {
    if let Ok(bytes) = env.convert_byte_array(data) {
        push_android_rtcm_bytes(&bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_android_gnss_bridge_injection() {
        push_android_location_sample(43.5297, 5.4474, 252.0, 1.8, 14);
        let snap = shared_gnss_bridge().snapshot();
        assert!(snap.connected);
        assert_eq!(snap.solution.num_sats, 14);
        assert!((snap.solution.lat_deg - 43.5297).abs() < 1e-6);
    }

    #[test]
    fn test_android_external_bluetooth_bytes_bridge() {
        let gga = sdnc_gnss::nmea::format_gga_sentence(43.5501, 5.4902, 260.0);
        push_android_external_gnss_bytes(gga.as_bytes(), "Bluetooth Emlid Reach");
        let snap = shared_gnss_bridge().snapshot();
        assert!(snap.connected);
        assert!(snap.raw_bytes_rx > 0);
        assert!((snap.solution.lat_deg - 43.5501).abs() < 1e-4);
    }
}
