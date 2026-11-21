//! Point d'entrée natif Android (`libsdnc_field_android.so`) + Pont JNI GNSS.
//!
//! Compile avec `cargo-ndk` vers `arm64-v8a` / `armeabi-v7a` / `x86_64` :
//! `cargo ndk -t arm64-v8a -o android/app/src/main/jniLibs build --release -p sdnc-app-android`

use sdnc_gnss::GnssController;
use std::sync::{Arc, OnceLock};

static ANDROID_GNSS_BRIDGE: OnceLock<Arc<GnssController>> = OnceLock::new();

/// Enregistre ou récupère le contrôleur GNSS partagé entre l'UI et le pont JNI Android.
pub fn shared_gnss_bridge() -> &'static Arc<GnssController> {
    ANDROID_GNSS_BRIDGE.get_or_init(|| Arc::new(GnssController::default()))
}

/// Injecte une mesure GPS Android dans le moteur Rust (utilisable depuis JNI ou tests unitaires).
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
            Ok(Box::new(SdncFieldApp::new(data_dir)))
        }),
    );
}

/// Pont JNI appelé par `MainActivity.kt` (`LocationManager` / FusedLocationProvider).
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
}
