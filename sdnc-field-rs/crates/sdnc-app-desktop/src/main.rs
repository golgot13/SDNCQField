#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use sdnc_ui::SdncFieldApp;
use std::path::PathBuf;

fn main() -> eframe::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("Démarrage SDNCQField-RS Desktop (Vulkan / WGPU)");

    let project_dir = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            std::env::current_dir()
                .unwrap_or_else(|_| PathBuf::from("."))
                .join("sdnc_workspace")
        });

    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_title("SDNCQField-RS — Carto 2D Vulkan, Cadastre DXF/EDIGÉO, IGN & GNSS RTK/PPK")
            .with_inner_size([1360.0, 840.0])
            .with_min_inner_size([800.0, 560.0]),
        ..Default::default()
    };

    eframe::run_native(
        "SDNCQField-RS",
        options,
        Box::new(move |cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let target_fmt = cc.wgpu_render_state.as_ref().map(|rs| rs.target_format);
            Ok(Box::new(SdncFieldApp::new_with_wgpu(project_dir, target_fmt)))
        }),
    )
}
