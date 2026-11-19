#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod commands;
mod state;
mod ui;
mod viewport;

use cad_render::CadWgpuState;
use eframe::egui;
use state::ElephantApp;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Elephant CAD")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 540.0]),
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 32,
        ..Default::default()
    };

    eframe::run_native(
        "Elephant CAD",
        options,
        Box::new(|cc| {
            if let Some(wgpu_state) = &cc.wgpu_render_state {
                let cad_state = CadWgpuState::new(
                    &wgpu_state.device,
                    wgpu_state.target_format,
                    Some(eframe::egui_wgpu::wgpu::TextureFormat::Depth32Float),
                );
                wgpu_state
                    .renderer
                    .write()
                    .callback_resources
                    .insert(cad_state);
            }
            Ok(Box::new(ElephantApp::new()))
        }),
    )
}
