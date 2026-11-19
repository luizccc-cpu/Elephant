use cad_geom::UnitSystem;
use eframe::egui;

use crate::state::{ElephantApp, GumballAlignMode};

impl ElephantApp {
    pub fn draw_status_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                let cam = &self.cameras[self.active_viewport];
                ui.label(format!(
                    "{} ({})",
                    cam.kind.title(),
                    cam.display_mode.label()
                ));
                ui.separator();
                ui.menu_button(format!("Units: {}", self.doc.units.label()), |ui| {
                    for u in UnitSystem::ALL {
                        let active = self.doc.units == u;
                        if ui
                            .selectable_label(active, format!("{} ({})", u.full_name(), u.label()))
                            .clicked()
                        {
                            let rescale = self.rescale_on_unit_change;
                            self.change_units(u, rescale);
                            ui.close_menu();
                        }
                    }
                });
                ui.separator();
                if let Some(p) = self.cursor_world {
                    let snap_tag = self
                        .last_snap_kind
                        .map(|k| format!(" [{}]", k.label()))
                        .unwrap_or_default();
                    ui.label(format!(
                        "CP: {:.2}, {:.2}, {:.2}{}",
                        p.x, p.y, p.z, snap_tag
                    ));
                } else {
                    ui.label("CP: --");
                }
                ui.separator();
                ui.checkbox(&mut self.gumball_enabled, "Gumball");
                if self.gumball_enabled
                    && ui
                        .small_button(self.gumball_align.label())
                        .on_hover_text("Cycle Gumball Alignment (World / CPlane / Object)")
                        .clicked()
                {
                    let idx = GumballAlignMode::ALL
                        .iter()
                        .position(|&m| m == self.gumball_align)
                        .unwrap_or(0);
                    self.gumball_align =
                        GumballAlignMode::ALL[(idx + 1) % GumballAlignMode::ALL.len()];
                }
                ui.checkbox(&mut self.snap.grid_snap, "Grid");
                ui.checkbox(&mut self.snap.ortho, "Ortho");
                ui.checkbox(&mut self.snap.smart_track, "SmartTrack");
                if ui.checkbox(&mut self.auto_cplane, "AutoCPlane").changed() {
                    self.update_auto_cplane();
                    self.grids_dirty = true;
                }
                ui.checkbox(&mut self.doc.record_history, "History");
                ui.separator();
                ui.checkbox(&mut self.snap.osnap_enabled, "OSnap");
                if self.snap.osnap_enabled {
                    ui.checkbox(&mut self.snap.osnap_end, "End");
                    ui.checkbox(&mut self.snap.osnap_mid, "Mid");
                    ui.checkbox(&mut self.snap.osnap_center, "Cen");
                    ui.checkbox(&mut self.snap.osnap_int, "Int");
                    ui.menu_button("More...", |ui| {
                        ui.checkbox(&mut self.snap.osnap_quad, "Quad");
                        ui.checkbox(&mut self.snap.osnap_perp, "Perp");
                        ui.checkbox(&mut self.snap.osnap_tan, "Tan");
                        ui.checkbox(&mut self.snap.osnap_knot, "Knot");
                        ui.checkbox(&mut self.snap.osnap_vertex, "Vertex");
                        ui.checkbox(&mut self.snap.osnap_near, "Near");
                    });
                }
                ui.separator();
                let sel_count = self.doc.objects.values().filter(|o| o.selected).count();
                ui.label(format!(
                    "Obj: {} | Sel: {}",
                    self.doc.objects.len(),
                    sel_count
                ));
            });
        });
    }
}
