use cad_script::{NodeKind, run_rhai_script};
use eframe::egui;

use crate::state::{ElephantApp, ToolPanelTab};

impl ElephantApp {
    pub fn draw_tools_panel(&mut self, ctx: &egui::Context) {
        if self.tool_tab == ToolPanelTab::None {
            return;
        }
        egui::SidePanel::left("tools_panel")
            .default_width(310.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.selectable_value(
                        &mut self.tool_tab,
                        ToolPanelTab::Parametric,
                        "Parametric Graph",
                    );
                    ui.selectable_value(&mut self.tool_tab, ToolPanelTab::Script, "Rhai Script");
                    if ui.small_button("X").clicked() {
                        self.tool_tab = ToolPanelTab::None;
                    }
                });
                ui.separator();

                match self.tool_tab {
                    ToolPanelTab::Parametric => {
                        ui.checkbox(&mut self.parametric_graph.live_preview, "Live 3D Preview");
                        if ui.button("Bake to Document").clicked() {
                            let n = self.parametric_graph.bake_into(&mut self.doc);
                            self.mark_scene_dirty();
                            self.log(format!("Baked {n} parametric object(s)."));
                        }
                        ui.separator();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for node in &mut self.parametric_graph.nodes {
                                ui.group(|ui| {
                                    ui.strong(&node.label);
                                    match &mut node.kind {
                                        NodeKind::NumberSlider { value, min, max } => {
                                            ui.add(
                                                egui::Slider::new(value, *min..=*max).text("Value"),
                                            );
                                        }
                                        NodeKind::LinearArray { step_x, step_y, .. } => {
                                            ui.add(
                                                egui::Slider::new(step_x, 0.5..=12.0)
                                                    .text("Step X"),
                                            );
                                            ui.add(
                                                egui::Slider::new(step_y, -12.0..=12.0)
                                                    .text("Step Y"),
                                            );
                                        }
                                        _ => {
                                            ui.weak("Connected node");
                                        }
                                    }
                                });
                            }
                        });
                    }
                    ToolPanelTab::Script => {
                        if ui.button("Run Script").clicked() {
                            match run_rhai_script(&self.script_source, &mut self.doc) {
                                Ok(res) => {
                                    self.mark_scene_dirty();
                                    for l in res.logs {
                                        self.log(format!("[Script] {l}"));
                                    }
                                    self.log(format!(
                                        "Script created {} object(s).",
                                        res.created_count
                                    ));
                                }
                                Err(e) => self.log(format!("Script error: {e}")),
                            }
                        }
                        ui.separator();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            ui.add(
                                egui::TextEdit::multiline(&mut self.script_source)
                                    .code_editor()
                                    .desired_rows(18)
                                    .desired_width(f32::INFINITY),
                            );
                        });
                    }
                    ToolPanelTab::None => {}
                }
            });
    }
}
