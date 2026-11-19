use cad_script::{
    NodeKind, SCRIPT_PRESETS, dispatch_mcp_json, mcp_tools_manifest_json, run_rhai_script,
};
use eframe::egui;

use crate::state::{ElephantApp, ToolPanelTab, WorkspaceViewMode};

impl ElephantApp {
    pub fn draw_tools_panel(&mut self, ctx: &egui::Context) {
        if self.tool_tab == ToolPanelTab::None {
            return;
        }
        egui::SidePanel::left("tools_panel")
            .default_width(330.0)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(
                        &mut self.tool_tab,
                        ToolPanelTab::Parametric,
                        "Parametric",
                    );
                    ui.selectable_value(&mut self.tool_tab, ToolPanelTab::Script, "Rhai Script");
                    ui.selectable_value(&mut self.tool_tab, ToolPanelTab::Mcp, "AI / MCP");
                    if ui.small_button("X").clicked() {
                        self.tool_tab = ToolPanelTab::None;
                    }
                });
                ui.separator();

                match self.tool_tab {
                    ToolPanelTab::Parametric => {
                        if ui.button("Open 2D Node Canvas ->").clicked() {
                            self.workspace_view = WorkspaceViewMode::NodeCanvas;
                        }
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
                                        NodeKind::ShrinkWrap {
                                            voxel_size,
                                            offset,
                                            smooth,
                                        } => {
                                            ui.add(
                                                egui::Slider::new(voxel_size, 0.08..=2.0)
                                                    .text("Voxel"),
                                            );
                                            ui.add(
                                                egui::Slider::new(offset, -1.0..=2.0).text("Offset"),
                                            );
                                            ui.add(egui::Slider::new(smooth, 0..=12).text("Smooth"));
                                        }
                                        _ => {
                                            ui.weak("Edit full graph in Node Canvas tab");
                                        }
                                    }
                                });
                            }
                        });
                    }
                    ToolPanelTab::Script => {
                        ui.horizontal_wrapped(|ui| {
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
                            ui.menu_button("Presets", |ui| {
                                for &(title, src) in SCRIPT_PRESETS {
                                    if ui.button(title).clicked() {
                                        self.script_source = src.to_string();
                                        ui.close_menu();
                                    }
                                }
                            });
                        });
                        ui.collapsing("Rhai CAD API Reference", |ui| {
                            ui.small(
                                "add_box, add_nurbs_box, add_sphere, add_nurbs_sphere, add_circle, \
                                 add_line, add_helix, add_interp_curve, add_cylinder, add_nurbs_cylinder, \
                                 add_subd_box, add_subd_sphere, subd_to_nurbs_last, extrude_last, \
                                 loft_last_n, revolve_last, boolean_union_last, boolean_diff_last, \
                                 shrinkwrap_all, clear_doc, log",
                            );
                        });
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
                    ToolPanelTab::Mcp => {
                        ui.small("Local JSON-RPC 2.0 / MCP CAD Tool Dispatcher (100% offline)");
                        ui.horizontal_wrapped(|ui| {
                            if ui.small_button("tools/list").clicked() {
                                self.mcp_request_input = r#"{"jsonrpc":"2.0","id":1,"method":"tools/list"}"#.to_string();
                            }
                            if ui.small_button("inspect_doc").clicked() {
                                self.mcp_request_input = r#"{"tool":"inspect_document","arguments":{}}"#.to_string();
                            }
                            if ui.small_button("subd_box").clicked() {
                                self.mcp_request_input = r#"{"tool":"create_subd_box","arguments":{"cx":0,"cy":0,"cz":1,"sx":3,"sy":2,"sz":2,"level":2}}"#.to_string();
                            }
                            if ui.small_button("to_nurbs").clicked() {
                                self.mcp_request_input = r#"{"tool":"subd_to_nurbs","arguments":{}}"#.to_string();
                            }
                            if ui.small_button("shrinkwrap").clicked() {
                                self.mcp_request_input = r#"{"tool":"shrinkwrap","arguments":{"voxel_size":0.25,"offset":0.05,"smoothing_iterations":4}}"#.to_string();
                            }
                            if ui.small_button("mass_props").clicked() {
                                self.mcp_request_input = r#"{"tool":"compute_mass_properties","arguments":{}}"#.to_string();
                            }
                        });
                        ui.horizontal(|ui| {
                            if ui.button("Dispatch JSON-RPC").clicked() {
                                let out = dispatch_mcp_json(&self.mcp_request_input, &mut self.doc);
                                self.mcp_response_log = out.response_json;
                                if out.mutated_document {
                                    self.mark_scene_dirty();
                                }
                                self.log("MCP JSON-RPC request dispatched.");
                            }
                            if ui.small_button("Manifest").clicked() {
                                self.mcp_response_log = mcp_tools_manifest_json();
                            }
                        });
                        ui.separator();
                        ui.label("Request JSON:");
                        ui.add(
                            egui::TextEdit::multiline(&mut self.mcp_request_input)
                                .code_editor()
                                .desired_rows(7)
                                .desired_width(f32::INFINITY),
                        );
                        ui.label("Response:");
                        egui::ScrollArea::vertical().max_height(260.0).show(ui, |ui| {
                            ui.add(
                                egui::TextEdit::multiline(&mut self.mcp_response_log)
                                    .code_editor()
                                    .desired_rows(10)
                                    .desired_width(f32::INFINITY),
                            );
                        });
                    }
                    ToolPanelTab::None => {}
                }
            });
    }
}
