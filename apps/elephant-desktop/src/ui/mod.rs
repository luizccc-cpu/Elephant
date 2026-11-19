pub mod inspector;
pub mod layout_view;
pub mod node_canvas;
pub mod radial_menu;
pub mod status_bar;
pub mod tool_palette;
pub mod tools_panel;
pub mod top_bar;

use cad_commands::AxisLockMode;
use eframe::egui;

use crate::state::{ElephantApp, WorkspaceViewMode};

impl eframe::App for ElephantApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if self.radial_menu_pos.is_some() {
                self.radial_menu_pos = None;
            } else if self.uvn_prompt.is_some() {
                self.uvn_prompt = None;
            } else if self.gumball_prompt.is_some() {
                self.gumball_prompt = None;
            } else if self.session.is_active() {
                if self.snap.axis_lock != AxisLockMode::None {
                    self.snap.axis_lock = AxisLockMode::None;
                } else {
                    let msg = self.session.cancel();
                    self.snap.clear_smart_track();
                    self.last_inference_guides.clear();
                    self.smart_track_hover = None;
                    self.log(msg);
                }
            } else if !self.selected_cps.is_empty() {
                self.selected_cps.clear();
            } else if !self.selected_subobjects.is_empty() {
                self.selected_subobjects.clear();
            } else {
                self.doc.clear_selection();
                self.gumball_custom_origin = None;
                self.mark_doc_dirty();
            }
        }
        if !ctx.wants_keyboard_input()
            && (ctx.input(|i| i.key_pressed(egui::Key::Q))
                || (!self.session.is_active() && ctx.input(|i| i.key_pressed(egui::Key::Space))))
        {
            if self.radial_menu_pos.is_some() {
                self.radial_menu_pos = None;
            } else {
                let pos = ctx
                    .input(|i| i.pointer.hover_pos())
                    .unwrap_or_else(|| ctx.screen_rect().center());
                self.radial_menu_pos = Some(pos);
            }
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z)) {
            self.execute_command("Undo");
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Y)) {
            self.execute_command("Redo");
        }
        if !ctx.wants_keyboard_input()
            && ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::A))
        {
            self.execute_command("SelAll");
        }
        if !ctx.wants_keyboard_input() && ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.execute_command("Delete");
        }
        if ctx.input(|i| i.key_pressed(egui::Key::F10)) {
            self.execute_command("PointsOn");
        }
        if ctx.input(|i| i.key_pressed(egui::Key::F11)) {
            self.execute_command("PointsOff");
        }

        self.draw_top_bar(ctx);
        self.draw_inspector(ctx);
        self.draw_status_bar(ctx);
        self.draw_tools_panel(ctx);
        self.draw_tool_palette(ctx);

        egui::TopBottomPanel::bottom("sheet_tabs")
            .exact_height(24.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let is_model = matches!(self.workspace_view, WorkspaceViewMode::Model);
                    if ui.selectable_label(is_model, "Model").clicked() {
                        self.workspace_view = WorkspaceViewMode::Model;
                    }
                    let is_nodes = matches!(self.workspace_view, WorkspaceViewMode::NodeCanvas);
                    if ui.selectable_label(is_nodes, "Node Canvas").clicked() {
                        self.workspace_view = WorkspaceViewMode::NodeCanvas;
                    }
                    let layouts: Vec<(u64, String)> =
                        self.doc.layouts.iter().map(|l| (l.id, l.name.clone())).collect();
                    for (lid, lname) in layouts {
                        let active = self.workspace_view == WorkspaceViewMode::Layout(lid);
                        if ui.selectable_label(active, lname).clicked() {
                            self.workspace_view = WorkspaceViewMode::Layout(lid);
                        }
                    }
                    if ui.small_button("+ Layout").clicked() {
                        let lid = self
                            .doc
                            .add_standard_4view_layout(None, cad_doc::PageSize::A3);
                        self.workspace_view = WorkspaceViewMode::Layout(lid);
                        self.cached_layout_projections.clear();
                    }
                });
            });

        self.ensure_render_caches();

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(egui::Color32::from_rgb(28, 30, 34)))
            .show(ctx, |ui| match self.workspace_view {
                WorkspaceViewMode::Layout(lid) => {
                    self.draw_layout_view(ui, lid);
                }
                WorkspaceViewMode::NodeCanvas => {
                    self.draw_node_canvas(ui);
                }
                WorkspaceViewMode::Model => {
                    if let Some(max_idx) = self.maximized_viewport {
                        self.draw_viewport(ui, max_idx);
                    } else {
                        ui.spacing_mut().item_spacing = egui::vec2(2.0, 2.0);
                        let avail = ui.available_size();
                        let half_w = (avail.x - 2.0).max(32.0) * 0.5;
                        let half_h = (avail.y - 2.0).max(32.0) * 0.5;

                        ui.vertical(|ui| {
                            ui.horizontal(|ui| {
                                ui.allocate_ui_with_layout(
                                    egui::vec2(half_w, half_h),
                                    egui::Layout::top_down(egui::Align::Min),
                                    |ui| self.draw_viewport(ui, 0),
                                );
                                ui.allocate_ui_with_layout(
                                    egui::vec2(half_w, half_h),
                                    egui::Layout::top_down(egui::Align::Min),
                                    |ui| self.draw_viewport(ui, 1),
                                );
                            });
                            ui.horizontal(|ui| {
                                ui.allocate_ui_with_layout(
                                    egui::vec2(half_w, half_h),
                                    egui::Layout::top_down(egui::Align::Min),
                                    |ui| self.draw_viewport(ui, 2),
                                );
                                ui.allocate_ui_with_layout(
                                    egui::vec2(half_w, half_h),
                                    egui::Layout::top_down(egui::Align::Min),
                                    |ui| self.draw_viewport(ui, 3),
                                );
                            });
                        });
                    }
                }
            });

        self.draw_gumball_prompt_window(ctx);
        self.draw_uvn_prompt_window(ctx);
        self.draw_radial_menu(ctx);
    }
}
