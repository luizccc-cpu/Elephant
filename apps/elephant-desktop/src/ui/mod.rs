pub mod inspector;
pub mod status_bar;
pub mod tools_panel;
pub mod top_bar;

use eframe::egui;

use crate::state::ElephantApp;

impl eframe::App for ElephantApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if self.gumball_prompt.is_some() {
                self.gumball_prompt = None;
            } else if self.session.is_active() {
                let msg = self.session.cancel();
                self.log(msg);
            } else {
                self.doc.clear_selection();
                self.mark_doc_dirty();
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

        self.ensure_render_caches();

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(egui::Color32::from_rgb(28, 30, 34)))
            .show(ctx, |ui| {
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
            });

        self.draw_gumball_prompt_window(ctx);
    }
}
