use cad_geom::{DMat4, DVec3};
use eframe::egui;

use crate::state::{ElephantApp, GumballMode};
use crate::viewport::{dist_point_to_segment, world_to_screen};

pub struct GumballHandle {
    pub axis: usize,
    pub p0: egui::Pos2,
    pub tip: egui::Pos2,
    pub rot_pos: egui::Pos2,
    pub px_per_world: f64,
}

pub fn gumball_matrix(axis: usize, mode: GumballMode, origin: DVec3, amount: f64) -> DMat4 {
    let dir = [DVec3::X, DVec3::Y, DVec3::Z][axis];
    match mode {
        GumballMode::Move => DMat4::from_translation(dir * amount),
        GumballMode::Rotate => {
            DMat4::from_translation(origin)
                * DMat4::from_axis_angle(dir, amount)
                * DMat4::from_translation(-origin)
        }
    }
}

pub fn compute_gumball_handles(
    app: &ElephantApp,
    vp_cols: &[f32; 16],
    rect: egui::Rect,
) -> (DVec3, Vec<GumballHandle>) {
    let mut gumball_origin = DVec3::ZERO;
    let mut handles = Vec::new();
    if !app.gumball_enabled || app.session.is_active() {
        return (gumball_origin, handles);
    }
    let Some(aabb) = app.doc.selection_aabb() else {
        return (gumball_origin, handles);
    };
    gumball_origin = aabb.center();
    let Some(p0) = world_to_screen(gumball_origin, vp_cols, rect) else {
        return (gumball_origin, handles);
    };

    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let mut px_per_unit = [0.0_f64; 3];
    for (a, dir) in axes.iter().enumerate() {
        if let Some(p1) = world_to_screen(gumball_origin + *dir, vp_cols, rect) {
            px_per_unit[a] = (p1 - p0).length() as f64;
        }
    }
    let max_px = px_per_unit.iter().copied().fold(0.0_f64, f64::max).max(1e-4);
    let world_len = 64.0 / max_px;
    for (a, dir) in axes.iter().enumerate() {
        if px_per_unit[a] > 1e-4 {
            if let Some(tip) = world_to_screen(gumball_origin + *dir * world_len, vp_cols, rect) {
                let len_px = (tip - p0).length();
                if len_px >= 10.0 {
                    let rot_pos = p0 + (tip - p0) * 0.52;
                    handles.push(GumballHandle {
                        axis: a,
                        p0,
                        tip,
                        rot_pos,
                        px_per_world: px_per_unit[a],
                    });
                }
            }
        }
    }

    (gumball_origin, handles)
}

pub fn hit_gumball(
    handles: &[GumballHandle],
    pos: egui::Pos2,
) -> Option<(usize, GumballMode, egui::Vec2, f64)> {
    for h in handles {
        if pos.distance(h.rot_pos) <= 9.0 {
            let dir = (h.tip - h.p0).normalized();
            return Some((h.axis, GumballMode::Rotate, dir, h.px_per_world));
        }
    }
    for h in handles {
        if dist_point_to_segment(pos, h.p0, h.tip) <= 8.0 || pos.distance(h.tip) <= 10.0 {
            let dir = (h.tip - h.p0).normalized();
            return Some((h.axis, GumballMode::Move, dir, h.px_per_world));
        }
    }
    None
}

pub fn draw_gumball_overlay(painter: &egui::Painter, handles: &[GumballHandle]) {
    if handles.is_empty() {
        return;
    }
    let axis_colors = [
        egui::Color32::from_rgb(235, 75, 75),
        egui::Color32::from_rgb(80, 210, 95),
        egui::Color32::from_rgb(75, 145, 245),
    ];
    for h in handles {
        let col = axis_colors[h.axis];
        painter.line_segment([h.p0, h.tip], egui::Stroke::new(2.2_f32, col));
        let dir = (h.tip - h.p0).normalized();
        let perp = egui::vec2(-dir.y, dir.x);
        let base = h.tip - dir * 10.0;
        painter.add(egui::Shape::convex_polygon(
            vec![h.tip, base + perp * 4.5, base - perp * 4.5],
            col,
            egui::Stroke::NONE,
        ));
        painter.circle_stroke(h.rot_pos, 5.0, egui::Stroke::new(2.0_f32, col));
    }
    painter.circle_filled(handles[0].p0, 3.5, egui::Color32::from_rgb(235, 235, 240));
}

impl ElephantApp {
    pub fn apply_raw_selected_transform(&mut self, mat: &DMat4) {
        for (_, obj) in &mut self.doc.objects {
            if obj.selected {
                obj.geometry.transform(mat);
                obj.refresh_aabb();
            }
        }
        self.doc.rebuild_spatial_index();
        self.mark_scene_dirty();
    }

    pub fn draw_gumball_prompt_window(&mut self, ctx: &egui::Context) {
        let Some(mut prompt) = self.gumball_prompt.take() else {
            return;
        };
        let axis_name = ["X", "Y", "Z"][prompt.axis];
        let title = match prompt.mode {
            GumballMode::Move => format!("Gumball Move {axis_name}"),
            GumballMode::Rotate => format!("Gumball Rotate {axis_name} (deg)"),
        };
        let mut keep = true;
        let mut commit_val: Option<f64> = None;
        egui::Window::new(title)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let res = ui.add(
                        egui::TextEdit::singleline(&mut prompt.text)
                            .desired_width(120.0)
                            .hint_text("Enter value..."),
                    );
                    if prompt.just_opened {
                        res.request_focus();
                        prompt.just_opened = false;
                    }
                    if (res.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                        || ui.button("Apply").clicked()
                    {
                        if let Ok(v) = prompt.text.trim().parse::<f64>() {
                            commit_val = Some(v);
                        }
                        keep = false;
                    }
                    if ui.button("Cancel").clicked() {
                        keep = false;
                    }
                });
            });
        if let Some(v) = commit_val {
            if let Some(aabb) = self.doc.selection_aabb() {
                let origin = aabb.center();
                let amount = match prompt.mode {
                    GumballMode::Move => v,
                    GumballMode::Rotate => v.to_radians(),
                };
                let mat = gumball_matrix(prompt.axis, prompt.mode, origin, amount);
                let label = match prompt.mode {
                    GumballMode::Move => "Gumball Move",
                    GumballMode::Rotate => "Gumball Rotate",
                };
                let n = self.doc.transform_selected(label, &mat);
                self.mark_scene_dirty();
                self.log(format!("{label} {axis_name} by {v} on {n} object(s)."));
            }
        } else if keep {
            self.gumball_prompt = Some(prompt);
        }
    }
}
