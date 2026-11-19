use cad_doc::{CadObject, ObjectId};
use cad_geom::{
    DMat4, DVec3, Geometry, SurfaceFrame, apply_weighted_deltas_curve,
    apply_weighted_deltas_surface, soft_transform_weights_curve, soft_transform_weights_surface,
};
use eframe::egui;

use crate::state::{ElephantApp, GumballMode, UvnNumericPrompt};
use crate::viewport::{dist_point_to_segment, world_to_screen};

pub struct GumballHandle {
    pub axis: usize,
    pub p0: egui::Pos2,
    pub tip: egui::Pos2,
    pub rot_pos: egui::Pos2,
    pub px_per_world: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct UvnHandle {
    pub axis: usize,
    pub label: &'static str,
    pub p0: egui::Pos2,
    pub tip: egui::Pos2,
    pub world_dir: DVec3,
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
    if !app.gumball_enabled
        || app.session.is_active()
        || (app.uvn_gumball_enabled && app.primary_uvn_target().is_some())
    {
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

pub fn compute_uvn_gumball_handles(
    app: &ElephantApp,
    vp_cols: &[f32; 16],
    rect: egui::Rect,
) -> Option<(ObjectId, SurfaceFrame, Vec<UvnHandle>)> {
    if !app.uvn_gumball_enabled || app.session.is_active() {
        return None;
    }
    let (oid, cp_idx) = app.primary_uvn_target()?;
    let obj = app.doc.objects.get(oid)?;
    let mut frame = match &obj.geometry {
        Geometry::Surface(s) => {
            let ui = cp_idx / s.count_v.max(1);
            let vj = cp_idx % s.count_v.max(1);
            s.control_point_uvn_frame(ui, vj)
        }
        Geometry::Curve(c) => {
            let p = c.degree;
            let mut u_sum = 0.0;
            for j in 1..=p {
                u_sum += c.knots[cp_idx + j];
            }
            let ug = u_sum / p.max(1) as f64;
            let (_, t, n, b) = c.frenet_frame(ug);
            SurfaceFrame {
                origin: c.control_points[cp_idx].position,
                u_dir: t,
                v_dir: b,
                normal: n,
            }
        }
        _ => return None,
    };

    // Average origin if multiple CPs on this object are selected
    let cps: &[cad_geom::ControlPoint] = match &obj.geometry {
        Geometry::Surface(s) => &s.control_points,
        Geometry::Curve(c) => &c.control_points,
        _ => return None,
    };
    let mut sum = DVec3::ZERO;
    let mut cnt = 0usize;
    for &(id, idx) in &app.selected_cps {
        if id == oid {
            if let Some(cp) = cps.get(idx) {
                sum += cp.position;
                cnt += 1;
            }
        }
    }
    if cnt > 0 {
        frame.origin = sum / cnt as f64;
    }

    let p0 = world_to_screen(frame.origin, vp_cols, rect)?;
    let axes = [(0, "U", frame.u_dir), (1, "V", frame.v_dir), (2, "N", frame.normal)];
    let mut px_per_unit = [0.0_f64; 3];
    for &(idx, _, dir) in &axes {
        if let Some(p1) = world_to_screen(frame.origin + dir, vp_cols, rect) {
            px_per_unit[idx] = (p1 - p0).length() as f64;
        }
    }
    let max_px = px_per_unit.iter().copied().fold(0.0_f64, f64::max).max(1e-4);
    let world_len = 66.0 / max_px;
    let mut handles = Vec::with_capacity(3);
    for (idx, label, dir) in axes {
        if px_per_unit[idx] > 1e-4 {
            if let Some(tip) = world_to_screen(frame.origin + dir * world_len, vp_cols, rect) {
                if (tip - p0).length() >= 8.0 {
                    handles.push(UvnHandle {
                        axis: idx,
                        label,
                        p0,
                        tip,
                        world_dir: dir,
                        px_per_world: px_per_unit[idx],
                    });
                }
            }
        }
    }
    Some((oid, frame, handles))
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

pub fn hit_uvn_gumball(handles: &[UvnHandle], pos: egui::Pos2) -> Option<UvnHandle> {
    for &h in handles {
        if dist_point_to_segment(pos, h.p0, h.tip) <= 9.0 || pos.distance(h.tip) <= 11.0 {
            return Some(h);
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

pub fn draw_uvn_gumball_overlay(painter: &egui::Painter, handles: &[UvnHandle]) {
    if handles.is_empty() {
        return;
    }
    let axis_colors = [
        egui::Color32::from_rgb(245, 98, 92),  // U: coral-red
        egui::Color32::from_rgb(80, 224, 130), // V: emerald
        egui::Color32::from_rgb(72, 198, 250), // N: sky-cyan
    ];
    for h in handles {
        let col = axis_colors[h.axis];
        painter.line_segment([h.p0, h.tip], egui::Stroke::new(2.4_f32, col));
        let dir = (h.tip - h.p0).normalized();
        let perp = egui::vec2(-dir.y, dir.x);
        let base = h.tip - dir * 10.0;
        painter.add(egui::Shape::convex_polygon(
            vec![h.tip, base + perp * 4.5, base - perp * 4.5],
            col,
            egui::Stroke::NONE,
        ));
        painter.text(
            h.tip + dir * 9.0,
            egui::Align2::CENTER_CENTER,
            h.label,
            egui::FontId::proportional(11.0),
            col,
        );
    }
    painter.circle_filled(handles[0].p0, 4.0, egui::Color32::from_rgb(255, 215, 70));
}

impl ElephantApp {
    pub fn primary_uvn_target(&self) -> Option<(ObjectId, usize)> {
        self.selected_cps.iter().copied().find(|&(oid, idx)| {
            self.points_on_objects.contains(&oid)
                && self.doc.objects.get(oid).is_some_and(|o| match &o.geometry {
                    Geometry::Curve(c) => idx < c.control_points.len(),
                    Geometry::Surface(s) => idx < s.control_points.len(),
                    _ => false,
                })
        })
    }

    pub fn compute_cp_weights_for_object(&self, oid: ObjectId, extra_idx: Option<usize>) -> Vec<f64> {
        let Some(obj) = self.doc.objects.get(oid) else {
            return Vec::new();
        };
        let mut selected: Vec<usize> = self
            .selected_cps
            .iter()
            .filter_map(|&(id, idx)| (id == oid).then_some(idx))
            .collect();
        if let Some(ex) = extra_idx {
            if !selected.contains(&ex) {
                selected.push(ex);
            }
        }
        if self.soft_transform_enabled {
            match &obj.geometry {
                Geometry::Surface(s) => soft_transform_weights_surface(
                    s,
                    &selected,
                    self.soft_transform_radius,
                    self.soft_transform_falloff,
                    self.soft_transform_lock,
                ),
                Geometry::Curve(c) => soft_transform_weights_curve(
                    c,
                    &selected,
                    self.soft_transform_radius,
                    self.soft_transform_falloff,
                    self.soft_transform_lock,
                ),
                _ => Vec::new(),
            }
        } else {
            let n = match &obj.geometry {
                Geometry::Surface(s) => s.control_points.len(),
                Geometry::Curve(c) => c.control_points.len(),
                _ => 0,
            };
            let mut weights = vec![0.0_f64; n];
            for idx in selected {
                if let Some(w) = weights.get_mut(idx) {
                    *w = 1.0;
                }
            }
            weights
        }
    }

    pub fn apply_cp_delta_from_base(
        &mut self,
        oid: ObjectId,
        before: &CadObject,
        weights: &[f64],
        total_delta: DVec3,
    ) {
        let Some(obj) = self.doc.objects.get_mut(oid) else {
            return;
        };
        match (&mut obj.geometry, &before.geometry) {
            (Geometry::Surface(s), Geometry::Surface(base)) => {
                apply_weighted_deltas_surface(s, base, weights, total_delta);
            }
            (Geometry::Curve(c), Geometry::Curve(base)) => {
                apply_weighted_deltas_curve(c, base, weights, total_delta);
            }
            _ => {}
        }
        obj.refresh_aabb();
        self.doc.rebuild_spatial_index();
        self.mark_scene_dirty();
    }

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

    pub fn draw_uvn_prompt_window(&mut self, ctx: &egui::Context) {
        let Some(mut prompt) = self.uvn_prompt.take() else {
            return;
        };
        let axis_name = ["U", "V", "N"][prompt.axis.min(2)];
        let mut keep = true;
        let mut commit_val: Option<f64> = None;
        egui::Window::new(format!("UVN Move {axis_name}"))
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    let res = ui.add(
                        egui::TextEdit::singleline(&mut prompt.text)
                            .desired_width(120.0)
                            .hint_text("Offset..."),
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
            if v.abs() > 1e-9 {
                if let Some(before) = self.doc.objects.get(prompt.object_id).cloned() {
                    let weights = self.compute_cp_weights_for_object(prompt.object_id, None);
                    self.apply_cp_delta_from_base(
                        prompt.object_id,
                        &before,
                        &weights,
                        prompt.axis_dir * v,
                    );
                    self.doc.record_object_modified(
                        "UVN Move Control Point",
                        prompt.object_id,
                        before,
                    );
                    self.log(format!("UVN Moved {axis_name} by {v:.4}."));
                }
            }
        } else if keep {
            self.uvn_prompt = Some(UvnNumericPrompt { ..prompt });
        }
    }
}
