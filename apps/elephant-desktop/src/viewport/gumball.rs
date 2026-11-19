use cad_doc::{CadObject, HistoryOp, ObjectId};
use cad_geom::{
    BRep, DMat4, DVec3, Geometry, NurbsSurface, Plane, SubObjectRef, SurfaceFrame,
    apply_weighted_deltas_curve, apply_weighted_deltas_surface, brep_boolean_split,
    project_curve_to_surface, push_pull_face, soft_transform_weights_curve,
    soft_transform_weights_surface, split_face_with_uv_curve,
};
use eframe::egui;

use crate::state::{ElephantApp, GumballAlignMode, GumballMode, UvnNumericPrompt};
use crate::viewport::{dist_point_to_segment, world_to_screen};

pub struct GumballHandle {
    pub axis: usize,
    pub p0: egui::Pos2,
    pub tip: egui::Pos2,
    pub rot_pos: egui::Pos2,
    pub scale_pos: egui::Pos2,
    pub extrude_pos: Option<egui::Pos2>,
    pub cut_pos: Option<egui::Pos2>,
    pub px_per_world: f64,
}

pub struct GumballPlanarHandle {
    pub axis_u: usize,
    pub axis_v: usize,
    pub quad: [egui::Pos2; 4],
    pub u_screen: egui::Vec2,
    pub v_screen: egui::Vec2,
}

#[derive(Default)]
pub struct GumballOverlay {
    pub origin_screen: Option<egui::Pos2>,
    pub handles: Vec<GumballHandle>,
    pub planar: Vec<GumballPlanarHandle>,
}

pub struct HitGumballResult {
    pub axis: usize,
    pub mode: GumballMode,
    pub axis_screen_dir: egui::Vec2,
    pub plane_u_screen: egui::Vec2,
    pub plane_v_screen: egui::Vec2,
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

pub fn gumball_matrix_aligned(
    axes: [DVec3; 3],
    axis: usize,
    mode: GumballMode,
    origin: DVec3,
    amount: f64,
    vec_delta: DVec3,
) -> DMat4 {
    let dir = axes[axis.min(2)].normalize_or(DVec3::X);
    match mode {
        GumballMode::Move => DMat4::from_translation(dir * amount),
        GumballMode::MovePlanar { .. } => DMat4::from_translation(vec_delta),
        GumballMode::Rotate => {
            DMat4::from_translation(origin)
                * DMat4::from_axis_angle(dir, amount)
                * DMat4::from_translation(-origin)
        }
        GumballMode::Scale3D => {
            let s = amount.max(0.01);
            DMat4::from_translation(origin)
                * DMat4::from_scale(DVec3::splat(s))
                * DMat4::from_translation(-origin)
        }
        GumballMode::Scale1D => {
            let s = amount.max(0.01);
            let mut scale_vec = DVec3::ONE;
            match axis.min(2) {
                0 => scale_vec.x = s,
                1 => scale_vec.y = s,
                _ => scale_vec.z = s,
            }
            let x = axes[0].normalize_or(DVec3::X);
            let y = axes[1].normalize_or(DVec3::Y);
            let z = axes[2].normalize_or(DVec3::Z);
            let basis = DMat4::from_cols_array(&[
                x.x, x.y, x.z, 0.0,
                y.x, y.y, y.z, 0.0,
                z.x, z.y, z.z, 0.0,
                0.0, 0.0, 0.0, 1.0,
            ]);
            DMat4::from_translation(origin)
                * basis
                * DMat4::from_scale(scale_vec)
                * basis.transpose()
                * DMat4::from_translation(-origin)
        }
        GumballMode::Extrude | GumballMode::Cut | GumballMode::Relocate => DMat4::IDENTITY,
    }
}

pub fn compute_gumball_frame(app: &ElephantApp, cplane: &Plane) -> Option<(DVec3, [DVec3; 3])> {
    if !app.gumball_enabled
        || app.session.is_active()
        || (app.uvn_gumball_enabled && app.primary_uvn_target().is_some())
    {
        return None;
    }

    let sub_face_center = app.selected_subobjects.iter().find_map(|&(oid, sub)| {
        if let SubObjectRef::Face(fi) = sub {
            if let Some(obj) = app.doc.objects.get(oid) {
                if let Geometry::BRep(b) = &obj.geometry {
                    return b
                        .faces
                        .get(fi)
                        .and_then(|f| f.interior_sample_point())
                        .map(|(_, _, pt, _)| pt);
                }
            }
        }
        None
    });

    let origin = if let Some(custom) = app.gumball_custom_origin {
        if app.doc.selection_aabb().is_some() {
            custom
        } else {
            return None;
        }
    } else if let Some(pt) = sub_face_center {
        pt
    } else {
        app.doc.selection_aabb()?.center()
    };

    let cplane_axes = [
        cplane.x_axis.normalize_or(DVec3::X),
        cplane.y_axis.normalize_or(DVec3::Y),
        cplane.z_axis.normalize_or(DVec3::Z),
    ];
    let axes = match app.gumball_align {
        GumballAlignMode::World => [DVec3::X, DVec3::Y, DVec3::Z],
        GumballAlignMode::CPlane => cplane_axes,
        GumballAlignMode::Object => {
            if let Some(p) = app.selection_object_plane() {
                [p.x_axis, p.y_axis, p.z_axis]
            } else {
                cplane_axes
            }
        }
    };
    Some((origin, axes))
}

pub fn compute_gumball_handles(
    app: &ElephantApp,
    cplane: &Plane,
    vp_cols: &[f32; 16],
    rect: egui::Rect,
) -> (DVec3, [DVec3; 3], GumballOverlay) {
    let default_axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let Some((origin, axes)) = compute_gumball_frame(app, cplane) else {
        return (DVec3::ZERO, default_axes, GumballOverlay::default());
    };
    let Some(p0) = world_to_screen(origin, vp_cols, rect) else {
        return (origin, axes, GumballOverlay::default());
    };

    let mut has_extrudable = app
        .selected_subobjects
        .iter()
        .any(|&(_, s)| matches!(s, SubObjectRef::Face(_)));
    let mut has_curve = false;
    for (_, obj) in &app.doc.objects {
        if obj.selected && obj.visible {
            match &obj.geometry {
                Geometry::Curve(_) => {
                    has_extrudable = true;
                    has_curve = true;
                }
                Geometry::Surface(_) => {
                    has_extrudable = true;
                }
                _ => {}
            }
        }
    }

    let mut px_per_unit = [0.0_f64; 3];
    for (a, dir) in axes.iter().enumerate() {
        if let Some(p1) = world_to_screen(origin + *dir, vp_cols, rect) {
            px_per_unit[a] = (p1 - p0).length() as f64;
        }
    }
    let max_px = px_per_unit.iter().copied().fold(0.0_f64, f64::max).max(1e-4);
    let world_len = 66.0 / max_px;

    let mut overlay = GumballOverlay {
        origin_screen: Some(p0),
        handles: Vec::with_capacity(3),
        planar: Vec::with_capacity(3),
    };

    for (a, dir) in axes.iter().enumerate() {
        if px_per_unit[a] > 1e-4 {
            if let Some(tip) = world_to_screen(origin + *dir * world_len, vp_cols, rect) {
                let len_px = (tip - p0).length();
                if len_px >= 10.0 {
                    let sdir = (tip - p0) / len_px;
                    let rot_pos = p0 + (tip - p0) * 0.50;
                    let extrude_pos = has_extrudable.then_some(p0 + (tip - p0) * 0.76);
                    let scale_pos = tip + sdir * 12.0;
                    let cut_pos = (has_curve && a == 2).then_some(tip + sdir * 23.0);
                    overlay.handles.push(GumballHandle {
                        axis: a,
                        p0,
                        tip,
                        rot_pos,
                        scale_pos,
                        extrude_pos,
                        cut_pos,
                        px_per_world: px_per_unit[a],
                    });
                }
            }
        }
    }

    // 2D Planar Move squares (XY, YZ, XZ)
    for (u_ax, v_ax) in [(0usize, 1usize), (1, 2), (0, 2)] {
        if px_per_unit[u_ax] > 1e-4 && px_per_unit[v_ax] > 1e-4 {
            let f0 = 0.24 * world_len;
            let f1 = 0.44 * world_len;
            let q00 = world_to_screen(origin + axes[u_ax] * f0 + axes[v_ax] * f0, vp_cols, rect);
            let q10 = world_to_screen(origin + axes[u_ax] * f1 + axes[v_ax] * f0, vp_cols, rect);
            let q11 = world_to_screen(origin + axes[u_ax] * f1 + axes[v_ax] * f1, vp_cols, rect);
            let q01 = world_to_screen(origin + axes[u_ax] * f0 + axes[v_ax] * f1, vp_cols, rect);
            if let (Some(a), Some(b), Some(c), Some(d)) = (q00, q10, q11, q01) {
                if a.distance(c) >= 6.0 && b.distance(d) >= 6.0 {
                    let u_scr = (b - a).normalized();
                    let v_scr = (d - a).normalized();
                    overlay.planar.push(GumballPlanarHandle {
                        axis_u: u_ax,
                        axis_v: v_ax,
                        quad: [a, b, c, d],
                        u_screen: u_scr,
                        v_screen: v_scr,
                    });
                }
            }
        }
    }

    (origin, axes, overlay)
}

pub fn hit_gumball(
    overlay: &GumballOverlay,
    pos: egui::Pos2,
    ctrl: bool,
    shift: bool,
) -> Option<HitGumballResult> {
    if ctrl {
        if let Some(op) = overlay.origin_screen {
            if pos.distance(op) <= 10.0 {
                return Some(HitGumballResult {
                    axis: 0,
                    mode: GumballMode::Relocate,
                    axis_screen_dir: egui::vec2(1.0, 0.0),
                    plane_u_screen: egui::Vec2::ZERO,
                    plane_v_screen: egui::Vec2::ZERO,
                    px_per_world: 1.0,
                });
            }
        }
    }

    for ph in &overlay.planar {
        if point_in_quad(pos, &ph.quad) {
            let px_per_world = overlay
                .handles
                .iter()
                .find(|h| h.axis == ph.axis_u)
                .map(|h| h.px_per_world)
                .unwrap_or(40.0);
            return Some(HitGumballResult {
                axis: ph.axis_u,
                mode: GumballMode::MovePlanar {
                    axis_u: ph.axis_u,
                    axis_v: ph.axis_v,
                },
                axis_screen_dir: ph.u_screen,
                plane_u_screen: ph.u_screen,
                plane_v_screen: ph.v_screen,
                px_per_world,
            });
        }
    }

    for h in &overlay.handles {
        if pos.distance(h.scale_pos) <= 8.0 {
            let dir = (h.tip - h.p0).normalized();
            return Some(HitGumballResult {
                axis: h.axis,
                mode: if shift {
                    GumballMode::Scale3D
                } else {
                    GumballMode::Scale1D
                },
                axis_screen_dir: dir,
                plane_u_screen: egui::Vec2::ZERO,
                plane_v_screen: egui::Vec2::ZERO,
                px_per_world: h.px_per_world,
            });
        }
    }

    for h in &overlay.handles {
        if let Some(ep) = h.extrude_pos {
            if pos.distance(ep) <= 7.5 {
                let dir = (h.tip - h.p0).normalized();
                return Some(HitGumballResult {
                    axis: h.axis,
                    mode: GumballMode::Extrude,
                    axis_screen_dir: dir,
                    plane_u_screen: egui::Vec2::ZERO,
                    plane_v_screen: egui::Vec2::ZERO,
                    px_per_world: h.px_per_world,
                });
            }
        }
        if let Some(cp) = h.cut_pos {
            if pos.distance(cp) <= 8.5 {
                let dir = (h.tip - h.p0).normalized();
                return Some(HitGumballResult {
                    axis: h.axis,
                    mode: GumballMode::Cut,
                    axis_screen_dir: dir,
                    plane_u_screen: egui::Vec2::ZERO,
                    plane_v_screen: egui::Vec2::ZERO,
                    px_per_world: h.px_per_world,
                });
            }
        }
    }

    for h in &overlay.handles {
        if pos.distance(h.rot_pos) <= 8.5 {
            let dir = (h.tip - h.p0).normalized();
            return Some(HitGumballResult {
                axis: h.axis,
                mode: GumballMode::Rotate,
                axis_screen_dir: dir,
                plane_u_screen: egui::Vec2::ZERO,
                plane_v_screen: egui::Vec2::ZERO,
                px_per_world: h.px_per_world,
            });
        }
    }

    for h in &overlay.handles {
        if dist_point_to_segment(pos, h.p0, h.tip) <= 7.5 || pos.distance(h.tip) <= 9.5 {
            let dir = (h.tip - h.p0).normalized();
            return Some(HitGumballResult {
                axis: h.axis,
                mode: GumballMode::Move,
                axis_screen_dir: dir,
                plane_u_screen: egui::Vec2::ZERO,
                plane_v_screen: egui::Vec2::ZERO,
                px_per_world: h.px_per_world,
            });
        }
    }
    None
}

pub fn draw_gumball_overlay(painter: &egui::Painter, overlay: &GumballOverlay) {
    if overlay.handles.is_empty() && overlay.planar.is_empty() {
        return;
    }
    let axis_colors = [
        egui::Color32::from_rgb(235, 75, 75),
        egui::Color32::from_rgb(80, 210, 95),
        egui::Color32::from_rgb(75, 145, 245),
    ];

    for ph in &overlay.planar {
        let other = 3usize.saturating_sub(ph.axis_u + ph.axis_v).min(2);
        let c = axis_colors[other];
        let fill = egui::Color32::from_rgba_unmultiplied(c.r(), c.g(), c.b(), 45);
        let stroke = egui::Stroke::new(1.3_f32, c);
        painter.add(egui::Shape::convex_polygon(
            ph.quad.to_vec(),
            fill,
            stroke,
        ));
    }

    for h in &overlay.handles {
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
        painter.circle_stroke(h.rot_pos, 4.6, egui::Stroke::new(2.0_f32, col));
        let sq = egui::Rect::from_center_size(h.scale_pos, egui::vec2(7.5, 7.5));
        painter.rect_filled(sq, 1.0, col);
        if let Some(ep) = h.extrude_pos {
            painter.circle_filled(ep, 4.0, col);
            painter.circle_stroke(ep, 4.0, egui::Stroke::new(1.0_f32, egui::Color32::WHITE));
        }
        if let Some(cp) = h.cut_pos {
            let cut_col = egui::Color32::from_rgb(65, 225, 245);
            let r = 5.5_f32;
            painter.add(egui::Shape::convex_polygon(
                vec![
                    cp + dir * r,
                    cp + perp * (r * 0.7),
                    cp - dir * r,
                    cp - perp * (r * 0.7),
                ],
                cut_col,
                egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
            ));
        }
    }

    if let Some(op) = overlay.origin_screen {
        painter.circle_filled(op, 4.0, egui::Color32::from_rgb(245, 245, 250));
        painter.circle_stroke(op, 4.0, egui::Stroke::new(1.2_f32, egui::Color32::BLACK));
    }
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
        Geometry::SubD(sd) => {
            let pos = *sd.vertices.get(cp_idx)?;
            let c = obj.aabb.center();
            let normal = (pos - c).normalize_or(DVec3::Z);
            let ref_v = if normal.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
            let u_dir = ref_v.cross(normal).normalize_or(DVec3::X);
            let v_dir = normal.cross(u_dir).normalize_or(DVec3::Y);
            SurfaceFrame {
                origin: pos,
                u_dir,
                v_dir,
                normal,
            }
        }
        _ => return None,
    };

    let mut sum = DVec3::ZERO;
    let mut cnt = 0usize;
    for &(id, idx) in &app.selected_cps {
        if id == oid {
            let pt_opt = match &obj.geometry {
                Geometry::Surface(s) => s.control_points.get(idx).map(|cp| cp.position),
                Geometry::Curve(c) => c.control_points.get(idx).map(|cp| cp.position),
                Geometry::SubD(sd) => sd.vertices.get(idx).copied(),
                _ => None,
            };
            if let Some(p) = pt_opt {
                sum += p;
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

pub fn hit_uvn_gumball(handles: &[UvnHandle], pos: egui::Pos2) -> Option<UvnHandle> {
    for &h in handles {
        if dist_point_to_segment(pos, h.p0, h.tip) <= 9.0 || pos.distance(h.tip) <= 11.0 {
            return Some(h);
        }
    }
    None
}

pub fn draw_uvn_gumball_overlay(painter: &egui::Painter, handles: &[UvnHandle]) {
    if handles.is_empty() {
        return;
    }
    let axis_colors = [
        egui::Color32::from_rgb(245, 98, 92),
        egui::Color32::from_rgb(80, 224, 130),
        egui::Color32::from_rgb(72, 198, 250),
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
                    Geometry::SubD(sd) => idx < sd.vertices.len(),
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
                Geometry::SubD(sd) => {
                    let r = self.soft_transform_radius.max(1e-4);
                    let sel_pts: Vec<DVec3> = selected
                        .iter()
                        .filter_map(|&si| sd.vertices.get(si).copied())
                        .collect();
                    sd.vertices
                        .iter()
                        .map(|&v| {
                            let d = sel_pts
                                .iter()
                                .map(|&sp| v.distance(sp))
                                .fold(f64::INFINITY, f64::min);
                            if d >= r {
                                0.0
                            } else {
                                let t = 1.0 - (d / r);
                                t * t * (3.0 - 2.0 * t)
                            }
                        })
                        .collect()
                }
                _ => Vec::new(),
            }
        } else {
            let n = match &obj.geometry {
                Geometry::Surface(s) => s.control_points.len(),
                Geometry::Curve(c) => c.control_points.len(),
                Geometry::SubD(sd) => sd.vertices.len(),
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
            (Geometry::SubD(sd), Geometry::SubD(base)) => {
                for (i, v) in sd.vertices.iter_mut().enumerate() {
                    let w = weights.get(i).copied().unwrap_or(0.0);
                    if w > 0.0 {
                        *v = base.vertices[i] + total_delta * w;
                    }
                }
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

    pub fn execute_gumball_extrude(&mut self, dir: DVec3) {
        if dir.length() < 1e-5 {
            return;
        }
        let tol = self.doc.tolerances.absolute.max(1e-3);
        let mut did_something = false;

        // 1. Sub-selected B-Rep faces -> PushPull direct face modeling
        let sub_faces: Vec<(ObjectId, usize)> = self
            .selected_subobjects
            .iter()
            .filter_map(|&(oid, sub)| match sub {
                SubObjectRef::Face(fi) => Some((oid, fi)),
                _ => None,
            })
            .collect();
        for (oid, fi) in sub_faces {
            if let Some(before) = self.doc.objects.get(oid).cloned() {
                if let Geometry::BRep(b) = &before.geometry {
                    if let Some((_, _, _, n)) = b.faces.get(fi).and_then(|f| f.interior_sample_point()) {
                        let signed = dir.dot(n.normalize_or(DVec3::Z));
                        if signed.abs() > 1e-5 {
                            match push_pull_face(b, fi, signed, tol) {
                                Ok(updated) => {
                                    self.doc.objects[oid].geometry = Geometry::BRep(updated);
                                    self.doc.objects[oid].refresh_aabb();
                                    self.doc.record_object_modified("PushPull", oid, before);
                                    did_something = true;
                                }
                                Err(e) => self.log(format!("PushPull error: {e}")),
                            }
                        }
                    }
                }
            }
        }

        // 2. Selected curves -> NURBS / B-Rep Extrude with Modeling History
        let curves: Vec<_> = self
            .doc
            .selected_ids_ordered()
            .into_iter()
            .filter_map(|id| {
                self.doc.objects.get(id).and_then(|o| match &o.geometry {
                    Geometry::Curve(c) => Some((id, o.layer_id, c.clone())),
                    _ => None,
                })
            })
            .collect();
        for (pid, lid, curve) in curves {
            let is_solid = curve.is_closed();
            if is_solid {
                if let Ok(solid) = BRep::extrude_curve_solid(&curve, dir) {
                    let cid = self
                        .doc
                        .add_object_on_layer("ExtrudeSolid", lid, Geometry::BRep(solid));
                    self.doc.add_history_record(
                        cid,
                        vec![pid],
                        HistoryOp::Extrude {
                            direction: dir,
                            solid: true,
                        },
                    );
                    did_something = true;
                    continue;
                }
            }
            let surf = NurbsSurface::extrude_curve(&curve, dir);
            let cid = self
                .doc
                .add_object_on_layer("Extrude", lid, Geometry::Surface(surf));
            self.doc.add_history_record(
                cid,
                vec![pid],
                HistoryOp::Extrude {
                    direction: dir,
                    solid: false,
                },
            );
            did_something = true;
        }

        if did_something {
            self.mark_scene_dirty();
            self.log(format!("Gumball Extrude ({:.3}).", dir.length()));
        }
    }

    pub fn execute_gumball_cut(&mut self, dir: DVec3) {
        let len = dir.length();
        if len < 1e-5 {
            return;
        }
        let tol = self.doc.tolerances.absolute.max(1e-3);
        let dir_u = dir / len;
        let span = len.max(10.0);
        let curves: Vec<_> = self
            .doc
            .selected_ids_ordered()
            .into_iter()
            .filter_map(|id| {
                self.doc.objects.get(id).and_then(|o| match &o.geometry {
                    Geometry::Curve(c) => Some((id, c.clone())),
                    _ => None,
                })
            })
            .collect();
        if curves.is_empty() {
            return;
        }

        let candidate_breps: Vec<(ObjectId, cad_doc::LayerId)> = self
            .doc
            .objects
            .iter()
            .filter(|(_, o)| {
                o.visible
                    && !o.selected
                    && matches!(o.geometry, Geometry::BRep(_))
                    && self.doc.is_layer_effectively_visible(o.layer_id)
                    && !self.doc.is_layer_effectively_locked(o.layer_id)
            })
            .map(|(id, o)| (id, o.layer_id))
            .collect();

        let mut split_total = 0usize;
        for (_, cutter_curve) in &curves {
            let mut base_c = cutter_curve.clone();
            base_c.transform(&DMat4::from_translation(-dir_u * span));
            if cutter_curve.is_closed() {
                if let Ok(cutter_solid) = BRep::extrude_curve_solid(&base_c, dir_u * (span * 2.0)) {
                    let cutter_bb = cutter_solid.bounding_box().expanded(0.05);
                    for &(tid, lid) in &candidate_breps {
                        let Some(target) = self.doc.objects.get(tid) else {
                            continue;
                        };
                        if !target.aabb.intersects(&cutter_bb) {
                            continue;
                        }
                        if let Geometry::BRep(tb) = &target.geometry {
                            if let Ok(pieces) = brep_boolean_split(tb, &cutter_solid, tol) {
                                if pieces.len() > 1 {
                                    let reps = pieces
                                        .into_iter()
                                        .map(|b| (lid, Geometry::BRep(b)))
                                        .collect();
                                    self.doc.replace_object_with_many("Gumball Cut", tid, reps);
                                    split_total += 1;
                                }
                            }
                        }
                    }
                    continue;
                }
            }
            for &(tid, lid) in &candidate_breps {
                let Some(target) = self.doc.objects.get(tid) else {
                    continue;
                };
                if let Geometry::BRep(tb) = &target.geometry {
                    let orig_n = tb.faces.len();
                    let mut next_faces = Vec::new();
                    for f in &tb.faces {
                        if let Some((_, uv)) =
                            project_curve_to_surface(cutter_curve, &f.surface, dir_u, 32)
                        {
                            next_faces.extend(split_face_with_uv_curve(
                                f,
                                &uv,
                                cutter_curve.is_closed(),
                                tol,
                            ));
                        } else {
                            next_faces.push(f.clone());
                        }
                    }
                    if next_faces.len() > orig_n {
                        let reps = next_faces
                            .into_iter()
                            .map(|f| (lid, Geometry::BRep(BRep::from_faces(vec![f], tol))))
                            .collect();
                        self.doc.replace_object_with_many("Gumball Cut", tid, reps);
                        split_total += 1;
                    }
                }
            }
        }
        if split_total > 0 {
            self.mark_scene_dirty();
            self.log(format!("Gumball Cut split {split_total} solid(s)."));
        } else {
            self.log("Gumball Cut found no intersecting target B-Reps.");
        }
    }

    pub fn draw_gumball_prompt_window(&mut self, ctx: &egui::Context) {
        let Some(mut prompt) = self.gumball_prompt.take() else {
            return;
        };
        let axis_name = ["X", "Y", "Z"][prompt.axis.min(2)];
        let title = match prompt.mode {
            GumballMode::Move => format!("Gumball Move {axis_name}"),
            GumballMode::MovePlanar { axis_u, axis_v } => {
                format!("Gumball Planar Move ({}{})", ["X", "Y", "Z"][axis_u], ["X", "Y", "Z"][axis_v])
            }
            GumballMode::Rotate => format!("Gumball Rotate {axis_name} (deg)"),
            GumballMode::Scale1D => format!("Gumball Scale 1D {axis_name}"),
            GumballMode::Scale3D => "Gumball Scale 3D".to_string(),
            GumballMode::Extrude => format!("Gumball Extrude {axis_name}"),
            GumballMode::Cut => format!("Gumball Cut {axis_name}"),
            GumballMode::Relocate => "Gumball Relocate".to_string(),
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
            let dir = prompt.axes[prompt.axis.min(2)].normalize_or(DVec3::X);
            match prompt.mode {
                GumballMode::Extrude => self.execute_gumball_extrude(dir * v),
                GumballMode::Cut => self.execute_gumball_cut(dir * v),
                _ => {
                    if let Some(aabb) = self.doc.selection_aabb() {
                        let origin = self.gumball_custom_origin.unwrap_or_else(|| aabb.center());
                        let amount = match prompt.mode {
                            GumballMode::Rotate => v.to_radians(),
                            _ => v,
                        };
                        let mat = gumball_matrix_aligned(
                            prompt.axes,
                            prompt.axis,
                            prompt.mode,
                            origin,
                            amount,
                            dir * v,
                        );
                        let label = match prompt.mode {
                            GumballMode::Rotate => "Gumball Rotate",
                            GumballMode::Scale1D => "Gumball Scale 1D",
                            GumballMode::Scale3D => "Gumball Scale 3D",
                            _ => "Gumball Move",
                        };
                        let n = self.doc.transform_selected(label, &mat);
                        if let Some(co) = self.gumball_custom_origin {
                            self.gumball_custom_origin = Some(mat.transform_point3(co));
                        }
                        self.mark_scene_dirty();
                        self.log(format!("{label} {axis_name} by {v} on {n} object(s)."));
                    }
                }
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

fn point_in_quad(p: egui::Pos2, q: &[egui::Pos2; 4]) -> bool {
    let mut sign_pos = 0usize;
    let mut sign_neg = 0usize;
    for i in 0..4 {
        let a = q[i];
        let b = q[(i + 1) % 4];
        let cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        if cross > 1e-3 {
            sign_pos += 1;
        } else if cross < -1e-3 {
            sign_neg += 1;
        }
    }
    sign_pos == 4 || sign_neg == 4
}
