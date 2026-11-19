pub mod gumball;

use std::collections::HashMap;
use std::sync::Arc;

use cad_doc::{AnnotationKind, ObjectId};
use cad_geom::{BoundaryLock, DVec3, Geometry, SubObjectRef};
use cad_render::{DisplayMode, ViewportPaintCallback};
use eframe::egui;

use crate::state::{
    ElephantApp, GripDrag, GumballDrag, GumballMode, GumballNumericPrompt, MarqueeDrag,
    UvnGumballDrag, UvnNumericPrompt,
};
use gumball::{
    compute_gumball_handles, compute_uvn_gumball_handles, draw_gumball_overlay,
    draw_uvn_gumball_overlay, gumball_matrix, hit_gumball, hit_uvn_gumball,
};

pub fn world_to_screen(pt: DVec3, m: &[f32; 16], rect: egui::Rect) -> Option<egui::Pos2> {
    let (x, y, z) = (pt.x as f32, pt.y as f32, pt.z as f32);
    let cx = m[0] * x + m[4] * y + m[8] * z + m[12];
    let cy = m[1] * x + m[5] * y + m[9] * z + m[13];
    let cw = m[3] * x + m[7] * y + m[11] * z + m[15];
    if cw <= 1e-4 {
        return None;
    }
    let ndc_x = cx / cw;
    let ndc_y = cy / cw;
    Some(egui::pos2(
        rect.left() + (ndc_x * 0.5 + 0.5) * rect.width(),
        rect.top() + (0.5 - ndc_y * 0.5) * rect.height(),
    ))
}

pub fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let len_sq = ab.length_sq();
    if len_sq < 1e-6 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

struct CpGrip {
    object_id: ObjectId,
    cp_index: usize,
    screen: egui::Pos2,
    world: DVec3,
}

impl ElephantApp {
    pub fn draw_viewport(&mut self, ui: &mut egui::Ui, idx: usize) {
        let is_active = self.active_viewport == idx;
        let title = self.cameras[idx].kind.title();

        let frame = egui::Frame::none()
            .fill(egui::Color32::from_rgb(38, 41, 46))
            .stroke(egui::Stroke::new(
                if is_active { 1.5_f32 } else { 1.0_f32 },
                if is_active {
                    egui::Color32::from_rgb(80, 150, 235)
                } else {
                    egui::Color32::from_rgb(65, 68, 74)
                },
            ));

        frame.show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let label_res = ui.selectable_label(is_active, title);
                    if label_res.clicked() {
                        self.active_viewport = idx;
                    }
                    if label_res.double_clicked() {
                        self.maximized_viewport = if self.maximized_viewport == Some(idx) {
                            None
                        } else {
                            Some(idx)
                        };
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let mut mode = self.cameras[idx].display_mode;
                        egui::ComboBox::from_id_salt(("vp_mode", idx))
                            .width(86.0)
                            .selected_text(mode.label())
                            .show_ui(ui, |ui| {
                                for m in DisplayMode::ALL {
                                    ui.selectable_value(&mut mode, m, m.label());
                                }
                            });
                        if mode != self.cameras[idx].display_mode {
                            self.active_viewport = idx;
                            self.set_active_viewport_display_mode(mode);
                        }
                    });
                });

                let avail = ui.available_size().max(egui::vec2(16.0, 16.0));
                let (rect, response) = ui.allocate_exact_size(avail, egui::Sense::click_and_drag());

                let aspect = (rect.width() / rect.height().max(1.0)) as f64;
                let world_per_px = self.cameras[idx].world_units_per_pixel(rect.height() as f64);
                let cplane = self.cameras[idx].cplane;
                let vp_cols = self.cameras[idx].view_proj_f32(aspect).to_cols_array();

                let (gumball_origin, gumball_handles) =
                    compute_gumball_handles(self, &vp_cols, rect);
                let uvn_bundle = compute_uvn_gumball_handles(self, &vp_cols, rect);

                if response.hovered() {
                    self.active_viewport = idx;
                    if self.session.is_active() {
                        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
                    }
                    let scroll = ui.input(|i| i.smooth_scroll_delta.y as f64);
                    if scroll.abs() > 0.0 {
                        self.cameras[idx].zoom(scroll);
                    }
                    if let Some(pos) = response.hover_pos() {
                        let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                        let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                        let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);
                        if let Some(raw_pt) = cplane.intersect_ray(&ray) {
                            let snap_world_r = (world_per_px * self.snap.snap_radius_px).max(1e-4);
                            let res = self.snap.resolve_with_radius(
                                raw_pt,
                                self.session.base_point(),
                                &self.doc,
                                &cplane,
                                snap_world_r,
                            );
                            self.cursor_world = Some(res.point);
                            self.last_snap_kind = res.kind;
                        } else {
                            self.cursor_world = None;
                            self.last_snap_kind = None;
                        }
                    }
                }

                if response.dragged_by(egui::PointerButton::Secondary) {
                    self.active_viewport = idx;
                    let delta = response.drag_delta();
                    let shift = ui.input(|i| i.modifiers.shift);
                    if self.cameras[idx].is_perspective && !shift {
                        self.cameras[idx].orbit(delta.x as f64, delta.y as f64);
                    } else {
                        self.cameras[idx].pan(delta.x as f64, delta.y as f64, rect.height() as f64);
                    }
                } else if response.clicked_by(egui::PointerButton::Secondary) {
                    if self.session.is_active() {
                        let outcome = self.session.finish_or_cancel(&mut self.doc);
                        for m in outcome.messages {
                            self.log(m);
                        }
                        self.mark_scene_dirty();
                    } else if let Some(prev) = self.last_command.clone() {
                        self.execute_command(&prev);
                    }
                }

                // Compute NURBS control point grips and nets for PointsOn objects
                let mut cp_grips: Vec<CpGrip> = Vec::new();
                let mut cp_polygons: Vec<Vec<egui::Pos2>> = Vec::new();
                let mut cp_net_segments: Vec<(egui::Pos2, egui::Pos2)> = Vec::new();
                if !self.points_on_objects.is_empty() && !self.session.is_active() {
                    for &oid in &self.points_on_objects {
                        let Some(obj) = self.doc.objects.get(oid) else {
                            continue;
                        };
                        if !obj.visible {
                            continue;
                        }
                        let cps: &[cad_geom::ControlPoint] = match &obj.geometry {
                            Geometry::Curve(c) => &c.control_points,
                            Geometry::Surface(s) => &s.control_points,
                            _ => &[],
                        };
                        let mut poly = Vec::with_capacity(cps.len());
                        for (ci, cp) in cps.iter().enumerate() {
                            if let Some(sp) = world_to_screen(cp.position, &vp_cols, rect) {
                                poly.push(sp);
                                cp_grips.push(CpGrip {
                                    object_id: oid,
                                    cp_index: ci,
                                    screen: sp,
                                    world: cp.position,
                                });
                            }
                        }
                        match &obj.geometry {
                            Geometry::Curve(_) if poly.len() >= 2 => {
                                cp_polygons.push(poly);
                            }
                            Geometry::Surface(s) => {
                                for (a, b) in s.control_net_segments() {
                                    if let (Some(sa), Some(sb)) = (
                                        world_to_screen(a, &vp_cols, rect),
                                        world_to_screen(b, &vp_cols, rect),
                                    ) {
                                        cp_net_segments.push((sa, sb));
                                    }
                                }
                            }
                            _ => {}
                        }
                    }
                }

                // Primary drag: UVN Gumball, Control Points, XYZ Gumball, or Marquee selection (when session is idle)
                if !self.session.is_active() {
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        self.active_viewport = idx;
                        if let Some(pos) = response.interact_pointer_pos() {
                            if let Some((uvn_oid, _, uvn_handles)) = &uvn_bundle {
                                if let Some(h) = hit_uvn_gumball(uvn_handles, pos) {
                                    if let Some(before_snapshot) =
                                        self.doc.objects.get(*uvn_oid).cloned()
                                    {
                                        let cp_weights =
                                            self.compute_cp_weights_for_object(*uvn_oid, None);
                                        let axis_screen_dir = (h.tip - h.p0).normalized();
                                        self.uvn_drag = Some(UvnGumballDrag {
                                            viewport_idx: idx,
                                            object_id: *uvn_oid,
                                            axis: h.axis,
                                            axis_dir: h.world_dir,
                                            axis_screen_dir,
                                            px_per_world: h.px_per_world,
                                            last_pos: pos,
                                            total_amount: 0.0,
                                            moved: false,
                                            cp_weights,
                                            before_snapshot,
                                        });
                                    }
                                }
                            }
                            if self.uvn_drag.is_none() {
                                if let Some(g) =
                                    cp_grips.iter().find(|g| pos.distance(g.screen) <= 9.0)
                                {
                                    if !self.selected_cps.contains(&(g.object_id, g.cp_index)) {
                                        self.selected_cps = vec![(g.object_id, g.cp_index)];
                                    }
                                    if let Some(before_snapshot) =
                                        self.doc.objects.get(g.object_id).cloned()
                                    {
                                        let cp_weights = self.compute_cp_weights_for_object(
                                            g.object_id,
                                            Some(g.cp_index),
                                        );
                                        self.grip_drag = Some(GripDrag {
                                            viewport_idx: idx,
                                            object_id: g.object_id,
                                            start_world: g.world,
                                            cp_weights,
                                            before_snapshot,
                                        });
                                    }
                                } else if let Some((axis, mode, axis_screen_dir, px_per_world)) =
                                    hit_gumball(&gumball_handles, pos)
                                {
                                    self.gumball_drag = Some(GumballDrag {
                                        viewport_idx: idx,
                                        axis,
                                        mode,
                                        origin: gumball_origin,
                                        last_pos: pos,
                                        axis_screen_dir,
                                        px_per_world,
                                        total_amount: 0.0,
                                        moved: false,
                                    });
                                } else {
                                    self.marquee = Some(MarqueeDrag {
                                        viewport_idx: idx,
                                        start: pos,
                                        current: pos,
                                    });
                                }
                            }
                        }
                    }

                    if let Some(mut udrag) = self.uvn_drag.take() {
                        if udrag.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    let d_px =
                                        (pos - udrag.last_pos).dot(udrag.axis_screen_dir) as f64;
                                    udrag.last_pos = pos;
                                    let scale = if udrag.axis == 2 {
                                        self.uvn_normal_drag_strength
                                    } else {
                                        1.0
                                    };
                                    let d_world = (d_px / udrag.px_per_world.max(1e-4)) * scale;
                                    if d_world.abs() > 1e-9 {
                                        udrag.total_amount += d_world;
                                        udrag.moved = true;
                                        self.apply_cp_delta_from_base(
                                            udrag.object_id,
                                            &udrag.before_snapshot,
                                            &udrag.cp_weights,
                                            udrag.axis_dir * udrag.total_amount,
                                        );
                                    }
                                }
                                self.uvn_drag = Some(udrag);
                            } else if udrag.moved && udrag.total_amount.abs() > 1e-9 {
                                self.doc.record_object_modified(
                                    "UVN Move Control Point",
                                    udrag.object_id,
                                    udrag.before_snapshot,
                                );
                                self.mark_scene_dirty();
                            }
                        } else {
                            self.uvn_drag = Some(udrag);
                        }
                    }

                    if let Some(gd) = self.grip_drag.take() {
                        if gd.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    let ndc_x =
                                        ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                                    let ndc_y =
                                        1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                                    let ray = self.cameras[idx].screen_to_ray(
                                        ndc_x as f64,
                                        ndc_y as f64,
                                        aspect,
                                    );
                                    let mut drag_plane = cplane;
                                    drag_plane.origin = gd.start_world;
                                    if let Some(new_pt) = drag_plane.intersect_ray(&ray) {
                                        let total_delta = new_pt - gd.start_world;
                                        self.apply_cp_delta_from_base(
                                            gd.object_id,
                                            &gd.before_snapshot,
                                            &gd.cp_weights,
                                            total_delta,
                                        );
                                    }
                                }
                                self.grip_drag = Some(gd);
                            } else {
                                self.doc.record_object_modified(
                                    "Move Control Point",
                                    gd.object_id,
                                    gd.before_snapshot,
                                );
                                self.mark_scene_dirty();
                            }
                        } else {
                            self.grip_drag = Some(gd);
                        }
                    }

                    if let Some(mut drag) = self.gumball_drag.take() {
                        if drag.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    let delta = pos - drag.last_pos;
                                    drag.last_pos = pos;
                                    let d_amt = match drag.mode {
                                        GumballMode::Move => {
                                            let px = delta.dot(drag.axis_screen_dir) as f64;
                                            px / drag.px_per_world.max(1e-4)
                                        }
                                        GumballMode::Rotate => (delta.x - delta.y) as f64 * 0.01,
                                    };
                                    if d_amt.abs() > 1e-9 {
                                        let mat = gumball_matrix(
                                            drag.axis,
                                            drag.mode,
                                            drag.origin,
                                            d_amt,
                                        );
                                        self.apply_raw_selected_transform(&mat);
                                        drag.total_amount += d_amt;
                                        drag.moved = true;
                                    }
                                }
                                self.gumball_drag = Some(drag);
                            } else if drag.moved && drag.total_amount.abs() > 1e-9 {
                                let revert = gumball_matrix(
                                    drag.axis,
                                    drag.mode,
                                    drag.origin,
                                    -drag.total_amount,
                                );
                                self.apply_raw_selected_transform(&revert);
                                let final_mat = gumball_matrix(
                                    drag.axis,
                                    drag.mode,
                                    drag.origin,
                                    drag.total_amount,
                                );
                                let label = match drag.mode {
                                    GumballMode::Move => "Gumball Move",
                                    GumballMode::Rotate => "Gumball Rotate",
                                };
                                self.doc.transform_selected(label, &final_mat);
                                self.mark_scene_dirty();
                            }
                        } else {
                            self.gumball_drag = Some(drag);
                        }
                    }

                    if let Some(mut m) = self.marquee.take() {
                        if m.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    m.current = pos;
                                }
                                self.marquee = Some(m);
                            } else {
                                let sel_rect = egui::Rect::from_two_pos(m.start, m.current);
                                if sel_rect.width() > 4.0 || sel_rect.height() > 4.0 {
                                    let shift = ui.input(|i| i.modifiers.shift);
                                    let ctrl = ui.input(|i| i.modifiers.ctrl);
                                    let hit_cps: Vec<(ObjectId, usize)> = cp_grips
                                        .iter()
                                        .filter(|g| sel_rect.contains(g.screen))
                                        .map(|g| (g.object_id, g.cp_index))
                                        .collect();
                                    if !cp_grips.is_empty() && !hit_cps.is_empty() {
                                        if !shift && !ctrl {
                                            self.selected_cps.clear();
                                        }
                                        for pair in hit_cps {
                                            if ctrl {
                                                self.selected_cps.retain(|&p| p != pair);
                                            } else if !self.selected_cps.contains(&pair) {
                                                self.selected_cps.push(pair);
                                            }
                                        }
                                    } else {
                                        let crossing = m.current.x < m.start.x;
                                        if !shift && !ctrl {
                                            self.doc.clear_selection();
                                            self.selected_cps.clear();
                                        }
                                        let ids: Vec<_> = self.doc.objects.keys().collect();
                                        for id in ids {
                                            let Some(obj) = self.doc.objects.get(id) else {
                                                continue;
                                            };
                                            let Some(layer) = self.doc.layers.get(obj.layer_id) else {
                                                continue;
                                            };
                                            if !layer.visible || layer.locked {
                                                continue;
                                            }
                                            let min = obj.aabb.min;
                                            let max = obj.aabb.max;
                                            let corners = [
                                                DVec3::new(min.x, min.y, min.z),
                                                DVec3::new(max.x, min.y, min.z),
                                                DVec3::new(min.x, max.y, min.z),
                                                DVec3::new(max.x, max.y, min.z),
                                                DVec3::new(min.x, min.y, max.z),
                                                DVec3::new(max.x, min.y, max.z),
                                                DVec3::new(min.x, max.y, max.z),
                                                DVec3::new(max.x, max.y, max.z),
                                            ];
                                            let mut obj_rect = egui::Rect::NOTHING;
                                            for c in corners {
                                                if let Some(sp) = world_to_screen(c, &vp_cols, rect) {
                                                    obj_rect.extend_with(sp);
                                                }
                                            }
                                            if obj_rect.is_positive() {
                                                let hit = if crossing {
                                                    sel_rect.intersects(obj_rect)
                                                } else {
                                                    sel_rect.contains_rect(obj_rect)
                                                };
                                                if hit {
                                                    self.doc.set_selected(id, !ctrl);
                                                }
                                            }
                                        }
                                        self.mark_doc_dirty();
                                    }
                                }
                            }
                        } else {
                            self.marquee = Some(m);
                        }
                    }
                }

                if response.clicked_by(egui::PointerButton::Primary) {
                    self.active_viewport = idx;
                    if let Some(pos) = response.interact_pointer_pos() {
                        let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                        let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                        let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);

                        if self.session.is_active() {
                            if let Some(raw_pt) = cplane.intersect_ray(&ray) {
                                let snap_world_r =
                                    (world_per_px * self.snap.snap_radius_px).max(1e-4);
                                let res = self.snap.resolve_with_radius(
                                    raw_pt,
                                    self.session.base_point(),
                                    &self.doc,
                                    &cplane,
                                    snap_world_r,
                                );
                                let outcome =
                                    self.session.on_point(res.point, &mut self.doc, &cplane);
                                for m in outcome.messages {
                                    self.log(m);
                                }
                                self.mark_scene_dirty();
                            }
                        } else if let Some((uvn_oid, _, uvn_handles)) = &uvn_bundle {
                            if let Some(h) = hit_uvn_gumball(uvn_handles, pos) {
                                self.uvn_prompt = Some(UvnNumericPrompt {
                                    object_id: *uvn_oid,
                                    axis: h.axis,
                                    axis_dir: h.world_dir,
                                    text: String::new(),
                                    just_opened: true,
                                });
                            } else if let Some(g) =
                                cp_grips.iter().find(|g| pos.distance(g.screen) <= 9.0)
                            {
                                let shift = ui.input(|i| i.modifiers.shift);
                                let pair = (g.object_id, g.cp_index);
                                if shift {
                                    if let Some(pos_idx) =
                                        self.selected_cps.iter().position(|&p| p == pair)
                                    {
                                        self.selected_cps.remove(pos_idx);
                                    } else {
                                        self.selected_cps.push(pair);
                                    }
                                } else {
                                    self.selected_cps = vec![pair];
                                }
                            } else {
                                let shift = ui.input(|i| i.modifiers.shift);
                                let ctrl = ui.input(|i| i.modifiers.ctrl);
                                let pick_tol = (world_per_px * 10.0).max(1e-4);
                                self.pick_viewport_object_or_subobject(&ray, pick_tol, shift, ctrl);
                            }
                        } else if let Some(g) =
                            cp_grips.iter().find(|g| pos.distance(g.screen) <= 9.0)
                        {
                            let shift = ui.input(|i| i.modifiers.shift);
                            let pair = (g.object_id, g.cp_index);
                            if shift {
                                if let Some(pos_idx) =
                                    self.selected_cps.iter().position(|&p| p == pair)
                                {
                                    self.selected_cps.remove(pos_idx);
                                } else {
                                    self.selected_cps.push(pair);
                                }
                            } else {
                                self.selected_cps = vec![pair];
                            }
                        } else if let Some((axis, mode, _, _)) = hit_gumball(&gumball_handles, pos) {
                            self.gumball_prompt = Some(GumballNumericPrompt {
                                axis,
                                mode,
                                text: String::new(),
                                just_opened: true,
                            });
                        } else {
                            let shift = ui.input(|i| i.modifiers.shift);
                            let ctrl = ui.input(|i| i.modifiers.ctrl);
                            let pick_tol = (world_per_px * 10.0).max(1e-4);
                            self.pick_viewport_object_or_subobject(&ray, pick_tol, shift, ctrl);
                        }
                    }
                }

                let mode = self.cameras[idx].display_mode;
                if let (Some(doc_scene), Some(grid_scene)) = (
                    self.cached_doc_scenes.get(&mode),
                    self.cached_grids[idx].as_ref(),
                ) {
                    let cb = eframe::egui_wgpu::Callback::new_paint_callback(
                        rect,
                        ViewportPaintCallback {
                            kind: self.cameras[idx].kind,
                            display_mode: mode,
                            view_proj: self.cameras[idx].view_proj_f32(aspect),
                            doc_scene: Arc::clone(doc_scene),
                            grid_scene: Arc::clone(grid_scene),
                            overlay_scene: self.cached_overlay.as_ref().map(Arc::clone),
                        },
                    );
                    ui.painter().add(cb);
                }

                let painter = ui.painter_at(rect);

                // Draw NURBS control points and control nets (PointsOn)
                if !cp_grips.is_empty() {
                    for poly in &cp_polygons {
                        painter.add(egui::Shape::line(
                            poly.clone(),
                            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(160, 165, 175)),
                        ));
                    }
                    for &(sa, sb) in &cp_net_segments {
                        painter.line_segment(
                            [sa, sb],
                            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(135, 142, 155)),
                        );
                    }
                    let mut weight_cache: HashMap<ObjectId, Vec<f64>> = HashMap::new();
                    if self.soft_transform_enabled && !self.selected_cps.is_empty() {
                        for &oid in &self.points_on_objects {
                            weight_cache
                                .insert(oid, self.compute_cp_weights_for_object(oid, None));
                        }
                    }
                    let lock_rings = match self.soft_transform_lock {
                        BoundaryLock::Free => 0,
                        BoundaryLock::FixG0 => 1,
                        BoundaryLock::FixG1 => 2,
                        BoundaryLock::FixG2 => 3,
                    };
                    for g in &cp_grips {
                        let is_selected = self.selected_cps.contains(&(g.object_id, g.cp_index));
                        let soft_w = weight_cache
                            .get(&g.object_id)
                            .and_then(|ws| ws.get(g.cp_index).copied())
                            .unwrap_or(0.0);
                        let is_locked = self.soft_transform_enabled
                            && lock_rings > 0
                            && self.doc.objects.get(g.object_id).is_some_and(|o| match &o.geometry {
                                Geometry::Surface(s) => {
                                    let ui_cp = g.cp_index / s.count_v.max(1);
                                    let vj_cp = g.cp_index % s.count_v.max(1);
                                    ui_cp < lock_rings
                                        || ui_cp + lock_rings >= s.count_u
                                        || vj_cp < lock_rings
                                        || vj_cp + lock_rings >= s.count_v
                                }
                                Geometry::Curve(c) => {
                                    g.cp_index < lock_rings
                                        || g.cp_index + lock_rings >= c.control_points.len()
                                }
                                _ => false,
                            });

                        let (size, fill, stroke) = if is_selected {
                            (
                                8.0_f32,
                                egui::Color32::from_rgb(255, 220, 50),
                                egui::Stroke::new(1.4_f32, egui::Color32::from_rgb(30, 30, 35)),
                            )
                        } else if self.soft_transform_enabled && is_locked {
                            (
                                5.5_f32,
                                egui::Color32::from_rgb(65, 85, 110),
                                egui::Stroke::new(1.2_f32, egui::Color32::from_rgb(125, 165, 215)),
                            )
                        } else if self.soft_transform_enabled && soft_w > 1e-3 {
                            let t = soft_w.clamp(0.0, 1.0) as f32;
                            let r_c = (75.0 + 180.0 * t) as u8;
                            let g_c = (215.0 - 70.0 * t) as u8;
                            let b_c = (245.0 - 190.0 * t) as u8;
                            (
                                6.5_f32,
                                egui::Color32::from_rgb(r_c, g_c, b_c),
                                egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                            )
                        } else {
                            (
                                6.0_f32,
                                egui::Color32::from_rgb(245, 90, 210),
                                egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                            )
                        };
                        let r = egui::Rect::from_center_size(g.screen, egui::vec2(size, size));
                        painter.rect(r, 0.0, fill, stroke);
                    }
                }

                if let Some((_, _, uvn_handles)) = &uvn_bundle {
                    draw_uvn_gumball_overlay(&painter, uvn_handles);
                } else {
                    draw_gumball_overlay(&painter, &gumball_handles);
                }

                // Draw CurvatureGraph combs for tracked curves
                if !self.curvature_graph_objects.is_empty() {
                    for &oid in &self.curvature_graph_objects {
                        let Some(obj) = self.doc.objects.get(oid) else {
                            continue;
                        };
                        if !obj.visible {
                            continue;
                        }
                        let Geometry::Curve(curve) = &obj.geometry else {
                            continue;
                        };
                        let samples = curve.curvature_samples(72);
                        let max_k = samples
                            .iter()
                            .map(|s| s.curvature)
                            .fold(0.0_f64, f64::max);
                        if max_k <= 1e-10 {
                            continue;
                        }
                        let diag = obj.aabb.size().length().max(1.0);
                        let factor = (diag * 0.18 / max_k) * self.curvature_graph_scale;
                        let mut envelope = Vec::with_capacity(samples.len());
                        for s in &samples {
                            let tip_w = s.point - s.curvature_vec * factor;
                            if let (Some(p_scr), Some(t_scr)) = (
                                world_to_screen(s.point, &vp_cols, rect),
                                world_to_screen(tip_w, &vp_cols, rect),
                            ) {
                                painter.line_segment(
                                    [p_scr, t_scr],
                                    egui::Stroke::new(
                                        1.0_f32,
                                        egui::Color32::from_rgb(245, 115, 95),
                                    ),
                                );
                                envelope.push(t_scr);
                            }
                        }
                        if envelope.len() >= 2 {
                            painter.add(egui::Shape::line(
                                envelope,
                                egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(80, 225, 245)),
                            ));
                        }
                    }
                }

                // Draw Naked Edges overlay when ShowEdges is active
                if self.show_naked_edges {
                    let naked_stroke =
                        egui::Stroke::new(2.5_f32, egui::Color32::from_rgb(255, 60, 210));
                    for (_, obj) in &self.doc.objects {
                        if !obj.visible {
                            continue;
                        }
                        if let Some(layer) = self.doc.layers.get(obj.layer_id) {
                            if !layer.visible {
                                continue;
                            }
                        }
                        if let Geometry::BRep(brep) = &obj.geometry {
                            for ei in brep.naked_edges() {
                                if let Some(edge) = brep.edges.get(ei) {
                                    let screen_pts: Vec<egui::Pos2> = edge
                                        .curve
                                        .sample_polyline(24)
                                        .into_iter()
                                        .filter_map(|p| world_to_screen(p, &vp_cols, rect))
                                        .collect();
                                    if screen_pts.len() >= 2 {
                                        painter.add(egui::Shape::line(screen_pts, naked_stroke));
                                    }
                                }
                            }
                        }
                    }
                }

                // Draw Sub-Object Selection Highlights (Faces orange, Edges cyan, Vertices yellow)
                for &(oid, sub) in &self.selected_subobjects {
                    let Some(obj) = self.doc.objects.get(oid) else {
                        continue;
                    };
                    if !obj.visible {
                        continue;
                    }
                    let Geometry::BRep(brep) = &obj.geometry else {
                        continue;
                    };
                    match sub {
                        SubObjectRef::Face(fi) => {
                            if let Some(face) = brep.faces.get(fi) {
                                let face_stroke = egui::Stroke::new(
                                    2.5_f32,
                                    egui::Color32::from_rgb(255, 165, 40),
                                );
                                for c in face.boundary_curves_3d() {
                                    let pts: Vec<egui::Pos2> = c
                                        .sample_polyline(28)
                                        .into_iter()
                                        .filter_map(|p| world_to_screen(p, &vp_cols, rect))
                                        .collect();
                                    if pts.len() >= 2 {
                                        painter.add(egui::Shape::line(pts, face_stroke));
                                    }
                                }
                                if let Some((_, _, center_pt, _)) = face.interior_sample_point() {
                                    if let Some(sp) = world_to_screen(center_pt, &vp_cols, rect) {
                                        painter.circle_filled(
                                            sp,
                                            4.0,
                                            egui::Color32::from_rgb(255, 165, 40),
                                        );
                                    }
                                }
                            }
                        }
                        SubObjectRef::Edge(ei) => {
                            if let Some(edge) = brep.edges.get(ei) {
                                let edge_stroke = egui::Stroke::new(
                                    3.0_f32,
                                    egui::Color32::from_rgb(60, 235, 255),
                                );
                                let pts: Vec<egui::Pos2> = edge
                                    .curve
                                    .sample_polyline(28)
                                    .into_iter()
                                    .filter_map(|p| world_to_screen(p, &vp_cols, rect))
                                    .collect();
                                if pts.len() >= 2 {
                                    painter.add(egui::Shape::line(pts, edge_stroke));
                                }
                            }
                        }
                        SubObjectRef::Vertex(vi) => {
                            if let Some(v) = brep.vertices.get(vi) {
                                if let Some(sp) = world_to_screen(v.position, &vp_cols, rect) {
                                    painter.circle_filled(
                                        sp,
                                        4.5,
                                        egui::Color32::from_rgb(255, 225, 65),
                                    );
                                }
                            }
                        }
                    }
                }

                // Draw Annotations (Dimensions, Text, TextDots, Leaders)
                for ann in &self.doc.annotations {
                    if let Some(layer) = self.doc.layers.get(ann.layer_id) {
                        if !layer.visible {
                            continue;
                        }
                    }
                    if let Some(sp) = world_to_screen(ann.position, &vp_cols, rect) {
                        match ann.kind {
                            AnnotationKind::TextDot => {
                                let galley = painter.layout_no_wrap(
                                    ann.text.clone(),
                                    egui::FontId::proportional(11.5),
                                    egui::Color32::WHITE,
                                );
                                let pad = egui::vec2(6.0, 3.0);
                                let pill =
                                    egui::Rect::from_center_size(sp, galley.size() + pad * 2.0);
                                painter.rect(
                                    pill,
                                    5.0,
                                    egui::Color32::from_rgb(48, 110, 195),
                                    egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                                );
                                painter.galley(pill.min + pad, galley, egui::Color32::WHITE);
                            }
                            AnnotationKind::Dimension
                            | AnnotationKind::Text
                            | AnnotationKind::Leader => {
                                let galley = painter.layout_no_wrap(
                                    ann.text.clone(),
                                    egui::FontId::proportional(11.5),
                                    egui::Color32::from_rgb(235, 240, 248),
                                );
                                let pad = egui::vec2(3.0, 1.5);
                                let bg =
                                    egui::Rect::from_center_size(sp, galley.size() + pad * 2.0);
                                painter.rect_filled(
                                    bg,
                                    2.0,
                                    egui::Color32::from_rgba_unmultiplied(28, 32, 38, 185),
                                );
                                painter.galley(
                                    bg.min + pad,
                                    galley,
                                    egui::Color32::from_rgb(235, 240, 248),
                                );
                            }
                        }
                    }
                }

                // Draw Marquee selection rectangle
                if let Some(m) = &self.marquee {
                    if m.viewport_idx == idx {
                        let sel_rect = egui::Rect::from_two_pos(m.start, m.current);
                        let crossing = m.current.x < m.start.x;
                        let (fill, stroke_col) = if crossing {
                            (
                                egui::Color32::from_rgba_unmultiplied(80, 200, 110, 35),
                                egui::Color32::from_rgb(90, 220, 120),
                            )
                        } else {
                            (
                                egui::Color32::from_rgba_unmultiplied(70, 145, 240, 35),
                                egui::Color32::from_rgb(90, 165, 250),
                            )
                        };
                        painter.rect(sel_rect, 0.0, fill, egui::Stroke::new(1.5_f32, stroke_col));
                    }
                }
            });
        });
    }

    fn pick_viewport_object_or_subobject(
        &mut self,
        ray: &cad_geom::Ray,
        pick_tol: f64,
        shift: bool,
        ctrl: bool,
    ) {
        if ctrl && shift {
            if let Some(hit_id) = self.doc.pick_ray_with_tolerance(ray, pick_tol) {
                let sub_hit = self.doc.objects.get(hit_id).and_then(|obj| match &obj.geometry {
                    Geometry::BRep(b) => b.pick_subobject(ray, pick_tol),
                    _ => None,
                });
                if let Some((sub, _)) = sub_hit {
                    let key = (hit_id, sub);
                    if let Some(pos_idx) =
                        self.selected_subobjects.iter().position(|&item| item == key)
                    {
                        self.selected_subobjects.remove(pos_idx);
                    } else {
                        self.selected_subobjects.push(key);
                        let label = match sub {
                            SubObjectRef::Face(fi) => format!("Face #{fi}"),
                            SubObjectRef::Edge(ei) => format!("Edge #{ei}"),
                            SubObjectRef::Vertex(vi) => format!("Vertex #{vi}"),
                        };
                        self.log(format!("Sub-selected {label}."));
                    }
                    self.doc.set_selected(hit_id, true);
                } else {
                    self.doc.set_selected(hit_id, true);
                }
                self.mark_doc_dirty();
            }
            return;
        }

        if !shift && !ctrl {
            self.doc.clear_selection();
            self.selected_cps.clear();
            self.selected_subobjects.clear();
        }
        if let Some(hit_id) = self.doc.pick_ray_with_tolerance(ray, pick_tol) {
            if ctrl {
                let cur = self
                    .doc
                    .objects
                    .get(hit_id)
                    .map(|o| o.selected)
                    .unwrap_or(false);
                self.doc.set_selected(hit_id, !cur);
            } else {
                self.doc.set_selected(hit_id, true);
            }
        }
        self.mark_doc_dirty();
    }
}
