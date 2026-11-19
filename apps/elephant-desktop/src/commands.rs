use cad_doc::{AnnotationItem, AnnotationKind, Document};
use cad_geom::{
    BooleanOp, Continuity, DMat4, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh,
    UnitSystem, blend_curves, chamfer_two_curves, fair_curve, fillet_two_curves,
    intersect_curves, match_curve, mesh_boolean, rebuild_adaptive, rebuild_uniform, tween_curves,
};
use cad_render::DisplayMode;

use crate::state::ElephantApp;

impl ElephantApp {
    pub fn execute_command(&mut self, raw: &str) {
        let trimmed = raw.trim();
        let cplane = self.cameras[self.active_viewport].cplane;

        if self.session.is_active() {
            if !trimmed.is_empty() {
                self.log(format!("> {trimmed}"));
            }
            if let Some(outcome) = self.session.on_text_input(trimmed, &mut self.doc, &cplane) {
                for m in outcome.messages {
                    self.log(m);
                }
                self.mark_scene_dirty();
                return;
            }
        }

        if trimmed.is_empty() {
            if let Some(prev) = self.last_command.clone() {
                self.execute_command(&prev);
            }
            return;
        }
        self.history_cursor = None;
        if self.command_history.last().map(|s| s.as_str()) != Some(trimmed) {
            self.command_history.push(trimmed.to_string());
            if self.command_history.len() > 100 {
                self.command_history.remove(0);
            }
        }
        self.last_command = Some(trimmed.to_string());
        self.log(format!("> {trimmed}"));
        let mut parts = trimmed.split_whitespace();
        let cmd = parts.next().unwrap_or("").to_ascii_lowercase();
        let args: Vec<&str> = parts.collect();

        if args.is_empty() {
            if let Some(prompt) = self.session.start_tool(&cmd, &self.doc) {
                self.log(prompt);
                return;
            }
        }

        match cmd.as_str() {
            "box" => {
                let s = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                let id = self.doc.add_object(
                    "Box",
                    Geometry::Mesh(TriangleMesh::from_box(
                        DVec3::new(0.0, 0.0, s * 0.5),
                        DVec3::splat(s),
                    )),
                );
                if let Some(obj) = self.doc.objects.get_mut(id) {
                    obj.name = Some("Box".to_string());
                }
                self.mark_scene_dirty();
                self.log(format!("Created Box ({s} x {s} x {s})."));
            }
            "sphere" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                let id = self.doc.add_object(
                    "Sphere",
                    Geometry::Mesh(TriangleMesh::from_sphere(
                        DVec3::new(0.0, 0.0, r),
                        r,
                        24,
                        16,
                    )),
                );
                if let Some(obj) = self.doc.objects.get_mut(id) {
                    obj.name = Some("Sphere".to_string());
                }
                self.mark_scene_dirty();
                self.log(format!("Created Sphere (r = {r})."));
            }
            "circle" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                match NurbsCurve::circle(&Plane::world_xy(), r) {
                    Ok(c) => {
                        self.doc.add_object("Circle", Geometry::Curve(c));
                        self.mark_scene_dirty();
                        self.log(format!("Created NURBS Circle (r = {r})."));
                    }
                    Err(e) => self.log(format!("Error: {e}")),
                }
            }
            "line" => {
                let len = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(5.0);
                let c = NurbsCurve::line(DVec3::ZERO, DVec3::new(len, 0.0, 0.0));
                self.doc.add_object("Line", Geometry::Curve(c));
                self.mark_scene_dirty();
                self.log(format!("Created Line (0,0,0) -> ({len},0,0)."));
            }
            "cylinder" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                let h = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(4.0);
                self.doc.add_object(
                    "Cylinder",
                    Geometry::Mesh(TriangleMesh::from_cylinder(DVec3::ZERO, r, h, 32)),
                );
                self.mark_scene_dirty();
                self.log(format!("Created Cylinder (r={r}, h={h})."));
            }
            "cone" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                let h = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(4.0);
                self.doc.add_object(
                    "Cone",
                    Geometry::Mesh(TriangleMesh::from_cone(DVec3::ZERO, r, h, 32)),
                );
                self.mark_scene_dirty();
                self.log(format!("Created Cone (r={r}, h={h})."));
            }
            "torus" => {
                let maj = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                let min = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                self.doc.add_object(
                    "Torus",
                    Geometry::Mesh(TriangleMesh::from_torus(DVec3::ZERO, maj, min, 32, 16)),
                );
                self.mark_scene_dirty();
                self.log(format!("Created Torus (R={maj}, r={min})."));
            }
            "plane" => {
                let w = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(6.0);
                let h = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(w);
                self.doc.add_object(
                    "Plane",
                    Geometry::Surface(NurbsSurface::plane_rect(&cplane, w, h)),
                );
                self.mark_scene_dirty();
                self.log(format!("Created Plane ({w} x {h})."));
            }
            "extrude" => {
                let h = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                let dir = cplane.z_axis * h;
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.is_empty() {
                    self.log("Select one or more curves first, then run Extrude.");
                } else {
                    for (lid, c) in &curves {
                        if let Some(solid) = TriangleMesh::extrude_closed_curve(c, dir) {
                            self.doc
                                .add_object_on_layer("ExtrudeSolid", *lid, Geometry::Mesh(solid));
                        } else {
                            let surf = NurbsSurface::extrude_curve(c, dir);
                            self.doc
                                .add_object_on_layer("Extrude", *lid, Geometry::Surface(surf));
                        }
                    }
                    self.mark_scene_dirty();
                    self.log(format!("Extruded {} curve(s) by {h}.", curves.len()));
                }
            }
            "planarsrf" => {
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                let mut created = 0;
                for (lid, c) in &curves {
                    if let Some(mesh) = TriangleMesh::from_planar_curve(c) {
                        self.doc
                            .add_object_on_layer("PlanarSrf", *lid, Geometry::Mesh(mesh));
                        created += 1;
                    }
                }
                if created == 0 {
                    self.log("Select closed planar curve(s) first, then run PlanarSrf.");
                } else {
                    self.mark_scene_dirty();
                    self.log(format!("Created {created} planar surface(s)."));
                }
            }
            "loft" => {
                let curves: Vec<NurbsCurve> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                if let Some(mesh) = TriangleMesh::loft_curves(&curves, 32) {
                    self.doc.add_object("Loft", Geometry::Mesh(mesh));
                    self.mark_scene_dirty();
                    self.log(format!("Lofted {} curves.", curves.len()));
                } else {
                    self.log("Select 2 or more curves to Loft.");
                }
            }
            "booleanunion" | "booleandifference" | "booleanintersection" => {
                let op = match cmd.as_str() {
                    "booleanunion" => BooleanOp::Union,
                    "booleandifference" => BooleanOp::Difference,
                    _ => BooleanOp::Intersection,
                };
                let meshes: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Mesh(m) => Some((o.layer_id, m.clone())),
                        _ => None,
                    })
                    .collect();
                if meshes.len() < 2 {
                    self.log("Select 2 or more solid meshes for Boolean operation.");
                } else {
                    let lid = meshes[0].0;
                    let mut acc = meshes[0].1.clone();
                    for (_, m) in &meshes[1..] {
                        acc = mesh_boolean(&acc, m, op);
                    }
                    self.doc.remove_selected();
                    self.doc
                        .add_object_on_layer("Boolean", lid, Geometry::Mesh(acc));
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Completed {cmd} on {} meshes.", meshes.len()));
                }
            }
            "pointson" => {
                let mut added = 0;
                for (id, obj) in &self.doc.objects {
                    if obj.selected
                        && matches!(obj.geometry, Geometry::Curve(_) | Geometry::Surface(_))
                        && self.points_on_objects.insert(id)
                    {
                        added += 1;
                    }
                }
                self.log(format!(
                    "Control points on ({added} added, {} active).",
                    self.points_on_objects.len()
                ));
            }
            "pointsoff" => {
                self.points_on_objects.clear();
                self.grip_drag = None;
                self.log("Control points turned off.");
            }
            "move" => {
                let dx = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let dy = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                let dz = args.get(2).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                let n = self
                    .doc
                    .transform_selected("Move", &DMat4::from_translation(DVec3::new(dx, dy, dz)));
                self.mark_scene_dirty();
                self.log(format!(
                    "Moved {n} selected object(s) by ({dx}, {dy}, {dz})."
                ));
            }
            "scale" => {
                let s = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                let n = self
                    .doc
                    .transform_selected("Scale", &DMat4::from_scale(DVec3::splat(s)));
                self.mark_scene_dirty();
                self.log(format!("Scaled {n} selected object(s) by {s}."));
            }
            "delete" => {
                let n = self.doc.remove_selected();
                self.prune_points_on();
                self.mark_scene_dirty();
                self.log(format!("Deleted {n} object(s)."));
            }
            "selall" => {
                let n = self.doc.select_all();
                self.mark_doc_dirty();
                self.log(format!("Selected {n} object(s)."));
            }
            "selnone" => {
                self.doc.clear_selection();
                self.mark_doc_dirty();
                self.log("Selection cleared.");
            }
            "invert" => {
                let n = self.doc.invert_selection();
                self.mark_doc_dirty();
                self.log(format!("Inverted selection ({n} selected)."));
            }
            "sellast" => {
                let n = self.doc.select_last();
                self.mark_doc_dirty();
                self.log(format!("Selected {n} last created object(s)."));
            }
            "selcrv" => {
                let n = self
                    .doc
                    .select_by_kind(|g| matches!(g, Geometry::Curve(_)));
                self.mark_doc_dirty();
                self.log(format!("Selected {n} curve(s)."));
            }
            "selsrf" => {
                let n = self
                    .doc
                    .select_by_kind(|g| matches!(g, Geometry::Surface(_)));
                self.mark_doc_dirty();
                self.log(format!("Selected {n} surface(s)."));
            }
            "selmesh" => {
                let n = self.doc.select_by_kind(|g| matches!(g, Geometry::Mesh(_)));
                self.mark_doc_dirty();
                self.log(format!("Selected {n} mesh(es)."));
            }
            "selpt" => {
                let n = self
                    .doc
                    .select_by_kind(|g| matches!(g, Geometry::Point(_)));
                self.mark_doc_dirty();
                self.log(format!("Selected {n} point(s)."));
            }
            "hide" => {
                let n = self.doc.hide_selected();
                self.prune_points_on();
                self.mark_scene_dirty();
                self.log(format!("Hid {n} object(s)."));
            }
            "show" => {
                let n = self.doc.show_all();
                self.mark_scene_dirty();
                self.log(format!("Showed {n} hidden object(s)."));
            }
            "isolate" => {
                let n = self.doc.isolate_selected();
                self.prune_points_on();
                self.mark_scene_dirty();
                self.log(format!("Isolated selection (hid {n} other object(s))."));
            }
            "group" => match self.doc.group_selected() {
                Some((gid, count)) => {
                    self.log(format!("Grouped {count} objects (Group #{gid})."))
                }
                None => self.log("Select 2 or more objects to group."),
            },
            "ungroup" => {
                let n = self.doc.ungroup_selected();
                self.log(format!("Ungrouped {n} object(s)."));
            }
            "ellipse" => {
                let rx = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(4.0);
                let ry = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                match NurbsCurve::ellipse(&cplane, rx, ry) {
                    Ok(c) => {
                        self.doc.add_object("Ellipse", Geometry::Curve(c));
                        self.mark_scene_dirty();
                        self.log(format!("Created Ellipse (rx={rx}, ry={ry})."));
                    }
                    Err(e) => self.log(format!("Ellipse error: {e}")),
                }
            }
            "join" => {
                let mut curves = Vec::new();
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            curves.push(c.clone());
                        }
                    }
                }
                if curves.len() < 2 {
                    self.log("Select 2 or more curves to join.");
                } else {
                    let joined = NurbsCurve::join_curves(&curves, 1e-3);
                    let removed = self.doc.remove_selected();
                    let count = joined.len();
                    for c in joined {
                        self.doc.add_object("Join", Geometry::Curve(c));
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Joined {removed} curve(s) into {count} curve(s)."));
                }
            }
            "explode" => {
                let mut pieces = Vec::new();
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            let segs = c.explode_segments();
                            if segs.len() > 1 {
                                pieces.extend(segs.into_iter().map(|s| (obj.layer_id, s)));
                            }
                        }
                    }
                }
                if pieces.is_empty() {
                    self.log("Select one or more polylines to explode.");
                } else {
                    let removed = self.doc.remove_selected();
                    let count = pieces.len();
                    for (lid, seg) in pieces {
                        self.doc
                            .add_object_on_layer("Explode", lid, Geometry::Curve(seg));
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Exploded {removed} polyline(s) into {count} segments."
                    ));
                }
            }
            "fillet" => {
                let radius = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select exactly 2 curves to fillet, then run Fillet [radius].");
                } else {
                    let (lid_a, ref ca) = curves[0];
                    let (_, ref cb) = curves[1];
                    let tol = self.doc.tolerances.absolute.max(1e-3);
                    let hits = intersect_curves(ca, cb, tol);
                    let far_end = |c: &NurbsCurve, ref_pt: DVec3| -> DVec3 {
                        let (u0, u1) = c.domain();
                        let p0 = c.evaluate(u0);
                        let p1 = c.evaluate(u1);
                        if p0.distance(ref_pt) >= p1.distance(ref_pt) {
                            p0
                        } else {
                            p1
                        }
                    };
                    let (pick_a, pick_b) = if let Some(h) = hits.first() {
                        (far_end(ca, h.point), far_end(cb, h.point))
                    } else {
                        let (ua0, ua1) = ca.domain();
                        let (ub0, ub1) = cb.domain();
                        (
                            ca.evaluate(0.5 * (ua0 + ua1)),
                            cb.evaluate(0.5 * (ub0 + ub1)),
                        )
                    };
                    if let Some((ta, tb, arc)) =
                        fillet_two_curves(ca, cb, pick_a, pick_b, radius, &cplane, tol)
                    {
                        self.doc.remove_selected();
                        self.doc
                            .add_object_on_layer("FilletCurve", lid_a, Geometry::Curve(ta));
                        self.doc
                            .add_object_on_layer("FilletCurve", lid_a, Geometry::Curve(tb));
                        self.doc
                            .add_object_on_layer("FilletArc", lid_a, Geometry::Curve(arc));
                        self.prune_points_on();
                        self.mark_scene_dirty();
                        self.log(format!("Filleted 2 curves with radius {radius:.3}."));
                    } else {
                        self.log(
                            "Could not fillet the selected curves (parallel, non-coplanar, or radius too large).",
                        );
                    }
                }
            }
            "chamfer" => {
                let d1 = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let d2 = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(d1);
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select exactly 2 curves to chamfer, then run Chamfer [d1] [d2].");
                } else {
                    let (lid_a, ref ca) = curves[0];
                    let (_, ref cb) = curves[1];
                    let tol = self.doc.tolerances.absolute.max(1e-3);
                    let hits = intersect_curves(ca, cb, tol);
                    let far_end = |c: &NurbsCurve, ref_pt: DVec3| -> DVec3 {
                        let (u0, u1) = c.domain();
                        let p0 = c.evaluate(u0);
                        let p1 = c.evaluate(u1);
                        if p0.distance(ref_pt) >= p1.distance(ref_pt) {
                            p0
                        } else {
                            p1
                        }
                    };
                    let (pick_a, pick_b) = if let Some(h) = hits.first() {
                        (far_end(ca, h.point), far_end(cb, h.point))
                    } else {
                        (ca.evaluate(ca.domain().0), cb.evaluate(cb.domain().0))
                    };
                    if let Some((ta, tb, line)) =
                        chamfer_two_curves(ca, cb, pick_a, pick_b, d1, d2, tol)
                    {
                        self.doc.remove_selected();
                        self.doc
                            .add_object_on_layer("ChamferCurve", lid_a, Geometry::Curve(ta));
                        self.doc
                            .add_object_on_layer("ChamferCurve", lid_a, Geometry::Curve(tb));
                        self.doc
                            .add_object_on_layer("ChamferLine", lid_a, Geometry::Curve(line));
                        self.prune_points_on();
                        self.mark_scene_dirty();
                        self.log(format!("Chamfered 2 curves ({d1:.3}, {d2:.3})."));
                    } else {
                        self.log("Could not chamfer selected curves.");
                    }
                }
            }
            "split" => {
                let selected: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected)
                    .filter_map(|(id, o)| match &o.geometry {
                        Geometry::Curve(c) => Some((id, o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if selected.is_empty() {
                    self.log("Select 1 curve (interactive split) or 2+ intersecting curves to Split.");
                } else if selected.len() == 1 {
                    if let Some(prompt) = self.session.start_tool("splitcurve", &self.doc) {
                        self.log(prompt);
                    }
                } else {
                    let tol = self.doc.tolerances.absolute.max(1e-3);
                    let mut split_count = 0usize;
                    let mut piece_total = 0usize;
                    for i in 0..selected.len() {
                        let (id, lid, ref ci) = selected[i];
                        let mut params = Vec::new();
                        for (j, (_, _, cj)) in selected.iter().enumerate() {
                            if i != j {
                                for hit in intersect_curves(ci, cj, tol) {
                                    params.push(hit.u_a);
                                }
                            }
                        }
                        if !params.is_empty() {
                            let pieces = ci.split_at_params(&params);
                            if pieces.len() > 1 {
                                piece_total += pieces.len();
                                split_count += 1;
                                let reps = pieces
                                    .into_iter()
                                    .map(|p| (lid, Geometry::Curve(p)))
                                    .collect();
                                self.doc.replace_object_with_many("Split", id, reps);
                            }
                        }
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    if split_count > 0 {
                        self.log(format!("Split {split_count} curve(s) into {piece_total} pieces."));
                    } else {
                        self.log("Selected curves have no mutual intersections.");
                    }
                }
            }
            "blendcrv" => {
                let mut cont = Continuity::G2;
                let mut bulge = 1.0_f64;
                for arg in &args {
                    match arg.to_ascii_lowercase().as_str() {
                        "g0" => cont = Continuity::G0,
                        "g1" => cont = Continuity::G1,
                        "g2" => cont = Continuity::G2,
                        "g3" => cont = Continuity::G3,
                        _ => {
                            if let Ok(v) = arg.parse::<f64>() {
                                bulge = v.clamp(0.05, 5.0);
                            }
                        }
                    }
                }
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select exactly 2 curves to BlendCrv [g0|g1|g2|g3] [bulge].");
                } else {
                    let (lid, ref ca) = curves[0];
                    let (_, ref cb) = curves[1];
                    let (at_end_a, at_end_b) = closest_endpoint_pair(ca, cb);
                    match blend_curves(ca, at_end_a, cb, at_end_b, cont, bulge, bulge) {
                        Ok(blended) => {
                            self.doc
                                .add_object_on_layer("BlendCrv", lid, Geometry::Curve(blended));
                            self.mark_scene_dirty();
                            self.log(format!("Created {} blend curve.", cont.label()));
                        }
                        Err(e) => self.log(format!("BlendCrv error: {e}")),
                    }
                }
            }
            "match" => {
                let cont = match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("g0") => Continuity::G0,
                    Some("g1") => Continuity::G1,
                    Some("g3") => Continuity::G3,
                    _ => Continuity::G2,
                };
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected)
                    .filter_map(|(id, o)| match &o.geometry {
                        Geometry::Curve(c) => Some((id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select 2 curves (1st is matched to 2nd): Match [g0|g1|g2].");
                } else {
                    let (id_a, ref ca) = curves[0];
                    let (_, ref cb) = curves[1];
                    let (at_end_a, at_end_b) = closest_endpoint_pair(ca, cb);
                    match match_curve(ca, at_end_a, cb, at_end_b, cont) {
                        Ok(matched) => {
                            let before = self.doc.objects[id_a].clone();
                            self.doc.objects[id_a].geometry = Geometry::Curve(matched);
                            self.doc.objects[id_a].refresh_aabb();
                            self.doc.record_object_modified("Match", id_a, before);
                            self.mark_scene_dirty();
                            self.log(format!("Matched curve with {}.", cont.label()));
                        }
                        Err(e) => self.log(format!("Match error: {e}")),
                    }
                }
            }
            "rebuild" => {
                let cp_count = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(10);
                let degree = args.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                let ids: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
                if ids.is_empty() {
                    self.log("Select curve(s) to Rebuild [cp_count] [degree].");
                } else {
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Curve(c) = &before.geometry {
                            match rebuild_uniform(c, cp_count, degree) {
                                Ok(rep) => {
                                    self.doc.objects[id].geometry = Geometry::Curve(rep.curve);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("Rebuild", id, before);
                                    self.log(format!(
                                        "Rebuilt curve: {} -> {} CPs (deg {}, max dev {:.5}).",
                                        rep.old_cp_count,
                                        rep.new_cp_count,
                                        degree,
                                        rep.max_deviation
                                    ));
                                }
                                Err(e) => self.log(format!("Rebuild error: {e}")),
                            }
                        }
                    }
                    self.mark_scene_dirty();
                }
            }
            "rebuildadaptive" | "elmo" => {
                let tol = args
                    .first()
                    .and_then(|v| v.parse::<f64>().ok())
                    .unwrap_or_else(|| self.doc.tolerances.absolute.max(0.01));
                let degree = args.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                let ids: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
                if ids.is_empty() {
                    self.log("Select curve(s) to run RebuildAdaptive / Elmo [max_tol] [degree].");
                } else {
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Curve(c) = &before.geometry {
                            match rebuild_adaptive(c, degree, tol, true, true) {
                                Ok(rep) => {
                                    self.doc.objects[id].geometry = Geometry::Curve(rep.curve);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("RebuildAdaptive", id, before);
                                    self.log(format!(
                                        "Elmo adaptive rebuild: {} -> {} CPs (max dev {:.5}).",
                                        rep.old_cp_count, rep.new_cp_count, rep.max_deviation
                                    ));
                                }
                                Err(e) => self.log(format!("RebuildAdaptive error: {e}")),
                            }
                        }
                    }
                    self.mark_scene_dirty();
                }
            }
            "fair" => {
                let strength = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.35);
                let ids: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
                if ids.is_empty() {
                    self.log("Select curve(s) to Fair [strength].");
                } else {
                    let count = ids.len();
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Curve(c) = &before.geometry {
                            let faired = fair_curve(c, strength, 4);
                            self.doc.objects[id].geometry = Geometry::Curve(faired);
                            self.doc.objects[id].refresh_aabb();
                            self.doc.record_object_modified("Fair", id, before);
                        }
                    }
                    self.mark_scene_dirty();
                    self.log(format!("Faired {count} curve(s)."));
                }
            }
            "tweencurves" => {
                let count = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                let curves: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select 2 curves to run TweenCurves [count].");
                } else {
                    let tweens = tween_curves(&curves[0].1, &curves[1].1, count);
                    let n = tweens.len();
                    for tw in tweens {
                        self.doc
                            .add_object_on_layer("TweenCurve", curves[0].0, Geometry::Curve(tw));
                    }
                    self.mark_scene_dirty();
                    self.log(format!("Created {n} tweened curve(s)."));
                }
            }
            "changedegree" => {
                let target_deg = args
                    .first()
                    .and_then(|v| v.parse::<usize>().ok())
                    .unwrap_or(3)
                    .clamp(1, 7);
                let ids: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
                if ids.is_empty() {
                    self.log("Select curve(s) to ChangeDegree [degree].");
                } else {
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Curve(c) = &before.geometry {
                            if target_deg > c.degree {
                                if let Ok(elevated) = c.elevate_degree(target_deg - c.degree) {
                                    self.doc.objects[id].geometry = Geometry::Curve(elevated);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ChangeDegree", id, before);
                                    self.log(format!("Elevated curve to degree {target_deg}."));
                                }
                            } else if target_deg < c.degree {
                                if let Ok(rep) = rebuild_uniform(c, c.control_points.len(), target_deg)
                                {
                                    self.doc.objects[id].geometry = Geometry::Curve(rep.curve);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ChangeDegree", id, before);
                                    self.log(format!(
                                        "Rebuilt curve to degree {target_deg} (max dev {:.5}).",
                                        rep.max_deviation
                                    ));
                                }
                            }
                        }
                    }
                    self.mark_scene_dirty();
                }
            }
            "curvaturegraph" => {
                let mut added = 0;
                for (id, obj) in &self.doc.objects {
                    if obj.selected
                        && matches!(obj.geometry, Geometry::Curve(_))
                        && self.curvature_graph_objects.insert(id)
                    {
                        added += 1;
                    }
                }
                self.log(format!(
                    "CurvatureGraph on ({added} added, {} active).",
                    self.curvature_graph_objects.len()
                ));
            }
            "curvaturegraphoff" => {
                self.curvature_graph_objects.clear();
                self.log("CurvatureGraph turned off.");
            }
            "length" => {
                let mut total = 0.0;
                let mut count = 0;
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            total += c.length_approx(64);
                            count += 1;
                        }
                    }
                }
                if count == 0 {
                    self.log("Select one or more curves to measure Length.");
                } else {
                    self.log(format!("Total length of {count} curve(s): {total:.4}"));
                }
            }
            "area" => {
                let mut total = 0.0;
                let mut count = 0;
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            if c.is_closed() {
                                total += c.planar_area_approx(&cplane, 64);
                                count += 1;
                            }
                        }
                    }
                }
                if count == 0 {
                    self.log("Select one or more closed planar curves to measure Area.");
                } else {
                    self.log(format!(
                        "Total planar area of {count} closed curve(s): {total:.4}"
                    ));
                }
            }
            "undo" => match self.doc.undo() {
                Some(label) => {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Undid '{label}'."));
                }
                None => self.log("Nothing to undo."),
            },
            "redo" => match self.doc.redo() {
                Some(label) => {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Redid '{label}'."));
                }
                None => self.log("Nothing to redo."),
            },
            "zoomextents" | "ze" => {
                self.zoom_extents_all();
                self.log("Zoomed to extents.");
            }
            "wireframe" => self.set_active_viewport_display_mode(DisplayMode::Wireframe),
            "shaded" | "shade" => self.set_active_viewport_display_mode(DisplayMode::Shaded),
            "ghosted" => self.set_active_viewport_display_mode(DisplayMode::Ghosted),
            "technical" => self.set_active_viewport_display_mode(DisplayMode::Technical),
            "arctic" => self.set_active_viewport_display_mode(DisplayMode::Arctic),
            "units" => {
                if args.is_empty() {
                    self.log(format!(
                        "Units: {} ({}), absolute tolerance = {:.6}",
                        self.doc.units.full_name(),
                        self.doc.units.label(),
                        self.doc.tolerances.absolute
                    ));
                } else {
                    let parsed = match args[0].to_ascii_lowercase().as_str() {
                        "mm" | "millimeter" | "millimeters" => Some(UnitSystem::Millimeters),
                        "cm" | "centimeter" | "centimeters" => Some(UnitSystem::Centimeters),
                        "m" | "meter" | "meters" => Some(UnitSystem::Meters),
                        "in" | "inch" | "inches" => Some(UnitSystem::Inches),
                        "ft" | "foot" | "feet" => Some(UnitSystem::Feet),
                        _ => None,
                    };
                    if let Some(unit) = parsed {
                        let rescale = match args.get(1).map(|s| s.to_ascii_lowercase()).as_deref() {
                            Some("keep" | "no" | "false") => false,
                            Some("rescale" | "yes" | "true") => true,
                            _ => self.rescale_on_unit_change,
                        };
                        self.change_units(unit, rescale);
                    } else {
                        self.log("Usage: Units [mm|cm|m|in|ft] [rescale|keep]");
                    }
                }
            }
            "text" => {
                let label = if args.is_empty() {
                    "Text".to_string()
                } else {
                    args.join(" ")
                };
                self.doc.add_annotation(AnnotationItem {
                    kind: AnnotationKind::Text,
                    layer_id: self.doc.active_layer,
                    color: None,
                    text: label.clone(),
                    position: cplane.origin,
                    p1: None,
                    p2: None,
                    offset_pt: None,
                });
                self.mark_doc_dirty();
                self.log(format!("Created Text '{label}' at CPlane origin."));
            }
            "textdot" | "dot" => {
                let label = if args.is_empty() {
                    "Note".to_string()
                } else {
                    args.join(" ")
                };
                self.session.tool = cad_commands::ActiveTool::TextDot { label };
                self.log(self.session.prompt());
            }
            "new" => {
                self.session.cancel();
                self.points_on_objects.clear();
                self.curvature_graph_objects.clear();
                self.grip_drag = None;
                self.gumball_drag = None;
                self.gumball_prompt = None;
                self.marquee = None;
                self.doc = Document::new();
                self.mark_scene_dirty();
                self.log("Created new empty document.");
            }
            "open" => self.open_project_dialog(),
            "save" => self.save_project_dialog(),
            "import" => self.import_mesh_dialog(),
            "export" => self.export_mesh_dialog(),
            "curve" => {
                let deg = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(3).clamp(1, 7);
                let closed = args.iter().any(|a| a.eq_ignore_ascii_case("close") || a.eq_ignore_ascii_case("closed"));
                self.session.tool = cad_commands::ActiveTool::Curve {
                    points: Vec::new(),
                    degree: deg,
                    closed,
                };
                self.log(self.session.prompt());
            }
            "interpcrv" => {
                let deg = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(3).clamp(1, 7);
                let closed = args.iter().any(|a| a.eq_ignore_ascii_case("close") || a.eq_ignore_ascii_case("closed"));
                self.session.tool = cad_commands::ActiveTool::InterpCrv {
                    points: Vec::new(),
                    degree: deg,
                    closed,
                };
                self.log(self.session.prompt());
            }
            "helix" => {
                let turns = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0).clamp(-100.0, 100.0);
                self.session.tool = cad_commands::ActiveTool::Helix {
                    axis_start: None,
                    axis_end: None,
                    turns,
                };
                self.log(self.session.prompt());
            }
            "help" => {
                self.log("Interactive: Line, Polyline, Curve, InterpCrv, HandleCurve, Conic, Helix, Rectangle, Arc, Circle, Polygon, Box, Cylinder, Cone, Revolve, Move, Copy, Rotate, Scale, Mirror, Offset, SplitCurve, Trim, InsertKnot, ArrayLinear, ArrayPolar, Distance, Dim, TextDot | Quick: Sphere, Torus, Plane, Extrude, PlanarSrf, Loft, BooleanUnion/Difference/Intersection, Join, Explode, Split, Fillet, Chamfer, BlendCrv, Match, Rebuild, RebuildAdaptive (Elmo), Fair, TweenCurves, ChangeDegree, CurvatureGraph, CurvatureGraphOff, PointsOn/Off, Units [mm|cm|m|in|ft], Wireframe/Shaded/Ghosted/Technical/Arctic, ZoomExtents (ZE), Undo, Redo, New");
            }
            _ => {
                self.log(format!(
                    "Unknown command '{cmd}'. Type 'Help' for available commands."
                ));
            }
        }
    }
}

fn closest_endpoint_pair(a: &NurbsCurve, b: &NurbsCurve) -> (bool, bool) {
    let (ua0, ua1) = a.domain();
    let (ub0, ub1) = b.domain();
    let ea = [a.evaluate(ua0), a.evaluate(ua1)];
    let eb = [b.evaluate(ub0), b.evaluate(ub1)];
    let mut best = (true, false);
    let mut best_d = f64::INFINITY;
    for (ia, &pa) in ea.iter().enumerate() {
        for (ib, &pb) in eb.iter().enumerate() {
            let d = pa.distance(pb);
            if d < best_d {
                best_d = d;
                best = (ia == 1, ib == 1);
            }
        }
    }
    best
}
