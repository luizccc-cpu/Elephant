use cad_doc::{AnnotationItem, AnnotationKind, Document};
use cad_geom::{
    BooleanOp, DMat4, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh, UnitSystem,
    mesh_boolean,
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
                let lines: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) if c.degree == 1 && c.control_points.len() == 2 => {
                            Some((
                                o.layer_id,
                                c.control_points[0].position,
                                c.control_points[1].position,
                            ))
                        }
                        _ => None,
                    })
                    .collect();
                if lines.len() != 2 {
                    self.log("Select exactly 2 line segments to fillet, then run Fillet [radius].");
                } else if let Some((la, lb, arc)) = NurbsCurve::fillet_two_lines(
                    lines[0].1, lines[0].2, lines[1].1, lines[1].2, radius,
                ) {
                    let lid = lines[0].0;
                    self.doc.remove_selected();
                    self.doc
                        .add_object_on_layer("FilletLine", lid, Geometry::Curve(la));
                    self.doc
                        .add_object_on_layer("FilletLine", lid, Geometry::Curve(lb));
                    self.doc
                        .add_object_on_layer("FilletArc", lid, Geometry::Curve(arc));
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Filleted 2 lines with radius {radius:.3}."));
                } else {
                    self.log("Could not fillet the selected lines (parallel, non-coplanar, or radius too large).");
                }
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
            "help" => {
                self.log("Interactive: Line, Polyline, Rectangle, Arc, Circle, Polygon, Box, Cylinder, Cone, Revolve, Move, Copy, Rotate, Scale, Mirror, Offset, ArrayLinear, ArrayPolar, Distance, Dim, TextDot | Quick: Sphere, Torus, Plane, Extrude, PlanarSrf, Loft, BooleanUnion/Difference/Intersection, Join, Explode, Fillet, PointsOn/Off, Units [mm|cm|m|in|ft], Wireframe/Shaded/Ghosted/Technical/Arctic, ZoomExtents (ZE), Undo, Redo, New");
            }
            _ => {
                self.log(format!(
                    "Unknown command '{cmd}'. Type 'Help' for available commands."
                ));
            }
        }
    }
}
