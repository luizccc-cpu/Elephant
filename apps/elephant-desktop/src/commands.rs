use cad_doc::{
    AnnotationItem, AnnotationKind, Document, GdtSpec, GdtSymbol, HistoryOp, ObjectId, PageSize,
};
use cad_geom::{
    BRep, BooleanOp, BoundaryLock, Continuity, DMat4, DVec3, DrawingProjection, Geometry,
    LoftStyle, Make2DLineKind, Make2DOptions, NurbsCurve, NurbsSurface, PatchBoundaryConstraint,
    PatchOptions, Plane, SoftTransformFalloff, SubObjectRef, SurfaceEdge, UnitSystem, blend_curves,
    blend_srf, brep_boolean, brep_boolean_split, cap_planar_holes, chamfer_brep_edges,
    chamfer_two_curves, closest_edge_pair, edge_srf, extend_srf, fair_curve, fillet_brep_edges,
    fillet_srf, fillet_two_curves, inset_face, intersect_breps, intersect_curves,
    intersect_surfaces, loft_nurbs, make_2d_projection, match_curve, match_srf, mesh_boolean,
    network_srf, offset_srf, partition_network_curves, patch_srf, project_curve_to_surface,
    pull_curve_to_surface, push_pull_face, rebuild_adaptive, rebuild_srf, rebuild_uniform,
    section_geometry_with_plane, shell_brep, split_face_with_uv_curve, sweep1, sweep2,
    tangent_edge_chain, trim_face_with_curve, tween_curves, untrim_brep,
};
use cad_render::{DisplayMode, SurfaceAnalysisConfig, SurfaceAnalysisMode};

use crate::state::{ElephantApp, InspectorTab, WorkspaceViewMode};

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

        let has_brep_or_srf = self.doc.objects.values().any(|o| {
            o.selected && matches!(o.geometry, Geometry::BRep(_) | Geometry::Surface(_))
        });
        let has_curve = self
            .doc
            .objects
            .values()
            .any(|o| o.selected && matches!(o.geometry, Geometry::Curve(_)));
        let surf_trim_intercept = cmd == "trim" && has_brep_or_srf && has_curve;

        if args.is_empty() && !surf_trim_intercept {
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
                    Geometry::BRep(BRep::from_box(
                        DVec3::new(0.0, 0.0, s * 0.5),
                        DVec3::splat(s),
                    )),
                );
                if let Some(obj) = self.doc.objects.get_mut(id) {
                    obj.name = Some("Box".to_string());
                }
                self.mark_scene_dirty();
                self.log(format!("Created B-Rep Box ({s} x {s} x {s})."));
            }
            "sphere" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                match NurbsSurface::sphere(DVec3::new(0.0, 0.0, r), r) {
                    Ok(srf) => {
                        let id = self.doc.add_object("Sphere", Geometry::Surface(srf));
                        if let Some(obj) = self.doc.objects.get_mut(id) {
                            obj.name = Some("Sphere".to_string());
                        }
                        self.mark_scene_dirty();
                        self.log(format!("Created NURBS Sphere (r = {r})."));
                    }
                    Err(e) => self.log(format!("Sphere error: {e}")),
                }
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
                match BRep::from_cylinder(DVec3::ZERO, r, h) {
                    Ok(brep) => {
                        self.doc.add_object("Cylinder", Geometry::BRep(brep));
                        self.mark_scene_dirty();
                        self.log(format!("Created B-Rep Cylinder (r={r}, h={h})."));
                    }
                    Err(e) => self.log(format!("Cylinder error: {e}")),
                }
            }
            "cone" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                let h = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(4.0);
                match BRep::from_cone(DVec3::ZERO, r, h) {
                    Ok(brep) => {
                        self.doc.add_object("Cone", Geometry::BRep(brep));
                        self.mark_scene_dirty();
                        self.log(format!("Created B-Rep Cone (r={r}, h={h})."));
                    }
                    Err(e) => self.log(format!("Cone error: {e}")),
                }
            }
            "torus" => {
                let maj = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                let min = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                match NurbsSurface::torus(DVec3::ZERO, maj, min) {
                    Ok(srf) => {
                        self.doc.add_object("Torus", Geometry::Surface(srf));
                        self.mark_scene_dirty();
                        self.log(format!("Created NURBS Torus (R={maj}, r={min})."));
                    }
                    Err(e) => self.log(format!("Torus error: {e}")),
                }
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
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
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
                if curves.is_empty() {
                    self.log("Select one or more curves first, then run Extrude.");
                } else {
                    for (pid, lid, c) in &curves {
                        if c.is_closed() {
                            if let Ok(solid) = BRep::extrude_curve_solid(c, dir) {
                                let cid = self.doc.add_object_on_layer(
                                    "ExtrudeSolid",
                                    *lid,
                                    Geometry::BRep(solid),
                                );
                                self.doc.add_history_record(
                                    cid,
                                    vec![*pid],
                                    HistoryOp::Extrude {
                                        direction: dir,
                                        solid: true,
                                    },
                                );
                                continue;
                            }
                        }
                        let surf = NurbsSurface::extrude_curve(c, dir);
                        let cid = self
                            .doc
                            .add_object_on_layer("Extrude", *lid, Geometry::Surface(surf));
                        self.doc.add_history_record(
                            cid,
                            vec![*pid],
                            HistoryOp::Extrude {
                                direction: dir,
                                solid: false,
                            },
                        );
                    }
                    self.mark_scene_dirty();
                    self.log(format!("Extruded {} curve(s) by {h}.", curves.len()));
                }
            }
            "planarsrf" => {
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Curve(c) if c.is_closed() => {
                                Some((id, o.layer_id, c.clone()))
                            }
                            _ => None,
                        })
                    })
                    .collect();
                if curves.is_empty() {
                    self.log("Select closed planar curve(s) first, then run PlanarSrf.");
                } else {
                    let lid0 = curves[0].1;
                    let pids: Vec<ObjectId> = curves.iter().map(|(id, _, _)| *id).collect();
                    let only_curves: Vec<NurbsCurve> =
                        curves.iter().map(|(_, _, c)| c.clone()).collect();
                    let mut created = 0usize;
                    if let Ok(brep) = BRep::from_planar_curves(&only_curves) {
                        let cid = self.doc.add_object_on_layer(
                            "PlanarSrf",
                            lid0,
                            Geometry::BRep(brep),
                        );
                        self.doc
                            .add_history_record(cid, pids, HistoryOp::PlanarSrf);
                        created = 1;
                    } else {
                        for (pid, lid, c) in &curves {
                            if let Ok(brep) = BRep::from_planar_curves(std::slice::from_ref(c)) {
                                let cid = self.doc.add_object_on_layer(
                                    "PlanarSrf",
                                    *lid,
                                    Geometry::BRep(brep),
                                );
                                self.doc
                                    .add_history_record(cid, vec![*pid], HistoryOp::PlanarSrf);
                                created += 1;
                            }
                        }
                    }
                    if created == 0 {
                        self.log("Could not build planar B-Rep from selected curve(s).");
                    } else {
                        self.mark_scene_dirty();
                        self.log(format!("Created {created} trimmed planar B-Rep(s)."));
                    }
                }
            }
            "loft" => {
                let mut style = LoftStyle::Normal;
                let mut closed = false;
                for arg in &args {
                    match arg.to_ascii_lowercase().as_str() {
                        "loose" => style = LoftStyle::Loose,
                        "tight" => style = LoftStyle::Tight,
                        "straight" => style = LoftStyle::Straight,
                        "normal" => style = LoftStyle::Normal,
                        "close" | "closed" => closed = true,
                        _ => {}
                    }
                }
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
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
                if curves.len() < 2 {
                    self.log("Select 2 or more curves to Loft [normal|loose|tight|straight] [closed].");
                } else {
                    let lid = curves[0].1;
                    let pids: Vec<ObjectId> = curves.iter().map(|(id, _, _)| *id).collect();
                    let raw: Vec<NurbsCurve> = curves.into_iter().map(|(_, _, c)| c).collect();
                    let n_crvs = raw.len();
                    match loft_nurbs(&raw, style, closed) {
                        Ok(srf) => {
                            let cid = self
                                .doc
                                .add_object_on_layer("Loft", lid, Geometry::Surface(srf));
                            self.doc
                                .add_history_record(cid, pids, HistoryOp::Loft { style, closed });
                            self.mark_scene_dirty();
                            self.log(format!("Lofted {n_crvs} curves into NURBS surface ({style:?})."));
                        }
                        Err(e) => self.log(format!("Loft error: {e}")),
                    }
                }
            }
            "sweep1" => {
                let roadlike = args
                    .iter()
                    .any(|a| a.eq_ignore_ascii_case("roadlike") || a.eq_ignore_ascii_case("road"));
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
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
                if curves.len() < 2 {
                    self.log("Select rail curve first, then 1+ profile curves: Sweep1 [roadlike].");
                } else {
                    let lid = curves[0].1;
                    let pids: Vec<ObjectId> = curves.iter().map(|(id, _, _)| *id).collect();
                    let rail = &curves[0].2;
                    let profs: Vec<NurbsCurve> =
                        curves[1..].iter().map(|(_, _, c)| c.clone()).collect();
                    let up = roadlike.then_some(cplane.z_axis);
                    match sweep1(rail, &profs, up) {
                        Ok(srf) => {
                            let cid = self
                                .doc
                                .add_object_on_layer("Sweep1", lid, Geometry::Surface(srf));
                            self.doc.add_history_record(cid, pids, HistoryOp::Sweep1);
                            self.mark_scene_dirty();
                            self.log(format!("Created Sweep1 NURBS surface from {} profile(s).", profs.len()));
                        }
                        Err(e) => self.log(format!("Sweep1 error: {e}")),
                    }
                }
            }
            "sweep2" => {
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
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
                if curves.len() < 3 {
                    self.log("Select 2 rail curves first, then 1+ profile curves: Sweep2.");
                } else {
                    let lid = curves[0].1;
                    let pids: Vec<ObjectId> = curves.iter().map(|(id, _, _)| *id).collect();
                    let profs: Vec<NurbsCurve> =
                        curves[2..].iter().map(|(_, _, c)| c.clone()).collect();
                    match sweep2(&curves[0].2, &curves[1].2, &profs) {
                        Ok(srf) => {
                            let cid = self
                                .doc
                                .add_object_on_layer("Sweep2", lid, Geometry::Surface(srf));
                            self.doc.add_history_record(cid, pids, HistoryOp::Sweep2);
                            self.mark_scene_dirty();
                            self.log(format!("Created Sweep2 NURBS surface from {} profile(s).", profs.len()));
                        }
                        Err(e) => self.log(format!("Sweep2 error: {e}")),
                    }
                }
            }
            "edgesrf" => {
                let curves: Vec<(ObjectId, cad_doc::LayerId, NurbsCurve)> = self
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
                if !(2..=4).contains(&curves.len()) {
                    self.log("Select 2, 3, or 4 boundary curves for EdgeSrf.");
                } else {
                    let lid = curves[0].1;
                    let pids: Vec<ObjectId> = curves.iter().map(|(id, _, _)| *id).collect();
                    let raw: Vec<NurbsCurve> = curves.into_iter().map(|(_, _, c)| c).collect();
                    match edge_srf(&raw) {
                        Ok(srf) => {
                            let cid = self
                                .doc
                                .add_object_on_layer("EdgeSrf", lid, Geometry::Surface(srf));
                            self.doc.add_history_record(cid, pids, HistoryOp::EdgeSrf);
                            self.mark_scene_dirty();
                            self.log("Created Coons NURBS EdgeSrf.");
                        }
                        Err(e) => self.log(format!("EdgeSrf error: {e}")),
                    }
                }
            }
            "networksrf" => {
                let curves: Vec<(cad_doc::LayerId, NurbsCurve)> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if curves.len() < 4 {
                    self.log("Select 4 or more intersecting curves (2+ U, 2+ V) for NetworkSrf.");
                } else {
                    let lid = curves[0].0;
                    let raw: Vec<NurbsCurve> = curves.into_iter().map(|c| c.1).collect();
                    let tol = self.doc.tolerances.absolute.max(1e-2);
                    if let Some((ug, vg)) = partition_network_curves(&raw, tol) {
                        match network_srf(&ug, &vg) {
                            Ok(srf) => {
                                self.doc
                                    .add_object_on_layer("NetworkSrf", lid, Geometry::Surface(srf));
                                self.mark_scene_dirty();
                                self.log(format!(
                                    "Created Gordon NURBS NetworkSrf ({} U x {} V curves).",
                                    ug.len(),
                                    vg.len()
                                ));
                            }
                            Err(e) => self.log(format!("NetworkSrf error: {e}")),
                        }
                    } else {
                        self.log(
                            "Could not partition curves into 2 crossing U/V families.",
                        );
                    }
                }
            }
            "blendsrf" => {
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
                let surfs: Vec<(cad_doc::LayerId, NurbsSurface)> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Surface(s) => Some((o.layer_id, s.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if surfs.len() != 2 {
                    self.log("Select 2 NURBS surfaces to BlendSrf [g0|g1|g2|g3] [bulge].");
                } else {
                    let (ea, eb) = closest_edge_pair(&surfs[0].1, &surfs[1].1);
                    match blend_srf(&surfs[0].1, ea, &surfs[1].1, eb, cont, bulge, bulge) {
                        Ok(blend) => {
                            self.doc
                                .add_object_on_layer("BlendSrf", surfs[0].0, Geometry::Surface(blend));
                            self.mark_scene_dirty();
                            self.log(format!("Created {} BlendSrf between closest edges.", cont.label()));
                        }
                        Err(e) => self.log(format!("BlendSrf error: {e}")),
                    }
                }
            }
            "matchsrf" => {
                let cont = match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("g0") => Continuity::G0,
                    Some("g1") => Continuity::G1,
                    Some("g3") => Continuity::G3,
                    _ => Continuity::G2,
                };
                let surfs: Vec<(cad_doc::ObjectId, NurbsSurface)> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Surface(s) => Some((id, s.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if surfs.len() != 2 {
                    self.log("Select 2 NURBS surfaces (1st matched to 2nd): MatchSrf [g0|g1|g2].");
                } else {
                    let (ea, eb) = closest_edge_pair(&surfs[0].1, &surfs[1].1);
                    match match_srf(&surfs[0].1, ea, &surfs[1].1, eb, cont) {
                        Ok(matched) => {
                            let id_a = surfs[0].0;
                            let before = self.doc.objects[id_a].clone();
                            self.doc.objects[id_a].geometry = Geometry::Surface(matched);
                            self.doc.objects[id_a].refresh_aabb();
                            self.doc.record_object_modified("MatchSrf", id_a, before);
                            self.mark_scene_dirty();
                            self.log(format!("Matched surface edge with {}.", cont.label()));
                        }
                        Err(e) => self.log(format!("MatchSrf error: {e}")),
                    }
                }
            }
            "offsetsrf" => {
                let dist = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let surfs: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Surface(s) => Some((id, o.layer_id, s.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if surfs.is_empty() {
                    self.log("Select NURBS surface(s) to OffsetSrf [distance].");
                } else {
                    let mut created = 0;
                    for (pid, lid, s) in &surfs {
                        if let Ok(off) = offset_srf(s, dist) {
                            let cid = self
                                .doc
                                .add_object_on_layer("OffsetSrf", *lid, Geometry::Surface(off));
                            self.doc.add_history_record(
                                cid,
                                vec![*pid],
                                HistoryOp::OffsetSrf {
                                    distance: dist,
                                    solid: false,
                                },
                            );
                            created += 1;
                        }
                    }
                    self.mark_scene_dirty();
                    self.log(format!("Offset {created} NURBS surface(s) by {dist:.3}."));
                }
            }
            "extendsrf" => {
                let len = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                let edge = match args.get(1).map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("umin") => SurfaceEdge::UMin,
                    Some("vmin") => SurfaceEdge::VMin,
                    Some("vmax") => SurfaceEdge::VMax,
                    _ => SurfaceEdge::UMax,
                };
                let ids: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter(|&id| {
                        self.doc
                            .objects
                            .get(id)
                            .is_some_and(|o| matches!(o.geometry, Geometry::Surface(_)))
                    })
                    .collect();
                if ids.is_empty() {
                    self.log("Select NURBS surface(s) to ExtendSrf [length] [umin|umax|vmin|vmax].");
                } else {
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Surface(s) = &before.geometry {
                            match extend_srf(s, edge, len) {
                                Ok(ext) => {
                                    self.doc.objects[id].geometry = Geometry::Surface(ext);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ExtendSrf", id, before);
                                    self.log(format!("Extended surface edge {edge:?} by {len:.3}."));
                                }
                                Err(e) => self.log(format!("ExtendSrf error: {e}")),
                            }
                        }
                    }
                    self.mark_scene_dirty();
                }
            }
            "patch" => {
                let mut opts = PatchOptions::default();
                for arg in &args {
                    match arg.to_ascii_lowercase().as_str() {
                        "g0" => opts.continuity = Continuity::G0,
                        "g1" => opts.continuity = Continuity::G1,
                        "g2" => opts.continuity = Continuity::G2,
                        _ => {
                            if let Ok(n) = arg.parse::<usize>() {
                                let sp = n.clamp(2, 8);
                                opts.spans_u = sp;
                                opts.spans_v = sp;
                            } else if let Ok(st) = arg.parse::<f64>() {
                                opts.stiffness = st.clamp(0.0001, 5.0);
                            }
                        }
                    }
                }
                let mut curves = Vec::new();
                let mut ref_surfs = Vec::new();
                let mut interior_pts = Vec::new();
                let mut target_lid = self.doc.active_layer;
                for id in self.doc.selected_ids_ordered() {
                    if let Some(o) = self.doc.objects.get(id) {
                        target_lid = o.layer_id;
                        match &o.geometry {
                            Geometry::Curve(c) => curves.push(c.clone()),
                            Geometry::Point(p) => interior_pts.push(*p),
                            Geometry::Surface(s) => ref_surfs.push(s.clone()),
                            _ => {}
                        }
                    }
                }
                if curves.is_empty() && interior_pts.len() < 3 {
                    self.log(
                        "Select boundary curves (and optional interior points or reference surfaces) to run Patch [spans] [stiffness] [g0|g1|g2].",
                    );
                } else {
                    let bcs: Vec<PatchBoundaryConstraint> = curves
                        .into_iter()
                        .map(|c| {
                            let mid = c.evaluate(0.5 * (c.domain().0 + c.domain().1));
                            let ref_surface = ref_surfs
                                .iter()
                                .filter_map(|rs| {
                                    let (_, _, _, d) = rs.closest_point(mid);
                                    (d < 0.25).then(|| (rs.clone(), SurfaceEdge::UMin))
                                })
                                .next();
                            PatchBoundaryConstraint {
                                curve: c,
                                reference_surface: ref_surface,
                            }
                        })
                        .collect();
                    match patch_srf(&bcs, &interior_pts, &opts) {
                        Ok(rep) => {
                            self.doc.add_object_on_layer(
                                "Patch",
                                target_lid,
                                Geometry::Surface(rep.surface),
                            );
                            self.mark_scene_dirty();
                            self.log(format!(
                                "Created NURBS Patch ({}x{} spans, {}, max dev {:.5}).",
                                opts.spans_u,
                                opts.spans_v,
                                opts.continuity.label(),
                                rep.max_deviation
                            ));
                        }
                        Err(e) => self.log(format!("Patch error: {e}")),
                    }
                }
            }
            "rebuildsrf" => {
                let nu = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(8);
                let nv = args.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(nu);
                let du = args.get(2).and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                let dv = args.get(3).and_then(|v| v.parse::<usize>().ok()).unwrap_or(du);
                let ids: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter(|&id| {
                        self.doc
                            .objects
                            .get(id)
                            .is_some_and(|o| matches!(o.geometry, Geometry::Surface(_)))
                    })
                    .collect();
                if ids.is_empty() {
                    self.log("Select NURBS surface(s) to RebuildSrf [u_count] [v_count] [deg_u] [deg_v].");
                } else {
                    for id in ids {
                        let before = self.doc.objects[id].clone();
                        if let Geometry::Surface(s) = &before.geometry {
                            match rebuild_srf(s, nu, nv, du, dv) {
                                Ok(rep) => {
                                    self.doc.objects[id].geometry = Geometry::Surface(rep.surface);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("RebuildSrf", id, before);
                                    self.log(format!(
                                        "Rebuilt surface: {}x{} -> {}x{} CPs (deg {}x{}, max dev {:.5}).",
                                        rep.old_u, rep.old_v, rep.new_u, rep.new_v, du, dv, rep.max_deviation
                                    ));
                                }
                                Err(e) => self.log(format!("RebuildSrf error: {e}")),
                            }
                        }
                    }
                    self.mark_scene_dirty();
                }
            }
            "dupborder" | "dupedge" => {
                let is_border = cmd == "dupborder";
                let selected_ids = self.doc.selected_ids_ordered();
                let mut created = 0usize;
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let mut had_source = false;

                for id in selected_ids {
                    let Some(obj) = self.doc.objects.get(id) else {
                        continue;
                    };
                    let lid = obj.layer_id;
                    match &obj.geometry {
                        Geometry::Surface(s) => {
                            had_source = true;
                            if let Ok(edges) = s.boundary_curves() {
                                let curves = if is_border {
                                    NurbsCurve::join_curves(&edges, tol)
                                } else {
                                    edges.to_vec()
                                };
                                for c in curves {
                                    self.doc
                                        .add_object_on_layer("DupEdge", lid, Geometry::Curve(c));
                                    created += 1;
                                }
                            }
                        }
                        Geometry::BRep(b) => {
                            had_source = true;
                            let sub_edges: Vec<usize> = self
                                .selected_subobjects
                                .iter()
                                .filter_map(|&(oid, sub)| match sub {
                                    SubObjectRef::Edge(ei) if oid == id => Some(ei),
                                    _ => None,
                                })
                                .collect();
                            let raw_curves: Vec<NurbsCurve> = if is_border {
                                b.naked_edges()
                                    .into_iter()
                                    .filter_map(|ei| b.edges.get(ei).map(|e| e.curve.clone()))
                                    .collect()
                            } else if !sub_edges.is_empty() {
                                sub_edges
                                    .into_iter()
                                    .filter_map(|ei| b.edges.get(ei).map(|e| e.curve.clone()))
                                    .collect()
                            } else {
                                b.edges.iter().map(|e| e.curve.clone()).collect()
                            };
                            let curves = if is_border {
                                NurbsCurve::join_curves(&raw_curves, tol)
                            } else {
                                raw_curves
                            };
                            for c in curves {
                                self.doc
                                    .add_object_on_layer("DupEdge", lid, Geometry::Curve(c));
                                created += 1;
                            }
                        }
                        _ => {}
                    }
                }
                if !had_source {
                    self.log(format!(
                        "Select NURBS surface(s) or B-Rep(s) first, then run {cmd}."
                    ));
                } else {
                    self.mark_scene_dirty();
                    self.log(format!("Extracted {created} boundary/edge curve(s)."));
                }
            }
            "flip" => {
                let ids: Vec<_> = self.doc.selected_ids_ordered();
                let mut flipped = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    match &before.geometry {
                        Geometry::Surface(s) => {
                            self.doc.objects[id].geometry = Geometry::Surface(s.flip_normal());
                            self.doc.objects[id].refresh_aabb();
                            self.doc.record_object_modified("Flip", id, before);
                            flipped += 1;
                        }
                        Geometry::BRep(b) => {
                            let mut fb = b.clone();
                            fb.flip_normals();
                            self.doc.objects[id].geometry = Geometry::BRep(fb);
                            self.doc.objects[id].refresh_aabb();
                            self.doc.record_object_modified("Flip", id, before);
                            flipped += 1;
                        }
                        Geometry::Curve(c) => {
                            self.doc.objects[id].geometry = Geometry::Curve(c.reverse());
                            self.doc.objects[id].refresh_aabb();
                            self.doc.record_object_modified("Flip", id, before);
                            flipped += 1;
                        }
                        _ => {}
                    }
                }
                if flipped == 0 {
                    self.log("Select NURBS surface(s), B-Rep(s), or curve(s) to Flip.");
                } else {
                    self.mark_scene_dirty();
                    self.log(format!("Flipped {flipped} object(s)."));
                }
            }
            "extractisocurve" => {
                let is_u = !args.iter().any(|a| a.eq_ignore_ascii_case("v"));
                let frac = args
                    .iter()
                    .find_map(|a| a.parse::<f64>().ok())
                    .unwrap_or(0.5)
                    .clamp(0.0, 1.0);
                let surfs: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Surface(s) => Some((o.layer_id, s.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if surfs.is_empty() {
                    self.log("Select NURBS surface(s) to ExtractIsocurve [u|v] [t].");
                } else {
                    let mut created = 0;
                    for (lid, s) in &surfs {
                        let (d0, d1) = if is_u { s.domain_u() } else { s.domain_v() };
                        let p = d0 + (d1 - d0) * frac;
                        if let Ok(iso) = s.extract_isocurve(is_u, p) {
                            self.doc.add_object_on_layer(
                                "ExtractIsocurve",
                                *lid,
                                Geometry::Curve(iso),
                            );
                            created += 1;
                        }
                    }
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Extracted {created} {}-isocurve(s) at t={frac:.2}.",
                        if is_u { "U" } else { "V" }
                    ));
                }
            }
            "splitsrf" => {
                let split_u = !args.iter().any(|a| a.eq_ignore_ascii_case("v"));
                let frac = args
                    .iter()
                    .find_map(|a| a.parse::<f64>().ok())
                    .unwrap_or(0.5)
                    .clamp(0.01, 0.99);
                let ids: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter(|&id| {
                        self.doc
                            .objects
                            .get(id)
                            .is_some_and(|o| matches!(o.geometry, Geometry::Surface(_)))
                    })
                    .collect();
                if ids.is_empty() {
                    self.log("Select NURBS surface(s) to SplitSrf [u|v] [t].");
                } else {
                    let mut split_cnt = 0;
                    for id in ids {
                        let lid = self.doc.objects[id].layer_id;
                        if let Geometry::Surface(s) = &self.doc.objects[id].geometry {
                            let (d0, d1) = if split_u { s.domain_u() } else { s.domain_v() };
                            let p = d0 + (d1 - d0) * frac;
                            let res = if split_u { s.split_u(p) } else { s.split_v(p) };
                            if let Some((a, b)) = res {
                                self.doc.replace_object_with_many(
                                    "SplitSrf",
                                    id,
                                    vec![(lid, Geometry::Surface(a)), (lid, Geometry::Surface(b))],
                                );
                                split_cnt += 1;
                            }
                        }
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Split {split_cnt} surface(s) along {}-isocurve.",
                        if split_u { "U" } else { "V" }
                    ));
                }
            }
            "uvn" => {
                match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("on" | "1" | "true") => self.uvn_gumball_enabled = true,
                    Some("off" | "0" | "false") => self.uvn_gumball_enabled = false,
                    _ => self.uvn_gumball_enabled = !self.uvn_gumball_enabled,
                }
                self.log(format!(
                    "UVN Gumball: {}.",
                    if self.uvn_gumball_enabled { "On" } else { "Off" }
                ));
            }
            "softtransform" => {
                if args.is_empty() {
                    self.soft_transform_enabled = !self.soft_transform_enabled;
                } else {
                    for arg in &args {
                        match arg.to_ascii_lowercase().as_str() {
                            "on" | "1" | "true" => self.soft_transform_enabled = true,
                            "off" | "0" | "false" => self.soft_transform_enabled = false,
                            "free" => self.soft_transform_lock = BoundaryLock::Free,
                            "g0" => self.soft_transform_lock = BoundaryLock::FixG0,
                            "g1" => self.soft_transform_lock = BoundaryLock::FixG1,
                            "g2" => self.soft_transform_lock = BoundaryLock::FixG2,
                            "smooth" => self.soft_transform_falloff = SoftTransformFalloff::Smooth,
                            "linear" => self.soft_transform_falloff = SoftTransformFalloff::Linear,
                            "sharp" => self.soft_transform_falloff = SoftTransformFalloff::Sharp,
                            _ => {
                                if let Ok(r) = arg.parse::<f64>() {
                                    self.soft_transform_radius = r.clamp(0.1, 200.0);
                                    self.soft_transform_enabled = true;
                                }
                            }
                        }
                    }
                }
                self.log(format!(
                    "NURBS Soft Transform: {} (radius={:.2}, lock={:?}, falloff={:?}).",
                    if self.soft_transform_enabled { "On" } else { "Off" },
                    self.soft_transform_radius,
                    self.soft_transform_lock,
                    self.soft_transform_falloff
                ));
            }
            "booleanunion" | "booleandifference" | "booleanintersection" => {
                let op = match cmd.as_str() {
                    "booleanunion" => BooleanOp::Union,
                    "booleandifference" => BooleanOp::Difference,
                    _ => BooleanOp::Intersection,
                };
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ordered = self.doc.selected_ids_ordered();
                let mut breps: Vec<(cad_doc::LayerId, BRep)> = Vec::new();
                let mut all_brep_capable = ordered.len() >= 2;
                let mut meshes: Vec<(cad_doc::LayerId, cad_geom::TriangleMesh)> = Vec::new();
                for id in &ordered {
                    let Some(o) = self.doc.objects.get(*id) else {
                        continue;
                    };
                    match &o.geometry {
                        Geometry::BRep(b) => {
                            breps.push((o.layer_id, b.clone()));
                            meshes.push((o.layer_id, b.tessellate_display()));
                        }
                        Geometry::Surface(s) => {
                            let b = BRep::from_surface(s.clone());
                            meshes.push((o.layer_id, s.tessellate_display()));
                            breps.push((o.layer_id, b));
                        }
                        Geometry::Mesh(m) => {
                            all_brep_capable = false;
                            meshes.push((o.layer_id, m.clone()));
                        }
                        _ => {
                            all_brep_capable = false;
                        }
                    }
                }
                if all_brep_capable && breps.len() >= 2 {
                    let lid = breps[0].0;
                    let mut acc = breps[0].1.clone();
                    let mut err_msg = None;
                    for (_, b) in &breps[1..] {
                        match brep_boolean(&acc, b, op, tol) {
                            Ok(next) => acc = next,
                            Err(e) => {
                                err_msg = Some(format!("{e}"));
                                break;
                            }
                        }
                    }
                    if let Some(err) = err_msg {
                        self.log(format!("B-Rep Boolean error: {err}"));
                    } else {
                        let vol_note = if acc.is_solid() {
                            format!(" (solid, vol={:.3})", acc.volume_approx())
                        } else {
                            format!(" (open, {} naked)", acc.naked_edges().len())
                        };
                        let n_ops = breps.len();
                        self.doc.remove_selected();
                        self.doc
                            .add_object_on_layer("BooleanBRep", lid, Geometry::BRep(acc));
                        self.prune_points_on();
                        self.mark_scene_dirty();
                        self.log(format!("Completed B-Rep {cmd} on {n_ops} solids{vol_note}."));
                    }
                } else if meshes.len() < 2 {
                    self.log("Select 2 or more solids/surfaces for Boolean operation.");
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
                    self.log(format!("Completed {cmd} on {} solids.", meshes.len()));
                }
            }
            "booleansplit" => {
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ops: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::BRep(b) => Some((id, o.layer_id, b.clone())),
                            Geometry::Surface(s) => {
                                Some((id, o.layer_id, BRep::from_surface(s.clone())))
                            }
                            _ => None,
                        })
                    })
                    .collect();
                if ops.len() < 2 {
                    self.log("Select target solid first, then cutter solid for BooleanSplit.");
                } else {
                    let (target_id, lid, ref a) = ops[0];
                    match brep_boolean_split(a, &ops[1].2, tol) {
                        Ok(pieces) => {
                            let cnt = pieces.len();
                            let reps = pieces
                                .into_iter()
                                .map(|b| (lid, Geometry::BRep(b)))
                                .collect();
                            self.doc
                                .replace_object_with_many("BooleanSplit", target_id, reps);
                            self.prune_points_on();
                            self.mark_scene_dirty();
                            self.log(format!("BooleanSplit created {cnt} B-Rep piece(s)."));
                        }
                        Err(e) => self.log(format!("BooleanSplit error: {e}")),
                    }
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
                self.selected_cps.clear();
                self.uvn_drag = None;
                self.uvn_prompt = None;
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
                    .select_by_kind(|g| matches!(g, Geometry::Surface(_) | Geometry::BRep(_)));
                self.mark_doc_dirty();
                self.log(format!("Selected {n} surface(s)/B-Rep(s)."));
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
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ordered = self.doc.selected_ids_ordered();
                let mut breps = Vec::new();
                let mut first_brep_layer = None;
                let mut curves = Vec::new();
                for id in ordered {
                    let Some(obj) = self.doc.objects.get(id) else {
                        continue;
                    };
                    match &obj.geometry {
                        Geometry::BRep(b) => {
                            if first_brep_layer.is_none() {
                                first_brep_layer = Some(obj.layer_id);
                            }
                            breps.push(b.clone());
                        }
                        Geometry::Surface(s) => {
                            if first_brep_layer.is_none() {
                                first_brep_layer = Some(obj.layer_id);
                            }
                            breps.push(BRep::from_surface(s.clone()));
                        }
                        Geometry::Curve(c) => curves.push(c.clone()),
                        _ => {}
                    }
                }
                if breps.len() >= 2 {
                    let lid = first_brep_layer.unwrap_or(self.doc.active_layer);
                    let joined = BRep::join_many(&breps, tol);
                    let status = if joined.is_solid() {
                        "closed solid".to_string()
                    } else {
                        format!("{} naked edge(s)", joined.naked_edges().len())
                    };
                    let removed = self.doc.remove_selected();
                    self.doc
                        .add_object_on_layer("JoinBRep", lid, Geometry::BRep(joined));
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Joined {removed} surface(s)/B-Rep(s) into 1 B-Rep ({status})."
                    ));
                } else if curves.len() >= 2 {
                    let joined = NurbsCurve::join_curves(&curves, tol);
                    let removed = self.doc.remove_selected();
                    let count = joined.len();
                    for c in joined {
                        self.doc.add_object("Join", Geometry::Curve(c));
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Joined {removed} curve(s) into {count} curve(s)."));
                } else {
                    self.log("Select 2 or more curves, surfaces, or B-Reps to Join.");
                }
            }
            "explode" => {
                let ids = self.doc.selected_ids_ordered();
                let mut exploded_objs = 0usize;
                let mut total_pieces = 0usize;
                for id in ids {
                    let Some(obj) = self.doc.objects.get(id) else {
                        continue;
                    };
                    let lid = obj.layer_id;
                    let reps: Option<Vec<(cad_doc::LayerId, Geometry)>> = match &obj.geometry {
                        Geometry::BRep(b) if b.faces.len() > 1 => Some(
                            b.explode()
                                .into_iter()
                                .map(|piece| {
                                    let geom = if piece.faces.len() == 1
                                        && !piece.faces[0].is_trimmed()
                                    {
                                        Geometry::Surface(piece.faces[0].surface.clone())
                                    } else {
                                        Geometry::BRep(piece)
                                    };
                                    (lid, geom)
                                })
                                .collect(),
                        ),
                        Geometry::Curve(c) => {
                            let segs = c.explode_segments();
                            if segs.len() > 1 {
                                Some(
                                    segs.into_iter()
                                        .map(|s| (lid, Geometry::Curve(s)))
                                        .collect(),
                                )
                            } else {
                                None
                            }
                        }
                        _ => None,
                    };
                    if let Some(pieces) = reps {
                        if !pieces.is_empty() {
                            total_pieces += pieces.len();
                            exploded_objs += 1;
                            self.doc.replace_object_with_many("Explode", id, pieces);
                        }
                    }
                }
                if exploded_objs == 0 {
                    self.log("Select multi-face B-Rep(s) or polyline(s) to Explode.");
                } else {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Exploded {exploded_objs} object(s) into {total_pieces} piece(s)."
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
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let surface_targets: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::BRep(b) => Some((id, o.layer_id, b.clone())),
                            Geometry::Surface(s) => {
                                Some((id, o.layer_id, BRep::from_surface(s.clone())))
                            }
                            _ => None,
                        })
                    })
                    .collect();
                let cutter_curves: Vec<NurbsCurve> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();

                if !surface_targets.is_empty() && !cutter_curves.is_empty() {
                    let mut split_count = 0usize;
                    let mut piece_total = 0usize;
                    for (id, lid, brep) in surface_targets {
                        let orig_len = brep.faces.len();
                        let mut frags = brep.faces;
                        for cutter in &cutter_curves {
                            let mut next = Vec::new();
                            for f in frags {
                                let (_, uv) =
                                    project_curve_to_surface(cutter, &f.surface, cplane.z_axis, 32)
                                        .unwrap_or_else(|| {
                                            pull_curve_to_surface(cutter, &f.surface, 32)
                                        });
                                next.extend(split_face_with_uv_curve(
                                    &f,
                                    &uv,
                                    cutter.is_closed(),
                                    tol,
                                ));
                            }
                            frags = next;
                        }
                        if frags.len() > orig_len {
                            split_count += 1;
                            piece_total += frags.len();
                            let reps = frags
                                .into_iter()
                                .map(|f| (lid, Geometry::BRep(BRep::from_faces(vec![f], tol))))
                                .collect();
                            self.doc.replace_object_with_many("Split", id, reps);
                        }
                    }
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    if split_count > 0 {
                        self.log(format!(
                            "Split {split_count} B-Rep/surface(s) into {piece_total} trimmed piece(s)."
                        ));
                    } else {
                        self.log("Selected curve(s) did not split any selected face.");
                    }
                    return;
                }

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
                    self.log("Select 1 curve (interactive split), 2+ curves, or Surface/B-Rep + Curve to Split.");
                } else if selected.len() == 1 {
                    if let Some(prompt) = self.session.start_tool("splitcurve", &self.doc) {
                        self.log(prompt);
                    }
                } else {
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
            "trim" => {
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let surface_targets: Vec<_> = self
                    .doc
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::BRep(b) => Some((id, b.clone())),
                            Geometry::Surface(s) => Some((id, BRep::from_surface(s.clone()))),
                            _ => None,
                        })
                    })
                    .collect();
                let cutter_curves: Vec<NurbsCurve> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                if surface_targets.is_empty() || cutter_curves.is_empty() {
                    self.log("Select Surface/B-Rep + cutting Curve(s) to Trim.");
                    return;
                }
                let custom_discard = if args.len() >= 3 {
                    if let (Ok(x), Ok(y), Ok(z)) = (
                        args[0].parse::<f64>(),
                        args[1].parse::<f64>(),
                        args[2].parse::<f64>(),
                    ) {
                        Some(DVec3::new(x, y, z))
                    } else {
                        None
                    }
                } else {
                    None
                };
                let mut trimmed_count = 0usize;
                for (id, brep) in surface_targets {
                    let mut new_faces = Vec::with_capacity(brep.faces.len());
                    let mut changed = false;
                    for face in &brep.faces {
                        let mut cur = face.clone();
                        for cutter in &cutter_curves {
                            let discard_pt = custom_discard
                                .unwrap_or_else(|| cutter.bounding_box().center());
                            if let Some(updated) = trim_face_with_curve(
                                &cur,
                                cutter,
                                discard_pt,
                                Some(cplane.z_axis),
                                tol,
                            ) {
                                cur = updated;
                                changed = true;
                            }
                        }
                        new_faces.push(cur);
                    }
                    if changed {
                        let before = self.doc.objects[id].clone();
                        self.doc.objects[id].geometry =
                            Geometry::BRep(BRep::from_faces(new_faces, tol));
                        self.doc.objects[id].refresh_aabb();
                        self.doc.record_object_modified("Trim", id, before);
                        trimmed_count += 1;
                    }
                }
                if trimmed_count > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Trimmed {trimmed_count} B-Rep/surface(s)."));
                } else {
                    self.log("Cutting curve(s) did not intersect selected surface face(s).");
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
                    .selected_ids_ordered()
                    .into_iter()
                    .filter_map(|id| {
                        self.doc.objects.get(id).and_then(|o| match &o.geometry {
                            Geometry::Curve(c) => Some((id, o.layer_id, c.clone())),
                            _ => None,
                        })
                    })
                    .collect();
                if curves.len() != 2 {
                    self.log("Select exactly 2 curves to BlendCrv [g0|g1|g2|g3] [bulge].");
                } else {
                    let (pid_a, lid, ref ca) = curves[0];
                    let (pid_b, _, ref cb) = curves[1];
                    let (at_end_a, at_end_b) = closest_endpoint_pair(ca, cb);
                    match blend_curves(ca, at_end_a, cb, at_end_b, cont, bulge, bulge) {
                        Ok(blended) => {
                            let cid = self
                                .doc
                                .add_object_on_layer("BlendCrv", lid, Geometry::Curve(blended));
                            self.doc.add_history_record(
                                cid,
                                vec![pid_a, pid_b],
                                HistoryOp::BlendCrv {
                                    continuity: cont,
                                    bulge,
                                },
                            );
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
                let n0 = args.first().and_then(|v| v.parse::<usize>().ok()).unwrap_or(10);
                let d0 = args.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                let ids = self.doc.selected_ids_ordered();
                let mut touched = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    match &before.geometry {
                        Geometry::Curve(c) => {
                            touched += 1;
                            match rebuild_uniform(c, n0, d0) {
                                Ok(rep) => {
                                    self.doc.objects[id].geometry = Geometry::Curve(rep.curve);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("Rebuild", id, before);
                                    self.log(format!(
                                        "Rebuilt curve: {} -> {} CPs (deg {}, max dev {:.5}).",
                                        rep.old_cp_count,
                                        rep.new_cp_count,
                                        d0,
                                        rep.max_deviation
                                    ));
                                }
                                Err(e) => self.log(format!("Rebuild error: {e}")),
                            }
                        }
                        Geometry::Surface(s) => {
                            touched += 1;
                            let nv = args.get(1).and_then(|v| v.parse::<usize>().ok()).unwrap_or(n0);
                            let du = args.get(2).and_then(|v| v.parse::<usize>().ok()).unwrap_or(3);
                            let dv = args.get(3).and_then(|v| v.parse::<usize>().ok()).unwrap_or(du);
                            match rebuild_srf(s, n0, nv, du, dv) {
                                Ok(rep) => {
                                    self.doc.objects[id].geometry = Geometry::Surface(rep.surface);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("RebuildSrf", id, before);
                                    self.log(format!(
                                        "Rebuilt surface: {}x{} -> {}x{} CPs (max dev {:.5}).",
                                        rep.old_u, rep.old_v, rep.new_u, rep.new_v, rep.max_deviation
                                    ));
                                }
                                Err(e) => self.log(format!("RebuildSrf error: {e}")),
                            }
                        }
                        _ => {}
                    }
                }
                if touched == 0 {
                    self.log("Select curve(s) or NURBS surface(s) to Rebuild.");
                } else {
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
                let target_u = args
                    .first()
                    .and_then(|v| v.parse::<usize>().ok())
                    .unwrap_or(3)
                    .clamp(1, 7);
                let target_v = args
                    .get(1)
                    .and_then(|v| v.parse::<usize>().ok())
                    .unwrap_or(target_u)
                    .clamp(1, 7);
                let ids = self.doc.selected_ids_ordered();
                let mut touched = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    match &before.geometry {
                        Geometry::Curve(c) => {
                            touched += 1;
                            if target_u > c.degree {
                                if let Ok(elevated) = c.elevate_degree(target_u - c.degree) {
                                    self.doc.objects[id].geometry = Geometry::Curve(elevated);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ChangeDegree", id, before);
                                    self.log(format!("Elevated curve to degree {target_u}."));
                                }
                            } else if target_u < c.degree {
                                if let Ok(rep) = rebuild_uniform(c, c.control_points.len(), target_u)
                                {
                                    self.doc.objects[id].geometry = Geometry::Curve(rep.curve);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ChangeDegree", id, before);
                                    self.log(format!(
                                        "Rebuilt curve to degree {target_u} (max dev {:.5}).",
                                        rep.max_deviation
                                    ));
                                }
                            }
                        }
                        Geometry::Surface(s) => {
                            touched += 1;
                            if target_u >= s.degree_u && target_v >= s.degree_v {
                                if let Ok(elevated) = s
                                    .elevate_degree(target_u - s.degree_u, target_v - s.degree_v)
                                {
                                    self.doc.objects[id].geometry = Geometry::Surface(elevated);
                                    self.doc.objects[id].refresh_aabb();
                                    self.doc.record_object_modified("ChangeDegree", id, before);
                                    self.log(format!(
                                        "Elevated surface to degree {target_u}x{target_v}."
                                    ));
                                }
                            } else if let Ok(rep) = rebuild_srf(
                                s,
                                s.count_u.max(target_u + 1),
                                s.count_v.max(target_v + 1),
                                target_u,
                                target_v,
                            ) {
                                self.doc.objects[id].geometry = Geometry::Surface(rep.surface);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified("ChangeDegree", id, before);
                                self.log(format!(
                                    "Rebuilt surface to degree {target_u}x{target_v} (max dev {:.5}).",
                                    rep.max_deviation
                                ));
                            }
                        }
                        _ => {}
                    }
                }
                if touched == 0 {
                    self.log("Select curve(s) or surface(s) to ChangeDegree [deg_u] [deg_v].");
                } else {
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
                let mut total_vol = 0.0;
                let mut solid_count = 0usize;
                let mut count = 0usize;
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        match &obj.geometry {
                            Geometry::Curve(c) if c.is_closed() => {
                                total += c.planar_area_approx(&cplane, 64);
                                count += 1;
                            }
                            Geometry::Surface(s) => {
                                total += s.area_approx();
                                count += 1;
                            }
                            Geometry::BRep(b) => {
                                total += b.area_approx();
                                if b.is_solid() {
                                    total_vol += b.volume_approx();
                                    solid_count += 1;
                                }
                                count += 1;
                            }
                            _ => {}
                        }
                    }
                }
                if count == 0 {
                    self.log("Select closed planar curve(s), NURBS surface(s), or B-Rep(s) to measure Area.");
                } else if solid_count > 0 {
                    self.log(format!(
                        "Total area of {count} object(s): {total:.4} | Solid volume ({solid_count}): {total_vol:.4}"
                    ));
                } else {
                    self.log(format!(
                        "Total area of {count} object(s): {total:.4}"
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
            "rendered" | "pbr" => self.set_active_viewport_display_mode(DisplayMode::Rendered),
            "concept" | "clay" => self.set_active_viewport_display_mode(DisplayMode::Concept),
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
                    gdt: None,
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
                self.selected_cps.clear();
                self.selected_subobjects.clear();
                self.uvn_drag = None;
                self.uvn_prompt = None;
                self.curvature_graph_objects.clear();
                self.grip_drag = None;
                self.gumball_drag = None;
                self.gumball_prompt = None;
                self.marquee = None;
                self.reset_phase4_interaction_state();
                self.surface_analysis = SurfaceAnalysisConfig::default();
                self.show_naked_edges = false;
                self.show_non_manifold_edges = false;
                self.edge_continuity_enabled = false;
                self.edge_continuity_reports.clear();
                self.last_mass_properties = None;
                self.workspace_view = WorkspaceViewMode::Model;
                self.cached_layout_projections.clear();
                self.cached_clipping_overlay = None;
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
            "intersect" => {
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ordered = self.doc.selected_ids_ordered();
                let mut breps: Vec<(cad_doc::LayerId, BRep)> = Vec::new();
                let mut surfs: Vec<(cad_doc::LayerId, NurbsSurface)> = Vec::new();
                let mut curves: Vec<(cad_doc::LayerId, NurbsCurve)> = Vec::new();
                for id in ordered {
                    let Some(o) = self.doc.objects.get(id) else {
                        continue;
                    };
                    match &o.geometry {
                        Geometry::BRep(b) => breps.push((o.layer_id, b.clone())),
                        Geometry::Surface(s) => surfs.push((o.layer_id, s.clone())),
                        Geometry::Curve(c) => curves.push((o.layer_id, c.clone())),
                        _ => {}
                    }
                }
                if breps.len() + surfs.len() >= 2 {
                    let mut all_breps = breps;
                    for (lid, s) in surfs {
                        all_breps.push((lid, BRep::from_surface(s)));
                    }
                    let mut created = 0usize;
                    for i in 0..all_breps.len() {
                        for j in (i + 1)..all_breps.len() {
                            let lid = all_breps[i].0;
                            if all_breps[i].1.faces.len() == 1
                                && !all_breps[i].1.faces[0].is_trimmed()
                                && all_breps[j].1.faces.len() == 1
                                && !all_breps[j].1.faces[0].is_trimmed()
                            {
                                for sic in intersect_surfaces(
                                    &all_breps[i].1.faces[0].surface,
                                    &all_breps[j].1.faces[0].surface,
                                    tol,
                                ) {
                                    self.doc.add_object_on_layer(
                                        "IntersectCrv",
                                        lid,
                                        Geometry::Curve(sic.curve_3d),
                                    );
                                    created += 1;
                                }
                            } else {
                                for c in intersect_breps(&all_breps[i].1, &all_breps[j].1, tol) {
                                    self.doc.add_object_on_layer(
                                        "IntersectCrv",
                                        lid,
                                        Geometry::Curve(c),
                                    );
                                    created += 1;
                                }
                            }
                        }
                    }
                    if created > 0 {
                        self.mark_scene_dirty();
                        self.log(format!("Intersect created {created} SSI curve(s)."));
                    } else {
                        self.log("No surface/B-Rep intersections found.");
                    }
                } else if curves.len() >= 2 {
                    let mut created = 0usize;
                    for i in 0..curves.len() {
                        for j in (i + 1)..curves.len() {
                            for hit in intersect_curves(&curves[i].1, &curves[j].1, tol) {
                                self.doc.add_object_on_layer(
                                    "IntersectPt",
                                    curves[i].0,
                                    Geometry::Point(hit.point),
                                );
                                created += 1;
                            }
                        }
                    }
                    if created > 0 {
                        self.mark_scene_dirty();
                        self.log(format!("Intersect created {created} intersection point(s)."));
                    } else {
                        self.log("No curve-curve intersections found.");
                    }
                } else {
                    self.log("Select 2+ surfaces/B-Reps or 2+ curves to Intersect.");
                }
            }
            "cap" => {
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut capped = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    let brep = match &before.geometry {
                        Geometry::BRep(b) => Some(b.clone()),
                        Geometry::Surface(s) => Some(BRep::from_surface(s.clone())),
                        _ => None,
                    };
                    if let Some(b) = brep {
                        match cap_planar_holes(&b, tol) {
                            Ok(res) => {
                                self.doc.objects[id].geometry = Geometry::BRep(res);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified("Cap", id, before);
                                capped += 1;
                            }
                            Err(e) => self.log(format!("Cap error: {e}")),
                        }
                    }
                }
                if capped > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Capped planar holes on {capped} object(s)."));
                } else if self.doc.selected_ids_ordered().is_empty() {
                    self.log("Select open B-Rep(s) or surface(s) to Cap.");
                }
            }
            "untrim" => {
                let ids = self.doc.selected_ids_ordered();
                let mut count = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let sub_faces: Vec<usize> = self
                            .selected_subobjects
                            .iter()
                            .filter_map(|&(oid, sub)| match sub {
                                SubObjectRef::Face(fi) if oid == id => Some(fi),
                                _ => None,
                            })
                            .collect();
                        let updated = untrim_brep(
                            b,
                            if sub_faces.is_empty() {
                                None
                            } else {
                                Some(sub_faces.as_slice())
                            },
                        );
                        self.doc.objects[id].geometry = Geometry::BRep(updated);
                        self.doc.objects[id].refresh_aabb();
                        self.doc.record_object_modified("Untrim", id, before);
                        count += 1;
                    }
                }
                if count > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Untrimmed {count} B-Rep(s)."));
                } else {
                    self.log("Select B-Rep(s) (or sub-object faces) to Untrim.");
                }
            }
            "filletsrf" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.5);
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let sub_faces: Vec<(cad_doc::LayerId, NurbsSurface)> = self
                    .selected_subobjects
                    .iter()
                    .filter_map(|&(oid, sub)| match sub {
                        SubObjectRef::Face(fi) => self.doc.objects.get(oid).and_then(|o| {
                            if let Geometry::BRep(b) = &o.geometry {
                                b.faces.get(fi).map(|f| (o.layer_id, f.surface.clone()))
                            } else {
                                None
                            }
                        }),
                        _ => None,
                    })
                    .collect();

                let surf_pair: Option<(cad_doc::LayerId, NurbsSurface, NurbsSurface, Option<(ObjectId, ObjectId)>)> =
                    if sub_faces.len() >= 2 {
                        Some((sub_faces[0].0, sub_faces[0].1.clone(), sub_faces[1].1.clone(), None))
                    } else {
                        let objs: Vec<_> = self
                            .doc
                            .selected_ids_ordered()
                            .into_iter()
                            .filter_map(|id| {
                                self.doc.objects.get(id).and_then(|o| match &o.geometry {
                                    Geometry::Surface(s) => Some((Some(id), o.layer_id, s.clone())),
                                    Geometry::BRep(b) if !b.faces.is_empty() => {
                                        Some((None, o.layer_id, b.faces[0].surface.clone()))
                                    }
                                    _ => None,
                                })
                            })
                            .collect();
                        if objs.len() >= 2 {
                            let ids = match (objs[0].0, objs[1].0) {
                                (Some(a), Some(b)) => Some((a, b)),
                                _ => None,
                            };
                            Some((objs[0].1, objs[0].2.clone(), objs[1].2.clone(), ids))
                        } else {
                            None
                        }
                    };

                if let Some((lid, srf_a, srf_b, srf_ids)) = surf_pair {
                    match fillet_srf(&srf_a, &srf_b, r, None, None, true, tol) {
                        Ok(res) => {
                            self.doc
                                .add_object_on_layer("FilletSrf", lid, Geometry::Surface(res.fillet));
                            if let Some((id_a, id_b)) = srf_ids {
                                if let Some(ta) = res.trimmed_a {
                                    let before_a = self.doc.objects[id_a].clone();
                                    self.doc.objects[id_a].geometry =
                                        Geometry::BRep(BRep::from_faces(vec![ta], tol));
                                    self.doc.objects[id_a].refresh_aabb();
                                    self.doc.record_object_modified("FilletSrf", id_a, before_a);
                                }
                                if let Some(tb) = res.trimmed_b {
                                    let before_b = self.doc.objects[id_b].clone();
                                    self.doc.objects[id_b].geometry =
                                        Geometry::BRep(BRep::from_faces(vec![tb], tol));
                                    self.doc.objects[id_b].refresh_aabb();
                                    self.doc.record_object_modified("FilletSrf", id_b, before_b);
                                }
                            }
                            self.prune_points_on();
                            self.mark_scene_dirty();
                            self.log(format!("Created rolling-ball FilletSrf (r={r:.3})."));
                        }
                        Err(e) => self.log(format!("FilletSrf error: {e}")),
                    }
                } else {
                    self.log("Select 2 surfaces (or Ctrl+Shift+Click 2 B-Rep faces) for FilletSrf.");
                }
            }
            "filletedge" | "chamferedge" => {
                let is_fillet = cmd == "filletedge";
                let dist = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.3);
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut done = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let mut edge_indices = Vec::new();
                        for &(oid, sub) in &self.selected_subobjects {
                            if oid == id {
                                if let SubObjectRef::Edge(ei) = sub {
                                    for ce in tangent_edge_chain(b, ei, 5.0_f64.to_radians()) {
                                        if !edge_indices.contains(&ce) {
                                            edge_indices.push(ce);
                                        }
                                    }
                                }
                            }
                        }
                        if edge_indices.is_empty() && !b.edges.is_empty() {
                            edge_indices.push(0);
                        }
                        let res = if is_fillet {
                            fillet_brep_edges(b, &edge_indices, dist, tol)
                        } else {
                            chamfer_brep_edges(b, &edge_indices, dist, tol)
                        };
                        match res {
                            Ok(updated) => {
                                let label = if is_fillet { "FilletEdge" } else { "ChamferEdge" };
                                self.doc.objects[id].geometry = Geometry::BRep(updated);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified(label, id, before);
                                done += 1;
                            }
                            Err(e) => self.log(format!("{cmd} error: {e}")),
                        }
                    }
                }
                if done > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!(
                        "{} applied (d={dist:.3}) on {done} B-Rep(s).",
                        if is_fillet { "FilletEdge" } else { "ChamferEdge" }
                    ));
                } else if self.doc.selected_ids_ordered().is_empty() {
                    self.log("Select B-Rep solid (or Ctrl+Shift+Click edges) first.");
                }
            }
            "shell" => {
                let thickness = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.2);
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut done = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let mut open_faces: Vec<usize> = self
                            .selected_subobjects
                            .iter()
                            .filter_map(|&(oid, sub)| match sub {
                                SubObjectRef::Face(fi) if oid == id => Some(fi),
                                _ => None,
                            })
                            .collect();
                        if open_faces.is_empty() && !b.faces.is_empty() {
                            open_faces.push(0);
                        }
                        match shell_brep(b, &open_faces, thickness, tol) {
                            Ok(shelled) => {
                                self.doc.objects[id].geometry = Geometry::BRep(shelled);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified("Shell", id, before);
                                done += 1;
                            }
                            Err(e) => self.log(format!("Shell error: {e}")),
                        }
                    }
                }
                if done > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Shelled {done} solid(s) (t={thickness:.3})."));
                } else if self.doc.selected_ids_ordered().is_empty() {
                    self.log("Select a solid (or Ctrl+Shift+Click open faces) for Shell.");
                }
            }
            "pushpull" => {
                let dist = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut done = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let face_idx = self
                            .selected_subobjects
                            .iter()
                            .find_map(|&(oid, sub)| match sub {
                                SubObjectRef::Face(fi) if oid == id => Some(fi),
                                _ => None,
                            })
                            .unwrap_or(0);
                        match push_pull_face(b, face_idx, dist, tol) {
                            Ok(updated) => {
                                self.doc.objects[id].geometry = Geometry::BRep(updated);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified("PushPull", id, before);
                                done += 1;
                            }
                            Err(e) => self.log(format!("PushPull error: {e}")),
                        }
                    }
                }
                if done > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("PushPull offset {done} B-Rep face(s) by {dist:.3}."));
                } else if self.doc.selected_ids_ordered().is_empty() {
                    self.log("Select a B-Rep (or Ctrl+Shift+Click a face) for PushPull.");
                }
            }
            "inset" => {
                let dist = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.25);
                let depth = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut done = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let face_idx = self
                            .selected_subobjects
                            .iter()
                            .find_map(|&(oid, sub)| match sub {
                                SubObjectRef::Face(fi) if oid == id => Some(fi),
                                _ => None,
                            })
                            .unwrap_or(0);
                        match inset_face(b, face_idx, dist, depth, tol) {
                            Ok(updated) => {
                                self.doc.objects[id].geometry = Geometry::BRep(updated);
                                self.doc.objects[id].refresh_aabb();
                                self.doc.record_object_modified("Inset", id, before);
                                done += 1;
                            }
                            Err(e) => self.log(format!("Inset error: {e}")),
                        }
                    }
                }
                if done > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Inset applied on {done} B-Rep face(s) (d={dist:.3}, depth={depth:.3})."));
                } else if self.doc.selected_ids_ordered().is_empty() {
                    self.log("Select a B-Rep (or Ctrl+Shift+Click a face) for Inset.");
                }
            }
            "extractsrf" => {
                let tol = self.doc.tolerances.absolute.max(1e-3);
                let ids = self.doc.selected_ids_ordered();
                let mut extracted_count = 0usize;
                for id in ids {
                    let face_indices: Vec<usize> = self
                        .selected_subobjects
                        .iter()
                        .filter_map(|&(oid, sub)| match sub {
                            SubObjectRef::Face(fi) if oid == id => Some(fi),
                            _ => None,
                        })
                        .collect();
                    if face_indices.is_empty() {
                        continue;
                    }
                    let before = self.doc.objects[id].clone();
                    if let Geometry::BRep(b) = &before.geometry {
                        let lid = before.layer_id;
                        let mut remaining = Vec::new();
                        for (fi, f) in b.faces.iter().enumerate() {
                            if face_indices.contains(&fi) {
                                let geom = if !f.is_trimmed() {
                                    Geometry::Surface(f.surface.clone())
                                } else {
                                    Geometry::BRep(BRep::from_faces(vec![f.clone()], tol))
                                };
                                self.doc.add_object_on_layer("ExtractSrf", lid, geom);
                                extracted_count += 1;
                            } else {
                                remaining.push(f.clone());
                            }
                        }
                        if remaining.is_empty() {
                            self.doc
                                .replace_object_with_many("ExtractSrf", id, Vec::new());
                        } else {
                            self.doc.objects[id].geometry =
                                Geometry::BRep(BRep::from_faces(remaining, tol));
                            self.doc.objects[id].refresh_aabb();
                            self.doc.record_object_modified("ExtractSrf", id, before);
                        }
                    }
                }
                if extracted_count > 0 {
                    self.selected_subobjects.clear();
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Extracted {extracted_count} B-Rep face(s)."));
                } else {
                    self.log("Ctrl+Shift+Click B-Rep face(s) first, then run ExtractSrf.");
                }
            }
            "tobrep" => {
                let ids = self.doc.selected_ids_ordered();
                let mut count = 0usize;
                for id in ids {
                    let before = self.doc.objects[id].clone();
                    if let Geometry::Surface(s) = &before.geometry {
                        self.doc.objects[id].geometry =
                            Geometry::BRep(BRep::from_surface(s.clone()));
                        self.doc.objects[id].refresh_aabb();
                        self.doc.record_object_modified("ToBRep", id, before);
                        count += 1;
                    }
                }
                if count > 0 {
                    self.prune_points_on();
                    self.mark_scene_dirty();
                    self.log(format!("Converted {count} surface(s) to B-Rep."));
                } else {
                    self.log("Select NURBS surface(s) to convert with ToBRep.");
                }
            }
            "showedges" => {
                match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("on" | "1" | "true") => self.show_naked_edges = true,
                    Some("off" | "0" | "false") => self.show_naked_edges = false,
                    _ => self.show_naked_edges = !self.show_naked_edges,
                }
                let mut naked_total = 0usize;
                for (_, obj) in &self.doc.objects {
                    if obj.visible {
                        if let Geometry::BRep(b) = &obj.geometry {
                            naked_total += b.naked_edges().len();
                        }
                    }
                }
                self.log(format!(
                    "ShowEdges (Naked): {} ({naked_total} naked edge(s) in visible B-Reps).",
                    if self.show_naked_edges { "On" } else { "Off" }
                ));
            }
            "autocplane" => {
                match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("on" | "1" | "true") => self.auto_cplane = true,
                    Some("off" | "0" | "false") => self.auto_cplane = false,
                    _ => self.auto_cplane = !self.auto_cplane,
                }
                self.update_auto_cplane();
                self.log(format!(
                    "AutoCPlane: {}.",
                    if self.auto_cplane { "On" } else { "Off" }
                ));
            }
            "smarttrack" => {
                match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("clear") => {
                        self.snap.clear_smart_track();
                        self.last_inference_guides.clear();
                        self.smart_track_hover = None;
                        self.log("Cleared SmartTrack temporary reference points.");
                    }
                    Some("on" | "1" | "true") => {
                        self.snap.smart_track = true;
                        self.log("SmartTrack: On.");
                    }
                    Some("off" | "0" | "false") => {
                        self.snap.smart_track = false;
                        self.snap.clear_smart_track();
                        self.last_inference_guides.clear();
                        self.smart_track_hover = None;
                        self.log("SmartTrack: Off.");
                    }
                    _ => {
                        self.snap.smart_track = !self.snap.smart_track;
                        if !self.snap.smart_track {
                            self.snap.clear_smart_track();
                            self.last_inference_guides.clear();
                            self.smart_track_hover = None;
                        }
                        self.log(format!(
                            "SmartTrack: {}.",
                            if self.snap.smart_track { "On" } else { "Off" }
                        ));
                    }
                }
            }
            "gumballalign" => {
                use crate::state::GumballAlignMode;
                self.gumball_align = match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("world") => GumballAlignMode::World,
                    Some("cplane") => GumballAlignMode::CPlane,
                    Some("object" | "obj") => GumballAlignMode::Object,
                    _ => match self.gumball_align {
                        GumballAlignMode::World => GumballAlignMode::CPlane,
                        GumballAlignMode::CPlane => GumballAlignMode::Object,
                        GumballAlignMode::Object => GumballAlignMode::World,
                    },
                };
                self.log(format!("Gumball alignment set to {:?}.", self.gumball_align));
            }
            "gumballrelocate" => {
                if args.len() >= 3 {
                    if let (Ok(x), Ok(y), Ok(z)) = (
                        args[0].parse::<f64>(),
                        args[1].parse::<f64>(),
                        args[2].parse::<f64>(),
                    ) {
                        self.gumball_custom_origin = Some(DVec3::new(x, y, z));
                        self.log(format!("Relocated Gumball origin to ({x:.3}, {y:.3}, {z:.3})."));
                    } else {
                        self.log("Usage: GumballRelocate <x> <y> <z> (or Ctrl+Drag Gumball center).");
                    }
                } else {
                    self.log("Hold Ctrl and drag the Gumball origin to relocate, or pass X Y Z / run GumballReset.");
                }
            }
            "gumballreset" => {
                self.gumball_custom_origin = None;
                self.log("Reset Gumball origin to selection centroid.");
            }
            "history" => {
                match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("on" | "1" | "true") => self.doc.record_history = true,
                    Some("off" | "0" | "false") => self.doc.record_history = false,
                    _ => self.doc.record_history = !self.doc.record_history,
                }
                self.log(format!(
                    "Modeling History recording: {}.",
                    if self.doc.record_history { "On" } else { "Off" }
                ));
            }
            "breakhistory" => {
                let ids = self.doc.selected_ids_ordered();
                if ids.is_empty() {
                    self.log("Select object(s) to break Modeling History.");
                } else {
                    let mut broken = 0usize;
                    for id in ids {
                        if self.doc.break_history_for(id) {
                            broken += 1;
                        }
                    }
                    if broken > 0 {
                        self.mark_doc_dirty();
                    }
                    self.log(format!("Broke Modeling History on {broken} object(s)."));
                }
            }
            "historypurge" => {
                let purged = self.doc.purge_broken_history();
                if purged > 0 {
                    self.mark_doc_dirty();
                }
                self.log(format!("Purged {purged} stale/broken history record(s)."));
            }
            "radialmenu" => {
                if self.radial_menu_pos.is_some() {
                    self.radial_menu_pos = None;
                } else {
                    self.radial_menu_pos = Some(eframe::egui::pos2(640.0, 400.0));
                }
            }
            "zebra" => {
                let was_curv = matches!(
                    self.surface_analysis.mode,
                    SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
                );
                if self.surface_analysis.mode == SurfaceAnalysisMode::Zebra && args.is_empty() {
                    self.surface_analysis.mode = SurfaceAnalysisMode::None;
                    self.log("Zebra analysis: Off.");
                } else {
                    if let Some(v) = args.first().and_then(|s| s.parse::<f32>().ok()) {
                        self.surface_analysis.zebra_stripes = v.clamp(2.0, 30.0);
                    }
                    self.surface_analysis.mode = SurfaceAnalysisMode::Zebra;
                    self.inspector_tab = InspectorTab::Analyze;
                    self.log("Zebra reflection stripe analysis active.");
                }
                if was_curv {
                    self.mark_doc_dirty();
                }
            }
            "emap" => {
                let was_curv = matches!(
                    self.surface_analysis.mode,
                    SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
                );
                if self.surface_analysis.mode == SurfaceAnalysisMode::EnvironmentMap && args.is_empty() {
                    self.surface_analysis.mode = SurfaceAnalysisMode::None;
                    self.log("Environment Map analysis: Off.");
                } else {
                    if let Some(p) = args.first().and_then(|s| s.parse::<u32>().ok()) {
                        self.surface_analysis.emap_preset = p.min(3);
                    }
                    self.surface_analysis.mode = SurfaceAnalysisMode::EnvironmentMap;
                    self.inspector_tab = InspectorTab::Analyze;
                    self.log("Environment Map (E-Map) analysis active.");
                }
                if was_curv {
                    self.mark_doc_dirty();
                }
            }
            "curvatureanalysis" | "gaussiancurvature" => {
                let already = self.surface_analysis.mode == SurfaceAnalysisMode::GaussianCurvature;
                self.surface_analysis.mode = if already {
                    SurfaceAnalysisMode::None
                } else {
                    SurfaceAnalysisMode::GaussianCurvature
                };
                self.mark_doc_dirty();
                self.inspector_tab = InspectorTab::Analyze;
                self.log(format!(
                    "Gaussian Curvature shader: {}.",
                    if already { "Off" } else { "On" }
                ));
            }
            "meancurvature" => {
                let already = self.surface_analysis.mode == SurfaceAnalysisMode::MeanCurvature;
                self.surface_analysis.mode = if already {
                    SurfaceAnalysisMode::None
                } else {
                    SurfaceAnalysisMode::MeanCurvature
                };
                self.mark_doc_dirty();
                self.inspector_tab = InspectorTab::Analyze;
                self.log(format!(
                    "Mean Curvature shader: {}.",
                    if already { "Off" } else { "On" }
                ));
            }
            "draftangle" => {
                let was_curv = matches!(
                    self.surface_analysis.mode,
                    SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
                );
                if self.surface_analysis.mode == SurfaceAnalysisMode::DraftAngle && args.is_empty() {
                    self.surface_analysis.mode = SurfaceAnalysisMode::None;
                    self.log("Draft Angle analysis: Off.");
                } else {
                    if let Some(deg) = args.first().and_then(|s| s.parse::<f32>().ok()) {
                        self.surface_analysis.draft_angle_deg = deg.clamp(0.2, 45.0);
                    }
                    self.surface_analysis.mode = SurfaceAnalysisMode::DraftAngle;
                    self.inspector_tab = InspectorTab::Analyze;
                    self.log(format!(
                        "Draft Angle analysis active (parting band ±{:.1}°).",
                        self.surface_analysis.draft_angle_deg
                    ));
                }
                if was_curv {
                    self.mark_doc_dirty();
                }
            }
            "analysisoff" => {
                let was_curv = matches!(
                    self.surface_analysis.mode,
                    SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
                );
                self.surface_analysis.mode = SurfaceAnalysisMode::None;
                if was_curv {
                    self.mark_doc_dirty();
                }
                self.log("Surface analysis shader turned off.");
            }
            "globaledgecontinuity" | "edgecontinuity" => {
                let enable = match args.first().map(|a| a.to_ascii_lowercase()).as_deref() {
                    Some("off" | "0" | "false") => false,
                    Some("on" | "1" | "true") => true,
                    _ => !self.edge_continuity_enabled,
                };
                self.edge_continuity_enabled = enable;
                if enable {
                    self.recompute_edge_continuity();
                    self.inspector_tab = InspectorTab::Analyze;
                    let total = self.edge_continuity_reports.len();
                    let g2 = self
                        .edge_continuity_reports
                        .iter()
                        .filter(|r| r.grade == cad_geom::ContinuityGrade::G2)
                        .count();
                    let g1 = self
                        .edge_continuity_reports
                        .iter()
                        .filter(|r| r.grade == cad_geom::ContinuityGrade::G1)
                        .count();
                    let g0 = self
                        .edge_continuity_reports
                        .iter()
                        .filter(|r| r.grade == cad_geom::ContinuityGrade::G0)
                        .count();
                    let gap = total.saturating_sub(g2 + g1 + g0);
                    self.log(format!(
                        "GlobalEdgeContinuity: {total} seam(s) -> G2:{g2}, G1:{g1}, G0:{g0}, Broken:{gap}."
                    ));
                } else {
                    self.log("GlobalEdgeContinuity overlay hidden.");
                }
            }
            "massproperties" | "volume" | "volumecentroid" => {
                self.recompute_mass_properties();
                self.inspector_tab = InspectorTab::Analyze;
                if let Some(mp) = &self.last_mass_properties {
                    let cm3_per_unit3 =
                        self.doc.units.scale_to(UnitSystem::Centimeters).powi(3);
                    let mass_g = mp.volume * cm3_per_unit3 * self.material_density_g_cm3;
                    let u_lbl = self.doc.units.label();
                    self.log(format!(
                        "MassProperties: Area={:.3} {u_lbl}^2, Vol={:.4} {u_lbl}^3, Mass={:.2} g, Centroid=({:.3}, {:.3}, {:.3}), I=({:.2}, {:.2}, {:.2}).",
                        mp.area,
                        mp.volume,
                        mass_g,
                        mp.centroid.x,
                        mp.centroid.y,
                        mp.centroid.z,
                        mp.principal_moments[0],
                        mp.principal_moments[1],
                        mp.principal_moments[2]
                    ));
                } else {
                    self.log("No visible surfaces/B-Reps/meshes for MassProperties.");
                }
            }
            "clippingplane" => {
                let bbox = self.doc.visible_bounding_box();
                let center = if bbox.is_empty() {
                    DVec3::ZERO
                } else {
                    bbox.center()
                };
                let span = if bbox.is_empty() {
                    16.0
                } else {
                    (bbox.size().max_element() * 1.6).max(8.0)
                };
                let mut plane = match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("x") => Plane::world_yz(),
                    Some("y") => Plane::world_zx(),
                    Some("cplane") => cplane,
                    _ => Plane::world_xy(),
                };
                plane.origin = center;
                let id = self.doc.add_clipping_plane(format!("Clip {}", self.doc.clipping_planes.len() + 1), plane);
                if let Some(cp) = self.doc.clipping_planes.iter_mut().find(|c| c.id == id) {
                    cp.size = span;
                }
                self.inspector_tab = InspectorTab::Analyze;
                self.mark_doc_dirty();
                self.log("Added live GPU Clipping Plane (adjust offset in Analyze tab).");
            }
            "clippingplaneflip" => {
                for cp in &mut self.doc.clipping_planes {
                    cp.flip = !cp.flip;
                }
                self.mark_doc_dirty();
                self.log("Flipped active clipping plane(s).");
            }
            "clippingplanetoggle" => {
                let next = !self.doc.clipping_planes.iter().any(|c| c.enabled);
                for cp in &mut self.doc.clipping_planes {
                    cp.enabled = next;
                }
                self.mark_doc_dirty();
                self.log(format!(
                    "Clipping planes: {}.",
                    if next { "Enabled" } else { "Disabled" }
                ));
            }
            "section" => {
                let plane = self
                    .doc
                    .active_clipping_planes()
                    .first()
                    .map(|c| c.plane)
                    .unwrap_or(cplane);
                let objs: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.visible && self.doc.is_layer_effectively_visible(o.layer_id))
                    .map(|o| (o.layer_id, o.geometry.clone()))
                    .collect();
                let mut added = 0usize;
                for (lid, geom) in objs {
                    for poly in section_geometry_with_plane(&geom, &plane) {
                        if let Ok(c) = NurbsCurve::polyline(&poly) {
                            self.doc
                                .add_object_on_layer("SectionCut", lid, Geometry::Curve(c));
                            added += 1;
                        }
                    }
                }
                if added > 0 {
                    self.mark_scene_dirty();
                }
                self.log(format!("Created {added} Section cut curve(s)."));
            }
            "make2d" => {
                let projection = match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("top") => DrawingProjection::Top,
                    Some("front") => DrawingProjection::Front,
                    Some("right") => DrawingProjection::Right,
                    Some("back") => DrawingProjection::Back,
                    Some("left") => DrawingProjection::Left,
                    Some("bottom") => DrawingProjection::Bottom,
                    Some("iso" | "isometric") => DrawingProjection::Isometric,
                    Some("perspective" | "persp") => {
                        let cam = &self.cameras[1];
                        let (eye, up) = cam.eye_and_up();
                        DrawingProjection::Perspective {
                            eye,
                            target: cam.target,
                            up,
                            fov_y_rad: cam.fov_y_rad,
                            aspect: 1.5,
                        }
                    }
                    _ => match self.cameras[self.active_viewport].kind {
                        cad_render::ViewportKind::Top => DrawingProjection::Top,
                        cad_render::ViewportKind::Front => DrawingProjection::Front,
                        cad_render::ViewportKind::Right => DrawingProjection::Right,
                        cad_render::ViewportKind::Perspective => {
                            let cam = &self.cameras[self.active_viewport];
                            let (eye, up) = cam.eye_and_up();
                            DrawingProjection::Perspective {
                                eye,
                                target: cam.target,
                                up,
                                fov_y_rad: cam.fov_y_rad,
                                aspect: 1.5,
                            }
                        }
                    },
                };
                let mut items: Vec<(Geometry, Option<[u8; 4]>)> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected && o.visible && self.doc.is_layer_effectively_visible(o.layer_id))
                    .map(|o| {
                        let col = o
                            .color
                            .or_else(|| self.doc.layers.get(o.layer_id).map(|l| l.color));
                        (o.geometry.clone(), col)
                    })
                    .collect();
                if items.is_empty() {
                    items = self
                        .doc
                        .objects
                        .values()
                        .filter(|o| o.visible && self.doc.is_layer_effectively_visible(o.layer_id))
                        .map(|o| {
                            let col = o
                                .color
                                .or_else(|| self.doc.layers.get(o.layer_id).map(|l| l.color));
                            (o.geometry.clone(), col)
                        })
                        .collect();
                }
                if items.is_empty() {
                    self.log("No visible geometry to project for Make2D.");
                } else {
                    let res = make_2d_projection(&items, projection, &Make2DOptions::default());
                    let parent_l = self.doc.add_layer(
                        format!("Make2D ({})", projection.label()),
                        [190, 210, 235, 255],
                    );
                    let vis_l = self
                        .doc
                        .add_sublayer("Visible", [235, 240, 248, 255], parent_l);
                    let sil_l = self
                        .doc
                        .add_sublayer("Silhouette", [255, 220, 120, 255], parent_l);
                    let hid_l = self
                        .doc
                        .add_sublayer("Hidden", [130, 140, 158, 255], parent_l);
                    let mut count = 0usize;
                    for seg in &res.segments {
                        if seg.points_2d.len() < 2 {
                            continue;
                        }
                        let pts3: Vec<DVec3> = seg
                            .points_2d
                            .iter()
                            .map(|p| DVec3::new(p[0], p[1], 0.0))
                            .collect();
                        if let Ok(c) = NurbsCurve::polyline(&pts3) {
                            let lid = match seg.kind {
                                Make2DLineKind::Visible => vis_l,
                                Make2DLineKind::Silhouette => sil_l,
                                Make2DLineKind::Hidden => hid_l,
                            };
                            self.doc
                                .add_object_on_layer("Make2DCrv", lid, Geometry::Curve(c));
                            count += 1;
                        }
                    }
                    self.mark_scene_dirty();
                    self.log(format!(
                        "Make2D ({}) created {count} 2D curve(s) on World XY.",
                        projection.label()
                    ));
                }
            }
            "layout" => {
                let want_new = args
                    .first()
                    .is_some_and(|a| a.eq_ignore_ascii_case("new"));
                let lid = if want_new || self.doc.layouts.is_empty() {
                    self.doc.add_standard_4view_layout(None, PageSize::A3)
                } else {
                    self.doc.layouts[0].id
                };
                self.workspace_view = WorkspaceViewMode::Layout(lid);
                self.layout_pan = eframe::egui::Vec2::ZERO;
                self.layout_zoom = 1.0;
                self.log("Switched to Paper Space 4-View Layout.");
            }
            "modelspace" => {
                self.workspace_view = WorkspaceViewMode::Model;
                self.log("Switched to 3D Model Space.");
            }
            "gdtframe" => {
                let symbol = match args.first().map(|s| s.to_ascii_lowercase()).as_deref() {
                    Some("flatness" | "flt") => GdtSymbol::Flatness,
                    Some("straightness" | "str") => GdtSymbol::Straightness,
                    Some("circularity" | "cir") => GdtSymbol::Circularity,
                    Some("cylindricity" | "cyl") => GdtSymbol::Cylindricity,
                    Some("perpendicularity" | "perp" | "per") => GdtSymbol::Perpendicularity,
                    Some("parallelism" | "par") => GdtSymbol::Parallelism,
                    Some("angularity" | "ang") => GdtSymbol::Angularity,
                    Some("profile" | "prs") => GdtSymbol::ProfileSurface,
                    Some("concentricity" | "con") => GdtSymbol::Concentricity,
                    Some("runout" | "run") => GdtSymbol::Runout,
                    _ => GdtSymbol::Position,
                };
                let tolerance = args
                    .get(1)
                    .and_then(|s| s.parse::<f64>().ok())
                    .unwrap_or(0.05)
                    .abs();
                let datums: Vec<String> = if args.len() > 2 {
                    args[2..].iter().map(|s| s.to_ascii_uppercase()).collect()
                } else {
                    vec!["A".to_string(), "B".to_string()]
                };
                let pos = self
                    .selection_object_plane()
                    .map(|p| p.origin)
                    .or_else(|| {
                        self.doc
                            .selected_ids_ordered()
                            .last()
                            .and_then(|&id| self.doc.objects.get(id))
                            .map(|o| o.aabb.center())
                    })
                    .unwrap_or(cplane.origin + cplane.z_axis * 1.5);
                self.doc.add_gdt_annotation(
                    GdtSpec {
                        symbol,
                        tolerance,
                        diameter_zone: symbol == GdtSymbol::Position,
                        modifier: None,
                        datums,
                    },
                    pos,
                );
                self.mark_doc_dirty();
                self.log(format!(
                    "Placed {} GD&T Feature Control Frame (tol={tolerance:.3}).",
                    symbol.label()
                ));
            }
            "datumtag" => {
                let label = args
                    .first()
                    .map(|s| s.to_ascii_uppercase())
                    .unwrap_or_else(|| "A".to_string());
                let pos = self
                    .selection_object_plane()
                    .map(|p| p.origin)
                    .or_else(|| {
                        self.doc
                            .selected_ids_ordered()
                            .last()
                            .and_then(|&id| self.doc.objects.get(id))
                            .map(|o| o.aabb.center())
                    })
                    .unwrap_or(cplane.origin);
                self.doc.add_datum_annotation(label.clone(), pos);
                self.mark_doc_dirty();
                self.log(format!("Placed Datum Tag [{label}]."));
            }
            "exportpdf" => {
                self.export_pdf_dialog();
            }
            "help" => {
                self.log("Interactive: Line, Polyline, Curve, InterpCrv, HandleCurve, Conic, Helix, Rectangle, Arc, Circle, Polygon, Box, Cylinder, Cone, Revolve, RailRevolve, ExtractIsocurve, SplitSrf, Move, Copy, Rotate, Scale, Mirror, Offset, SplitCurve, Trim, InsertKnot, ArrayLinear, ArrayPolar, Distance, Dim, TextDot, GdtFrame, DatumTag | B-Rep & Surfacing: BooleanUnion/Difference/Intersection/Split, Intersect, Cap, Untrim, FilletSrf, FilletEdge, ChamferEdge, Shell, PushPull, Inset, ExtractSrf, ToBRep, ShowEdges (Ctrl+Shift+Click sub-objects), Sphere, Torus, Plane, Extrude, PlanarSrf, Loft, Sweep1, Sweep2, EdgeSrf, NetworkSrf, BlendSrf, MatchSrf, OffsetSrf, ExtendSrf, Patch, RebuildSrf, DupBorder, DupEdge, Flip | Display & Analysis: Rendered, Concept, Arctic, Shaded, Wireframe, Technical, Ghosted, Zebra, EMap, GaussianCurvature, MeanCurvature, DraftAngle, AnalysisOff, GlobalEdgeContinuity, MassProperties, ClippingPlane, ClippingPlaneFlip, Section | Drafting & PDF: Make2D, Layout, ModelSpace, ExportPDF | Editing & UX: PointsOn/Off, UVN, SoftTransform, Join, Explode, Split, Fillet, Chamfer, BlendCrv, Match, Rebuild, RebuildAdaptive (Elmo), Fair, TweenCurves, ChangeDegree, CurvatureGraph, Area, AutoCPlane, SmartTrack, GumballAlign, GumballRelocate, GumballReset, History, BreakHistory, HistoryPurge, RadialMenu, Units [mm|cm|m|in|ft], ZoomExtents (ZE), Undo, Redo | Shortcuts: Q/Space=Radial Menu, Right/Left/Up=Axis Lock, T=SmartTrack point, Alt+Drag=Copy, Ctrl+Drag=Relocate Gumball");
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
