use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;

use cad_doc::Document;
use cad_geom::{
    BRep, BRepFace, ControlPoint, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh,
    subd_to_nurbs_brep,
};
use cad_math::UnitSystem;

use crate::error::IoError;

struct StepWriter {
    out: String,
    next_id: usize,
}

impl StepWriter {
    fn new() -> Self {
        Self {
            out: String::new(),
            next_id: 1,
        }
    }

    fn emit(&mut self, body: &str) -> usize {
        let id = self.next_id;
        self.next_id += 1;
        let _ = writeln!(self.out, "#{id}={body};");
        id
    }

    fn point(&mut self, p: DVec3) -> usize {
        self.emit(&format!(
            "CARTESIAN_POINT('',({:.8},{:.8},{:.8}))",
            p.x, p.y, p.z
        ))
    }

    fn curve(&mut self, curve: &NurbsCurve) -> usize {
        let cp_ids: Vec<usize> = curve
            .control_points
            .iter()
            .map(|cp| self.point(cp.position))
            .collect();
        let cp_list = id_tuple(&cp_ids);
        let (mults, unique_knots) = compress_knots(&curve.knots);
        let mult_str = usize_tuple(&mults);
        let knot_str = f64_tuple(&unique_knots);
        let rational = curve
            .control_points
            .iter()
            .any(|cp| (cp.weight - 1.0).abs() > 1e-7);

        if rational {
            let weights: Vec<f64> = curve.control_points.iter().map(|c| c.weight).collect();
            let w_str = f64_tuple(&weights);
            self.emit(&format!(
                "(BOUNDED_CURVE() B_SPLINE_CURVE({},({}),.UNSPECIFIED.,.F.,.F.) \
                 B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.) \
                 CURVE() GEOMETRIC_REPRESENTATION_ITEM() \
                 RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM(''))",
                curve.degree, cp_list, mult_str, knot_str, w_str
            ))
        } else {
            self.emit(&format!(
                "B_SPLINE_CURVE_WITH_KNOTS('',{},({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                curve.degree, cp_list, mult_str, knot_str
            ))
        }
    }

    fn surface(&mut self, srf: &NurbsSurface) -> usize {
        let mut row_strs = Vec::with_capacity(srf.count_u);
        let mut all_weights = Vec::with_capacity(srf.control_points.len());
        let mut rational = false;
        for u in 0..srf.count_u {
            let mut row_ids = Vec::with_capacity(srf.count_v);
            for v in 0..srf.count_v {
                let cp = srf.control_points[u * srf.count_v + v];
                if (cp.weight - 1.0).abs() > 1e-7 {
                    rational = true;
                }
                row_ids.push(self.point(cp.position));
            }
            row_strs.push(format!("({})", id_tuple(&row_ids)));
        }
        if rational {
            for u in 0..srf.count_u {
                let mut w_row = Vec::with_capacity(srf.count_v);
                for v in 0..srf.count_v {
                    w_row.push(format!("{:.8}", srf.control_points[u * srf.count_v + v].weight));
                }
                all_weights.push(format!("({})", w_row.join(",")));
            }
        }
        let cp_grid = row_strs.join(",");
        let (u_mults, u_knots) = compress_knots(&srf.knots_u);
        let (v_mults, v_knots) = compress_knots(&srf.knots_v);

        if rational {
            self.emit(&format!(
                "(BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},({}),.UNSPECIFIED.,.F.,.F.,.F.) \
                 B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.) \
                 GEOMETRIC_REPRESENTATION_ITEM() \
                 RATIONAL_B_SPLINE_SURFACE(({})) \
                 REPRESENTATION_ITEM('') SURFACE())",
                srf.degree_u,
                srf.degree_v,
                cp_grid,
                usize_tuple(&u_mults),
                usize_tuple(&v_mults),
                f64_tuple(&u_knots),
                f64_tuple(&v_knots),
                all_weights.join(","),
            ))
        } else {
            self.emit(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                srf.degree_u,
                srf.degree_v,
                cp_grid,
                usize_tuple(&u_mults),
                usize_tuple(&v_mults),
                f64_tuple(&u_knots),
                f64_tuple(&v_knots),
            ))
        }
    }

    fn brep(&mut self, name: &str, brep: &BRep) -> usize {
        let mut face_ids = Vec::with_capacity(brep.faces.len());
        for face in &brep.faces {
            let srf_id = self.surface(&face.surface);
            let bcurves = face.boundary_curves_3d();
            let mut oriented_edges = Vec::new();
            for c in &bcurves {
                let (t0, t1) = c.domain();
                let p0 = self.point(c.evaluate(t0));
                let p1 = self.point(c.evaluate(t1));
                let v0 = self.emit(&format!("VERTEX_POINT('',#{p0})"));
                let v1 = self.emit(&format!("VERTEX_POINT('',#{p1})"));
                let cid = self.curve(c);
                let ec = self.emit(&format!("EDGE_CURVE('',#{v0},#{v1},#{cid},.T.)"));
                let oe = self.emit(&format!("ORIENTED_EDGE('',*,*,#{ec},.T.)"));
                oriented_edges.push(oe);
            }
            let bounds = if oriented_edges.is_empty() {
                String::new()
            } else {
                let el = self.emit(&format!("EDGE_LOOP('',({}))", id_tuple(&oriented_edges)));
                let fb = self.emit(&format!("FACE_OUTER_BOUND('',#{el},.T.)"));
                format!("#{fb}")
            };
            let sense = if face.orientation_reversed { ".F." } else { ".T." };
            let af = self.emit(&format!(
                "ADVANCED_FACE('',({bounds}),#{srf_id},{sense})"
            ));
            face_ids.push(af);
        }

        if brep.is_solid() {
            let shell = self.emit(&format!("CLOSED_SHELL('',({}))", id_tuple(&face_ids)));
            self.emit(&format!(
                "MANIFOLD_SOLID_BREP('{}',#{shell})",
                escape_step_str(name)
            ))
        } else {
            let shell = self.emit(&format!("OPEN_SHELL('',({}))", id_tuple(&face_ids)));
            self.emit(&format!(
                "SHELL_BASED_SURFACE_MODEL('{}',(#{shell}))",
                escape_step_str(name)
            ))
        }
    }

    fn mesh(&mut self, name: &str, mesh: &TriangleMesh) -> Option<usize> {
        if mesh.positions.is_empty() || mesh.indices.is_empty() {
            return None;
        }
        let pt_ids: Vec<usize> = mesh
            .positions
            .iter()
            .map(|p| self.point(DVec3::from_array(*p)))
            .collect();
        let mut face_ids = Vec::with_capacity(mesh.indices.len());
        for &[i0, i1, i2] in &mesh.indices {
            let loop_id = self.emit(&format!(
                "POLY_LOOP('',(#{},#{},#{}))",
                pt_ids[i0 as usize], pt_ids[i1 as usize], pt_ids[i2 as usize]
            ));
            let fb = self.emit(&format!("FACE_OUTER_BOUND('',#{loop_id},.T.)"));
            let pf = self.emit(&format!("FACE_SURFACE('',(#{fb}),$,.T.)"));
            face_ids.push(pf);
        }
        let shell = self.emit(&format!("CLOSED_SHELL('',({}))", id_tuple(&face_ids)));
        Some(self.emit(&format!(
            "FACETED_BREP('{}',#{shell})",
            escape_step_str(name)
        )))
    }
}

/// Export all visible document geometry to an ISO-10303-21 (STEP AP214) string.
pub fn export_step_string(doc: &Document) -> String {
    let mut w = StepWriter::new();
    let unit_item = match doc.units {
        UnitSystem::Millimeters => "SI_UNIT(.MILLI.,.METRE.)",
        UnitSystem::Centimeters => "SI_UNIT(.CENTI.,.METRE.)",
        UnitSystem::Meters => "SI_UNIT($,.METRE.)",
        UnitSystem::Inches | UnitSystem::Feet => "SI_UNIT(.MILLI.,.METRE.)",
    };
    let origin = w.point(DVec3::ZERO);
    let z_dir = w.emit("DIRECTION('',(0.,0.,1.))");
    let x_dir = w.emit("DIRECTION('',(1.,0.,0.))");
    let placement = w.emit(&format!("AXIS2_PLACEMENT_3D('',#{origin},#{z_dir},#{x_dir})"));

    let mut rep_items = vec![placement];
    let scale: f64 = match doc.units {
        UnitSystem::Inches => 25.4,
        UnitSystem::Feet => 304.8,
        _ => 1.0,
    };

    for (idx, (_, obj)) in doc.objects.iter().enumerate() {
        if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
            continue;
        }
        let label = obj
            .name
            .clone()
            .unwrap_or_else(|| format!("Object_{}", idx + 1));
        let mut geom = obj.geometry.clone();
        if (scale - 1.0).abs() > 1e-9 {
            geom.transform(&cad_geom::DMat4::from_scale(DVec3::splat(scale)));
        }

        match &geom {
            Geometry::Point(p) => {
                rep_items.push(w.point(*p));
            }
            Geometry::Curve(c) => {
                rep_items.push(w.curve(c));
            }
            Geometry::Surface(s) => {
                rep_items.push(w.surface(s));
            }
            Geometry::BRep(b) => {
                rep_items.push(w.brep(&label, b));
            }
            Geometry::SubD(sd) => {
                if let Ok(brep) = subd_to_nurbs_brep(sd) {
                    rep_items.push(w.brep(&label, &brep));
                } else if let Some(fid) = w.mesh(&label, &sd.tessellate_display()) {
                    rep_items.push(fid);
                }
            }
            Geometry::Mesh(m) => {
                if let Some(fid) = w.mesh(&label, m) {
                    rep_items.push(fid);
                }
            }
        }
    }

    let app_ctx = w.emit("APPLICATION_CONTEXT('automotive design')");
    let prod_def_ctx = w.emit(&format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{app_ctx},'design')"
    ));
    let prod = w.emit("PRODUCT('Elephant','Elephant Export','',())");
    let prod_form = w.emit(&format!(
        "PRODUCT_DEFINITION_FORMATION('','',#{prod})"
    ));
    let prod_def = w.emit(&format!(
        "PRODUCT_DEFINITION('design','',#{prod_form},#{prod_def_ctx})"
    ));
    let prod_shape = w.emit(&format!(
        "PRODUCT_DEFINITION_SHAPE('','',#{prod_def})"
    ));
    let unit_id = w.emit(&format!("({unit_item} NAMED_UNIT(*))"));
    let ang_unit = w.emit("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    let solid_unit = w.emit("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    let uncert = w.emit(&format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#{unit_id},'distance_accuracy_value','')"
    ));
    let geom_ctx = w.emit(&format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncert})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{unit_id},#{ang_unit},#{solid_unit})) REPRESENTATION_CONTEXT('3D',''))"
    ));
    let rep = w.emit(&format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('Elephant',({}),#{geom_ctx})",
        id_tuple(&rep_items)
    ));
    let _ = w.emit(&format!("SHAPE_DEFINITION_REPRESENTATION(#{prod_shape},#{rep})"));

    format!(
        "ISO-10303-21;\n\
         HEADER;\n\
         FILE_DESCRIPTION(('Elephant CAD STEP AP214 Export'),'2;1');\n\
         FILE_NAME('elephant.stp','2026-01-01T00:00:00',('Elephant'),('Elephant'),'','','');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n\
         ENDSEC;\n\
         DATA;\n\
         {}\
         ENDSEC;\n\
         END-ISO-10303-21;\n",
        w.out
    )
}

#[derive(Debug, Clone)]
enum StepVal {
    Ref(usize),
    Num(f64),
    Str(String),
    Enum(String),
    List(Vec<StepVal>),
    Null,
}

#[derive(Debug, Clone)]
enum StepEntity {
    Simple { name: String, args: Vec<StepVal> },
    Complex(Vec<(String, Vec<StepVal>)>),
}

impl StepEntity {
    fn find(&self, name: &str) -> Option<&[StepVal]> {
        match self {
            Self::Simple { name: n, args } => (n == name).then_some(args.as_slice()),
            Self::Complex(parts) => parts.iter().find(|(n, _)| n == name).map(|(_, a)| a.as_slice()),
        }
    }
}

/// Import B-Reps, NURBS surfaces, curves, meshes, and points from an ISO-10303-21 STEP string.
pub fn import_step_str(text: &str, doc: &mut Document) -> Result<usize, IoError> {
    if !text.contains("ISO-10303-21") && !text.contains("DATA;") {
        return Err(IoError::InvalidFormat("missing ISO-10303-21 STEP header".to_string()));
    }
    let clean = strip_step_comments(text);
    let data_body = if let Some(idx) = clean.find("DATA;") {
        let rest = &clean[idx + 5..];
        if let Some(end) = rest.find("ENDSEC;") {
            &rest[..end]
        } else {
            rest
        }
    } else {
        clean.as_str()
    };

    let mut entities: HashMap<usize, StepEntity> = HashMap::new();
    for stmt in split_step_statements(data_body) {
        let trimmed = stmt.trim();
        if !trimmed.starts_with('#') {
            continue;
        }
        let Some(eq_pos) = trimmed.find('=') else {
            continue;
        };
        let Ok(id) = trimmed[1..eq_pos].trim().parse::<usize>() else {
            continue;
        };
        let rhs = trimmed[eq_pos + 1..].trim();
        if let Some(ent) = parse_step_entity_rhs(rhs) {
            entities.insert(id, ent);
        }
    }

    let ctx = StepResolver {
        entities: &entities,
    };
    let mut added = 0usize;
    let mut consumed_faces: HashSet<usize> = HashSet::new();
    let mut consumed_surfaces: HashSet<usize> = HashSet::new();
    let mut consumed_curves: HashSet<usize> = HashSet::new();
    let mut consumed_points: HashSet<usize> = HashSet::new();

    // Sort IDs for deterministic import order
    let mut all_ids: Vec<usize> = entities.keys().copied().collect();
    all_ids.sort_unstable();

    // 1. Solid & Shell B-Reps
    for &id in &all_ids {
        let Some(ent) = entities.get(&id) else {
            continue;
        };
        let (shell_ids, label) = if let Some(args) = ent.find("MANIFOLD_SOLID_BREP") {
            let name = args.first().and_then(StepVal::as_str).map(String::from);
            let sids = args
                .get(1)
                .and_then(StepVal::as_ref)
                .map(|r| vec![r])
                .unwrap_or_default();
            (sids, name)
        } else if let Some(args) = ent.find("SHELL_BASED_SURFACE_MODEL") {
            let name = args.first().and_then(StepVal::as_str).map(String::from);
            let sids = args
                .get(1)
                .and_then(StepVal::as_list)
                .map(|l| l.iter().filter_map(StepVal::as_ref).collect())
                .unwrap_or_default();
            (sids, name)
        } else {
            continue;
        };

        let mut brep_faces = Vec::new();
        for sid in shell_ids {
            if let Some(shell) = entities.get(&sid) {
                let f_args = shell
                    .find("CLOSED_SHELL")
                    .or_else(|| shell.find("OPEN_SHELL"))
                    .or_else(|| shell.find("ORIENTED_CLOSED_SHELL"));
                if let Some(args) = f_args {
                    if let Some(flist) = args.get(1).and_then(StepVal::as_list) {
                        for fv in flist {
                            if let Some(fid) = fv.as_ref() {
                                consumed_faces.insert(fid);
                                if let Some((face, used_srf, used_crvs)) = ctx.resolve_face(fid) {
                                    brep_faces.push(face);
                                    consumed_surfaces.insert(used_srf);
                                    consumed_curves.extend(used_crvs);
                                }
                            }
                        }
                    }
                }
            }
        }
        if !brep_faces.is_empty() {
            let geom = if brep_faces.len() == 1 && !brep_faces[0].is_trimmed() {
                Geometry::Surface(brep_faces.remove(0).surface)
            } else {
                Geometry::BRep(BRep::from_faces(brep_faces, 1e-4))
            };
            let oid = doc.add_object("ImportSTEP_BRep", geom);
            if let Some(obj) = doc.objects.get_mut(oid) {
                if label.as_ref().is_some_and(|l| !l.is_empty()) {
                    obj.name = label;
                }
            }
            added += 1;
        }
    }

    // 2. Faceted B-Reps (Polyhedral meshes)
    for &id in &all_ids {
        let Some(ent) = entities.get(&id) else {
            continue;
        };
        let Some(args) = ent.find("FACETED_BREP") else {
            continue;
        };
        let label = args.first().and_then(StepVal::as_str).map(String::from);
        let Some(shell_id) = args.get(1).and_then(StepVal::as_ref) else {
            continue;
        };
        if let Some(mesh) = ctx.resolve_faceted_shell(shell_id, &mut consumed_faces) {
            let oid = doc.add_object("ImportSTEP_Mesh", Geometry::Mesh(mesh));
            if let Some(obj) = doc.objects.get_mut(oid) {
                if label.as_ref().is_some_and(|l| !l.is_empty()) {
                    obj.name = label;
                }
            }
            added += 1;
        }
    }

    // 3. Collect all curves & surfaces owned by B-Rep EDGE_CURVEs so we don't duplicate edges as free curves
    for &id in &all_ids {
        if let Some(ent) = entities.get(&id) {
            if let Some(args) = ent.find("EDGE_CURVE") {
                if let Some(cid) = args.get(3).and_then(StepVal::as_ref) {
                    consumed_curves.insert(cid);
                }
            }
        }
    }

    // 4. Standalone surfaces
    for &id in &all_ids {
        if consumed_surfaces.contains(&id) {
            continue;
        }
        if let Some(srf) = ctx.resolve_surface(id, &mut consumed_points) {
            doc.add_object("ImportSTEP_Surface", Geometry::Surface(srf));
            added += 1;
        }
    }

    // 5. Standalone curves (lines, circles, NURBS)
    for &id in &all_ids {
        if consumed_curves.contains(&id) {
            continue;
        }
        if let Some(crv) = ctx.resolve_curve(id, &mut consumed_points) {
            doc.add_object("ImportSTEP_Curve", Geometry::Curve(crv));
            added += 1;
        }
    }

    // 6. Standalone points only when no higher geometry was imported (avoids importing every CP!)
    if added == 0 {
        for &id in &all_ids {
            if consumed_points.contains(&id) {
                continue;
            }
            if let Some(pt) = ctx.resolve_point(id) {
                doc.add_object("ImportSTEP_Point", Geometry::Point(pt));
                added += 1;
            }
        }
    }

    Ok(added)
}

struct StepResolver<'a> {
    entities: &'a HashMap<usize, StepEntity>,
}

impl<'a> StepResolver<'a> {
    fn resolve_point(&self, id: usize) -> Option<DVec3> {
        let ent = self.entities.get(&id)?;
        if let Some(args) = ent.find("CARTESIAN_POINT") {
            let coords = args.get(1).and_then(StepVal::as_list)?;
            if coords.len() >= 3 {
                return Some(DVec3::new(
                    coords[0].as_f64()?,
                    coords[1].as_f64()?,
                    coords[2].as_f64()?,
                ));
            }
        }
        if let Some(args) = ent.find("VERTEX_POINT") {
            let pid = args.get(1).and_then(StepVal::as_ref)?;
            return self.resolve_point(pid);
        }
        None
    }

    fn resolve_dir(&self, id: usize) -> Option<DVec3> {
        let ent = self.entities.get(&id)?;
        let args = ent.find("DIRECTION")?;
        let coords = args.get(1).and_then(StepVal::as_list)?;
        if coords.len() >= 3 {
            Some(
                DVec3::new(
                    coords[0].as_f64()?,
                    coords[1].as_f64()?,
                    coords[2].as_f64()?,
                )
                .normalize_or(DVec3::Z),
            )
        } else {
            None
        }
    }

    fn resolve_axis2_placement(&self, id: usize) -> Option<Plane> {
        let ent = self.entities.get(&id)?;
        let args = ent.find("AXIS2_PLACEMENT_3D")?;
        let origin = args
            .get(1)
            .and_then(StepVal::as_ref)
            .and_then(|r| self.resolve_point(r))
            .unwrap_or(DVec3::ZERO);
        let z_axis = args
            .get(2)
            .and_then(StepVal::as_ref)
            .and_then(|r| self.resolve_dir(r))
            .unwrap_or(DVec3::Z);
        let x_raw = args
            .get(3)
            .and_then(StepVal::as_ref)
            .and_then(|r| self.resolve_dir(r))
            .unwrap_or_else(|| {
                if z_axis.x.abs() < 0.9 {
                    DVec3::X
                } else {
                    DVec3::Y
                }
            });
        let x_axis = (x_raw - z_axis * x_raw.dot(z_axis)).normalize_or(DVec3::X);
        let y_axis = z_axis.cross(x_axis).normalize_or(DVec3::Y);
        Some(Plane {
            origin,
            x_axis,
            y_axis,
            z_axis,
        })
    }

    fn resolve_curve(&self, id: usize, used_pts: &mut HashSet<usize>) -> Option<NurbsCurve> {
        let ent = self.entities.get(&id)?;
        if let Some(args) = ent.find("CIRCLE") {
            let pl = args
                .get(1)
                .and_then(StepVal::as_ref)
                .and_then(|r| self.resolve_axis2_placement(r))?;
            let radius = args.get(2).and_then(StepVal::as_f64)?;
            return NurbsCurve::circle(&pl, radius).ok();
        }
        if let Some(args) = ent.find("LINE") {
            let p0 = args
                .get(1)
                .and_then(StepVal::as_ref)
                .and_then(|r| self.resolve_point(r))?;
            let vid = args.get(2).and_then(StepVal::as_ref)?;
            let vent = self.entities.get(&vid)?;
            let vargs = vent.find("VECTOR")?;
            let dir = vargs
                .get(1)
                .and_then(StepVal::as_ref)
                .and_then(|r| self.resolve_dir(r))
                .unwrap_or(DVec3::X);
            let mag = vargs.get(2).and_then(StepVal::as_f64).unwrap_or(1.0);
            return Some(NurbsCurve::line(p0, p0 + dir * mag));
        }

        // B-Spline curve (simple B_SPLINE_CURVE_WITH_KNOTS or complex rational entity)
        if let Some(args) = ent.find("B_SPLINE_CURVE_WITH_KNOTS") {
            let (deg, cp_ids_val, mult_val, knot_val) = if args.len() >= 8 {
                (
                    args.get(1)?.as_usize()?,
                    args.get(2)?.as_list()?,
                    args.get(6)?.as_list()?,
                    args.get(7)?.as_list()?,
                )
            } else {
                // Complex form
                let bsc = ent.find("B_SPLINE_CURVE")?;
                (
                    bsc.first()?.as_usize()?,
                    bsc.get(1)?.as_list()?,
                    args.first()?.as_list()?,
                    args.get(1)?.as_list()?,
                )
            };
            let weights: Option<Vec<f64>> = ent.find("RATIONAL_B_SPLINE_CURVE").and_then(|rw| {
                rw.first()?
                    .as_list()?
                    .iter()
                    .map(StepVal::as_f64)
                    .collect()
            });
            let mut cps = Vec::with_capacity(cp_ids_val.len());
            for (idx, v) in cp_ids_val.iter().enumerate() {
                let pid = v.as_ref()?;
                used_pts.insert(pid);
                let pos = self.resolve_point(pid)?;
                let w = weights.as_ref().and_then(|ws| ws.get(idx).copied()).unwrap_or(1.0);
                cps.push(ControlPoint::new(pos, w));
            }
            let mults: Vec<usize> = mult_val.iter().filter_map(StepVal::as_usize).collect();
            let uniq: Vec<f64> = knot_val.iter().filter_map(StepVal::as_f64).collect();
            let knots = expand_knots(&mults, &uniq);
            return NurbsCurve::new(deg, knots, cps).ok();
        }
        None
    }

    fn resolve_surface(&self, id: usize, used_pts: &mut HashSet<usize>) -> Option<NurbsSurface> {
        let ent = self.entities.get(&id)?;
        if let Some(args) = ent.find("B_SPLINE_SURFACE_WITH_KNOTS") {
            let (deg_u, deg_v, grid_val, um_val, vm_val, uk_val, vk_val) = if args.len() >= 13 {
                (
                    args.get(1)?.as_usize()?,
                    args.get(2)?.as_usize()?,
                    args.get(3)?.as_list()?,
                    args.get(8)?.as_list()?,
                    args.get(9)?.as_list()?,
                    args.get(10)?.as_list()?,
                    args.get(11)?.as_list()?,
                )
            } else {
                let bss = ent.find("B_SPLINE_SURFACE")?;
                (
                    bss.first()?.as_usize()?,
                    bss.get(1)?.as_usize()?,
                    bss.get(2)?.as_list()?,
                    args.first()?.as_list()?,
                    args.get(1)?.as_list()?,
                    args.get(2)?.as_list()?,
                    args.get(3)?.as_list()?,
                )
            };

            let weight_rows: Option<Vec<Vec<f64>>> = ent.find("RATIONAL_B_SPLINE_SURFACE").and_then(|rw| {
                rw.first()?
                    .as_list()?
                    .iter()
                    .map(|row| row.as_list()?.iter().map(StepVal::as_f64).collect())
                    .collect()
            });

            let count_u = grid_val.len();
            let count_v = grid_val.first()?.as_list()?.len();
            let mut cps = Vec::with_capacity(count_u * count_v);
            for (u, row_val) in grid_val.iter().enumerate() {
                let row = row_val.as_list()?;
                if row.len() != count_v {
                    return None;
                }
                for (v, pv) in row.iter().enumerate() {
                    let pid = pv.as_ref()?;
                    used_pts.insert(pid);
                    let pos = self.resolve_point(pid)?;
                    let w = weight_rows
                        .as_ref()
                        .and_then(|wr| wr.get(u)?.get(v).copied())
                        .unwrap_or(1.0);
                    cps.push(ControlPoint::new(pos, w));
                }
            }
            let u_mults: Vec<usize> = um_val.iter().filter_map(StepVal::as_usize).collect();
            let v_mults: Vec<usize> = vm_val.iter().filter_map(StepVal::as_usize).collect();
            let u_uniq: Vec<f64> = uk_val.iter().filter_map(StepVal::as_f64).collect();
            let v_uniq: Vec<f64> = vk_val.iter().filter_map(StepVal::as_f64).collect();
            let ku = expand_knots(&u_mults, &u_uniq);
            let kv = expand_knots(&v_mults, &v_uniq);
            return NurbsSurface::new(deg_u, deg_v, ku, kv, count_u, count_v, cps).ok();
        }
        None
    }

    fn resolve_face(&self, id: usize) -> Option<(BRepFace, usize, Vec<usize>)> {
        let ent = self.entities.get(&id)?;
        let args = ent
            .find("ADVANCED_FACE")
            .or_else(|| ent.find("FACE_SURFACE"))?;
        let srf_id = args.get(2).and_then(StepVal::as_ref)?;
        let mut dummy_pts = HashSet::new();
        let srf = self.resolve_surface(srf_id, &mut dummy_pts)?;
        let sense = args
            .get(3)
            .and_then(|v| match v {
                StepVal::Enum(s) => Some(s != "F"),
                _ => None,
            })
            .unwrap_or(true);

        let mut used_curves = Vec::new();
        if let Some(bounds) = args.get(1).and_then(StepVal::as_list) {
            for b in bounds {
                if let Some(bid) = b.as_ref() {
                    if let Some(bent) = self.entities.get(&bid) {
                        let bargs = bent
                            .find("FACE_OUTER_BOUND")
                            .or_else(|| bent.find("FACE_BOUND"));
                        if let Some(ba) = bargs {
                            if let Some(lid) = ba.get(1).and_then(StepVal::as_ref) {
                                if let Some(lent) = self.entities.get(&lid) {
                                    if let Some(largs) = lent.find("EDGE_LOOP") {
                                        if let Some(edges) = largs.get(1).and_then(StepVal::as_list) {
                                            for ev in edges {
                                                if let Some(oeid) = ev.as_ref() {
                                                    if let Some(oent) = self.entities.get(&oeid) {
                                                        if let Some(oargs) = oent.find("ORIENTED_EDGE") {
                                                            if let Some(ecid) = oargs.get(3).and_then(StepVal::as_ref) {
                                                                if let Some(ecent) = self.entities.get(&ecid) {
                                                                    if let Some(ecargs) = ecent.find("EDGE_CURVE") {
                                                                        if let Some(cid) = ecargs.get(3).and_then(StepVal::as_ref) {
                                                                            used_curves.push(cid);
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        let mut face = BRepFace::untrimmed(srf);
        face.orientation_reversed = !sense;
        Some((face, srf_id, used_curves))
    }

    fn resolve_faceted_shell(
        &self,
        shell_id: usize,
        consumed_faces: &mut HashSet<usize>,
    ) -> Option<TriangleMesh> {
        let shell = self.entities.get(&shell_id)?;
        let f_args = shell
            .find("CLOSED_SHELL")
            .or_else(|| shell.find("OPEN_SHELL"))?;
        let flist = f_args.get(1).and_then(StepVal::as_list)?;
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        for fv in flist {
            let Some(fid) = fv.as_ref() else { continue };
            consumed_faces.insert(fid);
            let Some(fent) = self.entities.get(&fid) else { continue };
            let Some(fargs) = fent
                .find("FACE_SURFACE")
                .or_else(|| fent.find("ADVANCED_FACE"))
            else {
                continue;
            };
            let Some(bounds) = fargs.get(1).and_then(StepVal::as_list) else {
                continue;
            };
            for bv in bounds {
                let Some(bid) = bv.as_ref() else { continue };
                let Some(bent) = self.entities.get(&bid) else { continue };
                let Some(ba) = bent
                    .find("FACE_OUTER_BOUND")
                    .or_else(|| bent.find("FACE_BOUND"))
                else {
                    continue;
                };
                let Some(lid) = ba.get(1).and_then(StepVal::as_ref) else { continue };
                let Some(lent) = self.entities.get(&lid) else { continue };
                let Some(pargs) = lent.find("POLY_LOOP") else { continue };
                let Some(pts_val) = pargs.get(1).and_then(StepVal::as_list) else {
                    continue;
                };
                let pts: Vec<DVec3> = pts_val
                    .iter()
                    .filter_map(|v| v.as_ref().and_then(|r| self.resolve_point(r)))
                    .collect();
                if pts.len() >= 3 {
                    let n = (pts[1] - pts[0]).cross(pts[2] - pts[0]).normalize_or(DVec3::Z);
                    let base = positions.len() as u32;
                    for p in &pts {
                        positions.push(p.to_array());
                        normals.push(n.to_array());
                    }
                    for k in 1..(pts.len() - 1) {
                        indices.push([base, base + k as u32, base + (k + 1) as u32]);
                    }
                }
            }
        }
        (!indices.is_empty()).then_some(TriangleMesh {
            positions,
            normals,
            indices,
        })
    }
}

impl StepVal {
    fn as_ref(&self) -> Option<usize> {
        match self {
            Self::Ref(r) => Some(*r),
            _ => None,
        }
    }
    fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Num(n) => Some(*n),
            _ => None,
        }
    }
    fn as_usize(&self) -> Option<usize> {
        match self {
            Self::Num(n) if *n >= 0.0 => Some(n.round() as usize),
            _ => None,
        }
    }
    fn as_str(&self) -> Option<&str> {
        match self {
            Self::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }
    fn as_list(&self) -> Option<&[StepVal]> {
        match self {
            Self::List(l) => Some(l.as_slice()),
            _ => None,
        }
    }
}

fn strip_step_comments(input: &str) -> String {
    let mut out = String::with_capacity(input.len());
    let bytes = input.as_bytes();
    let mut i = 0;
    let mut in_str = false;
    while i < bytes.len() {
        if in_str {
            let ch = bytes[i] as char;
            out.push(ch);
            if bytes[i] == b'\'' {
                in_str = false;
            }
            i += 1;
        } else if bytes[i] == b'\'' {
            in_str = true;
            out.push('\'');
            i += 1;
        } else if i + 1 < bytes.len() && bytes[i] == b'/' && bytes[i + 1] == b'*' {
            i += 2;
            while i + 1 < bytes.len() && !(bytes[i] == b'*' && bytes[i + 1] == b'/') {
                i += 1;
            }
            i = (i + 2).min(bytes.len());
        } else {
            out.push(bytes[i] as char);
            i += 1;
        }
    }
    out
}

fn split_step_statements(data: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut in_str = false;
    for ch in data.chars() {
        if in_str {
            cur.push(ch);
            if ch == '\'' {
                in_str = false;
            }
        } else if ch == '\'' {
            in_str = true;
            cur.push(ch);
        } else if ch == ';' {
            if !cur.trim().is_empty() {
                out.push(std::mem::take(&mut cur));
            } else {
                cur.clear();
            }
        } else {
            cur.push(ch);
        }
    }
    out
}

fn parse_step_entity_rhs(rhs: &str) -> Option<StepEntity> {
    let bytes = rhs.as_bytes();
    let mut pos = skip_ws(bytes, 0);
    if pos < bytes.len() && bytes[pos] == b'(' {
        // Complex entity instance: ( PART1(...) PART2(...) )
        pos += 1;
        let mut parts = Vec::new();
        loop {
            pos = skip_ws(bytes, pos);
            if pos >= bytes.len() || bytes[pos] == b')' {
                break;
            }
            let (name, next_pos) = parse_ident(bytes, pos);
            pos = skip_ws(bytes, next_pos);
            if pos < bytes.len() && bytes[pos] == b'(' {
                let (args, after) = parse_arg_list(bytes, pos);
                pos = after;
                if !name.is_empty() {
                    parts.push((name, args));
                }
            } else {
                break;
            }
        }
        Some(StepEntity::Complex(parts))
    } else {
        let (name, next_pos) = parse_ident(bytes, pos);
        let pos = skip_ws(bytes, next_pos);
        if pos < bytes.len() && bytes[pos] == b'(' {
            let (args, _) = parse_arg_list(bytes, pos);
            Some(StepEntity::Simple { name, args })
        } else {
            None
        }
    }
}

fn skip_ws(b: &[u8], mut i: usize) -> usize {
    while i < b.len() && b[i].is_ascii_whitespace() {
        i += 1;
    }
    i
}

fn parse_ident(b: &[u8], mut i: usize) -> (String, usize) {
    let start = i;
    while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
        i += 1;
    }
    let s = std::str::from_utf8(&b[start..i])
        .unwrap_or("")
        .to_ascii_uppercase();
    (s, i)
}

fn parse_arg_list(b: &[u8], mut i: usize) -> (Vec<StepVal>, usize) {
    if i >= b.len() || b[i] != b'(' {
        return (Vec::new(), i);
    }
    i += 1;
    let mut list = Vec::new();
    loop {
        i = skip_ws(b, i);
        if i >= b.len() {
            break;
        }
        if b[i] == b')' {
            i += 1;
            break;
        }
        let (val, next) = parse_val(b, i);
        list.push(val);
        i = skip_ws(b, next);
        if i < b.len() && b[i] == b',' {
            i += 1;
        }
    }
    (list, i)
}

fn parse_val(b: &[u8], mut i: usize) -> (StepVal, usize) {
    i = skip_ws(b, i);
    if i >= b.len() {
        return (StepVal::Null, i);
    }
    match b[i] {
        b'(' => {
            let (list, next) = parse_arg_list(b, i);
            (StepVal::List(list), next)
        }
        b'#' => {
            i += 1;
            let start = i;
            while i < b.len() && b[i].is_ascii_digit() {
                i += 1;
            }
            let r = std::str::from_utf8(&b[start..i])
                .ok()
                .and_then(|s| s.parse::<usize>().ok());
            (r.map(StepVal::Ref).unwrap_or(StepVal::Null), i)
        }
        b'\'' => {
            i += 1;
            let mut s = String::new();
            while i < b.len() {
                if b[i] == b'\'' {
                    if i + 1 < b.len() && b[i + 1] == b'\'' {
                        s.push('\'');
                        i += 2;
                        continue;
                    }
                    i += 1;
                    break;
                }
                s.push(b[i] as char);
                i += 1;
            }
            (StepVal::Str(s), i)
        }
        b'.' => {
            i += 1;
            let start = i;
            while i < b.len() && b[i] != b'.' {
                i += 1;
            }
            let e = std::str::from_utf8(&b[start..i])
                .unwrap_or("")
                .to_ascii_uppercase();
            if i < b.len() && b[i] == b'.' {
                i += 1;
            }
            (StepVal::Enum(e), i)
        }
        b'$' | b'*' => (StepVal::Null, i + 1),
        _ => {
            let start = i;
            while i < b.len()
                && (b[i].is_ascii_alphanumeric()
                    || matches!(b[i], b'+' | b'-' | b'.' | b'_'))
            {
                if b[i] == b'(' {
                    break;
                }
                i += 1;
            }
            // Handle typed value wrapper like LENGTH_MEASURE(1.0)
            let after_ws = skip_ws(b, i);
            if after_ws < b.len() && b[after_ws] == b'(' {
                let (args, end) = parse_arg_list(b, after_ws);
                return (args.into_iter().next().unwrap_or(StepVal::Null), end);
            }
            let tok = std::str::from_utf8(&b[start..i]).unwrap_or("");
            if let Ok(num) = tok.parse::<f64>() {
                (StepVal::Num(num), i)
            } else {
                (StepVal::Enum(tok.to_string()), i)
            }
        }
    }
}

fn compress_knots(knots: &[f64]) -> (Vec<usize>, Vec<f64>) {
    let mut mults = Vec::new();
    let mut uniq: Vec<f64> = Vec::new();
    for &k in knots {
        if let Some(&last) = uniq.last() {
            if (k - last).abs() < 1e-9 {
                if let Some(m) = mults.last_mut() {
                    *m += 1;
                }
                continue;
            }
        }
        uniq.push(k);
        mults.push(1);
    }
    (mults, uniq)
}

fn expand_knots(mults: &[usize], unique: &[f64]) -> Vec<f64> {
    let mut out = Vec::new();
    for (&m, &k) in mults.iter().zip(unique.iter()) {
        for _ in 0..m.max(1) {
            out.push(k);
        }
    }
    out
}

fn id_tuple(ids: &[usize]) -> String {
    ids.iter()
        .map(|id| format!("#{id}"))
        .collect::<Vec<_>>()
        .join(",")
}

fn usize_tuple(vals: &[usize]) -> String {
    vals.iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(",")
}

fn f64_tuple(vals: &[f64]) -> String {
    vals.iter()
        .map(|v| format!("{v:.8}"))
        .collect::<Vec<_>>()
        .join(",")
}

fn escape_step_str(s: &str) -> String {
    s.replace('\'', "''")
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::Plane;

    #[test]
    fn step_roundtrip_curve_surface_and_brep() {
        let mut doc = Document::new();
        let circle = NurbsCurve::circle(&Plane::world_xy(), 3.0).unwrap();
        doc.add_object("Circle", Geometry::Curve(circle));

        let sphere = NurbsSurface::sphere(DVec3::new(6.0, 0.0, 0.0), 2.0).unwrap();
        doc.add_object("Sphere", Geometry::Surface(sphere));

        let box_b = BRep::from_box(DVec3::new(-6.0, 0.0, 1.0), DVec3::new(2.0, 3.0, 2.0));
        doc.add_object("Box", Geometry::BRep(box_b));

        let step = export_step_string(&doc);
        assert!(step.starts_with("ISO-10303-21;"));
        assert!(step.contains("RATIONAL_B_SPLINE_CURVE"));
        assert!(step.contains("MANIFOLD_SOLID_BREP"));

        let mut doc2 = Document::new();
        let count = import_step_str(&step, &mut doc2).expect("import STEP");
        assert!(count >= 3, "expected at least 3 imported objects, got {count}");

        let bb1 = doc.visible_bounding_box();
        let bb2 = doc2.visible_bounding_box();
        assert!(bb1.min.distance(bb2.min) < 1e-3);
        assert!(bb1.max.distance(bb2.max) < 1e-3);
    }
}
