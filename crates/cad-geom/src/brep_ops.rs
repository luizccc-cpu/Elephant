use cad_math::linalg::solve_linear_system;
use cad_math::{DMat4, DVec3, Plane};

use crate::brep::{
    BRep, BRepFace, TrimLoop2D, clean_loop_2d, fit_plane_from_loop, intersect_three_planes,
    point_in_poly_uv, signed_area_uv,
};
use crate::csg::BooleanOp;
use crate::error::GeomError;
use crate::nurbs_curve::{ControlPoint, NurbsCurve};
use crate::nurbs_surface::NurbsSurface;
use crate::surface_ops::{LoftStyle, loft_nurbs, offset_srf};

/// Surface-Surface Intersection (SSI) branch with synced 3D curve and surface UV traces.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceIntersectionCurve {
    pub curve_3d: NurbsCurve,
    pub uv_a: Vec<[f64; 2]>,
    pub uv_b: Vec<[f64; 2]>,
    pub closed: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FilletSurfaceResult {
    pub fillet: NurbsSurface,
    pub rail_a_uv: Vec<[f64; 2]>,
    pub rail_b_uv: Vec<[f64; 2]>,
    pub trimmed_a: Option<BRepFace>,
    pub trimmed_b: Option<BRepFace>,
}

#[derive(Debug, Clone)]
struct UvTriMesh {
    pts: Vec<DVec3>,
    uvs: Vec<[f64; 2]>,
    tris: Vec<[usize; 3]>,
}

#[derive(Debug, Clone, Copy)]
struct RawSeg {
    p0: DVec3,
    p1: DVec3,
    uva0: [f64; 2],
    uva1: [f64; 2],
    uvb0: [f64; 2],
    uvb1: [f64; 2],
}

/// Compute intersection curve(s) between two NURBS surfaces using triangle-triangle seeding + 4D Newton polishing.
pub fn intersect_surfaces(
    srf_a: &NurbsSurface,
    srf_b: &NurbsSurface,
    tol: f64,
) -> Vec<SurfaceIntersectionCurve> {
    let bb_a = srf_a.bounding_box();
    let bb_b = srf_b.bounding_box();
    let pad = tol.max(1e-4) * 2.0;
    if bb_a.max.x + pad < bb_b.min.x
        || bb_b.max.x + pad < bb_a.min.x
        || bb_a.max.y + pad < bb_b.min.y
        || bb_b.max.y + pad < bb_a.min.y
        || bb_a.max.z + pad < bb_b.min.z
        || bb_b.max.z + pad < bb_a.min.z
    {
        return Vec::new();
    }

    let ma = tessellate_with_uv(srf_a);
    let mb = tessellate_with_uv(srf_b);

    let tri_bbs_b: Vec<(DVec3, DVec3)> = mb
        .tris
        .iter()
        .map(|&[i0, i1, i2]| {
            let p0 = mb.pts[i0];
            let p1 = mb.pts[i1];
            let p2 = mb.pts[i2];
            (p0.min(p1).min(p2), p0.max(p1).max(p2))
        })
        .collect();

    let mut raw_segs = Vec::new();
    for &[a0, a1, a2] in &ma.tris {
        let pa0 = ma.pts[a0];
        let pa1 = ma.pts[a1];
        let pa2 = ma.pts[a2];
        let min_a = pa0.min(pa1).min(pa2);
        let max_a = pa0.max(pa1).max(pa2);

        for (bi, &[b0, b1, b2]) in mb.tris.iter().enumerate() {
            let (min_b, max_b) = tri_bbs_b[bi];
            if max_a.x + 1e-7 < min_b.x
                || max_b.x + 1e-7 < min_a.x
                || max_a.y + 1e-7 < min_b.y
                || max_b.y + 1e-7 < min_a.y
                || max_a.z + 1e-7 < min_b.z
                || max_b.z + 1e-7 < min_a.z
            {
                continue;
            }
            if let Some(seg) = tri_tri_intersect(
                [pa0, pa1, pa2],
                [ma.uvs[a0], ma.uvs[a1], ma.uvs[a2]],
                [mb.pts[b0], mb.pts[b1], mb.pts[b2]],
                [mb.uvs[b0], mb.uvs[b1], mb.uvs[b2]],
            ) {
                raw_segs.push(seg);
            }
        }
    }

    if raw_segs.is_empty() {
        return Vec::new();
    }

    let diag = bb_a.size().length().max(bb_b.size().length()).max(1.0);
    let dedup_tol = (diag * 1e-5).max(tol * 0.25).max(1e-6);
    dedup_raw_segs(&mut raw_segs, dedup_tol);
    let chain_tol = (diag * 2e-3).max(tol * 2.0).max(1e-4);
    let chains = chain_ssi_segments(&mut raw_segs, srf_a, srf_b, chain_tol);

    let mut result = Vec::with_capacity(chains.len());
    for mut chain in chains {
        if chain.len() < 2 {
            continue;
        }
        for (_, uva, uvb) in &mut chain {
            let (p_ref, nuva, nuvb) = polish_ssi_point(srf_a, srf_b, *uva, *uvb);
            *uva = nuva;
            *uvb = nuvb;
            let _ = p_ref;
        }

        let mut pts3d: Vec<DVec3> = chain
            .iter()
            .map(|&(_, uva, _)| srf_a.evaluate(uva[0], uva[1]))
            .collect();
        let mut uv_a: Vec<[f64; 2]> = chain.iter().map(|c| c.1).collect();
        let mut uv_b: Vec<[f64; 2]> = chain.iter().map(|c| c.2).collect();

        // Deduplicate near-identical consecutive points
        let mut clean_p = Vec::with_capacity(pts3d.len());
        let mut clean_ua = Vec::with_capacity(uv_a.len());
        let mut clean_ub = Vec::with_capacity(uv_b.len());
        let min_step = (diag * 1e-6).max(1e-7);
        for i in 0..pts3d.len() {
            if clean_p.last().is_none_or(|&prev: &DVec3| prev.distance(pts3d[i]) > min_step) {
                clean_p.push(pts3d[i]);
                clean_ua.push(uv_a[i]);
                clean_ub.push(uv_b[i]);
            }
        }
        pts3d = clean_p;
        uv_a = clean_ua;
        uv_b = clean_ub;
        if pts3d.len() < 2 {
            continue;
        }

        let is_closed = pts3d.len() >= 4
            && pts3d.first().unwrap().distance(*pts3d.last().unwrap()) <= chain_tol * 2.0;

        let curve_3d = if is_closed {
            let mut open = pts3d.clone();
            open.pop();
            if open.len() >= 4 {
                NurbsCurve::interpolate(&open, 3, true)
                    .or_else(|_| NurbsCurve::polyline(&pts3d))
            } else {
                NurbsCurve::polyline(&pts3d)
            }
        } else if is_collinear(&pts3d, tol.max(1e-4)) {
            Ok(NurbsCurve::line(pts3d[0], *pts3d.last().unwrap()))
        } else if pts3d.len() >= 4 {
            NurbsCurve::interpolate(&pts3d, 3, false)
                .or_else(|_| NurbsCurve::polyline(&pts3d))
        } else {
            NurbsCurve::polyline(&pts3d)
        };

        if let Ok(curve_3d) = curve_3d {
            result.push(SurfaceIntersectionCurve {
                curve_3d,
                uv_a,
                uv_b,
                closed: is_closed,
            });
        }
    }

    result
}

/// Compute 3D intersection curves between two B-Reps across overlapping trimmed faces.
pub fn intersect_breps(a: &BRep, b: &BRep, tol: f64) -> Vec<NurbsCurve> {
    let mut out = Vec::new();
    for fa in &a.faces {
        let ba = fa.surface.bounding_box();
        for fb in &b.faces {
            let bb = fb.surface.bounding_box();
            let pad = tol.max(1e-4) * 2.0;
            if ba.max.x + pad < bb.min.x
                || bb.max.x + pad < ba.min.x
                || ba.max.y + pad < bb.min.y
                || bb.max.y + pad < ba.min.y
                || ba.max.z + pad < bb.min.z
                || bb.max.z + pad < ba.min.z
            {
                continue;
            }
            for sic in intersect_surfaces(&fa.surface, &fb.surface, tol) {
                let mut active_pts = Vec::new();
                for (&ua, &ub) in sic.uv_a.iter().zip(sic.uv_b.iter()) {
                    if fa.contains_uv(ua[0], ua[1]) && fb.contains_uv(ub[0], ub[1]) {
                        active_pts.push(fa.surface.evaluate(ua[0], ua[1]));
                    } else if active_pts.len() >= 2 {
                        if let Some(c) = build_curve_from_pts(&active_pts, tol) {
                            out.push(c);
                        }
                        active_pts.clear();
                    } else {
                        active_pts.clear();
                    }
                }
                if active_pts.len() >= 2 {
                    if let Some(c) = build_curve_from_pts(&active_pts, tol) {
                        out.push(c);
                    }
                }
            }
        }
    }
    out
}

/// Pull a 3D curve onto `srf` by closest-point projection.
pub fn pull_curve_to_surface(
    curve: &NurbsCurve,
    srf: &NurbsSurface,
    samples: usize,
) -> (NurbsCurve, Vec<[f64; 2]>) {
    let n = samples.max(12);
    let (u0, u1) = curve.domain();
    let mut uvs = Vec::with_capacity(n + 1);
    let mut pts3d = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = u0 + (u1 - u0) * (i as f64 / n as f64);
        let p = curve.evaluate(t);
        let (su, sv, sp, _) = srf.closest_point(p);
        uvs.push([su, sv]);
        pts3d.push(sp);
    }
    let c3d = if pts3d.len() >= 4 && !is_collinear(&pts3d, 1e-4) {
        NurbsCurve::interpolate(&pts3d, 3, false)
            .or_else(|_| NurbsCurve::polyline(&pts3d))
            .unwrap_or_else(|_| curve.clone())
    } else {
        NurbsCurve::polyline(&pts3d).unwrap_or_else(|_| curve.clone())
    };
    (c3d, uvs)
}

/// Project a 3D curve along `dir` onto `srf` via ray-surface intersections.
pub fn project_curve_to_surface(
    curve: &NurbsCurve,
    srf: &NurbsSurface,
    dir: DVec3,
    samples: usize,
) -> Option<(NurbsCurve, Vec<[f64; 2]>)> {
    let d_unit = dir.normalize_or_zero();
    if d_unit.length_squared() < 1e-12 {
        return None;
    }
    let diag = srf.bounding_box().size().length().max(1.0);
    let back_off = diag * 2.0 + 10.0;
    let n = samples.max(16);
    let (u0, u1) = curve.domain();
    let mut uvs = Vec::with_capacity(n + 1);
    let mut pts3d = Vec::with_capacity(n + 1);

    for i in 0..=n {
        let t = u0 + (u1 - u0) * (i as f64 / n as f64);
        let base_p = curve.evaluate(t);
        let ray = cad_math::Ray::new(base_p - d_unit * back_off, d_unit);
        if let Some(hit_t) = srf.ray_intersect(&ray) {
            let hit_p = ray.origin + ray.direction * hit_t;
            let (su, sv, sp, _) = srf.closest_point(hit_p);
            uvs.push([su, sv]);
            pts3d.push(sp);
        } else {
            let ray_rev = cad_math::Ray::new(base_p + d_unit * back_off, -d_unit);
            if let Some(hit_t) = srf.ray_intersect(&ray_rev) {
                let hit_p = ray_rev.origin + ray_rev.direction * hit_t;
                let (su, sv, sp, _) = srf.closest_point(hit_p);
                uvs.push([su, sv]);
                pts3d.push(sp);
            }
        }
    }
    if pts3d.len() < 2 {
        return None;
    }
    let c3d = build_curve_from_pts(&pts3d, 1e-3)?;
    Some((c3d, uvs))
}

/// Split a `BRepFace` by a 2D curve in its parameter space (supports interior closed loops and boundary-crossing splits).
pub fn split_face_with_uv_curve(
    face: &BRepFace,
    uv_curve: &[[f64; 2]],
    closed_hint: bool,
    _tol: f64,
) -> Vec<BRepFace> {
    if uv_curve.len() < 2 {
        return vec![face.clone()];
    }
    let outer = face.effective_outer_loop();
    if outer.len() < 3 {
        return vec![face.clone()];
    }

    let (u0, u1) = face.surface.domain_u();
    let (v0, v1) = face.surface.domain_v();
    let uv_span = (u1 - u0).max(v1 - v0).max(1e-6);
    let eps_uv = uv_span * 1e-3;

    // Skip if curve runs completely along existing outer boundary
    if uv_curve.iter().all(|&p| dist_to_poly_uv(p, &outer) <= eps_uv * 2.0) {
        return vec![face.clone()];
    }

    let end_gap = (uv_curve[0][0] - uv_curve.last().unwrap()[0])
        .hypot(uv_curve[0][1] - uv_curve.last().unwrap()[1]);
    let is_closed = uv_curve.len() >= 4
        && (closed_hint || end_gap <= uv_span * 0.025)
        && dist_to_poly_uv(uv_curve[0], &outer) > eps_uv * 2.0;

    if is_closed {
        let mut loop_pts = clean_loop_2d(uv_curve);
        if loop_pts.len() < 3 {
            return vec![face.clone()];
        }
        // Only split if loop lies inside outer loop
        if !point_in_poly_uv(loop_pts[0][0], loop_pts[0][1], &outer) {
            return vec![face.clone()];
        }
        if signed_area_uv(&loop_pts) < 0.0 {
            loop_pts.reverse();
        }
        let ccw_island = loop_pts.clone();
        let mut cw_hole = loop_pts;
        cw_hole.reverse();

        let mut outer_holes = Vec::new();
        let mut island_holes = Vec::new();
        for hole in &face.inner_loops {
            let hc = clean_loop_2d(&hole.points);
            if hc.len() >= 3 && point_in_poly_uv(hc[0][0], hc[0][1], &ccw_island) {
                island_holes.push(hole.clone());
            } else {
                outer_holes.push(hole.clone());
            }
        }
        outer_holes.push(TrimLoop2D {
            points: cw_hole,
            is_hole: true,
        });

        let outer_face = BRepFace {
            surface: face.surface.clone(),
            outer_loop: Some(TrimLoop2D {
                points: outer,
                is_hole: false,
            }),
            inner_loops: outer_holes,
            orientation_reversed: face.orientation_reversed,
        };
        let island_face = BRepFace {
            surface: face.surface.clone(),
            outer_loop: Some(TrimLoop2D {
                points: ccw_island,
                is_hole: false,
            }),
            inner_loops: island_holes,
            orientation_reversed: face.orientation_reversed,
        };
        return vec![outer_face, island_face];
    }

    // Open curve splitting: snap slightly-near endpoints to boundary and collect crossing chords
    let mut work_uv = uv_curve.to_vec();
    snap_endpoint_to_poly(&mut work_uv[0], &outer, uv_span * 0.015);
    let last_idx = work_uv.len() - 1;
    snap_endpoint_to_poly(&mut work_uv[last_idx], &outer, uv_span * 0.015);

    let crossings = find_polyline_loop_crossings(&work_uv, &outer, eps_uv * 0.5);
    if crossings.len() < 2 {
        return vec![face.clone()];
    }

    // Extract the longest interior chord crossing between two distinct boundary parameters
    let mut best_chord: Option<(BoundaryCrossing, BoundaryCrossing, Vec<[f64; 2]>)> = None;
    for w in crossings.windows(2) {
        let c0 = &w[0];
        let c1 = &w[1];
        if (c0.pos[0] - c1.pos[0]).hypot(c0.pos[1] - c1.pos[1]) <= eps_uv * 2.0 {
            continue;
        }
        let mut chord = extract_sub_polyline(&work_uv, c0, c1);
        if chord.len() >= 2 {
            let mid_idx = chord.len() / 2;
            let mid = if chord.len() == 2 {
                [0.5 * (chord[0][0] + chord[1][0]), 0.5 * (chord[0][1] + chord[1][1])]
            } else {
                chord[mid_idx]
            };
            if point_in_poly_uv(mid[0], mid[1], &outer)
                && dist_to_poly_uv(mid, &outer) > eps_uv * 0.1
            {
                let chord_len: f64 = chord
                    .windows(2)
                    .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
                    .sum();
                let cur_best_len: f64 = best_chord
                    .as_ref()
                    .map(|(_, _, pts)| {
                        pts.windows(2)
                            .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
                            .sum()
                    })
                    .unwrap_or(0.0);
                if chord_len > cur_best_len {
                    chord[0] = c0.pos;
                    *chord.last_mut().unwrap() = c1.pos;
                    best_chord = Some((*c0, *c1, chord));
                }
            }
        }
    }

    let Some((c_start, c_end, chord)) = best_chord else {
        return vec![face.clone()];
    };

    let (poly_a, poly_b) = split_outer_polygon(&outer, &c_start, &c_end, &chord);
    if poly_a.len() < 3 || poly_b.len() < 3 {
        return vec![face.clone()];
    }
    if signed_area_uv(&poly_a).abs() < 1e-12 || signed_area_uv(&poly_b).abs() < 1e-12 {
        return vec![face.clone()];
    }

    let mut holes_a = Vec::new();
    let mut holes_b = Vec::new();
    for hole in &face.inner_loops {
        let hc = clean_loop_2d(&hole.points);
        if hc.len() >= 3 {
            if point_in_poly_uv(hc[0][0], hc[0][1], &poly_a) {
                holes_a.push(hole.clone());
            } else {
                holes_b.push(hole.clone());
            }
        }
    }

    let fa = BRepFace {
        surface: face.surface.clone(),
        outer_loop: Some(TrimLoop2D {
            points: poly_a,
            is_hole: false,
        }),
        inner_loops: holes_a,
        orientation_reversed: face.orientation_reversed,
    };
    let fb = BRepFace {
        surface: face.surface.clone(),
        outer_loop: Some(TrimLoop2D {
            points: poly_b,
            is_hole: false,
        }),
        inner_loops: holes_b,
        orientation_reversed: face.orientation_reversed,
    };
    vec![fa, fb]
}

/// Trim a `BRepFace` by a 3D cutting curve (pull or directional projection) discarding the side near `pick_discard_pt`.
pub fn trim_face_with_curve(
    face: &BRepFace,
    curve: &NurbsCurve,
    pick_discard_pt: DVec3,
    proj_dir: Option<DVec3>,
    tol: f64,
) -> Option<BRepFace> {
    let (_, uv_pts) = if let Some(dir) = proj_dir {
        project_curve_to_surface(curve, &face.surface, dir, 32)
            .unwrap_or_else(|| pull_curve_to_surface(curve, &face.surface, 32))
    } else {
        pull_curve_to_surface(curve, &face.surface, 32)
    };
    let closed = curve.is_closed();
    let pieces = split_face_with_uv_curve(face, &uv_pts, closed, tol);
    if pieces.len() <= 1 {
        return None;
    }
    let (u_p, v_p, _, _) = face.surface.closest_point(pick_discard_pt);
    // Keep the piece that does NOT contain `(u_p, v_p)`, or whose interior center is farthest from `pick_discard_pt`
    for piece in &pieces {
        if !piece.contains_uv(u_p, v_p) {
            return Some(piece.clone());
        }
    }
    pieces.into_iter().max_by(|a, b| {
        let da = a
            .interior_sample_point()
            .map(|p| p.2.distance(pick_discard_pt))
            .unwrap_or(0.0);
        let db = b
            .interior_sample_point()
            .map(|p| p.2.distance(pick_discard_pt))
            .unwrap_or(0.0);
        da.total_cmp(&db)
    })
}

/// Remove trimming loops from selected faces (or all faces if `face_indices` is `None`).
pub fn untrim_brep(brep: &BRep, face_indices: Option<&[usize]>) -> BRep {
    let mut faces = brep.faces.clone();
    for (i, face) in faces.iter_mut().enumerate() {
        if face_indices.is_none_or(|list| list.contains(&i)) {
            face.outer_loop = None;
            face.inner_loops.clear();
        }
    }
    BRep::from_faces(faces, 1e-4)
}

/// True NURBS B-Rep solid Boolean operation (`Union`, `Difference`, `Intersection`).
pub fn brep_boolean(a: &BRep, b: &BRep, op: BooleanOp, tol: f64) -> Result<BRep, GeomError> {
    let split_a = split_brep_against(a, b, tol);
    let split_b = split_brep_against(b, a, tol);

    let diag = a
        .bounding_box()
        .union(&b.bounding_box())
        .size()
        .length()
        .max(1.0);
    let probe_eps = (diag * 1e-3).max(tol * 2.0).clamp(1e-4, 0.05);

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum FaceSide {
        Outside,
        Inside,
        CoplanarSame,
        CoplanarOpp,
    }

    let classify = |face: &BRepFace, solid: &BRep| -> FaceSide {
        let Some((_, _, p, n)) = face.interior_sample_point() else {
            return FaceSide::Outside;
        };
        let n_unit = n.normalize_or(DVec3::Z);
        let in_plus = solid.contains_point(p + n_unit * probe_eps);
        let in_minus = solid.contains_point(p - n_unit * probe_eps);
        match (in_plus, in_minus) {
            (true, true) => FaceSide::Inside,
            (false, false) => FaceSide::Outside,
            (false, true) => FaceSide::CoplanarSame,
            (true, false) => FaceSide::CoplanarOpp,
        }
    };

    let mut kept_faces = Vec::new();
    for face in split_a {
        match (op, classify(&face, b)) {
            (BooleanOp::Union, FaceSide::Outside | FaceSide::CoplanarSame) => kept_faces.push(face),
            (BooleanOp::Difference, FaceSide::Outside | FaceSide::CoplanarOpp) => {
                kept_faces.push(face)
            }
            (BooleanOp::Intersection, FaceSide::Inside | FaceSide::CoplanarSame) => {
                kept_faces.push(face)
            }
            _ => {}
        }
    }

    for mut face in split_b {
        match (op, classify(&face, a)) {
            (BooleanOp::Union, FaceSide::Outside) => kept_faces.push(face),
            (BooleanOp::Difference, FaceSide::Inside) => {
                face.flip_orientation();
                kept_faces.push(face);
            }
            (BooleanOp::Intersection, FaceSide::Inside) => kept_faces.push(face),
            _ => {}
        }
    }

    if kept_faces.is_empty() {
        return Err(GeomError::Degenerate("B-Rep Boolean produced empty result"));
    }

    Ok(BRep::from_faces(kept_faces, tol.max(1e-4)))
}

/// Split solid `a` by cutter `b` into difference and intersection B-Rep pieces.
pub fn brep_boolean_split(a: &BRep, b: &BRep, tol: f64) -> Result<Vec<BRep>, GeomError> {
    let mut pieces = Vec::new();
    if let Ok(diff) = brep_boolean(a, b, BooleanOp::Difference, tol) {
        if !diff.faces.is_empty() {
            pieces.push(diff);
        }
    }
    if let Ok(inter) = brep_boolean(a, b, BooleanOp::Intersection, tol) {
        if !inter.faces.is_empty() {
            pieces.push(inter);
        }
    }
    if pieces.is_empty() {
        return Err(GeomError::Degenerate("BRep BooleanSplit produced no pieces"));
    }
    Ok(pieces)
}

/// Cap closed planar naked-edge boundary loops on an open `BRep` to form a solid.
pub fn cap_planar_holes(brep: &BRep, tol: f64) -> Result<BRep, GeomError> {
    let naked = brep.naked_edges();
    if naked.is_empty() {
        return Ok(brep.clone());
    }

    let stitch_tol = (tol * 8.0).max(1e-3);
    let mut pool: Vec<Vec<DVec3>> = naked
        .into_iter()
        .map(|ei| {
            let c = &brep.edges[ei].curve;
            if c.degree == 1 {
                c.control_points.iter().map(|cp| cp.position).collect()
            } else {
                c.sample_polyline(32)
            }
        })
        .collect();

    let mut loops: Vec<Vec<DVec3>> = Vec::new();
    while let Some(mut chain) = pool.pop() {
        loop {
            let Some(&head) = chain.first() else { break };
            let Some(&tail) = chain.last() else { break };
            if chain.len() >= 4 && head.distance(tail) <= stitch_tol {
                break;
            }
            let mut joined = false;
            for i in 0..pool.len() {
                let s = pool[i][0];
                let e = *pool[i].last().unwrap();
                if tail.distance(s) <= stitch_tol {
                    let next = pool.remove(i);
                    chain.extend(next.into_iter().skip(1));
                    joined = true;
                    break;
                } else if tail.distance(e) <= stitch_tol {
                    let mut next = pool.remove(i);
                    next.reverse();
                    chain.extend(next.into_iter().skip(1));
                    joined = true;
                    break;
                } else if head.distance(e) <= stitch_tol {
                    let mut next = pool.remove(i);
                    next.pop();
                    next.extend(chain);
                    chain = next;
                    joined = true;
                    break;
                } else if head.distance(s) <= stitch_tol {
                    let mut next = pool.remove(i);
                    next.reverse();
                    next.pop();
                    next.extend(chain);
                    chain = next;
                    joined = true;
                    break;
                }
            }
            if !joined {
                break;
            }
        }
        if chain.len() >= 3 {
            loops.push(chain);
        }
    }

    let mut faces = brep.faces.clone();
    let mut capped = 0usize;
    for lp in loops {
        let mut open = lp.clone();
        if open.first().unwrap().distance(*open.last().unwrap()) <= stitch_tol {
            open.pop();
        }
        if open.len() < 3 {
            continue;
        }
        let Some(plane) = fit_plane_from_loop(&open) else {
            continue;
        };
        let max_plane_err = open
            .iter()
            .map(|&p| (p - plane.origin).dot(plane.z_axis).abs())
            .fold(0.0_f64, f64::max);
        if max_plane_err > (tol * 20.0).max(0.02) {
            continue;
        }
        let poly_curve = if let Ok(c) = NurbsCurve::polyline(&{
            let mut closed_pts = open.clone();
            closed_pts.push(open[0]);
            closed_pts
        }) {
            c
        } else {
            continue;
        };
        if let Ok(cap_brep) = BRep::from_planar_curves(&[poly_curve]) {
            faces.extend(cap_brep.faces);
            capped += 1;
        }
    }

    if capped == 0 {
        return Err(GeomError::Degenerate("no closed planar naked holes found to cap"));
    }

    let mut out = BRep::from_faces(faces, tol.max(1e-3));
    if out.is_solid() {
        out.orient_outward_consistent();
    }
    Ok(out)
}

/// Rolling-ball NURBS fillet surface between `srf_a` and `srf_b` with optional face trimming.
pub fn fillet_srf(
    srf_a: &NurbsSurface,
    srf_b: &NurbsSurface,
    radius: f64,
    pick_a: Option<DVec3>,
    pick_b: Option<DVec3>,
    trim_inputs: bool,
    tol: f64,
) -> Result<FilletSurfaceResult, GeomError> {
    if !radius.is_finite() || radius <= 1e-5 {
        return Err(GeomError::Degenerate("fillet radius must be positive"));
    }
    let ssi = intersect_surfaces(srf_a, srf_b, tol.max(1e-3));
    let spine_curve = if let Some(first) = ssi.into_iter().next() {
        first.curve_3d
    } else {
        // Fallback: use closest boundary curve if surfaces meet cleanly along an edge
        let (ea, _) = crate::surface_ops::closest_edge_pair(srf_a, srf_b);
        srf_a.boundary_curve(ea)?
    };

    let (_, _, ca) = srf_a.surface_frame(0.5, 0.5).let_center_from_srf(srf_a);
    let (_, _, cb) = srf_b.surface_frame(0.5, 0.5).let_center_from_srf(srf_b);
    let ref_a = pick_a.unwrap_or(ca);
    let ref_b = pick_b.unwrap_or(cb);

    let fillet = build_rolling_ball_fillet_strip(
        srf_a,
        false,
        srf_b,
        false,
        &spine_curve,
        radius,
        Some((ref_a, ref_b)),
    )?;

    let rail_a_uv: Vec<[f64; 2]> = (0..=24)
        .map(|i| {
            let u = i as f64 / 24.0;
            let p = fillet.evaluate(u, 0.0);
            let (su, sv, _, _) = srf_a.closest_point(p);
            [su, sv]
        })
        .collect();
    let rail_b_uv: Vec<[f64; 2]> = (0..=24)
        .map(|i| {
            let u = i as f64 / 24.0;
            let p = fillet.evaluate(u, 1.0);
            let (su, sv, _, _) = srf_b.closest_point(p);
            [su, sv]
        })
        .collect();

    let (trimmed_a, trimmed_b) = if trim_inputs {
        let fa = BRepFace::untrimmed(srf_a.clone());
        let fb = BRepFace::untrimmed(srf_b.clone());
        let pa = split_face_with_uv_curve(&fa, &rail_a_uv, false, tol);
        let pb = split_face_with_uv_curve(&fb, &rail_b_uv, false, tol);
        let ka = pa.into_iter().min_by(|x, y| {
            let dx = x
                .interior_sample_point()
                .map(|s| s.2.distance(ref_a))
                .unwrap_or(f64::INFINITY);
            let dy = y
                .interior_sample_point()
                .map(|s| s.2.distance(ref_a))
                .unwrap_or(f64::INFINITY);
            dx.total_cmp(&dy)
        });
        let kb = pb.into_iter().min_by(|x, y| {
            let dx = x
                .interior_sample_point()
                .map(|s| s.2.distance(ref_b))
                .unwrap_or(f64::INFINITY);
            let dy = y
                .interior_sample_point()
                .map(|s| s.2.distance(ref_b))
                .unwrap_or(f64::INFINITY);
            dx.total_cmp(&dy)
        });
        (ka, kb)
    } else {
        (None, None)
    };

    Ok(FilletSurfaceResult {
        fillet,
        rail_a_uv,
        rail_b_uv,
        trimmed_a,
        trimmed_b,
    })
}

trait FrameHelper {
    fn let_center_from_srf(&self, srf: &NurbsSurface) -> (DVec3, DVec3, DVec3);
}
impl FrameHelper for crate::nurbs_surface::SurfaceFrame {
    fn let_center_from_srf(&self, srf: &NurbsSurface) -> (DVec3, DVec3, DVec3) {
        let (u0, u1) = srf.domain_u();
        let (v0, v1) = srf.domain_v();
        (self.origin, self.normal, srf.evaluate(0.5 * (u0 + u1), 0.5 * (v0 + v1)))
    }
}

/// Find tangent-continuous connected chain of B-Rep edges starting at `start_edge` (Rhino 9 continuing chain).
pub fn tangent_edge_chain(brep: &BRep, start_edge: usize, angle_tol_rad: f64) -> Vec<usize> {
    if start_edge >= brep.edges.len() {
        return Vec::new();
    }
    let mut chain = vec![start_edge];
    let mut visited = vec![false; brep.edges.len()];
    visited[start_edge] = true;

    let edge_end_tangent = |ei: usize, at_end: bool| -> (DVec3, DVec3) {
        let c = &brep.edges[ei].curve;
        let (u0, u1) = c.domain();
        if at_end {
            let (p, d) = c.evaluate_derivative(u1);
            (p, d.normalize_or_zero())
        } else {
            let (p, d) = c.evaluate_derivative(u0);
            (p, -d.normalize_or_zero())
        }
    };

    for &dir_end in &[true, false] {
        let mut cur_edge = start_edge;
        let mut cur_at_end = dir_end;
        loop {
            let (p_tip, t_out) = edge_end_tangent(cur_edge, cur_at_end);
            let mut next_found = None;
            for (cand_i, _) in brep.edges.iter().enumerate() {
                if visited[cand_i] {
                    continue;
                }
                for &cand_at_end in &[false, true] {
                    let (cp, ct) = edge_end_tangent(cand_i, cand_at_end);
                    // Wait: `ct` is outward from `cand`'s endpoint; continuing smoothly requires `t_out` opposite `ct`!
                    if p_tip.distance(cp) < 1e-3 && t_out.angle_between(-ct) <= angle_tol_rad {
                        next_found = Some((cand_i, !cand_at_end));
                        break;
                    }
                }
                if next_found.is_some() {
                    break;
                }
            }
            if let Some((ni, next_end)) = next_found {
                visited[ni] = true;
                chain.push(ni);
                cur_edge = ni;
                cur_at_end = next_end;
            } else {
                break;
            }
        }
    }

    chain
}

/// Rolling-ball fillet along B-Rep solid/polysurface edge(s) with tangent-chain continuation & end-cap trimming.
pub fn fillet_brep_edges(
    brep: &BRep,
    edge_indices: &[usize],
    radius: f64,
    tol: f64,
) -> Result<BRep, GeomError> {
    blend_brep_edges_impl(brep, edge_indices, radius, true, tol)
}

/// Setback chamfer along B-Rep solid/polysurface edge(s) with tangent-chain continuation & end-cap trimming.
pub fn chamfer_brep_edges(
    brep: &BRep,
    edge_indices: &[usize],
    distance: f64,
    tol: f64,
) -> Result<BRep, GeomError> {
    blend_brep_edges_impl(brep, edge_indices, distance, false, tol)
}

fn blend_brep_edges_impl(
    brep: &BRep,
    edge_indices: &[usize],
    size: f64,
    is_fillet: bool,
    tol: f64,
) -> Result<BRep, GeomError> {
    if !size.is_finite() || size <= 1e-5 {
        return Err(GeomError::Degenerate("edge blend size must be positive"));
    }
    if edge_indices.is_empty() {
        return Err(GeomError::Degenerate("select at least one B-Rep edge"));
    }

    let mut targets: Vec<usize> = Vec::new();
    for &ei in edge_indices {
        if ei < brep.edges.len() {
            for ce in tangent_edge_chain(brep, ei, 10.0_f64.to_radians()) {
                if !targets.contains(&ce) {
                    targets.push(ce);
                }
            }
        }
    }

    let mut cur = brep.clone();
    // Resolve edge midpoints to match sequentially as topology updates across multiple edges
    let midpoints: Vec<DVec3> = targets
        .into_iter()
        .filter_map(|ei| {
            let e = brep.edges.get(ei)?;
            if e.adjacent_faces.len() != 2 || e.adjacent_faces[0] == e.adjacent_faces[1] {
                return None;
            }
            let (u0, u1) = e.curve.domain();
            Some(e.curve.evaluate(0.5 * (u0 + u1)))
        })
        .collect();

    if midpoints.is_empty() {
        return Err(GeomError::Degenerate(
            "selected edge(s) are not manifold 2-face edges",
        ));
    }

    for mid in midpoints {
        let Some(ei) = cur.edges.iter().position(|e| {
            e.adjacent_faces.len() == 2
                && e.adjacent_faces[0] != e.adjacent_faces[1]
                && e.curve.closest_point(mid).2 < (tol * 20.0).max(0.05)
        }) else {
            continue;
        };
        cur = apply_single_edge_blend(&cur, ei, size, is_fillet, tol)?;
    }

    Ok(cur)
}

fn apply_single_edge_blend(
    brep: &BRep,
    edge_idx: usize,
    size: f64,
    is_fillet: bool,
    tol: f64,
) -> Result<BRep, GeomError> {
    let edge = &brep.edges[edge_idx];
    let fa_idx = edge.adjacent_faces[0];
    let fb_idx = edge.adjacent_faces[1];
    let face_a = &brep.faces[fa_idx];
    let face_b = &brep.faces[fb_idx];

    let blend_srf = build_rolling_ball_or_chamfer_strip(
        face_a,
        face_b,
        &edge.curve,
        size,
        is_fillet,
    )?;

    let samples = 16usize;
    let rail_a_3d: Vec<DVec3> = (0..=samples)
        .map(|i| blend_srf.evaluate(i as f64 / samples as f64, 0.0))
        .collect();
    let rail_b_3d: Vec<DVec3> = (0..=samples)
        .map(|i| blend_srf.evaluate(i as f64 / samples as f64, 1.0))
        .collect();

    let rail_a_uv: Vec<[f64; 2]> = rail_a_3d
        .iter()
        .map(|&p| {
            let (u, v, _, _) = face_a.surface.closest_point(p);
            [u, v]
        })
        .collect();
    let rail_b_uv: Vec<[f64; 2]> = rail_b_3d
        .iter()
        .map(|&p| {
            let (u, v, _, _) = face_b.surface.closest_point(p);
            [u, v]
        })
        .collect();

    let (u0, u1) = edge.curve.domain();
    let edge_mid = edge.curve.evaluate(0.5 * (u0 + u1));

    let keep_farther_piece = |face: &BRepFace, uv: &[[f64; 2]]| -> BRepFace {
        let pieces = split_face_with_uv_curve(face, uv, false, tol);
        if pieces.len() <= 1 {
            return face.clone();
        }
        pieces
            .into_iter()
            .max_by(|p1, p2| {
                let d1 = p1
                    .interior_sample_point()
                    .map(|s| s.2.distance(edge_mid))
                    .unwrap_or(0.0);
                let d2 = p2
                    .interior_sample_point()
                    .map(|s| s.2.distance(edge_mid))
                    .unwrap_or(0.0);
                d1.total_cmp(&d2)
            })
            .unwrap()
    };

    let trimmed_a = keep_farther_piece(face_a, &rail_a_uv);
    let trimmed_b = keep_farther_piece(face_b, &rail_b_uv);

    // Sample cross-section profiles at start (u=0) and end (u=1) to notch adjacent end-cap faces
    let cap_samples = if is_fillet { 12 } else { 1 };
    let profile_start: Vec<DVec3> = (0..=cap_samples)
        .map(|k| blend_srf.evaluate(0.0, k as f64 / cap_samples as f64))
        .collect();
    let profile_end: Vec<DVec3> = (0..=cap_samples)
        .map(|k| blend_srf.evaluate(1.0, k as f64 / cap_samples as f64))
        .collect();

    let v_start_pos = edge.curve.evaluate(u0);
    let v_end_pos = edge.curve.evaluate(u1);

    let mut new_faces = Vec::with_capacity(brep.faces.len() + 1);
    for (fi, orig_face) in brep.faces.iter().enumerate() {
        if fi == fa_idx {
            new_faces.push(trimmed_a.clone());
        } else if fi == fb_idx {
            new_faces.push(trimmed_b.clone());
        } else {
            let mut updated = orig_face.clone();
            notch_cap_face_at_corner(&mut updated, v_start_pos, &profile_start, tol);
            notch_cap_face_at_corner(&mut updated, v_end_pos, &profile_end, tol);
            new_faces.push(updated);
        }
    }

    let mut blend_face = BRepFace::untrimmed(blend_srf);
    if let (Some((_, _, pa, na)), Some((_, _, pf, nf))) =
        (trimmed_a.interior_sample_point(), blend_face.interior_sample_point())
    {
        // Keep blend normal consistent with adjacent face normals
        let (u_near, v_near, _, _) = trimmed_a.surface.closest_point(blend_face.surface.evaluate(0.5, 0.0));
        let (_, na_seam) = trimmed_a.evaluate_point_and_normal(u_near, v_near);
        let (_, nf_seam) = blend_face.evaluate_point_and_normal(0.5, 0.0);
        let _ = (pa, na, pf, nf);
        if na_seam.dot(nf_seam) < 0.0 {
            blend_face.flip_orientation();
        }
    }
    new_faces.push(blend_face);

    let mut out = BRep::from_faces(new_faces, tol.max(1e-3));
    if out.is_solid() {
        out.orient_outward_consistent();
    }
    Ok(out)
}

/// Hollow a closed solid `BRep` into a thin-walled shell keeping outer faces except `open_face_indices`.
pub fn shell_brep(
    brep: &BRep,
    open_face_indices: &[usize],
    thickness: f64,
    tol: f64,
) -> Result<BRep, GeomError> {
    if !thickness.is_finite() || thickness <= 1e-5 {
        return Err(GeomError::Degenerate("shell thickness must be positive"));
    }
    let kept_indices: Vec<usize> = (0..brep.faces.len())
        .filter(|fi| !open_face_indices.contains(fi))
        .collect();
    if kept_indices.is_empty() {
        return Err(GeomError::Degenerate("shell must retain at least one kept face"));
    }

    let mut out_faces: Vec<BRepFace> = Vec::new();
    for &fi in &kept_indices {
        out_faces.push(brep.faces[fi].clone());
    }

    // Offset plane equations for planar faces: `(point_on_offset_plane, outward_normal)`
    let face_planes: Vec<Option<(DVec3, DVec3)>> = brep
        .faces
        .iter()
        .enumerate()
        .map(|(fi, f)| {
            if f.surface.degree_u == 1 && f.surface.degree_v == 1 && !open_face_indices.contains(&fi)
            {
                let (u0, u1) = f.surface.domain_u();
                let (v0, v1) = f.surface.domain_v();
                let (p, n) = f.evaluate_point_and_normal(0.5 * (u0 + u1), 0.5 * (v0 + v1));
                Some((p - n * thickness, n))
            } else if open_face_indices.contains(&fi)
                && f.surface.degree_u == 1
                && f.surface.degree_v == 1
            {
                // Rim vertices stay on the open face's plane
                let (u0, u1) = f.surface.domain_u();
                let (v0, v1) = f.surface.domain_v();
                let (p, n) = f.evaluate_point_and_normal(0.5 * (u0 + u1), 0.5 * (v0 + v1));
                Some((p, n))
            } else {
                None
            }
        })
        .collect();

    let all_planar = face_planes.iter().all(|p| p.is_some());

    // Compute mapped inner vertices if all faces are planar; otherwise offset NURBS surfaces individually
    if all_planar {
        let mut inner_vert_pos = Vec::with_capacity(brep.vertices.len());
        for v in &brep.vertices {
            let mut planes = Vec::new();
            for (fi, f) in brep.faces.iter().enumerate() {
                let (u, vv, sp, d) = f.surface.closest_point(v.position);
                if d <= (tol * 20.0).max(0.05) && f.contains_uv(u, vv) {
                    if let Some((pp, pn)) = face_planes[fi] {
                        let _ = sp;
                        // Deduplicate parallel planes
                        if planes
                            .iter()
                            .all(|&(_, en): &(DVec3, DVec3)| en.dot(pn).abs() < 0.99)
                        {
                            planes.push((pp, pn));
                        }
                    }
                }
            }
            let mapped = if planes.len() >= 3 {
                intersect_three_planes(
                    planes[0].0,
                    planes[0].1,
                    planes[1].0,
                    planes[1].1,
                    planes[2].0,
                    planes[2].1,
                )
                .unwrap_or(v.position)
            } else {
                v.position
            };
            inner_vert_pos.push(mapped);
        }

        let map_pt_to_inner = |p: DVec3| -> DVec3 {
            brep.vertices
                .iter()
                .enumerate()
                .min_by(|a, b| a.1.position.distance(p).total_cmp(&b.1.position.distance(p)))
                .filter(|(_, v)| v.position.distance(p) <= (tol * 20.0).max(0.05))
                .map(|(vi, _)| inner_vert_pos[vi])
                .unwrap_or(p)
        };

        // Inner cavity faces (reversed inward orientation so solid encloses wall thickness)
        for &fi in &kept_indices {
            let orig = &brep.faces[fi];
            let loop_uv = orig.effective_outer_loop();
            let inner_loop_3d: Vec<DVec3> = loop_uv
                .iter()
                .map(|&[u, v]| map_pt_to_inner(orig.surface.evaluate(u, v)))
                .collect();
            let mut closed_pts = inner_loop_3d.clone();
            closed_pts.push(inner_loop_3d[0]);
            if let Ok(poly) = NurbsCurve::polyline(&closed_pts) {
                if let Ok(mut inner_b) = BRep::from_planar_curves(&[poly]) {
                    if let Some(mut f_in) = inner_b.faces.pop() {
                        let (_, orig_n) = orig.evaluate_point_and_normal(0.5, 0.5);
                        let (_, in_n) = f_in.evaluate_point_and_normal(0.5, 0.5);
                        if orig_n.dot(in_n) > 0.0 {
                            f_in.flip_orientation();
                        }
                        out_faces.push(f_in);
                    }
                }
            }
        }

        // Ruled rim faces connecting open boundary edges to corresponding inner rim edges
        for edge in &brep.edges {
            let open_cnt = edge
                .adjacent_faces
                .iter()
                .filter(|fi| open_face_indices.contains(fi))
                .count();
            let kept_cnt = edge
                .adjacent_faces
                .iter()
                .filter(|fi| !open_face_indices.contains(fi))
                .count();
            if open_cnt > 0 && kept_cnt > 0 {
                let (u0, u1) = edge.curve.domain();
                let a0 = edge.curve.evaluate(u0);
                let a1 = edge.curve.evaluate(u1);
                let b0 = map_pt_to_inner(a0);
                let b1 = map_pt_to_inner(a1);
                let rim_srf = NurbsSurface::new(
                    1,
                    1,
                    vec![0.0, 0.0, 1.0, 1.0],
                    vec![0.0, 0.0, 1.0, 1.0],
                    2,
                    2,
                    vec![
                        ControlPoint::unweighted(a0),
                        ControlPoint::unweighted(b0),
                        ControlPoint::unweighted(a1),
                        ControlPoint::unweighted(b1),
                    ],
                )?;
                out_faces.push(BRepFace::untrimmed(rim_srf));
            }
        }
    } else {
        // Curved/freeform B-Rep shell fallback: offset each kept surface & stitch rim walls
        for &fi in &kept_indices {
            let orig = &brep.faces[fi];
            let dir = if orig.orientation_reversed {
                thickness
            } else {
                -thickness
            };
            let off_srf = offset_srf(&orig.surface, dir)?;
            let mut inner_face = BRepFace {
                surface: off_srf,
                outer_loop: orig.outer_loop.clone(),
                inner_loops: orig.inner_loops.clone(),
                orientation_reversed: !orig.orientation_reversed,
            };
            let _ = &mut inner_face;
            out_faces.push(inner_face);
        }

        for edge in &brep.edges {
            let is_open_boundary = edge
                .adjacent_faces
                .iter()
                .any(|fi| open_face_indices.contains(fi))
                || edge.adjacent_faces.len() == 1;
            let Some(&kept_fi) = edge
                .adjacent_faces
                .iter()
                .find(|fi| !open_face_indices.contains(fi))
            else {
                continue;
            };
            if is_open_boundary {
                let outer_c = &edge.curve;
                let kept_face = &brep.faces[kept_fi];
                let inner_c = NurbsCurve::interpolate(
                    &(0..=12)
                        .map(|i| {
                            let (u0, u1) = outer_c.domain();
                            let p = outer_c.evaluate(u0 + (u1 - u0) * (i as f64 / 12.0));
                            let (su, sv, _, _) = kept_face.surface.closest_point(p);
                            let (_, n) = kept_face.evaluate_point_and_normal(su, sv);
                            p - n * thickness
                        })
                        .collect::<Vec<_>>(),
                    3,
                    false,
                )
                .or_else(|_| Ok::<NurbsCurve, GeomError>(outer_c.clone()))?;
                let rim = loft_nurbs(&[outer_c.clone(), inner_c], LoftStyle::Straight, false)?;
                out_faces.push(BRepFace::untrimmed(rim));
            }
        }
    }

    let mut out = BRep::from_faces(out_faces, tol.max(1e-3));
    if out.is_solid() {
        out.orient_outward_consistent();
    }
    Ok(out)
}

/// Direct-modeling PushPull: extrude or offset a face along its outward normal while updating adjacent walls.
pub fn push_pull_face(
    brep: &BRep,
    face_idx: usize,
    distance: f64,
    tol: f64,
) -> Result<BRep, GeomError> {
    if face_idx >= brep.faces.len() {
        return Err(GeomError::Degenerate("PushPull face index out of range"));
    }
    if distance.abs() <= 1e-6 {
        return Ok(brep.clone());
    }

    let target = &brep.faces[face_idx];
    let Some((_, _, _, n_target)) = target.interior_sample_point() else {
        return Err(GeomError::Degenerate("PushPull face has degenerate normal"));
    };
    let shift = n_target.normalize_or(DVec3::Z) * distance;
    let boundary_pts: Vec<DVec3> = target
        .effective_outer_loop()
        .into_iter()
        .map(|[u, v]| target.surface.evaluate(u, v))
        .collect();
    let match_tol = (tol * 20.0).max(0.02);

    let mut new_faces = Vec::with_capacity(brep.faces.len() + 4);
    for (fi, face) in brep.faces.iter().enumerate() {
        if fi == face_idx {
            let mut moved = face.clone();
            moved.surface.transform(&DMat4::from_translation(shift));
            new_faces.push(moved);
            continue;
        }

        // Check if `face` shares boundary points with `face_idx` and is roughly side-wall planar
        let is_planar = face.surface.degree_u == 1 && face.surface.degree_v == 1;
        let wall_normal = face
            .interior_sample_point()
            .map(|s| s.3)
            .unwrap_or(DVec3::Z);
        let is_perpendicular_wall = is_planar && wall_normal.dot(n_target).abs() < 0.7;

        let shared_edge = brep.edges.iter().any(|e| {
            e.adjacent_faces.contains(&face_idx) && e.adjacent_faces.contains(&fi)
        });

        if is_perpendicular_wall && shared_edge {
            if !face.is_trimmed() {
                let mut updated = face.clone();
                for cp in &mut updated.surface.control_points {
                    if boundary_pts.iter().any(|bp| bp.distance(cp.position) <= match_tol)
                        || is_point_on_loop_3d(cp.position, &boundary_pts, match_tol)
                    {
                        cp.position += shift;
                    }
                }
                new_faces.push(updated);
            } else {
                let loop_uv = face.effective_outer_loop();
                let mut pts3d: Vec<DVec3> = loop_uv
                    .iter()
                    .map(|&[u, v]| {
                        let p = face.surface.evaluate(u, v);
                        if is_point_on_loop_3d(p, &boundary_pts, match_tol) {
                            p + shift
                        } else {
                            p
                        }
                    })
                    .collect();
                pts3d.push(pts3d[0]);
                if let Ok(poly) = NurbsCurve::polyline(&pts3d) {
                    if let Ok(mut b) = BRep::from_planar_curves(&[poly]) {
                        if let Some(mut f_new) = b.faces.pop() {
                            let (_, new_n) = f_new.evaluate_point_and_normal(0.5, 0.5);
                            if new_n.dot(wall_normal) < 0.0 {
                                f_new.flip_orientation();
                            }
                            new_faces.push(f_new);
                            continue;
                        }
                    }
                }
                new_faces.push(face.clone());
            }
        } else {
            new_faces.push(face.clone());
            // If non-wall or trimmed boundary touches pushed face, stitch connecting strip
            if shared_edge {
                for edge in &brep.edges {
                    if edge.adjacent_faces.contains(&face_idx) && edge.adjacent_faces.contains(&fi)
                    {
                        let mut shifted_edge = edge.curve.clone();
                        shifted_edge.transform(&DMat4::from_translation(shift));
                        if let Ok(wall) =
                            loft_nurbs(&[edge.curve.clone(), shifted_edge], LoftStyle::Straight, false)
                        {
                            new_faces.push(BRepFace::untrimmed(wall));
                        }
                    }
                }
            }
        }
    }

    let mut out = BRep::from_faces(new_faces, tol.max(1e-3));
    if out.is_solid() {
        out.orient_outward_consistent();
    }
    Ok(out)
}

/// Rhino-style Inset: offset a face boundary inward and optionally recess/boss (`depth`).
pub fn inset_face(
    brep: &BRep,
    face_idx: usize,
    inset_distance: f64,
    depth: f64,
    tol: f64,
) -> Result<BRep, GeomError> {
    if face_idx >= brep.faces.len() {
        return Err(GeomError::Degenerate("Inset face index out of range"));
    }
    if !inset_distance.is_finite() || inset_distance <= 1e-5 {
        return Err(GeomError::Degenerate("inset distance must be positive"));
    }

    let face = &brep.faces[face_idx];
    let outer_uv = face.effective_outer_loop();
    let pts3d: Vec<DVec3> = outer_uv
        .iter()
        .map(|&[u, v]| face.surface.evaluate(u, v))
        .collect();
    if pts3d.len() < 3 {
        return Err(GeomError::Degenerate("Inset face loop invalid"));
    }
    let Some((_, _, _, n_face)) = face.interior_sample_point() else {
        return Err(GeomError::Degenerate("Inset face normal degenerate"));
    };
    let plane = fit_plane_from_loop(&pts3d).unwrap_or_else(|| Plane {
        origin: pts3d[0],
        x_axis: (pts3d[1] - pts3d[0]).normalize_or(DVec3::X),
        y_axis: n_face.cross((pts3d[1] - pts3d[0]).normalize_or(DVec3::X)).normalize_or(DVec3::Y),
        z_axis: n_face,
    });

    // Offset polygon edges inward in the local plane
    let mut pts2d: Vec<[f64; 2]> = pts3d
        .iter()
        .map(|&p| {
            let d = p - plane.origin;
            [d.dot(plane.x_axis), d.dot(plane.y_axis)]
        })
        .collect();
    if signed_area_uv(&pts2d) < 0.0 {
        pts2d.reverse();
    }
    let inset2d = miter_offset_polygon_2d(&pts2d, -inset_distance);
    if inset2d.len() < 3 || signed_area_uv(&inset2d) <= 1e-9 {
        return Err(GeomError::Degenerate("inset distance too large for face"));
    }

    let inset3d: Vec<DVec3> = inset2d
        .iter()
        .map(|&[x, y]| plane.origin + plane.x_axis * x + plane.y_axis * y)
        .collect();
    let inset_uv: Vec<[f64; 2]> = inset3d
        .iter()
        .map(|&p| {
            let (u, v, _, _) = face.surface.closest_point(p);
            [u, v]
        })
        .collect();

    let mut frame_holes = face.inner_loops.clone();
    let mut cw_hole = inset_uv.clone();
    if signed_area_uv(&cw_hole) > 0.0 {
        cw_hole.reverse();
    }
    frame_holes.push(TrimLoop2D {
        points: cw_hole,
        is_hole: true,
    });
    let frame_face = BRepFace {
        surface: face.surface.clone(),
        outer_loop: Some(TrimLoop2D {
            points: outer_uv,
            is_hole: false,
        }),
        inner_loops: frame_holes,
        orientation_reversed: face.orientation_reversed,
    };

    let shift = n_face.normalize_or(DVec3::Z) * depth;
    let mut panel_face = BRepFace {
        surface: face.surface.clone(),
        outer_loop: Some(TrimLoop2D {
            points: inset_uv,
            is_hole: false,
        }),
        inner_loops: Vec::new(),
        orientation_reversed: face.orientation_reversed,
    };
    if depth.abs() > 1e-6 {
        panel_face.surface.transform(&DMat4::from_translation(shift));
    }

    let mut new_faces = Vec::with_capacity(brep.faces.len() + inset3d.len() + 2);
    for (fi, orig) in brep.faces.iter().enumerate() {
        if fi == face_idx {
            new_faces.push(frame_face.clone());
            new_faces.push(panel_face.clone());
            if depth.abs() > 1e-6 {
                for i in 0..inset3d.len() {
                    let a0 = inset3d[i];
                    let a1 = inset3d[(i + 1) % inset3d.len()];
                    let b0 = a0 + shift;
                    let b1 = a1 + shift;
                    if a0.distance(a1) < 1e-7 {
                        continue;
                    }
                    let wall = NurbsSurface::new(
                        1,
                        1,
                        vec![0.0, 0.0, 1.0, 1.0],
                        vec![0.0, 0.0, 1.0, 1.0],
                        2,
                        2,
                        vec![
                            ControlPoint::unweighted(a0),
                            ControlPoint::unweighted(b0),
                            ControlPoint::unweighted(a1),
                            ControlPoint::unweighted(b1),
                        ],
                    )?;
                    new_faces.push(BRepFace::untrimmed(wall));
                }
            }
        } else {
            new_faces.push(orig.clone());
        }
    }

    let mut out = BRep::from_faces(new_faces, tol.max(1e-3));
    if out.is_solid() {
        out.orient_outward_consistent();
    }
    Ok(out)
}

// --- Internal geometric helpers ---

fn split_brep_against(target: &BRep, cutter: &BRep, tol: f64) -> Vec<BRepFace> {
    let mut result = Vec::new();
    for fa in &target.faces {
        let mut fragments = vec![fa.clone()];
        let ba = fa.surface.bounding_box();
        for fb in &cutter.faces {
            let bb = fb.surface.bounding_box();
            let pad = tol.max(1e-4) * 2.0;
            if ba.max.x + pad < bb.min.x
                || bb.max.x + pad < ba.min.x
                || ba.max.y + pad < bb.min.y
                || bb.max.y + pad < ba.min.y
                || ba.max.z + pad < bb.min.z
                || bb.max.z + pad < ba.min.z
            {
                continue;
            }
            for sic in intersect_surfaces(&fa.surface, &fb.surface, tol) {
                let mut next = Vec::with_capacity(fragments.len());
                for frag in fragments {
                    next.extend(split_face_with_uv_curve(&frag, &sic.uv_a, sic.closed, tol));
                }
                fragments = next;
            }
        }
        result.extend(fragments);
    }
    result
}

fn build_curve_from_pts(pts: &[DVec3], tol: f64) -> Option<NurbsCurve> {
    if pts.len() < 2 {
        return None;
    }
    let closed = pts.len() >= 4 && pts[0].distance(*pts.last().unwrap()) <= tol * 4.0;
    if closed {
        let mut open = pts.to_vec();
        open.pop();
        if open.len() >= 4 {
            return NurbsCurve::interpolate(&open, 3, true)
                .ok()
                .or_else(|| NurbsCurve::polyline(pts).ok());
        }
    }
    if is_collinear(pts, tol.max(1e-4)) {
        return Some(NurbsCurve::line(pts[0], *pts.last().unwrap()));
    }
    if pts.len() >= 4 {
        if let Ok(c) = NurbsCurve::interpolate(pts, 3, false) {
            return Some(c);
        }
    }
    NurbsCurve::polyline(pts).ok()
}

fn is_collinear(pts: &[DVec3], tol: f64) -> bool {
    if pts.len() <= 2 {
        return true;
    }
    let a = pts[0];
    let b = *pts.last().unwrap();
    let ab = b - a;
    let len = ab.length();
    if len < 1e-9 {
        return pts.iter().all(|p| p.distance(a) <= tol);
    }
    let dir = ab / len;
    pts.iter().all(|&p| {
        let d = p - a;
        (d - dir * d.dot(dir)).length() <= tol
    })
}

fn tessellate_with_uv(srf: &NurbsSurface) -> UvTriMesh {
    let nu = if srf.degree_u <= 1 {
        (srf.count_u * 2).clamp(4, 10)
    } else {
        (srf.count_u * 4).clamp(24, 48)
    };
    let nv = if srf.degree_v <= 1 {
        (srf.count_v * 2).clamp(4, 10)
    } else {
        (srf.count_v * 4).clamp(24, 48)
    };
    let (u0, u1) = srf.domain_u();
    let (v0, v1) = srf.domain_v();

    let mut pts = Vec::with_capacity((nu + 1) * (nv + 1));
    let mut uvs = Vec::with_capacity((nu + 1) * (nv + 1));
    for iu in 0..=nu {
        let u = u0 + (u1 - u0) * (iu as f64 / nu as f64);
        for iv in 0..=nv {
            let v = v0 + (v1 - v0) * (iv as f64 / nv as f64);
            pts.push(srf.evaluate(u, v));
            uvs.push([u, v]);
        }
    }

    let stride = nv + 1;
    let mut tris = Vec::with_capacity(nu * nv * 2);
    for iu in 0..nu {
        for iv in 0..nv {
            let a = iu * stride + iv;
            let b = (iu + 1) * stride + iv;
            let c = (iu + 1) * stride + iv + 1;
            let d = iu * stride + iv + 1;
            tris.push([a, b, c]);
            tris.push([a, c, d]);
        }
    }
    UvTriMesh { pts, uvs, tris }
}

fn tri_tri_intersect(
    pa: [DVec3; 3],
    uva: [[f64; 2]; 3],
    pb: [DVec3; 3],
    uvb: [[f64; 2]; 3],
) -> Option<RawSeg> {
    let na = (pa[1] - pa[0]).cross(pa[2] - pa[0]);
    let nb = (pb[1] - pb[0]).cross(pb[2] - pb[0]);
    let la = na.length();
    let lb = nb.length();
    if la < 1e-12 || lb < 1e-12 {
        return None;
    }
    let na = na / la;
    let nb = nb / lb;
    let l_dir = na.cross(nb);
    if l_dir.length_squared() < 1e-10 {
        return None; // Coplanar or parallel triangles handled by perpendicular face intersections
    }
    let l_unit = l_dir.normalize();

    let (sa, uva_pair, pa_pair) = slice_tri_by_plane(pa, uva, pb[0], nb)?;
    let (sb, uvb_pair, _pb_pair) = slice_tri_by_plane(pb, uvb, pa[0], na)?;

    let ta0 = sa[0].dot(l_unit);
    let ta1 = sa[1].dot(l_unit);
    let tb0 = sb[0].dot(l_unit);
    let tb1 = sb[1].dot(l_unit);

    let (a_lo, a_hi, ua_lo, ua_hi, pa_lo, pa_hi) = if ta0 <= ta1 {
        (ta0, ta1, uva_pair[0], uva_pair[1], pa_pair[0], pa_pair[1])
    } else {
        (ta1, ta0, uva_pair[1], uva_pair[0], pa_pair[1], pa_pair[0])
    };
    let (b_lo, b_hi, ub_lo, ub_hi) = if tb0 <= tb1 {
        (tb0, tb1, uvb_pair[0], uvb_pair[1])
    } else {
        (tb1, tb0, uvb_pair[1], uvb_pair[0])
    };

    let t0 = a_lo.max(b_lo);
    let t1 = a_hi.min(b_hi);
    if t1 - t0 <= 1e-9 {
        return None;
    }

    let lerp_val = |lo: f64, hi: f64, t: f64| -> f64 {
        let d = hi - lo;
        if d.abs() < 1e-14 { 0.5 } else { ((t - lo) / d).clamp(0.0, 1.0) }
    };

    let fa0 = lerp_val(a_lo, a_hi, t0);
    let fa1 = lerp_val(a_lo, a_hi, t1);
    let fb0 = lerp_val(b_lo, b_hi, t0);
    let fb1 = lerp_val(b_lo, b_hi, t1);

    let mix_uv = |u0: [f64; 2], u1: [f64; 2], f: f64| -> [f64; 2] {
        [u0[0] + (u1[0] - u0[0]) * f, u0[1] + (u1[1] - u0[1]) * f]
    };

    Some(RawSeg {
        p0: pa_lo.lerp(pa_hi, fa0),
        p1: pa_lo.lerp(pa_hi, fa1),
        uva0: mix_uv(ua_lo, ua_hi, fa0),
        uva1: mix_uv(ua_lo, ua_hi, fa1),
        uvb0: mix_uv(ub_lo, ub_hi, fb0),
        uvb1: mix_uv(ub_lo, ub_hi, fb1),
    })
}

fn slice_tri_by_plane(
    pts: [DVec3; 3],
    uvs: [[f64; 2]; 3],
    plane_pt: DVec3,
    plane_n: DVec3,
) -> Option<([DVec3; 2], [[f64; 2]; 2], [DVec3; 2])> {
    let d = [
        (pts[0] - plane_pt).dot(plane_n),
        (pts[1] - plane_pt).dot(plane_n),
        (pts[2] - plane_pt).dot(plane_n),
    ];
    let eps = 1e-10;
    let mut hits: Vec<(DVec3, [f64; 2])> = Vec::with_capacity(4);
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        let di = d[i];
        let dj = d[j];
        if di.abs() <= eps && dj.abs() <= eps {
            hits.push((pts[i], uvs[i]));
            hits.push((pts[j], uvs[j]));
        } else if di.abs() <= eps {
            hits.push((pts[i], uvs[i]));
        } else if (di > 0.0 && dj < 0.0) || (di < 0.0 && dj > 0.0) {
            let t = di / (di - dj);
            hits.push((
                pts[i].lerp(pts[j], t),
                [
                    uvs[i][0] + (uvs[j][0] - uvs[i][0]) * t,
                    uvs[i][1] + (uvs[j][1] - uvs[i][1]) * t,
                ],
            ));
        }
    }
    // Deduplicate identical vertex touches
    let mut uniq: Vec<(DVec3, [f64; 2])> = Vec::with_capacity(hits.len());
    for h in hits {
        if uniq.iter().all(|u| u.0.distance(h.0) > 1e-10) {
            uniq.push(h);
        }
    }
    if uniq.len() < 2 {
        return None;
    }
    let a = uniq[0];
    let b = *uniq.last().unwrap();
    if a.0.distance(b.0) < 1e-10 {
        return None;
    }
    Some(([a.0, b.0], [a.1, b.1], [a.0, b.0]))
}

fn dedup_raw_segs(segs: &mut Vec<RawSeg>, tol: f64) {
    let mut kept: Vec<RawSeg> = Vec::with_capacity(segs.len());
    for &s in segs.iter() {
        let dup = kept.iter().any(|k| {
            let fwd = k.p0.distance(s.p0) <= tol
                && k.p1.distance(s.p1) <= tol
                && (k.uva0[0] - s.uva0[0]).hypot(k.uva0[1] - s.uva0[1]) < 0.05
                && (k.uvb0[0] - s.uvb0[0]).hypot(k.uvb0[1] - s.uvb0[1]) < 0.05;
            let rev = k.p0.distance(s.p1) <= tol
                && k.p1.distance(s.p0) <= tol
                && (k.uva0[0] - s.uva1[0]).hypot(k.uva0[1] - s.uva1[1]) < 0.05
                && (k.uvb0[0] - s.uvb1[0]).hypot(k.uvb0[1] - s.uvb1[1]) < 0.05;
            fwd || rev
        });
        if !dup {
            kept.push(s);
        }
    }
    *segs = kept;
}

type SsiPoint = (DVec3, [f64; 2], [f64; 2]);

fn chain_ssi_segments(
    segs: &mut Vec<RawSeg>,
    srf_a: &NurbsSurface,
    srf_b: &NurbsSurface,
    chain_tol: f64,
) -> Vec<Vec<SsiPoint>> {
    let span_a = {
        let (u0, u1) = srf_a.domain_u();
        let (v0, v1) = srf_a.domain_v();
        (u1 - u0).max(v1 - v0).max(1e-6)
    };
    let span_b = {
        let (u0, u1) = srf_b.domain_u();
        let (v0, v1) = srf_b.domain_v();
        (u1 - u0).max(v1 - v0).max(1e-6)
    };

    let uv_compatible = |u1: [f64; 2], u2: [f64; 2], span: f64| -> bool {
        (u1[0] - u2[0]).hypot(u1[1] - u2[1]) < span * 0.35
    };

    let mut chains = Vec::new();
    while !segs.is_empty() {
        let first = segs.remove(0);
        let mut chain = vec![
            (first.p0, first.uva0, first.uvb0),
            (first.p1, first.uva1, first.uvb1),
        ];

        loop {
            let head = *chain.first().unwrap();
            let tail = *chain.last().unwrap();
            let prev_tail = chain[chain.len() - 2];
            let next_head = chain[1];
            let mut best: Option<(usize, bool, bool, f64)> = None; // (idx, at_tail, flipped, dist)

            for (i, s) in segs.iter().enumerate() {
                for &(at_tail, end_pt, adj_pt) in &[(true, tail, prev_tail), (false, head, next_head)]
                {
                    let d0 = end_pt.0.distance(s.p0);
                    if d0 <= chain_tol
                        && s.p1.distance(adj_pt.0) > chain_tol * 0.25
                        && uv_compatible(end_pt.1, s.uva0, span_a)
                        && uv_compatible(end_pt.2, s.uvb0, span_b)
                        && best.is_none_or(|b| d0 < b.3)
                    {
                        best = Some((i, at_tail, false, d0));
                    }
                    let d1 = end_pt.0.distance(s.p1);
                    if d1 <= chain_tol
                        && s.p0.distance(adj_pt.0) > chain_tol * 0.25
                        && uv_compatible(end_pt.1, s.uva1, span_a)
                        && uv_compatible(end_pt.2, s.uvb1, span_b)
                        && best.is_none_or(|b| d1 < b.3)
                    {
                        best = Some((i, at_tail, true, d1));
                    }
                }
            }

            let Some((idx, at_tail, flipped, _)) = best else {
                break;
            };
            let next = segs.remove(idx);
            let (np, nua, nub) = if !flipped {
                (next.p1, next.uva1, next.uvb1)
            } else {
                (next.p0, next.uva0, next.uvb0)
            };
            if at_tail {
                chain.push((np, nua, nub));
            } else {
                chain.insert(0, (np, nua, nub));
            }
        }
        chains.push(chain);
    }

    chains
}

fn polish_ssi_point(
    srf_a: &NurbsSurface,
    srf_b: &NurbsSurface,
    mut uva: [f64; 2],
    mut uvb: [f64; 2],
) -> (DVec3, [f64; 2], [f64; 2]) {
    let (ua0, ua1) = srf_a.domain_u();
    let (va0, va1) = srf_a.domain_v();
    let (ub0, ub1) = srf_b.domain_u();
    let (vb0, vb1) = srf_b.domain_v();

    for _ in 0..8 {
        let da = srf_a.evaluate_derivatives(uva[0], uva[1]);
        let db = srf_b.evaluate_derivatives(uvb[0], uvb[1]);
        let r = da.point - db.point;
        let err2 = r.length_squared();
        if err2 < 1e-16 {
            return (0.5 * (da.point + db.point), uva, uvb);
        }

        let j = [da.du, da.dv, -db.du, -db.dv];
        let mut a = vec![vec![0.0_f64; 4]; 4];
        let mut rhs = vec![0.0_f64; 4];
        for row in 0..4 {
            rhs[row] = -j[row].dot(r);
            for col in 0..4 {
                a[row][col] = j[row].dot(j[col]);
            }
            a[row][row] += 1e-9;
        }
        let Some(step) = solve_linear_system(&a, &rhs) else {
            break;
        };
        let mut scale = 1.0;
        let mut accepted = false;
        for _ in 0..4 {
            let nua = [
                (uva[0] + step[0] * scale).clamp(ua0, ua1),
                (uva[1] + step[1] * scale).clamp(va0, va1),
            ];
            let nub = [
                (uvb[0] + step[2] * scale).clamp(ub0, ub1),
                (uvb[1] + step[3] * scale).clamp(vb0, vb1),
            ];
            let next_err2 = srf_a
                .evaluate(nua[0], nua[1])
                .distance_squared(srf_b.evaluate(nub[0], nub[1]));
            if next_err2 < err2 {
                uva = nua;
                uvb = nub;
                accepted = true;
                break;
            }
            scale *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    let pa = srf_a.evaluate(uva[0], uva[1]);
    (pa, uva, uvb)
}

#[derive(Debug, Clone, Copy)]
struct BoundaryCrossing {
    poly_seg: usize,
    poly_t: f64,
    edge_idx: usize,
    edge_t: f64,
    pos: [f64; 2],
}

fn dist_to_poly_uv(p: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let mut best = f64::INFINITY;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1e-20 {
            0.0
        } else {
            (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0)
        };
        let d = (p[0] - (a[0] + t * dx)).hypot(p[1] - (a[1] + t * dy));
        best = best.min(d);
    }
    best
}

fn snap_endpoint_to_poly(pt: &mut [f64; 2], poly: &[[f64; 2]], max_snap: f64) {
    let mut best = None;
    let mut best_d = max_snap;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let l2 = dx * dx + dy * dy;
        let t = if l2 < 1e-20 {
            0.0
        } else {
            (((pt[0] - a[0]) * dx + (pt[1] - a[1]) * dy) / l2).clamp(0.0, 1.0)
        };
        let proj = [a[0] + t * dx, a[1] + t * dy];
        let d = (pt[0] - proj[0]).hypot(pt[1] - proj[1]);
        if d < best_d {
            best_d = d;
            best = Some(proj);
        }
    }
    if let Some(proj) = best {
        *pt = proj;
    }
}

fn find_polyline_loop_crossings(
    poly: &[[f64; 2]],
    outer: &[[f64; 2]],
    tol: f64,
) -> Vec<BoundaryCrossing> {
    let mut hits = Vec::new();
    for si in 0..(poly.len() - 1) {
        let p0 = poly[si];
        let p1 = poly[si + 1];
        for ei in 0..outer.len() {
            let e0 = outer[ei];
            let e1 = outer[(ei + 1) % outer.len()];
            if let Some((tp, te, pos)) = seg_seg_intersect_uv(p0, p1, e0, e1, tol) {
                hits.push(BoundaryCrossing {
                    poly_seg: si,
                    poly_t: tp,
                    edge_idx: ei,
                    edge_t: te,
                    pos,
                });
            }
        }
    }
    hits.sort_by(|a, b| {
        a.poly_seg
            .cmp(&b.poly_seg)
            .then_with(|| a.poly_t.total_cmp(&b.poly_t))
    });
    let mut dedup: Vec<BoundaryCrossing> = Vec::with_capacity(hits.len());
    for h in hits {
        if dedup
            .last()
            .is_none_or(|prev| (prev.pos[0] - h.pos[0]).hypot(prev.pos[1] - h.pos[1]) > tol * 2.0)
        {
            dedup.push(h);
        }
    }
    dedup
}

fn seg_seg_intersect_uv(
    p0: [f64; 2],
    p1: [f64; 2],
    e0: [f64; 2],
    e1: [f64; 2],
    tol: f64,
) -> Option<(f64, f64, [f64; 2])> {
    let rx = p1[0] - p0[0];
    let ry = p1[1] - p0[1];
    let sx = e1[0] - e0[0];
    let sy = e1[1] - e0[1];
    let rxs = rx * sy - ry * sx;
    if rxs.abs() < 1e-13 {
        return None;
    }
    let qpx = e0[0] - p0[0];
    let qpy = e0[1] - p0[1];
    let t = (qpx * sy - qpy * sx) / rxs;
    let u = (qpx * ry - qpy * rx) / rxs;
    let r_len = rx.hypot(ry).max(1e-9);
    let s_len = sx.hypot(sy).max(1e-9);
    let pad_t = tol / r_len;
    let pad_u = tol / s_len;
    if t >= -pad_t && t <= 1.0 + pad_t && u >= -pad_u && u <= 1.0 + pad_u {
        let tc = t.clamp(0.0, 1.0);
        let uc = u.clamp(0.0, 1.0);
        let pos = [e0[0] + uc * sx, e0[1] + uc * sy];
        Some((tc, uc, pos))
    } else {
        None
    }
}

fn extract_sub_polyline(
    poly: &[[f64; 2]],
    c0: &BoundaryCrossing,
    c1: &BoundaryCrossing,
) -> Vec<[f64; 2]> {
    let mut out = vec![c0.pos];
    for idx in (c0.poly_seg + 1)..=c1.poly_seg {
        if idx < poly.len() {
            out.push(poly[idx]);
        }
    }
    out.push(c1.pos);
    clean_loop_2d(&out)
}

fn split_outer_polygon(
    outer: &[[f64; 2]],
    c_start: &BoundaryCrossing,
    c_end: &BoundaryCrossing,
    chord: &[[f64; 2]],
) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
    // Insert c_start and c_end cleanly along CCW outer polygon perimeter
    let n = outer.len();
    let mut marks = [
        (c_start.edge_idx, c_start.edge_t, c_start.pos, 0usize),
        (c_end.edge_idx, c_end.edge_t, c_end.pos, 1usize),
    ];
    marks.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));

    let mut perim: Vec<[f64; 2]> = Vec::with_capacity(n + 2);
    let mut idx_start = 0;
    let mut idx_end = 0;
    for i in 0..n {
        perim.push(outer[i]);
        for &(ei, _, pos, tag) in &marks {
            if ei == i {
                let inserted_at = perim.len();
                perim.push(pos);
                if tag == 0 {
                    idx_start = inserted_at;
                } else {
                    idx_end = inserted_at;
                }
            }
        }
    }

    let m = perim.len();
    // Loop A: chord (start -> end) + perimeter walk (end -> start)
    let mut poly_a = chord.to_vec();
    let mut cur = (idx_end + 1) % m;
    while cur != idx_start {
        poly_a.push(perim[cur]);
        cur = (cur + 1) % m;
    }

    // Loop B: chord reversed (end -> start) + perimeter walk (start -> end)
    let mut poly_b: Vec<[f64; 2]> = chord.iter().copied().rev().collect();
    let mut cur_b = (idx_start + 1) % m;
    while cur_b != idx_end {
        poly_b.push(perim[cur_b]);
        cur_b = (cur_b + 1) % m;
    }

    let mut clean_a = clean_loop_2d(&poly_a);
    let mut clean_b = clean_loop_2d(&poly_b);
    if signed_area_uv(&clean_a) < 0.0 {
        clean_a.reverse();
    }
    if signed_area_uv(&clean_b) < 0.0 {
        clean_b.reverse();
    }
    (clean_a, clean_b)
}

fn build_rolling_ball_fillet_strip(
    srf_a: &NurbsSurface,
    rev_a: bool,
    srf_b: &NurbsSurface,
    rev_b: bool,
    spine: &NurbsCurve,
    radius: f64,
    pick_pair: Option<(DVec3, DVec3)>,
) -> Result<NurbsSurface, GeomError> {
    let stations = (spine.control_points.len() * 2).clamp(13, 25);
    let (t0, t1) = spine.domain();

    // Determine consistent normal offset sign for both surfaces
    let p_mid = spine.evaluate(0.5 * (t0 + t1));
    let (ua_m, va_m, _, _) = srf_a.closest_point(p_mid);
    let (ub_m, vb_m, _, _) = srf_b.closest_point(p_mid);
    let (_, mut na_m) = srf_a.evaluate_point_and_normal(ua_m, va_m);
    let (_, mut nb_m) = srf_b.evaluate_point_and_normal(ub_m, vb_m);
    if rev_a {
        na_m = -na_m;
    }
    if rev_b {
        nb_m = -nb_m;
    }

    let (sign_a, sign_b) = if let Some((pa_pick, pb_pick)) = pick_pair {
        let sa = if (pa_pick - p_mid).dot(nb_m) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let sb = if (pb_pick - p_mid).dot(na_m) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        (sa, sb)
    } else {
        let (_, _, cent_a, _) = BRepFace::untrimmed(srf_a.clone())
            .interior_sample_point()
            .unwrap_or((0.5, 0.5, p_mid, na_m));
        let (_, _, cent_b, _) = BRepFace::untrimmed(srf_b.clone())
            .interior_sample_point()
            .unwrap_or((0.5, 0.5, p_mid, nb_m));
        let sa = if (cent_b - p_mid).dot(na_m) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let sb = if (cent_a - p_mid).dot(nb_m) >= 0.0 {
            1.0
        } else {
            -1.0
        };
        (sa, sb)
    };

    let mut arc_rows = Vec::with_capacity(stations + 1);
    for i in 0..=stations {
        let u = t0 + (t1 - t0) * (i as f64 / stations as f64);
        let p_sp = spine.evaluate(u);
        let (mut qa, mut qb, center) =
            solve_rolling_ball_station(srf_a, rev_a, srf_b, rev_b, p_sp, radius, sign_a, sign_b);
        let mut va = qa - center;
        let mut vb = qb - center;
        let la = va.length().max(1e-6);
        let lb = vb.length().max(1e-6);
        va /= la;
        vb /= lb;
        qa = center + va * radius;
        qb = center + vb * radius;

        let ang = va.angle_between(vb).clamp(1e-4, std::f64::consts::PI - 1e-4);
        let half = ang * 0.5;
        let w_mid = half.cos().max(0.05);
        let bis = (va + vb).normalize_or_zero();
        let p_mid_arc = if bis.length_squared() > 1e-10 {
            center + bis * (radius / half.cos())
        } else {
            0.5 * (qa + qb)
        };
        let arc_row = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                ControlPoint::new(qa, 1.0),
                ControlPoint::new(p_mid_arc, w_mid),
                ControlPoint::new(qb, 1.0),
            ],
        )?;
        arc_rows.push(arc_row);
    }

    // Loft rows then transpose so U runs along spine and V runs across arc cross-section
    let raw = loft_nurbs(&arc_rows, LoftStyle::Normal, false)?;
    Ok(raw.transpose())
}

fn build_rolling_ball_or_chamfer_strip(
    face_a: &BRepFace,
    face_b: &BRepFace,
    edge_curve: &NurbsCurve,
    size: f64,
    is_fillet: bool,
) -> Result<NurbsSurface, GeomError> {
    let (_, _, cent_a, _) = face_a
        .interior_sample_point()
        .ok_or(GeomError::Degenerate("degenerate face A for edge blend"))?;
    let (_, _, cent_b, _) = face_b
        .interior_sample_point()
        .ok_or(GeomError::Degenerate("degenerate face B for edge blend"))?;

    if is_fillet {
        return build_rolling_ball_fillet_strip(
            &face_a.surface,
            face_a.orientation_reversed,
            &face_b.surface,
            face_b.orientation_reversed,
            edge_curve,
            size,
            Some((cent_a, cent_b)),
        );
    }

    // Setback chamfer surface
    let stations = (edge_curve.control_points.len() * 2).clamp(9, 25);
    let (t0, t1) = edge_curve.domain();
    let mut rows = Vec::with_capacity(stations + 1);
    for i in 0..=stations {
        let u = t0 + (t1 - t0) * (i as f64 / stations as f64);
        let p = edge_curve.evaluate(u);
        let (_, t_e) = edge_curve.evaluate_derivative(u);
        let t_hat = t_e.normalize_or(DVec3::X);

        let (ua, va, _, _) = face_a.surface.closest_point(p);
        let (_, na) = face_a.evaluate_point_and_normal(ua, va);
        let mut da = na.cross(t_hat).normalize_or_zero();
        if da.dot(cent_a - p) < 0.0 {
            da = -da;
        }
        let (_, _, qa, _) = face_a.surface.closest_point(p + da * size);

        let (ub, vb, _, _) = face_b.surface.closest_point(p);
        let (_, nb) = face_b.evaluate_point_and_normal(ub, vb);
        let mut db = nb.cross(t_hat).normalize_or_zero();
        if db.dot(cent_b - p) < 0.0 {
            db = -db;
        }
        let (_, _, qb, _) = face_b.surface.closest_point(p + db * size);

        rows.push(NurbsCurve::line(qa, qb));
    }
    Ok(loft_nurbs(&rows, LoftStyle::Normal, false)?.transpose())
}

#[allow(clippy::too_many_arguments)]
fn solve_rolling_ball_station(
    srf_a: &NurbsSurface,
    rev_a: bool,
    srf_b: &NurbsSurface,
    rev_b: bool,
    seed_pt: DVec3,
    radius: f64,
    sign_a: f64,
    sign_b: f64,
) -> (DVec3, DVec3, DVec3) {
    let mut center = seed_pt;
    let (mut ua, mut va, mut qa, _) = srf_a.closest_point(center);
    let (mut ub, mut vb, mut qb, _) = srf_b.closest_point(center);

    for _ in 0..12 {
        let (_, mut na) = srf_a.evaluate_point_and_normal(ua, va);
        let (_, mut nb) = srf_b.evaluate_point_and_normal(ub, vb);
        if rev_a {
            na = -na;
        }
        if rev_b {
            nb = -nb;
        }
        na *= sign_a;
        nb *= sign_b;

        let tangent = na.cross(nb);
        if tangent.length_squared() < 1e-10 {
            center = qa + na * radius;
            break;
        }
        let t_dir = tangent.normalize();
        // Solve intersection of planes: na*(x - qa) = radius, nb*(x - qb) = radius, t_dir*(x - seed_pt) = 0
        if let Some(next_center) = intersect_three_planes(
            qa + na * radius,
            na,
            qb + nb * radius,
            nb,
            seed_pt,
            t_dir,
        ) {
            let moved = center.distance(next_center);
            center = next_center;
            (ua, va, qa, _) = srf_a.closest_point(center);
            (ub, vb, qb, _) = srf_b.closest_point(center);
            if moved < 1e-7 {
                break;
            }
        } else {
            break;
        }
    }
    (qa, qb, center)
}

fn notch_cap_face_at_corner(
    face: &mut BRepFace,
    corner_3d: DVec3,
    profile_3d: &[DVec3],
    tol: f64,
) {
    let match_tol = (tol * 20.0).max(0.02);
    let outer = face.effective_outer_loop();
    let corner_idx = outer
        .iter()
        .enumerate()
        .find(|&(_, &[u, v])| face.surface.evaluate(u, v).distance(corner_3d) <= match_tol)
        .map(|(i, _)| i);
    let Some(ci) = corner_idx else {
        return;
    };

    // Only notch planar cap faces where both profile endpoints lie on this face's plane
    let [u_mid, v_mid] = outer[ci];
    let (p_ref, n_ref) = face.surface.evaluate_point_and_normal(u_mid, v_mid);
    let max_plane_dist = profile_3d
        .iter()
        .map(|&p| (p - p_ref).dot(n_ref).abs())
        .fold(0.0_f64, f64::max);
    if max_plane_dist > match_tol {
        return;
    }

    let n = outer.len();
    let prev_3d = {
        let uv = outer[(ci + n - 1) % n];
        face.surface.evaluate(uv[0], uv[1])
    };
    let next_3d = {
        let uv = outer[(ci + 1) % n];
        face.surface.evaluate(uv[0], uv[1])
    };

    let p0 = profile_3d[0];
    let pn = *profile_3d.last().unwrap();
    let d_fwd = p0.distance(prev_3d) + pn.distance(next_3d);
    let d_rev = pn.distance(prev_3d) + p0.distance(next_3d);
    let ordered_prof: Vec<DVec3> = if d_fwd <= d_rev {
        profile_3d.to_vec()
    } else {
        profile_3d.iter().copied().rev().collect()
    };

    let mut replacement_uv = Vec::with_capacity(ordered_prof.len());
    for p in ordered_prof {
        let (u, v, _, _) = face.surface.closest_point(p);
        replacement_uv.push([u, v]);
    }

    let mut next_outer = Vec::with_capacity(n + replacement_uv.len());
    next_outer.extend_from_slice(&outer[..ci]);
    next_outer.extend_from_slice(&replacement_uv);
    next_outer.extend_from_slice(&outer[ci + 1..]);
    let cleaned = clean_loop_2d(&next_outer);
    if cleaned.len() >= 3 {
        face.outer_loop = Some(TrimLoop2D {
            points: cleaned,
            is_hole: false,
        });
    }
}

fn is_point_on_loop_3d(p: DVec3, loop_pts: &[DVec3], tol: f64) -> bool {
    for i in 0..loop_pts.len() {
        let a = loop_pts[i];
        let b = loop_pts[(i + 1) % loop_pts.len()];
        let ab = b - a;
        let l2 = ab.length_squared();
        let t = if l2 < 1e-18 {
            0.0
        } else {
            ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
        };
        if p.distance(a + ab * t) <= tol {
            return true;
        }
    }
    false
}

fn miter_offset_polygon_2d(pts: &[[f64; 2]], offset: f64) -> Vec<[f64; 2]> {
    let n = pts.len();
    if n < 3 {
        return Vec::new();
    }
    // Offset CCW polygon edges: outward normal is (+dy, -dx), so inward for negative offset shifts along (-dy, +dx)
    let mut lines: Vec<([f64; 2], [f64; 2])> = Vec::with_capacity(n);
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let len = dx.hypot(dy).max(1e-9);
        let dir = [dx / len, dy / len];
        let out_n = [dir[1], -dir[0]];
        let pt = [a[0] + out_n[0] * offset, a[1] + out_n[1] * offset];
        lines.push((pt, dir));
    }

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let (p0, d0) = lines[(i + n - 1) % n];
        let (p1, d1) = lines[i];
        let det = d0[0] * d1[1] - d0[1] * d1[0];
        if det.abs() < 1e-8 {
            out.push(p1);
        } else {
            let rx = p1[0] - p0[0];
            let ry = p1[1] - p0[1];
            let t = (rx * d1[1] - ry * d1[0]) / det;
            out.push([p0[0] + d0[0] * t, p0[1] + d0[1] * t]);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn surface_surface_intersection_plane_sphere_and_cylinder() {
        let sphere = NurbsSurface::sphere(DVec3::ZERO, 5.0).unwrap();
        let plane_z3 = Plane {
            origin: DVec3::new(0.0, 0.0, 3.0),
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        };
        let rect = NurbsSurface::plane_rect(&plane_z3, 12.0, 12.0);
        let hits = intersect_surfaces(&rect, &sphere, 1e-3);
        assert!(!hits.is_empty());
        let expected_circ = std::f64::consts::TAU * 4.0;
        let len = hits[0].curve_3d.arc_length();
        assert!(
            (len - expected_circ).abs() < 0.2,
            "expected sphere-plane slice circle length ~{expected_circ}, got {len}"
        );

        let cyl = NurbsSurface::cylinder(DVec3::ZERO, 2.0, 6.0).unwrap();
        let hits_cyl = intersect_surfaces(&cyl, &rect, 1e-3);
        assert!(!hits_cyl.is_empty());
        assert!((hits_cyl[0].curve_3d.arc_length() - std::f64::consts::TAU * 2.0).abs() < 0.2);
    }

    #[test]
    fn brep_boolean_box_difference_cylinder_and_box_box_ops() {
        // Box [-3, 3] x [-3, 3] x [0, 4] minus Cylinder (r=1.5, z=-1..6)
        let box_b = BRep::from_box(DVec3::new(0.0, 0.0, 2.0), DVec3::new(6.0, 6.0, 4.0));
        let cyl_b = BRep::from_cylinder(DVec3::new(0.0, 0.0, -1.0), 1.5, 7.0).unwrap();

        let drilled = brep_boolean(&box_b, &cyl_b, BooleanOp::Difference, 1e-3).unwrap();
        assert!(
            drilled.is_solid(),
            "drilled plate naked edges: {:?}",
            drilled.naked_edges()
        );
        let expected_vol = 6.0 * 6.0 * 4.0 - std::f64::consts::PI * 1.5 * 1.5 * 4.0;
        let actual_vol = drilled.volume_approx();
        assert!(
            (actual_vol - expected_vol).abs() < 1.5,
            "expected drilled vol ~{expected_vol:.2}, got {actual_vol:.2}"
        );
        assert!(!drilled.contains_point(DVec3::new(0.0, 0.0, 2.0)));
        assert!(drilled.contains_point(DVec3::new(2.2, 2.2, 2.0)));

        // Overlapping Box Union & Intersection
        let b1 = BRep::from_box(DVec3::new(0.0, 0.0, 1.0), DVec3::splat(2.0));
        let b2 = BRep::from_box(DVec3::new(1.0, 0.0, 1.0), DVec3::splat(2.0));
        let union = brep_boolean(&b1, &b2, BooleanOp::Union, 1e-3).unwrap();
        assert!(union.is_solid(), "union naked: {:?}", union.naked_edges());
        assert!((union.volume_approx() - 12.0).abs() < 1e-2);

        let inter = brep_boolean(&b1, &b2, BooleanOp::Intersection, 1e-3).unwrap();
        assert!(inter.is_solid());
        assert!((inter.volume_approx() - 4.0).abs() < 1e-2);
    }

    #[test]
    fn cap_untrim_fillet_srf_fillet_edge_chamfer_shell_push_pull_and_inset() {
        // Open NURBS cylinder capped via cap_planar_holes -> watertight solid
        let open_cyl = BRep::from_surface(NurbsSurface::cylinder(DVec3::ZERO, 2.0, 4.0).unwrap());
        assert!(!open_cyl.is_solid());
        let capped = cap_planar_holes(&open_cyl, 1e-3).unwrap();
        assert!(capped.is_solid());

        // Untrim restores full area on trimmed face
        let untrimmed = untrim_brep(&capped, None);
        assert!(untrimmed.faces.iter().all(|f| !f.is_trimmed()));

        // Rolling-ball FilletSrf between 2 perpendicular planes
        let p_horiz = NurbsSurface::plane_rect(&Plane::world_xy(), 6.0, 6.0);
        let p_vert_plane = Plane {
            origin: DVec3::new(0.0, 0.0, 0.0),
            x_axis: DVec3::X,
            y_axis: DVec3::Z,
            z_axis: -DVec3::Y,
        };
        let p_vert = NurbsSurface::plane_rect(&p_vert_plane, 6.0, 6.0);
        let fs = fillet_srf(
            &p_horiz,
            &p_vert,
            1.0,
            Some(DVec3::new(0.0, 2.0, 0.0)),
            Some(DVec3::new(0.0, 0.0, 2.0)),
            true,
            1e-3,
        )
        .unwrap();
        assert!(fs.trimmed_a.is_some() && fs.trimmed_b.is_some());
        let (_, n_seam) = fs.fillet.evaluate_point_and_normal(0.5, 0.0);
        let (_, n_h) = p_horiz.evaluate_point_and_normal(0.5, 0.5);
        assert!(n_seam.angle_between(n_h).min(n_seam.angle_between(-n_h)) < 1e-2);

        // FilletEdge and ChamferEdge on a solid Box
        let unit_box = BRep::from_box(DVec3::new(0.0, 0.0, 1.0), DVec3::splat(2.0));
        let filleted_box = fillet_brep_edges(&unit_box, &[0], 0.4, 1e-3).unwrap();
        assert!(
            filleted_box.is_solid(),
            "filleted box naked: {:?}",
            filleted_box.naked_edges()
        );
        assert!(filleted_box.volume_approx() < 8.0);
        assert!(filleted_box.faces.len() == 7);

        let chamfered_box = chamfer_brep_edges(&unit_box, &[0], 0.4, 1e-3).unwrap();
        assert!(chamfered_box.is_solid());
        assert!((chamfered_box.volume_approx() - (8.0 - 0.5 * 0.4 * 0.4 * 2.0)).abs() < 0.02);

        // Shell Box removing top face (+Z face index 0)
        let shelled = shell_brep(&unit_box, &[0], 0.2, 1e-3).unwrap();
        assert!(
            shelled.is_solid(),
            "shelled box naked: {:?}",
            shelled.naked_edges()
        );
        let expected_shell_vol = 8.0 - 1.6 * 1.6 * 1.8;
        assert!((shelled.volume_approx() - expected_shell_vol).abs() < 0.05);

        // PushPull top face +Z (index 0) by +2.0 -> height grows 2 -> 4, vol 8 -> 16
        let pushed = push_pull_face(&unit_box, 0, 2.0, 1e-3).unwrap();
        assert!(pushed.is_solid(), "pushed naked: {:?}", pushed.naked_edges());
        assert!((pushed.volume_approx() - 16.0).abs() < 0.05);

        // Inset top face by 0.4, depth -0.5 -> pocket subtracted (8 - 1.2*1.2*0.5 = 7.28)
        let inset = inset_face(&unit_box, 0, 0.4, -0.5, 1e-3).unwrap();
        assert!(inset.is_solid(), "inset naked: {:?}", inset.naked_edges());
        assert!((inset.volume_approx() - 7.28).abs() < 0.05);
    }
}
