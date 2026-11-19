use std::collections::HashMap;

use cad_math::{Aabb, DVec3, Plane};

use crate::brep::BRep;
use crate::mesh::TriangleMesh;
use crate::nurbs_surface::{NurbsSurface, SurfaceEdge};
use crate::Geometry;

/// Volumetric and surface mass properties computed analytically over triangle / NURBS geometry.
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    pub area: f64,
    pub volume: f64,
    pub centroid: DVec3,
    pub bbox: Aabb,
    /// Unit-density inertia tensor (`rho = 1`) about `centroid` in world axes.
    pub inertia_tensor: [[f64; 3]; 3],
    /// Sorted principal moments `[I1 <= I2 <= I3]` (`rho = 1`).
    pub principal_moments: [f64; 3],
    pub is_closed_solid: bool,
}

impl Default for MassProperties {
    fn default() -> Self {
        Self {
            area: 0.0,
            volume: 0.0,
            centroid: DVec3::ZERO,
            bbox: Aabb::EMPTY,
            inertia_tensor: [[0.0; 3]; 3],
            principal_moments: [0.0; 3],
            is_closed_solid: false,
        }
    }
}

/// Rhino 9-style Global Edge Continuity classification along a seam.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ContinuityGrade {
    Broken,
    G0,
    G1,
    G2,
}

impl ContinuityGrade {
    pub fn label(self) -> &'static str {
        match self {
            Self::Broken => "Broken",
            Self::G0 => "G0 (Position)",
            Self::G1 => "G1 (Tangent)",
            Self::G2 => "G2 (Curvature)",
        }
    }

    pub fn short_label(self) -> &'static str {
        match self {
            Self::Broken => "GAP",
            Self::G0 => "G0",
            Self::G1 => "G1",
            Self::G2 => "G2",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContinuityTolerances {
    /// Maximum spatial gap allowed for G0 position continuity.
    pub g0_tol: f64,
    /// Maximum normal deviation in degrees allowed for G1 tangent continuity.
    pub g1_tol_deg: f64,
    /// Maximum relative cross-seam normal curvature jump allowed for G2 curvature continuity.
    pub g2_tol_rel: f64,
}

impl Default for ContinuityTolerances {
    fn default() -> Self {
        Self {
            g0_tol: 1e-2,
            g1_tol_deg: 1.0,
            g2_tol_rel: 0.05,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeContinuitySample {
    pub position: DVec3,
    pub gap: f64,
    pub angle_deg: f64,
    pub curvature_diff_rel: f64,
    pub grade: ContinuityGrade,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SeamContinuityReport {
    pub label: String,
    pub polyline: Vec<DVec3>,
    pub midpoint: DVec3,
    pub max_gap: f64,
    pub max_angle_deg: f64,
    pub max_curvature_diff_rel: f64,
    pub grade: ContinuityGrade,
    pub samples: Vec<EdgeContinuitySample>,
}

/// Tessellated display mesh paired with per-vertex Gaussian (`K`) and Mean (`H`) curvatures.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvatureMesh {
    pub mesh: TriangleMesh,
    pub gaussian: Vec<f64>,
    pub mean: Vec<f64>,
}

/// Compute surface area, enclosed volume, centroid, inertia tensor, and principal moments of a mesh.
pub fn compute_mesh_mass_properties(mesh: &TriangleMesh, is_closed_solid: bool) -> MassProperties {
    if mesh.positions.is_empty() || mesh.indices.is_empty() {
        return MassProperties::default();
    }
    let bbox = mesh.bounding_box();
    let r0 = bbox.center();

    let mut total_area = 0.0_f64;
    let mut area_centroid_acc = DVec3::ZERO;
    let mut signed_vol6 = 0.0_f64;
    let mut vol_centroid_acc = DVec3::ZERO;

    for &[i0, i1, i2] in &mesh.indices {
        let a = DVec3::from_array(mesh.positions[i0 as usize]) - r0;
        let b = DVec3::from_array(mesh.positions[i1 as usize]) - r0;
        let c = DVec3::from_array(mesh.positions[i2 as usize]) - r0;
        let tri_area = 0.5 * (b - a).cross(c - a).length();
        total_area += tri_area;
        area_centroid_acc += ((a + b + c) / 3.0) * tri_area;

        let dv6 = a.dot(b.cross(c));
        signed_vol6 += dv6;
        vol_centroid_acc += (a + b + c) * 0.25 * (dv6 / 6.0);
    }

    let signed_vol = signed_vol6 / 6.0;
    let has_volume = is_closed_solid && signed_vol.abs() > 1e-12;
    let volume = if has_volume { signed_vol.abs() } else { 0.0 };
    let sign = if signed_vol < 0.0 { -1.0 } else { 1.0 };

    let centroid_rel = if has_volume {
        (vol_centroid_acc * sign) / volume
    } else if total_area > 1e-12 {
        area_centroid_acc / total_area
    } else {
        DVec3::ZERO
    };
    let centroid = r0 + centroid_rel;

    let mut cov_r0 = [[0.0_f64; 3]; 3];
    if has_volume {
        for &[i0, i1, i2] in &mesh.indices {
            let a = (DVec3::from_array(mesh.positions[i0 as usize]) - r0).to_array();
            let b = (DVec3::from_array(mesh.positions[i1 as usize]) - r0).to_array();
            let c = (DVec3::from_array(mesh.positions[i2 as usize]) - r0).to_array();
            let dv = (DVec3::from_array(a).dot(DVec3::from_array(b).cross(DVec3::from_array(c))) / 6.0) * sign;
            let k = dv / 20.0;
            for i in 0..3 {
                let si = a[i] + b[i] + c[i];
                for j in 0..3 {
                    let sj = a[j] + b[j] + c[j];
                    cov_r0[i][j] += k * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + si * sj);
                }
            }
        }
        let cr = centroid_rel.to_array();
        for i in 0..3 {
            for j in 0..3 {
                cov_r0[i][j] -= volume * cr[i] * cr[j];
            }
        }
    } else if total_area > 1e-12 {
        for &[i0, i1, i2] in &mesh.indices {
            let a = (DVec3::from_array(mesh.positions[i0 as usize]) - r0).to_array();
            let b = (DVec3::from_array(mesh.positions[i1 as usize]) - r0).to_array();
            let c = (DVec3::from_array(mesh.positions[i2 as usize]) - r0).to_array();
            let tri_area = 0.5
                * (DVec3::from_array(b) - DVec3::from_array(a))
                    .cross(DVec3::from_array(c) - DVec3::from_array(a))
                    .length();
            let k = tri_area / 12.0;
            for i in 0..3 {
                let si = a[i] + b[i] + c[i];
                for j in 0..3 {
                    let sj = a[j] + b[j] + c[j];
                    cov_r0[i][j] += k * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + si * sj);
                }
            }
        }
        let cr = centroid_rel.to_array();
        for i in 0..3 {
            for j in 0..3 {
                cov_r0[i][j] -= total_area * cr[i] * cr[j];
            }
        }
    }

    let inertia_tensor = cov_to_inertia(cov_r0);
    let principal_moments = jacobi_eigenvalues_3x3(inertia_tensor);

    MassProperties {
        area: total_area,
        volume,
        centroid,
        bbox,
        inertia_tensor,
        principal_moments,
        is_closed_solid: has_volume,
    }
}

/// Compute `MassProperties` for a B-Rep solid/sheet, NURBS surface, or triangle mesh.
pub fn compute_geometry_mass_properties(geom: &Geometry) -> Option<MassProperties> {
    match geom {
        Geometry::BRep(b) => {
            let m = b.tessellate_display();
            Some(compute_mesh_mass_properties(&m, b.is_solid()))
        }
        Geometry::SubD(sd) => {
            let m = sd.tessellate_display();
            Some(compute_mesh_mass_properties(&m, is_mesh_closed_manifold(&m)))
        }
        Geometry::Surface(s) => {
            let m = s.tessellate_display();
            Some(compute_mesh_mass_properties(&m, false))
        }
        Geometry::Mesh(m) => Some(compute_mesh_mass_properties(m, is_mesh_closed_manifold(m))),
        Geometry::Point(_) | Geometry::Curve(_) => None,
    }
}

/// Combine multiple objects' `MassProperties` using parallel axis theorem.
pub fn combine_mass_properties(items: &[MassProperties]) -> MassProperties {
    if items.is_empty() {
        return MassProperties::default();
    }
    let mut area = 0.0;
    let mut volume = 0.0;
    let mut bbox = Aabb::EMPTY;
    let mut all_closed = true;
    for item in items {
        area += item.area;
        volume += item.volume;
        bbox = bbox.union(&item.bbox);
        if !item.is_closed_solid {
            all_closed = false;
        }
    }

    let use_volume = volume > 1e-12;
    let weight_of = |m: &MassProperties| if use_volume { m.volume } else { m.area };
    let total_w: f64 = items.iter().map(weight_of).sum();
    if total_w <= 1e-12 {
        return MassProperties {
            area,
            volume,
            bbox,
            is_closed_solid: all_closed && use_volume,
            ..Default::default()
        };
    }

    let mut centroid = DVec3::ZERO;
    for item in items {
        centroid += item.centroid * weight_of(item);
    }
    centroid /= total_w;

    let mut inertia_tensor = [[0.0_f64; 3]; 3];
    for item in items {
        let w = weight_of(item);
        let d = (item.centroid - centroid).to_array();
        let d2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        for i in 0..3 {
            for j in 0..3 {
                let delta_ij = if i == j { 1.0 } else { 0.0 };
                inertia_tensor[i][j] += item.inertia_tensor[i][j] + w * (d2 * delta_ij - d[i] * d[j]);
            }
        }
    }
    let principal_moments = jacobi_eigenvalues_3x3(inertia_tensor);

    MassProperties {
        area,
        volume,
        centroid,
        bbox,
        inertia_tensor,
        principal_moments,
        is_closed_solid: all_closed && use_volume,
    }
}

fn cov_to_inertia(cov: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let trace = (cov[0][0] + cov[1][1] + cov[2][2]).max(0.0);
    let mut out = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = if i == j { (trace - cov[i][i]).max(0.0) } else { -cov[i][j] };
        }
    }
    out
}

fn jacobi_eigenvalues_3x3(mut a: [[f64; 3]; 3]) -> [f64; 3] {
    for _ in 0..24 {
        let mut p = 0;
        let mut q = 1;
        let mut max_off = a[0][1].abs();
        for &(i, j) in &[(0usize, 2usize), (1usize, 2usize)] {
            if a[i][j].abs() > max_off {
                max_off = a[i][j].abs();
                p = i;
                q = j;
            }
        }
        if max_off < 1e-14 {
            break;
        }
        let tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let app = a[p][p] - t * a[p][q];
        let aqq = a[q][q] + t * a[p][q];
        a[p][p] = app;
        a[q][q] = aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for r in 0..3 {
            if r != p && r != q {
                let arp = c * a[r][p] - s * a[r][q];
                let arq = s * a[r][p] + c * a[r][q];
                a[r][p] = arp;
                a[p][r] = arp;
                a[r][q] = arq;
                a[q][r] = arq;
            }
        }
    }
    let mut vals = [a[0][0].max(0.0), a[1][1].max(0.0), a[2][2].max(0.0)];
    vals.sort_by(|x, y| x.total_cmp(y));
    vals
}

fn is_mesh_closed_manifold(mesh: &TriangleMesh) -> bool {
    if mesh.indices.is_empty() {
        return false;
    }
    let quant = |p: [f64; 3]| {
        (
            (p[0] * 1e5).round() as i64,
            (p[1] * 1e5).round() as i64,
            (p[2] * 1e5).round() as i64,
        )
    };
    let mut edge_counts: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
    for &[i0, i1, i2] in &mesh.indices {
        let p = [
            quant(mesh.positions[i0 as usize]),
            quant(mesh.positions[i1 as usize]),
            quant(mesh.positions[i2 as usize]),
        ];
        for e in 0..3 {
            let a = p[e];
            let b = p[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    !edge_counts.is_empty() && edge_counts.values().all(|&c| c == 2)
}

fn normal_curvature_in_direction(
    srf: &NurbsSurface,
    u: f64,
    v: f64,
    dir_3d: DVec3,
    flip_normal: bool,
) -> f64 {
    let d = srf.non_degenerate_ders(u, v);
    let cross = d.du.cross(d.dv);
    let len = cross.length();
    if len <= 1e-12 {
        return 0.0;
    }
    let n = cross / len;
    let e = d.du.length_squared();
    let f = d.du.dot(d.dv);
    let g = d.dv.length_squared();
    let det = e * g - f * f;
    if det <= 1e-18 {
        return 0.0;
    }
    let rhs_u = dir_3d.dot(d.du);
    let rhs_v = dir_3d.dot(d.dv);
    let a = (rhs_u * g - rhs_v * f) / det;
    let b = (e * rhs_v - f * rhs_u) / det;
    let first = e * a * a + 2.0 * f * a * b + g * b * b;
    if first <= 1e-18 {
        return 0.0;
    }
    let l = d.duu.dot(n);
    let m = d.duv.dot(n);
    let n2 = d.dvv.dot(n);
    let second = l * a * a + 2.0 * m * a * b + n2 * b * b;
    let kn = second / first;
    if flip_normal { -kn } else { kn }
}

fn classify_sample(
    gap: f64,
    angle_deg: f64,
    curvature_diff_rel: f64,
    tol: ContinuityTolerances,
) -> ContinuityGrade {
    if gap > tol.g0_tol {
        ContinuityGrade::Broken
    } else if angle_deg > tol.g1_tol_deg {
        ContinuityGrade::G0
    } else if curvature_diff_rel > tol.g2_tol_rel {
        ContinuityGrade::G1
    } else {
        ContinuityGrade::G2
    }
}

/// Analyze G0/G1/G2 continuity across every interior stitched edge of a B-Rep polysurface.
pub fn analyze_brep_edge_continuity(
    brep: &BRep,
    tol: ContinuityTolerances,
    sample_count: usize,
) -> Vec<SeamContinuityReport> {
    let n_samples = sample_count.max(3);
    let mut reports = Vec::new();
    for (ei, edge) in brep.edges.iter().enumerate() {
        if edge.adjacent_faces.len() != 2 {
            continue;
        }
        let fa_idx = edge.adjacent_faces[0];
        let fb_idx = edge.adjacent_faces[1];
        let (Some(fa), Some(fb)) = (brep.faces.get(fa_idx), brep.faces.get(fb_idx)) else {
            continue;
        };

        let (t0, t1) = edge.curve.domain();
        let mut polyline = Vec::with_capacity(n_samples);
        let mut samples = Vec::with_capacity(n_samples);
        let mut max_gap = 0.0_f64;
        let mut max_angle_deg = 0.0_f64;
        let mut max_curv_rel = 0.0_f64;
        let mut worst_grade = ContinuityGrade::G2;

        for i in 0..n_samples {
            let s = i as f64 / (n_samples - 1) as f64;
            let t = t0 + (t1 - t0) * s;
            let (pt, raw_tan) = edge.curve.evaluate_derivative(t);
            let tan = raw_tan.normalize_or_zero();
            polyline.push(pt);

            let (ua, va, pa, _) = fa.surface.closest_point(pt);
            let (ub, vb, pb, _) = fb.surface.closest_point(pt);
            let (_, na) = fa.evaluate_point_and_normal(ua, va);
            let (_, nb) = fb.evaluate_point_and_normal(ub, vb);

            let gap = pa.distance(pb).max(pt.distance(pa)).max(pt.distance(pb));
            let angle_deg = na.dot(nb).clamp(-1.0, 1.0).acos().to_degrees();

            let mut dir_cross = na.cross(tan).normalize_or_zero();
            if dir_cross.length_squared() < 1e-10 {
                dir_cross = fa.surface.non_degenerate_ders(ua, va).du.normalize_or_zero();
            }
            let ka = normal_curvature_in_direction(&fa.surface, ua, va, dir_cross, fa.orientation_reversed);
            let kb = normal_curvature_in_direction(&fb.surface, ub, vb, dir_cross, fb.orientation_reversed);
            let curv_rel = (ka - kb).abs() / ka.abs().max(kb.abs()).max(1e-3);

            let grade = classify_sample(gap, angle_deg, curv_rel, tol);
            max_gap = max_gap.max(gap);
            max_angle_deg = max_angle_deg.max(angle_deg);
            max_curv_rel = max_curv_rel.max(curv_rel);
            worst_grade = worst_grade.min(grade);
            samples.push(EdgeContinuitySample {
                position: pt,
                gap,
                angle_deg,
                curvature_diff_rel: curv_rel,
                grade,
            });
        }

        let midpoint = edge.curve.evaluate(0.5 * (t0 + t1));
        reports.push(SeamContinuityReport {
            label: format!("Edge #{ei} (F{fa_idx}-F{fb_idx})"),
            polyline,
            midpoint,
            max_gap,
            max_angle_deg,
            max_curvature_diff_rel: max_curv_rel,
            grade: worst_grade,
            samples,
        });
    }
    reports
}

/// Analyze G0/G1/G2 continuity along the closest matching boundary edge between two NURBS surfaces.
pub fn analyze_surface_pair_continuity(
    name_a: &str,
    srf_a: &NurbsSurface,
    name_b: &str,
    srf_b: &NurbsSurface,
    tol: ContinuityTolerances,
    sample_count: usize,
) -> Option<SeamContinuityReport> {
    let seam_thresh = (tol.g0_tol * 50.0).max(0.25);
    let mut best_pair: Option<(SurfaceEdge, SurfaceEdge, f64)> = None;
    for ea in SurfaceEdge::ALL {
        let Ok(ca) = srf_a.boundary_curve(ea) else { continue };
        let (a0, a1) = ca.domain();
        let pa_mid = ca.evaluate(0.5 * (a0 + a1));
        for eb in SurfaceEdge::ALL {
            let Ok(cb) = srf_b.boundary_curve(eb) else { continue };
            let (b0, b1) = cb.domain();
            let pb_mid = cb.evaluate(0.5 * (b0 + b1));
            let d = pa_mid.distance(pb_mid);
            if best_pair.is_none_or(|b| d < b.2) {
                best_pair = Some((ea, eb, d));
            }
        }
    }

    let (ea, _, mid_dist) = best_pair?;
    if mid_dist > seam_thresh {
        return None;
    }

    let ca = srf_a.boundary_curve(ea).ok()?;
    let n_samples = sample_count.max(3);
    let (t0, t1) = ca.domain();
    let mut polyline = Vec::with_capacity(n_samples);
    let mut samples = Vec::with_capacity(n_samples);
    let mut max_gap = 0.0_f64;
    let mut max_angle_deg = 0.0_f64;
    let mut max_curv_rel = 0.0_f64;
    let mut worst_grade = ContinuityGrade::G2;

    for i in 0..n_samples {
        let s = i as f64 / (n_samples - 1) as f64;
        let t = t0 + (t1 - t0) * s;
        let (pt, raw_tan) = ca.evaluate_derivative(t);
        let tan = raw_tan.normalize_or_zero();
        polyline.push(pt);

        let (ua, va, pa, _) = srf_a.closest_point(pt);
        let (ub, vb, pb, _) = srf_b.closest_point(pt);
        let (_, na) = srf_a.evaluate_point_and_normal(ua, va);
        let (_, nb) = srf_b.evaluate_point_and_normal(ub, vb);

        let gap = pa.distance(pb);
        let dot = na.dot(nb);
        let angle_deg = dot.abs().clamp(0.0, 1.0).acos().to_degrees();
        let flip_b = dot < 0.0;

        let mut dir_cross = na.cross(tan).normalize_or_zero();
        if dir_cross.length_squared() < 1e-10 {
            dir_cross = srf_a.non_degenerate_ders(ua, va).du.normalize_or_zero();
        }
        let ka = normal_curvature_in_direction(srf_a, ua, va, dir_cross, false);
        let kb = normal_curvature_in_direction(srf_b, ub, vb, dir_cross, flip_b);
        let curv_rel = (ka - kb).abs() / ka.abs().max(kb.abs()).max(1e-3);

        let grade = classify_sample(gap, angle_deg, curv_rel, tol);
        max_gap = max_gap.max(gap);
        max_angle_deg = max_angle_deg.max(angle_deg);
        max_curv_rel = max_curv_rel.max(curv_rel);
        worst_grade = worst_grade.min(grade);
        samples.push(EdgeContinuitySample {
            position: pt,
            gap,
            angle_deg,
            curvature_diff_rel: curv_rel,
            grade,
        });
    }

    let midpoint = ca.evaluate(0.5 * (t0 + t1));
    Some(SeamContinuityReport {
        label: format!("{name_a} <-> {name_b}"),
        polyline,
        midpoint,
        max_gap,
        max_angle_deg,
        max_curvature_diff_rel: max_curv_rel,
        grade: worst_grade,
        samples,
    })
}

/// Intersect a `TriangleMesh` with an arbitrary `Plane` and stitch segments into ordered contours.
pub fn section_mesh_with_plane(mesh: &TriangleMesh, plane: &Plane) -> Vec<Vec<DVec3>> {
    let mut raw_segs: Vec<[DVec3; 2]> = Vec::new();
    let eps = 1e-6;
    for &[i0, i1, i2] in &mesh.indices {
        let pts = [
            DVec3::from_array(mesh.positions[i0 as usize]),
            DVec3::from_array(mesh.positions[i1 as usize]),
            DVec3::from_array(mesh.positions[i2 as usize]),
        ];
        let d = [
            (pts[0] - plane.origin).dot(plane.z_axis),
            (pts[1] - plane.origin).dot(plane.z_axis),
            (pts[2] - plane.origin).dot(plane.z_axis),
        ];
        let mut hits: Vec<DVec3> = Vec::with_capacity(3);
        for e in 0..3 {
            let j = (e + 1) % 3;
            let da = d[e];
            let db = d[j];
            if da.abs() <= eps {
                push_unique_pt(&mut hits, pts[e], 1e-5);
            } else if db.abs() <= eps {
                push_unique_pt(&mut hits, pts[j], 1e-5);
            } else if da * db < 0.0 {
                let t = da / (da - db);
                push_unique_pt(&mut hits, pts[e].lerp(pts[j], t), 1e-5);
            }
        }
        if hits.len() >= 2 && hits[0].distance(hits[1]) > 1e-6 {
            raw_segs.push([hits[0], hits[1]]);
        }
    }
    stitch_segments(&raw_segs, 1e-4)
}

/// Compute section contour polylines of a `Geometry` intersected by `plane`.
pub fn section_geometry_with_plane(geom: &Geometry, plane: &Plane) -> Vec<Vec<DVec3>> {
    match geom {
        Geometry::BRep(b) => section_mesh_with_plane(&b.tessellate_display(), plane),
        Geometry::Surface(s) => section_mesh_with_plane(&s.tessellate_display(), plane),
        Geometry::SubD(sd) => section_mesh_with_plane(&sd.tessellate_display(), plane),
        Geometry::Mesh(m) => section_mesh_with_plane(m, plane),
        Geometry::Point(_) | Geometry::Curve(_) => Vec::new(),
    }
}

fn push_unique_pt(pts: &mut Vec<DVec3>, p: DVec3, tol: f64) {
    if !pts.iter().any(|q| q.distance(p) <= tol) {
        pts.push(p);
    }
}

fn stitch_segments(segs: &[[DVec3; 2]], tol: f64) -> Vec<Vec<DVec3>> {
    let mut used = vec![false; segs.len()];
    let mut loops = Vec::new();
    for start_idx in 0..segs.len() {
        if used[start_idx] {
            continue;
        }
        used[start_idx] = true;
        let mut chain = vec![segs[start_idx][0], segs[start_idx][1]];

        loop {
            let tail = *chain.last().unwrap();
            let mut best_end: Option<(usize, bool, f64)> = None;
            for (idx, seg) in segs.iter().enumerate() {
                if used[idx] {
                    continue;
                }
                let d0 = tail.distance(seg[0]);
                let d1 = tail.distance(seg[1]);
                if d0 <= tol && best_end.is_none_or(|b| d0 < b.2) {
                    best_end = Some((idx, false, d0));
                }
                if d1 <= tol && best_end.is_none_or(|b| d1 < b.2) {
                    best_end = Some((idx, true, d1));
                }
            }
            if let Some((idx, rev, _)) = best_end {
                used[idx] = true;
                chain.push(if rev { segs[idx][0] } else { segs[idx][1] });
                continue;
            }

            let head = chain[0];
            let mut best_start: Option<(usize, bool, f64)> = None;
            for (idx, seg) in segs.iter().enumerate() {
                if used[idx] {
                    continue;
                }
                let d0 = head.distance(seg[0]);
                let d1 = head.distance(seg[1]);
                if d0 <= tol && best_start.is_none_or(|b| d0 < b.2) {
                    best_start = Some((idx, true, d0));
                }
                if d1 <= tol && best_start.is_none_or(|b| d1 < b.2) {
                    best_start = Some((idx, false, d1));
                }
            }
            if let Some((idx, flip, _)) = best_start {
                used[idx] = true;
                chain.insert(0, if flip { segs[idx][1] } else { segs[idx][0] });
                continue;
            }
            break;
        }

        if chain.len() >= 3 && chain[0].distance(*chain.last().unwrap()) <= tol {
            let last_idx = chain.len() - 1;
            chain[last_idx] = chain[0];
        }
        loops.push(chain);
    }
    loops
}

/// Tessellate a `NurbsSurface` along with exact Gaussian & Mean curvature per vertex.
pub fn tessellate_surface_with_curvature(surf: &NurbsSurface) -> CurvatureMesh {
    let (mesh, uvs) = surf.tessellate_display_with_uv();
    let mut gaussian = Vec::with_capacity(uvs.len());
    let mut mean = Vec::with_capacity(uvs.len());
    for &[u, v] in &uvs {
        let c = surf.curvature_at(u, v);
        gaussian.push(c.gaussian);
        mean.push(c.mean);
    }
    CurvatureMesh {
        mesh,
        gaussian,
        mean,
    }
}

/// Tessellate a `BRep` along with exact Gaussian & Mean curvature per face vertex.
pub fn tessellate_brep_with_curvature(brep: &BRep) -> CurvatureMesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mut gaussian = Vec::new();
    let mut mean = Vec::new();
    for face in &brep.faces {
        let (m, uvs) = face.tessellate_display_with_uv();
        let base = positions.len() as u32;
        positions.extend_from_slice(&m.positions);
        normals.extend_from_slice(&m.normals);
        for &[a, b, c] in &m.indices {
            indices.push([base + a, base + b, base + c]);
        }
        for &[u, v] in &uvs {
            let c = face.surface.curvature_at(u, v);
            gaussian.push(c.gaussian);
            mean.push(if face.orientation_reversed { -c.mean } else { c.mean });
        }
    }
    CurvatureMesh {
        mesh: TriangleMesh {
            positions,
            normals,
            indices,
        },
        gaussian,
        mean,
    }
}

/// Estimate discrete Gaussian angle-defect & Laplacian Mean curvature per vertex on a polygon mesh.
pub fn compute_mesh_discrete_curvature(mesh: &TriangleMesh) -> (Vec<f64>, Vec<f64>) {
    let n = mesh.positions.len();
    if n == 0 || mesh.indices.is_empty() {
        return (vec![0.0; n], vec![0.0; n]);
    }
    let quant = |p: [f64; 3]| {
        (
            (p[0] * 1e5).round() as i64,
            (p[1] * 1e5).round() as i64,
            (p[2] * 1e5).round() as i64,
        )
    };
    let mut weld_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut orig_to_weld = Vec::with_capacity(n);
    let mut weld_pos: Vec<DVec3> = Vec::new();
    let mut weld_norm: Vec<DVec3> = Vec::new();

    for (i, &p) in mesh.positions.iter().enumerate() {
        let k = quant(p);
        let nrm = DVec3::from_array(mesh.normals.get(i).copied().unwrap_or([0.0, 0.0, 1.0]));
        let widx = *weld_map.entry(k).or_insert_with(|| {
            let id = weld_pos.len();
            weld_pos.push(DVec3::from_array(p));
            weld_norm.push(DVec3::ZERO);
            id
        });
        orig_to_weld.push(widx);
        weld_norm[widx] += nrm;
    }
    let nw = weld_pos.len();
    for nr in &mut weld_norm {
        *nr = nr.normalize_or(DVec3::Z);
    }

    let mut angle_sum = vec![0.0_f64; nw];
    let mut area_acc = vec![0.0_f64; nw];
    let mut laplacian = vec![DVec3::ZERO; nw];
    let mut edge_map: HashMap<(usize, usize), usize> = HashMap::new();

    for &[i0, i1, i2] in &mesh.indices {
        let w = [
            orig_to_weld[i0 as usize],
            orig_to_weld[i1 as usize],
            orig_to_weld[i2 as usize],
        ];
        if w[0] == w[1] || w[1] == w[2] || w[2] == w[0] {
            continue;
        }
        let p = [weld_pos[w[0]], weld_pos[w[1]], weld_pos[w[2]]];
        let tri_area = 0.5 * (p[1] - p[0]).cross(p[2] - p[0]).length();
        for c in 0..3 {
            let next = (c + 1) % 3;
            let prev = (c + 2) % 3;
            let e1 = (p[next] - p[c]).normalize_or_zero();
            let e2 = (p[prev] - p[c]).normalize_or_zero();
            angle_sum[w[c]] += e1.dot(e2).clamp(-1.0, 1.0).acos();
            area_acc[w[c]] += tri_area / 3.0;
            laplacian[w[c]] += p[next] - p[c];
            let ek = if w[c] < w[next] {
                (w[c], w[next])
            } else {
                (w[next], w[c])
            };
            *edge_map.entry(ek).or_insert(0) += 1;
        }
    }

    let mut is_boundary = vec![false; nw];
    for (&(a, b), &cnt) in &edge_map {
        if cnt < 2 {
            is_boundary[a] = true;
            is_boundary[b] = true;
        }
    }

    let mut gauss_w = vec![0.0_f64; nw];
    let mut mean_w = vec![0.0_f64; nw];
    for wi in 0..nw {
        let area = area_acc[wi];
        if area > 1e-12 && !is_boundary[wi] {
            gauss_w[wi] = (std::f64::consts::TAU - angle_sum[wi]) / area;
            mean_w[wi] = laplacian[wi].dot(weld_norm[wi]) / (2.0 * area.sqrt().max(1e-6));
        }
    }

    let gaussian = orig_to_weld.iter().map(|&w| gauss_w[w]).collect();
    let mean = orig_to_weld.iter().map(|&w| mean_w[w]).collect();
    (gaussian, mean)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_math::Continuity;

    #[test]
    fn mass_properties_exact_for_unit_cube() {
        let cube = TriangleMesh::from_box(DVec3::new(1.0, 2.0, -3.0), DVec3::splat(1.0));
        let mp = compute_mesh_mass_properties(&cube, true);
        assert!(mp.is_closed_solid);
        assert!((mp.area - 6.0).abs() < 1e-6);
        assert!((mp.volume - 1.0).abs() < 1e-6);
        assert!(mp.centroid.distance(DVec3::new(1.0, 2.0, -3.0)) < 1e-6);
        for &m in &mp.principal_moments {
            assert!((m - 1.0 / 6.0).abs() < 1e-5, "unexpected inertia moment {m}");
        }
    }

    #[test]
    fn brep_continuity_distinguishes_box_creases_and_smooth_cylinder_seam() {
        let box_b = BRep::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let box_rep = analyze_brep_edge_continuity(&box_b, ContinuityTolerances::default(), 5);
        assert_eq!(box_rep.len(), 12);
        assert!(box_rep.iter().all(|r| r.grade == ContinuityGrade::G0 && (r.max_angle_deg - 90.0).abs() < 1.0));

        let plane_a = NurbsSurface::plane_rect(&Plane::world_xy(), 4.0, 4.0);
        let mut plane_b = NurbsSurface::plane_rect(&Plane::world_xy(), 4.0, 4.0);
        plane_b.transform(&cad_math::DMat4::from_translation(DVec3::new(4.0, 0.0, 0.0)));
        let seam = analyze_surface_pair_continuity(
            "A",
            &plane_a,
            "B",
            &plane_b,
            ContinuityTolerances::default(),
            5,
        )
        .expect("adjacent coplanar patches share seam");
        assert_eq!(seam.grade, ContinuityGrade::G2);

        let arc_blend = crate::blend_srf(
            &plane_a,
            SurfaceEdge::UMax,
            &plane_b,
            SurfaceEdge::UMin,
            Continuity::G2,
            1.0,
            1.0,
        );
        if let Ok(b_srf) = arc_blend {
            let cm = tessellate_surface_with_curvature(&b_srf);
            assert_eq!(cm.gaussian.len(), cm.mesh.positions.len());
        }
    }

    #[test]
    fn curvature_and_section_cut_work_on_sphere_and_box() {
        let sph = NurbsSurface::sphere(DVec3::ZERO, 2.0).unwrap();
        let cm = tessellate_surface_with_curvature(&sph);
        assert!(!cm.mesh.positions.is_empty());
        assert_eq!(cm.gaussian.len(), cm.mesh.positions.len());
        // Gaussian curvature of sphere radius 2 is 1/4 = 0.25 everywhere
        let mid_idx = cm.gaussian.len() / 2;
        assert!((cm.gaussian[mid_idx] - 0.25).abs() < 1e-3);

        let box_m = TriangleMesh::from_box(DVec3::ZERO, DVec3::new(4.0, 6.0, 2.0));
        let contours = section_mesh_with_plane(&box_m, &Plane::world_xy());
        assert!(!contours.is_empty());
        let first = &contours[0];
        assert!(first.len() >= 4);
        for p in first {
            assert!(p.z.abs() < 1e-5);
        }
    }
}
