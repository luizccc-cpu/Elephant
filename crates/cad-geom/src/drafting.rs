use std::collections::HashMap;

use cad_math::{Aabb, DMat4, DVec3, DVec4};
use serde::{Deserialize, Serialize};

use crate::Geometry;
use crate::mesh::TriangleMesh;

/// View direction / camera specification for 2D technical drafting projections (`Make2D` & Layouts).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DrawingProjection {
    Top,
    Front,
    Right,
    Back,
    Left,
    Bottom,
    Isometric,
    Perspective {
        eye: DVec3,
        target: DVec3,
        up: DVec3,
        fov_y_rad: f64,
        aspect: f64,
    },
}

impl DrawingProjection {
    pub const ORTHO_ALL: [Self; 7] = [
        Self::Top,
        Self::Front,
        Self::Right,
        Self::Back,
        Self::Left,
        Self::Bottom,
        Self::Isometric,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Top => "Top",
            Self::Front => "Front",
            Self::Right => "Right",
            Self::Back => "Back",
            Self::Left => "Left",
            Self::Bottom => "Bottom",
            Self::Isometric => "Isometric",
            Self::Perspective { .. } => "Perspective",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Make2DLineKind {
    Visible,
    Hidden,
    Silhouette,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Make2DSegment {
    pub points_2d: Vec<[f64; 2]>,
    pub kind: Make2DLineKind,
    pub color: Option<[u8; 4]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Make2DOptions {
    pub include_hidden: bool,
    pub include_silhouettes: bool,
    pub include_creases: bool,
    pub samples_per_curve: usize,
    pub crease_angle_deg: f64,
}

impl Default for Make2DOptions {
    fn default() -> Self {
        Self {
            include_hidden: true,
            include_silhouettes: true,
            include_creases: true,
            samples_per_curve: 24,
            crease_angle_deg: 28.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Make2DResult {
    pub segments: Vec<Make2DSegment>,
    pub bounds_min: [f64; 2],
    pub bounds_max: [f64; 2],
}

enum Projector {
    Ortho {
        right: DVec3,
        up: DVec3,
        forward: DVec3,
        origin: DVec3,
    },
    Persp {
        view_proj: DMat4,
        forward: DVec3,
        scale_xy: f64,
    },
}

impl Projector {
    fn from_projection(proj: DrawingProjection, scene_bbox: &Aabb) -> Self {
        let origin = if scene_bbox.is_empty() {
            DVec3::ZERO
        } else {
            scene_bbox.center()
        };
        let diag = if scene_bbox.is_empty() {
            10.0
        } else {
            scene_bbox.size().length().max(1.0)
        };
        match proj {
            DrawingProjection::Top => Self::Ortho {
                right: DVec3::X,
                up: DVec3::Y,
                forward: DVec3::NEG_Z,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Bottom => Self::Ortho {
                right: DVec3::X,
                up: DVec3::NEG_Y,
                forward: DVec3::Z,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Front => Self::Ortho {
                right: DVec3::X,
                up: DVec3::Z,
                forward: DVec3::Y,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Back => Self::Ortho {
                right: DVec3::NEG_X,
                up: DVec3::Z,
                forward: DVec3::NEG_Y,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Right => Self::Ortho {
                right: DVec3::Y,
                up: DVec3::Z,
                forward: DVec3::NEG_X,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Left => Self::Ortho {
                right: DVec3::NEG_Y,
                up: DVec3::Z,
                forward: DVec3::X,
                origin: DVec3::ZERO,
            },
            DrawingProjection::Isometric => {
                let forward = DVec3::new(-1.0, 1.0, -1.0).normalize();
                let right = DVec3::new(1.0, 1.0, 0.0).normalize();
                let up = right.cross(forward).normalize();
                Self::Ortho {
                    right,
                    up: if up.z < 0.0 { -up } else { up },
                    forward,
                    origin: DVec3::ZERO,
                }
            }
            DrawingProjection::Perspective {
                eye,
                target,
                up,
                fov_y_rad,
                aspect,
            } => {
                let forward = (target - eye).normalize_or(DVec3::NEG_Z);
                let dist = eye.distance(target).max(diag * 0.5);
                let near = (dist * 0.01).max(0.05);
                let far = (dist + diag * 4.0).max(near + 100.0);
                let view = DMat4::look_at_rh(eye, target, up);
                let pm = DMat4::perspective_rh(fov_y_rad.clamp(0.1, 2.8), aspect.max(0.1), near, far);
                let scale_xy = dist * (fov_y_rad * 0.5).tan() * aspect.max(0.1);
                let _ = origin;
                Self::Persp {
                    view_proj: pm * view,
                    forward,
                    scale_xy: scale_xy.max(1.0),
                }
            }
        }
    }

    fn forward_dir(&self) -> DVec3 {
        match *self {
            Self::Ortho { forward, .. } | Self::Persp { forward, .. } => forward,
        }
    }

    fn is_persp(&self) -> bool {
        matches!(self, Self::Persp { .. })
    }

    fn project(&self, p: DVec3) -> Option<[f64; 3]> {
        match *self {
            Self::Ortho {
                right,
                up,
                forward,
                origin,
            } => {
                let r = p - origin;
                Some([r.dot(right), r.dot(up), r.dot(forward)])
            }
            Self::Persp {
                view_proj,
                scale_xy,
                ..
            } => {
                let c = view_proj * DVec4::new(p.x, p.y, p.z, 1.0);
                if c.w <= 1e-5 {
                    return None;
                }
                let ndc_x = c.x / c.w;
                let ndc_y = c.y / c.w;
                let ndc_z = c.z / c.w;
                Some([ndc_x * scale_xy, ndc_y * scale_xy, ndc_z])
            }
        }
    }
}

struct OccluderTri {
    v: [[f64; 3]; 3],
    min_x: f64,
    max_x: f64,
    min_y: f64,
    max_y: f64,
    min_z: f64,
    area2: f64,
}

impl OccluderTri {
    fn from_projected(v0: [f64; 3], v1: [f64; 3], v2: [f64; 3]) -> Option<Self> {
        let area2 = (v1[0] - v0[0]) * (v2[1] - v0[1]) - (v1[1] - v0[1]) * (v2[0] - v0[0]);
        if area2.abs() <= 1e-12 {
            return None;
        }
        Some(Self {
            v: [v0, v1, v2],
            min_x: v0[0].min(v1[0]).min(v2[0]),
            max_x: v0[0].max(v1[0]).max(v2[0]),
            min_y: v0[1].min(v1[1]).min(v2[1]),
            max_y: v0[1].max(v1[1]).max(v2[1]),
            min_z: v0[2].min(v1[2]).min(v2[2]),
            area2,
        })
    }

    fn depth_at(&self, x: f64, y: f64) -> Option<f64> {
        if x < self.min_x - 1e-9 || x > self.max_x + 1e-9 || y < self.min_y - 1e-9 || y > self.max_y + 1e-9 {
            return None;
        }
        let [a, b, c] = self.v;
        let inv = 1.0 / self.area2;
        let u = ((b[0] - x) * (c[1] - y) - (b[1] - y) * (c[0] - x)) * inv;
        let v = ((c[0] - x) * (a[1] - y) - (c[1] - y) * (a[0] - x)) * inv;
        let w = 1.0 - u - v;
        if u >= -1e-6 && v >= -1e-6 && w >= -1e-6 {
            Some(u * a[2] + v * b[2] + w * c[2])
        } else {
            None
        }
    }
}

struct CandidateEdgePoly {
    points_3d: Vec<DVec3>,
    is_silhouette: bool,
    color: Option<[u8; 4]>,
}

/// Project 3D CAD geometry into analytic 2D vector linework with exact hidden-line removal.
pub fn make_2d_projection(
    items: &[(Geometry, Option<[u8; 4]>)],
    projection: DrawingProjection,
    opts: &Make2DOptions,
) -> Make2DResult {
    let mut scene_bbox = Aabb::EMPTY;
    for (g, _) in items {
        scene_bbox = scene_bbox.union(&g.bounding_box());
    }
    if scene_bbox.is_empty() {
        return Make2DResult {
            segments: Vec::new(),
            bounds_min: [0.0, 0.0],
            bounds_max: [0.0, 0.0],
        };
    }

    let proj = Projector::from_projection(projection, &scene_bbox);
    let view_dir = proj.forward_dir();
    let diag = scene_bbox.size().length().max(1.0);
    let crease_cos = opts.crease_angle_deg.to_radians().cos();
    let curve_samples = opts.samples_per_curve.max(8);

    let mut occluders: Vec<OccluderTri> = Vec::new();
    let mut candidates: Vec<CandidateEdgePoly> = Vec::new();

    for (geom, col) in items {
        match geom {
            Geometry::Point(_) => {}
            Geometry::Curve(c) => {
                let pts = if c.degree == 1 {
                    c.control_points.iter().map(|cp| cp.position).collect()
                } else {
                    c.sample_polyline(curve_samples)
                };
                if pts.len() >= 2 {
                    candidates.push(CandidateEdgePoly {
                        points_3d: pts,
                        is_silhouette: false,
                        color: *col,
                    });
                }
            }
            Geometry::Surface(s) => {
                let mesh = s.tessellate_display();
                push_mesh_occluders(&mesh, &proj, &mut occluders);
                if let Ok(b_curves) = s.boundary_curves() {
                    for c in b_curves {
                        let pts = if c.degree == 1 {
                            c.control_points.iter().map(|cp| cp.position).collect()
                        } else {
                            c.sample_polyline(curve_samples)
                        };
                        if pts.len() >= 2 {
                            candidates.push(CandidateEdgePoly {
                                points_3d: pts,
                                is_silhouette: false,
                                color: *col,
                            });
                        }
                    }
                }
                if opts.include_silhouettes || opts.include_creases {
                    extract_mesh_feature_edges(
                        &mesh,
                        view_dir,
                        crease_cos,
                        opts.include_silhouettes,
                        opts.include_creases,
                        false,
                        *col,
                        &mut candidates,
                    );
                }
            }
            Geometry::BRep(b) => {
                let mesh = b.tessellate_display();
                push_mesh_occluders(&mesh, &proj, &mut occluders);
                for edge in &b.edges {
                    let pts = if edge.curve.degree == 1 && edge.curve.control_points.len() == 2 {
                        vec![
                            edge.curve.control_points[0].position,
                            edge.curve.control_points[1].position,
                        ]
                    } else {
                        edge.curve.sample_polyline(curve_samples)
                    };
                    if pts.len() >= 2 {
                        candidates.push(CandidateEdgePoly {
                            points_3d: pts,
                            is_silhouette: false,
                            color: *col,
                        });
                    }
                }
                if opts.include_silhouettes || opts.include_creases {
                    extract_mesh_feature_edges(
                        &mesh,
                        view_dir,
                        crease_cos,
                        opts.include_silhouettes,
                        false,
                        false,
                        *col,
                        &mut candidates,
                    );
                }
            }
            Geometry::Mesh(m) => {
                push_mesh_occluders(m, &proj, &mut occluders);
                extract_mesh_feature_edges(
                    m,
                    view_dir,
                    crease_cos,
                    opts.include_silhouettes,
                    opts.include_creases,
                    true,
                    *col,
                    &mut candidates,
                );
            }
        }
    }

    let z_bias = if proj.is_persp() {
        1e-4
    } else {
        (diag * 1.5e-3).max(1e-4)
    };
    let sub_steps = (curve_samples / 3).clamp(4, 16);

    let mut segments: Vec<Make2DSegment> = Vec::new();
    let mut bmin = [f64::INFINITY, f64::INFINITY];
    let mut bmax = [f64::NEG_INFINITY, f64::NEG_INFINITY];

    for cand in &candidates {
        let mut cur_kind: Option<Make2DLineKind> = None;
        let mut cur_pts: Vec<[f64; 2]> = Vec::new();

        for win in cand.points_3d.windows(2) {
            let p0 = win[0];
            let p1 = win[1];
            let Some(q0) = proj.project(p0) else { continue };
            let Some(q1) = proj.project(p1) else { continue };
            let seg_len2 = (q1[0] - q0[0]).powi(2) + (q1[1] - q0[1]).powi(2);
            if seg_len2 < 1e-12 {
                continue;
            }
            let n_sub = if cand.points_3d.len() <= 2 { sub_steps } else { 3 };
            for s in 0..n_sub {
                let t0 = s as f64 / n_sub as f64;
                let t1 = (s + 1) as f64 / n_sub as f64;
                let a = [
                    q0[0] + (q1[0] - q0[0]) * t0,
                    q0[1] + (q1[1] - q0[1]) * t0,
                    q0[2] + (q1[2] - q0[2]) * t0,
                ];
                let b = [
                    q0[0] + (q1[0] - q0[0]) * t1,
                    q0[1] + (q1[1] - q0[1]) * t1,
                    q0[2] + (q1[2] - q0[2]) * t1,
                ];
                let mx = 0.5 * (a[0] + b[0]);
                let my = 0.5 * (a[1] + b[1]);
                let mz = 0.5 * (a[2] + b[2]);

                let mut occluded = false;
                let limit_z = mz - z_bias;
                for occ in &occluders {
                    if occ.min_z >= limit_z {
                        continue;
                    }
                    if let Some(oz) = occ.depth_at(mx, my) {
                        if oz < limit_z {
                            occluded = true;
                            break;
                        }
                    }
                }

                let kind = if occluded {
                    Make2DLineKind::Hidden
                } else if cand.is_silhouette {
                    Make2DLineKind::Silhouette
                } else {
                    Make2DLineKind::Visible
                };

                if kind == Make2DLineKind::Hidden && !opts.include_hidden {
                    if cur_pts.len() >= 2 {
                        segments.push(Make2DSegment {
                            points_2d: std::mem::take(&mut cur_pts),
                            kind: cur_kind.unwrap_or(Make2DLineKind::Visible),
                            color: cand.color,
                        });
                    }
                    cur_pts.clear();
                    cur_kind = None;
                    continue;
                }

                let p2a = [a[0], a[1]];
                let p2b = [b[0], b[1]];
                expand_2d(&mut bmin, &mut bmax, p2a);
                expand_2d(&mut bmin, &mut bmax, p2b);

                if cur_kind == Some(kind) && cur_pts.last().is_some_and(|l| dist2(l, &p2a) < 1e-10) {
                    cur_pts.push(p2b);
                } else {
                    if cur_pts.len() >= 2 {
                        segments.push(Make2DSegment {
                            points_2d: std::mem::take(&mut cur_pts),
                            kind: cur_kind.unwrap_or(kind),
                            color: cand.color,
                        });
                    }
                    cur_kind = Some(kind);
                    cur_pts = vec![p2a, p2b];
                }
            }
        }
        if cur_pts.len() >= 2 {
            segments.push(Make2DSegment {
                points_2d: cur_pts,
                kind: cur_kind.unwrap_or(Make2DLineKind::Visible),
                color: cand.color,
            });
        }
    }

    if !bmin[0].is_finite() {
        bmin = [0.0, 0.0];
        bmax = [0.0, 0.0];
    }

    Make2DResult {
        segments,
        bounds_min: bmin,
        bounds_max: bmax,
    }
}

fn expand_2d(bmin: &mut [f64; 2], bmax: &mut [f64; 2], p: [f64; 2]) {
    bmin[0] = bmin[0].min(p[0]);
    bmin[1] = bmin[1].min(p[1]);
    bmax[0] = bmax[0].max(p[0]);
    bmax[1] = bmax[1].max(p[1]);
}

fn dist2(a: &[f64; 2], b: &[f64; 2]) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

fn push_mesh_occluders(mesh: &TriangleMesh, proj: &Projector, out: &mut Vec<OccluderTri>) {
    for &[i0, i1, i2] in &mesh.indices {
        let Some(p0) = proj.project(DVec3::from_array(mesh.positions[i0 as usize])) else { continue };
        let Some(p1) = proj.project(DVec3::from_array(mesh.positions[i1 as usize])) else { continue };
        let Some(p2) = proj.project(DVec3::from_array(mesh.positions[i2 as usize])) else { continue };
        if let Some(tri) = OccluderTri::from_projected(p0, p1, p2) {
            out.push(tri);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn extract_mesh_feature_edges(
    mesh: &TriangleMesh,
    view_dir: DVec3,
    crease_cos: f64,
    include_silhouettes: bool,
    include_creases: bool,
    include_boundaries: bool,
    color: Option<[u8; 4]>,
    out: &mut Vec<CandidateEdgePoly>,
) {
    let quant = |p: [f64; 3]| {
        (
            (p[0] * 1e5).round() as i64,
            (p[1] * 1e5).round() as i64,
            (p[2] * 1e5).round() as i64,
        )
    };
    let mut weld_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut weld_pos: Vec<DVec3> = Vec::new();
    let mut orig_to_weld = Vec::with_capacity(mesh.positions.len());
    for &p in &mesh.positions {
        let idx = *weld_map.entry(quant(p)).or_insert_with(|| {
            let id = weld_pos.len();
            weld_pos.push(DVec3::from_array(p));
            id
        });
        orig_to_weld.push(idx);
    }

    let mut edge_faces: HashMap<(usize, usize), Vec<DVec3>> = HashMap::new();
    for &[i0, i1, i2] in &mesh.indices {
        let w = [
            orig_to_weld[i0 as usize],
            orig_to_weld[i1 as usize],
            orig_to_weld[i2 as usize],
        ];
        if w[0] == w[1] || w[1] == w[2] || w[2] == w[0] {
            continue;
        }
        let fnrm = (weld_pos[w[1]] - weld_pos[w[0]])
            .cross(weld_pos[w[2]] - weld_pos[w[0]])
            .normalize_or_zero();
        if fnrm.length_squared() < 1e-12 {
            continue;
        }
        for e in 0..3 {
            let a = w[e];
            let b = w[(e + 1) % 3];
            let key = if a < b { (a, b) } else { (b, a) };
            edge_faces.entry(key).or_default().push(fnrm);
        }
    }

    for ((a, b), norms) in edge_faces {
        let pa = weld_pos[a];
        let pb = weld_pos[b];
        if norms.len() == 1 {
            if include_boundaries || include_silhouettes {
                out.push(CandidateEdgePoly {
                    points_3d: vec![pa, pb],
                    is_silhouette: include_silhouettes,
                    color,
                });
            }
        } else if norms.len() >= 2 {
            let n0 = norms[0];
            let n1 = norms[1];
            let is_sil = include_silhouettes && n0.dot(view_dir) * n1.dot(view_dir) <= 0.0;
            let is_crease = include_creases && n0.dot(n1) < crease_cos;
            if is_sil || is_crease {
                out.push(CandidateEdgePoly {
                    points_3d: vec![pa, pb],
                    is_silhouette: is_sil,
                    color,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BRep;

    #[test]
    fn make2d_isometric_box_classifies_visible_and_hidden_edges() {
        let cube = BRep::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let res = make_2d_projection(
            &[(Geometry::BRep(cube), None)],
            DrawingProjection::Isometric,
            &Make2DOptions::default(),
        );
        let vis_count = res
            .segments
            .iter()
            .filter(|s| s.kind == Make2DLineKind::Visible || s.kind == Make2DLineKind::Silhouette)
            .count();
        let hid_count = res
            .segments
            .iter()
            .filter(|s| s.kind == Make2DLineKind::Hidden)
            .count();
        assert!(vis_count > 0, "expected visible edges in isometric Make2D");
        assert!(hid_count > 0, "expected hidden back edges in isometric Make2D");
        assert!(res.bounds_max[0] > res.bounds_min[0]);
        assert!(res.bounds_max[1] > res.bounds_min[1]);
    }
}
