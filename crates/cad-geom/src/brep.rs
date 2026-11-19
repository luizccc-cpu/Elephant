use std::collections::HashMap;

use cad_math::linalg::solve_linear_system;
use cad_math::{Aabb, DMat4, DVec3, PARAM_EPSILON, Plane, Ray};
use serde::{Deserialize, Serialize};

use crate::error::GeomError;
use crate::mesh::TriangleMesh;
use crate::nurbs_curve::NurbsCurve;
use crate::nurbs_surface::{NurbsSurface, SurfaceEdge};

/// Closed 2D trimming loop in a NURBS surface's `(u, v)` parameter space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrimLoop2D {
    pub points: Vec<[f64; 2]>,
    pub is_hole: bool,
}

/// A possibly-trimmed NURBS face inside a `BRep`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRepFace {
    pub surface: NurbsSurface,
    pub outer_loop: Option<TrimLoop2D>,
    pub inner_loops: Vec<TrimLoop2D>,
    pub orientation_reversed: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRepEdge {
    pub curve: NurbsCurve,
    pub adjacent_faces: Vec<usize>,
    pub start_vertex: usize,
    pub end_vertex: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BRepVertex {
    pub position: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SubObjectRef {
    Face(usize),
    Edge(usize),
    Vertex(usize),
}

/// Boundary Representation (B-Rep) polysurface / solid holding trimmed NURBS faces and stitched topology.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BRep {
    pub faces: Vec<BRepFace>,
    pub edges: Vec<BRepEdge>,
    pub vertices: Vec<BRepVertex>,
}

impl BRepFace {
    pub fn untrimmed(surface: NurbsSurface) -> Self {
        Self {
            surface,
            outer_loop: None,
            inner_loops: Vec::new(),
            orientation_reversed: false,
        }
    }

    pub fn is_trimmed(&self) -> bool {
        self.outer_loop.is_some() || !self.inner_loops.is_empty()
    }

    pub fn flip_orientation(&mut self) {
        self.orientation_reversed = !self.orientation_reversed;
    }

    /// CCW outer trim polygon in `(u, v)` space without duplicate closing vertex.
    pub fn effective_outer_loop(&self) -> Vec<[f64; 2]> {
        if let Some(outer) = &self.outer_loop {
            let mut pts = clean_loop_2d(&outer.points);
            if signed_area_uv(&pts) < 0.0 {
                pts.reverse();
            }
            pts
        } else {
            let (u0, u1) = self.surface.domain_u();
            let (v0, v1) = self.surface.domain_v();
            vec![[u0, v0], [u1, v0], [u1, v1], [u0, v1]]
        }
    }

    pub fn contains_uv(&self, u: f64, v: f64) -> bool {
        let (u0, u1) = self.surface.domain_u();
        let (v0, v1) = self.surface.domain_v();
        if u < u0 - 1e-6 || u > u1 + 1e-6 || v < v0 - 1e-6 || v > v1 + 1e-6 {
            return false;
        }
        let outer = self.effective_outer_loop();
        if !point_in_poly_uv(u, v, &outer) {
            return false;
        }
        for hole in &self.inner_loops {
            let h = clean_loop_2d(&hole.points);
            if h.len() >= 3 && point_in_poly_uv(u, v, &h) {
                return false;
            }
        }
        true
    }

    pub fn evaluate_point_and_normal(&self, u: f64, v: f64) -> (DVec3, DVec3) {
        let (p, mut n) = self.surface.evaluate_point_and_normal(u, v);
        if self.orientation_reversed {
            n = -n;
        }
        (p, n)
    }

    pub fn interior_sample_point(&self) -> Option<(f64, f64, DVec3, DVec3)> {
        if !self.is_trimmed() {
            let (u0, u1) = self.surface.domain_u();
            let (v0, v1) = self.surface.domain_v();
            let u = 0.5 * (u0 + u1);
            let v = 0.5 * (v0 + v1);
            let (p, n) = self.evaluate_point_and_normal(u, v);
            return Some((u, v, p, n));
        }

        let outer = self.effective_outer_loop();
        if outer.len() < 3 {
            return None;
        }
        let mut holes: Vec<Vec<[f64; 2]>> = self
            .inner_loops
            .iter()
            .filter_map(|h| {
                let mut c = clean_loop_2d(&h.points);
                if c.len() < 3 {
                    return None;
                }
                if signed_area_uv(&c) > 0.0 {
                    c.reverse();
                }
                Some(c)
            })
            .collect();

        let (merged, _) = merge_holes_into_outer(&outer, &mut holes);
        let tris = triangulate_uv_polygon(&merged);
        if let Some((best_tri, _)) = tris
            .iter()
            .filter_map(|&[i0, i1, i2]| {
                let a = merged[i0];
                let b = merged[i1];
                let c = merged[i2];
                let area = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs();
                (area > 1e-12).then_some(([i0, i1, i2], area))
            })
            .max_by(|a, b| a.1.total_cmp(&b.1))
        {
            let u = (merged[best_tri[0]][0] + merged[best_tri[1]][0] + merged[best_tri[2]][0])
                / 3.0;
            let v = (merged[best_tri[0]][1] + merged[best_tri[1]][1] + merged[best_tri[2]][1])
                / 3.0;
            if self.contains_uv(u, v) {
                let (p, n) = self.evaluate_point_and_normal(u, v);
                return Some((u, v, p, n));
            }
        }

        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for &[u, v] in &outer {
            u_min = u_min.min(u);
            u_max = u_max.max(u);
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }
        for iu in 1..12 {
            let u = u_min + (u_max - u_min) * (iu as f64 / 12.0);
            for iv in 1..12 {
                let v = v_min + (v_max - v_min) * (iv as f64 / 12.0);
                if self.contains_uv(u, v) {
                    let (p, n) = self.evaluate_point_and_normal(u, v);
                    return Some((u, v, p, n));
                }
            }
        }
        None
    }

    /// Conformal watertight display tessellation handling holes and curved trimmed surfaces.
    pub fn tessellate_display(&self) -> TriangleMesh {
        self.tessellate_display_with_uv().0
    }

    /// Conformal watertight display tessellation paired with per-vertex `(u, v)` surface parameters.
    pub fn tessellate_display_with_uv(&self) -> (TriangleMesh, Vec<[f64; 2]>) {
        if !self.is_trimmed() {
            let (mut mesh, uvs) = self.surface.tessellate_display_with_uv();
            if self.orientation_reversed {
                for tri in &mut mesh.indices {
                    tri.swap(1, 2);
                }
                for n in &mut mesh.normals {
                    *n = [-n[0], -n[1], -n[2]];
                }
            }
            return (mesh, uvs);
        }

        let curved = self.surface.degree_u > 1 || self.surface.degree_v > 1;
        let (u0, u1) = self.surface.domain_u();
        let (v0, v1) = self.surface.domain_v();
        let max_uv_step = if curved {
            0.08 * (u1 - u0).max(v1 - v0)
        } else {
            f64::INFINITY
        };

        let mut outer = self.effective_outer_loop();
        if outer.len() < 3 {
            return (
                TriangleMesh {
                    positions: Vec::new(),
                    normals: Vec::new(),
                    indices: Vec::new(),
                },
                Vec::new(),
            );
        }
        if curved && self.inner_loops.is_empty() {
            let mut u_min = f64::INFINITY;
            let mut u_max = f64::NEG_INFINITY;
            let mut v_min = f64::INFINITY;
            let mut v_max = f64::NEG_INFINITY;
            for &[u, v] in &outer {
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
            let du = u_max - u_min;
            let dv = v_max - v_min;
            let rect_area = du * dv;
            let uv_span = (u1 - u0).max(v1 - v0);
            let on_rect = du > 1e-7
                && dv > 1e-7
                && signed_area_uv(&outer).abs() >= 0.995 * rect_area
                && outer.iter().all(|&[u, v]| {
                    (u - u_min)
                        .abs()
                        .min((u - u_max).abs())
                        .min((v - v_min).abs())
                        .min((v - v_max).abs())
                        <= uv_span * 1e-3
                });
            if on_rect {
                let nu = if self.surface.degree_u <= 1 {
                    2
                } else {
                    (du / (u1 - u0).max(1e-9) * 36.0).ceil().clamp(6.0, 48.0) as usize
                };
                let nv = if self.surface.degree_v <= 1 {
                    2
                } else {
                    (dv / (v1 - v0).max(1e-9) * 36.0).ceil().clamp(6.0, 48.0) as usize
                };
                let mut positions = Vec::with_capacity((nu + 1) * (nv + 1));
                let mut normals = Vec::with_capacity((nu + 1) * (nv + 1));
                let mut uvs = Vec::with_capacity((nu + 1) * (nv + 1));
                for iu in 0..=nu {
                    let u = u_min + du * (iu as f64 / nu as f64);
                    for iv in 0..=nv {
                        let v = v_min + dv * (iv as f64 / nv as f64);
                        let (p, n) = self.evaluate_point_and_normal(u, v);
                        positions.push(p.to_array());
                        normals.push(n.to_array());
                        uvs.push([u, v]);
                    }
                }
                let stride = nv + 1;
                let mut indices = Vec::with_capacity(nu * nv * 2);
                for iu in 0..nu {
                    for iv in 0..nv {
                        let a = (iu * stride + iv) as u32;
                        let b = ((iu + 1) * stride + iv) as u32;
                        let c = ((iu + 1) * stride + iv + 1) as u32;
                        let d = (iu * stride + iv + 1) as u32;
                        if self.orientation_reversed {
                            indices.push([a, c, b]);
                            indices.push([a, d, c]);
                        } else {
                            indices.push([a, b, c]);
                            indices.push([a, c, d]);
                        }
                    }
                }
                return (
                    TriangleMesh {
                        positions,
                        normals,
                        indices,
                    },
                    uvs,
                );
            }
        }
        if curved {
            outer = refine_uv_loop(&outer, max_uv_step);
        }

        let mut holes: Vec<Vec<[f64; 2]>> = self
            .inner_loops
            .iter()
            .filter_map(|h| {
                let mut pts = clean_loop_2d(&h.points);
                if pts.len() < 3 {
                    return None;
                }
                if signed_area_uv(&pts) > 0.0 {
                    pts.reverse();
                }
                if curved {
                    pts = refine_uv_loop(&pts, max_uv_step);
                }
                Some(pts)
            })
            .collect();

        let (mut uv_verts, _) = merge_holes_into_outer(&outer, &mut holes);
        let mut tris = triangulate_uv_polygon(&uv_verts);
        if tris.is_empty() {
            return (
                TriangleMesh {
                    positions: Vec::new(),
                    normals: Vec::new(),
                    indices: Vec::new(),
                },
                Vec::new(),
            );
        }

        if curved {
            let diag = self.surface.bounding_box().size().length().max(1.0);
            let chord_tol = diag * 0.0025;
            let interior_uv_step = 0.16 * (u1 - u0).max(v1 - v0);
            conformal_refine_uv_mesh(
                &self.surface,
                &mut uv_verts,
                &mut tris,
                interior_uv_step,
                chord_tol,
            );
        }

        let mut positions = Vec::with_capacity(uv_verts.len());
        let mut normals = Vec::with_capacity(uv_verts.len());
        for &[u, v] in &uv_verts {
            let (p, n) = self.evaluate_point_and_normal(u, v);
            positions.push(p.to_array());
            normals.push(n.to_array());
        }

        let mut indices = Vec::with_capacity(tris.len());
        for [a, b, c] in tris {
            let pa = DVec3::from_array(positions[a]);
            let pb = DVec3::from_array(positions[b]);
            let pc = DVec3::from_array(positions[c]);
            if (pb - pa).cross(pc - pa).length_squared() < 1e-18 {
                continue;
            }
            if self.orientation_reversed {
                indices.push([a as u32, c as u32, b as u32]);
            } else {
                indices.push([a as u32, b as u32, c as u32]);
            }
        }

        (
            TriangleMesh {
                positions,
                normals,
                indices,
            },
            uv_verts,
        )
    }

    pub fn boundary_curves_3d(&self) -> Vec<NurbsCurve> {
        if !self.is_trimmed() {
            return SurfaceEdge::ALL
                .into_iter()
                .filter_map(|e| {
                    let c = self.surface.boundary_curve(e).ok()?;
                    (c.arc_length() >= 1e-6).then_some(c)
                })
                .collect();
        }

        let mut out = Vec::new();
        let outer = self.effective_outer_loop();
        out.extend(loop_uv_to_curves_3d(&self.surface, &outer));
        for hole in &self.inner_loops {
            let h = clean_loop_2d(&hole.points);
            if h.len() >= 3 {
                out.extend(loop_uv_to_curves_3d(&self.surface, &h));
            }
        }
        out
    }

    pub fn area_approx(&self) -> f64 {
        let mesh = self.tessellate_display();
        mesh_area(&mesh)
    }
}

impl BRep {
    /// Assemble faces into a stitched `BRep`, discovering vertices and shared edges within `tol`.
    pub fn from_faces(faces: Vec<BRepFace>, tol: f64) -> Self {
        let mut brep = Self {
            faces,
            edges: Vec::new(),
            vertices: Vec::new(),
        };
        brep.rebuild_topology(tol);
        brep
    }

    pub fn from_surface(surface: NurbsSurface) -> Self {
        Self::from_faces(vec![BRepFace::untrimmed(surface)], 1e-4)
    }

    /// Recompute `vertices` and `edges` (with `adjacent_faces`) from `self.faces`.
    pub fn rebuild_topology(&mut self, tol: f64) {
        self.edges.clear();
        self.vertices.clear();
        let v_tol = (tol * 4.0).max(1e-4);
        let e_tol = (tol * 8.0).max(2e-4);

        let get_or_insert_vertex = |pt: DVec3, vertices: &mut Vec<BRepVertex>| -> usize {
            for (vi, v) in vertices.iter().enumerate() {
                if v.position.distance(pt) <= v_tol {
                    return vi;
                }
            }
            let idx = vertices.len();
            vertices.push(BRepVertex { position: pt });
            idx
        };

        for (fi, face) in self.faces.iter().enumerate() {
            for curve in face.boundary_curves_3d() {
                if curve.arc_length() < 1e-6 {
                    continue;
                }
                let (u0, u1) = curve.domain();
                let s_pt = curve.evaluate(u0);
                let e_pt = curve.evaluate(u1);

                let matched = self
                    .edges
                    .iter()
                    .position(|existing| curves_match_topologically(&existing.curve, &curve, e_tol));

                if let Some(ei) = matched {
                    self.edges[ei].adjacent_faces.push(fi);
                } else {
                    let sv = get_or_insert_vertex(s_pt, &mut self.vertices);
                    let ev = get_or_insert_vertex(e_pt, &mut self.vertices);
                    self.edges.push(BRepEdge {
                        curve,
                        adjacent_faces: vec![fi],
                        start_vertex: sv,
                        end_vertex: ev,
                    });
                }
            }
        }
    }

    /// Watertight 6-face NURBS solid box centered at `center` with extents `size`.
    pub fn from_box(center: DVec3, size: DVec3) -> Self {
        let sx = size.x.abs().max(1e-4);
        let sy = size.y.abs().max(1e-4);
        let sz = size.z.abs().max(1e-4);
        let h = DVec3::new(sx * 0.5, sy * 0.5, sz * 0.5);
        let specs = [
            (DVec3::new(0.0, 0.0, h.z), DVec3::X, DVec3::Y, DVec3::Z, sx, sy),
            (DVec3::new(0.0, 0.0, -h.z), -DVec3::X, DVec3::Y, -DVec3::Z, sx, sy),
            (DVec3::new(h.x, 0.0, 0.0), DVec3::Y, DVec3::Z, DVec3::X, sy, sz),
            (DVec3::new(-h.x, 0.0, 0.0), -DVec3::Y, DVec3::Z, -DVec3::X, sy, sz),
            (DVec3::new(0.0, h.y, 0.0), -DVec3::X, DVec3::Z, DVec3::Y, sx, sz),
            (DVec3::new(0.0, -h.y, 0.0), DVec3::X, DVec3::Z, -DVec3::Y, sx, sz),
        ];
        let faces = specs
            .into_iter()
            .map(|(off, x, y, z, w, ht)| {
                let plane = Plane {
                    origin: center + off,
                    x_axis: x,
                    y_axis: y,
                    z_axis: z,
                };
                BRepFace::untrimmed(NurbsSurface::plane_rect(&plane, w, ht))
            })
            .collect();
        Self::from_faces(faces, 1e-4)
    }

    /// Closed rational NURBS cylinder solid (wall + bottom and top circular planar caps).
    pub fn from_cylinder(
        base_center: DVec3,
        radius: f64,
        height: f64,
    ) -> Result<Self, GeomError> {
        let wall = NurbsSurface::cylinder(base_center, radius, height)?;
        let r = radius.abs();
        let mut faces = Vec::with_capacity(3);
        faces.push(BRepFace::untrimmed(wall));

        let sign_h = if height >= 0.0 { 1.0 } else { -1.0 };
        let bot_plane = Plane {
            origin: base_center,
            x_axis: DVec3::X,
            y_axis: DVec3::Y * sign_h,
            z_axis: -DVec3::Z * sign_h,
        };
        let top_plane = Plane {
            origin: base_center + DVec3::Z * height,
            x_axis: DVec3::X,
            y_axis: DVec3::Y * sign_h,
            z_axis: DVec3::Z * sign_h,
        };
        faces.push(circular_cap_face(&bot_plane, r, bot_plane.z_axis));
        faces.push(circular_cap_face(&top_plane, r, top_plane.z_axis));
        let mut brep = Self::from_faces(faces, 1e-3);
        if brep.signed_volume_approx() < 0.0 {
            brep.flip_normals();
        }
        Ok(brep)
    }

    /// Closed rational NURBS cone solid (wall + circular base cap).
    pub fn from_cone(base_center: DVec3, radius: f64, height: f64) -> Result<Self, GeomError> {
        let wall = NurbsSurface::cone(base_center, radius, height)?;
        let r = radius.abs();
        let sign_h = if height >= 0.0 { 1.0 } else { -1.0 };
        let bot_plane = Plane {
            origin: base_center,
            x_axis: DVec3::X,
            y_axis: DVec3::Y * sign_h,
            z_axis: -DVec3::Z * sign_h,
        };
        let cap = circular_cap_face(&bot_plane, r, bot_plane.z_axis);
        let mut brep = Self::from_faces(vec![BRepFace::untrimmed(wall), cap], 1e-3);
        if brep.signed_volume_approx() < 0.0 {
            brep.flip_normals();
        }
        Ok(brep)
    }

    pub fn from_sphere(center: DVec3, radius: f64) -> Result<Self, GeomError> {
        Ok(Self::from_surface(NurbsSurface::sphere(center, radius)?))
    }

    pub fn from_torus(
        center: DVec3,
        major_radius: f64,
        minor_radius: f64,
    ) -> Result<Self, GeomError> {
        Ok(Self::from_surface(NurbsSurface::torus(
            center,
            major_radius,
            minor_radius,
        )?))
    }

    /// Construct a planar trimmed `BRep` from 1+ closed planar curves (outer boundary + inner holes).
    pub fn from_planar_curves(curves: &[NurbsCurve]) -> Result<Self, GeomError> {
        if curves.is_empty() {
            return Err(GeomError::Degenerate("from_planar_curves requires at least 1 curve"));
        }
        let mut pts_per_curve = Vec::with_capacity(curves.len());
        let mut all_pts = Vec::new();
        for c in curves {
            if !c.is_closed() {
                return Err(GeomError::Degenerate("PlanarSrf requires closed curves"));
            }
            let mut pts: Vec<DVec3> = if c.degree == 1 {
                c.control_points.iter().map(|cp| cp.position).collect()
            } else {
                c.sample_polyline(48)
            };
            if pts.len() >= 2 && pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-5 {
                pts.pop();
            }
            if pts.len() < 3 {
                return Err(GeomError::Degenerate("planar loop has too few points"));
            }
            all_pts.extend_from_slice(&pts);
            pts_per_curve.push(pts);
        }

        let plane = fit_plane_from_loop(&pts_per_curve[0])
            .ok_or(GeomError::Degenerate("could not determine loop plane"))?;

        let mut min_x = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for &p in &all_pts {
            let rel = p - plane.origin;
            let x = rel.dot(plane.x_axis);
            let y = rel.dot(plane.y_axis);
            min_x = min_x.min(x);
            max_x = max_x.max(x);
            min_y = min_y.min(y);
            max_y = max_y.max(y);
        }
        let pad = ((max_x - min_x).max(max_y - min_y) * 0.05).max(1e-2);
        min_x -= pad;
        max_x += pad;
        min_y -= pad;
        max_y += pad;
        let w = (max_x - min_x).max(1e-3);
        let h = (max_y - min_y).max(1e-3);
        let srf_plane = Plane {
            origin: plane.origin
                + plane.x_axis * (0.5 * (min_x + max_x))
                + plane.y_axis * (0.5 * (min_y + max_y)),
            x_axis: plane.x_axis,
            y_axis: plane.y_axis,
            z_axis: plane.z_axis,
        };
        let srf = NurbsSurface::plane_rect(&srf_plane, w, h);

        let uv_loops: Vec<(Vec<[f64; 2]>, f64)> = pts_per_curve
            .iter()
            .map(|pts| {
                let mut uv: Vec<[f64; 2]> = pts
                    .iter()
                    .map(|&p| {
                        let rel = p - srf_plane.origin;
                        let u = (rel.dot(srf_plane.x_axis) / w + 0.5).clamp(0.0, 1.0);
                        let v = (rel.dot(srf_plane.y_axis) / h + 0.5).clamp(0.0, 1.0);
                        [u, v]
                    })
                    .collect();
                uv = clean_loop_2d(&uv);
                let area = signed_area_uv(&uv);
                (uv, area.abs())
            })
            .collect();

        // Largest-area loop is outer boundary; remaining nested loops are holes
        let outer_idx = uv_loops
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.1.total_cmp(&b.1.1))
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mut outer_pts = uv_loops[outer_idx].0.clone();
        if signed_area_uv(&outer_pts) < 0.0 {
            outer_pts.reverse();
        }
        let mut inner_loops = Vec::new();
        for (i, (mut pts, _)) in uv_loops.into_iter().enumerate() {
            if i != outer_idx && pts.len() >= 3 {
                if signed_area_uv(&pts) > 0.0 {
                    pts.reverse();
                }
                inner_loops.push(TrimLoop2D {
                    points: pts,
                    is_hole: true,
                });
            }
        }

        let face = BRepFace {
            surface: srf,
            outer_loop: Some(TrimLoop2D {
                points: outer_pts,
                is_hole: false,
            }),
            inner_loops,
            orientation_reversed: false,
        };
        Ok(Self::from_faces(vec![face], 1e-4))
    }

    /// Extrude a closed planar curve into a capped solid `BRep` with NURBS walls + trimmed planar caps.
    pub fn extrude_curve_solid(curve: &NurbsCurve, direction: DVec3) -> Result<Self, GeomError> {
        if !curve.is_closed() || direction.length() < 1e-6 {
            return Err(GeomError::Degenerate(
                "extrude_curve_solid requires a closed curve and non-zero direction",
            ));
        }
        let mut faces = Vec::new();
        let segs = curve.explode_segments();
        for seg in &segs {
            faces.push(BRepFace::untrimmed(NurbsSurface::extrude_curve(
                seg, direction,
            )));
        }

        let mut bot = Self::from_planar_curves(std::slice::from_ref(curve))?;
        let mut top_curve = curve.clone();
        top_curve.transform(&DMat4::from_translation(direction));
        let mut top = Self::from_planar_curves(&[top_curve])?;

        if let Some(bot_face) = bot.faces.pop() {
            faces.push(bot_face);
        }
        if let Some(top_face) = top.faces.pop() {
            faces.push(top_face);
        }

        let mut brep = Self::from_faces(faces, 1e-3);
        // Align each face normal consistently relative to closed solid interior
        brep.orient_outward_consistent();
        Ok(brep)
    }

    pub fn join_many(breps: &[Self], tol: f64) -> Self {
        let mut faces = Vec::new();
        for b in breps {
            faces.extend(b.faces.iter().cloned());
        }
        Self::from_faces(faces, tol)
    }

    pub fn explode(&self) -> Vec<Self> {
        self.faces
            .iter()
            .cloned()
            .map(|f| Self::from_faces(vec![f], 1e-4))
            .collect()
    }

    pub fn naked_edges(&self) -> Vec<usize> {
        self.edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| (e.adjacent_faces.len() == 1).then_some(i))
            .collect()
    }

    pub fn non_manifold_edges(&self) -> Vec<usize> {
        self.edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| (e.adjacent_faces.len() > 2).then_some(i))
            .collect()
    }

    pub fn is_solid(&self) -> bool {
        !self.faces.is_empty()
            && !self.edges.is_empty()
            && self.edges.iter().all(|e| e.adjacent_faces.len() == 2)
    }

    pub fn flip_normals(&mut self) {
        for f in &mut self.faces {
            f.flip_orientation();
        }
    }

    /// Flip individual faces of a closed shell if their outward sample rays point inward, ensuring signed volume > 0.
    pub fn orient_outward_consistent(&mut self) {
        for fi in 0..self.faces.len() {
            if let Some((_, _, p, n)) = self.faces[fi].interior_sample_point() {
                if n.length_squared() < 1e-12 {
                    continue;
                }
                let diag = self.bounding_box().size().length().max(1.0);
                let eps = diag * 1e-4;
                // Ray starting outside a closed solid hits an even number of faces; odd means p + n*eps is inside.
                if ray_hits_count(self, p + n * eps, n) % 2 == 1 {
                    self.faces[fi].flip_orientation();
                }
            }
        }
        if self.signed_volume_approx() < 0.0 {
            self.flip_normals();
        }
    }

    pub fn bounding_box(&self) -> Aabb {
        let mut out = Aabb::EMPTY;
        for f in &self.faces {
            out = out.union(&f.surface.bounding_box());
        }
        out
    }

    pub fn transform(&mut self, mat: &DMat4) {
        let det = mat.determinant();
        for f in &mut self.faces {
            f.surface.transform(mat);
            if det < 0.0 {
                f.flip_orientation();
            }
        }
        for e in &mut self.edges {
            e.curve.transform(mat);
        }
        for v in &mut self.vertices {
            v.position = mat.transform_point3(v.position);
        }
    }

    pub fn tessellate_display(&self) -> TriangleMesh {
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();
        for face in &self.faces {
            let m = face.tessellate_display();
            let base = positions.len() as u32;
            positions.extend_from_slice(&m.positions);
            normals.extend_from_slice(&m.normals);
            for &[a, b, c] in &m.indices {
                indices.push([base + a, base + b, base + c]);
            }
        }
        TriangleMesh {
            positions,
            normals,
            indices,
        }
    }

    pub fn ray_intersect(&self, ray: &Ray) -> Option<f64> {
        let mut best: Option<f64> = None;
        for f in &self.faces {
            if let Some(t) = f.tessellate_display().ray_intersect(ray) {
                if best.is_none_or(|b| t < b) {
                    best = Some(t);
                }
            }
        }
        best
    }

    /// Pick a specific B-Rep edge or face under `ray` (`Ctrl+Shift+Click` sub-object selection).
    pub fn pick_subobject(&self, ray: &Ray, edge_tol: f64) -> Option<(SubObjectRef, f64)> {
        let mut best_edge: Option<(usize, f64, f64)> = None;
        for (ei, edge) in self.edges.iter().enumerate() {
            if let Some((t, d)) = edge.curve.ray_distance(ray, 32) {
                if d <= edge_tol
                    && best_edge.is_none_or(|b| d < b.2 || ((d - b.2).abs() < 1e-9 && t < b.1))
                {
                    best_edge = Some((ei, t, d));
                }
            }
        }
        let mut best_face: Option<(usize, f64)> = None;
        for (fi, face) in self.faces.iter().enumerate() {
            if let Some(t) = face.tessellate_display().ray_intersect(ray) {
                if best_face.is_none_or(|b| t < b.1) {
                    best_face = Some((fi, t));
                }
            }
        }
        match (best_edge, best_face) {
            (Some((ei, et, _)), Some((fi, ft))) => {
                if et <= ft + edge_tol * 2.5 {
                    Some((SubObjectRef::Edge(ei), et))
                } else {
                    Some((SubObjectRef::Face(fi), ft))
                }
            }
            (Some((ei, et, _)), None) => Some((SubObjectRef::Edge(ei), et)),
            (None, Some((fi, ft))) => Some((SubObjectRef::Face(fi), ft)),
            (None, None) => None,
        }
    }

    pub fn area_approx(&self) -> f64 {
        mesh_area(&self.tessellate_display())
    }

    pub fn signed_volume_approx(&self) -> f64 {
        let mesh = self.tessellate_display();
        let mut vol6 = 0.0;
        for &[i0, i1, i2] in &mesh.indices {
            let a = DVec3::from_array(mesh.positions[i0 as usize]);
            let b = DVec3::from_array(mesh.positions[i1 as usize]);
            let c = DVec3::from_array(mesh.positions[i2 as usize]);
            vol6 += a.dot(b.cross(c));
        }
        vol6 / 6.0
    }

    pub fn volume_approx(&self) -> f64 {
        self.signed_volume_approx().abs()
    }

    /// 3-ray majority parity inside-test against the display mesh of this solid.
    pub fn contains_point(&self, pt: DVec3) -> bool {
        let bb = self.bounding_box().expanded(1e-3);
        if pt.x < bb.min.x
            || pt.x > bb.max.x
            || pt.y < bb.min.y
            || pt.y > bb.max.y
            || pt.z < bb.min.z
            || pt.z > bb.max.z
        {
            return false;
        }
        let mesh = self.tessellate_display();
        let dirs = [
            DVec3::new(0.7131, 0.5173, 0.4732).normalize(),
            DVec3::new(-0.3719, 0.8191, -0.4368).normalize(),
            DVec3::new(0.2943, -0.4117, 0.8626).normalize(),
        ];
        let mut inside_votes = 0usize;
        for dir in dirs {
            let hits = ray_mesh_unique_hit_count(&mesh, pt, dir);
            if hits % 2 == 1 {
                inside_votes += 1;
            }
        }
        inside_votes >= 2
    }

    pub fn snap_points(&self) -> Vec<DVec3> {
        let mut pts: Vec<DVec3> = self.vertices.iter().map(|v| v.position).collect();
        for e in &self.edges {
            let (u0, u1) = e.curve.domain();
            pts.push(e.curve.evaluate(0.5 * (u0 + u1)));
        }
        pts
    }
}

// --- Internal geometry & topology helpers ---

pub(crate) fn circular_cap_face(plane: &Plane, radius: f64, target_normal: DVec3) -> BRepFace {
    let side = radius * 2.1;
    let srf = NurbsSurface::plane_rect(plane, side, side);
    let n = 64_usize;
    let mut uv_pts = Vec::with_capacity(n);
    for i in 0..n {
        let a = std::f64::consts::TAU * (i as f64 / n as f64);
        let p = plane.origin + (plane.x_axis * a.cos() + plane.y_axis * a.sin()) * radius;
        let rel = p - plane.origin;
        let u = (rel.dot(plane.x_axis) / side + 0.5).clamp(0.0, 1.0);
        let v = (rel.dot(plane.y_axis) / side + 0.5).clamp(0.0, 1.0);
        uv_pts.push([u, v]);
    }
    if signed_area_uv(&uv_pts) < 0.0 {
        uv_pts.reverse();
    }
    let (_, srf_n) = srf.evaluate_point_and_normal(0.5, 0.5);
    let orientation_reversed = srf_n.dot(target_normal) < 0.0;
    BRepFace {
        surface: srf,
        outer_loop: Some(TrimLoop2D {
            points: uv_pts,
            is_hole: false,
        }),
        inner_loops: Vec::new(),
        orientation_reversed,
    }
}

fn ray_hits_count(brep: &BRep, origin: DVec3, dir: DVec3) -> usize {
    let mesh = brep.tessellate_display();
    ray_mesh_unique_hit_count(&mesh, origin, dir)
}

fn ray_mesh_unique_hit_count(mesh: &TriangleMesh, origin: DVec3, dir: DVec3) -> usize {
    let ray = Ray::new(origin, dir);
    let mut ts = Vec::new();
    for &[i0, i1, i2] in &mesh.indices {
        let v0 = DVec3::from_array(mesh.positions[i0 as usize]);
        let v1 = DVec3::from_array(mesh.positions[i1 as usize]);
        let v2 = DVec3::from_array(mesh.positions[i2 as usize]);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let h = ray.direction.cross(e2);
        let a = e1.dot(h);
        if a.abs() < PARAM_EPSILON {
            continue;
        }
        let f = 1.0 / a;
        let s = ray.origin - v0;
        let u = f * s.dot(h);
        if !(-1e-7..=1.0 + 1e-7).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = f * ray.direction.dot(q);
        if v < -1e-7 || u + v > 1.0 + 1e-7 {
            continue;
        }
        let t = f * e2.dot(q);
        if t > 1e-6 {
            ts.push(t);
        }
    }
    ts.sort_by(|a, b| a.total_cmp(b));
    ts.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
    ts.len()
}

fn mesh_area(mesh: &TriangleMesh) -> f64 {
    let mut total = 0.0;
    for &[i0, i1, i2] in &mesh.indices {
        let a = DVec3::from_array(mesh.positions[i0 as usize]);
        let b = DVec3::from_array(mesh.positions[i1 as usize]);
        let c = DVec3::from_array(mesh.positions[i2 as usize]);
        total += 0.5 * (b - a).cross(c - a).length();
    }
    total
}

pub(crate) fn clean_loop_2d(pts: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(pts.len());
    for &p in pts {
        if out
            .last()
            .is_none_or(|&q| (q[0] - p[0]).hypot(q[1] - p[1]) > 1e-9)
        {
            out.push(p);
        }
    }
    if out.len() >= 2 {
        let first = out[0];
        let last = *out.last().unwrap();
        if (first[0] - last[0]).hypot(first[1] - last[1]) <= 1e-9 {
            out.pop();
        }
    }
    out
}

pub(crate) fn signed_area_uv(pts: &[[f64; 2]]) -> f64 {
    if pts.len() < 3 {
        return 0.0;
    }
    let mut a = 0.0;
    for i in 0..pts.len() {
        let p = pts[i];
        let q = pts[(i + 1) % pts.len()];
        a += p[0] * q[1] - q[0] * p[1];
    }
    a * 0.5
}

pub(crate) fn point_in_poly_uv(u: f64, v: f64, poly: &[[f64; 2]]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    // Treat points directly on boundary as inside
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        if point_to_seg_dist_uv([u, v], a, b) <= 1e-7 {
            return true;
        }
    }
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i][0], poly[i][1]);
        let (xj, yj) = (poly[j][0], poly[j][1]);
        let intersect =
            ((yi > v) != (yj > v)) && (u < (xj - xi) * (v - yi) / (yj - yi) + xi);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn point_to_seg_dist_uv(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-20 {
        return (p[0] - a[0]).hypot(p[1] - a[1]);
    }
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0);
    let px = a[0] + t * dx;
    let py = a[1] + t * dy;
    (p[0] - px).hypot(p[1] - py)
}

fn refine_uv_loop(pts: &[[f64; 2]], max_step: f64) -> Vec<[f64; 2]> {
    if pts.len() < 2 || !max_step.is_finite() || max_step <= 1e-7 {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(pts.len() * 2);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        out.push(a);
        let dist = (b[0] - a[0]).hypot(b[1] - a[1]);
        if dist > max_step {
            let sub = ((dist / max_step).ceil() as usize).clamp(2, 16);
            for k in 1..sub {
                let t = k as f64 / sub as f64;
                out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
            }
        }
    }
    out
}

/// Eberly's mutually-visible vertex bridging merges clockwise holes into CCW outer loop.
fn merge_holes_into_outer(outer: &[[f64; 2]], holes: &mut [Vec<[f64; 2]>]) -> (Vec<[f64; 2]>, bool) {
    let mut combined = outer.to_vec();
    if holes.is_empty() {
        return (combined, false);
    }

    // Sort holes by descending rightmost U coordinate
    holes.sort_by(|ha, hb| {
        let max_a = ha.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        let max_b = hb.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        max_b.total_cmp(&max_a)
    });

    for hole in holes.iter() {
        if hole.len() < 3 {
            continue;
        }
        let Some((h_idx, &h_pt)) = hole
            .iter()
            .enumerate()
            .max_by(|a, b| a.1[0].total_cmp(&b.1[0]))
        else {
            continue;
        };

        // Find +U ray intersection with combined outer polygon edges
        let mut best_u = f64::INFINITY;
        let mut edge_start = 0usize;
        for i in 0..combined.len() {
            let a = combined[i];
            let b = combined[(i + 1) % combined.len()];
            let (low, high) = if a[1] <= b[1] { (a, b) } else { (b, a) };
            if h_pt[1] >= low[1] - 1e-11 && h_pt[1] <= high[1] + 1e-11 && (high[1] - low[1]).abs() > 1e-12 {
                let t = (h_pt[1] - a[1]) / (b[1] - a[1]);
                let x_hit = a[0] + t * (b[0] - a[0]);
                if x_hit >= h_pt[0] - 1e-11 && x_hit < best_u {
                    best_u = x_hit;
                    edge_start = i;
                }
            }
        }

        let candidate_idx = if best_u.is_finite() {
            let va = combined[edge_start];
            let vb = combined[(edge_start + 1) % combined.len()];
            let p_idx = if va[0] >= vb[0] {
                edge_start
            } else {
                (edge_start + 1) % combined.len()
            };
            // Verify mutual visibility inside triangle (h_pt, [best_u, h_pt.y], combined[p_idx])
            let p_pt = combined[p_idx];
            let mut reflex_cand = p_idx;
            let mut best_dist = f64::INFINITY;
            for (vi, &v_pt) in combined.iter().enumerate() {
                if vi == p_idx {
                    continue;
                }
                if v_pt[0] > h_pt[0] - 1e-11
                    && point_in_tri_uv(v_pt, h_pt, [best_u, h_pt[1]], p_pt)
                {
                    let d = (v_pt[0] - h_pt[0]).hypot(v_pt[1] - h_pt[1]);
                    if d < best_dist {
                        best_dist = d;
                        reflex_cand = vi;
                    }
                }
            }
            reflex_cand
        } else {
            // Closest visible vertex fallback
            (0..combined.len())
                .min_by(|&ia, &ib| {
                    let da = (combined[ia][0] - h_pt[0]).hypot(combined[ia][1] - h_pt[1]);
                    let db = (combined[ib][0] - h_pt[0]).hypot(combined[ib][1] - h_pt[1]);
                    da.total_cmp(&db)
                })
                .unwrap_or(0)
        };

        // Splice hole at `candidate_idx`
        let p_cand = combined[candidate_idx];
        let mut next = Vec::with_capacity(combined.len() + hole.len() + 2);
        next.extend_from_slice(&combined[..=candidate_idx]);
        for k in 0..hole.len() {
            next.push(hole[(h_idx + k) % hole.len()]);
        }
        next.push(h_pt);
        next.push(p_cand);
        next.extend_from_slice(&combined[candidate_idx + 1..]);
        combined = next;
    }

    (combined, true)
}

fn triangulate_uv_polygon(poly: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = poly.len();
    if n < 3 {
        return Vec::new();
    }
    let mut indices: Vec<usize> = (0..n).collect();
    if signed_area_uv(poly) < 0.0 {
        indices.reverse();
    }
    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    let mut guard = 0usize;
    let max_guard = n * n + 16;

    while indices.len() > 3 && guard < max_guard {
        guard += 1;
        let m = indices.len();
        let mut clipped = false;
        for i in 0..m {
            let ip = indices[(i + m - 1) % m];
            let ic = indices[i];
            let inx = indices[(i + 1) % m];
            let a = poly[ip];
            let b = poly[ic];
            let c = poly[inx];
            let cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
            if cross <= 1e-12 {
                continue;
            }
            let mut inside = false;
            for &other in &indices {
                if other == ip || other == ic || other == inx {
                    continue;
                }
                if (poly[other][0] - a[0]).hypot(poly[other][1] - a[1]) < 1e-10
                    || (poly[other][0] - b[0]).hypot(poly[other][1] - b[1]) < 1e-10
                    || (poly[other][0] - c[0]).hypot(poly[other][1] - c[1]) < 1e-10
                {
                    continue;
                }
                if point_in_tri_uv(poly[other], a, b, c) {
                    inside = true;
                    break;
                }
            }
            if !inside {
                tris.push([ip, ic, inx]);
                indices.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            break;
        }
    }
    if indices.len() == 3 {
        tris.push([indices[0], indices[1], indices[2]]);
    }
    tris
}

fn point_in_tri_uv(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let cross = |o: [f64; 2], u: [f64; 2], v: [f64; 2]| -> f64 {
        (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])
    };
    let c1 = cross(a, b, p);
    let c2 = cross(b, c, p);
    let c3 = cross(c, a, p);
    c1 >= -1e-11 && c2 >= -1e-11 && c3 >= -1e-11
}

/// Red-green conformal triangle subdivision so curved trimmed surfaces never exhibit T-junctions.
fn conformal_refine_uv_mesh(
    srf: &NurbsSurface,
    verts: &mut Vec<[f64; 2]>,
    tris: &mut Vec<[usize; 3]>,
    max_uv_step: f64,
    chord_tol: f64,
) {
    for _pass in 0..6 {
        if verts.len() >= 8000 {
            break;
        }
        let pos3d: Vec<DVec3> = verts.iter().map(|&[u, v]| srf.evaluate(u, v)).collect();
        let mut split_map: HashMap<(usize, usize), usize> = HashMap::new();

        for &[a, b, c] in tris.iter() {
            for (u, v) in [(a, b), (b, c), (c, a)] {
                let key = if u < v { (u, v) } else { (v, u) };
                if split_map.contains_key(&key) {
                    continue;
                }
                let uva = verts[key.0];
                let uvb = verts[key.1];
                let uv_dist = (uvb[0] - uva[0]).hypot(uvb[1] - uva[1]);
                let uvm = [0.5 * (uva[0] + uvb[0]), 0.5 * (uva[1] + uvb[1])];
                let pm = srf.evaluate(uvm[0], uvm[1]);
                let chord_err = pm.distance(0.5 * (pos3d[key.0] + pos3d[key.1]));
                if (uv_dist > max_uv_step || chord_err > chord_tol) && verts.len() < 8000 {
                    let mid_idx = verts.len();
                    verts.push(uvm);
                    split_map.insert(key, mid_idx);
                }
            }
        }
        if split_map.is_empty() {
            break;
        }

        let mut next_tris = Vec::with_capacity(tris.len() * 2);
        let get_mid = |u: usize, v: usize| -> Option<usize> {
            let key = if u < v { (u, v) } else { (v, u) };
            split_map.get(&key).copied()
        };

        for &[v0, v1, v2] in tris.iter() {
            let m01 = get_mid(v0, v1);
            let m12 = get_mid(v1, v2);
            let m20 = get_mid(v2, v0);
            match (m01, m12, m20) {
                (Some(a), Some(b), Some(c)) => {
                    next_tris.push([v0, a, c]);
                    next_tris.push([a, v1, b]);
                    next_tris.push([c, b, v2]);
                    next_tris.push([a, b, c]);
                }
                (Some(a), Some(b), None) => {
                    next_tris.push([v0, a, v2]);
                    next_tris.push([a, b, v2]);
                    next_tris.push([a, v1, b]);
                }
                (None, Some(b), Some(c)) => {
                    next_tris.push([v0, v1, b]);
                    next_tris.push([v0, b, c]);
                    next_tris.push([c, b, v2]);
                }
                (Some(a), None, Some(c)) => {
                    next_tris.push([v0, a, c]);
                    next_tris.push([a, v1, v2]);
                    next_tris.push([c, a, v2]);
                }
                (Some(a), None, None) => {
                    next_tris.push([v0, a, v2]);
                    next_tris.push([a, v1, v2]);
                }
                (None, Some(b), None) => {
                    next_tris.push([v0, v1, b]);
                    next_tris.push([v0, b, v2]);
                }
                (None, None, Some(c)) => {
                    next_tris.push([v0, v1, c]);
                    next_tris.push([c, v1, v2]);
                }
                (None, None, None) => next_tris.push([v0, v1, v2]),
            }
        }
        *tris = next_tris;
    }
}

fn loop_uv_to_curves_3d(srf: &NurbsSurface, uv_pts: &[[f64; 2]]) -> Vec<NurbsCurve> {
    let n = uv_pts.len();
    if n < 2 {
        return Vec::new();
    }
    let curved_srf = srf.degree_u > 1 || srf.degree_v > 1;
    let pts3d: Vec<DVec3> = uv_pts.iter().map(|&[u, v]| srf.evaluate(u, v)).collect();

    // Detect sharp corners AND line-to-curve tangent transitions (e.g. Box edge into fillet arc)
    let mut corners = Vec::new();
    for i in 0..n {
        let prev = pts3d[(i + n - 1) % n];
        let curr = pts3d[i];
        let next = pts3d[(i + 1) % n];
        let v0 = curr - prev;
        let v1 = next - curr;
        let l0 = v0.length();
        let l1 = v1.length();
        let d0 = if l0 > 1e-12 { v0 / l0 } else { DVec3::ZERO };
        let d1 = if l1 > 1e-12 { v1 / l1 } else { DVec3::ZERO };
        let ang = d0.angle_between(d1);
        let prev2 = pts3d[(i + n - 2) % n];
        let next2 = pts3d[(i + 2) % n];
        let l_prev2 = prev.distance(prev2);
        let l_next2 = next.distance(next2);
        let a_prev = (prev - prev2).normalize_or_zero().angle_between(d0);
        let a_next = d1.angle_between((next2 - next).normalize_or_zero());
        let prev_straight_or_corner =
            a_prev < 0.5_f64.to_radians() || a_prev > 20.0_f64.to_radians();
        let next_straight_or_corner =
            a_next < 0.5_f64.to_radians() || a_next > 20.0_f64.to_radians();
        let line_to_arc = (l0 > l1 * 5.0
            && l_next2 < l0 * 0.35
            && prev_straight_or_corner
            && ang > 1.5_f64.to_radians())
            || (l1 > l0 * 5.0
                && l_prev2 < l1 * 0.35
                && next_straight_or_corner
                && ang > 1.5_f64.to_radians());
        if d0.length_squared() < 1e-12
            || d1.length_squared() < 1e-12
            || ang > 20.0_f64.to_radians()
            || line_to_arc
        {
            corners.push(i);
        }
    }

    if corners.is_empty() {
        let mut closed = downsample_points(&pts3d, 24);
        if closed.len() >= 4 {
            if let Ok(c) = NurbsCurve::interpolate(&closed, 3, true) {
                return vec![c];
            }
        }
        closed.push(closed[0]);
        return NurbsCurve::polyline(&closed).into_iter().collect();
    }

    let mut out = Vec::with_capacity(corners.len());
    for ci in 0..corners.len() {
        let start = corners[ci];
        let end = corners[(ci + 1) % corners.len()];
        let mut chain_pts = Vec::new();
        let mut chain_uv = Vec::new();
        let mut cur = start;
        loop {
            chain_pts.push(pts3d[cur]);
            chain_uv.push(uv_pts[cur]);
            if cur == end && chain_pts.len() >= 2 {
                break;
            }
            cur = (cur + 1) % n;
            if cur == start && chain_pts.len() > 1 {
                chain_pts.push(pts3d[end]);
                chain_uv.push(uv_pts[end]);
                break;
            }
        }
        if !curved_srf && chain_pts.len() == 2 {
            out.push(NurbsCurve::line(chain_pts[0], chain_pts[1]));
            continue;
        }
        if !curved_srf && points_collinear(&chain_pts, 1e-4) {
            out.push(NurbsCurve::line(chain_pts[0], *chain_pts.last().unwrap()));
            continue;
        }
        if curved_srf && chain_uv.len() == 2 {
            // Resample along curved surface UV edge cleanly
            let ua = chain_uv[0];
            let ub = chain_uv[1];
            let mut rich = Vec::with_capacity(25);
            for k in 0..=24 {
                let t = k as f64 / 24.0;
                rich.push(srf.evaluate(ua[0] + (ub[0] - ua[0]) * t, ua[1] + (ub[1] - ua[1]) * t));
            }
            chain_pts = rich;
        }
        if chain_pts.len() >= 6 && chain_pts[0].distance(*chain_pts.last().unwrap()) <= 1e-4 {
            let mut open = downsample_points(&chain_pts, 25);
            open.pop();
            if open.len() >= 4 {
                if let Ok(c) = NurbsCurve::interpolate(&open, 3, true) {
                    out.push(c);
                    continue;
                }
            }
        }
        let sub = downsample_points(&chain_pts, 16);
        if points_collinear(&sub, 1e-4) {
            out.push(NurbsCurve::line(sub[0], *sub.last().unwrap()));
        } else if sub.len() >= 4 {
            if let Ok(c) = NurbsCurve::interpolate(&sub, 3, false) {
                out.push(c);
                continue;
            }
            if let Ok(c) = NurbsCurve::polyline(&sub) {
                out.push(c);
            }
        } else if let Ok(c) = NurbsCurve::polyline(&sub) {
            out.push(c);
        }
    }
    out
}

fn downsample_points(pts: &[DVec3], max_pts: usize) -> Vec<DVec3> {
    if pts.len() <= max_pts {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(max_pts);
    for i in 0..max_pts {
        let idx = (i * (pts.len() - 1)) / (max_pts - 1);
        out.push(pts[idx]);
    }
    out
}

fn points_collinear(pts: &[DVec3], tol: f64) -> bool {
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

fn curves_match_topologically(a: &NurbsCurve, b: &NurbsCurve, tol: f64) -> bool {
    let (ua0, ua1) = a.domain();
    let (ub0, ub1) = b.domain();
    let a_start = a.evaluate(ua0);
    let a_end = a.evaluate(ua1);
    let b_start = b.evaluate(ub0);
    let b_end = b.evaluate(ub1);

    let a_closed = a_start.distance(a_end) <= tol;
    let b_closed = b_start.distance(b_end) <= tol;
    if a_closed && b_closed {
        for k in 0..4 {
            let ta = ua0 + (ua1 - ua0) * (k as f64 / 4.0);
            let tb = ub0 + (ub1 - ub0) * (k as f64 / 4.0);
            if b.closest_point(a.evaluate(ta)).2 > tol || a.closest_point(b.evaluate(tb)).2 > tol {
                return false;
            }
        }
        return true;
    }

    let direct = (0..=4).all(|k| {
        let t = k as f64 / 4.0;
        a.evaluate(ua0 + (ua1 - ua0) * t)
            .distance(b.evaluate(ub0 + (ub1 - ub0) * t))
            <= tol
    });
    if direct {
        return true;
    }
    (0..=4).all(|k| {
        let t = k as f64 / 4.0;
        a.evaluate(ua0 + (ua1 - ua0) * t)
            .distance(b.evaluate(ub1 - (ub1 - ub0) * t))
            <= tol
    })
}

pub(crate) fn fit_plane_from_loop(pts: &[DVec3]) -> Option<Plane> {
    if pts.len() < 3 {
        return None;
    }
    let mut normal = DVec3::ZERO;
    let mut centroid = DVec3::ZERO;
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        centroid += a;
    }
    if normal.length() < 1e-8 {
        return None;
    }
    let z_axis = normal.normalize();
    let origin = centroid / pts.len() as f64;
    let x_axis = (pts[1] - pts[0]).normalize_or_zero();
    if x_axis.length_squared() < 1e-12 {
        return None;
    }
    let y_axis = z_axis.cross(x_axis).normalize_or_zero();
    Some(Plane {
        origin,
        x_axis,
        y_axis,
        z_axis,
    })
}

pub(crate) fn intersect_three_planes(
    p0: DVec3,
    n0: DVec3,
    p1: DVec3,
    n1: DVec3,
    p2: DVec3,
    n2: DVec3,
) -> Option<DVec3> {
    let mat = vec![
        vec![n0.x, n0.y, n0.z],
        vec![n1.x, n1.y, n1.z],
        vec![n2.x, n2.y, n2.z],
    ];
    let rhs = vec![n0.dot(p0), n1.dot(p1), n2.dot(p2)];
    let sol = solve_linear_system(&mat, &rhs)?;
    Some(DVec3::new(sol[0], sol[1], sol[2]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brep_box_and_solid_primitives_have_manifold_topology_and_exact_volume() {
        let b = BRep::from_box(DVec3::new(0.0, 0.0, 1.0), DVec3::new(2.0, 4.0, 6.0));
        assert_eq!(b.faces.len(), 6);
        assert_eq!(b.edges.len(), 12);
        assert_eq!(b.vertices.len(), 8);
        assert!(b.is_solid());
        assert!(b.naked_edges().is_empty());
        assert!((b.area_approx() - 88.0).abs() < 1e-6);
        assert!((b.volume_approx() - 48.0).abs() < 1e-6);
        assert!(b.contains_point(DVec3::new(0.0, 0.0, 1.0)));
        assert!(!b.contains_point(DVec3::new(5.0, 0.0, 1.0)));

        let cyl = BRep::from_cylinder(DVec3::ZERO, 2.0, 5.0).unwrap();
        assert!(cyl.is_solid(), "naked edges: {:?}", cyl.naked_edges());
        let expected_cyl_vol = std::f64::consts::PI * 4.0 * 5.0;
        assert!((cyl.volume_approx() - expected_cyl_vol).abs() < 0.8);
        assert!(cyl.contains_point(DVec3::new(0.5, 0.5, 2.5)));
        assert!(!cyl.contains_point(DVec3::new(3.0, 0.0, 2.5)));

        let cone = BRep::from_cone(DVec3::ZERO, 2.0, 3.0).unwrap();
        assert!(cone.is_solid(), "cone naked edges: {:?}", cone.naked_edges());

        let sph = BRep::from_sphere(DVec3::ZERO, 3.0).unwrap();
        assert!((sph.area_approx() - 4.0 * std::f64::consts::PI * 9.0).abs() < 1.5);

        let tor = BRep::from_torus(DVec3::ZERO, 4.0, 1.0).unwrap();
        assert!(tor.area_approx() > 70.0);

        let rect =
            NurbsCurve::rectangle(&Plane::world_xy(), DVec3::ZERO, DVec3::new(3.0, 2.0, 0.0))
                .unwrap();
        let ext = BRep::extrude_curve_solid(&rect, DVec3::new(0.0, 0.0, 4.0)).unwrap();
        assert!(ext.is_solid());
        assert!((ext.volume_approx() - 24.0).abs() < 1e-5);
    }

    #[test]
    fn planar_trimmed_face_with_hole_join_explode_and_subobject_pick() {
        let outer = NurbsCurve::rectangle(
            &Plane::world_xy(),
            DVec3::new(-4.0, -4.0, 0.0),
            DVec3::new(4.0, 4.0, 0.0),
        )
        .unwrap();
        let hole = NurbsCurve::circle(&Plane::world_xy(), 1.5).unwrap();
        let plate = BRep::from_planar_curves(&[outer, hole]).unwrap();
        assert_eq!(plate.faces.len(), 1);
        assert_eq!(plate.faces[0].inner_loops.len(), 1);
        assert!(!plate.faces[0].contains_uv(0.5, 0.5));
        assert!(plate.faces[0].contains_uv(0.15, 0.15));
        let expected_area = 64.0 - std::f64::consts::PI * 2.25;
        assert!((plate.area_approx() - expected_area).abs() < 0.25);

        // Explode + Join round-trip recovers watertight solid Box
        let b = BRep::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let parts = b.explode();
        assert_eq!(parts.len(), 6);
        let rejoined = BRep::join_many(&parts, 1e-4);
        assert!(rejoined.is_solid());
        assert_eq!(rejoined.faces.len(), 6);

        // Sub-object ray picking hits both Edge and Face
        let edge_ray = Ray::new(DVec3::new(1.0, 0.0, 5.0), DVec3::new(0.0, 0.0, -1.0));
        assert!(matches!(
            b.pick_subobject(&edge_ray, 0.15),
            Some((SubObjectRef::Edge(_), _))
        ));
        let face_ray = Ray::new(DVec3::new(0.0, 0.0, 5.0), DVec3::new(0.0, 0.0, -1.0));
        assert!(matches!(
            b.pick_subobject(&face_ray, 0.15),
            Some((SubObjectRef::Face(_), _))
        ));
    }
}
