use cad_math::{Aabb, DMat4, DVec3, PARAM_EPSILON, Plane, Ray};
use serde::{Deserialize, Serialize};

use crate::NurbsCurve;

/// Indexed double-precision triangle mesh for CAD solids, primitives, and display tessellation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TriangleMesh {
    pub positions: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub indices: Vec<[u32; 3]>,
}

impl TriangleMesh {
    pub fn from_box(center: DVec3, size: DVec3) -> Self {
        let h = size * 0.5;
        let faces = [
            // +Z, -Z, +X, -X, +Y, -Y
            ([0.0, 0.0, 1.0], [[-h.x, -h.y, h.z], [h.x, -h.y, h.z], [h.x, h.y, h.z], [-h.x, h.y, h.z]]),
            ([0.0, 0.0, -1.0], [[h.x, -h.y, -h.z], [-h.x, -h.y, -h.z], [-h.x, h.y, -h.z], [h.x, h.y, -h.z]]),
            ([1.0, 0.0, 0.0], [[h.x, -h.y, h.z], [h.x, -h.y, -h.z], [h.x, h.y, -h.z], [h.x, h.y, h.z]]),
            ([-1.0, 0.0, 0.0], [[-h.x, -h.y, -h.z], [-h.x, -h.y, h.z], [-h.x, h.y, h.z], [-h.x, h.y, -h.z]]),
            ([0.0, 1.0, 0.0], [[-h.x, h.y, h.z], [h.x, h.y, h.z], [h.x, h.y, -h.z], [-h.x, h.y, -h.z]]),
            ([0.0, -1.0, 0.0], [[-h.x, -h.y, -h.z], [h.x, -h.y, -h.z], [h.x, -h.y, h.z], [-h.x, -h.y, h.z]]),
        ];
        let mut positions = Vec::with_capacity(24);
        let mut normals = Vec::with_capacity(24);
        let mut indices = Vec::with_capacity(12);
        for (n, corners) in faces {
            let base = positions.len() as u32;
            for c in corners {
                positions.push([center.x + c[0], center.y + c[1], center.z + c[2]]);
                normals.push(n);
            }
            indices.push([base, base + 1, base + 2]);
            indices.push([base, base + 2, base + 3]);
        }
        Self {
            positions,
            normals,
            indices,
        }
    }

    pub fn from_sphere(center: DVec3, radius: f64, slices: usize, stacks: usize) -> Self {
        let slices = slices.max(4);
        let stacks = stacks.max(3);
        let mut positions = Vec::with_capacity((stacks + 1) * (slices + 1));
        let mut normals = Vec::with_capacity((stacks + 1) * (slices + 1));

        for i in 0..=stacks {
            let phi = std::f64::consts::PI * (i as f64 / stacks as f64);
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();
            for j in 0..=slices {
                let theta = std::f64::consts::TAU * (j as f64 / slices as f64);
                let n = DVec3::new(sin_phi * theta.cos(), sin_phi * theta.sin(), cos_phi);
                let p = center + n * radius;
                positions.push(p.to_array());
                normals.push(n.to_array());
            }
        }

        let mut indices = Vec::with_capacity(stacks * slices * 2);
        let stride = slices + 1;
        for i in 0..stacks {
            for j in 0..slices {
                let a = (i * stride + j) as u32;
                let b = ((i + 1) * stride + j) as u32;
                let c = ((i + 1) * stride + j + 1) as u32;
                let d = (i * stride + j + 1) as u32;
                if i != 0 {
                    indices.push([a, b, d]);
                }
                if i + 1 != stacks {
                    indices.push([b, c, d]);
                }
            }
        }

        Self {
            positions,
            normals,
            indices,
        }
    }

    pub fn bounding_box(&self) -> Aabb {
        Aabb::from_points(self.positions.iter().map(|&p| DVec3::from_array(p)))
    }

    pub fn transform(&mut self, mat: &DMat4) {
        let normal_mat = mat.inverse().transpose();
        for p in &mut self.positions {
            let v = mat.transform_point3(DVec3::from_array(*p));
            *p = v.to_array();
        }
        for n in &mut self.normals {
            let v = normal_mat.transform_vector3(DVec3::from_array(*n)).normalize_or_zero();
            *n = v.to_array();
        }
    }

    /// Möller–Trumbore ray-triangle intersection returning closest hit distance `t >= 0`.
    pub fn ray_intersect(&self, ray: &Ray) -> Option<f64> {
        let mut best: Option<f64> = None;
        for &[i0, i1, i2] in &self.indices {
            let v0 = DVec3::from_array(self.positions[i0 as usize]);
            let v1 = DVec3::from_array(self.positions[i1 as usize]);
            let v2 = DVec3::from_array(self.positions[i2 as usize]);

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
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
            let v = f * ray.direction.dot(q);
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let t = f * e2.dot(q);
            if t >= 0.0 && best.is_none_or(|b| t < b) {
                best = Some(t);
            }
        }
        best
    }

    /// Watertight solid cylinder centered at `base_center` extending `+height` along Z.
    pub fn from_cylinder(base_center: DVec3, radius: f64, height: f64, segments: usize) -> Self {
        let segs = segments.max(6);
        let r = radius.abs().max(1e-4);
        let h = height;
        let z0 = base_center.z;
        let z1 = base_center.z + h;
        let top_n = if h >= 0.0 { [0.0, 0.0, 1.0] } else { [0.0, 0.0, -1.0] };
        let bot_n = [0.0, 0.0, -top_n[2]];

        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        // Side wall
        let side_base = positions.len() as u32;
        for i in 0..=segs {
            let a = std::f64::consts::TAU * (i as f64 / segs as f64);
            let nx = a.cos();
            let ny = a.sin();
            positions.push([base_center.x + r * nx, base_center.y + r * ny, z0]);
            normals.push([nx, ny, 0.0]);
            positions.push([base_center.x + r * nx, base_center.y + r * ny, z1]);
            normals.push([nx, ny, 0.0]);
        }
        for i in 0..segs as u32 {
            let a = side_base + i * 2;
            let b = a + 1;
            let c = a + 2;
            let d = a + 3;
            if h >= 0.0 {
                indices.push([a, c, d]);
                indices.push([a, d, b]);
            } else {
                indices.push([a, d, c]);
                indices.push([a, b, d]);
            }
        }

        // Bottom cap
        let bot_center_idx = positions.len() as u32;
        positions.push([base_center.x, base_center.y, z0]);
        normals.push(bot_n);
        let bot_ring = positions.len() as u32;
        for i in 0..=segs {
            let a = std::f64::consts::TAU * (i as f64 / segs as f64);
            positions.push([base_center.x + r * a.cos(), base_center.y + r * a.sin(), z0]);
            normals.push(bot_n);
        }
        for i in 0..segs as u32 {
            if h >= 0.0 {
                indices.push([bot_center_idx, bot_ring + i + 1, bot_ring + i]);
            } else {
                indices.push([bot_center_idx, bot_ring + i, bot_ring + i + 1]);
            }
        }

        // Top cap
        let top_center_idx = positions.len() as u32;
        positions.push([base_center.x, base_center.y, z1]);
        normals.push(top_n);
        let top_ring = positions.len() as u32;
        for i in 0..=segs {
            let a = std::f64::consts::TAU * (i as f64 / segs as f64);
            positions.push([base_center.x + r * a.cos(), base_center.y + r * a.sin(), z1]);
            normals.push(top_n);
        }
        for i in 0..segs as u32 {
            if h >= 0.0 {
                indices.push([top_center_idx, top_ring + i, top_ring + i + 1]);
            } else {
                indices.push([top_center_idx, top_ring + i + 1, top_ring + i]);
            }
        }

        Self {
            positions,
            normals,
            indices,
        }
    }

    /// Watertight solid cone with base centered at `base_center` and apex at `base_center + (0,0,height)`.
    pub fn from_cone(base_center: DVec3, radius: f64, height: f64, segments: usize) -> Self {
        let segs = segments.max(6);
        let r = radius.abs().max(1e-4);
        let z0 = base_center.z;
        let z1 = base_center.z + height;
        let slope_z = r / height.abs().max(1e-4);

        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        for i in 0..segs {
            let a0 = std::f64::consts::TAU * (i as f64 / segs as f64);
            let a1 = std::f64::consts::TAU * ((i + 1) as f64 / segs as f64);
            let amid = 0.5 * (a0 + a1);
            let n0 = DVec3::new(a0.cos(), a0.sin(), slope_z).normalize().to_array();
            let n1 = DVec3::new(a1.cos(), a1.sin(), slope_z).normalize().to_array();
            let nm = DVec3::new(amid.cos(), amid.sin(), slope_z).normalize().to_array();
            let base = positions.len() as u32;
            positions.push([base_center.x + r * a0.cos(), base_center.y + r * a0.sin(), z0]);
            normals.push(n0);
            positions.push([base_center.x + r * a1.cos(), base_center.y + r * a1.sin(), z0]);
            normals.push(n1);
            positions.push([base_center.x, base_center.y, z1]);
            normals.push(nm);
            if height >= 0.0 {
                indices.push([base, base + 1, base + 2]);
            } else {
                indices.push([base, base + 2, base + 1]);
            }
        }

        let bot_n = if height >= 0.0 { [0.0, 0.0, -1.0] } else { [0.0, 0.0, 1.0] };
        let bot_center = positions.len() as u32;
        positions.push([base_center.x, base_center.y, z0]);
        normals.push(bot_n);
        let ring = positions.len() as u32;
        for i in 0..=segs {
            let a = std::f64::consts::TAU * (i as f64 / segs as f64);
            positions.push([base_center.x + r * a.cos(), base_center.y + r * a.sin(), z0]);
            normals.push(bot_n);
        }
        for i in 0..segs as u32 {
            if height >= 0.0 {
                indices.push([bot_center, ring + i + 1, ring + i]);
            } else {
                indices.push([bot_center, ring + i, ring + i + 1]);
            }
        }

        Self {
            positions,
            normals,
            indices,
        }
    }

    /// Watertight torus in XY plane centered at `center`.
    pub fn from_torus(
        center: DVec3,
        major_radius: f64,
        minor_radius: f64,
        major_segs: usize,
        minor_segs: usize,
    ) -> Self {
        let u_segs = major_segs.max(8);
        let v_segs = minor_segs.max(6);
        let maj = major_radius.abs().max(1e-3);
        let min = minor_radius.abs().max(1e-4);

        let mut positions = Vec::with_capacity((u_segs + 1) * (v_segs + 1));
        let mut normals = Vec::with_capacity((u_segs + 1) * (v_segs + 1));
        let mut indices = Vec::with_capacity(u_segs * v_segs * 2);

        for i in 0..=u_segs {
            let u = std::f64::consts::TAU * (i as f64 / u_segs as f64);
            let cu = u.cos();
            let su = u.sin();
            for j in 0..=v_segs {
                let v = std::f64::consts::TAU * (j as f64 / v_segs as f64);
                let cv = v.cos();
                let sv = v.sin();
                let n = DVec3::new(cu * cv, su * cv, sv);
                let p = center + DVec3::new((maj + min * cv) * cu, (maj + min * cv) * su, min * sv);
                positions.push(p.to_array());
                normals.push(n.to_array());
            }
        }

        let stride = (v_segs + 1) as u32;
        for i in 0..u_segs as u32 {
            for j in 0..v_segs as u32 {
                let a = i * stride + j;
                let b = (i + 1) * stride + j;
                let c = (i + 1) * stride + j + 1;
                let d = i * stride + j + 1;
                indices.push([a, b, c]);
                indices.push([a, c, d]);
            }
        }

        Self {
            positions,
            normals,
            indices,
        }
    }

    /// Planar triangulated mesh from a closed planar NURBS curve.
    pub fn from_planar_curve(curve: &NurbsCurve) -> Option<Self> {
        let (pts, plane) = extract_closed_planar_loop(curve)?;
        let tris = triangulate_polygon_2d(&pts, &plane);
        if tris.is_empty() {
            return None;
        }
        let n = plane.z_axis.to_array();
        let positions: Vec<[f64; 3]> = pts.iter().map(|p| p.to_array()).collect();
        let normals = vec![n; positions.len()];
        let indices: Vec<[u32; 3]> = tris
            .into_iter()
            .map(|[a, b, c]| [a as u32, b as u32, c as u32])
            .collect();
        Some(Self {
            positions,
            normals,
            indices,
        })
    }

    /// Watertight solid extrusion of a closed planar curve along `direction` with end caps.
    pub fn extrude_closed_curve(curve: &NurbsCurve, direction: DVec3) -> Option<Self> {
        if direction.length() < 1e-6 {
            return None;
        }
        let (mut pts, plane) = extract_closed_planar_loop(curve)?;
        // Ensure CCW order in `plane`
        if signed_area_2d(&pts, &plane) < 0.0 {
            pts.reverse();
        }
        let cap_tris = triangulate_polygon_2d(&pts, &plane);
        if cap_tris.is_empty() {
            return None;
        }
        let forward = direction.dot(plane.z_axis) >= 0.0;
        let n_pts = pts.len();
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut indices = Vec::new();

        // Bottom cap
        let bot_n = if forward { -plane.z_axis } else { plane.z_axis }.to_array();
        let bot_base = positions.len() as u32;
        for p in &pts {
            positions.push(p.to_array());
            normals.push(bot_n);
        }
        for [a, b, c] in &cap_tris {
            if forward {
                indices.push([bot_base + *a as u32, bot_base + *c as u32, bot_base + *b as u32]);
            } else {
                indices.push([bot_base + *a as u32, bot_base + *b as u32, bot_base + *c as u32]);
            }
        }

        // Top cap
        let top_n = if forward { plane.z_axis } else { -plane.z_axis }.to_array();
        let top_base = positions.len() as u32;
        for p in &pts {
            positions.push((*p + direction).to_array());
            normals.push(top_n);
        }
        for [a, b, c] in &cap_tris {
            if forward {
                indices.push([top_base + *a as u32, top_base + *b as u32, top_base + *c as u32]);
            } else {
                indices.push([top_base + *a as u32, top_base + *c as u32, top_base + *b as u32]);
            }
        }

        // Side walls (CCW loop => outward normal is (p1 - p0) x plane.z_axis)
        for i in 0..n_pts {
            let p0 = pts[i];
            let p1 = pts[(i + 1) % n_pts];
            let edge = p1 - p0;
            let wall_n = edge.cross(plane.z_axis).normalize_or_zero().to_array();
            let base = positions.len() as u32;
            positions.push(p0.to_array());
            normals.push(wall_n);
            positions.push(p1.to_array());
            normals.push(wall_n);
            positions.push((p1 + direction).to_array());
            normals.push(wall_n);
            positions.push((p0 + direction).to_array());
            normals.push(wall_n);
            if forward {
                indices.push([base, base + 1, base + 2]);
                indices.push([base, base + 2, base + 3]);
            } else {
                indices.push([base, base + 2, base + 1]);
                indices.push([base, base + 3, base + 2]);
            }
        }

        Some(Self {
            positions,
            normals,
            indices,
        })
    }

    /// Loft a mesh surface through 2 or more curves.
    pub fn loft_curves(curves: &[NurbsCurve], steps_u: usize) -> Option<Self> {
        if curves.len() < 2 {
            return None;
        }
        let cols = steps_u.max(8) + 1;
        let rows = curves.len();
        let grid: Vec<Vec<DVec3>> = curves
            .iter()
            .map(|c| c.sample_polyline(cols - 1))
            .collect();

        let mut positions = Vec::with_capacity(rows * cols);
        let mut normals = vec![[0.0, 0.0, 1.0]; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let p = grid[r][c];
                let du = if c + 1 < cols {
                    grid[r][c + 1] - grid[r][c.saturating_sub(1)]
                } else {
                    grid[r][c] - grid[r][c - 1]
                };
                let dv = if r + 1 < rows && r > 0 {
                    grid[r + 1][c] - grid[r - 1][c]
                } else if r + 1 < rows {
                    grid[r + 1][c] - grid[r][c]
                } else {
                    grid[r][c] - grid[r - 1][c]
                };
                let n = du.cross(dv).normalize_or_zero();
                positions.push(p.to_array());
                if n.length_squared() > 1e-8 {
                    normals[r * cols + c] = n.to_array();
                }
            }
        }

        let mut indices = Vec::with_capacity((rows - 1) * (cols - 1) * 2);
        for r in 0..(rows - 1) {
            for c in 0..(cols - 1) {
                let a = (r * cols + c) as u32;
                let b = (r * cols + c + 1) as u32;
                let d = ((r + 1) * cols + c) as u32;
                let e = ((r + 1) * cols + c + 1) as u32;
                indices.push([a, b, e]);
                indices.push([a, e, d]);
            }
        }

        Some(Self {
            positions,
            normals,
            indices,
        })
    }

    /// Revolve a profile curve around `(axis_origin, axis_dir)` by `angle_rad`.
    pub fn revolve_curve(
        curve: &NurbsCurve,
        axis_origin: DVec3,
        axis_dir: DVec3,
        angle_rad: f64,
        steps_u: usize,
        steps_v: usize,
    ) -> Self {
        let dir = axis_dir.normalize_or_zero();
        let profile = if curve.degree == 1 {
            curve.control_points.iter().map(|c| c.position).collect::<Vec<_>>()
        } else {
            curve.sample_polyline(steps_u.max(16))
        };
        let rows = steps_v.max(12) + 1;
        let cols = profile.len();

        let mut grid = Vec::with_capacity(rows);
        for r in 0..rows {
            let a = angle_rad * (r as f64 / (rows - 1) as f64);
            let mat = DMat4::from_translation(axis_origin)
                * DMat4::from_axis_angle(dir, a)
                * DMat4::from_translation(-axis_origin);
            let ring: Vec<DVec3> = profile.iter().map(|&p| mat.transform_point3(p)).collect();
            grid.push(ring);
        }

        let mut positions = Vec::with_capacity(rows * cols);
        let mut normals = vec![[0.0, 0.0, 1.0]; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let p = grid[r][c];
                let du = if c + 1 < cols && c > 0 {
                    grid[r][c + 1] - grid[r][c - 1]
                } else if c + 1 < cols {
                    grid[r][c + 1] - grid[r][c]
                } else {
                    grid[r][c] - grid[r][c.saturating_sub(1)]
                };
                let dv = if r + 1 < rows && r > 0 {
                    grid[r + 1][c] - grid[r - 1][c]
                } else if r + 1 < rows {
                    grid[r + 1][c] - grid[r][c]
                } else {
                    grid[r][c] - grid[r - 1][c]
                };
                let n = du.cross(dv).normalize_or_zero();
                positions.push(p.to_array());
                if n.length_squared() > 1e-8 {
                    normals[r * cols + c] = n.to_array();
                }
            }
        }

        let mut indices = Vec::with_capacity((rows - 1) * (cols.saturating_sub(1)) * 2);
        for r in 0..(rows - 1) {
            for c in 0..cols.saturating_sub(1) {
                let a = (r * cols + c) as u32;
                let b = (r * cols + c + 1) as u32;
                let d = ((r + 1) * cols + c) as u32;
                let e = ((r + 1) * cols + c + 1) as u32;
                indices.push([a, b, e]);
                indices.push([a, e, d]);
            }
        }

        Self {
            positions,
            normals,
            indices,
        }
    }
}

fn extract_closed_planar_loop(curve: &NurbsCurve) -> Option<(Vec<DVec3>, Plane)> {
    if !curve.is_closed() {
        return None;
    }
    let mut pts: Vec<DVec3> = if curve.degree == 1 {
        curve.control_points.iter().map(|c| c.position).collect()
    } else {
        curve.sample_polyline(36)
    };
    if pts.len() >= 2 && pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-4 {
        pts.pop();
    }
    if pts.len() < 3 {
        return None;
    }
    // Newell's method for best-fit plane normal
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
    let y_axis = z_axis.cross(x_axis).normalize_or_zero();
    let plane = Plane {
        origin,
        x_axis,
        y_axis,
        z_axis,
    };
    Some((pts, plane))
}

fn signed_area_2d(pts: &[DVec3], plane: &Plane) -> f64 {
    let mut area = 0.0;
    for i in 0..pts.len() {
        let a = pts[i] - plane.origin;
        let b = pts[(i + 1) % pts.len()] - plane.origin;
        area += a.dot(plane.x_axis) * b.dot(plane.y_axis)
            - b.dot(plane.x_axis) * a.dot(plane.y_axis);
    }
    area * 0.5
}

fn triangulate_polygon_2d(pts: &[DVec3], plane: &Plane) -> Vec<[usize; 3]> {
    let mut poly2d: Vec<(f64, f64)> = pts
        .iter()
        .map(|&p| {
            let d = p - plane.origin;
            (d.dot(plane.x_axis), d.dot(plane.y_axis))
        })
        .collect();
    let n = poly2d.len();
    if n < 3 {
        return Vec::new();
    }
    let mut indices: Vec<usize> = (0..n).collect();
    if signed_area_2d(pts, plane) < 0.0 {
        indices.reverse();
    }

    let cross2 = |a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    };
    let point_in_tri = |p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)| -> bool {
        let c1 = cross2(a, b, p);
        let c2 = cross2(b, c, p);
        let c3 = cross2(c, a, p);
        c1 >= -1e-9 && c2 >= -1e-9 && c3 >= -1e-9
    };

    let mut tris = Vec::with_capacity(n - 2);
    let mut guard = 0;
    while indices.len() > 3 && guard < n * n {
        guard += 1;
        let m = indices.len();
        let mut clipped = false;
        for i in 0..m {
            let i_prev = indices[(i + m - 1) % m];
            let i_curr = indices[i];
            let i_next = indices[(i + 1) % m];
            let a = poly2d[i_prev];
            let b = poly2d[i_curr];
            let c = poly2d[i_next];
            if cross2(a, b, c) <= 1e-10 {
                continue;
            }
            let mut inside = false;
            for &other in &indices {
                if other == i_prev || other == i_curr || other == i_next {
                    continue;
                }
                if point_in_tri(poly2d[other], a, b, c) {
                    inside = true;
                    break;
                }
            }
            if !inside {
                tris.push([i_prev, i_curr, i_next]);
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
    let _ = &mut poly2d;
    tris
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn box_ray_hit() {
        let mesh = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        let t = mesh.ray_intersect(&ray).expect("should hit box front face");
        assert!((t - 9.0).abs() < 1e-9);
    }

    #[test]
    fn phase_d_cylinder_cone_torus_and_solid_extrude() {
        let cyl = TriangleMesh::from_cylinder(DVec3::ZERO, 2.0, 5.0, 24);
        assert!(!cyl.indices.is_empty());
        let cone = TriangleMesh::from_cone(DVec3::ZERO, 2.0, 4.0, 24);
        assert!(!cone.indices.is_empty());
        let torus = TriangleMesh::from_torus(DVec3::ZERO, 4.0, 1.0, 24, 12);
        assert!(!torus.indices.is_empty());

        let rect = NurbsCurve::rectangle(&Plane::world_xy(), DVec3::ZERO, DVec3::new(3.0, 2.0, 0.0)).unwrap();
        let solid = TriangleMesh::extrude_closed_curve(&rect, DVec3::new(0.0, 0.0, 4.0)).unwrap();
        let bb = solid.bounding_box();
        assert!((bb.max.z - 4.0).abs() < 1e-9);
    }
}
