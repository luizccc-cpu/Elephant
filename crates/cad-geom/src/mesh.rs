use cad_math::{Aabb, DMat4, DVec3, PARAM_EPSILON, Ray};
use serde::{Deserialize, Serialize};

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
}
