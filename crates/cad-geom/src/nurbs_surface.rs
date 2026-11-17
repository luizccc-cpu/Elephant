use cad_math::{Aabb, DMat4, DVec3, DVec4, PARAM_EPSILON, Plane};
use serde::{Deserialize, Serialize};

use crate::error::GeomError;
use crate::mesh::TriangleMesh;
use crate::nurbs_curve::{ControlPoint, NurbsCurve, basis_function_derivatives, basis_functions, find_span};

/// Clean-room tensor-product NURBS surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsSurface {
    pub degree_u: usize,
    pub degree_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub count_u: usize,
    pub count_v: usize,
    /// Row-major `u * count_v + v`.
    pub control_points: Vec<ControlPoint>,
}

impl NurbsSurface {
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        count_u: usize,
        count_v: usize,
        control_points: Vec<ControlPoint>,
    ) -> Result<Self, GeomError> {
        if degree_u == 0 {
            return Err(GeomError::InvalidDegree(degree_u));
        }
        if degree_v == 0 {
            return Err(GeomError::InvalidDegree(degree_v));
        }
        if count_u * count_v != control_points.len() {
            return Err(GeomError::InvalidGridDimensions {
                count_u,
                count_v,
                total: control_points.len(),
            });
        }
        if count_u < degree_u + 1 {
            return Err(GeomError::InsufficientControlPoints {
                degree: degree_u,
                found: count_u,
                needed: degree_u + 1,
            });
        }
        if count_v < degree_v + 1 {
            return Err(GeomError::InsufficientControlPoints {
                degree: degree_v,
                found: count_v,
                needed: degree_v + 1,
            });
        }
        if knots_u.len() != count_u + degree_u + 1 {
            return Err(GeomError::InvalidKnotCount {
                found: knots_u.len(),
                expected: count_u + degree_u + 1,
            });
        }
        if knots_v.len() != count_v + degree_v + 1 {
            return Err(GeomError::InvalidKnotCount {
                found: knots_v.len(),
                expected: count_v + degree_v + 1,
            });
        }
        for pair in knots_u.windows(2).chain(knots_v.windows(2)) {
            if pair[1] + PARAM_EPSILON < pair[0] {
                return Err(GeomError::NonMonotonicKnots);
            }
        }
        for cp in &control_points {
            if !cp.weight.is_finite() || cp.weight <= 0.0 {
                return Err(GeomError::NonPositiveWeight(cp.weight));
            }
        }
        Ok(Self {
            degree_u,
            degree_v,
            knots_u,
            knots_v,
            count_u,
            count_v,
            control_points,
        })
    }

    /// Construct a flat rectangular NURBS surface on `plane`.
    pub fn plane_rect(plane: &Plane, width: f64, height: f64) -> Self {
        let hx = plane.x_axis * (width * 0.5);
        let hy = plane.y_axis * (height * 0.5);
        let o = plane.origin;
        Self {
            degree_u: 1,
            degree_v: 1,
            knots_u: vec![0.0, 0.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            count_u: 2,
            count_v: 2,
            control_points: vec![
                ControlPoint::unweighted(o - hx - hy),
                ControlPoint::unweighted(o - hx + hy),
                ControlPoint::unweighted(o + hx - hy),
                ControlPoint::unweighted(o + hx + hy),
            ],
        }
    }

    /// Linearly extrude a NURBS curve along `direction` into a NURBS surface.
    pub fn extrude_curve(curve: &NurbsCurve, direction: DVec3) -> Self {
        let count_u = curve.control_points.len();
        let mut control_points = Vec::with_capacity(count_u * 2);
        for cp in &curve.control_points {
            control_points.push(*cp);
            control_points.push(ControlPoint::new(cp.position + direction, cp.weight));
        }
        Self {
            degree_u: curve.degree,
            degree_v: 1,
            knots_u: curve.knots.clone(),
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            count_u,
            count_v: 2,
            control_points,
        }
    }

    #[inline]
    pub fn domain_u(&self) -> (f64, f64) {
        (self.knots_u[self.degree_u], self.knots_u[self.count_u])
    }

    #[inline]
    pub fn domain_v(&self) -> (f64, f64) {
        (self.knots_v[self.degree_v], self.knots_v[self.count_v])
    }

    pub fn evaluate(&self, u: f64, v: f64) -> DVec3 {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);

        let span_u = find_span(self.count_u - 1, self.degree_u, u, &self.knots_u);
        let span_v = find_span(self.count_v - 1, self.degree_v, v, &self.knots_v);
        let nu = basis_functions(span_u, u, self.degree_u, &self.knots_u);
        let nv = basis_functions(span_v, v, self.degree_v, &self.knots_v);

        let mut sw = DVec4::ZERO;
        for (i, &bu) in nu.iter().enumerate() {
            let ui = span_u - self.degree_u + i;
            let mut temp = DVec4::ZERO;
            for (j, &bv) in nv.iter().enumerate() {
                let vj = span_v - self.degree_v + j;
                temp += self.control_points[ui * self.count_v + vj].to_homogeneous() * bv;
            }
            sw += temp * bu;
        }
        if sw.w.abs() < PARAM_EPSILON {
            DVec3::ZERO
        } else {
            sw.truncate() / sw.w
        }
    }

    /// Evaluate `(point, normal)` at `(u, v)`.
    pub fn evaluate_point_and_normal(&self, u: f64, v: f64) -> (DVec3, DVec3) {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);

        let span_u = find_span(self.count_u - 1, self.degree_u, u, &self.knots_u);
        let span_v = find_span(self.count_v - 1, self.degree_v, v, &self.knots_v);
        let du = basis_function_derivatives(span_u, u, self.degree_u, 1, &self.knots_u);
        let dv = basis_function_derivatives(span_v, v, self.degree_v, 1, &self.knots_v);

        let mut s00 = DVec4::ZERO;
        let mut s10 = DVec4::ZERO;
        let mut s01 = DVec4::ZERO;

        for i in 0..=self.degree_u {
            let ui = span_u - self.degree_u + i;
            let mut row0 = DVec4::ZERO;
            let mut row1 = DVec4::ZERO;
            for j in 0..=self.degree_v {
                let vj = span_v - self.degree_v + j;
                let h = self.control_points[ui * self.count_v + vj].to_homogeneous();
                row0 += h * dv[0][j];
                row1 += h * dv[1][j];
            }
            s00 += row0 * du[0][i];
            s10 += row0 * du[1][i];
            s01 += row1 * du[0][i];
        }

        let w = s00.w;
        if w.abs() < PARAM_EPSILON {
            return (DVec3::ZERO, DVec3::Z);
        }
        let point = s00.truncate() / w;
        let d_du = (s10.truncate() - point * s10.w) / w;
        let d_dv = (s01.truncate() - point * s01.w) / w;
        let normal = d_du.cross(d_dv).normalize_or_zero();
        (point, normal)
    }

    /// Tessellate the surface into a `TriangleMesh` with `steps_u x steps_v` quads.
    pub fn tessellate(&self, steps_u: usize, steps_v: usize) -> TriangleMesh {
        let su = steps_u.max(1);
        let sv = steps_v.max(1);
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();

        let mut positions = Vec::with_capacity((su + 1) * (sv + 1));
        let mut normals = Vec::with_capacity((su + 1) * (sv + 1));
        for iu in 0..=su {
            let u = u0 + (u1 - u0) * (iu as f64 / su as f64);
            for iv in 0..=sv {
                let v = v0 + (v1 - v0) * (iv as f64 / sv as f64);
                let (p, n) = self.evaluate_point_and_normal(u, v);
                positions.push(p.to_array());
                normals.push(n.to_array());
            }
        }

        let mut indices = Vec::with_capacity(su * sv * 2);
        let stride = sv + 1;
        for iu in 0..su {
            for iv in 0..sv {
                let a = (iu * stride + iv) as u32;
                let b = ((iu + 1) * stride + iv) as u32;
                let c = ((iu + 1) * stride + iv + 1) as u32;
                let d = (iu * stride + iv + 1) as u32;
                indices.push([a, b, c]);
                indices.push([a, c, d]);
            }
        }

        TriangleMesh {
            positions,
            normals,
            indices,
        }
    }

    pub fn bounding_box(&self) -> Aabb {
        Aabb::from_points(self.control_points.iter().map(|cp| cp.position))
    }

    pub fn transform(&mut self, mat: &DMat4) {
        for cp in &mut self.control_points {
            cp.position = mat.transform_point3(cp.position);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrude_circle_creates_cylinder_surface() {
        let circle = NurbsCurve::circle(&Plane::world_xy(), 2.0).unwrap();
        let cyl = NurbsSurface::extrude_curve(&circle, DVec3::new(0.0, 0.0, 5.0));
        let p = cyl.evaluate(0.25, 0.5);
        assert!((p - DVec3::new(0.0, 2.0, 2.5)).length() < 1e-9);
        let mesh = cyl.tessellate(16, 4);
        assert_eq!(mesh.indices.len(), 16 * 4 * 2);
    }
}
