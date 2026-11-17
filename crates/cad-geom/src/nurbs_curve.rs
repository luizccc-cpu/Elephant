use cad_math::{Aabb, DMat4, DVec3, DVec4, PARAM_EPSILON, Plane};
use serde::{Deserialize, Serialize};

use crate::error::GeomError;

/// Homogeneous-ready NURBS control point in 3D space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ControlPoint {
    pub position: DVec3,
    pub weight: f64,
}

impl ControlPoint {
    pub fn new(position: DVec3, weight: f64) -> Self {
        Self { position, weight }
    }

    pub fn unweighted(position: DVec3) -> Self {
        Self {
            position,
            weight: 1.0,
        }
    }

    #[inline]
    pub fn to_homogeneous(self) -> DVec4 {
        DVec4::new(
            self.position.x * self.weight,
            self.position.y * self.weight,
            self.position.z * self.weight,
            self.weight,
        )
    }
}

/// Clean-room rational/non-rational B-Spline (NURBS) curve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NurbsCurve {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub control_points: Vec<ControlPoint>,
}

impl NurbsCurve {
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        control_points: Vec<ControlPoint>,
    ) -> Result<Self, GeomError> {
        if degree == 0 {
            return Err(GeomError::InvalidDegree(degree));
        }
        let n = control_points.len();
        if n < degree + 1 {
            return Err(GeomError::InsufficientControlPoints {
                degree,
                found: n,
                needed: degree + 1,
            });
        }
        let expected_knots = n + degree + 1;
        if knots.len() != expected_knots {
            return Err(GeomError::InvalidKnotCount {
                found: knots.len(),
                expected: expected_knots,
            });
        }
        for pair in knots.windows(2) {
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
            degree,
            knots,
            control_points,
        })
    }

    /// Construct a degree-1 NURBS line segment.
    pub fn line(start: DVec3, end: DVec3) -> Self {
        Self {
            degree: 1,
            knots: vec![0.0, 0.0, 1.0, 1.0],
            control_points: vec![
                ControlPoint::unweighted(start),
                ControlPoint::unweighted(end),
            ],
        }
    }

    /// Construct a degree-1 NURBS polyline.
    pub fn polyline(points: &[DVec3]) -> Result<Self, GeomError> {
        if points.len() < 2 {
            return Err(GeomError::InsufficientControlPoints {
                degree: 1,
                found: points.len(),
                needed: 2,
            });
        }
        let n = points.len();
        let mut knots = Vec::with_capacity(n + 2);
        knots.push(0.0);
        for i in 0..n {
            knots.push(i as f64);
        }
        knots.push((n - 1) as f64);
        let control_points = points.iter().copied().map(ControlPoint::unweighted).collect();
        Self::new(1, knots, control_points)
    }

    /// Exact 9-point rational quadratic NURBS circle on `plane` with `radius`.
    pub fn circle(plane: &Plane, radius: f64) -> Result<Self, GeomError> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err(GeomError::Degenerate("circle radius must be positive"));
        }
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let x = plane.x_axis * radius;
        let y = plane.y_axis * radius;
        let o = plane.origin;

        let control_points = vec![
            ControlPoint::new(o + x, 1.0),
            ControlPoint::new(o + x + y, w),
            ControlPoint::new(o + y, 1.0),
            ControlPoint::new(o - x + y, w),
            ControlPoint::new(o - x, 1.0),
            ControlPoint::new(o - x - y, w),
            ControlPoint::new(o - y, 1.0),
            ControlPoint::new(o + x - y, w),
            ControlPoint::new(o + x, 1.0),
        ];
        let knots = vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ];
        Self::new(2, knots, control_points)
    }

    #[inline]
    pub fn domain(&self) -> (f64, f64) {
        let p = self.degree;
        let n = self.control_points.len();
        (self.knots[p], self.knots[n])
    }

    /// Evaluate the 3D point on the curve at parameter `u` (clamped to domain).
    pub fn evaluate(&self, u: f64) -> DVec3 {
        let (u_min, u_max) = self.domain();
        let u = u.clamp(u_min, u_max);
        let span = find_span(self.control_points.len() - 1, self.degree, u, &self.knots);
        let basis = basis_functions(span, u, self.degree, &self.knots);

        let mut cw = DVec4::ZERO;
        for (i, &b) in basis.iter().enumerate() {
            cw += self.control_points[span - self.degree + i].to_homogeneous() * b;
        }
        if cw.w.abs() < PARAM_EPSILON {
            DVec3::ZERO
        } else {
            cw.truncate() / cw.w
        }
    }

    /// Evaluate `(point, first_derivative)` at parameter `u` via the rational quotient rule.
    pub fn evaluate_derivative(&self, u: f64) -> (DVec3, DVec3) {
        let (u_min, u_max) = self.domain();
        let u = u.clamp(u_min, u_max);
        let span = find_span(self.control_points.len() - 1, self.degree, u, &self.knots);
        let ders = basis_function_derivatives(span, u, self.degree, 1, &self.knots);

        let mut cw0 = DVec4::ZERO;
        let mut cw1 = DVec4::ZERO;
        for i in 0..=self.degree {
            let h = self.control_points[span - self.degree + i].to_homogeneous();
            cw0 += h * ders[0][i];
            cw1 += h * ders[1][i];
        }
        let w = cw0.w;
        if w.abs() < PARAM_EPSILON {
            return (DVec3::ZERO, DVec3::ZERO);
        }
        let point = cw0.truncate() / w;
        let deriv = (cw1.truncate() - point * cw1.w) / w;
        (point, deriv)
    }

    /// Uniformly sample `segments` intervals into `segments + 1` points.
    pub fn sample_polyline(&self, segments: usize) -> Vec<DVec3> {
        let segments = segments.max(1);
        let (u0, u1) = self.domain();
        (0..=segments)
            .map(|i| {
                let t = i as f64 / segments as f64;
                self.evaluate(u0 + (u1 - u0) * t)
            })
            .collect()
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

/// Binary search for the knot span index `i` such that `knots[i] <= u < knots[i + 1]`.
pub(crate) fn find_span(n: usize, degree: usize, u: f64, knots: &[f64]) -> usize {
    if u >= knots[n + 1] - PARAM_EPSILON {
        return n;
    }
    if u <= knots[degree] + PARAM_EPSILON {
        return degree;
    }
    let mut low = degree;
    let mut high = n + 1;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// Compute non-vanishing B-spline basis functions `N_{i-p, p}(u) .. N_{i, p}(u)`.
pub(crate) fn basis_functions(span: usize, u: f64, degree: usize, knots: &[f64]) -> Vec<f64> {
    let mut n = vec![0.0; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    n[0] = 1.0;

    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom.abs() < PARAM_EPSILON {
                0.0
            } else {
                n[r] / denom
            };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Compute B-spline basis functions and derivatives up to `max_order <= degree`.
pub(crate) fn basis_function_derivatives(
    span: usize,
    u: f64,
    degree: usize,
    max_order: usize,
    knots: &[f64],
) -> Vec<Vec<f64>> {
    let max_order = max_order.min(degree);
    let mut ndu = vec![vec![0.0; degree + 1]; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    ndu[0][0] = 1.0;

    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r].abs() < PARAM_EPSILON {
                0.0
            } else {
                ndu[r][j - 1] / ndu[j][r]
            };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    let mut ders = vec![vec![0.0; degree + 1]; max_order + 1];
    for j in 0..=degree {
        ders[0][j] = ndu[j][degree];
    }

    let mut a = vec![vec![0.0; degree + 1]; 2];
    for r in 0..=degree {
        let mut s1 = 0;
        let mut s2 = 1;
        a[0][0] = 1.0;

        for k in 1..=max_order {
            let mut d = 0.0;
            let rk = r as isize - k as isize;
            let pk = (degree - k) as isize;
            if r >= k {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if (r as isize - 1) <= pk {
                k - 1
            } else {
                degree - r
            };
            for j in j1..=j2 {
                a[s2][j] = (a[s1][j] - a[s1][j - 1])
                    / ndu[(pk + 1) as usize][(rk + j as isize) as usize];
                d += a[s2][j] * ndu[(rk + j as isize) as usize][pk as usize];
            }
            if r as isize <= pk {
                a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                d += a[s2][k] * ndu[r][pk as usize];
            }
            ders[k][r] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }

    let mut factor = degree as f64;
    for k in 1..=max_order {
        for j in 0..=degree {
            ders[k][j] *= factor;
        }
        factor *= (degree - k) as f64;
    }

    ders
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nurbs_circle_lies_on_radius() {
        let plane = Plane::world_xy();
        let circle = NurbsCurve::circle(&plane, 5.0).expect("valid NURBS circle");
        for i in 0..=32 {
            let u = i as f64 / 32.0;
            let p = circle.evaluate(u);
            assert!((p.length() - 5.0).abs() < 1e-9, "point {p} off radius 5");
            assert!(p.z.abs() < 1e-12);
        }
    }

    #[test]
    fn line_derivative_matches_direction() {
        let line = NurbsCurve::line(DVec3::new(1.0, 2.0, 3.0), DVec3::new(4.0, 6.0, 3.0));
        let (p, d) = line.evaluate_derivative(0.5);
        assert!((p - DVec3::new(2.5, 4.0, 3.0)).length() < 1e-9);
        assert!((d - DVec3::new(3.0, 4.0, 0.0)).length() < 1e-9);
    }
}
