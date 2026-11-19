use cad_math::{Aabb, DMat4, DVec3, DVec4, PARAM_EPSILON, Plane, Ray};
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

    /// Closest `(t, distance)` from `ray` (`t >= 0`) to a polyline approximation of this curve.
    pub fn ray_distance(&self, ray: &Ray, samples: usize) -> Option<(f64, f64)> {
        let pts: Vec<DVec3> = if self.degree == 1 {
            self.control_points.iter().map(|c| c.position).collect()
        } else {
            self.sample_polyline(samples.max(32))
        };
        if pts.len() < 2 {
            return pts.first().map(|&p| ray.distance_to_point(p));
        }
        let mut best: Option<(f64, f64)> = None;
        for w in pts.windows(2) {
            let (t, d) = ray.distance_to_segment(w[0], w[1]);
            if best.is_none_or(|b| d < b.1 || ((d - b.1).abs() < 1e-12 && t < b.0)) {
                best = Some((t, d));
            }
        }
        best
    }

    pub fn transform(&mut self, mat: &DMat4) {
        for cp in &mut self.control_points {
            cp.position = mat.transform_point3(cp.position);
        }
    }

    /// Construct a closed planar rectangle polyline from two opposite corners on `plane`.
    pub fn rectangle(plane: &Plane, corner_a: DVec3, corner_b: DVec3) -> Result<Self, GeomError> {
        let delta = corner_b - corner_a;
        let dx = delta.dot(plane.x_axis);
        let dy = delta.dot(plane.y_axis);
        if dx.abs() < 1e-6 || dy.abs() < 1e-6 {
            return Err(GeomError::Degenerate("rectangle must have non-zero width and height"));
        }
        let p0 = corner_a;
        let p1 = corner_a + plane.x_axis * dx;
        let p2 = corner_a + plane.x_axis * dx + plane.y_axis * dy;
        let p3 = corner_a + plane.y_axis * dy;
        Self::polyline(&[p0, p1, p2, p3, p0])
    }

    /// Exact 9-point rational quadratic NURBS ellipse on `plane` with semi-axes `rx`, `ry`.
    pub fn ellipse(plane: &Plane, rx: f64, ry: f64) -> Result<Self, GeomError> {
        if !rx.is_finite() || rx <= 0.0 || !ry.is_finite() || ry <= 0.0 {
            return Err(GeomError::Degenerate("ellipse radii must be positive"));
        }
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let x = plane.x_axis * rx;
        let y = plane.y_axis * ry;
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

    /// Regular closed `sides`-gon in `plane` centered at `center` with a vertex at `radius_pt`.
    pub fn polygon(
        plane: &Plane,
        center: DVec3,
        radius_pt: DVec3,
        sides: usize,
    ) -> Result<Self, GeomError> {
        if sides < 3 {
            return Err(GeomError::Degenerate("polygon requires at least 3 sides"));
        }
        let v = radius_pt - center;
        let vx = v.dot(plane.x_axis);
        let vy = v.dot(plane.y_axis);
        let r = (vx * vx + vy * vy).sqrt();
        if r < 1e-6 {
            return Err(GeomError::Degenerate("polygon radius must be positive"));
        }
        let base_angle = vy.atan2(vx);
        let mut pts = Vec::with_capacity(sides + 1);
        for i in 0..=sides {
            let a = base_angle + (i % sides) as f64 * (std::f64::consts::TAU / sides as f64);
            pts.push(center + (plane.x_axis * a.cos() + plane.y_axis * a.sin()) * r);
        }
        Self::polyline(&pts)
    }

    /// Exact rational quadratic NURBS circular arc on `plane` from `start_angle` sweeping `sweep_angle` (radians).
    pub fn arc_from_center(
        plane: &Plane,
        center: DVec3,
        radius: f64,
        start_angle: f64,
        sweep_angle: f64,
    ) -> Result<Self, GeomError> {
        if !radius.is_finite() || radius <= 0.0 || sweep_angle.abs() < 1e-6 {
            return Err(GeomError::Degenerate("arc radius and sweep must be non-zero"));
        }
        let narcs = (sweep_angle.abs() / std::f64::consts::FRAC_PI_2).ceil().clamp(1.0, 4.0) as usize;
        let dtheta = sweep_angle / narcs as f64;
        let w1 = (dtheta * 0.5).cos();

        let mut p0 = center
            + (plane.x_axis * start_angle.cos() + plane.y_axis * start_angle.sin()) * radius;
        let mut t0 = -plane.x_axis * start_angle.sin() + plane.y_axis * start_angle.cos();
        if sweep_angle < 0.0 {
            t0 = -t0;
        }

        let mut cps = Vec::with_capacity(2 * narcs + 1);
        cps.push(ControlPoint::new(p0, 1.0));

        for i in 1..=narcs {
            let a = start_angle + dtheta * i as f64;
            let p2 = center + (plane.x_axis * a.cos() + plane.y_axis * a.sin()) * radius;
            let mut t2 = -plane.x_axis * a.sin() + plane.y_axis * a.cos();
            if sweep_angle < 0.0 {
                t2 = -t2;
            }
            // Intersect tangent lines p0 + s*t0 and p2 + t*t2
            let denom = t0.x * t2.y - t0.y * t2.x;
            let p1 = if denom.abs() > 1e-9 || t0.cross(t2).length() > 1e-9 {
                let d = p2 - p0;
                let n = t0.cross(t2);
                let s = d.cross(t2).dot(n) / n.length_squared();
                p0 + t0 * s
            } else {
                (p0 + p2) * 0.5
            };
            cps.push(ControlPoint::new(p1, w1));
            cps.push(ControlPoint::new(p2, 1.0));
            p0 = p2;
            t0 = t2;
        }

        let mut knots = Vec::with_capacity(2 * narcs + 4);
        knots.extend_from_slice(&[0.0, 0.0, 0.0]);
        for i in 1..narcs {
            let k = i as f64 / narcs as f64;
            knots.push(k);
            knots.push(k);
        }
        knots.extend_from_slice(&[1.0, 1.0, 1.0]);
        Self::new(2, knots, cps)
    }

    /// Exact rational quadratic NURBS circular arc passing through `p0`, `p1`, and `p2`.
    pub fn arc_3pt(p0: DVec3, p1: DVec3, p2: DVec3) -> Result<Self, GeomError> {
        let a = p1 - p0;
        let b = p2 - p0;
        let normal = a.cross(b);
        let denom = 2.0 * normal.length_squared();
        if denom < 1e-10 {
            return Err(GeomError::Degenerate("3-point arc points are collinear"));
        }
        let z_axis = normal.normalize();
        let center = p0
            + (b * a.length_squared() - a * b.length_squared()).cross(normal) / denom;
        let radius = center.distance(p0);
        let x_axis = (p0 - center).normalize();
        let y_axis = z_axis.cross(x_axis).normalize();
        let plane = Plane {
            origin: center,
            x_axis,
            y_axis,
            z_axis,
        };

        let angle_of = |pt: DVec3| -> f64 {
            let v = pt - center;
            let mut ang = v.dot(y_axis).atan2(v.dot(x_axis));
            if ang < 0.0 {
                ang += std::f64::consts::TAU;
            }
            ang
        };
        let a1 = angle_of(p1);
        let mut a2 = angle_of(p2);
        if a2 < 1e-9 {
            a2 = std::f64::consts::TAU;
        }
        let sweep = if a1 <= a2 { a2 } else { a2 - std::f64::consts::TAU };
        Self::arc_from_center(&plane, center, radius, 0.0, sweep)
    }

    pub fn is_closed(&self) -> bool {
        let (u0, u1) = self.domain();
        self.evaluate(u0).distance(self.evaluate(u1)) < 1e-4
    }

    pub fn length_approx(&self, samples: usize) -> f64 {
        let pts = if self.degree == 1 {
            self.control_points.iter().map(|c| c.position).collect()
        } else {
            self.sample_polyline(samples.max(8))
        };
        pts.windows(2).map(|w| w[0].distance(w[1])).sum()
    }

    pub fn planar_area_approx(&self, plane: &Plane, samples: usize) -> f64 {
        if !self.is_closed() {
            return 0.0;
        }
        let pts = if self.degree == 1 {
            self.control_points.iter().map(|c| c.position).collect()
        } else {
            self.sample_polyline(samples.max(16))
        };
        let mut area2 = 0.0;
        for w in pts.windows(2) {
            let a = w[0] - plane.origin;
            let b = w[1] - plane.origin;
            area2 += a.dot(plane.x_axis) * b.dot(plane.y_axis)
                - b.dot(plane.x_axis) * a.dot(plane.y_axis);
        }
        0.5 * area2.abs()
    }

    /// Explode a degree-1 polyline into individual line segments.
    pub fn explode_segments(&self) -> Vec<NurbsCurve> {
        if self.degree == 1 && self.control_points.len() > 2 {
            self.control_points
                .windows(2)
                .filter(|w| w[0].position.distance(w[1].position) > 1e-9)
                .map(|w| Self::line(w[0].position, w[1].position))
                .collect()
        } else {
            vec![self.clone()]
        }
    }

    /// Offset this curve in `plane` by `distance` toward the side of `side_pt`.
    pub fn offset_in_plane(
        &self,
        plane: &Plane,
        distance: f64,
        side_pt: DVec3,
    ) -> Result<Self, GeomError> {
        if !distance.is_finite() || distance.abs() < 1e-6 {
            return Err(GeomError::Degenerate("offset distance must be non-zero"));
        }
        let mut pts: Vec<DVec3> = if self.degree == 1 {
            self.control_points.iter().map(|c| c.position).collect()
        } else {
            self.sample_polyline(48)
        };
        if pts.len() < 2 {
            return Err(GeomError::Degenerate("curve too short to offset"));
        }
        let closed = pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-4;
        if closed {
            pts.pop();
        }
        if pts.len() < 2 {
            return Err(GeomError::Degenerate("curve too short to offset"));
        }

        // Find closest segment to `side_pt` to choose offset side sign
        let mut best_d = f64::INFINITY;
        let mut sign = 1.0_f64;
        let seg_count = if closed { pts.len() } else { pts.len() - 1 };
        for i in 0..seg_count {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            let ab = b - a;
            let len_sq = ab.length_squared();
            if len_sq < 1e-12 {
                continue;
            }
            let t = ((side_pt - a).dot(ab) / len_sq).clamp(0.0, 1.0);
            let proj = a + ab * t;
            let d = side_pt.distance(proj);
            if d < best_d {
                best_d = d;
                let left = plane.z_axis.cross(ab).normalize_or_zero();
                sign = if (side_pt - proj).dot(left) >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        let off = distance.abs() * sign;

        let seg_normal = |a: DVec3, b: DVec3| -> DVec3 {
            plane.z_axis.cross(b - a).normalize_or_zero()
        };

        let mut out = Vec::with_capacity(pts.len() + 1);
        for i in 0..pts.len() {
            let p = pts[i];
            let n = if closed {
                let prev = pts[(i + pts.len() - 1) % pts.len()];
                let next = pts[(i + 1) % pts.len()];
                let n0 = seg_normal(prev, p);
                let n1 = seg_normal(p, next);
                let bis = (n0 + n1).normalize_or_zero();
                let denom = bis.dot(n0);
                if denom.abs() > 0.25 {
                    bis * (off / denom)
                } else {
                    n1 * off
                }
            } else if i == 0 {
                seg_normal(p, pts[1]) * off
            } else if i + 1 == pts.len() {
                seg_normal(pts[i - 1], p) * off
            } else {
                let n0 = seg_normal(pts[i - 1], p);
                let n1 = seg_normal(p, pts[i + 1]);
                let bis = (n0 + n1).normalize_or_zero();
                let denom = bis.dot(n0);
                if denom.abs() > 0.25 {
                    bis * (off / denom)
                } else {
                    n1 * off
                }
            };
            out.push(p + n);
        }
        if closed {
            out.push(out[0]);
        }
        Self::polyline(&out)
    }

    /// Join contiguous curves whose endpoints match within `tol`.
    pub fn join_curves(curves: &[NurbsCurve], tol: f64) -> Vec<NurbsCurve> {
        let mut chains: Vec<Vec<DVec3>> = curves
            .iter()
            .map(|c| {
                if c.degree == 1 {
                    c.control_points.iter().map(|cp| cp.position).collect()
                } else {
                    c.sample_polyline(32)
                }
            })
            .filter(|pts| pts.len() >= 2)
            .collect();

        let mut result = Vec::new();
        while !chains.is_empty() {
            let mut cur = chains.remove(0);
            loop {
                let mut merged = false;
                let head = *cur.first().unwrap();
                let tail = *cur.last().unwrap();
                if head.distance(tail) <= tol && cur.len() > 2 {
                    break;
                }
                let mut i = 0;
                while i < chains.len() {
                    let c_head = *chains[i].first().unwrap();
                    let c_tail = *chains[i].last().unwrap();
                    if tail.distance(c_head) <= tol {
                        let mut next = chains.remove(i);
                        cur.extend(next.drain(1..));
                        merged = true;
                        break;
                    } else if tail.distance(c_tail) <= tol {
                        let mut next = chains.remove(i);
                        next.reverse();
                        cur.extend(next.drain(1..));
                        merged = true;
                        break;
                    } else if head.distance(c_tail) <= tol {
                        let mut next = chains.remove(i);
                        next.pop();
                        next.extend(cur);
                        cur = next;
                        merged = true;
                        break;
                    } else if head.distance(c_head) <= tol {
                        let mut next = chains.remove(i);
                        next.reverse();
                        next.pop();
                        next.extend(cur);
                        cur = next;
                        merged = true;
                        break;
                    }
                    i += 1;
                }
                if !merged {
                    break;
                }
            }
            if let Ok(poly) = Self::polyline(&cur) {
                result.push(poly);
            }
        }
        result
    }

    /// Compute trimmed lines + fillet arc between two lines `(a0,a1)` and `(b0,b1)` with `radius`.
    pub fn fillet_two_lines(
        a0: DVec3,
        a1: DVec3,
        b0: DVec3,
        b1: DVec3,
        radius: f64,
    ) -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
        if !radius.is_finite() || radius <= 1e-6 {
            return None;
        }
        let da = (a1 - a0).normalize_or_zero();
        let db = (b1 - b0).normalize_or_zero();
        let normal = da.cross(db);
        if normal.length() < 1e-6 {
            return None;
        }
        // Closest approach on the two infinite lines
        let r = a0 - b0;
        let a_dot_b = da.dot(db);
        let denom = 1.0 - a_dot_b * a_dot_b;
        if denom.abs() < 1e-8 {
            return None;
        }
        let s = (a_dot_b * db.dot(r) - da.dot(r)) / denom;
        let t = (db.dot(r) - a_dot_b * da.dot(r)) / denom;
        let pa = a0 + da * s;
        let pb = b0 + db * t;
        if pa.distance(pb) > 1e-2 {
            return None;
        }
        let corner = (pa + pb) * 0.5;

        let a_far = if a0.distance(corner) >= a1.distance(corner) { a0 } else { a1 };
        let b_far = if b0.distance(corner) >= b1.distance(corner) { b0 } else { b1 };
        let ua = (a_far - corner).normalize_or_zero();
        let ub = (b_far - corner).normalize_or_zero();
        let angle = ua.angle_between(ub);
        if angle < 1e-4 || (std::f64::consts::PI - angle).abs() < 1e-4 {
            return None;
        }
        let tan_dist = radius / (angle * 0.5).tan();
        if tan_dist >= a_far.distance(corner) || tan_dist >= b_far.distance(corner) {
            return None;
        }
        let ta = corner + ua * tan_dist;
        let tb = corner + ub * tan_dist;
        let bis = (ua + ub).normalize_or_zero();
        let center_dist = radius / (angle * 0.5).sin();
        let center = corner + bis * center_dist;
        let mid_arc = center - bis * radius;
        let arc = Self::arc_3pt(ta, mid_arc, tb).ok()?;
        Some((Self::line(a_far, ta), Self::line(b_far, tb), arc))
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

    #[test]
    fn phase_b_rectangle_arc_join_explode_and_fillet() {
        let plane = Plane::world_xy();
        let rect = NurbsCurve::rectangle(&plane, DVec3::ZERO, DVec3::new(4.0, 3.0, 0.0)).unwrap();
        assert!(rect.is_closed());
        assert!((rect.length_approx(16) - 14.0).abs() < 1e-9);
        assert!((rect.planar_area_approx(&plane, 16) - 12.0).abs() < 1e-9);

        let segs = rect.explode_segments();
        assert_eq!(segs.len(), 4);
        let joined = NurbsCurve::join_curves(&segs, 1e-5);
        assert_eq!(joined.len(), 1);
        assert!(joined[0].is_closed());

        let arc = NurbsCurve::arc_3pt(
            DVec3::new(5.0, 0.0, 0.0),
            DVec3::new(0.0, 5.0, 0.0),
            DVec3::new(-5.0, 0.0, 0.0),
        )
        .unwrap();
        assert!((arc.evaluate(0.5) - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-6);

        let (la, lb, fillet) = NurbsCurve::fillet_two_lines(
            DVec3::new(10.0, 0.0, 0.0),
            DVec3::ZERO,
            DVec3::ZERO,
            DVec3::new(0.0, 10.0, 0.0),
            2.0,
        )
        .unwrap();
        assert!((la.length_approx(4) - 8.0).abs() < 1e-6);
        assert!((lb.length_approx(4) - 8.0).abs() < 1e-6);
        assert!((fillet.length_approx(32) - std::f64::consts::PI).abs() < 1e-3);
    }

    #[test]
    fn curve_ray_distance_hits_perimeter_and_misses_center() {
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        let center_ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        let (_, dist_center) = circle.ray_distance(&center_ray, 48).unwrap();
        assert!((dist_center - 5.0).abs() < 0.05);

        let edge_ray = Ray::new(DVec3::new(5.05, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        let (t_edge, dist_edge) = circle.ray_distance(&edge_ray, 48).unwrap();
        assert!((t_edge - 10.0).abs() < 1e-6);
        assert!(dist_edge < 0.08);
    }
}
