use cad_math::linalg::solve_linear_system_vec3;
use cad_math::{Aabb, DMat4, DVec3, DVec4, PARAM_EPSILON, Plane, Ray};
use serde::{Deserialize, Serialize};

use crate::curve_ops::{OffsetCornerStyle, join_two_exact, offset_nurbs_curve_in_plane};
use crate::error::GeomError;

/// Curvature sample along a NURBS curve for analysis and CurvatureGraph display.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurvatureSample {
    pub u: f64,
    pub point: DVec3,
    pub tangent: DVec3,
    pub curvature_vec: DVec3,
    pub curvature: f64,
}

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

    /// Evaluate rational derivatives `C(u), C'(u), ..., C^(max_order)(u)` via Leibniz rule (A4.2).
    pub fn evaluate_derivatives(&self, u: f64, max_order: usize) -> Vec<DVec3> {
        let (u_min, u_max) = self.domain();
        let u = u.clamp(u_min, u_max);
        let d = max_order.min(self.degree);
        let span = find_span(self.control_points.len() - 1, self.degree, u, &self.knots);
        let ders = basis_function_derivatives(span, u, self.degree, d, &self.knots);

        let mut aders = vec![DVec3::ZERO; d + 1];
        let mut wders = vec![0.0_f64; d + 1];
        for k in 0..=d {
            let mut cw = DVec4::ZERO;
            for i in 0..=self.degree {
                let h = self.control_points[span - self.degree + i].to_homogeneous();
                cw += h * ders[k][i];
            }
            aders[k] = cw.truncate();
            wders[k] = cw.w;
        }

        let mut ck = vec![DVec3::ZERO; max_order + 1];
        if wders[0].abs() < PARAM_EPSILON {
            return ck;
        }
        for k in 0..=d {
            let mut v = aders[k];
            for i in 1..=k {
                let b = binomial_coeff(k, i);
                v -= ck[k - i] * (b * wders[i]);
            }
            ck[k] = v / wders[0];
        }
        ck
    }

    /// Evaluate `(curvature_vector, scalar_curvature)` at parameter `u`.
    pub fn curvature_at(&self, u: f64) -> (DVec3, f64) {
        let ders = self.evaluate_derivatives(u, 2);
        let c1 = ders[1];
        let c2 = ders[2];
        let s1 = c1.length();
        if s1 < 1e-12 {
            return (DVec3::ZERO, 0.0);
        }
        let t_hat = c1 / s1;
        let k_vec = (c2 - t_hat * c2.dot(t_hat)) / (s1 * s1);
        (k_vec, k_vec.length())
    }

    /// Orthonormal Frenet-like frame `(point, tangent, normal, binormal)` at parameter `u`.
    pub fn frenet_frame(&self, u: f64) -> (DVec3, DVec3, DVec3, DVec3) {
        let ders = self.evaluate_derivatives(u, 2);
        let p = ders[0];
        let t = ders[1].normalize_or_zero();
        if t.length_squared() < 1e-12 {
            return (p, DVec3::X, DVec3::Y, DVec3::Z);
        }
        let (k_vec, k) = self.curvature_at(u);
        let (n, b) = if k > 1e-9 {
            let n_raw = k_vec / k;
            let b = t.cross(n_raw).normalize_or_zero();
            if b.length_squared() > 1e-12 {
                (b.cross(t).normalize(), b)
            } else {
                fallback_frame_axes(t)
            }
        } else {
            fallback_frame_axes(t)
        };
        (p, t, n, b)
    }

    /// Sample curvature along the curve for analysis and CurvatureGraph rendering.
    pub fn curvature_samples(&self, samples: usize) -> Vec<CurvatureSample> {
        let n = samples.max(4);
        let (u0, u1) = self.domain();
        (0..=n)
            .map(|i| {
                let u = u0 + (u1 - u0) * (i as f64 / n as f64);
                let ders = self.evaluate_derivatives(u, 2);
                let c1 = ders[1];
                let c2 = ders[2];
                let s1 = c1.length();
                let tangent = if s1 > 1e-12 { c1 / s1 } else { DVec3::X };
                let curvature_vec = if s1 > 1e-12 {
                    (c2 - tangent * c2.dot(tangent)) / (s1 * s1)
                } else {
                    DVec3::ZERO
                };
                CurvatureSample {
                    u,
                    point: ders[0],
                    tangent,
                    curvature_vec,
                    curvature: curvature_vec.length(),
                }
            })
            .collect()
    }

    /// Arc length integrated over `[u_a, u_b]` via composite 5-point Gauss-Legendre quadrature.
    pub fn arc_length_between(&self, u_a: f64, u_b: f64) -> f64 {
        let (u0, u1) = self.domain();
        let a = u_a.clamp(u0, u1);
        let b = u_b.clamp(u0, u1);
        if b <= a + 1e-14 {
            return 0.0;
        }
        const GL_NODES: [f64; 5] = [
            -0.906179845938664,
            -0.5384693101056831,
            0.0,
            0.5384693101056831,
            0.906179845938664,
        ];
        const GL_WEIGHTS: [f64; 5] = [
            0.2369268850561891,
            0.4786286704993665,
            0.5688888888888889,
            0.4786286704993665,
            0.2369268850561891,
        ];

        let mut breaks = vec![a];
        for &k in &self.knots {
            if k > a + 1e-12 && k < b - 1e-12 && k - *breaks.last().unwrap() > 1e-12 {
                breaks.push(k);
            }
        }
        breaks.push(b);

        let sub_per_span = if self.degree == 1 { 1 } else { 4 };
        let mut total = 0.0;
        for w in breaks.windows(2) {
            let span_len = w[1] - w[0];
            for s in 0..sub_per_span {
                let s0 = w[0] + span_len * (s as f64 / sub_per_span as f64);
                let s1 = w[0] + span_len * ((s + 1) as f64 / sub_per_span as f64);
                let mid = 0.5 * (s0 + s1);
                let half = 0.5 * (s1 - s0);
                let mut sum = 0.0;
                for (&xi, &wi) in GL_NODES.iter().zip(GL_WEIGHTS.iter()) {
                    let (_, deriv) = self.evaluate_derivative(mid + half * xi);
                    sum += wi * deriv.length();
                }
                total += half * sum;
            }
        }
        total
    }

    pub fn arc_length(&self) -> f64 {
        let (u0, u1) = self.domain();
        self.arc_length_between(u0, u1)
    }

    /// Parameter `u` at `target_len` distance from `u0`.
    pub fn parameter_at_arc_length(&self, target_len: f64) -> f64 {
        let (u0, u1) = self.domain();
        let total = self.arc_length();
        if target_len <= 0.0 || total <= 1e-12 {
            return u0;
        }
        if target_len >= total {
            return u1;
        }
        let mut lo = u0;
        let mut hi = u1;
        for _ in 0..48 {
            let mid = 0.5 * (lo + hi);
            let len = self.arc_length_between(u0, mid);
            if (len - target_len).abs() < 1e-10 {
                return mid;
            }
            if len < target_len {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// Closest point on the curve to `query`, returning `(u, point_on_curve, distance)`.
    pub fn closest_point(&self, query: DVec3) -> (f64, DVec3, f64) {
        let (u0, u1) = self.domain();
        let seed_steps = (self.control_points.len() * 8).clamp(48, 256);
        let mut candidates = Vec::with_capacity(seed_steps + self.knots.len() + 2);
        for i in 0..=seed_steps {
            candidates.push(u0 + (u1 - u0) * (i as f64 / seed_steps as f64));
        }
        for &k in &self.knots {
            if k > u0 && k < u1 {
                candidates.push(k);
            }
        }
        candidates.sort_by(|a, b| a.total_cmp(b));
        candidates.dedup_by(|a, b| (*a - *b).abs() < 1e-11);

        let dists: Vec<(f64, f64)> = candidates
            .iter()
            .map(|&u| (u, self.evaluate(u).distance_squared(query)))
            .collect();

        let mut best_u = u0;
        let mut best_pt = self.evaluate(u0);
        let mut best_d2 = best_pt.distance_squared(query);

        let p_end = self.evaluate(u1);
        let d2_end = p_end.distance_squared(query);
        if d2_end < best_d2 {
            best_d2 = d2_end;
            best_u = u1;
            best_pt = p_end;
        }

        // Collect local minima + global best seeds
        let mut seeds: Vec<(f64, f64)> = Vec::new();
        for i in 0..dists.len() {
            let left_ok = i == 0 || dists[i].1 <= dists[i - 1].1;
            let right_ok = i + 1 == dists.len() || dists[i].1 <= dists[i + 1].1;
            if left_ok && right_ok {
                seeds.push(dists[i]);
            }
        }
        seeds.sort_by(|a, b| a.1.total_cmp(&b.1));
        seeds.truncate(4);

        for (mut u, _) in seeds {
            let mut cur_pt = self.evaluate(u);
            let mut cur_d2 = cur_pt.distance_squared(query);
            for _ in 0..16 {
                let ders = self.evaluate_derivatives(u, 2);
                let diff = ders[0] - query;
                let f = diff.dot(ders[1]);
                let fp = ders[1].length_squared() + diff.dot(ders[2]);
                if f.abs() < 1e-12 || fp.abs() < 1e-14 {
                    break;
                }
                let mut step = -f / fp;
                let max_step = (u1 - u0) * 0.25;
                if step.abs() > max_step {
                    step = step.signum() * max_step;
                }
                let mut accepted = false;
                for _ in 0..6 {
                    let next_u = (u + step).clamp(u0, u1);
                    let next_pt = self.evaluate(next_u);
                    let next_d2 = next_pt.distance_squared(query);
                    if next_d2 <= cur_d2 + 1e-15 {
                        let moved = (next_u - u).abs();
                        u = next_u;
                        cur_pt = next_pt;
                        cur_d2 = next_d2;
                        accepted = true;
                        if moved < 1e-12 {
                            break;
                        }
                        break;
                    }
                    step *= 0.5;
                }
                if !accepted || step.abs() < 1e-12 {
                    break;
                }
            }
            if cur_d2 < best_d2 {
                best_d2 = cur_d2;
                best_u = u;
                best_pt = cur_pt;
            }
        }

        (best_u, best_pt, best_d2.sqrt())
    }

    pub fn is_rational(&self) -> bool {
        self.control_points
            .iter()
            .any(|cp| (cp.weight - 1.0).abs() > 1e-9)
    }

    pub fn reverse(&self) -> Self {
        let (u0, u1) = self.domain();
        let control_points: Vec<ControlPoint> = self.control_points.iter().copied().rev().collect();
        let knots: Vec<f64> = self.knots.iter().rev().map(|&k| u0 + u1 - k).collect();
        Self {
            degree: self.degree,
            knots,
            control_points,
        }
    }

    pub fn reparameterize(&self, new_u0: f64, new_u1: f64) -> Self {
        let (u0, u1) = self.domain();
        let span = u1 - u0;
        let new_span = new_u1 - new_u0;
        if span.abs() < PARAM_EPSILON || new_span <= PARAM_EPSILON {
            return self.clone();
        }
        let scale = new_span / span;
        let knots = self
            .knots
            .iter()
            .map(|&k| new_u0 + (k - u0) * scale)
            .collect();
        Self {
            degree: self.degree,
            knots,
            control_points: self.control_points.clone(),
        }
    }

    /// Insert knot `u` `times` times using Boehm's algorithm (A5.1) in 4D homogeneous space.
    pub fn insert_knot(&self, mut u: f64, times: usize) -> Result<Self, GeomError> {
        if times == 0 {
            return Ok(self.clone());
        }
        let (u0, u1) = self.domain();
        if u <= u0 + 1e-11 || u >= u1 - 1e-11 {
            return Ok(self.clone());
        }
        for &k in &self.knots {
            if (k - u).abs() <= 1e-11 {
                u = k;
                break;
            }
        }

        let p = self.degree;
        let n = self.control_points.len() - 1;
        let s = self.knots.iter().filter(|&&k| (k - u).abs() <= 1e-11).count();
        let r = times.min(p.saturating_sub(s));
        if r == 0 {
            return Ok(self.clone());
        }

        let k = self
            .knots
            .iter()
            .enumerate()
            .take(n + 1)
            .filter(|&(_, &kv)| kv <= u + 1e-11)
            .map(|(idx, _)| idx)
            .last()
            .unwrap_or(p)
            .max(p);

        let mut uq = Vec::with_capacity(self.knots.len() + r);
        uq.extend_from_slice(&self.knots[..=k]);
        uq.extend(std::iter::repeat_n(u, r));
        uq.extend_from_slice(&self.knots[(k + 1)..]);

        let pw: Vec<DVec4> = self
            .control_points
            .iter()
            .map(|cp| cp.to_homogeneous())
            .collect();
        let mut qw = vec![DVec4::ZERO; n + 1 + r];
        let mut rw = vec![DVec4::ZERO; p - s + 1];

        for i in 0..=(k - p) {
            qw[i] = pw[i];
        }
        for i in (k - s)..=n {
            qw[i + r] = pw[i];
        }
        for i in 0..=(p - s) {
            rw[i] = pw[k - p + i];
        }

        let mut l_last = 0;
        for j in 1..=r {
            let l = k - p + j;
            l_last = l;
            for i in 0..=(p - j - s) {
                let denom = self.knots[i + k + 1] - self.knots[l + i];
                let alpha = if denom.abs() < PARAM_EPSILON {
                    0.0
                } else {
                    (u - self.knots[l + i]) / denom
                };
                rw[i] = rw[i + 1] * alpha + rw[i] * (1.0 - alpha);
            }
            qw[l] = rw[0];
            qw[k + r - j - s] = rw[p - j - s];
        }
        for i in (l_last + 1)..(k - s) {
            qw[i] = rw[i - l_last];
        }

        let cps = qw
            .into_iter()
            .map(|h| {
                let w = h.w;
                let pos = if w.abs() > PARAM_EPSILON {
                    h.truncate() / w
                } else {
                    h.truncate()
                };
                ControlPoint::new(pos, w.max(1e-12))
            })
            .collect();

        Self::new(p, uq, cps)
    }

    pub fn refine_knots(&self, knots_to_insert: &[f64]) -> Result<Self, GeomError> {
        let mut cur = self.clone();
        for &u in knots_to_insert {
            cur = cur.insert_knot(u, 1)?;
        }
        Ok(cur)
    }

    /// Split the curve at interior parameter `u`, returning `Some((left, right))`.
    pub fn split_at(&self, mut u: f64) -> Option<(Self, Self)> {
        let (u0, u1) = self.domain();
        if u <= u0 + 1e-9 || u >= u1 - 1e-9 {
            return None;
        }
        for &k in &self.knots {
            if (k - u).abs() <= 1e-11 {
                u = k;
                break;
            }
        }
        let p = self.degree;
        let s = self.knots.iter().filter(|&&k| (k - u).abs() <= 1e-11).count();
        let refined = self.insert_knot(u, p.saturating_sub(s)).ok()?;

        let first_k = refined
            .knots
            .iter()
            .enumerate()
            .skip(p)
            .find(|&(_, &kv)| (kv - u).abs() <= 1e-11)
            .map(|(idx, _)| idx)?;

        if first_k == 0 || first_k > refined.control_points.len() {
            return None;
        }

        let left_cps = refined.control_points[..first_k].to_vec();
        let mut left_knots = refined.knots[..(first_k + p)].to_vec();
        left_knots.push(u);
        let left = Self::new(p, left_knots, left_cps).ok()?;

        let right_cps = refined.control_points[(first_k - 1)..].to_vec();
        let mut right_knots = Vec::with_capacity(1 + refined.knots.len() - first_k);
        right_knots.push(u);
        right_knots.extend_from_slice(&refined.knots[first_k..]);
        let right = Self::new(p, right_knots, right_cps).ok()?;

        Some((left, right))
    }

    /// Split the curve into exact single-span rational Bézier segments (`p + 1` control points each).
    pub fn decompose_to_bezier_spans(&self) -> Vec<Self> {
        let (u0, u1) = self.domain();
        let mut interior = Vec::new();
        for &k in &self.knots {
            if k > u0 + 1e-9 && k < u1 - 1e-9 {
                if interior.last().is_none_or(|&prev: &f64| (k - prev).abs() > 1e-9) {
                    interior.push(k);
                }
            }
        }
        let mut spans = Vec::with_capacity(interior.len() + 1);
        let mut cur = self.clone();
        for u in interior {
            if let Some((left, right)) = cur.split_at(u) {
                spans.push(left);
                cur = right;
            }
        }
        spans.push(cur);
        spans
    }

    /// Elevate curve degree by `delta` while preserving exact geometry and parameterization.
    pub fn elevate_degree(&self, delta: usize) -> Result<Self, GeomError> {
        if delta == 0 {
            return Ok(self.clone());
        }
        let mut spans = self.decompose_to_bezier_spans();
        for span in &mut spans {
            for _ in 0..delta {
                let p = span.degree;
                let pw: Vec<DVec4> = span.control_points.iter().map(|cp| cp.to_homogeneous()).collect();
                let mut qw = Vec::with_capacity(p + 2);
                qw.push(pw[0]);
                for i in 1..=p {
                    let alpha = i as f64 / (p + 1) as f64;
                    qw.push(pw[i - 1] * alpha + pw[i] * (1.0 - alpha));
                }
                qw.push(pw[p]);
                let cps = qw
                    .into_iter()
                    .map(|h| {
                        let w = h.w;
                        ControlPoint::new(h.truncate() / w, w)
                    })
                    .collect();
                let (s0, s1) = span.domain();
                let mut knots = Vec::with_capacity(2 * (p + 2));
                knots.extend(std::iter::repeat_n(s0, p + 2));
                knots.extend(std::iter::repeat_n(s1, p + 2));
                *span = Self::new(p + 1, knots, cps)?;
            }
        }

        let new_p = self.degree + delta;
        let (u0, u1) = self.domain();
        let mut cps = Vec::new();
        let mut knots = Vec::new();
        knots.extend(std::iter::repeat_n(u0, new_p + 1));
        for (idx, span) in spans.iter().enumerate() {
            if idx == 0 {
                cps.extend_from_slice(&span.control_points);
            } else {
                let (s0, _) = span.domain();
                knots.extend(std::iter::repeat_n(s0, new_p));
                cps.extend_from_slice(&span.control_points[1..]);
            }
        }
        knots.extend(std::iter::repeat_n(u1, new_p + 1));
        Self::new(new_p, knots, cps)
    }

    /// Collect parameter values of kinks (multiplicity >= degree interior knots or tangent jumps > `angle_tol_rad`).
    pub fn kink_parameters(&self, angle_tol_rad: f64) -> Vec<f64> {
        let (u0, u1) = self.domain();
        let mut kinks = Vec::new();
        let mut i = self.degree + 1;
        while i < self.control_points.len() {
            let u = self.knots[i];
            let mut mult = 1;
            while i + mult < self.control_points.len()
                && (self.knots[i + mult] - u).abs() <= 1e-11
            {
                mult += 1;
            }
            if u > u0 + 1e-9 && u < u1 - 1e-9 && mult >= self.degree {
                let eps = ((u1 - u0) * 1e-6).max(1e-8);
                let (_, t_left) = self.evaluate_derivative((u - eps).max(u0));
                let (_, t_right) = self.evaluate_derivative((u + eps).min(u1));
                let tl = t_left.normalize_or_zero();
                let tr = t_right.normalize_or_zero();
                if tl.length_squared() < 1e-12
                    || tr.length_squared() < 1e-12
                    || tl.angle_between(tr) > angle_tol_rad
                    || self.degree == 1
                {
                    kinks.push(u);
                }
            }
            i += mult;
        }
        kinks
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

    /// Build a NURBS control-point curve (`degree` clamped to `[1, points.len() - 1]`).
    pub fn from_control_points(
        points: &[DVec3],
        degree: usize,
        closed: bool,
    ) -> Result<Self, GeomError> {
        if points.len() < 2 {
            return Err(GeomError::InsufficientControlPoints {
                degree: 1,
                found: points.len(),
                needed: 2,
            });
        }
        if closed && points.len() >= 3 && degree >= 2 {
            let mut clean: Vec<DVec3> = Vec::with_capacity(points.len());
            for &p in points {
                if clean.last().is_none_or(|prev| prev.distance(p) > 1e-9) {
                    clean.push(p);
                }
            }
            if clean.len() >= 3 && clean.first().unwrap().distance(*clean.last().unwrap()) < 1e-9 {
                clean.pop();
            }
            if clean.len() >= 3 {
                let m = clean.len();
                let p = degree.clamp(2, m);
                // Uniform periodic unclamped spline over [-pad .. m + pad], then clamped to [0, 1]
                let n_ext = m + p + 2;
                let mut cps = Vec::with_capacity(n_ext);
                for i in 0..n_ext {
                    cps.push(ControlPoint::unweighted(clean[i % m]));
                }
                let knot_len = n_ext + p + 1;
                let mut knots = Vec::with_capacity(knot_len);
                let step = 1.0 / m as f64;
                for j in 0..knot_len {
                    knots.push((j as isize - p as isize - 1) as f64 * step);
                }
                let unclamped = Self::new(p, knots, cps)?;
                let (_, rest) = unclamped.split_at(0.0).unwrap_or((unclamped.clone(), unclamped));
                let (closed_curve, _) = rest.split_at(1.0).unwrap_or((rest.clone(), rest));
                return Ok(closed_curve);
            }
        }

        let mut pts = points.to_vec();
        if closed && pts.first().unwrap().distance(*pts.last().unwrap()) > 1e-9 {
            pts.push(pts[0]);
        }
        let n = pts.len();
        let p = degree.clamp(1, n - 1);
        let knots = uniform_clamped_knots(n, p);
        let cps = pts.into_iter().map(ControlPoint::unweighted).collect();
        Self::new(p, knots, cps)
    }

    /// Global NURBS interpolation through `points` (Piegl & Tiller A9.1, centripetal parameterization).
    pub fn interpolate(points: &[DVec3], degree: usize, closed: bool) -> Result<Self, GeomError> {
        if points.len() < 2 {
            return Err(GeomError::InsufficientControlPoints {
                degree: 1,
                found: points.len(),
                needed: 2,
            });
        }
        let mut pts = Vec::with_capacity(points.len());
        for &p in points {
            if pts.last().is_none_or(|prev: &DVec3| prev.distance(p) > 1e-9) {
                pts.push(p);
            }
        }
        if closed && pts.len() >= 3 && pts.first().unwrap().distance(*pts.last().unwrap()) < 1e-9 {
            pts.pop();
        }
        if pts.len() < 2 {
            return Err(GeomError::Degenerate("interpolation points must be distinct"));
        }

        if closed && pts.len() >= 3 {
            let p = degree.clamp(1, pts.len());
            if p == 1 {
                pts.push(pts[0]);
                return Self::polyline(&pts);
            }
            return interpolate_closed_periodic(&pts, p);
        }

        let mut open_pts = pts;
        if closed {
            open_pts.push(open_pts[0]);
        }
        let m = open_pts.len() - 1;
        let p = degree.clamp(1, m);
        if p == 1 {
            return Self::polyline(&open_pts);
        }

        let mut params = vec![0.0_f64; m + 1];
        let mut total = 0.0;
        for i in 0..m {
            let d = open_pts[i].distance(open_pts[i + 1]).sqrt();
            total += d;
            params[i + 1] = total;
        }
        if total < 1e-12 {
            return Err(GeomError::Degenerate("interpolation points are coincident"));
        }
        for t in &mut params {
            *t /= total;
        }
        params[m] = 1.0;

        let mut knots = Vec::with_capacity(m + p + 2);
        knots.extend(std::iter::repeat_n(0.0, p + 1));
        for j in 1..=(m - p) {
            let sum: f64 = params[j..(j + p)].iter().sum();
            knots.push(sum / p as f64);
        }
        knots.extend(std::iter::repeat_n(1.0, p + 1));

        let n_cp = m + 1;
        let mut mat = vec![vec![0.0; n_cp]; n_cp];
        for (k, &uk) in params.iter().enumerate() {
            let span = find_span(m, p, uk, &knots);
            let basis = basis_functions(span, uk, p, &knots);
            for (i, &b) in basis.iter().enumerate() {
                mat[k][span - p + i] = b;
            }
        }

        let solved = solve_linear_system_vec3(&mat, &open_pts)
            .ok_or(GeomError::Degenerate("singular interpolation matrix"))?;
        let cps = solved.into_iter().map(ControlPoint::unweighted).collect();
        Self::new(p, knots, cps)
    }

    /// Composite cubic NURBS curve built from Bézier handle segments `(p0, h_out, h_in, p1)`.
    pub fn handle_curve(segments: &[(DVec3, DVec3, DVec3, DVec3)]) -> Result<Self, GeomError> {
        if segments.is_empty() {
            return Err(GeomError::Degenerate("handle curve requires at least 1 segment"));
        }
        let mut curves = Vec::with_capacity(segments.len());
        for &(p0, h0, h1, p1) in segments {
            let cps = vec![
                ControlPoint::unweighted(p0),
                ControlPoint::unweighted(h0),
                ControlPoint::unweighted(h1),
                ControlPoint::unweighted(p1),
            ];
            let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
            curves.push(Self::new(3, knots, cps)?);
        }
        let mut acc = curves.remove(0);
        for seg in curves {
            acc = join_two_exact(&acc, &seg)?;
        }
        Ok(acc)
    }

    /// Exact rational quadratic conic section from `p0` to `p2` with shoulder parameter `rho in (0.01, 0.99)`.
    pub fn conic(p0: DVec3, apex: DVec3, p2: DVec3, rho: f64) -> Result<Self, GeomError> {
        if !rho.is_finite() || rho <= 0.01 || rho >= 0.99 {
            return Err(GeomError::Degenerate("conic rho must lie in (0.01, 0.99)"));
        }
        if (apex - p0).cross(p2 - apex).length() < 1e-8 {
            return Err(GeomError::Degenerate("conic control points are collinear"));
        }
        let w1 = rho / (1.0 - rho);
        let cps = vec![
            ControlPoint::new(p0, 1.0),
            ControlPoint::new(apex, w1),
            ControlPoint::new(p2, 1.0),
        ];
        Self::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], cps)
    }

    /// Interpolated cubic NURBS helix or spiral around `(axis_start, axis_end)`.
    pub fn helix(
        axis_start: DVec3,
        axis_end: DVec3,
        start_pt: DVec3,
        turns: f64,
        end_radius: Option<f64>,
    ) -> Result<Self, GeomError> {
        let axis = axis_end - axis_start;
        let height = axis.length();
        if height < 1e-6 || !turns.is_finite() || turns.abs() < 0.05 {
            return Err(GeomError::Degenerate("helix requires non-zero axis and turns"));
        }
        let z_dir = axis / height;
        let radial_vec = start_pt - axis_start;
        let rad_proj = radial_vec - z_dir * radial_vec.dot(z_dir);
        let r0 = rad_proj.length();
        if r0 < 1e-6 {
            return Err(GeomError::Degenerate("helix start point lies on axis"));
        }
        let x_dir = rad_proj / r0;
        let y_dir = z_dir.cross(x_dir).normalize();
        let r1 = end_radius.unwrap_or(r0).max(1e-4);

        let samples = ((turns.abs() * 16.0).ceil() as usize).max(16);
        let mut pts = Vec::with_capacity(samples + 1);
        for i in 0..=samples {
            let t = i as f64 / samples as f64;
            let angle = std::f64::consts::TAU * turns * t;
            let r = r0 + (r1 - r0) * t;
            let p = axis_start
                + z_dir * (height * t)
                + (x_dir * angle.cos() + y_dir * angle.sin()) * r;
            pts.push(p);
        }
        Self::interpolate(&pts, 3, false)
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

    /// Explode a polyline into segments, or a multi-segment NURBS curve at tangent/curvature discontinuities.
    pub fn explode_segments(&self) -> Vec<NurbsCurve> {
        if self.degree == 1 && self.control_points.len() > 2 {
            return self
                .control_points
                .windows(2)
                .filter(|w| w[0].position.distance(w[1].position) > 1e-9)
                .map(|w| Self::line(w[0].position, w[1].position))
                .collect();
        }
        let (u0, u1) = self.domain();
        let eps = ((u1 - u0) * 1e-5).max(1e-7);
        let mut splits = Vec::new();
        let mut i = self.degree + 1;
        while i < self.control_points.len() {
            let u = self.knots[i];
            let mut mult = 1;
            while i + mult < self.control_points.len()
                && (self.knots[i + mult] - u).abs() <= 1e-11
            {
                mult += 1;
            }
            if u > u0 + 1e-9 && u < u1 - 1e-9 && mult >= self.degree {
                let ul = (u - eps).max(u0);
                let ur = (u + eps).min(u1);
                let (_, tl) = self.evaluate_derivative(ul);
                let (_, tr) = self.evaluate_derivative(ur);
                let (kl, _) = self.curvature_at(ul);
                let (kr, _) = self.curvature_at(ur);
                let tangent_jump = tl.normalize_or_zero().angle_between(tr.normalize_or_zero()) > 1e-3;
                let curv_jump = kl.distance(kr) > 1e-3;
                if tangent_jump || curv_jump {
                    splits.push(u);
                }
            }
            i += mult;
        }
        if splits.is_empty() {
            return vec![self.clone()];
        }
        let mut out = Vec::with_capacity(splits.len() + 1);
        let mut cur = self.clone();
        for u in splits {
            if let Some((left, right)) = cur.split_at(u) {
                out.push(left);
                cur = right;
            }
        }
        out.push(cur);
        out
    }

    /// Offset this curve in `plane` by `distance` toward the side of `side_pt`.
    pub fn offset_in_plane(
        &self,
        plane: &Plane,
        distance: f64,
        side_pt: DVec3,
    ) -> Result<Self, GeomError> {
        offset_nurbs_curve_in_plane(self, plane, distance, side_pt, OffsetCornerStyle::Sharp)
    }

    /// Join contiguous curves whose endpoints match within `tol`, preserving exact NURBS geometry.
    pub fn join_curves(curves: &[NurbsCurve], tol: f64) -> Vec<NurbsCurve> {
        let mut pool: Vec<NurbsCurve> = curves
            .iter()
            .filter(|c| c.control_points.len() >= 2)
            .cloned()
            .collect();

        let mut result = Vec::new();
        while !pool.is_empty() {
            let mut cur = pool.remove(0);
            loop {
                if cur.is_closed() {
                    break;
                }
                let (u0, u1) = cur.domain();
                let head = cur.evaluate(u0);
                let tail = cur.evaluate(u1);
                let mut merged = false;

                for i in 0..pool.len() {
                    let (cu0, cu1) = pool[i].domain();
                    let c_head = pool[i].evaluate(cu0);
                    let c_tail = pool[i].evaluate(cu1);
                    if tail.distance(c_head) <= tol {
                        let next = pool.remove(i);
                        if let Ok(joined) = join_two_exact(&cur, &next) {
                            cur = joined;
                            merged = true;
                            break;
                        }
                    } else if tail.distance(c_tail) <= tol {
                        let next = pool.remove(i).reverse();
                        if let Ok(joined) = join_two_exact(&cur, &next) {
                            cur = joined;
                            merged = true;
                            break;
                        }
                    } else if head.distance(c_tail) <= tol {
                        let next = pool.remove(i);
                        if let Ok(joined) = join_two_exact(&next, &cur) {
                            cur = joined;
                            merged = true;
                            break;
                        }
                    } else if head.distance(c_head) <= tol {
                        let next = pool.remove(i).reverse();
                        if let Ok(joined) = join_two_exact(&next, &cur) {
                            cur = joined;
                            merged = true;
                            break;
                        }
                    }
                }
                if !merged {
                    break;
                }
            }
            result.push(cur);
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

pub(crate) fn uniform_clamped_knots(num_cps: usize, degree: usize) -> Vec<f64> {
    let mut knots = Vec::with_capacity(num_cps + degree + 1);
    knots.extend(std::iter::repeat_n(0.0, degree + 1));
    let interior = num_cps.saturating_sub(degree + 1);
    for i in 1..=interior {
        knots.push(i as f64 / (interior + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

fn binomial_coeff(n: usize, k: usize) -> f64 {
    let mut res = 1.0_f64;
    for i in 1..=k.min(n - k) {
        res = res * (n - i + 1) as f64 / i as f64;
    }
    res
}

fn fallback_frame_axes(t: DVec3) -> (DVec3, DVec3) {
    let ref_v = if t.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
    let n = ref_v.cross(t).normalize();
    let b = t.cross(n).normalize();
    (n, b)
}

fn interpolate_closed_periodic(pts: &[DVec3], degree: usize) -> Result<NurbsCurve, GeomError> {
    let m = pts.len();
    let p = degree.clamp(2, m);

    let mut ts = vec![0.0_f64; m + 1];
    let mut total = 0.0;
    for i in 0..m {
        total += pts[i].distance(pts[(i + 1) % m]).sqrt();
        ts[i + 1] = total;
    }
    if total < 1e-12 {
        return Err(GeomError::Degenerate("closed interpolation points are coincident"));
    }
    for t in &mut ts {
        *t /= total;
    }
    ts[m] = 1.0;

    let periodic_t = |idx: isize| -> f64 {
        let m_i = m as isize;
        let q = idx.div_euclid(m_i);
        let r = idx.rem_euclid(m_i) as usize;
        ts[r] + q as f64
    };

    // Knot sequence tau: for odd degree knots coincide with ts; for even degree knots sit at midpoints
    let periodic_tau = |idx: isize| -> f64 {
        if p % 2 == 1 {
            periodic_t(idx)
        } else {
            0.5 * (periodic_t(idx - 1) + periodic_t(idx))
        }
    };

    // Build padded unclamped B-spline covering [-1, m+1] so [0.0, 1.0] is strictly interior
    let pad = 1_usize;
    let n_ext = m + p + 2 * pad;
    let knot_len = n_ext + p + 1;
    let mut knots = Vec::with_capacity(knot_len);
    for j in 0..knot_len {
        let idx = j as isize - (p + pad) as isize;
        knots.push(periodic_tau(idx));
    }

    // Build m x m collocation matrix by evaluating wrapped basis functions at t_0..t_{m-1}
    let mut mat = vec![vec![0.0_f64; m]; m];
    for row in 0..m {
        let u = ts[row];
        let span = find_span(n_ext - 1, p, u, &knots);
        let basis = basis_functions(span, u, p, &knots);
        for (i, &b) in basis.iter().enumerate() {
            let ext_idx = span - p + i;
            let col = (ext_idx as isize - pad as isize).rem_euclid(m as isize) as usize;
            mat[row][col] += b;
        }
    }

    let q_cps = solve_linear_system_vec3(&mat, pts)
        .ok_or(GeomError::Degenerate("singular closed interpolation matrix"))?;

    let mut ext_cps = Vec::with_capacity(n_ext);
    for i in 0..n_ext {
        let idx = (i as isize - pad as isize).rem_euclid(m as isize) as usize;
        ext_cps.push(ControlPoint::unweighted(q_cps[idx]));
    }

    let unclamped = NurbsCurve::new(p, knots, ext_cps)?;
    let (_, rest) = unclamped
        .split_at(0.0)
        .ok_or(GeomError::Degenerate("failed to clamp periodic start"))?;
    let (closed_curve, _) = rest
        .split_at(1.0)
        .ok_or(GeomError::Degenerate("failed to clamp periodic end"))?;
    Ok(closed_curve)
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
