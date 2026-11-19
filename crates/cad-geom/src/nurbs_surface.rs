use cad_math::linalg::solve_linear_system;
use cad_math::{Aabb, DMat4, DVec3, DVec4, PARAM_EPSILON, Plane, Ray};
use serde::{Deserialize, Serialize};

use crate::error::GeomError;
use crate::mesh::TriangleMesh;
use crate::nurbs_curve::{
    ControlPoint, NurbsCurve, basis_function_derivatives, basis_functions, find_span,
};
use crate::surface_ops::revolve_nurbs;

/// Boundary edge identifier on a tensor-product NURBS surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfaceEdge {
    UMin,
    UMax,
    VMin,
    VMax,
}

impl SurfaceEdge {
    pub const ALL: [Self; 4] = [Self::UMin, Self::UMax, Self::VMin, Self::VMax];
}

/// First- and second-order partial derivatives on a NURBS surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceDerivatives {
    pub point: DVec3,
    pub du: DVec3,
    pub dv: DVec3,
    pub duu: DVec3,
    pub duv: DVec3,
    pub dvv: DVec3,
}

/// Orthonormal local UVN frame on a NURBS surface (powers Rhino 9-style UVN Gumball).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceFrame {
    pub origin: DVec3,
    pub u_dir: DVec3,
    pub v_dir: DVec3,
    pub normal: DVec3,
}

/// Differential surface curvature metrics computed from First and Second Fundamental Forms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCurvature {
    pub point: DVec3,
    pub normal: DVec3,
    pub gaussian: f64,
    pub mean: f64,
    pub k1: f64,
    pub k2: f64,
}

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

    /// Exact rational NURBS sphere centered at `center` with `radius`.
    pub fn sphere(center: DVec3, radius: f64) -> Result<Self, GeomError> {
        let r = radius.abs();
        if !r.is_finite() || r < 1e-6 {
            return Err(GeomError::Degenerate("sphere radius must be positive"));
        }
        let xz_plane = Plane {
            origin: center,
            x_axis: DVec3::X,
            y_axis: DVec3::Z,
            z_axis: -DVec3::Y,
        };
        // Semicircle from north pole (+Z) through +X equator to south pole (-Z) so outward normal is +X.
        let meridian = NurbsCurve::arc_from_center(
            &xz_plane,
            center,
            r,
            std::f64::consts::FRAC_PI_2,
            -std::f64::consts::PI,
        )?;
        let mut srf = revolve_nurbs(&meridian, center, DVec3::Z, std::f64::consts::TAU)?;
        let (u0, u1) = srf.domain_u();
        let (v0, v1) = srf.domain_v();
        let (p, n) = srf.evaluate_point_and_normal(0.5 * (u0 + u1), 0.5 * (v0 + v1));
        if n.dot(p - center) < 0.0 {
            srf = srf.flip_normal();
        }
        Ok(srf)
    }

    /// Exact rational NURBS cylindrical surface centered at `base_center` along `+Z`.
    pub fn cylinder(base_center: DVec3, radius: f64, height: f64) -> Result<Self, GeomError> {
        let r = radius.abs();
        if !r.is_finite() || r < 1e-6 || !height.is_finite() || height.abs() < 1e-6 {
            return Err(GeomError::Degenerate("cylinder radius and height must be non-zero"));
        }
        let p0 = base_center + DVec3::X * r;
        let p1 = p0 + DVec3::Z * height;
        let generatrix = NurbsCurve::line(p0, p1);
        let mut srf = revolve_nurbs(&generatrix, base_center, DVec3::Z, std::f64::consts::TAU)?;
        let (u0, u1) = srf.domain_u();
        let (v0, v1) = srf.domain_v();
        let (p, n) = srf.evaluate_point_and_normal(0.5 * (u0 + u1), v0);
        let radial = DVec3::new(p.x - base_center.x, p.y - base_center.y, 0.0);
        if n.dot(radial) < 0.0 {
            srf = srf.flip_normal();
        }
        let _ = v1;
        Ok(srf)
    }

    /// Exact rational NURBS conical surface centered at `base_center` with apex at `+height * Z`.
    pub fn cone(base_center: DVec3, radius: f64, height: f64) -> Result<Self, GeomError> {
        let r = radius.abs();
        if !r.is_finite() || r < 1e-6 || !height.is_finite() || height.abs() < 1e-6 {
            return Err(GeomError::Degenerate("cone radius and height must be non-zero"));
        }
        let p0 = base_center + DVec3::X * r;
        let p1 = base_center + DVec3::Z * height;
        let generatrix = NurbsCurve::line(p0, p1);
        let mut srf = revolve_nurbs(&generatrix, base_center, DVec3::Z, std::f64::consts::TAU)?;
        let (u0, u1) = srf.domain_u();
        let (v0, _) = srf.domain_v();
        let (p, n) = srf.evaluate_point_and_normal(u0 + (u1 - u0) * 0.25, v0);
        let radial = DVec3::new(p.x - base_center.x, p.y - base_center.y, 0.0);
        if n.dot(radial) < 0.0 {
            srf = srf.flip_normal();
        }
        Ok(srf)
    }

    /// Exact rational NURBS torus centered at `center` in XY plane.
    pub fn torus(center: DVec3, major_radius: f64, minor_radius: f64) -> Result<Self, GeomError> {
        let maj = major_radius.abs();
        let min = minor_radius.abs();
        if !maj.is_finite() || !min.is_finite() || min < 1e-6 || maj < min + 1e-6 {
            return Err(GeomError::Degenerate("torus requires major radius > minor radius > 0"));
        }
        let xz_plane = Plane {
            origin: center + DVec3::X * maj,
            x_axis: DVec3::X,
            y_axis: DVec3::Z,
            z_axis: -DVec3::Y,
        };
        let section = NurbsCurve::circle(&xz_plane, min)?;
        let mut srf = revolve_nurbs(&section, center, DVec3::Z, std::f64::consts::TAU)?;
        let (u0, _) = srf.domain_u();
        let (v0, _) = srf.domain_v();
        let (p, n) = srf.evaluate_point_and_normal(u0, v0);
        let outward = p - (center + DVec3::X * maj);
        if n.dot(outward) < 0.0 {
            srf = srf.flip_normal();
        }
        Ok(srf)
    }

    #[inline]
    pub fn get_cp(&self, u_idx: usize, v_idx: usize) -> ControlPoint {
        self.control_points[u_idx * self.count_v + v_idx]
    }

    #[inline]
    pub fn set_cp(&mut self, u_idx: usize, v_idx: usize, cp: ControlPoint) {
        self.control_points[u_idx * self.count_v + v_idx] = cp;
    }

    #[inline]
    pub fn domain_u(&self) -> (f64, f64) {
        (self.knots_u[self.degree_u], self.knots_u[self.count_u])
    }

    #[inline]
    pub fn domain_v(&self) -> (f64, f64) {
        (self.knots_v[self.degree_v], self.knots_v[self.count_v])
    }

    pub fn is_rational(&self) -> bool {
        self.control_points
            .iter()
            .any(|cp| (cp.weight - 1.0).abs() > 1e-9)
    }

    pub fn is_closed_u(&self) -> bool {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        for s in 0..=4 {
            let v = v0 + (v1 - v0) * (s as f64 / 4.0);
            if self.evaluate(u0, v).distance(self.evaluate(u1, v)) > 1e-5 {
                return false;
            }
        }
        true
    }

    pub fn is_closed_v(&self) -> bool {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        for s in 0..=4 {
            let u = u0 + (u1 - u0) * (s as f64 / 4.0);
            if self.evaluate(u, v0).distance(self.evaluate(u, v1)) > 1e-5 {
                return false;
            }
        }
        true
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

    /// Evaluate exact point and rational partial derivatives up to 2nd order (S, Su, Sv, Suu, Suv, Svv).
    pub fn evaluate_derivatives(&self, u: f64, v: f64) -> SurfaceDerivatives {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);

        let span_u = find_span(self.count_u - 1, self.degree_u, u, &self.knots_u);
        let span_v = find_span(self.count_v - 1, self.degree_v, v, &self.knots_v);
        let du = basis_function_derivatives(span_u, u, self.degree_u, 2, &self.knots_u);
        let dv = basis_function_derivatives(span_v, v, self.degree_v, 2, &self.knots_v);

        let mut h00 = DVec4::ZERO;
        let mut h10 = DVec4::ZERO;
        let mut h01 = DVec4::ZERO;
        let mut h20 = DVec4::ZERO;
        let mut h11 = DVec4::ZERO;
        let mut h02 = DVec4::ZERO;

        let has_du2 = du.len() > 2;
        let has_dv2 = dv.len() > 2;

        for i in 0..=self.degree_u {
            let ui = span_u - self.degree_u + i;
            let mut row0 = DVec4::ZERO;
            let mut row1 = DVec4::ZERO;
            let mut row2 = DVec4::ZERO;
            for j in 0..=self.degree_v {
                let vj = span_v - self.degree_v + j;
                let h = self.control_points[ui * self.count_v + vj].to_homogeneous();
                row0 += h * dv[0][j];
                row1 += h * dv[1][j];
                if has_dv2 {
                    row2 += h * dv[2][j];
                }
            }
            h00 += row0 * du[0][i];
            h10 += row0 * du[1][i];
            h01 += row1 * du[0][i];
            h11 += row1 * du[1][i];
            if has_du2 {
                h20 += row0 * du[2][i];
            }
            if has_dv2 {
                h02 += row2 * du[0][i];
            }
        }

        let w = h00.w;
        if w.abs() < PARAM_EPSILON {
            return SurfaceDerivatives {
                point: DVec3::ZERO,
                du: DVec3::ZERO,
                dv: DVec3::ZERO,
                duu: DVec3::ZERO,
                duv: DVec3::ZERO,
                dvv: DVec3::ZERO,
            };
        }

        let point = h00.truncate() / w;
        let d_du = (h10.truncate() - point * h10.w) / w;
        let d_dv = (h01.truncate() - point * h01.w) / w;
        let d_duu = (h20.truncate() - d_du * (2.0 * h10.w) - point * h20.w) / w;
        let d_duv = (h11.truncate() - d_dv * h10.w - d_du * h01.w - point * h11.w) / w;
        let d_dvv = (h02.truncate() - d_dv * (2.0 * h01.w) - point * h02.w) / w;

        SurfaceDerivatives {
            point,
            du: d_du,
            dv: d_dv,
            duu: d_duu,
            duv: d_duv,
            dvv: d_dvv,
        }
    }

    fn non_degenerate_ders(&self, u: f64, v: f64) -> SurfaceDerivatives {
        let base = self.evaluate_derivatives(u, v);
        if base.du.cross(base.dv).length_squared() > 1e-16 {
            return base;
        }
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let eps_u = (u1 - u0) * 1e-4;
        let eps_v = (v1 - v0) * 1e-4;
        let nu = if (u - u0).abs() <= eps_u {
            (u + eps_u).clamp(u0, u1)
        } else if (u1 - u).abs() <= eps_u {
            (u - eps_u).clamp(u0, u1)
        } else {
            (u + eps_u * 0.5).clamp(u0, u1)
        };
        let nv = if (v - v0).abs() <= eps_v {
            (v + eps_v).clamp(v0, v1)
        } else if (v1 - v).abs() <= eps_v {
            (v - eps_v).clamp(v0, v1)
        } else {
            (v + eps_v * 0.5).clamp(v0, v1)
        };
        let nudged = self.evaluate_derivatives(nu, nv);
        SurfaceDerivatives {
            point: base.point,
            du: nudged.du,
            dv: nudged.dv,
            duu: nudged.duu,
            duv: nudged.duv,
            dvv: nudged.dvv,
        }
    }

    /// Evaluate `(point, normal)` at `(u, v)`. Handles singular poles cleanly via interior nudge.
    pub fn evaluate_point_and_normal(&self, u: f64, v: f64) -> (DVec3, DVec3) {
        let d = self.non_degenerate_ders(u, v);
        let n = d.du.cross(d.dv).normalize_or_zero();
        (d.point, if n.length_squared() > 1e-12 { n } else { DVec3::Z })
    }

    /// Orthonormal `(origin, u_dir, v_dir, normal)` local surface frame at `(u, v)`.
    pub fn surface_frame(&self, u: f64, v: f64) -> SurfaceFrame {
        let d = self.non_degenerate_ders(u, v);
        let mut n = d.du.cross(d.dv).normalize_or_zero();
        if n.length_squared() < 1e-12 {
            n = DVec3::Z;
        }
        let mut u_dir = d.du.normalize_or_zero();
        if u_dir.length_squared() < 1e-12 {
            let ref_v = if n.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
            u_dir = ref_v.cross(n).normalize_or_zero();
        }
        let v_dir = n.cross(u_dir).normalize_or_zero();
        u_dir = v_dir.cross(n).normalize_or_zero();
        SurfaceFrame {
            origin: d.point,
            u_dir,
            v_dir: if d.dv.length_squared() > 1e-12 && v_dir.dot(d.dv) < 0.0 {
                -v_dir
            } else {
                v_dir
            },
            normal: n,
        }
    }

    /// Gaussian, Mean, and Principal curvatures at `(u, v)`.
    pub fn curvature_at(&self, u: f64, v: f64) -> SurfaceCurvature {
        let d = self.non_degenerate_ders(u, v);
        let e = d.du.length_squared();
        let f = d.du.dot(d.dv);
        let g = d.dv.length_squared();
        let det = e * g - f * f;
        if det <= 1e-18 {
            return SurfaceCurvature {
                point: d.point,
                normal: DVec3::Z,
                gaussian: 0.0,
                mean: 0.0,
                k1: 0.0,
                k2: 0.0,
            };
        }
        let normal = d.du.cross(d.dv) / det.sqrt();
        let l = d.duu.dot(normal);
        let m = d.duv.dot(normal);
        let n = d.dvv.dot(normal);
        let gaussian = (l * n - m * m) / det;
        let mean = (e * n - 2.0 * f * m + g * l) / (2.0 * det);
        let disc = (mean * mean - gaussian).max(0.0).sqrt();
        SurfaceCurvature {
            point: d.point,
            normal,
            gaussian,
            mean,
            k1: mean + disc,
            k2: mean - disc,
        }
    }

    pub fn greville_u(&self) -> Vec<f64> {
        greville_abscissae(&self.knots_u, self.degree_u, self.count_u)
    }

    pub fn greville_v(&self) -> Vec<f64> {
        greville_abscissae(&self.knots_v, self.degree_v, self.count_v)
    }

    /// UVN orientation frame centered at control point `(u_idx, v_idx)` for Rhino 9 UVN Gumball editing.
    pub fn control_point_uvn_frame(&self, u_idx: usize, v_idx: usize) -> SurfaceFrame {
        let ui = u_idx.min(self.count_u.saturating_sub(1));
        let vj = v_idx.min(self.count_v.saturating_sub(1));
        let gu = self.greville_u();
        let gv = self.greville_v();
        let mut frame = self.surface_frame(gu[ui], gv[vj]);
        frame.origin = self.get_cp(ui, vj).position;
        if frame.u_dir.length_squared() < 1e-12 && ui + 1 < self.count_u {
            frame.u_dir = (self.get_cp(ui + 1, vj).position - frame.origin).normalize_or_zero();
        }
        if frame.u_dir.length_squared() < 1e-12 {
            frame.u_dir = DVec3::X;
        }
        if frame.normal.length_squared() < 1e-12 {
            frame.normal = DVec3::Z;
        }
        if frame.v_dir.length_squared() < 1e-12 {
            frame.v_dir = frame.normal.cross(frame.u_dir).normalize_or(DVec3::Y);
        }
        frame
    }

    /// Extract exact rational isocurve as a `NurbsCurve`. If `fixed_is_u` is true, holds U=`param` and varies V; else holds V=`param` and varies U.
    pub fn extract_isocurve(&self, fixed_is_u: bool, param: f64) -> Result<NurbsCurve, GeomError> {
        if fixed_is_u {
            let (u0, u1) = self.domain_u();
            let u = param.clamp(u0, u1);
            let span_u = find_span(self.count_u - 1, self.degree_u, u, &self.knots_u);
            let nu = basis_functions(span_u, u, self.degree_u, &self.knots_u);
            let mut cps = Vec::with_capacity(self.count_v);
            for vj in 0..self.count_v {
                let mut h = DVec4::ZERO;
                for (i, &bu) in nu.iter().enumerate() {
                    let ui = span_u - self.degree_u + i;
                    h += self.control_points[ui * self.count_v + vj].to_homogeneous() * bu;
                }
                let w = h.w.max(1e-12);
                cps.push(ControlPoint::new(h.truncate() / w, w));
            }
            NurbsCurve::new(self.degree_v, self.knots_v.clone(), cps)
        } else {
            let (v0, v1) = self.domain_v();
            let v = param.clamp(v0, v1);
            let span_v = find_span(self.count_v - 1, self.degree_v, v, &self.knots_v);
            let nv = basis_functions(span_v, v, self.degree_v, &self.knots_v);
            let mut cps = Vec::with_capacity(self.count_u);
            for ui in 0..self.count_u {
                let mut h = DVec4::ZERO;
                for (j, &bv) in nv.iter().enumerate() {
                    let vj = span_v - self.degree_v + j;
                    h += self.control_points[ui * self.count_v + vj].to_homogeneous() * bv;
                }
                let w = h.w.max(1e-12);
                cps.push(ControlPoint::new(h.truncate() / w, w));
            }
            NurbsCurve::new(self.degree_u, self.knots_u.clone(), cps)
        }
    }

    pub fn boundary_curve(&self, edge: SurfaceEdge) -> Result<NurbsCurve, GeomError> {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        match edge {
            SurfaceEdge::UMin => self.extract_isocurve(true, u0),
            SurfaceEdge::UMax => self.extract_isocurve(true, u1),
            SurfaceEdge::VMin => self.extract_isocurve(false, v0),
            SurfaceEdge::VMax => self.extract_isocurve(false, v1),
        }
    }

    pub fn boundary_curves(&self) -> Result<[NurbsCurve; 4], GeomError> {
        Ok([
            self.boundary_curve(SurfaceEdge::UMin)?,
            self.boundary_curve(SurfaceEdge::UMax)?,
            self.boundary_curve(SurfaceEdge::VMin)?,
            self.boundary_curve(SurfaceEdge::VMax)?,
        ])
    }

    /// Closest point on surface to `query`, returning `(u, v, point, distance)`.
    pub fn closest_point(&self, query: DVec3) -> (f64, f64, DVec3, f64) {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let nu = (self.count_u * 3).clamp(12, 28);
        let nv = (self.count_v * 3).clamp(12, 28);

        let mut seeds: Vec<(f64, f64, f64)> = Vec::new();
        let mut best_u = u0;
        let mut best_v = v0;
        let mut best_pt = self.evaluate(u0, v0);
        let mut best_d2 = best_pt.distance_squared(query);

        let mut grid = vec![vec![0.0_f64; nv + 1]; nu + 1];
        for iu in 0..=nu {
            let u = u0 + (u1 - u0) * (iu as f64 / nu as f64);
            for iv in 0..=nv {
                let v = v0 + (v1 - v0) * (iv as f64 / nv as f64);
                let p = self.evaluate(u, v);
                let d2 = p.distance_squared(query);
                grid[iu][iv] = d2;
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_u = u;
                    best_v = v;
                    best_pt = p;
                }
            }
        }

        for iu in 0..=nu {
            let u = u0 + (u1 - u0) * (iu as f64 / nu as f64);
            for iv in 0..=nv {
                let v = v0 + (v1 - v0) * (iv as f64 / nv as f64);
                let d2 = grid[iu][iv];
                let ok_u0 = iu == 0 || d2 <= grid[iu - 1][iv];
                let ok_u1 = iu == nu || d2 <= grid[iu + 1][iv];
                let ok_v0 = iv == 0 || d2 <= grid[iu][iv - 1];
                let ok_v1 = iv == nv || d2 <= grid[iu][iv + 1];
                if ok_u0 && ok_u1 && ok_v0 && ok_v1 {
                    seeds.push((u, v, d2));
                }
            }
        }
        seeds.sort_by(|a, b| a.2.total_cmp(&b.2));
        seeds.truncate(4);

        let span_u = (u1 - u0).max(1e-9);
        let span_v = (v1 - v0).max(1e-9);

        for (mut u, mut v, _) in seeds {
            let mut cur_pt = self.evaluate(u, v);
            let mut cur_d2 = cur_pt.distance_squared(query);
            for _ in 0..20 {
                let d = self.evaluate_derivatives(u, v);
                let r = d.point - query;
                let g1 = r.dot(d.du);
                let g2 = r.dot(d.dv);
                if g1.abs() < 1e-12 && g2.abs() < 1e-12 {
                    break;
                }
                let j11 = d.du.length_squared() + r.dot(d.duu);
                let j12 = d.du.dot(d.dv) + r.dot(d.duv);
                let j22 = d.dv.length_squared() + r.dot(d.dvv);
                // Regularized 2x2 Gauss-Newton fallback if Hessian is indefinite
                let mut h11 = j11;
                let mut h12 = j12;
                let mut h22 = j22;
                if h11 * h22 - h12 * h12 <= 1e-14 || h11 <= 0.0 {
                    h11 = d.du.length_squared() + 1e-8;
                    h12 = d.du.dot(d.dv);
                    h22 = d.dv.length_squared() + 1e-8;
                }
                let Some(delta) = solve_linear_system(&[vec![h11, h12], vec![h12, h22]], &[-g1, -g2])
                else {
                    break;
                };
                let mut du_step = delta[0].clamp(-0.25 * span_u, 0.25 * span_u);
                let mut dv_step = delta[1].clamp(-0.25 * span_v, 0.25 * span_v);
                if du_step.abs() + dv_step.abs() < 1e-12 {
                    break;
                }
                let mut accepted = false;
                for _ in 0..8 {
                    let nu = (u + du_step).clamp(u0, u1);
                    let nv = (v + dv_step).clamp(v0, v1);
                    let np = self.evaluate(nu, nv);
                    let nd2 = np.distance_squared(query);
                    if nd2 <= cur_d2 + 1e-14 {
                        u = nu;
                        v = nv;
                        cur_pt = np;
                        cur_d2 = nd2;
                        accepted = true;
                        break;
                    }
                    du_step *= 0.5;
                    dv_step *= 0.5;
                }
                if !accepted {
                    break;
                }
            }
            if cur_d2 < best_d2 {
                best_d2 = cur_d2;
                best_u = u;
                best_v = v;
                best_pt = cur_pt;
            }
        }

        (best_u, best_v, best_pt, best_d2.sqrt())
    }

    fn row_curve(&self, u_idx: usize) -> Result<NurbsCurve, GeomError> {
        let cps: Vec<ControlPoint> = (0..self.count_v)
            .map(|vj| self.get_cp(u_idx, vj))
            .collect();
        NurbsCurve::new(self.degree_v, self.knots_v.clone(), cps)
    }

    fn col_curve(&self, v_idx: usize) -> Result<NurbsCurve, GeomError> {
        let cps: Vec<ControlPoint> = (0..self.count_u)
            .map(|ui| self.get_cp(ui, v_idx))
            .collect();
        NurbsCurve::new(self.degree_u, self.knots_u.clone(), cps)
    }

    fn from_u_columns(cols: Vec<NurbsCurve>, degree_v: usize, knots_v: Vec<f64>) -> Result<Self, GeomError> {
        let count_v = cols.len();
        let Some(first) = cols.first() else {
            return Err(GeomError::Degenerate("no surface columns"));
        };
        let count_u = first.control_points.len();
        let degree_u = first.degree;
        let knots_u = first.knots.clone();
        let mut cps = vec![ControlPoint::unweighted(DVec3::ZERO); count_u * count_v];
        for (vj, col) in cols.into_iter().enumerate() {
            if col.control_points.len() != count_u {
                return Err(GeomError::Degenerate("mismatched U column count"));
            }
            for (ui, cp) in col.control_points.into_iter().enumerate() {
                cps[ui * count_v + vj] = cp;
            }
        }
        Self::new(degree_u, degree_v, knots_u, knots_v, count_u, count_v, cps)
    }

    fn from_v_rows(rows: Vec<NurbsCurve>, degree_u: usize, knots_u: Vec<f64>) -> Result<Self, GeomError> {
        let count_u = rows.len();
        let Some(first) = rows.first() else {
            return Err(GeomError::Degenerate("no surface rows"));
        };
        let count_v = first.control_points.len();
        let degree_v = first.degree;
        let knots_v = first.knots.clone();
        let mut cps = Vec::with_capacity(count_u * count_v);
        for row in rows {
            if row.control_points.len() != count_v {
                return Err(GeomError::Degenerate("mismatched V row count"));
            }
            cps.extend_from_slice(&row.control_points);
        }
        Self::new(degree_u, degree_v, knots_u, knots_v, count_u, count_v, cps)
    }

    pub fn insert_knot_u(&self, u: f64, times: usize) -> Result<Self, GeomError> {
        let mut cols = Vec::with_capacity(self.count_v);
        for vj in 0..self.count_v {
            cols.push(self.col_curve(vj)?.insert_knot(u, times)?);
        }
        Self::from_u_columns(cols, self.degree_v, self.knots_v.clone())
    }

    pub fn insert_knot_v(&self, v: f64, times: usize) -> Result<Self, GeomError> {
        let mut rows = Vec::with_capacity(self.count_u);
        for ui in 0..self.count_u {
            rows.push(self.row_curve(ui)?.insert_knot(v, times)?);
        }
        Self::from_v_rows(rows, self.degree_u, self.knots_u.clone())
    }

    pub fn elevate_degree(&self, delta_u: usize, delta_v: usize) -> Result<Self, GeomError> {
        let mut cur = self.clone();
        if delta_u > 0 {
            let mut cols = Vec::with_capacity(cur.count_v);
            for vj in 0..cur.count_v {
                cols.push(cur.col_curve(vj)?.elevate_degree(delta_u)?);
            }
            cur = Self::from_u_columns(cols, cur.degree_v, cur.knots_v.clone())?;
        }
        if delta_v > 0 {
            let mut rows = Vec::with_capacity(cur.count_u);
            for ui in 0..cur.count_u {
                rows.push(cur.row_curve(ui)?.elevate_degree(delta_v)?);
            }
            cur = Self::from_v_rows(rows, cur.degree_u, cur.knots_u.clone())?;
        }
        Ok(cur)
    }

    pub fn split_u(&self, u: f64) -> Option<(Self, Self)> {
        let mut left_cols = Vec::with_capacity(self.count_v);
        let mut right_cols = Vec::with_capacity(self.count_v);
        for vj in 0..self.count_v {
            let col = self.col_curve(vj).ok()?;
            let (l, r) = col.split_at(u)?;
            left_cols.push(l);
            right_cols.push(r);
        }
        let left = Self::from_u_columns(left_cols, self.degree_v, self.knots_v.clone()).ok()?;
        let right = Self::from_u_columns(right_cols, self.degree_v, self.knots_v.clone()).ok()?;
        Some((left, right))
    }

    pub fn split_v(&self, v: f64) -> Option<(Self, Self)> {
        let mut left_rows = Vec::with_capacity(self.count_u);
        let mut right_rows = Vec::with_capacity(self.count_u);
        for ui in 0..self.count_u {
            let row = self.row_curve(ui).ok()?;
            let (l, r) = row.split_at(v)?;
            left_rows.push(l);
            right_rows.push(r);
        }
        let left = Self::from_v_rows(left_rows, self.degree_u, self.knots_u.clone()).ok()?;
        let right = Self::from_v_rows(right_rows, self.degree_u, self.knots_u.clone()).ok()?;
        Some((left, right))
    }

    pub fn reverse_u(&self) -> Self {
        let (u0, u1) = self.domain_u();
        let knots_u: Vec<f64> = self.knots_u.iter().rev().map(|&k| u0 + u1 - k).collect();
        let mut cps = Vec::with_capacity(self.control_points.len());
        for ui in (0..self.count_u).rev() {
            for vj in 0..self.count_v {
                cps.push(self.get_cp(ui, vj));
            }
        }
        Self {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            knots_u,
            knots_v: self.knots_v.clone(),
            count_u: self.count_u,
            count_v: self.count_v,
            control_points: cps,
        }
    }

    pub fn reverse_v(&self) -> Self {
        let (v0, v1) = self.domain_v();
        let knots_v: Vec<f64> = self.knots_v.iter().rev().map(|&k| v0 + v1 - k).collect();
        let mut cps = Vec::with_capacity(self.control_points.len());
        for ui in 0..self.count_u {
            for vj in (0..self.count_v).rev() {
                cps.push(self.get_cp(ui, vj));
            }
        }
        Self {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            knots_u: self.knots_u.clone(),
            knots_v,
            count_u: self.count_u,
            count_v: self.count_v,
            control_points: cps,
        }
    }

    pub fn transpose(&self) -> Self {
        let mut cps = Vec::with_capacity(self.control_points.len());
        for vj in 0..self.count_v {
            for ui in 0..self.count_u {
                cps.push(self.get_cp(ui, vj));
            }
        }
        Self {
            degree_u: self.degree_v,
            degree_v: self.degree_u,
            knots_u: self.knots_v.clone(),
            knots_v: self.knots_u.clone(),
            count_u: self.count_v,
            count_v: self.count_u,
            control_points: cps,
        }
    }

    pub fn flip_normal(&self) -> Self {
        self.reverse_u()
    }

    pub fn area_approx(&self) -> f64 {
        let mesh = self.tessellate_display();
        let mut total = 0.0;
        for &[i0, i1, i2] in &mesh.indices {
            let p0 = DVec3::from_array(mesh.positions[i0 as usize]);
            let p1 = DVec3::from_array(mesh.positions[i1 as usize]);
            let p2 = DVec3::from_array(mesh.positions[i2 as usize]);
            total += 0.5 * (p1 - p0).cross(p2 - p0).length();
        }
        total
    }

    /// Watertight adaptive curvature-based tessellation across U and V spans.
    pub fn tessellate_adaptive(
        &self,
        max_angle_rad: f64,
        max_chord_err: f64,
        min_steps_per_span: usize,
        max_steps_per_axis: usize,
    ) -> TriangleMesh {
        let mut u_params = seed_span_params(
            &self.knots_u,
            self.degree_u,
            if self.degree_u == 1 { 1 } else { min_steps_per_span.max(1) },
        );
        let mut v_params = seed_span_params(
            &self.knots_v,
            self.degree_v,
            if self.degree_v == 1 { 1 } else { min_steps_per_span.max(1) },
        );
        let max_axis = max_steps_per_axis.max(8);

        if self.degree_u > 1 {
            for _ in 0..6 {
                let probe_v = sample_probes(&self.domain_v(), 7);
                let mut next = Vec::with_capacity(u_params.len() * 2);
                let mut split_any = false;
                for w in u_params.windows(2) {
                    next.push(w[0]);
                    if next.len() + 1 < max_axis
                        && needs_split_u(self, w[0], w[1], &probe_v, max_angle_rad, max_chord_err)
                    {
                        next.push(0.5 * (w[0] + w[1]));
                        split_any = true;
                    }
                }
                next.push(*u_params.last().unwrap());
                u_params = next;
                if !split_any {
                    break;
                }
            }
        }

        if self.degree_v > 1 {
            for _ in 0..6 {
                let probe_u = sample_probes(&self.domain_u(), 7);
                let mut next = Vec::with_capacity(v_params.len() * 2);
                let mut split_any = false;
                for w in v_params.windows(2) {
                    next.push(w[0]);
                    if next.len() + 1 < max_axis
                        && needs_split_v(self, w[0], w[1], &probe_u, max_angle_rad, max_chord_err)
                    {
                        next.push(0.5 * (w[0] + w[1]));
                        split_any = true;
                    }
                }
                next.push(*v_params.last().unwrap());
                v_params = next;
                if !split_any {
                    break;
                }
            }
        }

        self.tessellate_at_params(&u_params, &v_params)
    }

    pub fn tessellate_display(&self) -> TriangleMesh {
        let diag = self.bounding_box().size().length().max(1.0);
        self.tessellate_adaptive(8.0_f64.to_radians(), diag * 0.002, 2, 64)
    }

    fn tessellate_at_params(&self, u_params: &[f64], v_params: &[f64]) -> TriangleMesh {
        let nu = u_params.len();
        let nv = v_params.len();
        let mut positions = Vec::with_capacity(nu * nv);
        let mut normals = Vec::with_capacity(nu * nv);

        for &u in u_params {
            for &v in v_params {
                let (p, n) = self.evaluate_point_and_normal(u, v);
                positions.push(p.to_array());
                normals.push(n.to_array());
            }
        }

        let mut indices = Vec::with_capacity((nu - 1) * (nv - 1) * 2);
        let stride = nv;
        for iu in 0..(nu - 1) {
            for iv in 0..(nv - 1) {
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

    /// Tessellate the surface into a `TriangleMesh` with uniform `steps_u x steps_v` quads.
    pub fn tessellate(&self, steps_u: usize, steps_v: usize) -> TriangleMesh {
        let su = steps_u.max(1);
        let sv = steps_v.max(1);
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let u_params: Vec<f64> = (0..=su)
            .map(|iu| u0 + (u1 - u0) * (iu as f64 / su as f64))
            .collect();
        let v_params: Vec<f64> = (0..=sv)
            .map(|iv| v0 + (v1 - v0) * (iv as f64 / sv as f64))
            .collect();
        self.tessellate_at_params(&u_params, &v_params)
    }

    pub fn bounding_box(&self) -> Aabb {
        Aabb::from_points(self.control_points.iter().map(|cp| cp.position))
    }

    pub fn transform(&mut self, mat: &DMat4) {
        for cp in &mut self.control_points {
            cp.position = mat.transform_point3(cp.position);
        }
    }

    /// Ray intersection against the adaptive tessellation of the surface, returning hit distance `t >= 0`.
    pub fn ray_intersect(&self, ray: &Ray) -> Option<f64> {
        self.tessellate_display().ray_intersect(ray)
    }

    /// Control-polygon grid edges across `u` and `v` directions.
    pub fn control_net_segments(&self) -> Vec<(DVec3, DVec3)> {
        let mut segs = Vec::with_capacity(
            self.count_u * self.count_v.saturating_sub(1)
                + self.count_u.saturating_sub(1) * self.count_v,
        );
        for u in 0..self.count_u {
            for v in 0..self.count_v {
                let p = self.control_points[u * self.count_v + v].position;
                if v + 1 < self.count_v {
                    let pv = self.control_points[u * self.count_v + (v + 1)].position;
                    segs.push((p, pv));
                }
                if u + 1 < self.count_u {
                    let pu = self.control_points[(u + 1) * self.count_v + v].position;
                    segs.push((p, pu));
                }
            }
        }
        segs
    }

    /// Sample boundary and interior U/V isocurve polylines for viewport wireframe rendering.
    pub fn isocurve_polylines(
        &self,
        inner_u: usize,
        inner_v: usize,
        samples: usize,
    ) -> Vec<Vec<DVec3>> {
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        let steps = samples.max(2);
        let u_curves = inner_u + 1;
        let v_curves = inner_v + 1;
        let mut out = Vec::with_capacity(u_curves + 1 + v_curves + 1);

        for iu in 0..=u_curves {
            let u = u0 + (u1 - u0) * (iu as f64 / u_curves as f64);
            let mut pts = Vec::with_capacity(steps + 1);
            for s in 0..=steps {
                let v = v0 + (v1 - v0) * (s as f64 / steps as f64);
                pts.push(self.evaluate(u, v));
            }
            out.push(pts);
        }

        for iv in 0..=v_curves {
            let v = v0 + (v1 - v0) * (iv as f64 / v_curves as f64);
            let mut pts = Vec::with_capacity(steps + 1);
            for s in 0..=steps {
                let u = u0 + (u1 - u0) * (s as f64 / steps as f64);
                pts.push(self.evaluate(u, v));
            }
            out.push(pts);
        }

        out
    }
}

pub(crate) fn greville_abscissae(knots: &[f64], degree: usize, count: usize) -> Vec<f64> {
    (0..count)
        .map(|i| {
            let mut sum = 0.0;
            for j in 1..=degree {
                sum += knots[i + j];
            }
            sum / degree as f64
        })
        .collect()
}

fn seed_span_params(knots: &[f64], degree: usize, per_span: usize) -> Vec<f64> {
    let n = knots.len() - degree - 1;
    let u0 = knots[degree];
    let u1 = knots[n];
    let mut unique = vec![u0];
    for &k in &knots[degree..=n] {
        if k > *unique.last().unwrap() + 1e-10 && k < u1 - 1e-10 {
            unique.push(k);
        }
    }
    unique.push(u1);

    let mut params = Vec::new();
    for w in unique.windows(2) {
        for s in 0..per_span {
            let t = s as f64 / per_span as f64;
            params.push(w[0] + (w[1] - w[0]) * t);
        }
    }
    params.push(u1);
    params
}

fn sample_probes(domain: &(f64, f64), count: usize) -> Vec<f64> {
    let (a, b) = *domain;
    (0..count)
        .map(|i| a + (b - a) * (i as f64 / (count - 1).max(1) as f64))
        .collect()
}

fn needs_split_u(
    srf: &NurbsSurface,
    ua: f64,
    ub: f64,
    probe_v: &[f64],
    max_angle_rad: f64,
    max_chord_err: f64,
) -> bool {
    if (ub - ua).abs() < 1e-6 {
        return false;
    }
    let um = 0.5 * (ua + ub);
    for &v in probe_v {
        let (pa, na) = srf.evaluate_point_and_normal(ua, v);
        let (pb, nb) = srf.evaluate_point_and_normal(ub, v);
        let pm = srf.evaluate(um, v);
        if pm.distance(0.5 * (pa + pb)) > max_chord_err {
            return true;
        }
        if na.length_squared() > 0.5
            && nb.length_squared() > 0.5
            && na.angle_between(nb) > max_angle_rad
        {
            return true;
        }
    }
    false
}

fn needs_split_v(
    srf: &NurbsSurface,
    va: f64,
    vb: f64,
    probe_u: &[f64],
    max_angle_rad: f64,
    max_chord_err: f64,
) -> bool {
    if (vb - va).abs() < 1e-6 {
        return false;
    }
    let vm = 0.5 * (va + vb);
    for &u in probe_u {
        let (pa, na) = srf.evaluate_point_and_normal(u, va);
        let (pb, nb) = srf.evaluate_point_and_normal(u, vb);
        let pm = srf.evaluate(u, vm);
        if pm.distance(0.5 * (pa + pb)) > max_chord_err {
            return true;
        }
        if na.length_squared() > 0.5
            && nb.length_squared() > 0.5
            && na.angle_between(nb) > max_angle_rad
        {
            return true;
        }
    }
    false
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

    #[test]
    fn surface_ray_intersect_control_net_and_isocurves() {
        let surf = NurbsSurface::plane_rect(&Plane::world_xy(), 4.0, 2.0);
        let hit_ray = Ray::new(DVec3::new(0.5, 0.25, 5.0), DVec3::new(0.0, 0.0, -1.0));
        let t = surf.ray_intersect(&hit_ray).expect("ray should hit plane rect");
        assert!((t - 5.0).abs() < 1e-6);

        let miss_ray = Ray::new(DVec3::new(3.5, 0.0, 5.0), DVec3::new(0.0, 0.0, -1.0));
        assert!(surf.ray_intersect(&miss_ray).is_none());

        let segs = surf.control_net_segments();
        assert_eq!(segs.len(), 4);

        let isos = surf.isocurve_polylines(2, 2, 8);
        assert_eq!(isos.len(), 8);
        assert!(isos.iter().all(|poly| poly.len() == 9));

        // Flat degree 1x1 plane tessellates adaptively to exact 2 triangles
        let adaptive = surf.tessellate_display();
        assert_eq!(adaptive.indices.len(), 2);
    }

    #[test]
    fn exact_rational_primitives_derivatives_and_curvatures() {
        let center = DVec3::new(1.0, -2.0, 3.0);
        let radius = 4.0;
        let sphere = NurbsSurface::sphere(center, radius).unwrap();
        assert!(sphere.is_rational());
        assert!(sphere.is_closed_v());
        for iu in 1..6 {
            let u = iu as f64 / 6.0;
            for iv in 0..8 {
                let v = iv as f64 / 8.0;
                let (p, n) = sphere.evaluate_point_and_normal(u, v);
                assert!((p.distance(center) - radius).abs() < 1e-9);
                let outward = (p - center).normalize();
                assert!(n.dot(outward) > 0.999, "expected outward normal at ({u}, {v})");
                let k = sphere.curvature_at(u, v);
                assert!((k.gaussian - 1.0 / (radius * radius)).abs() < 1e-5);
                assert!((k.mean.abs() - 1.0 / radius).abs() < 1e-5);
            }
        }
        let adaptive_sphere = sphere.tessellate_display();
        assert!(adaptive_sphere.indices.len() > 200);
        assert!(adaptive_sphere
            .normals
            .iter()
            .all(|n| n[0].is_finite() && n[1].is_finite() && n[2].is_finite()));

        let cyl = NurbsSurface::cylinder(DVec3::ZERO, 3.0, 5.0).unwrap();
        let kc = cyl.curvature_at(0.5, 0.25);
        assert!(kc.gaussian.abs() < 1e-6);
        assert!((kc.mean.abs() - 1.0 / 6.0).abs() < 1e-5);

        let cone = NurbsSurface::cone(DVec3::ZERO, 2.0, 4.0).unwrap();
        let kcone = cone.curvature_at(0.3, 0.3);
        assert!(kcone.gaussian.abs() < 1e-5);

        let torus = NurbsSurface::torus(DVec3::ZERO, 5.0, 1.5).unwrap();
        let k_outer = torus.curvature_at(0.0, 0.0);
        let k_inner = torus.curvature_at(0.5, 0.0);
        assert!(k_outer.gaussian > 0.0);
        assert!(k_inner.gaussian < 0.0);
    }

    #[test]
    fn surface_queries_isocurve_split_insert_elevate_and_uvn_frame() {
        let mut srf = NurbsSurface::sphere(DVec3::ZERO, 5.0).unwrap();
        let (_, _, pt, dist) = srf.closest_point(DVec3::new(10.0, 0.0, 0.0));
        assert!((pt - DVec3::new(5.0, 0.0, 0.0)).length() < 1e-5);
        assert!((dist - 5.0).abs() < 1e-5);

        let iso_eq = srf.extract_isocurve(true, 0.5).unwrap();
        assert!((iso_eq.arc_length() - std::f64::consts::TAU * 5.0).abs() < 1e-3);

        let borders = srf.boundary_curves().unwrap();
        assert_eq!(borders.len(), 4);

        let refined = srf.insert_knot_u(0.35, 1).unwrap();
        let elevated = srf.elevate_degree(1, 1).unwrap();
        assert!((refined.evaluate(0.35, 0.4) - srf.evaluate(0.35, 0.4)).length() < 1e-8);
        assert!((elevated.evaluate(0.35, 0.4) - srf.evaluate(0.35, 0.4)).length() < 1e-8);

        let (left, right) = srf.split_u(0.5).unwrap();
        assert!((left.evaluate(0.5, 0.25) - right.evaluate(0.5, 0.25)).length() < 1e-9);

        let frame = srf.control_point_uvn_frame(2, 2);
        assert!((frame.u_dir.length() - 1.0).abs() < 1e-9);
        assert!((frame.v_dir.length() - 1.0).abs() < 1e-9);
        assert!((frame.normal.length() - 1.0).abs() < 1e-9);
        srf.transform(&DMat4::from_translation(DVec3::new(1.0, 0.0, 0.0)));
        assert!((srf.area_approx() - 4.0 * std::f64::consts::PI * 25.0).abs() < 4.0);
    }
}
