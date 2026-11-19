use cad_math::linalg::solve_least_squares_vec3;
use cad_math::{Continuity, DVec3, PARAM_EPSILON, Plane};

use crate::error::GeomError;
use crate::nurbs_curve::{
    ControlPoint, NurbsCurve, basis_functions, find_span, uniform_clamped_knots,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveIntersection {
    pub u_a: f64,
    pub u_b: f64,
    pub point: DVec3,
    pub distance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OffsetCornerStyle {
    Sharp,
    Round,
    Smooth,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RebuildReport {
    pub curve: NurbsCurve,
    pub old_cp_count: usize,
    pub new_cp_count: usize,
    pub max_deviation: f64,
}

/// Exact NURBS join of two curves assumed to meet at `a.end` -> `b.start`.
pub(crate) fn join_two_exact(a: &NurbsCurve, b: &NurbsCurve) -> Result<NurbsCurve, GeomError> {
    let target_deg = a.degree.max(b.degree);
    let ca = if a.degree < target_deg {
        a.elevate_degree(target_deg - a.degree)?
    } else {
        a.clone()
    };
    let cb = if b.degree < target_deg {
        b.elevate_degree(target_deg - b.degree)?
    } else {
        b.clone()
    };

    let len_a = ca.arc_length().max(1e-4);
    let len_b = cb.arc_length().max(1e-4);
    let ca = ca.reparameterize(0.0, len_a);
    let cb = cb.reparameterize(len_a, len_a + len_b);

    let w_end_a = ca.control_points.last().map(|c| c.weight).unwrap_or(1.0);
    let w_start_b = cb.control_points.first().map(|c| c.weight).unwrap_or(1.0);
    let weight_scale = if w_start_b.abs() > PARAM_EPSILON {
        w_end_a / w_start_b
    } else {
        1.0
    };

    let p = target_deg;
    let mut cps = ca.control_points.clone();
    for cp in cb.control_points.iter().skip(1) {
        cps.push(ControlPoint::new(cp.position, cp.weight * weight_scale));
    }

    // Clamp start (p+1), internal knots of ca, p copies of junction len_a, internal knots of cb, end (p+1)
    let mut knots = Vec::with_capacity(cps.len() + p + 1);
    knots.extend_from_slice(&ca.knots[..(ca.knots.len() - 1)]);
    knots.extend_from_slice(&cb.knots[(p + 1)..]);

    NurbsCurve::new(p, knots, cps)
}

/// Find intersections between two NURBS curves within `tol`.
pub fn intersect_curves(
    a: &NurbsCurve,
    b: &NurbsCurve,
    tol: f64,
) -> Vec<CurveIntersection> {
    let tol = tol.max(1e-7);
    let bb_a = a.bounding_box();
    let bb_b = b.bounding_box();
    if bb_a.max.x + tol < bb_b.min.x
        || bb_b.max.x + tol < bb_a.min.x
        || bb_a.max.y + tol < bb_b.min.y
        || bb_b.max.y + tol < bb_a.min.y
        || bb_a.max.z + tol < bb_b.min.z
        || bb_b.max.z + tol < bb_a.min.z
    {
        return Vec::new();
    }

    let (ua0, ua1) = a.domain();
    let (ub0, ub1) = b.domain();
    let na = (a.control_points.len() * 8).clamp(48, 192);
    let nb = (b.control_points.len() * 8).clamp(48, 192);

    let pts_a: Vec<(f64, DVec3)> = (0..=na)
        .map(|i| {
            let u = ua0 + (ua1 - ua0) * (i as f64 / na as f64);
            (u, a.evaluate(u))
        })
        .collect();
    let pts_b: Vec<(f64, DVec3)> = (0..=nb)
        .map(|i| {
            let v = ub0 + (ub1 - ub0) * (i as f64 / nb as f64);
            (v, b.evaluate(v))
        })
        .collect();

    let mut hits: Vec<CurveIntersection> = Vec::new();

    for wa in pts_a.windows(2) {
        let seg_a_min = wa[0].1.min(wa[1].1);
        let seg_a_max = wa[0].1.max(wa[1].1);
        let seg_a_len = wa[0].1.distance(wa[1].1);
        let pad_a = seg_a_len * 0.6 + tol;

        for wb in pts_b.windows(2) {
            let seg_b_min = wb[0].1.min(wb[1].1);
            let seg_b_max = wb[0].1.max(wb[1].1);
            let seg_b_len = wb[0].1.distance(wb[1].1);
            let pad = pad_a + seg_b_len * 0.6;

            if seg_a_max.x + pad < seg_b_min.x
                || seg_b_max.x + pad < seg_a_min.x
                || seg_a_max.y + pad < seg_b_min.y
                || seg_b_max.y + pad < seg_a_min.y
                || seg_a_max.z + pad < seg_b_min.z
                || seg_b_max.z + pad < seg_a_min.z
            {
                continue;
            }

            let (s0, t0, approx_d) = segment_closest_params(wa[0].1, wa[1].1, wb[0].1, wb[1].1);
            if approx_d > (seg_a_len + seg_b_len) * 0.5 + tol * 4.0 {
                continue;
            }

            let mut u = wa[0].0 + (wa[1].0 - wa[0].0) * s0;
            let mut v = wb[0].0 + (wb[1].0 - wb[0].0) * t0;

            // 2D Gauss-Newton polish minimizing ||C_a(u) - C_b(v)||^2
            for _ in 0..18 {
                let (pa, da) = a.evaluate_derivative(u);
                let (pb, db) = b.evaluate_derivative(v);
                let r = pa - pb;
                let cur_d2 = r.length_squared();
                if cur_d2 < 1e-24 {
                    break;
                }
                let j00 = da.dot(da) + 1e-14;
                let j01 = -da.dot(db);
                let j11 = db.dot(db) + 1e-14;
                let det = j00 * j11 - j01 * j01;
                if det.abs() < 1e-22 {
                    break;
                }
                let g0 = r.dot(da);
                let g1 = -r.dot(db);
                let du = -(j11 * g0 - j01 * g1) / det;
                let dv = -(-j01 * g0 + j00 * g1) / det;

                let mut step_scale = 1.0;
                let mut improved = false;
                for _ in 0..6 {
                    let un = (u + du * step_scale).clamp(ua0, ua1);
                    let vn = (v + dv * step_scale).clamp(ub0, ub1);
                    let d2 = a.evaluate(un).distance_squared(b.evaluate(vn));
                    if d2 < cur_d2 {
                        u = un;
                        v = vn;
                        improved = true;
                        break;
                    }
                    step_scale *= 0.5;
                }
                if !improved || (du.abs() + dv.abs()) * step_scale < 1e-12 {
                    break;
                }
            }

            let pa = a.evaluate(u);
            let pb = b.evaluate(v);
            let dist = pa.distance(pb);
            if dist <= tol {
                let mid = (pa + pb) * 0.5;
                let duplicate = hits.iter().any(|h| {
                    ((h.u_a - u).abs() < 1e-4 * (ua1 - ua0).max(1.0)
                        && (h.u_b - v).abs() < 1e-4 * (ub1 - ub0).max(1.0))
                        || h.point.distance(mid) <= tol.max(1e-4)
                });
                if !duplicate {
                    hits.push(CurveIntersection {
                        u_a: u,
                        u_b: v,
                        point: mid,
                        distance: dist,
                    });
                }
            }
        }
    }

    hits.sort_by(|x, y| x.u_a.total_cmp(&y.u_a));
    hits
}

fn segment_closest_params(a0: DVec3, a1: DVec3, b0: DVec3, b1: DVec3) -> (f64, f64, f64) {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let r = a0 - b0;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t) = if a <= 1e-14 && e <= 1e-14 {
        (0.0, 0.0)
    } else if a <= 1e-14 {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= 1e-14 {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > 1e-14 {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let p1 = a0 + d1 * s;
    let p2 = b0 + d2 * t;
    (s, t, p1.distance(p2))
}

impl NurbsCurve {
    /// Split the curve at multiple parameters. For closed curves split at >= 2 params, wraps across the seam.
    pub fn split_at_params(&self, params: &[f64]) -> Vec<Self> {
        let (u0, u1) = self.domain();
        let eps = (u1 - u0) * 1e-6;
        let mut clean: Vec<f64> = params
            .iter()
            .copied()
            .filter(|&u| u > u0 + eps && u < u1 - eps)
            .collect();
        clean.sort_by(|a, b| a.total_cmp(b));
        clean.dedup_by(|a, b| (*a - *b).abs() <= eps);

        if clean.is_empty() {
            return vec![self.clone()];
        }

        let mut pieces = Vec::with_capacity(clean.len() + 1);
        let mut cur = self.clone();
        for u in clean {
            if let Some((left, right)) = cur.split_at(u) {
                pieces.push(left);
                cur = right;
            }
        }
        pieces.push(cur);

        if self.is_closed() && pieces.len() >= 3 {
            let last = pieces.pop().unwrap();
            let first = pieces.remove(0);
            if let Ok(wrapped) = join_two_exact(&last, &first) {
                pieces.insert(0, wrapped);
            } else {
                pieces.insert(0, first);
                pieces.push(last);
            }
        }
        pieces
    }

    /// Offset this NURBS curve in `plane` by `distance` toward `side_pt`.
    pub fn offset_nurbs_in_plane(
        &self,
        plane: &Plane,
        distance: f64,
        side_pt: DVec3,
        corner: OffsetCornerStyle,
    ) -> Result<Self, GeomError> {
        offset_nurbs_curve_in_plane(self, plane, distance, side_pt, corner)
    }
}

pub(crate) fn offset_nurbs_curve_in_plane(
    curve: &NurbsCurve,
    plane: &Plane,
    distance: f64,
    side_pt: DVec3,
    corner: OffsetCornerStyle,
) -> Result<NurbsCurve, GeomError> {
    if !distance.is_finite() || distance.abs() < 1e-6 {
        return Err(GeomError::Degenerate("offset distance must be non-zero"));
    }

    if curve.degree == 1 {
        return offset_polyline_in_plane(curve, plane, distance, side_pt, corner);
    }

    let closed = curve.is_closed();
    let sample_count = (curve.control_points.len() * 10).clamp(48, 200);
    let (u0, u1) = curve.domain();

    // Determine offset side using closest point on curve to `side_pt`
    let (u_close, pt_close, _) = curve.closest_point(side_pt);
    let (_, deriv_close) = curve.evaluate_derivative(u_close);
    let left_close = plane.z_axis.cross(deriv_close).normalize_or_zero();
    let sign = if (side_pt - pt_close).dot(left_close) >= 0.0 {
        1.0
    } else {
        -1.0
    };
    let off = distance.abs() * sign;

    let end_i = if closed { sample_count - 1 } else { sample_count };
    let mut offset_pts = Vec::with_capacity(end_i + 1);
    for i in 0..=end_i {
        let u = u0 + (u1 - u0) * (i as f64 / sample_count as f64);
        let (p, deriv) = curve.evaluate_derivative(u);
        let n = plane.z_axis.cross(deriv).normalize_or_zero();
        offset_pts.push(p + n * off);
    }

    NurbsCurve::interpolate(&offset_pts, 3, closed)
}

fn offset_polyline_in_plane(
    curve: &NurbsCurve,
    plane: &Plane,
    distance: f64,
    side_pt: DVec3,
    corner: OffsetCornerStyle,
) -> Result<NurbsCurve, GeomError> {
    let mut pts: Vec<DVec3> = curve.control_points.iter().map(|c| c.position).collect();
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
            sign = if (side_pt - proj).dot(left) >= 0.0 {
                1.0
            } else {
                -1.0
            };
        }
    }
    let off = distance.abs() * sign;

    let seg_normal = |a: DVec3, b: DVec3| -> DVec3 {
        plane.z_axis.cross(b - a).normalize_or_zero()
    };

    if corner == OffsetCornerStyle::Round && pts.len() >= 3 {
        let mut pieces: Vec<NurbsCurve> = Vec::new();
        for i in 0..seg_count {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            let n = seg_normal(a, b) * off;
            let oa = a + n;
            let ob = b + n;
            pieces.push(NurbsCurve::line(oa, ob));

            if closed || i + 1 < seg_count {
                let v = b;
                let c = pts[(i + 2) % pts.len()];
                let n_next = seg_normal(b, c) * off;
                let next_start = b + n_next;
                let cross_z = (ob - v).cross(next_start - v).dot(plane.z_axis);
                let dot_n = n.normalize_or_zero().dot(n_next.normalize_or_zero());
                // Insert round arc when corner turns outward relative to offset side
                if dot_n < 0.999 && (cross_z * sign) < -1e-6 {
                    let mid_dir = (n + n_next).normalize_or_zero();
                    let mid_pt = v + mid_dir * distance.abs();
                    if let Ok(arc) = NurbsCurve::arc_3pt(ob, mid_pt, next_start) {
                        pieces.push(arc);
                    }
                }
            }
        }
        let joined = NurbsCurve::join_curves(&pieces, 1e-3);
        if let Some(first) = joined.into_iter().next() {
            return Ok(first);
        }
    }

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
    let poly = NurbsCurve::polyline(&out)?;
    if corner == OffsetCornerStyle::Smooth && out.len() >= 4 {
        NurbsCurve::interpolate(&out, 3, closed)
    } else {
        Ok(poly)
    }
}

/// Trim `target` against `cutters` by removing the sub-segment closest to `remove_near`.
pub fn trim_curve_with_cutters(
    target: &NurbsCurve,
    cutters: &[NurbsCurve],
    remove_near: DVec3,
    tol: f64,
) -> Option<Vec<NurbsCurve>> {
    let mut split_params = Vec::new();
    for cutter in cutters {
        for hit in intersect_curves(target, cutter, tol) {
            split_params.push(hit.u_a);
        }
    }
    if split_params.is_empty() {
        return None;
    }

    let pieces = target.split_at_params(&split_params);
    if pieces.len() <= 1 {
        return None;
    }

    let mut drop_idx = 0;
    let mut best_d = f64::INFINITY;
    for (idx, piece) in pieces.iter().enumerate() {
        let (_, _, d) = piece.closest_point(remove_near);
        if d < best_d {
            best_d = d;
            drop_idx = idx;
        }
    }

    let kept: Vec<NurbsCurve> = pieces
        .into_iter()
        .enumerate()
        .filter_map(|(idx, p)| (idx != drop_idx).then_some(p))
        .collect();
    Some(kept)
}

/// Fillet two arbitrary NURBS curves in `plane` with `radius`, keeping the sides near `pick_a` and `pick_b`.
pub fn fillet_two_curves(
    a: &NurbsCurve,
    b: &NurbsCurve,
    pick_a: DVec3,
    pick_b: DVec3,
    radius: f64,
    plane: &Plane,
    tol: f64,
) -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    if !radius.is_finite() || radius <= 1e-6 {
        return None;
    }
    if a.degree == 1
        && a.control_points.len() == 2
        && b.degree == 1
        && b.control_points.len() == 2
    {
        if let Some(res) = NurbsCurve::fillet_two_lines(
            a.control_points[0].position,
            a.control_points[1].position,
            b.control_points[0].position,
            b.control_points[1].position,
            radius,
        ) {
            return Some(res);
        }
    }

    let (ua_mid, _) = a.domain();
    let (_, da) = a.evaluate_derivative(ua_mid);
    let pa = a.evaluate(ua_mid);
    let na = plane.z_axis.cross(da).normalize_or_zero();

    let (ub_mid, _) = b.domain();
    let (_, db) = b.evaluate_derivative(ub_mid);
    let pb = b.evaluate(ub_mid);
    let nb = plane.z_axis.cross(db).normalize_or_zero();

    let off_a_pos = a.offset_nurbs_in_plane(plane, radius, pa + na * radius, OffsetCornerStyle::Sharp).ok()?;
    let off_a_neg = a.offset_nurbs_in_plane(plane, radius, pa - na * radius, OffsetCornerStyle::Sharp).ok()?;
    let off_b_pos = b.offset_nurbs_in_plane(plane, radius, pb + nb * radius, OffsetCornerStyle::Sharp).ok()?;
    let off_b_neg = b.offset_nurbs_in_plane(plane, radius, pb - nb * radius, OffsetCornerStyle::Sharp).ok()?;

    let mid_pick = (pick_a + pick_b) * 0.5;
    let check_tol = tol * 20.0 + radius * 0.05;
    let mut best_center: Option<(DVec3, f64, DVec3, f64, DVec3, f64)> = None;

    for oa in [&off_a_pos, &off_a_neg] {
        for ob in [&off_b_pos, &off_b_neg] {
            for hit in intersect_curves(oa, ob, tol.max(1e-3)) {
                let center = hit.point;
                let (ua, ta, da_proj) = a.closest_point(center);
                let (ub, tb, db_proj) = b.closest_point(center);
                if (da_proj - radius).abs() > check_tol || (db_proj - radius).abs() > check_tol {
                    continue;
                }
                if ta.distance(tb) < 1e-5 {
                    continue;
                }
                let score = center.distance(mid_pick);
                if best_center.is_none_or(|b_cur| score < b_cur.5) {
                    best_center = Some((center, ua, ta, ub, tb, score));
                }
            }
        }
    }

    let (center, ua, ta, ub, tb, _) = best_center?;

    let trimmed_a = split_keep_side(a, ua, pick_a);
    let trimmed_b = split_keep_side(b, ub, pick_b);

    let ra = (ta - center).normalize_or_zero();
    let rb = (tb - center).normalize_or_zero();
    let bis = (ra + rb).normalize_or_zero();
    let mid_dir = if bis.length_squared() > 1e-8 {
        bis
    } else {
        plane.z_axis.cross(ra).normalize_or_zero()
    };
    let mid_arc = center + mid_dir * radius;
    let arc = NurbsCurve::arc_3pt(ta, mid_arc, tb).ok()?;

    Some((trimmed_a, trimmed_b, arc))
}

fn split_keep_side(curve: &NurbsCurve, u_split: f64, keep_near: DVec3) -> NurbsCurve {
    if let Some((left, right)) = curve.split_at(u_split) {
        let (_, _, dl) = left.closest_point(keep_near);
        let (_, _, dr) = right.closest_point(keep_near);
        if dl <= dr { left } else { right }
    } else {
        curve.clone()
    }
}

/// Chamfer two curves at their intersection by walking back `dist_a` and `dist_b` toward `pick_a`/`pick_b`.
pub fn chamfer_two_curves(
    a: &NurbsCurve,
    b: &NurbsCurve,
    pick_a: DVec3,
    pick_b: DVec3,
    dist_a: f64,
    dist_b: f64,
    tol: f64,
) -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    if !dist_a.is_finite() || !dist_b.is_finite() || dist_a <= 1e-6 || dist_b <= 1e-6 {
        return None;
    }

    // Fast path for two straight line segments (even if they meet by extension)
    if a.degree == 1
        && a.control_points.len() == 2
        && b.degree == 1
        && b.control_points.len() == 2
    {
        let a0 = a.control_points[0].position;
        let a1 = a.control_points[1].position;
        let b0 = b.control_points[0].position;
        let b1 = b.control_points[1].position;
        let da = (a1 - a0).normalize_or_zero();
        let db = (b1 - b0).normalize_or_zero();
        let a_dot_b = da.dot(db);
        let denom = 1.0 - a_dot_b * a_dot_b;
        if denom.abs() > 1e-8 {
            let r = a0 - b0;
            let s = (a_dot_b * db.dot(r) - da.dot(r)) / denom;
            let t = (db.dot(r) - a_dot_b * da.dot(r)) / denom;
            let pa = a0 + da * s;
            let pb = b0 + db * t;
            if pa.distance(pb) <= 1e-2 {
                let corner = (pa + pb) * 0.5;
                let a_far = if a0.distance(corner) >= a1.distance(corner) { a0 } else { a1 };
                let b_far = if b0.distance(corner) >= b1.distance(corner) { b0 } else { b1 };
                if dist_a < a_far.distance(corner) && dist_b < b_far.distance(corner) {
                    let ca = corner + (a_far - corner).normalize() * dist_a;
                    let cb = corner + (b_far - corner).normalize() * dist_b;
                    return Some((
                        NurbsCurve::line(a_far, ca),
                        NurbsCurve::line(b_far, cb),
                        NurbsCurve::line(ca, cb),
                    ));
                }
            }
        }
    }

    let hits = intersect_curves(a, b, tol.max(1e-3));
    let mid_pick = (pick_a + pick_b) * 0.5;
    let hit = hits
        .into_iter()
        .min_by(|h1, h2| h1.point.distance(mid_pick).total_cmp(&h2.point.distance(mid_pick)))?;

    let (ua0, ua1) = a.domain();
    let (u_pick_a, _, _) = a.closest_point(pick_a);
    let len_hit_a = a.arc_length_between(ua0, hit.u_a);
    let target_len_a = if u_pick_a >= hit.u_a {
        len_hit_a + dist_a
    } else {
        len_hit_a - dist_a
    };
    let u_trim_a = a.parameter_at_arc_length(target_len_a).clamp(ua0, ua1);
    let trimmed_a = split_keep_side(a, u_trim_a, pick_a);

    let (ub0, ub1) = b.domain();
    let (u_pick_b, _, _) = b.closest_point(pick_b);
    let len_hit_b = b.arc_length_between(ub0, hit.u_b);
    let target_len_b = if u_pick_b >= hit.u_b {
        len_hit_b + dist_b
    } else {
        len_hit_b - dist_b
    };
    let u_trim_b = b.parameter_at_arc_length(target_len_b).clamp(ub0, ub1);
    let trimmed_b = split_keep_side(b, u_trim_b, pick_b);

    let pa = a.evaluate(u_trim_a);
    let pb = b.evaluate(u_trim_b);
    Some((trimmed_a, trimmed_b, NurbsCurve::line(pa, pb)))
}

/// Create a Class-A NURBS blend curve connecting `a` and `b` with `G0`/`G1`/`G2`/`G3` continuity.
pub fn blend_curves(
    a: &NurbsCurve,
    at_end_a: bool,
    b: &NurbsCurve,
    at_end_b: bool,
    continuity: Continuity,
    bulge_a: f64,
    bulge_b: f64,
) -> Result<NurbsCurve, GeomError> {
    let (ua0, ua1) = a.domain();
    let ua = if at_end_a { ua1 } else { ua0 };
    let sign_a = if at_end_a { 1.0 } else { -1.0 };
    let da = a.evaluate_derivatives(ua, 2);
    let p0 = da[0];
    let t0 = (da[1] * sign_a).normalize_or_zero();
    let (k0, _) = a.curvature_at(ua);

    let (ub0, ub1) = b.domain();
    let ub = if at_end_b { ub1 } else { ub0 };
    let sign_b = if at_end_b { -1.0 } else { 1.0 };
    let db = b.evaluate_derivatives(ub, 2);
    let q0 = db[0];
    let t1_in = (db[1] * sign_b).normalize_or_zero();
    let (k1, _) = b.curvature_at(ub);

    let chord = p0.distance(q0);
    if chord < 1e-8 {
        return Err(GeomError::Degenerate("blend endpoints are coincident"));
    }

    if continuity == Continuity::G0 || t0.length_squared() < 1e-12 || t1_in.length_squared() < 1e-12
    {
        return Ok(NurbsCurve::line(p0, q0));
    }

    let alpha = chord * bulge_a.clamp(0.05, 5.0);
    let beta = chord * bulge_b.clamp(0.05, 5.0);

    match continuity {
        Continuity::G0 => Ok(NurbsCurve::line(p0, q0)),
        Continuity::G1 => {
            let p1 = p0 + t0 * (alpha / 3.0);
            let p2 = q0 - t1_in * (beta / 3.0);
            NurbsCurve::new(
                3,
                vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    ControlPoint::unweighted(p0),
                    ControlPoint::unweighted(p1),
                    ControlPoint::unweighted(p2),
                    ControlPoint::unweighted(q0),
                ],
            )
        }
        Continuity::G2 => {
            // Degree 5 Bézier: C'(0) = 5(P1 - P0) = alpha * T0
            // C''(0) = 20(P2 - 2 P1 + P0) = alpha^2 * k0  =>  P2 = P1 + (alpha/5)*T0 + (alpha^2/20)*k0
            let p1 = p0 + t0 * (alpha / 5.0);
            let p2 = p1 + t0 * (alpha / 5.0) + k0 * (alpha * alpha / 20.0);

            let p4 = q0 - t1_in * (beta / 5.0);
            let p3 = p4 - t1_in * (beta / 5.0) + k1 * (beta * beta / 20.0);

            NurbsCurve::new(
                5,
                vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                vec![
                    ControlPoint::unweighted(p0),
                    ControlPoint::unweighted(p1),
                    ControlPoint::unweighted(p2),
                    ControlPoint::unweighted(p3),
                    ControlPoint::unweighted(p4),
                    ControlPoint::unweighted(q0),
                ],
            )
        }
        Continuity::G3 => {
            // Degree 7 Bézier preserving exact G2 curvature + smooth interior transition
            let p1 = p0 + t0 * (alpha / 7.0);
            let p2 = p1 + t0 * (alpha / 7.0) + k0 * (alpha * alpha / 42.0);
            let p6 = q0 - t1_in * (beta / 7.0);
            let p5 = p6 - t1_in * (beta / 7.0) + k1 * (beta * beta / 42.0);
            let p3 = p2.lerp(p5, 0.35) + (p2 - p1) * 0.45;
            let p4 = p5.lerp(p2, 0.35) + (p5 - p6) * 0.45;

            NurbsCurve::new(
                7,
                vec![
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                ],
                vec![
                    ControlPoint::unweighted(p0),
                    ControlPoint::unweighted(p1),
                    ControlPoint::unweighted(p2),
                    ControlPoint::unweighted(p3),
                    ControlPoint::unweighted(p4),
                    ControlPoint::unweighted(p5),
                    ControlPoint::unweighted(p6),
                    ControlPoint::unweighted(q0),
                ],
            )
        }
    }
}

/// Adjust the endpoint control points of `target` so it meets `reference` with `continuity`.
pub fn match_curve(
    target: &NurbsCurve,
    at_end_target: bool,
    reference: &NurbsCurve,
    at_end_ref: bool,
    continuity: Continuity,
) -> Result<NurbsCurve, GeomError> {
    let needed_deg = match continuity {
        Continuity::G0 => 1,
        Continuity::G1 => 2,
        Continuity::G2 | Continuity::G3 => 3,
    };
    let mut work = if target.degree < needed_deg {
        target.elevate_degree(needed_deg - target.degree)?
    } else {
        target.clone()
    };

    let min_cps = match continuity {
        Continuity::G0 => 2,
        Continuity::G1 => 4,
        Continuity::G2 | Continuity::G3 => 6,
    };
    while work.control_points.len() < min_cps {
        let (u0, u1) = work.domain();
        let mid = 0.5 * (u0 + u1);
        let next = work.insert_knot(mid, 1)?;
        if next.control_points.len() == work.control_points.len() {
            let q = u0 + (u1 - u0) * 0.25;
            work = work.insert_knot(q, 1)?;
        } else {
            work = next;
        }
    }

    let (ur0, ur1) = reference.domain();
    let ur = if at_end_ref { ur1 } else { ur0 };
    let sign_ref = if at_end_ref { 1.0 } else { -1.0 };
    let ref_ders = reference.evaluate_derivatives(ur, 2);
    let q_pos = ref_ders[0];
    let q_tan = (ref_ders[1] * sign_ref).normalize_or_zero();
    let (q_curv, _) = reference.curvature_at(ur);

    let oriented = if at_end_target {
        work.reverse()
    } else {
        work
    };
    let mut cps = oriented.control_points.clone();
    let p = oriented.degree;
    let knots = &oriented.knots;

    // Orient target outward from matched start so matched start tangent equals `q_tan`
    let old_start_tan = {
        let (_, d1) = oriented.evaluate_derivative(knots[p]);
        d1
    };
    let t_dir = if q_tan.length_squared() > 1e-12 {
        if q_tan.dot(old_start_tan) >= 0.0 {
            q_tan
        } else {
            -q_tan
        }
    } else {
        old_start_tan.normalize_or_zero()
    };

    // Unweighted start adjustment for G0/G1/G2
    let k0 = knots[p];
    let d1_span = (knots[p + 1] - k0).max(1e-6);
    cps[0] = ControlPoint::unweighted(q_pos);

    if matches!(continuity, Continuity::G1 | Continuity::G2 | Continuity::G3) && t_dir.length_squared() > 1e-12 {
        let len1 = cps[1].position.distance(q_pos).max(oriented.arc_length() * 0.08);
        cps[1] = ControlPoint::unweighted(q_pos + t_dir * len1);

        if matches!(continuity, Continuity::G2 | Continuity::G3) && p >= 2 && cps.len() >= 3 {
            let d2_span = (knots[p + 2] - k0).max(d1_span);
            let speed = (p as f64 / d1_span) * len1;
            // C''(0) = [p(p-1) / d1_span] * ( (P2 - P1)/d2_span - (P1 - P0)/d1_span )
            // Set parallel part from current P2 projection and match normal part to speed^2 * q_curv
            let delta1 = (cps[1].position - cps[0].position) / d1_span;
            let cur_p2_rel = cps[2].position - cps[1].position;
            let cur_along = cur_p2_rel.dot(t_dir).max(len1 * 0.25);
            let factor = (p * (p - 1)) as f64 / d1_span;
            let c2_target_perp = q_curv * (speed * speed);
            let p2_perp = c2_target_perp * (d2_span / factor);
            let p2_tangent = t_dir * (delta1.dot(t_dir) * d2_span);
            let _ = cur_along;
            cps[2] = ControlPoint::unweighted(cps[1].position + p2_tangent + p2_perp);
        }
    }

    let updated = NurbsCurve::new(p, knots.clone(), cps)?;
    Ok(if at_end_target {
        updated.reverse()
    } else {
        updated
    })
}

/// Uniform least-squares NURBS curve rebuild with `target_cp_count` control points and `target_degree`.
pub fn rebuild_uniform(
    curve: &NurbsCurve,
    target_cp_count: usize,
    target_degree: usize,
) -> Result<RebuildReport, GeomError> {
    let old_cp_count = curve.control_points.len();
    let cp_count = target_cp_count.max(2);
    let p = target_degree.clamp(1, cp_count - 1);
    let closed = curve.is_closed();

    let n_samples = (cp_count * 6).max(64);
    let (u0, u1) = curve.domain();
    let sample_pts: Vec<DVec3> = (0..=n_samples)
        .map(|i| {
            let t = i as f64 / n_samples as f64;
            curve.evaluate(u0 + (u1 - u0) * t)
        })
        .collect();

    let rebuilt = fit_curve_with_knots(
        &sample_pts,
        p,
        uniform_clamped_knots(cp_count, p),
        closed,
        false,
        None,
        None,
    )?;

    let max_deviation = max_curve_deviation(curve, &rebuilt, n_samples);
    Ok(RebuildReport {
        curve: rebuilt,
        old_cp_count,
        new_cp_count: cp_count,
        max_deviation,
    })
}

/// Rhino 9 *Elmo*-inspired adaptive curve rebuild: fits minimal control points within `max_deviation`.
pub fn rebuild_adaptive(
    curve: &NurbsCurve,
    target_degree: usize,
    max_deviation: f64,
    preserve_kinks: bool,
    preserve_end_tangents: bool,
) -> Result<RebuildReport, GeomError> {
    let old_cp_count = curve.control_points.len();
    let tol = max_deviation.max(1e-6);

    let spans: Vec<NurbsCurve> = if preserve_kinks {
        curve.explode_segments()
    } else {
        vec![curve.clone()]
    };

    let mut rebuilt_spans = Vec::with_capacity(spans.len());
    for span in &spans {
        let fitted = rebuild_span_adaptive(span, target_degree, tol, preserve_end_tangents)?;
        rebuilt_spans.push(fitted);
    }

    let mut acc = rebuilt_spans.remove(0);
    for seg in rebuilt_spans {
        acc = join_two_exact(&acc, &seg)?;
    }

    let new_cp_count = acc.control_points.len();
    let max_dev = max_curve_deviation(curve, &acc, 120);
    Ok(RebuildReport {
        curve: acc,
        old_cp_count,
        new_cp_count,
        max_deviation: max_dev,
    })
}

fn rebuild_span_adaptive(
    span: &NurbsCurve,
    target_degree: usize,
    max_deviation: f64,
    preserve_end_tangents: bool,
) -> Result<NurbsCurve, GeomError> {
    let p = target_degree.clamp(1, 7);
    if span.degree == 1 && span.control_points.len() == 2 && p == 1 {
        return Ok(span.clone());
    }

    let (u0, u1) = span.domain();
    let n_samples = 72_usize;
    let sample_pts: Vec<DVec3> = (0..=n_samples)
        .map(|i| span.evaluate(u0 + (u1 - u0) * (i as f64 / n_samples as f64)))
        .collect();

    let (_, d0) = span.evaluate_derivative(u0);
    let (_, d1) = span.evaluate_derivative(u1);
    let t0 = (preserve_end_tangents && d0.length_squared() > 1e-12).then(|| d0.normalize());
    let t1 = (preserve_end_tangents && d1.length_squared() > 1e-12).then(|| d1.normalize());

    let min_cps = if preserve_end_tangents && p >= 2 {
        (p + 1).max(4)
    } else {
        p + 1
    };
    let mut knots = uniform_clamped_knots(min_cps, p);
    let mut best_curve = fit_curve_with_knots(&sample_pts, p, knots.clone(), false, true, t0, t1)?;

    for _iter in 0..48 {
        let (err, worst_t) = worst_sample_deviation(&sample_pts, &best_curve);
        if err <= max_deviation {
            break;
        }
        let cp_count = knots.len() - p - 1;
        if cp_count >= 64 {
            break;
        }
        // Insert knot near worst parameter (avoid colliding with existing knots)
        let insert_u = nudge_away_from_knots(worst_t, &knots);
        let pos = knots.partition_point(|&k| k < insert_u);
        knots.insert(pos, insert_u);
        if let Ok(candidate) = fit_curve_with_knots(&sample_pts, p, knots.clone(), false, true, t0, t1) {
            best_curve = candidate;
        } else {
            break;
        }
    }

    Ok(best_curve)
}

fn nudge_away_from_knots(mut u: f64, knots: &[f64]) -> f64 {
    u = u.clamp(0.01, 0.99);
    for &k in knots {
        if (u - k).abs() < 1e-3 {
            u = if u < 0.5 { u + 2e-3 } else { u - 2e-3 };
        }
    }
    u.clamp(0.005, 0.995)
}

fn fit_curve_with_knots(
    sample_pts: &[DVec3],
    degree: usize,
    knots: Vec<f64>,
    closed: bool,
    pin_ends: bool,
    tan_start: Option<DVec3>,
    tan_end: Option<DVec3>,
) -> Result<NurbsCurve, GeomError> {
    let m = sample_pts.len() - 1;
    let n_cp = knots.len() - degree - 1;
    if n_cp < degree + 1 {
        return Err(GeomError::InsufficientControlPoints {
            degree,
            found: n_cp,
            needed: degree + 1,
        });
    }

    let mut params = vec![0.0_f64; m + 1];
    let mut total = 0.0;
    for i in 0..m {
        total += sample_pts[i].distance(sample_pts[i + 1]);
        params[i + 1] = total;
    }
    if total < 1e-12 {
        return Err(GeomError::Degenerate("curve has zero length"));
    }
    for t in &mut params {
        *t /= total;
    }
    params[m] = 1.0;

    let mut design = vec![vec![0.0; n_cp]; sample_pts.len()];
    let mut rhs = sample_pts.to_vec();

    for (row_idx, &u) in params.iter().enumerate() {
        let span = find_span(n_cp - 1, degree, u, &knots);
        let basis = basis_functions(span, u, degree, &knots);
        for (i, &b) in basis.iter().enumerate() {
            design[row_idx][span - degree + i] = b;
        }
    }

    if pin_ends {
        for _ in 0..8 {
            let mut end_row_0 = vec![0.0; n_cp];
            end_row_0[0] = 100.0;
            design.push(end_row_0);
            rhs.push(sample_pts[0] * 100.0);

            let mut end_row_1 = vec![0.0; n_cp];
            end_row_1[n_cp - 1] = 100.0;
            design.push(end_row_1);
            rhs.push(sample_pts[m] * 100.0);
        }
    }

    let mut cps = solve_least_squares_vec3(&design, &rhs, 1e-9)
        .ok_or(GeomError::Degenerate("least-squares rebuild failed"))?;

    if pin_ends {
        cps[0] = sample_pts[0];
        cps[n_cp - 1] = sample_pts[m];
    }
    if closed && n_cp >= 3 {
        let mid = (cps[0] + cps[n_cp - 1]) * 0.5;
        cps[0] = mid;
        cps[n_cp - 1] = mid;
    }
    if let Some(ts) = tan_start {
        if n_cp >= 4 {
            let d = (cps[1] - cps[0]).length().max(total * 0.05);
            let dir = if ts.dot(cps[1] - cps[0]) >= 0.0 { ts } else { -ts };
            cps[1] = cps[0] + dir * d;
        }
    }
    if let Some(te) = tan_end {
        if n_cp >= 4 {
            let d = (cps[n_cp - 1] - cps[n_cp - 2]).length().max(total * 0.05);
            let dir = if te.dot(cps[n_cp - 1] - cps[n_cp - 2]) >= 0.0 {
                te
            } else {
                -te
            };
            cps[n_cp - 2] = cps[n_cp - 1] - dir * d;
        }
    }

    let control_points = cps.into_iter().map(ControlPoint::unweighted).collect();
    NurbsCurve::new(degree, knots, control_points)
}

fn worst_sample_deviation(sample_pts: &[DVec3], candidate: &NurbsCurve) -> (f64, f64) {
    let mut worst_err = 0.0;
    let mut worst_t = 0.5;
    let m = sample_pts.len() - 1;
    for (i, &p) in sample_pts.iter().enumerate() {
        let (u, _, d) = candidate.closest_point(p);
        if d > worst_err {
            worst_err = d;
            let t_hint = i as f64 / m as f64;
            worst_t = 0.5 * (u + t_hint);
        }
    }
    (worst_err, worst_t)
}

fn max_curve_deviation(orig: &NurbsCurve, rebuilt: &NurbsCurve, samples: usize) -> f64 {
    let (u0, u1) = orig.domain();
    let mut max_d = 0.0_f64;
    for i in 0..=samples {
        let u = u0 + (u1 - u0) * (i as f64 / samples as f64);
        let p = orig.evaluate(u);
        let (_, _, d) = rebuilt.closest_point(p);
        max_d = max_d.max(d);
    }
    max_d
}

/// Fair/smooth a curve by relaxing interior control points while pinning endpoints.
pub fn fair_curve(curve: &NurbsCurve, strength: f64, iterations: usize) -> NurbsCurve {
    let n = curve.control_points.len();
    if n <= 2 || iterations == 0 {
        return curve.clone();
    }
    let alpha = strength.clamp(0.05, 0.9);
    let mut out = curve.clone();
    let start_idx = if n >= 5 { 2 } else { 1 };
    let end_idx = if n >= 5 { n - 2 } else { n - 1 };

    for _ in 0..iterations {
        let prev = out.control_points.clone();
        for i in start_idx..end_idx {
            let avg = (prev[i - 1].position + prev[i + 1].position) * 0.5;
            out.control_points[i].position = prev[i].position.lerp(avg, alpha);
        }
    }
    out
}

/// Generate `count` intermediate tweened NURBS curves between `a` and `b`.
pub fn tween_curves(a: &NurbsCurve, b: &NurbsCurve, count: usize) -> Vec<NurbsCurve> {
    if count == 0 {
        return Vec::new();
    }
    let deg = a.degree.max(b.degree).max(3);
    let cp_target = a.control_points.len().max(b.control_points.len()).max(deg + 1);
    let Ok(ra) = rebuild_uniform(a, cp_target, deg) else {
        return Vec::new();
    };
    let Ok(rb) = rebuild_uniform(b, cp_target, deg) else {
        return Vec::new();
    };

    let ca = ra.curve;
    let mut cb = rb.curve;
    let (ua0, ua1) = ca.domain();
    let (ub0, ub1) = cb.domain();
    let direct = ca.evaluate(ua0).distance(cb.evaluate(ub0))
        + ca.evaluate(ua1).distance(cb.evaluate(ub1));
    let flipped = ca.evaluate(ua0).distance(cb.evaluate(ub1))
        + ca.evaluate(ua1).distance(cb.evaluate(ub0));
    if flipped < direct {
        cb = cb.reverse();
    }

    let mut out = Vec::with_capacity(count);
    for k in 1..=count {
        let t = k as f64 / (count + 1) as f64;
        let cps: Vec<ControlPoint> = ca
            .control_points
            .iter()
            .zip(cb.control_points.iter())
            .map(|(pa, pb)| ControlPoint::unweighted(pa.position.lerp(pb.position, t)))
            .collect();
        if let Ok(c) = NurbsCurve::new(deg, ca.knots.clone(), cps) {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derivatives_curvature_arc_length_and_closest_point_on_circle() {
        let plane = Plane::world_xy();
        let circle = NurbsCurve::circle(&plane, 4.0).unwrap();
        let (_, k) = circle.curvature_at(0.125);
        assert!((k - 0.25).abs() < 1e-5, "expected kappa=0.25, got {k}");

        let len = circle.arc_length();
        assert!((len - 2.0 * std::f64::consts::PI * 4.0).abs() < 1e-4);

        let (_, pt, dist) = circle.closest_point(DVec3::new(6.0, 6.0, 0.0));
        assert!((pt.length() - 4.0).abs() < 1e-6);
        assert!((dist - (72.0_f64.sqrt() - 4.0)).abs() < 1e-5);
    }

    #[test]
    fn knot_insert_degree_elevate_and_split_preserve_rational_circle() {
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        let inserted = circle.insert_knot(0.37, 1).unwrap();
        let elevated = circle.elevate_degree(1).unwrap();
        assert_eq!(elevated.degree, 3);
        for i in 0..=40 {
            let u = i as f64 / 40.0;
            assert!(circle.evaluate(u).distance(inserted.evaluate(u)) < 1e-9);
            assert!(circle.evaluate(u).distance(elevated.evaluate(u)) < 1e-9);
        }

        let (left, right) = circle.split_at(0.5).unwrap();
        assert!((left.evaluate(0.5) - DVec3::new(-5.0, 0.0, 0.0)).length() < 1e-9);
        assert!((right.evaluate(0.5) - DVec3::new(-5.0, 0.0, 0.0)).length() < 1e-9);
        assert!((left.evaluate(0.25) - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-9);
        assert!((right.evaluate(0.75) - DVec3::new(0.0, -5.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn freeform_constructors_open_closed_conic_helix() {
        let pts = vec![
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(2.0, 3.0, 0.0),
            DVec3::new(5.0, 1.0, 0.0),
            DVec3::new(8.0, 4.0, 0.0),
            DVec3::new(10.0, 0.0, 0.0),
        ];
        let open_interp = NurbsCurve::interpolate(&pts, 3, false).unwrap();
        for p in &pts {
            let (_, _, d) = open_interp.closest_point(*p);
            assert!(d < 1e-7, "open interpolation missed point by {d}");
        }

        let loop_pts = vec![
            DVec3::new(3.0, 0.0, 0.0),
            DVec3::new(0.0, 3.0, 0.0),
            DVec3::new(-3.0, 0.0, 0.0),
            DVec3::new(0.0, -3.0, 0.0),
        ];
        let closed_interp = NurbsCurve::interpolate(&loop_pts, 3, true).unwrap();
        assert!(closed_interp.is_closed());
        for p in &loop_pts {
            let (_, _, d) = closed_interp.closest_point(*p);
            assert!(d < 1e-6, "closed interpolation missed point by {d}");
        }
        // Verify C1/C2 seam continuity at 0 vs 1
        let d_start = closed_interp.evaluate_derivatives(0.0, 2);
        let d_end = closed_interp.evaluate_derivatives(1.0, 2);
        assert!((d_start[1].normalize() - d_end[1].normalize()).length() < 1e-6);
        assert!((d_start[2] - d_end[2]).length() < 1e-5);

        let conic = NurbsCurve::conic(
            DVec3::new(-2.0, 0.0, 0.0),
            DVec3::new(0.0, 4.0, 0.0),
            DVec3::new(2.0, 0.0, 0.0),
            0.5,
        )
        .unwrap();
        assert!((conic.evaluate(0.5) - DVec3::new(0.0, 2.0, 0.0)).length() < 1e-9);

        let helix = NurbsCurve::helix(
            DVec3::ZERO,
            DVec3::new(0.0, 0.0, 6.0),
            DVec3::new(2.0, 0.0, 0.0),
            2.0,
            None,
        )
        .unwrap();
        assert!((helix.evaluate(0.0) - DVec3::new(2.0, 0.0, 0.0)).length() < 1e-6);
        assert!((helix.evaluate(1.0) - DVec3::new(2.0, 0.0, 6.0)).length() < 1e-6);
    }

    #[test]
    fn ccx_exact_join_explode_and_trim() {
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        let line = NurbsCurve::line(DVec3::new(-10.0, 3.0, 0.0), DVec3::new(10.0, 3.0, 0.0));
        let hits = intersect_curves(&circle, &line, 1e-4);
        assert_eq!(hits.len(), 2);
        for h in &hits {
            assert!((h.point.y - 3.0).abs() < 1e-4);
            assert!((h.point.x.abs() - 4.0).abs() < 1e-4);
        }

        // Exact NURBS join + explode preserves arc without polyline degradation
        let arc = NurbsCurve::arc_3pt(
            DVec3::new(4.0, 0.0, 0.0),
            DVec3::new(0.0, 4.0, 0.0),
            DVec3::new(-4.0, 0.0, 0.0),
        )
        .unwrap();
        let stem = NurbsCurve::line(DVec3::new(-4.0, 0.0, 0.0), DVec3::new(-4.0, -5.0, 0.0));
        let joined = NurbsCurve::join_curves(&[arc.clone(), stem.clone()], 1e-4);
        assert_eq!(joined.len(), 1);
        assert!(joined[0].is_rational());
        let exploded = joined[0].explode_segments();
        assert_eq!(exploded.len(), 2);

        // Trim top half of circle with horizontal line
        let trimmed = trim_curve_with_cutters(
            &circle,
            &[line],
            DVec3::new(0.0, 5.0, 0.0),
            1e-4,
        )
        .unwrap();
        assert_eq!(trimmed.len(), 1);
        assert!(trimmed[0].arc_length() > circle.arc_length() * 0.5);
    }

    #[test]
    fn spline_offset_fillet_chamfer_blend_match_and_elmo_rebuild() {
        let plane = Plane::world_xy();
        let spline = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(3.0, 2.0, 0.0),
                DVec3::new(6.0, 0.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let off = spline
            .offset_nurbs_in_plane(&plane, 1.0, DVec3::new(3.0, 5.0, 0.0), OffsetCornerStyle::Smooth)
            .unwrap();
        let (_, _, d_mid) = spline.closest_point(off.evaluate(0.5));
        assert!((d_mid - 1.0).abs() < 0.05);

        // BlendCrv G2 verifies tangent and curvature continuity at both ends
        let c1 = NurbsCurve::arc_3pt(
            DVec3::new(-6.0, 0.0, 0.0),
            DVec3::new(-3.0, 2.0, 0.0),
            DVec3::new(-1.0, 0.0, 0.0),
        )
        .unwrap();
        let c2 = NurbsCurve::line(DVec3::new(2.0, 0.0, 0.0), DVec3::new(6.0, 1.0, 0.0));
        let blend = blend_curves(&c1, true, &c2, false, Continuity::G2, 1.0, 1.0).unwrap();
        let (u0, u1) = blend.domain();
        let (_, k1_end) = c1.curvature_at(c1.domain().1);
        let (_, kb_start) = blend.curvature_at(u0);
        assert!((k1_end - kb_start).abs() < 1e-4, "G2 mismatch: {k1_end} vs {kb_start}");
        let (_, kb_end) = blend.curvature_at(u1);
        assert!(kb_end < 1e-4, "expected 0 curvature at line end, got {kb_end}");

        // Match curve G1
        let to_match = NurbsCurve::line(DVec3::new(2.0, 2.0, 0.0), DVec3::new(7.0, 2.0, 0.0));
        let matched = match_curve(&to_match, false, &c1, true, Continuity::G1).unwrap();
        let (p_m, d_m) = matched.evaluate_derivative(matched.domain().0);
        let (p_r, d_r) = c1.evaluate_derivative(c1.domain().1);
        assert!(p_m.distance(p_r) < 1e-6);
        assert!(d_m.normalize().angle_between(d_r.normalize()) < 1e-5);

        // Adaptive Elmo rebuild shrinks an over-refined curve cleanly
        let dense_line = NurbsCurve::interpolate(
            &(0..=20)
                .map(|i| {
                    let x = i as f64;
                    DVec3::new(x, (x * 0.4).sin() * 1.5, 0.0)
                })
                .collect::<Vec<_>>(),
            3,
            false,
        )
        .unwrap();
        let rep = rebuild_adaptive(&dense_line, 3, 0.02, true, true).unwrap();
        assert!(rep.max_deviation <= 0.025);
        assert!(rep.new_cp_count < rep.old_cp_count);

        // Fillet two curved arcs/splines
        let curve_a = NurbsCurve::line(DVec3::new(-5.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0));
        let curve_b = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, -3.0, 0.0),
                DVec3::new(0.0, 2.0, 0.0),
                DVec3::new(0.5, 6.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let (ta, tb, fillet_arc) = fillet_two_curves(
            &curve_a,
            &curve_b,
            DVec3::new(-4.0, 0.0, 0.0),
            DVec3::new(0.4, 5.0, 0.0),
            0.8,
            &plane,
            1e-3,
        )
        .unwrap();
        assert!(ta.arc_length() > 1.0 && tb.arc_length() > 1.0 && fillet_arc.arc_length() > 0.5);

        // Chamfer & Tween & Fair
        let (ca, cb, ch) = chamfer_two_curves(
            &curve_a,
            &curve_b,
            DVec3::new(-4.0, 0.0, 0.0),
            DVec3::new(0.4, 5.0, 0.0),
            0.5,
            0.5,
            1e-3,
        )
        .unwrap();
        assert!(ca.arc_length() > 1.0 && cb.arc_length() > 1.0 && ch.arc_length() > 0.3);

        let faired = fair_curve(&dense_line, 0.4, 3);
        assert_eq!(faired.control_points.len(), dense_line.control_points.len());

        let tweens = tween_curves(&c1, &c2, 3);
        assert_eq!(tweens.len(), 3);
    }
}
