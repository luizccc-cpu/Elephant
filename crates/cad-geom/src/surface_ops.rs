use cad_math::linalg::{solve_least_squares_vec3, solve_linear_system, solve_linear_system_vec3};
use cad_math::{Continuity, DMat4, DVec3, PARAM_EPSILON, Plane};
use serde::{Deserialize, Serialize};

use crate::curve_ops::{intersect_curves, rebuild_uniform};
use crate::error::GeomError;
use crate::nurbs_curve::{
    ControlPoint, NurbsCurve, basis_function_derivatives, basis_functions, find_span,
    uniform_clamped_knots,
};
use crate::nurbs_surface::{NurbsSurface, SurfaceEdge, greville_abscissae};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LoftStyle {
    Normal,
    Loose,
    Tight,
    Straight,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SoftTransformFalloff {
    Smooth,
    Linear,
    Sharp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BoundaryLock {
    Free,
    FixG0,
    FixG1,
    FixG2,
}

#[derive(Debug, Clone)]
pub struct PatchOptions {
    pub spans_u: usize,
    pub spans_v: usize,
    pub degree: usize,
    pub stiffness: f64,
    pub continuity: Continuity,
}

impl Default for PatchOptions {
    fn default() -> Self {
        Self {
            spans_u: 6,
            spans_v: 6,
            degree: 3,
            stiffness: 0.05,
            continuity: Continuity::G1,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PatchBoundaryConstraint {
    pub curve: NurbsCurve,
    pub reference_surface: Option<(NurbsSurface, SurfaceEdge)>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PatchReport {
    pub surface: NurbsSurface,
    pub max_deviation: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceRebuildReport {
    pub surface: NurbsSurface,
    pub old_u: usize,
    pub old_v: usize,
    pub new_u: usize,
    pub new_v: usize,
    pub max_deviation: f64,
}

/// Exact rational Piegl & Tiller A7.1 NURBS surface of revolution.
pub fn revolve_nurbs(
    profile: &NurbsCurve,
    axis_origin: DVec3,
    axis_dir: DVec3,
    angle_rad: f64,
) -> Result<NurbsSurface, GeomError> {
    let axis = axis_dir.normalize_or_zero();
    if axis.length_squared() < 1e-12 {
        return Err(GeomError::Degenerate("revolve axis cannot be zero length"));
    }
    let angle = angle_rad.clamp(-std::f64::consts::TAU, std::f64::consts::TAU);
    if !angle.is_finite() || angle.abs() < 1e-6 {
        return Err(GeomError::Degenerate("revolve angle must be non-zero"));
    }

    let narcs = (angle.abs() / std::f64::consts::FRAC_PI_2)
        .ceil()
        .clamp(1.0, 4.0) as usize;
    let dtheta = angle / narcs as f64;
    let wm = (dtheta * 0.5).cos();

    let count_v = 2 * narcs + 1;
    let mut knots_v = Vec::with_capacity(count_v + 3);
    knots_v.extend_from_slice(&[0.0, 0.0, 0.0]);
    for i in 1..narcs {
        let k = i as f64 / narcs as f64;
        knots_v.push(k);
        knots_v.push(k);
    }
    knots_v.extend_from_slice(&[1.0, 1.0, 1.0]);

    let count_u = profile.control_points.len();
    let mut cps = vec![ControlPoint::unweighted(DVec3::ZERO); count_u * count_v];

    for (ui, &pcp) in profile.control_points.iter().enumerate() {
        let rel = pcp.position - axis_origin;
        let axial = axis * rel.dot(axis);
        let radial = rel - axial;
        let r = radial.length();
        let center_on_axis = axis_origin + axial;

        if r < 1e-11 {
            for vj in 0..count_v {
                let wv = if vj % 2 == 1 { wm } else { 1.0 };
                cps[ui * count_v + vj] =
                    ControlPoint::new(center_on_axis, (pcp.weight * wv).max(1e-12));
            }
            continue;
        }

        let x_dir = radial / r;
        let y_dir = axis.cross(x_dir);

        let dir_at = |ang: f64| -> DVec3 { x_dir * ang.cos() + y_dir * ang.sin() };
        cps[ui * count_v] = ControlPoint::new(pcp.position, pcp.weight);

        for k in 0..narcs {
            let a_mid = (k as f64 + 0.5) * dtheta;
            let a_end = (k + 1) as f64 * dtheta;
            let p_mid = center_on_axis + dir_at(a_mid) * (r / wm);
            let p_end = center_on_axis + dir_at(a_end) * r;
            cps[ui * count_v + 2 * k + 1] =
                ControlPoint::new(p_mid, (pcp.weight * wm).max(1e-12));
            cps[ui * count_v + 2 * k + 2] = ControlPoint::new(p_end, pcp.weight);
        }
    }

    NurbsSurface::new(
        profile.degree,
        2,
        profile.knots.clone(),
        knots_v,
        count_u,
        count_v,
        cps,
    )
}

/// Revolve `profile` along a custom 3D `rail` curve around `(axis_origin, axis_dir)`.
pub fn rail_revolve_nurbs(
    profile: &NurbsCurve,
    rail: &NurbsCurve,
    axis_origin: DVec3,
    axis_dir: DVec3,
) -> Result<NurbsSurface, GeomError> {
    let axis = axis_dir.normalize_or_zero();
    if axis.length_squared() < 1e-12 {
        return Err(GeomError::Degenerate("rail revolve axis cannot be zero length"));
    }
    let ns = (rail.control_points.len() * 3).clamp(16, 36);
    let (r0, r1) = rail.domain();
    let p_rail0 = rail.evaluate(r0);
    let rel0 = p_rail0 - axis_origin;
    let z0 = rel0.dot(axis);
    let rad0 = rel0 - axis * z0;
    let r_ref = rad0.length();
    if r_ref < 1e-6 {
        return Err(GeomError::Degenerate("rail start lies on revolve axis"));
    }
    let ref_x = rad0 / r_ref;
    let ref_y = axis.cross(ref_x);

    let mut stations = Vec::with_capacity(ns + 1);
    let mut prev_angle = 0.0_f64;
    let closed = rail.is_closed();
    let count = if closed { ns } else { ns + 1 };

    for i in 0..count {
        let u = r0 + (r1 - r0) * (i as f64 / ns as f64);
        let pr = rail.evaluate(u);
        let rel = pr - axis_origin;
        let z = rel.dot(axis);
        let rad = rel - axis * z;
        let r_len = rad.length();
        let scale = (r_len / r_ref).max(1e-3);
        let mut raw_ang = if r_len > 1e-9 {
            rad.dot(ref_y).atan2(rad.dot(ref_x))
        } else {
            prev_angle
        };
        while raw_ang - prev_angle > std::f64::consts::PI {
            raw_ang -= std::f64::consts::TAU;
        }
        while raw_ang - prev_angle < -std::f64::consts::PI {
            raw_ang += std::f64::consts::TAU;
        }
        prev_angle = raw_ang;
        let rot = DMat4::from_axis_angle(axis, raw_ang);
        let dz = axis * (z - z0);

        let cps: Vec<ControlPoint> = profile
            .control_points
            .iter()
            .map(|&cp| {
                let d = cp.position - axis_origin;
                let a_part = axis * d.dot(axis);
                let r_part = d - a_part;
                let new_pos =
                    axis_origin + a_part + dz + rot.transform_vector3(r_part * scale);
                ControlPoint::new(new_pos, cp.weight)
            })
            .collect();
        stations.push(NurbsCurve::new(profile.degree, profile.knots.clone(), cps)?);
    }

    loft_nurbs(&stations, LoftStyle::Normal, closed)
}

/// Align directions, elevate degrees, and unify knot vectors across `curves` for lofting/sweeping.
pub fn make_curves_compatible(curves: &[NurbsCurve]) -> Result<Vec<NurbsCurve>, GeomError> {
    if curves.is_empty() {
        return Err(GeomError::Degenerate("need at least one curve"));
    }
    let mut aligned: Vec<NurbsCurve> = Vec::with_capacity(curves.len());
    aligned.push(curves[0].reparameterize(0.0, 1.0));
    for c in &curves[1..] {
        let mut norm = c.reparameterize(0.0, 1.0);
        let prev = aligned.last().unwrap();
        let same = prev.evaluate(0.0).distance(norm.evaluate(0.0))
            + prev.evaluate(0.5).distance(norm.evaluate(0.5))
            + prev.evaluate(1.0).distance(norm.evaluate(1.0));
        let flip = prev.evaluate(0.0).distance(norm.evaluate(1.0))
            + prev.evaluate(0.5).distance(norm.evaluate(0.5))
            + prev.evaluate(1.0).distance(norm.evaluate(0.0));
        if flip + 1e-9 < same {
            norm = norm.reverse().reparameterize(0.0, 1.0);
        }
        aligned.push(norm);
    }

    let max_deg = aligned.iter().map(|c| c.degree).max().unwrap_or(1);
    for c in &mut aligned {
        if c.degree < max_deg {
            *c = c.elevate_degree(max_deg - c.degree)?;
        }
    }

    // Collect merged interior knots with max multiplicity at each unique value
    let mut merged_interior: Vec<(f64, usize)> = Vec::new();
    for c in &aligned {
        let mut counts: Vec<(f64, usize)> = Vec::new();
        for &k in c.knots.iter().skip(max_deg + 1).take(c.control_points.len() - max_deg - 1) {
            if let Some(last) = counts.last_mut() {
                if (last.0 - k).abs() <= 1e-6 {
                    last.1 += 1;
                    continue;
                }
            }
            counts.push((k, 1));
        }
        for (val, mult) in counts {
            if let Some(found) = merged_interior
                .iter_mut()
                .find(|(v, _)| (*v - val).abs() <= 1e-6)
            {
                found.1 = found.1.max(mult);
            } else {
                merged_interior.push((val, mult));
            }
        }
    }
    merged_interior.sort_by(|a, b| a.0.total_cmp(&b.0));

    let extra_knots: usize = merged_interior.iter().map(|(_, m)| *m).sum();
    let max_cp = aligned
        .iter()
        .map(|c| c.control_points.len())
        .max()
        .unwrap_or(2);
    if max_deg + 1 + extra_knots > 64 {
        let target_deg = max_deg.clamp(1, 3);
        let target_cp = max_cp.clamp(target_deg + 1, 24);
        let mut rebuilt = Vec::with_capacity(aligned.len());
        for c in &aligned {
            rebuilt.push(rebuild_uniform(c, target_cp, target_deg)?.curve);
        }
        return Ok(rebuilt);
    }

    for c in &mut aligned {
        for &(kv, target_mult) in &merged_interior {
            let cur_mult = c.knots.iter().filter(|&&x| (x - kv).abs() <= 1e-6).count();
            if target_mult > cur_mult {
                *c = c.insert_knot(kv, target_mult - cur_mult)?;
            }
        }
    }

    let expected_len = aligned[0].control_points.len();
    if aligned.iter().any(|c| c.control_points.len() != expected_len) {
        let target_deg = max_deg.clamp(1, 3);
        let target_cp = max_cp.clamp(target_deg + 1, 24);
        let mut rebuilt = Vec::with_capacity(aligned.len());
        for c in &aligned {
            rebuilt.push(rebuild_uniform(c, target_cp, target_deg)?.curve);
        }
        return Ok(rebuilt);
    }

    Ok(aligned)
}

/// True NURBS skinning (`Loft`) across 2 or more curves.
pub fn loft_nurbs(
    curves: &[NurbsCurve],
    style: LoftStyle,
    closed: bool,
) -> Result<NurbsSurface, GeomError> {
    if curves.len() < 2 {
        return Err(GeomError::Degenerate("loft requires at least 2 curves"));
    }

    let mut work = make_curves_compatible(curves)?;
    let is_closed = closed && work.len() >= 3;
    if is_closed && style == LoftStyle::Straight {
        let first = work[0].clone();
        work.push(first);
    }

    let count_u = work[0].control_points.len();
    let deg_u = work[0].degree;
    let knots_u = work[0].knots.clone();

    if is_closed && style != LoftStyle::Straight && style != LoftStyle::Loose {
        return loft_closed_periodic(&work, style);
    }
    if is_closed && style == LoftStyle::Loose {
        let first = work[0].clone();
        work.push(first);
    }

    let m = work.len();
    let v_params = compute_loft_v_params(&work, style == LoftStyle::Tight);

    let (deg_v, knots_v) = match style {
        LoftStyle::Straight => {
            let mut kv = Vec::with_capacity(m + 2);
            kv.push(0.0);
            kv.extend_from_slice(&v_params);
            kv.push(1.0);
            (1, kv)
        }
        _ => {
            let p = (m - 1).clamp(1, 3);
            (p, averaged_clamped_knots(&v_params, p))
        }
    };

    if matches!(style, LoftStyle::Straight | LoftStyle::Loose) {
        let mut cps = Vec::with_capacity(count_u * m);
        for ui in 0..count_u {
            for curve in &work {
                cps.push(curve.control_points[ui]);
            }
        }
        return NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, count_u, m, cps);
    }

    // Collocation along V for each U control-point column
    let mut mat = vec![vec![0.0_f64; m]; m];
    for (row, &v) in v_params.iter().enumerate() {
        let span = find_span(m - 1, deg_v, v, &knots_v);
        let basis = basis_functions(span, v, deg_v, &knots_v);
        for (k, &b) in basis.iter().enumerate() {
            mat[row][span - deg_v + k] = b;
        }
    }

    let rational = work.iter().any(|c| c.is_rational());
    let mut cps = vec![ControlPoint::unweighted(DVec3::ZERO); count_u * m];

    for ui in 0..count_u {
        if rational {
            let hw: Vec<DVec3> = work
                .iter()
                .map(|c| c.control_points[ui].position * c.control_points[ui].weight)
                .collect();
            let weights: Vec<f64> = work.iter().map(|c| c.control_points[ui].weight).collect();
            if let (Some(sol_xyz), Some(sol_w)) = (
                solve_linear_system_vec3(&mat, &hw),
                solve_linear_system(&mat, &weights),
            ) {
                if sol_w.iter().all(|&w| w > 1e-6) {
                    for vj in 0..m {
                        cps[ui * m + vj] =
                            ControlPoint::new(sol_xyz[vj] / sol_w[vj], sol_w[vj]);
                    }
                    continue;
                }
            }
        }

        let pts: Vec<DVec3> = work.iter().map(|c| c.control_points[ui].position).collect();
        let sol = solve_linear_system_vec3(&mat, &pts)
            .ok_or(GeomError::Degenerate("singular loft collocation matrix"))?;
        for vj in 0..m {
            cps[ui * m + vj] = ControlPoint::unweighted(sol[vj]);
        }
    }

    NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, count_u, m, cps)
}

fn loft_closed_periodic(curves: &[NurbsCurve], style: LoftStyle) -> Result<NurbsSurface, GeomError> {
    let m = curves.len();
    let count_u = curves[0].control_points.len();
    let deg_u = curves[0].degree;
    let knots_u = curves[0].knots.clone();
    let p = 3_usize.min(m);
    let centripetal = style == LoftStyle::Tight;

    let mut ts = vec![0.0_f64; m + 1];
    let mut total = 0.0;
    for vj in 0..m {
        let next = (vj + 1) % m;
        let mut avg = 0.0;
        for ui in 0..count_u {
            let d = curves[vj].control_points[ui]
                .position
                .distance(curves[next].control_points[ui].position);
            avg += if centripetal { d.sqrt() } else { d };
        }
        avg /= count_u as f64;
        total += avg.max(1e-5);
        ts[vj + 1] = total;
    }
    for t in &mut ts {
        *t /= total;
    }
    ts[m] = 1.0;

    let periodic_t = |idx: isize| -> f64 {
        let m_i = m as isize;
        ts[idx.rem_euclid(m_i) as usize] + idx.div_euclid(m_i) as f64
    };
    let periodic_tau = |idx: isize| -> f64 {
        if p % 2 == 1 {
            periodic_t(idx)
        } else {
            0.5 * (periodic_t(idx - 1) + periodic_t(idx))
        }
    };

    let pad = 1_usize;
    let n_ext = m + p + 2 * pad;
    let mut ext_knots = Vec::with_capacity(n_ext + p + 1);
    for j in 0..(n_ext + p + 1) {
        ext_knots.push(periodic_tau(j as isize - (p + pad) as isize));
    }

    let mut mat = vec![vec![0.0_f64; m]; m];
    for (row, &v) in ts[..m].iter().enumerate() {
        let span = find_span(n_ext - 1, p, v, &ext_knots);
        let basis = basis_functions(span, v, p, &ext_knots);
        for (i, &b) in basis.iter().enumerate() {
            let col = ((span - p + i) as isize - pad as isize).rem_euclid(m as isize) as usize;
            mat[row][col] += b;
        }
    }

    let mut col_curves = Vec::with_capacity(count_u);
    for ui in 0..count_u {
        let pts: Vec<DVec3> = curves.iter().map(|c| c.control_points[ui].position).collect();
        let q = solve_linear_system_vec3(&mat, &pts)
            .ok_or(GeomError::Degenerate("singular closed loft matrix"))?;
        let ext_cps: Vec<ControlPoint> = (0..n_ext)
            .map(|i| {
                let idx = (i as isize - pad as isize).rem_euclid(m as isize) as usize;
                ControlPoint::unweighted(q[idx])
            })
            .collect();
        let unclamped = NurbsCurve::new(p, ext_knots.clone(), ext_cps)?;
        let (_, rest) = unclamped
            .split_at(0.0)
            .ok_or(GeomError::Degenerate("closed loft clamp start failed"))?;
        let (closed_col, _) = rest
            .split_at(1.0)
            .ok_or(GeomError::Degenerate("closed loft clamp end failed"))?;
        col_curves.push(closed_col);
    }

    let count_v = col_curves[0].control_points.len();
    let knots_v = col_curves[0].knots.clone();
    let mut cps = Vec::with_capacity(count_u * count_v);
    for col in col_curves {
        cps.extend_from_slice(&col.control_points);
    }
    NurbsSurface::new(deg_u, p, knots_u, knots_v, count_u, count_v, cps)
}

/// One-rail NURBS sweep using rotation-minimizing Bishop frames (or Roadlike frames).
pub fn sweep1(
    rail: &NurbsCurve,
    profiles: &[NurbsCurve],
    roadlike_up: Option<DVec3>,
) -> Result<NurbsSurface, GeomError> {
    if profiles.is_empty() {
        return Err(GeomError::Degenerate("sweep1 requires at least 1 profile curve"));
    }
    let rail = rail.reparameterize(0.0, 1.0);
    let compat_profiles = make_curves_compatible(profiles)?;

    let ns = (rail.control_points.len() * 3).clamp(13, 33);
    let closed_rail = rail.is_closed();
    let mut params: Vec<f64> = (0..=ns).map(|i| i as f64 / ns as f64).collect();
    let frames = build_rail_frames(&rail, &params, roadlike_up, closed_rail);

    // Match each profile to its closest parameter along rail and extract local-frame CP coordinates
    let mut attached: Vec<(f64, Vec<(DVec3, f64)>)> = Vec::with_capacity(compat_profiles.len());
    for prof in &compat_profiles {
        let centroid = prof
            .control_points
            .iter()
            .fold(DVec3::ZERO, |acc, cp| acc + cp.position)
            / prof.control_points.len() as f64;
        let p_start = prof.evaluate(prof.domain().0);
        let (u_c, _, d_c) = rail.closest_point(centroid);
        let (u_s, _, d_s) = rail.closest_point(p_start);
        let u_attach = if d_s < d_c { u_s } else { u_c };
        let (o, t, n, b) = interpolate_rail_frame(&rail, &params, &frames, u_attach, roadlike_up);
        let local_cps = prof
            .control_points
            .iter()
            .map(|cp| {
                let d = cp.position - o;
                (DVec3::new(d.dot(t), d.dot(n), d.dot(b)), cp.weight)
            })
            .collect();
        attached.push((u_attach, local_cps));
    }
    attached.sort_by(|a, b| a.0.total_cmp(&b.0));

    let deg_u = compat_profiles[0].degree;
    let knots_u = compat_profiles[0].knots.clone();
    let mut station_curves = Vec::with_capacity(params.len());

    if closed_rail {
        params.pop();
    }
    for (idx, &u) in params.iter().enumerate() {
        let (o, t, n, b) = frames[idx];
        let local_cps = blend_attached_profiles(&attached, u);
        let cps: Vec<ControlPoint> = local_cps
            .into_iter()
            .map(|(lc, w)| ControlPoint::new(o + t * lc.x + n * lc.y + b * lc.z, w))
            .collect();
        station_curves.push(NurbsCurve::new(deg_u, knots_u.clone(), cps)?);
    }

    loft_nurbs(&station_curves, LoftStyle::Normal, closed_rail)
}

/// Two-rail NURBS sweep keeping surface boundaries anchored to `rail1` and `rail2`.
pub fn sweep2(
    rail1: &NurbsCurve,
    rail2: &NurbsCurve,
    profiles: &[NurbsCurve],
) -> Result<NurbsSurface, GeomError> {
    if profiles.is_empty() {
        return Err(GeomError::Degenerate("sweep2 requires at least 1 profile curve"));
    }
    let r1 = rail1.reparameterize(0.0, 1.0);
    let mut r2 = rail2.reparameterize(0.0, 1.0);
    let direct = r1.evaluate(0.0).distance(r2.evaluate(0.0))
        + r1.evaluate(0.5).distance(r2.evaluate(0.5))
        + r1.evaluate(1.0).distance(r2.evaluate(1.0));
    let flipped = r1.evaluate(0.0).distance(r2.evaluate(1.0))
        + r1.evaluate(0.5).distance(r2.evaluate(0.5))
        + r1.evaluate(1.0).distance(r2.evaluate(0.0));
    if flipped + 1e-9 < direct {
        r2 = r2.reverse().reparameterize(0.0, 1.0);
    }

    // Orient each profile so start is closer to rail1 and end is closer to rail2 before compatibility pass
    let mut oriented_profiles = Vec::with_capacity(profiles.len());
    for p in profiles {
        let (pu0, pu1) = p.domain();
        let ps = p.evaluate(pu0);
        let pe = p.evaluate(pu1);
        let (_, _, ds1) = r1.closest_point(ps);
        let (_, _, ds2) = r2.closest_point(ps);
        let (_, _, de1) = r1.closest_point(pe);
        let (_, _, de2) = r2.closest_point(pe);
        if ds2 + de1 < ds1 + de2 {
            oriented_profiles.push(p.reverse());
        } else {
            oriented_profiles.push(p.clone());
        }
    }
    let compat_profiles = make_curves_compatible(&oriented_profiles)?;

    let ns = (r1.control_points.len().max(r2.control_points.len()) * 3).clamp(15, 33);
    let closed_rails = r1.is_closed() && r2.is_closed();
    let params: Vec<f64> = (0..=ns).map(|i| i as f64 / ns as f64).collect();
    let span_frames = build_two_rail_frames(&r1, &r2, &params);

    let mut attached: Vec<(f64, Vec<(DVec3, f64)>)> = Vec::with_capacity(compat_profiles.len());
    for prof in &compat_profiles {
        let (pu0, _) = prof.domain();
        let (u_attach, _, _) = r1.closest_point(prof.evaluate(pu0));
        let (o, e1, e2, e3, len) = interpolate_two_rail_frame(&r1, &r2, &params, &span_frames, u_attach);
        let inv_l = 1.0 / len.max(1e-6);
        let local_cps = prof
            .control_points
            .iter()
            .map(|cp| {
                let d = cp.position - o;
                (
                    DVec3::new(d.dot(e1) * inv_l, d.dot(e2) * inv_l, d.dot(e3) * inv_l),
                    cp.weight,
                )
            })
            .collect();
        attached.push((u_attach, local_cps));
    }
    attached.sort_by(|a, b| a.0.total_cmp(&b.0));

    let deg_u = compat_profiles[0].degree;
    let knots_u = compat_profiles[0].knots.clone();
    let end_idx = if closed_rails { ns } else { ns + 1 };
    let mut station_curves = Vec::with_capacity(end_idx);

    for idx in 0..end_idx {
        let u = params[idx];
        let (o, e1, e2, e3, len) = span_frames[idx];
        let local_cps = blend_attached_profiles(&attached, u);
        let n_cp = local_cps.len();
        let mut cps: Vec<ControlPoint> = local_cps
            .into_iter()
            .map(|(lc, w)| ControlPoint::new(o + (e1 * lc.x + e2 * lc.y + e3 * lc.z) * len, w))
            .collect();
        if n_cp >= 2 {
            cps[0].position = r1.evaluate(u);
            cps[n_cp - 1].position = r2.evaluate(u);
        }
        station_curves.push(NurbsCurve::new(deg_u, knots_u.clone(), cps)?);
    }

    loft_nurbs(&station_curves, LoftStyle::Normal, closed_rails)
}

/// Coons / ruled surface constructed from 2, 3, or 4 boundary curves.
pub fn edge_srf(curves: &[NurbsCurve]) -> Result<NurbsSurface, GeomError> {
    if curves.len() == 2 {
        return loft_nurbs(curves, LoftStyle::Straight, false);
    }
    if curves.len() != 3 && curves.len() != 4 {
        return Err(GeomError::Degenerate("EdgeSrf requires 2, 3, or 4 boundary curves"));
    }

    let ordered = order_boundary_loop(curves)?;
    let (v0_raw, u1_raw, v1_raw, u0_raw) = if ordered.len() == 3 {
        let c0 = ordered[0].clone();
        let c1 = ordered[1].clone();
        let c2_rev = ordered[2].reverse();
        let p0 = c0.evaluate(c0.domain().0);
        let degen = NurbsCurve::line(p0, p0);
        (c0, c1, c2_rev, degen)
    } else {
        (
            ordered[0].clone(),
            ordered[1].clone(),
            ordered[2].reverse(),
            ordered[3].reverse(),
        )
    };

    let compat_u = make_curves_compatible_no_flip(&[v0_raw, v1_raw])?;
    let compat_v = make_curves_compatible_no_flip(&[u0_raw, u1_raw])?;
    let v0 = &compat_u[0];
    let v1 = &compat_u[1];
    let u0 = &compat_v[0];
    let u1 = &compat_v[1];

    let deg_u = v0.degree;
    let deg_v = u0.degree;
    let count_u = v0.control_points.len();
    let count_v = u0.control_points.len();
    let gu = greville_abscissae(&v0.knots, deg_u, count_u);
    let gv = greville_abscissae(&u0.knots, deg_v, count_v);

    let c00 = v0.evaluate(0.0);
    let c10 = v0.evaluate(1.0);
    let c01 = v1.evaluate(0.0);
    let c11 = v1.evaluate(1.0);

    let mut grid = vec![vec![DVec3::ZERO; count_v]; count_u];
    for (iu, &u) in gu.iter().enumerate() {
        let pv0 = v0.evaluate(u);
        let pv1 = v1.evaluate(u);
        for (iv, &v) in gv.iter().enumerate() {
            let pu0 = u0.evaluate(v);
            let pu1 = u1.evaluate(v);
            let ruled_u = pv0 * (1.0 - v) + pv1 * v;
            let ruled_v = pu0 * (1.0 - u) + pu1 * u;
            let bilin = c00 * (1.0 - u) * (1.0 - v)
                + c10 * u * (1.0 - v)
                + c01 * (1.0 - u) * v
                + c11 * u * v;
            grid[iu][iv] = ruled_u + ruled_v - bilin;
        }
    }

    fit_surface_with_knots(
        &grid,
        &gu,
        &gv,
        deg_u,
        deg_v,
        v0.knots.clone(),
        u0.knots.clone(),
    )
}

/// Partition an unordered curve selection into 2 crossing curve families for `NetworkSrf`.
pub fn partition_network_curves(
    curves: &[NurbsCurve],
    tol: f64,
) -> Option<(Vec<NurbsCurve>, Vec<NurbsCurve>)> {
    let n = curves.len();
    if n < 4 {
        return None;
    }
    let mut adj = vec![Vec::<usize>::new(); n];
    for i in 0..n {
        for j in (i + 1)..n {
            if !intersect_curves(&curves[i], &curves[j], tol.max(1e-2)).is_empty() {
                adj[i].push(j);
                adj[j].push(i);
            }
        }
    }
    let mut color: Vec<Option<bool>> = vec![None; n];
    for start in 0..n {
        if color[start].is_some() {
            continue;
        }
        color[start] = Some(false);
        let mut queue = vec![start];
        while let Some(u) = queue.pop() {
            let c = color[u].unwrap();
            for &v in &adj[u] {
                match color[v] {
                    None => {
                        color[v] = Some(!c);
                        queue.push(v);
                    }
                    Some(existing) if existing == c => return None,
                    _ => {}
                }
            }
        }
    }
    let mut group_a = Vec::new();
    let mut group_b = Vec::new();
    for (i, c) in curves.iter().enumerate() {
        if color[i] == Some(false) {
            group_a.push(c.clone());
        } else {
            group_b.push(c.clone());
        }
    }
    if group_a.len() >= 2 && group_b.len() >= 2 {
        Some((group_a, group_b))
    } else {
        None
    }
}

/// Gordon surface interpolating crossing families `u_curves` and `v_curves`.
pub fn network_srf(
    u_curves: &[NurbsCurve],
    v_curves: &[NurbsCurve],
) -> Result<NurbsSurface, GeomError> {
    if u_curves.len() < 2 || v_curves.len() < 2 {
        return Err(GeomError::Degenerate(
            "NetworkSrf requires at least 2 curves in both U and V",
        ));
    }

    let mut us: Vec<NurbsCurve> = u_curves.iter().map(|c| c.reparameterize(0.0, 1.0)).collect();
    let mut vs: Vec<NurbsCurve> = v_curves.iter().map(|c| c.reparameterize(0.0, 1.0)).collect();

    // Sort us along vs[0], sort vs along us[0]
    us.sort_by(|a, b| {
        let ua = vs[0].closest_point(a.evaluate(0.5)).0;
        let ub = vs[0].closest_point(b.evaluate(0.5)).0;
        ua.total_cmp(&ub)
    });
    vs.sort_by(|a, b| {
        let va = us[0].closest_point(a.evaluate(0.5)).0;
        let vb = us[0].closest_point(b.evaluate(0.5)).0;
        va.total_cmp(&vb)
    });

    // Align orientations consistently with crossing references
    for i in 1..us.len() {
        let same = us[0].evaluate(0.0).distance(us[i].evaluate(0.0))
            + us[0].evaluate(1.0).distance(us[i].evaluate(1.0));
        let flip = us[0].evaluate(0.0).distance(us[i].evaluate(1.0))
            + us[0].evaluate(1.0).distance(us[i].evaluate(0.0));
        if flip < same {
            us[i] = us[i].reverse().reparameterize(0.0, 1.0);
        }
    }
    for j in 1..vs.len() {
        let same = vs[0].evaluate(0.0).distance(vs[j].evaluate(0.0))
            + vs[0].evaluate(1.0).distance(vs[j].evaluate(1.0));
        let flip = vs[0].evaluate(0.0).distance(vs[j].evaluate(1.0))
            + vs[0].evaluate(1.0).distance(vs[j].evaluate(0.0));
        if flip < same {
            vs[j] = vs[j].reverse().reparameterize(0.0, 1.0);
        }
    }

    let s_u = loft_nurbs(&us, LoftStyle::Normal, false)?;
    let s_v = loft_nurbs(&vs, LoftStyle::Normal, false)?.transpose();

    let nu_rows = us.len();
    let nv_cols = vs.len();
    let u_grid_params = compute_loft_v_params(&vs, false);
    let v_grid_params = compute_loft_v_params(&us, false);
    let mut inter_pts = vec![vec![DVec3::ZERO; nu_rows]; nv_cols];
    for (j, vc) in vs.iter().enumerate() {
        for (i, uc) in us.iter().enumerate() {
            let p_est = vc.evaluate(v_grid_params[i]);
            let (_, pu, _) = uc.closest_point(p_est);
            let (_, pv, _) = vc.closest_point(pu);
            inter_pts[j][i] = 0.5 * (pu + pv);
        }
    }
    let deg_iu = (nv_cols - 1).clamp(1, 3);
    let deg_iv = (nu_rows - 1).clamp(1, 3);
    let s_uv = fit_surface_grid(
        &inter_pts,
        &u_grid_params,
        &v_grid_params,
        deg_iu,
        deg_iv,
        nv_cols,
        nu_rows,
    )?;

    let out_cu = s_u.count_u.max(s_v.count_u).clamp(8, 24);
    let out_cv = s_v.count_v.max(s_u.count_v).clamp(8, 24);
    let out_du = s_u.degree_u.max(s_v.degree_u).clamp(1, 3);
    let out_dv = s_u.degree_v.max(s_v.degree_v).clamp(1, 3);

    let ku = uniform_clamped_knots(out_cu, out_du);
    let kv = uniform_clamped_knots(out_cv, out_dv);
    let gu = greville_abscissae(&ku, out_du, out_cu);
    let gv = greville_abscissae(&kv, out_dv, out_cv);

    let (su_u0, su_u1) = s_u.domain_u();
    let (su_v0, su_v1) = s_u.domain_v();
    let (sv_u0, sv_u1) = s_v.domain_u();
    let (sv_v0, sv_v1) = s_v.domain_v();
    let (suv_u0, suv_u1) = s_uv.domain_u();
    let (suv_v0, suv_v1) = s_uv.domain_v();

    let mut samples = vec![vec![DVec3::ZERO; out_cv]; out_cu];
    for (iu, &u) in gu.iter().enumerate() {
        for (iv, &v) in gv.iter().enumerate() {
            let p_u = s_u.evaluate(su_u0 + (su_u1 - su_u0) * u, su_v0 + (su_v1 - su_v0) * v);
            let p_v = s_v.evaluate(sv_u0 + (sv_u1 - sv_u0) * u, sv_v0 + (sv_v1 - sv_v0) * v);
            let p_uv = s_uv.evaluate(
                suv_u0 + (suv_u1 - suv_u0) * u,
                suv_v0 + (suv_v1 - suv_v0) * v,
            );
            samples[iu][iv] = p_u + p_v - p_uv;
        }
    }

    fit_surface_with_knots(&samples, &gu, &gv, out_du, out_dv, ku, kv)
}

/// Closest pair of boundary edges between two NURBS surfaces.
pub fn closest_edge_pair(a: &NurbsSurface, b: &NurbsSurface) -> (SurfaceEdge, SurfaceEdge) {
    let mut best = (SurfaceEdge::UMax, SurfaceEdge::UMin);
    let mut best_d = f64::INFINITY;
    for ea in SurfaceEdge::ALL {
        let Ok(ca) = a.boundary_curve(ea) else { continue };
        let (ua0, ua1) = ca.domain();
        let pa0 = ca.evaluate(ua0);
        let pam = ca.evaluate(0.5 * (ua0 + ua1));
        let pa1 = ca.evaluate(ua1);
        for eb in SurfaceEdge::ALL {
            let Ok(cb) = b.boundary_curve(eb) else { continue };
            let (ub0, ub1) = cb.domain();
            let pb0 = cb.evaluate(ub0);
            let pbm = cb.evaluate(0.5 * (ub0 + ub1));
            let pb1 = cb.evaluate(ub1);
            let direct = pa0.distance(pb0) + pam.distance(pbm) + pa1.distance(pb1);
            let reverse = pa0.distance(pb1) + pam.distance(pbm) + pa1.distance(pb0);
            let d = direct.min(reverse);
            if d < best_d {
                best_d = d;
                best = (ea, eb);
            }
        }
    }
    best
}

/// Class-A surface-to-surface blend (`G0`, `G1`, `G2`, or `G3`) across chosen boundary edges.
pub fn blend_srf(
    srf_a: &NurbsSurface,
    edge_a: SurfaceEdge,
    srf_b: &NurbsSurface,
    edge_b: SurfaceEdge,
    continuity: Continuity,
    bulge_a: f64,
    bulge_b: f64,
) -> Result<NurbsSurface, GeomError> {
    let ca = srf_a.boundary_curve(edge_a)?;
    let cb = srf_b.boundary_curve(edge_b)?;
    let direct = ca.evaluate(ca.domain().0).distance(cb.evaluate(cb.domain().0))
        + ca.evaluate(ca.domain().1).distance(cb.evaluate(cb.domain().1));
    let reversed = ca.evaluate(ca.domain().0).distance(cb.evaluate(cb.domain().1))
        + ca.evaluate(ca.domain().1).distance(cb.evaluate(cb.domain().0));
    let flip_b = reversed + 1e-9 < direct;

    let nu = srf_a.count_u.max(srf_b.count_u).max(srf_a.count_v).max(srf_b.count_v).clamp(9, 25);
    let p_v = match continuity {
        Continuity::G0 => 1,
        Continuity::G1 => 3,
        Continuity::G2 => 5,
        Continuity::G3 => 7,
    };
    let count_v = p_v + 1;
    let deg_u = 3_usize.min(nu - 1);
    let knots_u = uniform_clamped_knots(nu, deg_u);
    let u_vals: Vec<f64> = greville_abscissae(&knots_u, deg_u, nu);

    let mut cols_by_v = vec![vec![DVec3::ZERO; nu]; count_v];
    let ba = bulge_a.clamp(0.05, 5.0);
    let bb = bulge_b.clamp(0.05, 5.0);

    for (iu, &t) in u_vals.iter().enumerate() {
        let (pa, da1, da2) = eval_edge_outward(srf_a, edge_a, t);
        let tb = if flip_b { 1.0 - t } else { t };
        let (pb, db1, db2) = eval_edge_outward(srf_b, edge_b, tb);

        let strip_cps = blend_bezier_column(pa, da1, da2, pb, db1, db2, continuity, ba, bb);
        for (iv, pt) in strip_cps.into_iter().enumerate() {
            cols_by_v[iv][iu] = pt;
        }
    }

    let mut mat = vec![vec![0.0_f64; nu]; nu];
    for (row, &u) in u_vals.iter().enumerate() {
        let span = find_span(nu - 1, deg_u, u, &knots_u);
        let basis = basis_functions(span, u, deg_u, &knots_u);
        for (k, &b) in basis.iter().enumerate() {
            mat[row][span - deg_u + k] = b;
        }
    }

    let mut cps = vec![ControlPoint::unweighted(DVec3::ZERO); nu * count_v];
    for iv in 0..count_v {
        let row_cps = solve_linear_system_vec3(&mat, &cols_by_v[iv])
            .ok_or(GeomError::Degenerate("singular blend surface U collocation"))?;
        for iu in 0..nu {
            cps[iu * count_v + iv] = ControlPoint::unweighted(row_cps[iu]);
        }
    }

    let knots_v = uniform_clamped_knots(count_v, p_v);
    let mut blend = NurbsSurface::new(deg_u, p_v, knots_u, knots_v, nu, count_v, cps)?;
    let (ua0, ua1) = srf_a.domain_u();
    let (va0, va1) = srf_a.domain_v();
    let (su, sv) = match edge_a {
        SurfaceEdge::UMin => (ua0, 0.5 * (va0 + va1)),
        SurfaceEdge::UMax => (ua1, 0.5 * (va0 + va1)),
        SurfaceEdge::VMin => (0.5 * (ua0 + ua1), va0),
        SurfaceEdge::VMax => (0.5 * (ua0 + ua1), va1),
    };
    let (_, seam_n) = srf_a.evaluate_point_and_normal(su, sv);
    let (_, blend_n) = blend.evaluate_point_and_normal(0.5, 0.0);
    if seam_n.dot(blend_n) < 0.0 {
        blend = blend.flip_normal();
    }
    Ok(blend)
}

/// Match `target`'s `target_edge` to `reference`'s `ref_edge` with `G0`, `G1`, or `G2` continuity.
pub fn match_srf(
    target: &NurbsSurface,
    target_edge: SurfaceEdge,
    reference: &NurbsSurface,
    ref_edge: SurfaceEdge,
    continuity: Continuity,
) -> Result<NurbsSurface, GeomError> {
    // Work in canonical orientation where matched edge is UMin of `work`
    let (mut work, undo_fn): (NurbsSurface, fn(NurbsSurface) -> NurbsSurface) = match target_edge {
        SurfaceEdge::UMin => (target.clone(), |s| s),
        SurfaceEdge::UMax => (target.reverse_u(), |s| s.reverse_u()),
        SurfaceEdge::VMin => (target.transpose(), |s| s.transpose()),
        SurfaceEdge::VMax => (target.reverse_v().transpose(), |s| s.transpose().reverse_v()),
    };

    let min_perp = match continuity {
        Continuity::G0 => 2,
        Continuity::G1 => 4,
        Continuity::G2 | Continuity::G3 => 6,
    };
    let min_deg = match continuity {
        Continuity::G0 => 1,
        Continuity::G1 => 2,
        Continuity::G2 | Continuity::G3 => 3,
    };
    if work.degree_u < min_deg {
        work = work.elevate_degree(min_deg - work.degree_u, 0)?;
    }
    while work.count_u < min_perp {
        let (u0, u1) = work.domain_u();
        let next = work.insert_knot_u(0.5 * (u0 + u1), 1)?;
        if next.count_u == work.count_u {
            work = work.insert_knot_u(u0 + (u1 - u0) * 0.25, 1)?;
        } else {
            work = next;
        }
    }

    let c_target_edge = work.boundary_curve(SurfaceEdge::UMin)?;
    let c_ref_edge = reference.boundary_curve(ref_edge)?;
    let direct = c_target_edge
        .evaluate(c_target_edge.domain().0)
        .distance(c_ref_edge.evaluate(c_ref_edge.domain().0))
        + c_target_edge
            .evaluate(c_target_edge.domain().1)
            .distance(c_ref_edge.evaluate(c_ref_edge.domain().1));
    let reversed = c_target_edge
        .evaluate(c_target_edge.domain().0)
        .distance(c_ref_edge.evaluate(c_ref_edge.domain().1))
        + c_target_edge
            .evaluate(c_target_edge.domain().1)
            .distance(c_ref_edge.evaluate(c_ref_edge.domain().0));
    let flip_ref = reversed + 1e-9 < direct;

    let nv = work.count_v;
    let gv = work.greville_v();
    let (v0, v1) = work.domain_v();
    let v_span = (v1 - v0).max(1e-9);
    let target_center = work.evaluate(0.5 * (work.domain_u().0 + work.domain_u().1), 0.5 * (v0 + v1));
    let ref_center = reference.evaluate(
        0.5 * (reference.domain_u().0 + reference.domain_u().1),
        0.5 * (reference.domain_v().0 + reference.domain_v().1),
    );
    let target_offset_dir = target_center - ref_center;

    let mut row0_pts = Vec::with_capacity(nv);
    let mut row1_pts = Vec::with_capacity(nv);
    let mut row2_pts = Vec::with_capacity(nv);

    let p_u = work.degree_u;
    let ku = &work.knots_u;
    let u0 = ku[p_u];
    let d1_span = (ku[p_u + 1] - u0).max(1e-6);
    let d2_span = if work.count_u >= 3 {
        (ku[p_u + 2] - u0).max(d1_span)
    } else {
        d1_span
    };

    // Collocation matrix along V using Greville abscissae
    let mut mat_v = vec![vec![0.0_f64; nv]; nv];
    for (r, &v) in gv.iter().enumerate() {
        let span = find_span(nv - 1, work.degree_v, v, &work.knots_v);
        let basis = basis_functions(span, v, work.degree_v, &work.knots_v);
        for (k, &b) in basis.iter().enumerate() {
            mat_v[r][span - work.degree_v + k] = b;
        }
    }

    for (iv, &v) in gv.iter().enumerate() {
        let t = ((v - v0) / v_span).clamp(0.0, 1.0);
        let tr = if flip_ref { 1.0 - t } else { t };
        let (p_ref, d1_ref, d2_ref) = eval_edge_outward(reference, ref_edge, tr);
        row0_pts.push(p_ref);

        let cur_r0 = work.evaluate(u0, v);
        let cur_r1_cp = work.get_cp(1, iv).position;
        let step1 = cur_r1_cp.distance(cur_r0).max(d1_ref.length() * 0.15).max(1e-3);

        let tan_ref_dir = d1_ref.normalize_or_zero();
        let tan_dir = if tan_ref_dir.length_squared() > 1e-12 {
            if tan_ref_dir.dot(target_offset_dir) >= -0.25 {
                tan_ref_dir
            } else {
                -tan_ref_dir
            }
        } else {
            (work.evaluate(u0 + d1_span, v) - cur_r0).normalize_or_zero()
        };

        let p1 = p_ref + tan_dir * step1;
        row1_pts.push(p1);

        if matches!(continuity, Continuity::G2 | Continuity::G3) && p_u >= 2 && work.count_u >= 3 {
            let speed = (p_u as f64 / d1_span) * step1;
            let ref_speed = d1_ref.length().max(1e-6);
            let k_ref_vec = {
                let t_hat = d1_ref / ref_speed;
                (d2_ref - t_hat * d2_ref.dot(t_hat)) / (ref_speed * ref_speed)
            };
            let delta1 = (p1 - p_ref) / d1_span;
            let factor = (p_u * (p_u - 1)) as f64 / d1_span;
            let s2_perp = k_ref_vec * (speed * speed);
            let p2_perp = s2_perp * (d2_span / factor);
            let p2_tan = tan_dir * (delta1.dot(tan_dir) * d2_span);
            row2_pts.push(p1 + p2_tan + p2_perp);
        }
    }

    let c0 = solve_linear_system_vec3(&mat_v, &row0_pts)
        .ok_or(GeomError::Degenerate("match_srf G0 solve failed"))?;
    for iv in 0..nv {
        work.set_cp(0, iv, ControlPoint::unweighted(c0[iv]));
    }

    if matches!(continuity, Continuity::G1 | Continuity::G2 | Continuity::G3) {
        let c1 = solve_linear_system_vec3(&mat_v, &row1_pts)
            .ok_or(GeomError::Degenerate("match_srf G1 solve failed"))?;
        for iv in 0..nv {
            work.set_cp(1, iv, ControlPoint::unweighted(c1[iv]));
        }
        if matches!(continuity, Continuity::G2 | Continuity::G3) && work.count_u >= 3 {
            let c2 = solve_linear_system_vec3(&mat_v, &row2_pts)
                .ok_or(GeomError::Degenerate("match_srf G2 solve failed"))?;
            for iv in 0..nv {
                work.set_cp(2, iv, ControlPoint::unweighted(c2[iv]));
            }
        }
    }

    Ok(undo_fn(work))
}

/// Smooth NURBS surface offset along unit surface normals.
pub fn offset_srf(srf: &NurbsSurface, distance: f64) -> Result<NurbsSurface, GeomError> {
    if !distance.is_finite() {
        return Err(GeomError::Degenerate("offset distance must be finite"));
    }
    if srf.degree_u == 1 && srf.degree_v == 1 && srf.count_u == 2 && srf.count_v == 2 {
        let (_, n) = srf.evaluate_point_and_normal(0.5, 0.5);
        let mut out = srf.clone();
        for cp in &mut out.control_points {
            cp.position += n * distance;
        }
        return Ok(out);
    }

    let deg_u = srf.degree_u.clamp(1, 3);
    let deg_v = srf.degree_v.clamp(1, 3);
    let count_u = srf.count_u.clamp(deg_u + 1, 20);
    let count_v = srf.count_v.clamp(deg_v + 1, 20);
    let mu = (count_u * 2).clamp(12, 32);
    let mv = (count_v * 2).clamp(12, 32);

    let (u0, u1) = srf.domain_u();
    let (v0, v1) = srf.domain_v();
    let u_params: Vec<f64> = (0..=mu).map(|i| i as f64 / mu as f64).collect();
    let v_params: Vec<f64> = (0..=mv).map(|j| j as f64 / mv as f64).collect();
    let mut pts = vec![vec![DVec3::ZERO; mv + 1]; mu + 1];

    for (i, &u) in u_params.iter().enumerate() {
        let su = u0 + (u1 - u0) * u;
        for (j, &v) in v_params.iter().enumerate() {
            let sv = v0 + (v1 - v0) * v;
            let (p, n) = srf.evaluate_point_and_normal(su, sv);
            pts[i][j] = p + n * distance;
        }
    }

    fit_surface_grid(&pts, &u_params, &v_params, deg_u, deg_v, count_u, count_v)
}

/// Smoothly extend a NURBS surface beyond `edge` by `length`.
pub fn extend_srf(
    srf: &NurbsSurface,
    edge: SurfaceEdge,
    length: f64,
) -> Result<NurbsSurface, GeomError> {
    if !length.is_finite() || length <= 1e-6 {
        return Err(GeomError::Degenerate("extend length must be positive"));
    }
    let (work, undo_fn): (NurbsSurface, fn(NurbsSurface) -> NurbsSurface) = match edge {
        SurfaceEdge::UMax => (srf.clone(), |s| s),
        SurfaceEdge::UMin => (srf.reverse_u(), |s| s.reverse_u()),
        SurfaceEdge::VMax => (srf.transpose(), |s| s.transpose()),
        SurfaceEdge::VMin => (srf.transpose().reverse_u(), |s| s.reverse_u().transpose()),
    };

    let (u0, u1) = work.domain_u();
    let (v0, v1) = work.domain_v();
    let orig_nu = (work.count_u * 2).clamp(12, 24);
    let ext_nu = 6_usize;
    let nv = (work.count_v * 2).clamp(12, 24);

    // Build sampled surface + smooth outward tangent extension strip along UMax
    let mut sample_u_coords = Vec::with_capacity(orig_nu + ext_nu + 1);
    let mut avg_len_u = 0.0_f64;
    for j in 0..=4 {
        let v = v0 + (v1 - v0) * (j as f64 / 4.0);
        avg_len_u += work.extract_isocurve(false, v)?.arc_length();
    }
    avg_len_u = (avg_len_u / 5.0).max(length * 0.25);
    let base_frac = avg_len_u / (avg_len_u + length);

    for i in 0..=orig_nu {
        sample_u_coords.push((i as f64 / orig_nu as f64) * base_frac);
    }
    for k in 1..=ext_nu {
        sample_u_coords.push(base_frac + (1.0 - base_frac) * (k as f64 / ext_nu as f64));
    }
    let v_params: Vec<f64> = (0..=nv).map(|j| j as f64 / nv as f64).collect();

    let mut pts = vec![vec![DVec3::ZERO; nv + 1]; sample_u_coords.len()];
    for (j, &vf) in v_params.iter().enumerate() {
        let v = v0 + (v1 - v0) * vf;
        for i in 0..=orig_nu {
            let u = u0 + (u1 - u0) * (i as f64 / orig_nu as f64);
            pts[i][j] = work.evaluate(u, v);
        }
        let d_end = work.evaluate_derivatives(u1, v);
        let dir = d_end.du.normalize_or_zero();
        for k in 1..=ext_nu {
            let s = length * (k as f64 / ext_nu as f64);
            pts[orig_nu + k][j] = d_end.point + dir * s;
        }
    }

    let deg_u = work.degree_u.clamp(1, 3);
    let deg_v = work.degree_v.clamp(1, 3);
    let target_cu = (work.count_u + 2).clamp(deg_u + 1, 22);
    let target_cv = work.count_v.clamp(deg_v + 1, 20);
    let extended = fit_surface_grid(
        &pts,
        &sample_u_coords,
        &v_params,
        deg_u,
        deg_v,
        target_cu,
        target_cv,
    )?;
    Ok(undo_fn(extended))
}

/// Rhino 9-inspired `Patch`: constrained regularized least-squares NURBS surface fitting with thin-plate fairing & optional G1/G2 reference edges.
pub fn patch_srf(
    boundaries: &[PatchBoundaryConstraint],
    interior_points: &[DVec3],
    options: &PatchOptions,
) -> Result<PatchReport, GeomError> {
    let mut all_samples: Vec<(DVec3, Option<(DVec3, DVec3)>)> = Vec::new();
    for bc in boundaries {
        let samples = (bc.curve.control_points.len() * 3).clamp(16, 40);
        let (u0, u1) = bc.curve.domain();
        for i in 0..=samples {
            let u = u0 + (u1 - u0) * (i as f64 / samples as f64);
            let pt = bc.curve.evaluate(u);
            let ref_info = if let Some((ref_srf, _)) = &bc.reference_surface {
                let (ru, rv, _, _) = ref_srf.closest_point(pt);
                let frame = ref_srf.surface_frame(ru, rv);
                Some((frame.normal, frame.origin))
            } else {
                None
            };
            all_samples.push((pt, ref_info));
        }
    }
    for &p in interior_points {
        all_samples.push((p, None));
    }

    if all_samples.len() < 3 {
        return Err(GeomError::Degenerate("Patch needs at least 3 constraint points"));
    }

    let raw_pts: Vec<DVec3> = all_samples.iter().map(|s| s.0).collect();
    let plane = best_fit_plane(&raw_pts);

    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut local_xy = Vec::with_capacity(all_samples.len());
    for (p, _) in &all_samples {
        let d = *p - plane.origin;
        let x = d.dot(plane.x_axis);
        let y = d.dot(plane.y_axis);
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
        local_xy.push((x, y));
    }
    let dx = (max_x - min_x).max(1e-3);
    let dy = (max_y - min_y).max(1e-3);
    let pad_x = dx * 0.02;
    let pad_y = dy * 0.02;
    min_x -= pad_x;
    max_x += pad_x;
    min_y -= pad_y;
    max_y += pad_y;
    let span_x = max_x - min_x;
    let span_y = max_y - min_y;

    let deg = options.degree.clamp(1, 3);
    let spans_u = options.spans_u.clamp(2, 8);
    let spans_v = options.spans_v.clamp(2, 8);
    let nu = spans_u + deg;
    let nv = spans_v + deg;
    let total_vars = nu * nv;
    let ku = uniform_clamped_knots(nu, deg);
    let kv = uniform_clamped_knots(nv, deg);

    let mut design: Vec<Vec<f64>> = Vec::new();
    let mut rhs: Vec<DVec3> = Vec::new();

    let eval_row = |u: f64, v: f64, weight: f64| -> Vec<f64> {
        let mut row = vec![0.0_f64; total_vars];
        let su = find_span(nu - 1, deg, u, &ku);
        let sv = find_span(nv - 1, deg, v, &kv);
        let bu = basis_functions(su, u, deg, &ku);
        let bv = basis_functions(sv, v, deg, &kv);
        for (i, &ui) in bu.iter().enumerate() {
            let ci = su - deg + i;
            for (j, &vj) in bv.iter().enumerate() {
                let cj = sv - deg + j;
                row[ci * nv + cj] = ui * vj * weight;
            }
        }
        row
    };

    let centroid_2d = (0.5 * (min_x + max_x), 0.5 * (min_y + max_y));

    for (idx, &(pt, ref_info)) in all_samples.iter().enumerate() {
        let (lx, ly) = local_xy[idx];
        let u = ((lx - min_x) / span_x).clamp(0.0, 1.0);
        let v = ((ly - min_y) / span_y).clamp(0.0, 1.0);
        let w_pos = 12.0;
        design.push(eval_row(u, v, w_pos));
        rhs.push(pt * w_pos);

        // Enforce G1/G2 slope along inward chord direction when a reference surface is attached
        if matches!(options.continuity, Continuity::G1 | Continuity::G2 | Continuity::G3) {
            if let Some((ref_n, _)) = ref_info {
                let inward_2d = DVec3::new(centroid_2d.0 - lx, centroid_2d.1 - ly, 0.0);
                let in_len = inward_2d.length();
                if in_len > 1e-4 {
                    let dir_3d =
                        (plane.x_axis * inward_2d.x + plane.y_axis * inward_2d.y) / in_len;
                    let proj = dir_3d - ref_n * dir_3d.dot(ref_n);
                    let tan_in = proj.normalize_or_zero();
                    if tan_in.length_squared() > 1e-6 {
                        let step = (span_x.min(span_y) * 0.04).max(1e-3);
                        let p_off = pt + tan_in * step;
                        let du_off = ((p_off - plane.origin).dot(plane.x_axis) - min_x) / span_x;
                        let dv_off = ((p_off - plane.origin).dot(plane.y_axis) - min_y) / span_y;
                        let w_tan = 3.0;
                        design.push(eval_row(du_off.clamp(0.0, 1.0), dv_off.clamp(0.0, 1.0), w_tan));
                        rhs.push(p_off * w_tan);
                    }
                }
            }
        }
    }

    // Thin-plate spline bending regularization over parametric domain (helps smooth interior fairness)
    let w_bend = (options.stiffness.max(1e-4)).sqrt() * 0.8;
    let reg_u = nu + 2;
    let reg_v = nv + 2;
    let step_scale = w_bend / ((reg_u * reg_v) as f64).sqrt();
    if deg >= 2 {
        for iu in 0..=reg_u {
            let u = iu as f64 / reg_u as f64;
            let su = find_span(nu - 1, deg, u, &ku);
            let du = basis_function_derivatives(su, u, deg, 2, &ku);
            for iv in 0..=reg_v {
                let v = iv as f64 / reg_v as f64;
                let sv = find_span(nv - 1, deg, v, &kv);
                let dv = basis_function_derivatives(sv, v, deg, 2, &kv);

                let mut row_uu = vec![0.0_f64; total_vars];
                let mut row_uv = vec![0.0_f64; total_vars];
                let mut row_vv = vec![0.0_f64; total_vars];
                for i in 0..=deg {
                    let ci = su - deg + i;
                    for j in 0..=deg {
                        let cj = sv - deg + j;
                        let idx = ci * nv + cj;
                        row_uu[idx] = du[2][i] * dv[0][j] * step_scale;
                        row_uv[idx] = du[1][i] * dv[1][j] * step_scale * std::f64::consts::SQRT_2;
                        row_vv[idx] = du[0][i] * dv[2][j] * step_scale;
                    }
                }
                design.push(row_uu);
                rhs.push(DVec3::ZERO);
                design.push(row_uv);
                rhs.push(DVec3::ZERO);
                design.push(row_vv);
                rhs.push(DVec3::ZERO);
            }
        }
    }

    let cps_vec = solve_least_squares_vec3(&design, &rhs, 1e-8)
        .ok_or(GeomError::Degenerate("Patch least-squares solver failed"))?;
    let cps: Vec<ControlPoint> = cps_vec.into_iter().map(ControlPoint::unweighted).collect();
    let surface = NurbsSurface::new(deg, deg, ku, kv, nu, nv, cps)?;

    let mut max_dev = 0.0_f64;
    for &(pt, _) in &all_samples {
        let (_, _, _, d) = surface.closest_point(pt);
        max_dev = max_dev.max(d);
    }

    Ok(PatchReport {
        surface,
        max_deviation: max_dev,
    })
}

/// Uniform least-squares surface rebuild returning `SurfaceRebuildReport`.
pub fn rebuild_srf(
    srf: &NurbsSurface,
    count_u: usize,
    count_v: usize,
    deg_u: usize,
    deg_v: usize,
) -> Result<SurfaceRebuildReport, GeomError> {
    let old_u = srf.count_u;
    let old_v = srf.count_v;
    let new_u = count_u.max(2);
    let new_v = count_v.max(2);
    let du = deg_u.clamp(1, new_u - 1);
    let dv = deg_v.clamp(1, new_v - 1);

    let mu = (new_u * 3).clamp(16, 40);
    let mv = (new_v * 3).clamp(16, 40);
    let (u0, u1) = srf.domain_u();
    let (v0, v1) = srf.domain_v();
    let u_params: Vec<f64> = (0..=mu).map(|i| i as f64 / mu as f64).collect();
    let v_params: Vec<f64> = (0..=mv).map(|j| j as f64 / mv as f64).collect();

    let mut pts = vec![vec![DVec3::ZERO; mv + 1]; mu + 1];
    for (i, &u) in u_params.iter().enumerate() {
        let su = u0 + (u1 - u0) * u;
        for (j, &v) in v_params.iter().enumerate() {
            let sv = v0 + (v1 - v0) * v;
            pts[i][j] = srf.evaluate(su, sv);
        }
    }

    let rebuilt = fit_surface_grid(&pts, &u_params, &v_params, du, dv, new_u, new_v)?;
    let mut max_dev = 0.0_f64;
    for i in 0..=16 {
        let u = i as f64 / 16.0;
        let su = u0 + (u1 - u0) * u;
        for j in 0..=16 {
            let v = j as f64 / 16.0;
            let sv = v0 + (v1 - v0) * v;
            let orig_p = srf.evaluate(su, sv);
            let (_, _, _, d) = rebuilt.closest_point(orig_p);
            max_dev = max_dev.max(d);
        }
    }

    Ok(SurfaceRebuildReport {
        surface: rebuilt,
        old_u,
        old_v,
        new_u,
        new_v,
        max_deviation: max_dev,
    })
}

/// Rhino 9 NURBS Soft Transform falloff weights over a surface control-point grid with boundary locking.
pub fn soft_transform_weights_surface(
    srf: &NurbsSurface,
    selected_cps: &[usize],
    radius: f64,
    falloff: SoftTransformFalloff,
    boundary_lock: BoundaryLock,
) -> Vec<f64> {
    let total = srf.control_points.len();
    let mut weights = vec![0.0_f64; total];
    if selected_cps.is_empty() {
        return weights;
    }

    let lock_rings = match boundary_lock {
        BoundaryLock::Free => 0,
        BoundaryLock::FixG0 => 1,
        BoundaryLock::FixG1 => 2,
        BoundaryLock::FixG2 => 3,
    };

    let sel_positions: Vec<DVec3> = selected_cps
        .iter()
        .filter_map(|&idx| srf.control_points.get(idx).map(|cp| cp.position))
        .collect();
    if sel_positions.is_empty() {
        return weights;
    }

    let r = radius.max(1e-6);
    for ui in 0..srf.count_u {
        for vj in 0..srf.count_v {
            let idx = ui * srf.count_v + vj;
            let locked = lock_rings > 0
                && (ui < lock_rings
                    || ui + lock_rings >= srf.count_u
                    || vj < lock_rings
                    || vj + lock_rings >= srf.count_v);
            if locked {
                weights[idx] = 0.0;
                continue;
            }
            if selected_cps.contains(&idx) {
                weights[idx] = 1.0;
                continue;
            }
            let pos = srf.control_points[idx].position;
            let min_d = sel_positions
                .iter()
                .map(|sp| sp.distance(pos))
                .fold(f64::INFINITY, f64::min);
            if min_d >= r {
                weights[idx] = 0.0;
            } else {
                let t = (1.0 - min_d / r).clamp(0.0, 1.0);
                weights[idx] = match falloff {
                    SoftTransformFalloff::Smooth => t * t * (3.0 - 2.0 * t),
                    SoftTransformFalloff::Linear => t,
                    SoftTransformFalloff::Sharp => t * t,
                };
            }
        }
    }
    weights
}

/// Rhino 9 NURBS Soft Transform falloff weights over a curve control polygon with optional endpoint locking.
pub fn soft_transform_weights_curve(
    curve: &NurbsCurve,
    selected_cps: &[usize],
    radius: f64,
    falloff: SoftTransformFalloff,
    boundary_lock: BoundaryLock,
) -> Vec<f64> {
    let n = curve.control_points.len();
    let mut weights = vec![0.0_f64; n];
    if selected_cps.is_empty() {
        return weights;
    }
    let lock_rings = match boundary_lock {
        BoundaryLock::Free => 0,
        BoundaryLock::FixG0 => 1,
        BoundaryLock::FixG1 => 2,
        BoundaryLock::FixG2 => 3,
    };
    let sel_positions: Vec<DVec3> = selected_cps
        .iter()
        .filter_map(|&idx| curve.control_points.get(idx).map(|cp| cp.position))
        .collect();
    let r = radius.max(1e-6);
    for (i, cp) in curve.control_points.iter().enumerate() {
        if lock_rings > 0 && (i < lock_rings || i + lock_rings >= n) {
            weights[i] = 0.0;
            continue;
        }
        if selected_cps.contains(&i) {
            weights[i] = 1.0;
            continue;
        }
        let min_d = sel_positions
            .iter()
            .map(|sp| sp.distance(cp.position))
            .fold(f64::INFINITY, f64::min);
        if min_d < r {
            let t = (1.0 - min_d / r).clamp(0.0, 1.0);
            weights[i] = match falloff {
                SoftTransformFalloff::Smooth => t * t * (3.0 - 2.0 * t),
                SoftTransformFalloff::Linear => t,
                SoftTransformFalloff::Sharp => t * t,
            };
        }
    }
    weights
}

pub fn apply_weighted_deltas_surface(
    srf: &mut NurbsSurface,
    base: &NurbsSurface,
    weights: &[f64],
    delta: DVec3,
) {
    for (cp, (base_cp, &w)) in srf
        .control_points
        .iter_mut()
        .zip(base.control_points.iter().zip(weights.iter()))
    {
        cp.position = base_cp.position + delta * w;
    }
}

pub fn apply_weighted_deltas_curve(
    curve: &mut NurbsCurve,
    base: &NurbsCurve,
    weights: &[f64],
    delta: DVec3,
) {
    for (cp, (base_cp, &w)) in curve
        .control_points
        .iter_mut()
        .zip(base.control_points.iter().zip(weights.iter()))
    {
        cp.position = base_cp.position + delta * w;
    }
}

pub fn apply_soft_transform_surface(
    srf: &mut NurbsSurface,
    selected_cps: &[usize],
    delta: DVec3,
    radius: f64,
    falloff: SoftTransformFalloff,
    boundary_lock: BoundaryLock,
) {
    let weights = soft_transform_weights_surface(srf, selected_cps, radius, falloff, boundary_lock);
    let base = srf.clone();
    apply_weighted_deltas_surface(srf, &base, &weights, delta);
}

// --- Internal surfacing helpers ---

fn fit_surface_grid(
    pts: &[Vec<DVec3>],
    u_params: &[f64],
    v_params: &[f64],
    deg_u: usize,
    deg_v: usize,
    count_u: usize,
    count_v: usize,
) -> Result<NurbsSurface, GeomError> {
    let ku = uniform_clamped_knots(count_u, deg_u);
    let kv = uniform_clamped_knots(count_v, deg_v);
    fit_surface_with_knots(pts, u_params, v_params, deg_u, deg_v, ku, kv)
}

fn fit_surface_with_knots(
    pts: &[Vec<DVec3>],
    u_params: &[f64],
    v_params: &[f64],
    deg_u: usize,
    deg_v: usize,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
) -> Result<NurbsSurface, GeomError> {
    let mu = u_params.len();
    let mv = v_params.len();
    let count_u = knots_u.len() - deg_u - 1;
    let count_v = knots_v.len() - deg_v - 1;
    if pts.len() != mu || pts.iter().any(|r| r.len() != mv) {
        return Err(GeomError::Degenerate("surface fit grid dimension mismatch"));
    }

    // Step 1: Fit along U for each sampled V column -> intermediate Q[count_u][mv]
    let mut nu_mat = vec![vec![0.0_f64; count_u]; mu];
    for (row, &u) in u_params.iter().enumerate() {
        let span = find_span(count_u - 1, deg_u, u, &knots_u);
        let basis = basis_functions(span, u, deg_u, &knots_u);
        for (k, &b) in basis.iter().enumerate() {
            nu_mat[row][span - deg_u + k] = b;
        }
    }

    let mut q_inter = vec![vec![DVec3::ZERO; mv]; count_u];
    for j in 0..mv {
        let col_rhs: Vec<DVec3> = (0..mu).map(|i| pts[i][j]).collect();
        let sol = if mu == count_u {
            solve_linear_system_vec3(&nu_mat, &col_rhs)
        } else {
            solve_least_squares_vec3(&nu_mat, &col_rhs, 1e-10)
        }
        .ok_or(GeomError::Degenerate("surface U fit failed"))?;
        for i in 0..count_u {
            q_inter[i][j] = sol[i];
        }
    }

    // Step 2: Fit along V for each U control-point row -> P[count_u][count_v]
    let mut nv_mat = vec![vec![0.0_f64; count_v]; mv];
    for (row, &v) in v_params.iter().enumerate() {
        let span = find_span(count_v - 1, deg_v, v, &knots_v);
        let basis = basis_functions(span, v, deg_v, &knots_v);
        for (k, &b) in basis.iter().enumerate() {
            nv_mat[row][span - deg_v + k] = b;
        }
    }

    let mut cps = Vec::with_capacity(count_u * count_v);
    for row in &q_inter {
        let sol = if mv == count_v {
            solve_linear_system_vec3(&nv_mat, row)
        } else {
            solve_least_squares_vec3(&nv_mat, row, 1e-10)
        }
        .ok_or(GeomError::Degenerate("surface V fit failed"))?;
        for pt in sol {
            cps.push(ControlPoint::unweighted(pt));
        }
    }

    NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, count_u, count_v, cps)
}

fn compute_loft_v_params(curves: &[NurbsCurve], centripetal: bool) -> Vec<f64> {
    let m = curves.len();
    let count_u = curves[0].control_points.len();
    let mut v_params = vec![0.0_f64; m];
    let mut total = 0.0;
    for vj in 0..(m - 1) {
        let mut avg = 0.0;
        for ui in 0..count_u {
            let d = curves[vj].control_points[ui]
                .position
                .distance(curves[vj + 1].control_points[ui].position);
            avg += if centripetal { d.sqrt() } else { d };
        }
        avg /= count_u as f64;
        total += avg.max(1e-5);
        v_params[vj + 1] = total;
    }
    for v in &mut v_params {
        *v /= total;
    }
    v_params[m - 1] = 1.0;
    v_params
}

fn averaged_clamped_knots(params: &[f64], degree: usize) -> Vec<f64> {
    let n = params.len() - 1;
    let mut knots = Vec::with_capacity(params.len() + degree + 1);
    knots.extend(std::iter::repeat_n(0.0, degree + 1));
    if n > degree {
        for j in 1..=(n - degree) {
            let mut sum = 0.0;
            for i in j..(j + degree) {
                sum += params[i];
            }
            knots.push(sum / degree as f64);
        }
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

fn build_rail_frames(
    rail: &NurbsCurve,
    params: &[f64],
    roadlike_up: Option<DVec3>,
    closed_rail: bool,
) -> Vec<(DVec3, DVec3, DVec3, DVec3)> {
    let mut frames = Vec::with_capacity(params.len());
    let (_, _, n0, _) = rail.frenet_frame(params[0]);
    let mut prev_p = rail.evaluate(params[0]);
    let mut prev_t = rail.evaluate_derivative(params[0]).1.normalize_or(DVec3::X);
    let mut prev_n = if let Some(up) = roadlike_up {
        let p = up - prev_t * up.dot(prev_t);
        p.normalize_or(n0)
    } else {
        n0
    };
    let prev_b = prev_t.cross(prev_n).normalize_or(DVec3::Z);
    prev_n = prev_b.cross(prev_t).normalize_or(prev_n);
    frames.push((prev_p, prev_t, prev_n, prev_b));

    for &u in &params[1..] {
        let p = rail.evaluate(u);
        let t = rail.evaluate_derivative(u).1.normalize_or(prev_t);
        if let Some(up) = roadlike_up {
            let mut n = (up - t * up.dot(t)).normalize_or_zero();
            if n.length_squared() < 1e-10 {
                n = (prev_n - t * prev_n.dot(t)).normalize_or(DVec3::X);
            }
            let b = t.cross(n).normalize();
            frames.push((p, t, n, b));
            prev_n = n;
            prev_t = t;
            continue;
        }
        // Double-reflection rotation-minimizing Bishop frame (Wang et al. 2008)
        let v1 = p - prev_p;
        let c1 = v1.length_squared();
        let (rl, tl) = if c1 > 1e-16 {
            let r = prev_n - v1 * (2.0 / c1 * v1.dot(prev_n));
            let s = prev_t - v1 * (2.0 / c1 * v1.dot(prev_t));
            (r, s)
        } else {
            (prev_n, prev_t)
        };
        let v2 = t - tl;
        let c2 = v2.length_squared();
        let mut n = if c2 > 1e-16 {
            rl - v2 * (2.0 / c2 * v2.dot(rl))
        } else {
            rl
        };
        n = (n - t * n.dot(t)).normalize_or(prev_n);
        let b = t.cross(n).normalize();
        frames.push((p, t, n, b));
        prev_p = p;
        prev_t = t;
        prev_n = n;
    }

    if closed_rail && roadlike_up.is_none() && frames.len() >= 3 {
        let first_n = frames[0].2;
        let last_n = frames.last().unwrap().2;
        let t0 = frames[0].1;
        let dot = first_n.dot(last_n).clamp(-1.0, 1.0);
        let total_twist = last_n.cross(first_n).dot(t0).atan2(dot);
        let m = frames.len() - 1;
        for (i, frame) in frames.iter_mut().enumerate() {
            let ang = total_twist * (i as f64 / m as f64);
            let rot = DMat4::from_axis_angle(frame.1, ang);
            frame.2 = rot.transform_vector3(frame.2).normalize();
            frame.3 = frame.1.cross(frame.2).normalize();
        }
    }

    frames
}

fn interpolate_rail_frame(
    rail: &NurbsCurve,
    params: &[f64],
    frames: &[(DVec3, DVec3, DVec3, DVec3)],
    u: f64,
    roadlike_up: Option<DVec3>,
) -> (DVec3, DVec3, DVec3, DVec3) {
    let idx = params.partition_point(|&v| v < u).min(frames.len() - 1);
    let p = rail.evaluate(u);
    let t = rail.evaluate_derivative(u).1.normalize_or(frames[idx].1);
    let raw_n = if let Some(up) = roadlike_up {
        (up - t * up.dot(t)).normalize_or(frames[idx].2)
    } else {
        (frames[idx].2 - t * frames[idx].2.dot(t)).normalize_or(frames[idx].2)
    };
    let b = t.cross(raw_n).normalize_or(frames[idx].3);
    let n = b.cross(t).normalize();
    (p, t, n, b)
}

fn build_two_rail_frames(
    r1: &NurbsCurve,
    r2: &NurbsCurve,
    params: &[f64],
) -> Vec<(DVec3, DVec3, DVec3, DVec3, f64)> {
    let mut frames = Vec::with_capacity(params.len());
    let mut prev_e3 = DVec3::Z;
    for (idx, &u) in params.iter().enumerate() {
        let p1 = r1.evaluate(u);
        let p2 = r2.evaluate(u);
        let span = p2 - p1;
        let len = span.length().max(1e-6);
        let e1 = span / len;
        let ref_v = if idx == 0 {
            if e1.z.abs() < 0.9 { DVec3::Z } else { DVec3::Y }
        } else {
            prev_e3
        };
        let mut e3 = (ref_v - e1 * ref_v.dot(e1)).normalize_or_zero();
        if e3.length_squared() < 1e-10 {
            let alt = if e1.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
            e3 = (alt - e1 * alt.dot(e1)).normalize();
        }
        let e2 = e3.cross(e1).normalize();
        e3 = e1.cross(e2).normalize();
        prev_e3 = e3;
        frames.push((p1, e1, e2, e3, len));
    }
    frames
}

fn interpolate_two_rail_frame(
    r1: &NurbsCurve,
    r2: &NurbsCurve,
    params: &[f64],
    frames: &[(DVec3, DVec3, DVec3, DVec3, f64)],
    u: f64,
) -> (DVec3, DVec3, DVec3, DVec3, f64) {
    let idx = params.partition_point(|&v| v < u).min(frames.len() - 1);
    let p1 = r1.evaluate(u);
    let p2 = r2.evaluate(u);
    let span = p2 - p1;
    let len = span.length().max(1e-6);
    let e1 = span / len;
    let mut e3 = (frames[idx].3 - e1 * frames[idx].3.dot(e1)).normalize_or(frames[idx].3);
    let e2 = e3.cross(e1).normalize();
    e3 = e1.cross(e2).normalize();
    (p1, e1, e2, e3, len)
}

fn blend_attached_profiles(
    attached: &[(f64, Vec<(DVec3, f64)>)],
    u: f64,
) -> Vec<(DVec3, f64)> {
    if attached.len() == 1 || u <= attached[0].0 {
        return attached[0].1.clone();
    }
    let last = attached.len() - 1;
    if u >= attached[last].0 {
        return attached[last].1.clone();
    }
    for w in attached.windows(2) {
        if u >= w[0].0 && u <= w[1].0 {
            let denom = (w[1].0 - w[0].0).max(1e-9);
            let t = ((u - w[0].0) / denom).clamp(0.0, 1.0);
            let s = t * t * (3.0 - 2.0 * t);
            return w[0]
                .1
                .iter()
                .zip(w[1].1.iter())
                .map(|(&(p0, w0), &(p1, w1))| (p0.lerp(p1, s), w0 + (w1 - w0) * s))
                .collect();
        }
    }
    attached[0].1.clone()
}

fn order_boundary_loop(curves: &[NurbsCurve]) -> Result<Vec<NurbsCurve>, GeomError> {
    let mut pool: Vec<NurbsCurve> = curves
        .iter()
        .map(|c| c.reparameterize(0.0, 1.0))
        .collect();
    let mut chain = vec![pool.remove(0)];
    while !pool.is_empty() {
        let cur_end = chain.last().unwrap().evaluate(1.0);
        let mut best_i = 0;
        let mut best_flip = false;
        let mut best_d = f64::INFINITY;
        for (i, cand) in pool.iter().enumerate() {
            let d_start = cur_end.distance(cand.evaluate(0.0));
            let d_end = cur_end.distance(cand.evaluate(1.0));
            if d_start < best_d {
                best_d = d_start;
                best_i = i;
                best_flip = false;
            }
            if d_end < best_d {
                best_d = d_end;
                best_i = i;
                best_flip = true;
            }
        }
        let next = pool.remove(best_i);
        chain.push(if best_flip {
            next.reverse().reparameterize(0.0, 1.0)
        } else {
            next
        });
    }
    Ok(chain)
}

fn make_curves_compatible_no_flip(curves: &[NurbsCurve]) -> Result<Vec<NurbsCurve>, GeomError> {
    let mut out: Vec<NurbsCurve> = curves.iter().map(|c| c.reparameterize(0.0, 1.0)).collect();
    let max_deg = out.iter().map(|c| c.degree).max().unwrap_or(1);
    for c in &mut out {
        if c.degree < max_deg {
            *c = c.elevate_degree(max_deg - c.degree)?;
        }
    }
    let mut merged: Vec<(f64, usize)> = Vec::new();
    for c in &out {
        let mut counts: Vec<(f64, usize)> = Vec::new();
        for &k in c.knots.iter().skip(max_deg + 1).take(c.control_points.len() - max_deg - 1) {
            if let Some(last) = counts.last_mut() {
                if (last.0 - k).abs() <= 1e-6 {
                    last.1 += 1;
                    continue;
                }
            }
            counts.push((k, 1));
        }
        for (v, m) in counts {
            if let Some(f) = merged.iter_mut().find(|(val, _)| (*val - v).abs() <= 1e-6) {
                f.1 = f.1.max(m);
            } else {
                merged.push((v, m));
            }
        }
    }
    merged.sort_by(|a, b| a.0.total_cmp(&b.0));
    let extra: usize = merged.iter().map(|(_, m)| *m).sum();
    if max_deg + 1 + extra > 64 {
        let target_cp = out.iter().map(|c| c.control_points.len()).max().unwrap_or(4).clamp(4, 24);
        let deg = max_deg.clamp(1, 3);
        let mut rebuilt = Vec::with_capacity(out.len());
        for c in &out {
            rebuilt.push(rebuild_uniform(c, target_cp, deg)?.curve);
        }
        return Ok(rebuilt);
    }
    for c in &mut out {
        for &(kv, target_mult) in &merged {
            let cur_mult = c.knots.iter().filter(|&&x| (x - kv).abs() <= 1e-6).count();
            if target_mult > cur_mult {
                *c = c.insert_knot(kv, target_mult - cur_mult)?;
            }
        }
    }
    Ok(out)
}

fn eval_edge_outward(srf: &NurbsSurface, edge: SurfaceEdge, t_01: f64) -> (DVec3, DVec3, DVec3) {
    let (u0, u1) = srf.domain_u();
    let (v0, v1) = srf.domain_v();
    let lu = (u1 - u0).max(PARAM_EPSILON);
    let lv = (v1 - v0).max(PARAM_EPSILON);
    let t = t_01.clamp(0.0, 1.0);
    match edge {
        SurfaceEdge::UMin => {
            let d = srf.evaluate_derivatives(u0, v0 + lv * t);
            (d.point, -d.du * lu, d.duu * (lu * lu))
        }
        SurfaceEdge::UMax => {
            let d = srf.evaluate_derivatives(u1, v0 + lv * t);
            (d.point, d.du * lu, d.duu * (lu * lu))
        }
        SurfaceEdge::VMin => {
            let d = srf.evaluate_derivatives(u0 + lu * t, v0);
            (d.point, -d.dv * lv, d.dvv * (lv * lv))
        }
        SurfaceEdge::VMax => {
            let d = srf.evaluate_derivatives(u0 + lu * t, v1);
            (d.point, d.dv * lv, d.dvv * (lv * lv))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn blend_bezier_column(
    pa: DVec3,
    da1: DVec3,
    da2: DVec3,
    pb: DVec3,
    db1: DVec3,
    db2: DVec3,
    continuity: Continuity,
    bulge_a: f64,
    bulge_b: f64,
) -> Vec<DVec3> {
    if continuity == Continuity::G0 {
        return vec![pa, pb];
    }
    let chord = pa.distance(pb).max(1e-3);
    let dir_a = da1.normalize_or((pb - pa).normalize_or(DVec3::X));
    let dir_b = (-db1).normalize_or((pb - pa).normalize_or(DVec3::X));

    let la = 0.35 * chord * bulge_a;
    let lb = 0.35 * chord * bulge_b;

    if continuity == Continuity::G1 {
        return vec![pa, pa + dir_a * la, pb - dir_b * lb, pb];
    }

    let speed_a = da1.length().max(1e-6);
    let speed_b = db1.length().max(1e-6);
    let k_a = (da2 - dir_a * da2.dot(dir_a)) / (speed_a * speed_a);
    let k_b = (db2 - (-dir_b) * db2.dot(-dir_b)) / (speed_b * speed_b);

    let deg = if continuity == Continuity::G3 { 7.0 } else { 5.0 };
    let h1 = (la * 3.0 / deg).max(0.05 * chord);
    let h2 = (lb * 3.0 / deg).max(0.05 * chord);
    let p1 = pa + dir_a * h1;
    let p_last_m1 = pb - dir_b * h2;
    let denom = deg * (deg - 1.0);
    let c2_a = k_a * (deg * h1).powi(2);
    let c2_b = k_b * (deg * h2).powi(2);
    let p2 = p1 * 2.0 - pa + c2_a / denom;
    let p_last_m2 = p_last_m1 * 2.0 - pb + c2_b / denom;

    if continuity == Continuity::G2 {
        vec![pa, p1, p2, p_last_m2, p_last_m1, pb]
    } else {
        let mid_a = p2.lerp(p_last_m2, 0.33);
        let mid_b = p2.lerp(p_last_m2, 0.67);
        vec![pa, p1, p2, mid_a, mid_b, p_last_m2, p_last_m1, pb]
    }
}

fn best_fit_plane(points: &[DVec3]) -> Plane {
    let origin = points.iter().copied().sum::<DVec3>() / points.len() as f64;
    let mut cxx = 0.0;
    let mut cxy = 0.0;
    let mut cxz = 0.0;
    let mut cyy = 0.0;
    let mut cyz = 0.0;
    let mut czz = 0.0;
    for &p in points {
        let d = p - origin;
        cxx += d.x * d.x;
        cxy += d.x * d.y;
        cxz += d.x * d.z;
        cyy += d.y * d.y;
        cyz += d.y * d.z;
        czz += d.z * d.z;
    }

    // Jacobi eigendecomposition of symmetric 3x3 covariance matrix
    let mut a = [[cxx, cxy, cxz], [cxy, cyy, cyz], [cxz, cyz, czz]];
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..24 {
        let mut p = 0;
        let mut q = 1;
        let mut max_off = a[0][1].abs();
        if a[0][2].abs() > max_off {
            max_off = a[0][2].abs();
            p = 0;
            q = 2;
        }
        if a[1][2].abs() > max_off {
            max_off = a[1][2].abs();
            p = 1;
            q = 2;
        }
        if max_off < 1e-14 {
            break;
        }
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        let phi = 0.5 * (2.0 * apq).atan2(aqq - app);
        let c = phi.cos();
        let s = phi.sin();
        for i in 0..3 {
            if i != p && i != q {
                let aip = a[i][p];
                let aiq = a[i][q];
                a[i][p] = c * aip - s * aiq;
                a[p][i] = a[i][p];
                a[i][q] = s * aip + c * aiq;
                a[q][i] = a[i][q];
            }
        }
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for i in 0..3 {
            let vip = v[i][p];
            let viq = v[i][q];
            v[i][p] = c * vip - s * viq;
            v[i][q] = s * vip + c * viq;
        }
    }

    let mut cols = [0, 1, 2];
    cols.sort_by(|&i, &j| a[j][j].total_cmp(&a[i][i]));
    let x_axis = DVec3::new(v[0][cols[0]], v[1][cols[0]], v[2][cols[0]]).normalize_or(DVec3::X);
    let y_raw = DVec3::new(v[0][cols[1]], v[1][cols[1]], v[2][cols[1]]).normalize_or(DVec3::Y);
    let z_axis = x_axis.cross(y_raw).normalize_or(DVec3::Z);
    let y_axis = z_axis.cross(x_axis).normalize_or(DVec3::Y);
    Plane {
        origin,
        x_axis,
        y_axis,
        z_axis,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loft_revolve_and_rail_revolve_create_accurate_nurbs_surfaces() {
        let p0 = Plane::world_xy();
        let mut p1 = Plane::world_xy();
        p1.origin = DVec3::new(0.0, 0.0, 3.0);
        let mut p2 = Plane::world_xy();
        p2.origin = DVec3::new(0.0, 0.0, 6.0);

        let c0 = NurbsCurve::circle(&p0, 2.0).unwrap();
        let c1 = NurbsCurve::circle(&p1, 3.0).unwrap();
        let c2 = NurbsCurve::circle(&p2, 1.5).unwrap();
        let lofted = loft_nurbs(&[c0.clone(), c1.clone(), c2.clone()], LoftStyle::Normal, false)
            .unwrap();
        assert!(lofted.is_rational());
        let (_, _, _, d1) = lofted.closest_point(c1.evaluate(0.25));
        assert!(d1 < 1e-6, "loft missed middle curve by {d1}");
        let (_, _, _, d2) = lofted.closest_point(c2.evaluate(0.6));
        assert!(d2 < 1e-6, "loft missed top curve by {d2}");

        let prof = NurbsCurve::line(DVec3::new(3.0, 0.0, 0.0), DVec3::new(3.0, 0.0, 4.0));
        let rev = revolve_nurbs(&prof, DVec3::ZERO, DVec3::Z, std::f64::consts::PI).unwrap();
        assert!((rev.evaluate(0.5, 0.5) - DVec3::new(0.0, 3.0, 2.0)).length() < 1e-8);

        let ell_rail = NurbsCurve::ellipse(&Plane::world_xy(), 4.0, 2.0).unwrap();
        let rail_prof = NurbsCurve::line(DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0, 0.0, 3.0));
        let rrev = rail_revolve_nurbs(&rail_prof, &ell_rail, DVec3::ZERO, DVec3::Z).unwrap();
        let (_, _, _, de) = rrev.closest_point(DVec3::new(0.0, 2.0, 1.5));
        assert!(de < 0.05);
    }

    #[test]
    fn sweep1_sweep2_edgesrf_and_networksrf() {
        // Sweep1 along 3D arc
        let rail = NurbsCurve::arc_3pt(
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(4.0, 4.0, 0.0),
            DVec3::new(8.0, 0.0, 0.0),
        )
        .unwrap();
        let profile = NurbsCurve::line(DVec3::new(0.0, 0.0, -1.0), DVec3::new(0.0, 0.0, 1.0));
        let sw1 = sweep1(&rail, &[profile], None).unwrap();
        let (_, _, _, dr) = sw1.closest_point(rail.evaluate(0.5));
        assert!(dr < 0.05);

        // Sweep2 anchors edges strictly onto both rails
        let r1 = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(4.0, 0.5, 0.0),
                DVec3::new(8.0, 0.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let r2 = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 4.0, 0.0),
                DVec3::new(4.0, 5.5, 0.0),
                DVec3::new(8.0, 4.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let sec = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(0.0, 2.0, 1.5),
                DVec3::new(0.0, 4.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let sw2 = sweep2(&r1, &r2, &[sec]).unwrap();
        assert!(sw2.evaluate(0.0, 0.5).distance(r1.evaluate(0.5)) < 0.05);
        assert!(sw2.evaluate(1.0, 0.5).distance(r2.evaluate(0.5)) < 0.05);

        // EdgeSrf (4-sided and 3-sided Coons patches)
        let e0 = NurbsCurve::line(DVec3::new(0.0, 0.0, 0.0), DVec3::new(4.0, 0.0, 0.0));
        let e1 = NurbsCurve::interpolate(
            &[
                DVec3::new(4.0, 0.0, 0.0),
                DVec3::new(4.5, 2.0, 1.0),
                DVec3::new(4.0, 4.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let e2 = NurbsCurve::line(DVec3::new(4.0, 4.0, 0.0), DVec3::new(0.0, 4.0, 0.0));
        let e3 = NurbsCurve::line(DVec3::new(0.0, 4.0, 0.0), DVec3::new(0.0, 0.0, 0.0));
        let coons4 = edge_srf(&[e0.clone(), e1, e2, e3]).unwrap();
        let (_, _, _, d_edge) = coons4.closest_point(DVec3::new(4.5, 2.0, 1.0));
        assert!(d_edge < 0.05);

        let tri_c1 = NurbsCurve::line(DVec3::new(4.0, 0.0, 0.0), DVec3::new(2.0, 3.0, 1.0));
        let tri_c2 = NurbsCurve::line(DVec3::new(2.0, 3.0, 1.0), DVec3::ZERO);
        let coons3 = edge_srf(&[e0, tri_c1, tri_c2]).unwrap();
        let (_, _, _, d_apex) = coons3.closest_point(DVec3::new(2.0, 3.0, 1.0));
        assert!(d_apex < 1e-4);

        // NetworkSrf partitions intersecting curve grid and interpolates
        let u_a = NurbsCurve::line(DVec3::new(0.0, 0.0, 0.0), DVec3::new(6.0, 0.0, 0.0));
        let u_b = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 3.0, 0.0),
                DVec3::new(3.0, 3.0, 1.5),
                DVec3::new(6.0, 3.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let u_c = NurbsCurve::line(DVec3::new(0.0, 6.0, 0.0), DVec3::new(6.0, 6.0, 0.0));
        let v_a = NurbsCurve::line(DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 6.0, 0.0));
        let v_b = NurbsCurve::interpolate(
            &[
                DVec3::new(3.0, 0.0, 0.0),
                DVec3::new(3.0, 3.0, 1.5),
                DVec3::new(3.0, 6.0, 0.0),
            ],
            2,
            false,
        )
        .unwrap();
        let v_c = NurbsCurve::line(DVec3::new(6.0, 0.0, 0.0), DVec3::new(6.0, 6.0, 0.0));
        let (ug, vg) =
            partition_network_curves(&[u_a, v_a, u_b, v_b, u_c, v_c], 1e-3).unwrap();
        let net = network_srf(&ug, &vg).unwrap();
        let (_, _, _, dn) = net.closest_point(DVec3::new(3.0, 3.0, 1.5));
        assert!(dn < 0.05, "NetworkSrf missed dome peak by {dn}");
    }

    #[test]
    fn blend_match_offset_extend_patch_rebuild_and_soft_transform() {
        let p_left = Plane {
            origin: DVec3::new(-4.0, 0.0, 0.0),
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        };
        let p_right = Plane {
            origin: DVec3::new(4.0, 0.0, 0.0),
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        };
        let srf_a = NurbsSurface::plane_rect(&p_left, 4.0, 4.0);
        let srf_b = NurbsSurface::plane_rect(&p_right, 4.0, 4.0);
        let (ea, eb) = closest_edge_pair(&srf_a, &srf_b);
        assert_eq!(ea, SurfaceEdge::UMax);
        assert_eq!(eb, SurfaceEdge::UMin);

        let blend = blend_srf(&srf_a, ea, &srf_b, eb, Continuity::G2, 1.0, 1.0).unwrap();
        let f_seam_a = srf_a.surface_frame(1.0, 0.5);
        let f_blend_a = blend.surface_frame(0.5, 0.0);
        assert!(f_seam_a.normal.angle_between(f_blend_a.normal) < 1e-4);

        // MatchSrf G1 tilts srf_right onto reference srf_left
        let mut tilted = NurbsSurface::plane_rect(&p_right, 4.0, 4.0);
        tilted.transform(&DMat4::from_rotation_y(0.4));
        let matched = match_srf(&tilted, SurfaceEdge::UMin, &srf_a, SurfaceEdge::UMax, Continuity::G1)
            .unwrap();
        let (_, _, _, d_gap) = matched.closest_point(DVec3::new(-2.0, 0.0, 0.0));
        assert!(d_gap < 1e-4);
        assert!(matched.surface_frame(0.0, 0.5).normal.angle_between(DVec3::Z) < 0.03);

        // Offset, Extend, Rebuild
        let off = offset_srf(&srf_a, 2.5).unwrap();
        assert!((off.evaluate(0.5, 0.5).z - 2.5).abs() < 1e-6);

        let ext = extend_srf(&srf_a, SurfaceEdge::UMax, 3.0).unwrap();
        assert!(ext.bounding_box().max.x > 0.9);

        let cyl = NurbsSurface::cylinder(DVec3::ZERO, 2.0, 5.0).unwrap();
        let rep = rebuild_srf(&cyl, 6, 16, 2, 3).unwrap();
        assert!(rep.max_deviation < 0.05);

        // Rhino 9-style Patch with boundary + raised interior peak
        let rect = NurbsCurve::rectangle(&Plane::world_xy(), DVec3::ZERO, DVec3::new(6.0, 6.0, 0.0))
            .unwrap();
        let segs = rect.explode_segments();
        let bcs: Vec<PatchBoundaryConstraint> = segs
            .into_iter()
            .map(|c| PatchBoundaryConstraint {
                curve: c,
                reference_surface: None,
            })
            .collect();
        let patch_rep = patch_srf(
            &bcs,
            &[DVec3::new(3.0, 3.0, 2.0)],
            &PatchOptions::default(),
        )
        .unwrap();
        assert!(patch_rep.max_deviation < 0.2);

        // Soft Transform with FixG0, FixG1, FixG2 continuity ring preservation
        let mut grid_srf = NurbsSurface::plane_rect(&Plane::world_xy(), 10.0, 10.0);
        for _ in 0..2 {
            grid_srf = grid_srf.elevate_degree(1, 1).unwrap();
        }
        // Refine to 9x9 grid so FixG2 has interior movable CPs
        for t in [0.16, 0.33, 0.5, 0.66, 0.84, 0.25] {
            grid_srf = grid_srf.insert_knot_u(t, 1).unwrap().insert_knot_v(t, 1).unwrap();
        }
        let center_idx = (grid_srf.count_u / 2) * grid_srf.count_v + (grid_srf.count_v / 2);

        for (lock, min_rings) in [
            (BoundaryLock::FixG0, 1),
            (BoundaryLock::FixG1, 2),
            (BoundaryLock::FixG2, 3),
        ] {
            let before = grid_srf.clone();
            let mut sculpted = grid_srf.clone();
            apply_soft_transform_surface(
                &mut sculpted,
                &[center_idx],
                DVec3::new(0.0, 0.0, 2.0),
                6.0,
                SoftTransformFalloff::Smooth,
                lock,
            );
            assert!(
                (sculpted.control_points[center_idx].position.z
                    - (before.control_points[center_idx].position.z + 2.0))
                    .abs()
                    < 1e-9
            );
            for u in 0..sculpted.count_u {
                for v in 0..sculpted.count_v {
                    let is_locked = u < min_rings
                        || u + min_rings >= sculpted.count_u
                        || v < min_rings
                        || v + min_rings >= sculpted.count_v;
                    if is_locked {
                        assert!(
                            sculpted.get_cp(u, v).position.distance(before.get_cp(u, v).position)
                                < 1e-12,
                            "Boundary CP ({u},{v}) moved under {lock:?}"
                        );
                    }
                }
            }
        }
    }
}
