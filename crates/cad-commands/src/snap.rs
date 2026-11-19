use cad_doc::Document;
use cad_geom::{Geometry, NurbsCurve, intersect_curves};
use cad_math::{DVec3, Plane};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapKind {
    End,
    Mid,
    Center,
    Quad,
    Int,
    Perp,
    Tan,
    Knot,
    Vertex,
    SmartTrack,
    AxisLock,
    Near,
    Grid,
    Ortho,
}

impl SnapKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::End => "End",
            Self::Mid => "Mid",
            Self::Center => "Cen",
            Self::Quad => "Quad",
            Self::Int => "Int",
            Self::Perp => "Perp",
            Self::Tan => "Tan",
            Self::Knot => "Knot",
            Self::Vertex => "Vert",
            Self::SmartTrack => "STrack",
            Self::AxisLock => "Axis",
            Self::Near => "Near",
            Self::Grid => "Grid",
            Self::Ortho => "Ortho",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisLockMode {
    None,
    X,
    Y,
    Z,
}

impl AxisLockMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::None => "Free",
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InferenceGuideKind {
    AxisX,
    AxisY,
    AxisZ,
    Extension,
    Perp,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InferenceGuide {
    pub start: DVec3,
    pub end: DVec3,
    pub kind: InferenceGuideKind,
    pub label: &'static str,
}

#[derive(Debug, Clone)]
pub struct SnapConfig {
    pub grid_snap: bool,
    pub grid_step: f64,
    pub ortho: bool,
    pub osnap_enabled: bool,
    pub osnap_end: bool,
    pub osnap_mid: bool,
    pub osnap_center: bool,
    pub osnap_quad: bool,
    pub osnap_int: bool,
    pub osnap_perp: bool,
    pub osnap_tan: bool,
    pub osnap_knot: bool,
    pub osnap_vertex: bool,
    pub osnap_near: bool,
    pub smart_track: bool,
    pub smart_track_points: Vec<DVec3>,
    pub axis_lock: AxisLockMode,
    pub snap_radius: f64,
    pub snap_radius_px: f64,
}

impl Default for SnapConfig {
    fn default() -> Self {
        Self {
            grid_snap: false,
            grid_step: 1.0,
            ortho: false,
            osnap_enabled: true,
            osnap_end: true,
            osnap_mid: true,
            osnap_center: true,
            osnap_quad: true,
            osnap_int: true,
            osnap_perp: false,
            osnap_tan: true,
            osnap_knot: true,
            osnap_vertex: true,
            osnap_near: false,
            smart_track: true,
            smart_track_points: Vec::new(),
            axis_lock: AxisLockMode::None,
            snap_radius: 0.6,
            snap_radius_px: 14.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct SnapResult {
    pub point: DVec3,
    pub kind: Option<SnapKind>,
    pub guides: Vec<InferenceGuide>,
}

impl SnapConfig {
    pub fn add_smart_track_point(&mut self, pt: DVec3) {
        if let Some(pos) = self
            .smart_track_points
            .iter()
            .position(|&p| p.distance(pt) <= 1e-4)
        {
            self.smart_track_points.remove(pos);
        }
        self.smart_track_points.push(pt);
        if self.smart_track_points.len() > 6 {
            self.smart_track_points.remove(0);
        }
    }

    pub fn clear_smart_track(&mut self) {
        self.smart_track_points.clear();
    }

    pub fn resolve(
        &self,
        raw_point: DVec3,
        base_point: Option<DVec3>,
        doc: &Document,
        cplane: &Plane,
    ) -> SnapResult {
        self.resolve_with_radius(raw_point, base_point, doc, cplane, self.snap_radius)
    }

    pub fn resolve_with_radius(
        &self,
        raw_point: DVec3,
        base_point: Option<DVec3>,
        doc: &Document,
        cplane: &Plane,
        world_snap_radius: f64,
    ) -> SnapResult {
        let osnap_hit = if self.osnap_enabled {
            self.find_osnap(raw_point, base_point, doc, world_snap_radius)
        } else {
            None
        };

        if self.axis_lock != AxisLockMode::None {
            let base = base_point.unwrap_or(cplane.origin);
            let (dir, guide_kind, lbl) = match self.axis_lock {
                AxisLockMode::X => (cplane.x_axis.normalize_or(DVec3::X), InferenceGuideKind::AxisX, "Lock X"),
                AxisLockMode::Y => (cplane.y_axis.normalize_or(DVec3::Y), InferenceGuideKind::AxisY, "Lock Y"),
                AxisLockMode::Z => (cplane.z_axis.normalize_or(DVec3::Z), InferenceGuideKind::AxisZ, "Lock Z"),
                AxisLockMode::None => unreachable!(),
            };
            let source_pt = osnap_hit.map(|h| h.0).unwrap_or(raw_point);
            let t = (source_pt - base).dot(dir);
            let mut locked_pt = base + dir * t;
            if osnap_hit.is_none() && self.grid_snap && self.grid_step > 0.0 {
                let stepped = (t / self.grid_step).round() * self.grid_step;
                locked_pt = base + dir * stepped;
            }
            let mut guides = vec![InferenceGuide {
                start: base,
                end: locked_pt,
                kind: guide_kind,
                label: lbl,
            }];
            if let Some((snap_pt, _)) = osnap_hit {
                if snap_pt.distance(locked_pt) > 1e-4 {
                    guides.push(InferenceGuide {
                        start: snap_pt,
                        end: locked_pt,
                        kind: InferenceGuideKind::Extension,
                        label: "Align",
                    });
                }
            }
            return SnapResult {
                point: locked_pt,
                kind: Some(SnapKind::AxisLock),
                guides,
            };
        }

        if let Some((pt, kind)) = osnap_hit {
            return SnapResult {
                point: pt,
                kind: Some(kind),
                guides: Vec::new(),
            };
        }

        let mut guides = Vec::new();
        if self.smart_track {
            if let Some((st_pt, st_guides)) =
                self.resolve_smart_track(raw_point, base_point, cplane, world_snap_radius)
            {
                return SnapResult {
                    point: st_pt,
                    kind: Some(SnapKind::SmartTrack),
                    guides: st_guides,
                };
            }
        }

        let mut p = raw_point;
        let mut kind = None;

        if self.ortho {
            if let Some(base) = base_point {
                let delta = p - base;
                let dx = delta.dot(cplane.x_axis);
                let dy = delta.dot(cplane.y_axis);
                if dx.abs() >= dy.abs() {
                    p = base + cplane.x_axis * dx;
                    guides.push(InferenceGuide {
                        start: base,
                        end: p,
                        kind: InferenceGuideKind::AxisX,
                        label: "Ortho X",
                    });
                } else {
                    p = base + cplane.y_axis * dy;
                    guides.push(InferenceGuide {
                        start: base,
                        end: p,
                        kind: InferenceGuideKind::AxisY,
                        label: "Ortho Y",
                    });
                }
                kind = Some(SnapKind::Ortho);
            }
        }

        if self.grid_snap && self.grid_step > 0.0 {
            let rel = p - cplane.origin;
            let u = (rel.dot(cplane.x_axis) / self.grid_step).round() * self.grid_step;
            let v = (rel.dot(cplane.y_axis) / self.grid_step).round() * self.grid_step;
            p = cplane.origin + cplane.x_axis * u + cplane.y_axis * v;
            kind = Some(SnapKind::Grid);
        }

        SnapResult {
            point: p,
            kind,
            guides,
        }
    }

    fn resolve_smart_track(
        &self,
        raw_point: DVec3,
        base_point: Option<DVec3>,
        cplane: &Plane,
        snap_radius: f64,
    ) -> Option<(DVec3, Vec<InferenceGuide>)> {
        let mut rays: Vec<(DVec3, DVec3, InferenceGuideKind, &'static str, bool)> = Vec::new();
        let xa = cplane.x_axis.normalize_or(DVec3::X);
        let ya = cplane.y_axis.normalize_or(DVec3::Y);
        let za = cplane.z_axis.normalize_or(DVec3::Z);

        for &anchor in &self.smart_track_points {
            rays.push((anchor, xa, InferenceGuideKind::AxisX, "STrack X", false));
            rays.push((anchor, ya, InferenceGuideKind::AxisY, "STrack Y", false));
        }
        if self.smart_track_points.len() >= 2 {
            let a = self.smart_track_points[self.smart_track_points.len() - 2];
            let b = self.smart_track_points[self.smart_track_points.len() - 1];
            let ext = (b - a).normalize_or_zero();
            if ext.length_squared() > 1e-8 {
                rays.push((b, ext, InferenceGuideKind::Extension, "Ext", false));
                let perp = za.cross(ext).normalize_or_zero();
                if perp.length_squared() > 1e-8 {
                    rays.push((b, perp, InferenceGuideKind::Perp, "Perp", false));
                }
            }
        }
        if let Some(base) = base_point {
            rays.push((base, xa, InferenceGuideKind::AxisX, "Track X", true));
            rays.push((base, ya, InferenceGuideKind::AxisY, "Track Y", true));
        }

        if rays.is_empty() {
            return None;
        }

        let thresh = snap_radius.max(1e-5) * 0.85;
        let mut hits: Vec<(usize, DVec3, f64)> = Vec::new();
        for (i, &(orig, dir, _, _, _)) in rays.iter().enumerate() {
            let proj = orig + dir * (raw_point - orig).dot(dir);
            let d = proj.distance(raw_point);
            if d <= thresh {
                hits.push((i, proj, d));
            }
        }
        hits.sort_by(|a, b| a.2.total_cmp(&b.2));

        // 2-guide intersection (highest precision inference)
        for i in 0..hits.len() {
            let r0 = rays[hits[i].0];
            for j in (i + 1)..hits.len() {
                let r1 = rays[hits[j].0];
                if r0.1.dot(r1.1).abs() > 0.995 {
                    continue;
                }
                if let Some(inter) = closest_line_line_mid(r0.0, r0.1, r1.0, r1.1) {
                    if inter.distance(raw_point) <= snap_radius.max(1e-5) * 1.15 {
                        return Some((
                            inter,
                            vec![
                                InferenceGuide {
                                    start: r0.0,
                                    end: inter,
                                    kind: r0.2,
                                    label: r0.3,
                                },
                                InferenceGuide {
                                    start: r1.0,
                                    end: inter,
                                    kind: r1.2,
                                    label: r1.3,
                                },
                            ],
                        ));
                    }
                }
            }
        }

        // Single ray track (prefer explicit SmartTrack anchor rays over plain base Ortho if Ortho is off)
        if let Some(&(idx, proj, _)) = hits.first() {
            let r = rays[idx];
            if r.4 && self.ortho {
                return None;
            }
            return Some((
                proj,
                vec![InferenceGuide {
                    start: r.0,
                    end: proj,
                    kind: r.2,
                    label: r.3,
                }],
            ));
        }
        None
    }

    fn find_osnap(
        &self,
        raw_point: DVec3,
        base_point: Option<DVec3>,
        doc: &Document,
        snap_radius: f64,
    ) -> Option<(DVec3, SnapKind)> {
        let mut best_dist = snap_radius.max(1e-6);
        let mut best: Option<(DVec3, SnapKind)> = None;
        let mut nearby_curves: Vec<&NurbsCurve> = Vec::new();

        for (_, obj) in &doc.objects {
            if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
                continue;
            }
            match &obj.geometry {
                Geometry::Point(pt) => {
                    if self.osnap_end {
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((*pt, SnapKind::End));
                        }
                    }
                }
                Geometry::Curve(curve) => {
                    if self.osnap_int || self.osnap_perp || self.osnap_tan {
                        let bb = obj.aabb.expanded(snap_radius * 4.0);
                        if raw_point.x >= bb.min.x
                            && raw_point.x <= bb.max.x
                            && raw_point.y >= bb.min.y
                            && raw_point.y <= bb.max.y
                            && raw_point.z >= bb.min.z
                            && raw_point.z <= bb.max.z
                        {
                            nearby_curves.push(curve);
                        }
                    }
                    let (u0, u1) = curve.domain();
                    if self.osnap_end {
                        if curve.degree == 1 {
                            for cp in &curve.control_points {
                                let d = cp.position.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((cp.position, SnapKind::End));
                                }
                            }
                        } else {
                            for u in [u0, u1] {
                                let pt = curve.evaluate(u);
                                let d = pt.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((pt, SnapKind::End));
                                }
                            }
                        }
                    }
                    if self.osnap_mid {
                        let pt = curve.evaluate((u0 + u1) * 0.5);
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((pt, SnapKind::Mid));
                        }
                    }
                    if self.osnap_center {
                        let pt = obj.aabb.center();
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((pt, SnapKind::Center));
                        }
                    }
                    if self.osnap_quad {
                        let p0 = curve.evaluate(u0);
                        let p1 = curve.evaluate(u1);
                        if p0.distance(p1) < 1e-4 {
                            for frac in [0.25, 0.75] {
                                let pt = curve.evaluate(u0 + (u1 - u0) * frac);
                                let d = pt.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((pt, SnapKind::Quad));
                                }
                            }
                        }
                    }
                    if self.osnap_knot {
                        for u in unique_interior_knots(&curve.knots, u0, u1) {
                            let pt = curve.evaluate(u);
                            let d = pt.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((pt, SnapKind::Knot));
                            }
                        }
                    }
                    if self.osnap_vertex && curve.degree > 1 {
                        for cp in &curve.control_points {
                            let d = cp.position.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((cp.position, SnapKind::Vertex));
                            }
                        }
                    }
                }
                Geometry::Surface(surf) => {
                    let (u0, u1) = surf.domain_u();
                    let (v0, v1) = surf.domain_v();
                    if self.osnap_end {
                        for (u, v) in [(u0, v0), (u1, v0), (u0, v1), (u1, v1)] {
                            let pt = surf.evaluate(u, v);
                            let d = pt.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((pt, SnapKind::End));
                            }
                        }
                    }
                    if self.osnap_knot {
                        let ku = unique_interior_knots(&surf.knots_u, u0, u1);
                        let kv = unique_interior_knots(&surf.knots_v, v0, v1);
                        for &u in &ku {
                            for &v in &kv {
                                let pt = surf.evaluate(u, v);
                                let d = pt.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((pt, SnapKind::Knot));
                                }
                            }
                        }
                    }
                    if self.osnap_center {
                        let pt = obj.aabb.center();
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((pt, SnapKind::Center));
                        }
                    }
                }
                Geometry::BRep(brep) => {
                    if self.osnap_end {
                        for v in &brep.vertices {
                            let d = v.position.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((v.position, SnapKind::End));
                            }
                        }
                        for edge in &brep.edges {
                            let (u0, u1) = edge.curve.domain();
                            for u in [u0, u1] {
                                let pt = edge.curve.evaluate(u);
                                let d = pt.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((pt, SnapKind::End));
                                }
                            }
                        }
                    } else if self.osnap_vertex {
                        for v in &brep.vertices {
                            let d = v.position.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((v.position, SnapKind::Vertex));
                            }
                        }
                    }
                    if self.osnap_mid {
                        for edge in &brep.edges {
                            let (u0, u1) = edge.curve.domain();
                            let pt = edge.curve.evaluate(0.5 * (u0 + u1));
                            let d = pt.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((pt, SnapKind::Mid));
                            }
                        }
                    }
                    if self.osnap_center {
                        let pt = obj.aabb.center();
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((pt, SnapKind::Center));
                        }
                    }
                }
                Geometry::Mesh(mesh) => {
                    if self.osnap_vertex {
                        let bb = obj.aabb.expanded(snap_radius);
                        if raw_point.x >= bb.min.x
                            && raw_point.x <= bb.max.x
                            && raw_point.y >= bb.min.y
                            && raw_point.y <= bb.max.y
                            && raw_point.z >= bb.min.z
                            && raw_point.z <= bb.max.z
                        {
                            let step = (mesh.positions.len() / 512).max(1);
                            for pos in mesh.positions.iter().step_by(step) {
                                let pt = DVec3::from_array(*pos);
                                let d = pt.distance(raw_point);
                                if d < best_dist {
                                    best_dist = d;
                                    best = Some((pt, SnapKind::Vertex));
                                }
                            }
                        }
                    }
                    if self.osnap_center {
                        let pt = obj.aabb.center();
                        let d = pt.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((pt, SnapKind::Center));
                        }
                    }
                }
            }
        }

        if self.osnap_int && nearby_curves.len() >= 2 {
            let tol = (snap_radius * 0.5).max(1e-3);
            for i in 0..nearby_curves.len() {
                for j in (i + 1)..nearby_curves.len() {
                    for hit in intersect_curves(nearby_curves[i], nearby_curves[j], tol) {
                        let d = hit.point.distance(raw_point);
                        let wins_tie = (d - best_dist).abs() <= 1e-6
                            && !matches!(best.map(|b| b.1), Some(SnapKind::End));
                        if d < best_dist || wins_tie {
                            best_dist = d;
                            best = Some((hit.point, SnapKind::Int));
                        }
                    }
                }
            }
        }

        if self.osnap_perp {
            if let Some(base) = base_point {
                for curve in &nearby_curves {
                    let (_, foot, _) = curve.closest_point(base);
                    let d = foot.distance(raw_point);
                    if d < best_dist {
                        best_dist = d;
                        best = Some((foot, SnapKind::Perp));
                    }
                }
            }
        }

        if self.osnap_tan {
            if let Some(base) = base_point {
                for curve in &nearby_curves {
                    if let Some(tp) = find_tangent_point(curve, base, raw_point, best_dist) {
                        let d = tp.distance(raw_point);
                        if d < best_dist {
                            best_dist = d;
                            best = Some((tp, SnapKind::Tan));
                        }
                    }
                }
            }
        }

        if best.is_none() && self.osnap_near {
            let mut near_best = snap_radius.max(1e-6);
            let mut check_curve_near = |curve: &NurbsCurve| {
                let pts = curve.sample_polyline(16);
                for w in pts.windows(2) {
                    let ab = w[1] - w[0];
                    let len_sq = ab.length_squared();
                    let t = if len_sq > 1e-12 {
                        ((raw_point - w[0]).dot(ab) / len_sq).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let proj = w[0] + ab * t;
                    let d = proj.distance(raw_point);
                    if d < near_best {
                        near_best = d;
                        best = Some((proj, SnapKind::Near));
                    }
                }
            };
            for (_, obj) in &doc.objects {
                if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
                    continue;
                }
                match &obj.geometry {
                    Geometry::Curve(curve) => check_curve_near(curve),
                    Geometry::BRep(brep) => {
                        for edge in &brep.edges {
                            check_curve_near(&edge.curve);
                        }
                    }
                    _ => {}
                }
            }
        }

        best
    }
}

fn unique_interior_knots(knots: &[f64], u0: f64, u1: f64) -> Vec<f64> {
    let mut out = Vec::new();
    for &u in knots {
        if u > u0 + 1e-7 && u < u1 - 1e-7 && out.last().is_none_or(|&prev: &f64| (u - prev).abs() > 1e-7) {
            out.push(u);
        }
    }
    out
}

fn find_tangent_point(
    curve: &NurbsCurve,
    base: DVec3,
    raw_point: DVec3,
    max_dist: f64,
) -> Option<DVec3> {
    if curve.degree < 2 {
        return None;
    }
    let (u0, u1) = curve.domain();
    let samples = 48usize;
    let mut best_pt = None;
    let mut best_d = max_dist;

    let (_, dt_raw) = curve.evaluate_derivative(raw_point.length().clamp(u0, u1));
    let mut plane_n = {
        let (p0, t0) = curve.evaluate_derivative(u0);
        let (pm, tm) = curve.evaluate_derivative(0.5 * (u0 + u1));
        let c1 = (p0 - base).cross(t0);
        let c2 = (pm - base).cross(tm);
        if c1.length_squared() > 1e-10 {
            c1.normalize()
        } else if c2.length_squared() > 1e-10 {
            c2.normalize()
        } else if dt_raw.length_squared() > 1e-10 {
            DVec3::Z
        } else {
            DVec3::Z
        }
    };
    if plane_n.length_squared() < 1e-8 {
        plane_n = DVec3::Z;
    }

    let eval_cross = |u: f64| -> (DVec3, DVec3, f64) {
        let (p, t) = curve.evaluate_derivative(u);
        let v = p - base;
        (p, t, v.cross(t).dot(plane_n))
    };

    let mut prev_u = u0;
    let (_, _, mut prev_f) = eval_cross(u0);
    for i in 1..=samples {
        let u = u0 + (u1 - u0) * (i as f64 / samples as f64);
        let (_, _, f) = eval_cross(u);
        if (prev_f <= 0.0 && f >= 0.0) || (prev_f >= 0.0 && f <= 0.0) {
            let mut a = prev_u;
            let mut b = u;
            let mut fa = prev_f;
            for _ in 0..16 {
                let mid = 0.5 * (a + b);
                let (_, _, fm) = eval_cross(mid);
                if (fa <= 0.0 && fm >= 0.0) || (fa >= 0.0 && fm <= 0.0) {
                    b = mid;
                } else {
                    a = mid;
                    fa = fm;
                }
            }
            let um = 0.5 * (a + b);
            let (pm, tm, _) = eval_cross(um);
            let v_dir = (pm - base).normalize_or_zero();
            let t_dir = tm.normalize_or_zero();
            if v_dir.dot(t_dir).abs() > 0.99 {
                let d = pm.distance(raw_point);
                if d < best_d {
                    best_d = d;
                    best_pt = Some(pm);
                }
            }
        }
        prev_u = u;
        prev_f = f;
    }
    best_pt
}

fn closest_line_line_mid(o0: DVec3, d0: DVec3, o1: DVec3, d1: DVec3) -> Option<DVec3> {
    let r = o0 - o1;
    let a = d0.dot(d0);
    let e = d1.dot(d1);
    let f = d1.dot(r);
    let b = d0.dot(d1);
    let c = d0.dot(r);
    let denom = a * e - b * b;
    if denom.abs() < 1e-10 {
        return None;
    }
    let s = (b * f - c * e) / denom;
    let t = (a * f - b * c) / denom;
    let p0 = o0 + d0 * s;
    let p1 = o1 + d1 * t;
    if p0.distance(p1) <= 1e-2 {
        Some(0.5 * (p0 + p1))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::NurbsCurve;

    #[test]
    fn osnap_snaps_to_line_endpoint_and_midpoint() {
        let mut doc = Document::new();
        doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0))),
        );
        let cfg = SnapConfig::default();
        let res_end = cfg.resolve(DVec3::new(0.2, 0.1, 0.0), None, &doc, &Plane::world_xy());
        assert_eq!(res_end.kind, Some(SnapKind::End));
        assert!(res_end.point.length() < 1e-9);

        let res_mid = cfg.resolve(DVec3::new(5.1, -0.1, 0.0), None, &doc, &Plane::world_xy());
        assert_eq!(res_mid.kind, Some(SnapKind::Mid));
        assert!((res_mid.point - DVec3::new(5.0, 0.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn ortho_constrains_to_dominant_axis() {
        let doc = Document::new();
        let cfg = SnapConfig {
            osnap_enabled: false,
            smart_track: false,
            ortho: true,
            ..Default::default()
        };
        let res = cfg.resolve(
            DVec3::new(8.0, 2.0, 0.0),
            Some(DVec3::ZERO),
            &doc,
            &Plane::world_xy(),
        );
        assert_eq!(res.kind, Some(SnapKind::Ortho));
        assert!((res.point - DVec3::new(8.0, 0.0, 0.0)).length() < 1e-9);
        assert_eq!(res.guides.len(), 1);
    }

    #[test]
    fn osnap_quad_and_near_snap_on_curves() {
        let mut doc = Document::new();
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        doc.add_object("Circle", Geometry::Curve(circle));

        let cfg = SnapConfig::default();
        let res_quad = cfg.resolve(DVec3::new(0.1, 4.9, 0.0), None, &doc, &Plane::world_xy());
        assert_eq!(res_quad.kind, Some(SnapKind::Quad));
        assert!((res_quad.point - DVec3::new(0.0, 5.0, 0.0)).length() < 1e-3);

        let mut doc2 = Document::new();
        doc2.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0))),
        );
        let cfg_near = SnapConfig {
            osnap_near: true,
            smart_track: false,
            ..Default::default()
        };
        let res_near = cfg_near.resolve(DVec3::new(2.5, 0.2, 0.0), None, &doc2, &Plane::world_xy());
        assert_eq!(res_near.kind, Some(SnapKind::Near));
        assert!((res_near.point - DVec3::new(2.5, 0.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn osnap_skips_hidden_objects_and_supports_zoom_radius_and_corners() {
        use cad_geom::NurbsSurface;

        let mut doc = Document::new();
        let rect = NurbsCurve::rectangle(&Plane::world_xy(), DVec3::ZERO, DVec3::new(4.0, 4.0, 0.0)).unwrap();
        let rect_id = doc.add_object("Rect", Geometry::Curve(rect));
        let cfg = SnapConfig::default();

        // Corner of polyline (4,0,0) is an interior vertex of closed polyline
        let res_corner = cfg.resolve_with_radius(DVec3::new(3.9, 0.1, 0.0), None, &doc, &Plane::world_xy(), 0.3);
        assert_eq!(res_corner.kind, Some(SnapKind::End));
        assert!((res_corner.point - DVec3::new(4.0, 0.0, 0.0)).length() < 1e-9);

        // Smaller zoom radius rejects 0.14 distance while larger radius accepts
        let res_tight = cfg.resolve_with_radius(DVec3::new(3.9, 0.1, 0.0), None, &doc, &Plane::world_xy(), 0.05);
        assert_eq!(res_tight.kind, None);

        // Hidden object must be skipped
        doc.objects[rect_id].visible = false;
        let res_hidden = cfg.resolve_with_radius(DVec3::new(4.0, 0.0, 0.0), None, &doc, &Plane::world_xy(), 0.5);
        assert_eq!(res_hidden.kind, None);

        // Surface corner snap
        let mut doc_srf = Document::new();
        let srf = NurbsSurface::plane_rect(&Plane::world_xy(), 6.0, 4.0);
        doc_srf.add_object("Plane", Geometry::Surface(srf));
        let res_srf = cfg.resolve_with_radius(DVec3::new(2.9, 1.9, 0.0), None, &doc_srf, &Plane::world_xy(), 0.5);
        assert_eq!(res_srf.kind, Some(SnapKind::End));
        assert!((res_srf.point - DVec3::new(3.0, 2.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn osnap_intersection_and_perpendicular() {
        let mut doc = Document::new();
        doc.add_object(
            "LineH",
            Geometry::Curve(NurbsCurve::line(
                DVec3::new(-5.0, 0.0, 0.0),
                DVec3::new(5.0, 0.0, 0.0),
            )),
        );
        doc.add_object(
            "LineV",
            Geometry::Curve(NurbsCurve::line(
                DVec3::new(1.0, -3.0, 0.0),
                DVec3::new(1.0, 7.0, 0.0),
            )),
        );

        let cfg_int = SnapConfig::default();
        let res_int = cfg_int.resolve(DVec3::new(1.15, 0.1, 0.0), None, &doc, &Plane::world_xy());
        assert_eq!(res_int.kind, Some(SnapKind::Int));
        assert!((res_int.point - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-6);

        let cfg_perp = SnapConfig {
            osnap_end: false,
            osnap_mid: false,
            osnap_center: false,
            osnap_quad: false,
            osnap_int: false,
            osnap_perp: true,
            smart_track: false,
            ..Default::default()
        };
        let res_perp = cfg_perp.resolve(
            DVec3::new(-2.1, 0.1, 0.0),
            Some(DVec3::new(-2.0, 4.0, 0.0)),
            &doc,
            &Plane::world_xy(),
        );
        assert_eq!(res_perp.kind, Some(SnapKind::Perp));
        assert!((res_perp.point - DVec3::new(-2.0, 0.0, 0.0)).length() < 1e-6);
    }

    #[test]
    fn osnap_tangent_knot_and_vertex_snap_accurately() {
        use cad_geom::TriangleMesh;

        let mut doc = Document::new();
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        doc.add_object("Circle", Geometry::Curve(circle.clone()));

        // Base at (-10, 0, 0): exact tangent to r=5 circle is at (-2.5, +4.330127, 0)
        let cfg_tan = SnapConfig {
            osnap_end: false,
            osnap_mid: false,
            osnap_center: false,
            osnap_quad: false,
            osnap_int: false,
            osnap_perp: false,
            osnap_knot: false,
            osnap_vertex: false,
            osnap_tan: true,
            smart_track: false,
            ..Default::default()
        };
        let expected_tan = DVec3::new(-2.5, 5.0 * 3.0_f64.sqrt() * 0.5, 0.0);
        let res_tan = cfg_tan.resolve_with_radius(
            expected_tan + DVec3::new(0.15, 0.1, 0.0),
            Some(DVec3::new(-10.0, 0.0, 0.0)),
            &doc,
            &Plane::world_xy(),
            0.5,
        );
        assert_eq!(res_tan.kind, Some(SnapKind::Tan));
        assert!(res_tan.point.distance(expected_tan) < 1e-4);

        // Interior knot snap on a 5-point cubic interpolated spline (has interior knot)
        let mut doc_knot = Document::new();
        let spline = NurbsCurve::interpolate(
            &[
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(3.0, 2.0, 0.0),
                DVec3::new(6.0, -1.0, 0.0),
                DVec3::new(9.0, 2.0, 0.0),
                DVec3::new(12.0, 0.0, 0.0),
            ],
            3,
            false,
        )
        .unwrap();
        let (u0, u1) = spline.domain();
        let knot_u = *unique_interior_knots(&spline.knots, u0, u1).first().unwrap();
        let knot_pt = spline.evaluate(knot_u);
        doc_knot.add_object("Spline", Geometry::Curve(spline));
        let cfg_knot = SnapConfig {
            osnap_end: false,
            osnap_mid: false,
            osnap_center: false,
            osnap_quad: false,
            osnap_int: false,
            osnap_knot: true,
            osnap_vertex: false,
            smart_track: false,
            ..Default::default()
        };
        let res_knot = cfg_knot.resolve_with_radius(
            knot_pt + DVec3::new(0.1, 0.05, 0.0),
            None,
            &doc_knot,
            &Plane::world_xy(),
            0.5,
        );
        assert_eq!(res_knot.kind, Some(SnapKind::Knot));
        assert!(res_knot.point.distance(knot_pt) < 1e-9);

        // Vertex snap on a mesh
        let mut doc_mesh = Document::new();
        doc_mesh.add_object(
            "MeshBox",
            Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::new(4.0, 4.0, 2.0))),
        );
        let cfg_vert = SnapConfig {
            osnap_center: false,
            osnap_vertex: true,
            smart_track: false,
            ..Default::default()
        };
        let res_vert = cfg_vert.resolve_with_radius(
            DVec3::new(1.9, 1.9, 1.0),
            None,
            &doc_mesh,
            &Plane::world_xy(),
            0.4,
        );
        assert_eq!(res_vert.kind, Some(SnapKind::Vertex));
        assert!((res_vert.point - DVec3::new(2.0, 2.0, 1.0)).length() < 1e-9);
    }

    #[test]
    fn smart_track_guides_and_axis_lock_constrain_points() {
        let doc = Document::new();
        let mut cfg = SnapConfig {
            osnap_enabled: false,
            smart_track: true,
            ..Default::default()
        };
        cfg.add_smart_track_point(DVec3::new(4.0, 0.0, 0.0));
        cfg.add_smart_track_point(DVec3::new(0.0, 6.0, 0.0));

        // Near virtual intersection (4, 6, 0) snaps cleanly and emits 2 guides
        let res_st = cfg.resolve(DVec3::new(4.15, 5.9, 0.0), None, &doc, &Plane::world_xy());
        assert_eq!(res_st.kind, Some(SnapKind::SmartTrack));
        assert!((res_st.point - DVec3::new(4.0, 6.0, 0.0)).length() < 1e-6);
        assert_eq!(res_st.guides.len(), 2);

        // SketchUp-style AxisLock X projects onto X axis through base
        cfg.axis_lock = AxisLockMode::X;
        let res_ax = cfg.resolve(
            DVec3::new(7.5, 9.0, 3.0),
            Some(DVec3::new(1.0, 2.0, 0.0)),
            &doc,
            &Plane::world_xy(),
        );
        assert_eq!(res_ax.kind, Some(SnapKind::AxisLock));
        assert!((res_ax.point - DVec3::new(7.5, 2.0, 0.0)).length() < 1e-9);
        assert!(!res_ax.guides.is_empty());
    }
}
