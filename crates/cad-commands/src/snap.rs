use cad_doc::Document;
use cad_geom::Geometry;
use cad_math::{DVec3, Plane};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapKind {
    End,
    Mid,
    Center,
    Quad,
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
            Self::Near => "Near",
            Self::Grid => "Grid",
            Self::Ortho => "Ortho",
        }
    }
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
    pub osnap_near: bool,
    pub snap_radius: f64,
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
            osnap_near: false,
            snap_radius: 0.6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnapResult {
    pub point: DVec3,
    pub kind: Option<SnapKind>,
}

impl SnapConfig {
    pub fn resolve(
        &self,
        raw_point: DVec3,
        base_point: Option<DVec3>,
        doc: &Document,
        cplane: &Plane,
    ) -> SnapResult {
        if self.osnap_enabled {
            if let Some((pt, kind)) = self.find_osnap(raw_point, doc) {
                return SnapResult {
                    point: pt,
                    kind: Some(kind),
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
                p = if dx.abs() >= dy.abs() {
                    base + cplane.x_axis * dx
                } else {
                    base + cplane.y_axis * dy
                };
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

        SnapResult { point: p, kind }
    }

    fn find_osnap(&self, raw_point: DVec3, doc: &Document) -> Option<(DVec3, SnapKind)> {
        let mut best_dist = self.snap_radius;
        let mut best: Option<(DVec3, SnapKind)> = None;

        for (_, obj) in &doc.objects {
            if let Some(layer) = doc.layers.get(obj.layer_id) {
                if !layer.visible {
                    continue;
                }
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
                    let (u0, u1) = curve.domain();
                    if self.osnap_end {
                        for u in [u0, u1] {
                            let pt = curve.evaluate(u);
                            let d = pt.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((pt, SnapKind::End));
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
                }
                Geometry::Mesh(_) | Geometry::Surface(_) => {
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

        if best.is_none() && self.osnap_near {
            let mut near_best = self.snap_radius;
            for (_, obj) in &doc.objects {
                if let Some(layer) = doc.layers.get(obj.layer_id) {
                    if !layer.visible {
                        continue;
                    }
                }
                if let Geometry::Curve(curve) = &obj.geometry {
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
                }
            }
        }

        best
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
            ..Default::default()
        };
        let res_near = cfg_near.resolve(DVec3::new(2.5, 0.2, 0.0), None, &doc2, &Plane::world_xy());
        assert_eq!(res_near.kind, Some(SnapKind::Near));
        assert!((res_near.point - DVec3::new(2.5, 0.0, 0.0)).length() < 1e-9);
    }
}
