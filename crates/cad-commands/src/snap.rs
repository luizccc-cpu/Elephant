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
            osnap_near: false,
            snap_radius: 0.6,
            snap_radius_px: 14.0,
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
        if self.osnap_enabled {
            if let Some((pt, kind)) = self.find_osnap(raw_point, doc, world_snap_radius) {
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

    fn find_osnap(
        &self,
        raw_point: DVec3,
        doc: &Document,
        snap_radius: f64,
    ) -> Option<(DVec3, SnapKind)> {
        let mut best_dist = snap_radius.max(1e-6);
        let mut best: Option<(DVec3, SnapKind)> = None;

        for (_, obj) in &doc.objects {
            if !obj.visible {
                continue;
            }
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
                }
                Geometry::Surface(surf) => {
                    if self.osnap_end {
                        let (u0, u1) = surf.domain_u();
                        let (v0, v1) = surf.domain_v();
                        for (u, v) in [(u0, v0), (u1, v0), (u0, v1), (u1, v1)] {
                            let pt = surf.evaluate(u, v);
                            let d = pt.distance(raw_point);
                            if d < best_dist {
                                best_dist = d;
                                best = Some((pt, SnapKind::End));
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
                Geometry::Mesh(_) => {
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
            let mut near_best = snap_radius.max(1e-6);
            for (_, obj) in &doc.objects {
                if !obj.visible {
                    continue;
                }
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
}
