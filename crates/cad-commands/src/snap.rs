use cad_doc::Document;
use cad_geom::Geometry;
use cad_math::{DVec3, Plane};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapKind {
    End,
    Mid,
    Center,
    Grid,
    Ortho,
}

impl SnapKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::End => "End",
            Self::Mid => "Mid",
            Self::Center => "Cen",
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
}
