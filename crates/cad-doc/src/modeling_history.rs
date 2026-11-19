use cad_geom::{
    BRep, Continuity, DMat4, DVec3, Geometry, LoftStyle, NurbsCurve, NurbsSurface,
    OffsetCornerStyle, Plane, blend_curves, edge_srf, loft_nurbs, offset_srf, revolve_nurbs, sweep1,
    sweep2,
};
use serde::{Deserialize, Serialize};

use crate::object::ObjectId;

/// Parametric operation linking parent objects to a generated child object (non-blocking history).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HistoryOp {
    Extrude {
        direction: DVec3,
        solid: bool,
    },
    Loft {
        style: LoftStyle,
        closed: bool,
    },
    Revolve {
        axis_origin: DVec3,
        axis_dir: DVec3,
        angle_rad: f64,
    },
    Sweep1,
    Sweep2,
    EdgeSrf,
    PlanarSrf,
    BlendCrv {
        continuity: Continuity,
        bulge: f64,
    },
    OffsetCurve {
        distance: f64,
        plane: Plane,
        side_pt: DVec3,
        corner: OffsetCornerStyle,
    },
    OffsetSrf {
        distance: f64,
        solid: bool,
    },
    Mirror {
        plane_origin: DVec3,
        plane_normal: DVec3,
    },
    ArrayLinear {
        index: usize,
        step: DVec3,
    },
    ArrayPolar {
        index: usize,
        total: usize,
        center: DVec3,
        axis: DVec3,
    },
}

/// Parent-to-child dependency record inside a `Document`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HistoryRecord {
    pub child_id: ObjectId,
    pub parent_ids: Vec<ObjectId>,
    pub op: HistoryOp,
}

impl HistoryOp {
    pub fn label(&self) -> &'static str {
        match self {
            Self::Extrude { .. } => "Extrude",
            Self::Loft { .. } => "Loft",
            Self::Revolve { .. } => "Revolve",
            Self::Sweep1 => "Sweep1",
            Self::Sweep2 => "Sweep2",
            Self::EdgeSrf => "EdgeSrf",
            Self::PlanarSrf => "PlanarSrf",
            Self::BlendCrv { .. } => "BlendCrv",
            Self::OffsetCurve { .. } => "Offset",
            Self::OffsetSrf { .. } => "OffsetSrf",
            Self::Mirror { .. } => "Mirror",
            Self::ArrayLinear { .. } => "ArrayLinear",
            Self::ArrayPolar { .. } => "ArrayPolar",
        }
    }

    pub fn evaluate(&self, parents: &[&Geometry], _tol: f64) -> Option<Geometry> {
        match *self {
            Self::Extrude { direction, solid } => {
                let Geometry::Curve(curve) = parents.first()? else {
                    return None;
                };
                if solid && curve.is_closed() {
                    if let Ok(brep) = BRep::extrude_curve_solid(curve, direction) {
                        return Some(Geometry::BRep(brep));
                    }
                }
                Some(Geometry::Surface(NurbsSurface::extrude_curve(
                    curve, direction,
                )))
            }
            Self::Loft { style, closed } => {
                let curves: Vec<NurbsCurve> = parents
                    .iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                (curves.len() >= 2)
                    .then(|| loft_nurbs(&curves, style, closed).ok())
                    .flatten()
                    .map(Geometry::Surface)
            }
            Self::Revolve {
                axis_origin,
                axis_dir,
                angle_rad,
            } => {
                let Geometry::Curve(profile) = parents.first()? else {
                    return None;
                };
                revolve_nurbs(profile, axis_origin, axis_dir, angle_rad)
                    .ok()
                    .map(Geometry::Surface)
            }
            Self::Sweep1 => {
                if parents.len() < 2 {
                    return None;
                }
                let Geometry::Curve(rail) = parents[0] else {
                    return None;
                };
                let profiles: Vec<NurbsCurve> = parents[1..]
                    .iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                (!profiles.is_empty())
                    .then(|| sweep1(rail, &profiles, None).ok())
                    .flatten()
                    .map(Geometry::Surface)
            }
            Self::Sweep2 => {
                if parents.len() < 3 {
                    return None;
                }
                let (Geometry::Curve(r1), Geometry::Curve(r2)) = (parents[0], parents[1]) else {
                    return None;
                };
                let profiles: Vec<NurbsCurve> = parents[2..]
                    .iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                (!profiles.is_empty())
                    .then(|| sweep2(r1, r2, &profiles).ok())
                    .flatten()
                    .map(Geometry::Surface)
            }
            Self::EdgeSrf => {
                let curves: Vec<NurbsCurve> = parents
                    .iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                (curves.len() >= 2 && curves.len() <= 4)
                    .then(|| edge_srf(&curves).ok())
                    .flatten()
                    .map(Geometry::Surface)
            }
            Self::PlanarSrf => {
                let curves: Vec<NurbsCurve> = parents
                    .iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) if c.is_closed() => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                (!curves.is_empty())
                    .then(|| BRep::from_planar_curves(&curves).ok())
                    .flatten()
                    .map(Geometry::BRep)
            }
            Self::BlendCrv { continuity, bulge } => {
                if parents.len() < 2 {
                    return None;
                }
                let (Geometry::Curve(ca), Geometry::Curve(cb)) = (parents[0], parents[1]) else {
                    return None;
                };
                let (at_end_a, at_end_b) = closest_endpoint_pair(ca, cb);
                blend_curves(ca, at_end_a, cb, at_end_b, continuity, bulge, bulge)
                    .ok()
                    .map(Geometry::Curve)
            }
            Self::OffsetCurve {
                distance,
                plane,
                side_pt,
                corner,
            } => {
                let Geometry::Curve(c) = parents.first()? else {
                    return None;
                };
                c.offset_nurbs_in_plane(&plane, distance, side_pt, corner)
                    .ok()
                    .map(Geometry::Curve)
            }
            Self::OffsetSrf { distance, solid: _ } => {
                let Geometry::Surface(s) = parents.first()? else {
                    return None;
                };
                offset_srf(s, distance).ok().map(Geometry::Surface)
            }
            Self::Mirror {
                plane_origin,
                plane_normal,
            } => {
                let mut g = (*parents.first()?).clone();
                let mat = mirror_matrix(plane_origin, plane_normal);
                g.transform(&mat);
                Some(g)
            }
            Self::ArrayLinear { index, step } => {
                let mut g = (*parents.first()?).clone();
                let mat = DMat4::from_translation(step * index as f64);
                g.transform(&mat);
                Some(g)
            }
            Self::ArrayPolar {
                index,
                total,
                center,
                axis,
            } => {
                if total == 0 {
                    return None;
                }
                let mut g = (*parents.first()?).clone();
                let angle = std::f64::consts::TAU * (index as f64 / total as f64);
                let ax = axis.normalize_or(DVec3::Z);
                let mat = DMat4::from_translation(center)
                    * DMat4::from_axis_angle(ax, angle)
                    * DMat4::from_translation(-center);
                g.transform(&mat);
                Some(g)
            }
        }
    }
}

fn mirror_matrix(origin: DVec3, normal: DVec3) -> DMat4 {
    let n = normal.normalize_or(DVec3::X);
    let reflect = DMat4::from_cols_array(&[
        1.0 - 2.0 * n.x * n.x, -2.0 * n.x * n.y, -2.0 * n.x * n.z, 0.0,
        -2.0 * n.y * n.x, 1.0 - 2.0 * n.y * n.y, -2.0 * n.y * n.z, 0.0,
        -2.0 * n.z * n.x, -2.0 * n.z * n.y, 1.0 - 2.0 * n.z * n.z, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]);
    DMat4::from_translation(origin) * reflect * DMat4::from_translation(-origin)
}

fn closest_endpoint_pair(a: &NurbsCurve, b: &NurbsCurve) -> (bool, bool) {
    let (ua0, ua1) = a.domain();
    let (ub0, ub1) = b.domain();
    let ea = [a.evaluate(ua0), a.evaluate(ua1)];
    let eb = [b.evaluate(ub0), b.evaluate(ub1)];
    let mut best = (true, false);
    let mut best_d = f64::INFINITY;
    for (ia, &pa) in ea.iter().enumerate() {
        for (ib, &pb) in eb.iter().enumerate() {
            let d = pa.distance(pb);
            if d < best_d {
                best_d = d;
                best = (ia == 1, ib == 1);
            }
        }
    }
    best
}
