//! Clean-room NURBS curves/surfaces, meshes, B-Rep solids, and geometric primitives for Elephant.

pub mod analysis;
pub mod brep;
pub mod brep_ops;
pub mod csg;
pub mod curve_ops;
pub mod drafting;
pub mod error;
pub mod mesh;
pub mod nurbs_curve;
pub mod nurbs_surface;
pub mod surface_ops;

pub use analysis::{
    ContinuityGrade, ContinuityTolerances, CurvatureMesh, EdgeContinuitySample, MassProperties,
    SeamContinuityReport, analyze_brep_edge_continuity, analyze_surface_pair_continuity,
    combine_mass_properties, compute_geometry_mass_properties, compute_mesh_discrete_curvature,
    compute_mesh_mass_properties, section_geometry_with_plane, section_mesh_with_plane,
    tessellate_brep_with_curvature, tessellate_surface_with_curvature,
};
pub use brep::{BRep, BRepEdge, BRepFace, BRepVertex, SubObjectRef, TrimLoop2D};
pub use drafting::{
    DrawingProjection, Make2DLineKind, Make2DOptions, Make2DResult, Make2DSegment,
    make_2d_projection,
};
pub use brep_ops::{
    FilletSurfaceResult, SurfaceIntersectionCurve, brep_boolean, brep_boolean_split,
    cap_planar_holes, chamfer_brep_edges, fillet_brep_edges, fillet_srf, inset_face,
    intersect_breps, intersect_surfaces, project_curve_to_surface, pull_curve_to_surface,
    push_pull_face, shell_brep, split_face_with_uv_curve, tangent_edge_chain,
    trim_face_with_curve, untrim_brep,
};
pub use cad_math::{Aabb, Continuity, DMat4, DVec3, Plane, Ray, Tolerances, UnitSystem};
pub use csg::{BooleanOp, mesh_boolean};
pub use curve_ops::{
    CurveIntersection, OffsetCornerStyle, RebuildReport, blend_curves, chamfer_two_curves,
    fair_curve, fillet_two_curves, intersect_curves, match_curve, rebuild_adaptive,
    rebuild_uniform, trim_curve_with_cutters, tween_curves,
};
pub use error::GeomError;
pub use mesh::TriangleMesh;
pub use nurbs_curve::{ControlPoint, CurvatureSample, NurbsCurve};
pub use nurbs_surface::{
    NurbsSurface, SurfaceCurvature, SurfaceDerivatives, SurfaceEdge, SurfaceFrame,
};
pub use surface_ops::{
    BoundaryLock, LoftStyle, PatchBoundaryConstraint, PatchOptions, PatchReport,
    SoftTransformFalloff, SurfaceRebuildReport, apply_soft_transform_surface,
    apply_weighted_deltas_curve, apply_weighted_deltas_surface, blend_srf, closest_edge_pair,
    edge_srf, extend_srf, loft_nurbs, make_curves_compatible, match_srf, network_srf, offset_srf,
    partition_network_curves, patch_srf, rail_revolve_nurbs, rebuild_srf, revolve_nurbs,
    soft_transform_weights_curve, soft_transform_weights_surface, sweep1, sweep2,
};
use serde::{Deserialize, Serialize};

/// Unified geometric entity stored in an Elephant document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Geometry {
    Point(DVec3),
    Curve(NurbsCurve),
    Surface(NurbsSurface),
    BRep(BRep),
    Mesh(TriangleMesh),
}

impl Geometry {
    pub fn bounding_box(&self) -> Aabb {
        match self {
            Self::Point(p) => Aabb::from_points([*p]),
            Self::Curve(c) => c.bounding_box(),
            Self::Surface(s) => s.bounding_box(),
            Self::BRep(b) => b.bounding_box(),
            Self::Mesh(m) => m.bounding_box(),
        }
    }

    pub fn transform(&mut self, mat: &DMat4) {
        match self {
            Self::Point(p) => *p = mat.transform_point3(*p),
            Self::Curve(c) => c.transform(mat),
            Self::Surface(s) => s.transform(mat),
            Self::BRep(b) => b.transform(mat),
            Self::Mesh(m) => m.transform(mat),
        }
    }

    /// Key snap points (endpoints, midpoint, B-Rep vertices) for OSnap queries.
    pub fn snap_points(&self) -> Vec<DVec3> {
        match self {
            Self::Point(p) => vec![*p],
            Self::Curve(c) => {
                let (u0, u1) = c.domain();
                vec![c.evaluate(u0), c.evaluate((u0 + u1) * 0.5), c.evaluate(u1)]
            }
            Self::Surface(s) => {
                let (u0, u1) = s.domain_u();
                let (v0, v1) = s.domain_v();
                vec![
                    s.evaluate(u0, v0),
                    s.evaluate(u1, v0),
                    s.evaluate(u0, v1),
                    s.evaluate(u1, v1),
                ]
            }
            Self::BRep(b) => b.snap_points(),
            Self::Mesh(m) => m.positions.iter().map(|&p| DVec3::from_array(p)).collect(),
        }
    }
}
