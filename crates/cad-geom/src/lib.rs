//! Clean-room NURBS curves/surfaces, meshes, and geometric primitives for Elephant.

pub mod error;
pub mod mesh;
pub mod nurbs_curve;
pub mod nurbs_surface;

pub use cad_math::{Aabb, DMat4, DVec3, Plane, Ray, Tolerances};
pub use error::GeomError;
pub use mesh::TriangleMesh;
pub use nurbs_curve::{ControlPoint, NurbsCurve};
pub use nurbs_surface::NurbsSurface;
use serde::{Deserialize, Serialize};

/// Unified geometric entity stored in an Elephant document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Geometry {
    Point(DVec3),
    Curve(NurbsCurve),
    Surface(NurbsSurface),
    Mesh(TriangleMesh),
}

impl Geometry {
    pub fn bounding_box(&self) -> Aabb {
        match self {
            Self::Point(p) => Aabb::from_points([*p]),
            Self::Curve(c) => c.bounding_box(),
            Self::Surface(s) => s.bounding_box(),
            Self::Mesh(m) => m.bounding_box(),
        }
    }

    pub fn transform(&mut self, mat: &DMat4) {
        match self {
            Self::Point(p) => *p = mat.transform_point3(*p),
            Self::Curve(c) => c.transform(mat),
            Self::Surface(s) => s.transform(mat),
            Self::Mesh(m) => m.transform(mat),
        }
    }

    /// Key snap points (endpoints, midpoint) for OSnap queries.
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
            Self::Mesh(m) => m.positions.iter().map(|&p| DVec3::from_array(p)).collect(),
        }
    }
}
