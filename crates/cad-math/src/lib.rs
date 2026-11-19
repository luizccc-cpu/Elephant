//! Double-precision math, geometric tolerances, rays, planes, and bounding boxes for Elephant.

pub mod linalg;

pub use glam::{DMat4, DQuat, DVec2, DVec3, DVec4, Mat4, Vec2, Vec3, Vec4};
use serde::{Deserialize, Serialize};

/// Geometric continuity order for curves and surfaces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Continuity {
    G0,
    G1,
    G2,
    G3,
}

impl Continuity {
    pub fn label(self) -> &'static str {
        match self {
            Self::G0 => "G0 (Position)",
            Self::G1 => "G1 (Tangent)",
            Self::G2 => "G2 (Curvature)",
            Self::G3 => "G3 (Acceleration)",
        }
    }
}

/// Default absolute model-space tolerance (similar to standard CAD millimeter precision).
pub const DEFAULT_ABSOLUTE_TOLERANCE: f64 = 1.0e-3;
/// Default relative tolerance.
pub const DEFAULT_RELATIVE_TOLERANCE: f64 = 1.0e-6;
/// Default angle tolerance in radians (~0.1 degrees).
pub const DEFAULT_ANGLE_TOLERANCE_RAD: f64 = 0.0017453292519943296;
/// Tight numerical epsilon for knot/parameter comparisons.
pub const PARAM_EPSILON: f64 = 1.0e-12;

/// Physical model unit system for a document.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum UnitSystem {
    Millimeters,
    Centimeters,
    Meters,
    Inches,
    Feet,
}

impl Default for UnitSystem {
    fn default() -> Self {
        Self::Millimeters
    }
}

impl UnitSystem {
    pub const ALL: [Self; 5] = [
        Self::Millimeters,
        Self::Centimeters,
        Self::Meters,
        Self::Inches,
        Self::Feet,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Millimeters => "mm",
            Self::Centimeters => "cm",
            Self::Meters => "m",
            Self::Inches => "in",
            Self::Feet => "ft",
        }
    }

    pub fn full_name(self) -> &'static str {
        match self {
            Self::Millimeters => "Millimeters",
            Self::Centimeters => "Centimeters",
            Self::Meters => "Meters",
            Self::Inches => "Inches",
            Self::Feet => "Feet",
        }
    }

    pub fn meters_per_unit(self) -> f64 {
        match self {
            Self::Millimeters => 0.001,
            Self::Centimeters => 0.01,
            Self::Meters => 1.0,
            Self::Inches => 0.0254,
            Self::Feet => 0.3048,
        }
    }

    /// Scale factor to convert coordinates expressed in `self` into `target`.
    pub fn scale_to(self, target: Self) -> f64 {
        self.meters_per_unit() / target.meters_per_unit()
    }

    pub fn default_tolerances(self) -> Tolerances {
        let absolute = match self {
            Self::Millimeters => 1.0e-3,
            Self::Centimeters => 1.0e-4,
            Self::Meters => 1.0e-5,
            Self::Inches => 1.0e-4,
            Self::Feet => 1.0e-5,
        };
        Tolerances {
            absolute,
            relative: DEFAULT_RELATIVE_TOLERANCE,
            angle_rad: DEFAULT_ANGLE_TOLERANCE_RAD,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Tolerances {
    pub absolute: f64,
    pub relative: f64,
    pub angle_rad: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            absolute: DEFAULT_ABSOLUTE_TOLERANCE,
            relative: DEFAULT_RELATIVE_TOLERANCE,
            angle_rad: DEFAULT_ANGLE_TOLERANCE_RAD,
        }
    }
}

/// Axis-aligned bounding box in 3D double-precision space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb {
    pub const EMPTY: Self = Self {
        min: DVec3::splat(f64::INFINITY),
        max: DVec3::splat(f64::NEG_INFINITY),
    };

    pub fn from_points<I>(points: I) -> Self
    where
        I: IntoIterator<Item = DVec3>,
    {
        let mut box_ = Self::EMPTY;
        for p in points {
            box_.expand_point(p);
        }
        box_
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    pub fn expand_point(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn size(&self) -> DVec3 {
        if self.is_empty() {
            DVec3::ZERO
        } else {
            self.max - self.min
        }
    }

    /// Return this AABB padded outward on all sides by `margin`.
    pub fn expanded(&self, margin: f64) -> Self {
        if self.is_empty() {
            Self::EMPTY
        } else {
            let pad = DVec3::splat(margin);
            Self {
                min: self.min - pad,
                max: self.max + pad,
            }
        }
    }

    /// True if two non-empty axis-aligned bounding boxes overlap or touch.
    pub fn intersects(&self, other: &Self) -> bool {
        !self.is_empty()
            && !other.is_empty()
            && self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

/// 3D Ray for viewport picking and intersection tests.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Ray {
    pub origin: DVec3,
    pub direction: DVec3,
}

impl Ray {
    pub fn new(origin: DVec3, direction: DVec3) -> Self {
        Self {
            origin,
            direction: direction.normalize_or_zero(),
        }
    }

    pub fn point_at(&self, t: f64) -> DVec3 {
        self.origin + self.direction * t
    }

    /// Closest point on the ray (`t >= 0`) to `point`, returning `(t, distance)`.
    pub fn distance_to_point(&self, point: DVec3) -> (f64, f64) {
        let len_sq = self.direction.length_squared();
        let t = if len_sq < PARAM_EPSILON {
            0.0
        } else {
            ((point - self.origin).dot(self.direction) / len_sq).max(0.0)
        };
        (t, self.point_at(t).distance(point))
    }

    /// Minimum distance between the ray (`t >= 0`) and line segment `[a, b]`, returning `(t, distance)`.
    pub fn distance_to_segment(&self, a: DVec3, b: DVec3) -> (f64, f64) {
        let u = self.direction;
        let v = b - a;
        let w0 = self.origin - a;
        let a_dot = u.dot(u);
        let b_dot = u.dot(v);
        let c_dot = v.dot(v);
        let d_dot = u.dot(w0);
        let e_dot = v.dot(w0);

        if c_dot < PARAM_EPSILON {
            return self.distance_to_point(a);
        }
        if a_dot < PARAM_EPSILON {
            let s0 = (e_dot / c_dot).clamp(0.0, 1.0);
            return (0.0, self.origin.distance(a + v * s0));
        }

        let denom = a_dot * c_dot - b_dot * b_dot;
        if denom.abs() < PARAM_EPSILON {
            let cand_a = self.distance_to_point(a);
            let cand_b = self.distance_to_point(b);
            let s0 = (e_dot / c_dot).clamp(0.0, 1.0);
            let cand_0 = (0.0, self.origin.distance(a + v * s0));
            return [cand_a, cand_b, cand_0]
                .into_iter()
                .min_by(|x, y| x.1.total_cmp(&y.1).then_with(|| x.0.total_cmp(&y.0)))
                .unwrap();
        }

        let t_raw = (b_dot * e_dot - c_dot * d_dot) / denom;
        let s_raw = (a_dot * e_dot - b_dot * d_dot) / denom;

        if t_raw >= 0.0 && (0.0..=1.0).contains(&s_raw) {
            return (t_raw, self.point_at(t_raw).distance(a + v * s_raw));
        }

        let mut best: Option<(f64, f64)> = None;
        let mut consider = |t: f64, s: f64| {
            let dist = self.point_at(t).distance(a + v * s);
            if best.is_none_or(|b_cur| dist < b_cur.1 || (dist == b_cur.1 && t < b_cur.0)) {
                best = Some((t, dist));
            }
        };

        if s_raw < 0.0 {
            consider((-d_dot / a_dot).max(0.0), 0.0);
        } else if s_raw > 1.0 {
            consider(((b_dot - d_dot) / a_dot).max(0.0), 1.0);
        }
        if t_raw < 0.0 {
            consider(0.0, (e_dot / c_dot).clamp(0.0, 1.0));
        }

        best.unwrap_or_else(|| self.distance_to_point(a))
    }

    /// Slab intersection against an AABB; returns `Some((t_min, t_max))` if hit.
    pub fn intersect_aabb(&self, aabb: &Aabb) -> Option<(f64, f64)> {
        if aabb.is_empty() {
            return None;
        }
        let mut tmin = f64::NEG_INFINITY;
        let mut tmax = f64::INFINITY;

        for axis in 0..3 {
            let o = self.origin[axis];
            let d = self.direction[axis];
            let bmin = aabb.min[axis];
            let bmax = aabb.max[axis];

            if d.abs() < PARAM_EPSILON {
                if o < bmin || o > bmax {
                    return None;
                }
            } else {
                let inv_d = 1.0 / d;
                let mut t1 = (bmin - o) * inv_d;
                let mut t2 = (bmax - o) * inv_d;
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                tmin = tmin.max(t1);
                tmax = tmax.min(t2);
                if tmin > tmax {
                    return None;
                }
            }
        }

        if tmax < 0.0 {
            None
        } else {
            Some((tmin.max(0.0), tmax))
        }
    }
}

/// Construction plane (CPlane) in 3D space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Plane {
    pub origin: DVec3,
    pub x_axis: DVec3,
    pub y_axis: DVec3,
    pub z_axis: DVec3,
}

impl Plane {
    pub fn world_xy() -> Self {
        Self {
            origin: DVec3::ZERO,
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        }
    }

    pub fn world_zx() -> Self {
        Self {
            origin: DVec3::ZERO,
            x_axis: DVec3::X,
            y_axis: DVec3::Z,
            z_axis: -DVec3::Y,
        }
    }

    pub fn world_yz() -> Self {
        Self {
            origin: DVec3::ZERO,
            x_axis: DVec3::Y,
            y_axis: DVec3::Z,
            z_axis: DVec3::X,
        }
    }

    /// Intersect a ray with this plane; returns world-space hit point if not parallel.
    pub fn intersect_ray(&self, ray: &Ray) -> Option<DVec3> {
        let denom = ray.direction.dot(self.z_axis);
        if denom.abs() < PARAM_EPSILON {
            return None;
        }
        let t = (self.origin - ray.origin).dot(self.z_axis) / denom;
        if t < 0.0 {
            None
        } else {
            Some(ray.point_at(t))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_and_ray_hit() {
        let aabb = Aabb::from_points([DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0)]);
        let ray = Ray::new(DVec3::new(0.0, 0.0, 5.0), DVec3::new(0.0, 0.0, -1.0));
        let hit = ray.intersect_aabb(&aabb).expect("ray should hit box");
        assert!((hit.0 - 4.0).abs() < 1e-9);
        assert!((hit.1 - 6.0).abs() < 1e-9);
    }

    #[test]
    fn plane_ray_intersection() {
        let plane = Plane::world_xy();
        let ray = Ray::new(DVec3::new(3.0, 4.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        let hit = plane.intersect_ray(&ray).expect("ray should hit XY plane");
        assert!((hit - DVec3::new(3.0, 4.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn unit_system_scaling_and_tolerances() {
        assert!((UnitSystem::Millimeters.scale_to(UnitSystem::Centimeters) - 0.1).abs() < 1e-12);
        assert!((UnitSystem::Meters.scale_to(UnitSystem::Millimeters) - 1000.0).abs() < 1e-9);
        assert!((UnitSystem::Inches.scale_to(UnitSystem::Millimeters) - 25.4).abs() < 1e-9);
        assert!((UnitSystem::Feet.scale_to(UnitSystem::Inches) - 12.0).abs() < 1e-9);
        assert!((UnitSystem::Millimeters.default_tolerances().absolute - 1e-3).abs() < 1e-12);
        assert!((UnitSystem::Meters.default_tolerances().absolute - 1e-5).abs() < 1e-12);
    }

    #[test]
    fn aabb_expanded_and_ray_distances() {
        let bb = Aabb::from_points([DVec3::ZERO, DVec3::new(1.0, 2.0, 3.0)]).expanded(0.5);
        assert!((bb.min - DVec3::splat(-0.5)).length() < 1e-9);
        assert!((bb.max - DVec3::new(1.5, 2.5, 3.5)).length() < 1e-9);
        assert!(Aabb::EMPTY.expanded(1.0).is_empty());

        let ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        let (tp, dp) = ray.distance_to_point(DVec3::new(3.0, 4.0, 2.0));
        assert!((tp - 8.0).abs() < 1e-9);
        assert!((dp - 5.0).abs() < 1e-9);

        // Crossing segment at z=0
        let (t1, d1) = ray.distance_to_segment(DVec3::new(-2.0, 0.5, 0.0), DVec3::new(2.0, 0.5, 0.0));
        assert!((t1 - 10.0).abs() < 1e-9);
        assert!((d1 - 0.5).abs() < 1e-9);

        // Clamped to segment endpoint
        let (t2, d2) = ray.distance_to_segment(DVec3::new(3.0, 0.0, 1.0), DVec3::new(8.0, 0.0, 1.0));
        assert!((t2 - 9.0).abs() < 1e-9);
        assert!((d2 - 3.0).abs() < 1e-9);

        // Parallel to ray
        let (t3, d3) = ray.distance_to_segment(DVec3::new(2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 5.0));
        assert!((d3 - 2.0).abs() < 1e-9);
        assert!(t3 >= 5.0 - 1e-9 && t3 <= 10.0 + 1e-9);

        // Segment entirely behind ray origin
        let (t4, d4) = ray.distance_to_segment(DVec3::new(-1.0, 0.0, 15.0), DVec3::new(1.0, 0.0, 15.0));
        assert!(t4.abs() < 1e-9);
        assert!((d4 - 5.0).abs() < 1e-9);
    }
}
