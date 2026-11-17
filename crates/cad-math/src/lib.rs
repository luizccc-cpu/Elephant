//! Double-precision math, geometric tolerances, rays, planes, and bounding boxes for Elephant.

pub use glam::{DMat4, DQuat, DVec2, DVec3, DVec4, Mat4, Vec2, Vec3, Vec4};
use serde::{Deserialize, Serialize};

/// Default absolute model-space tolerance (similar to standard CAD millimeter precision).
pub const DEFAULT_ABSOLUTE_TOLERANCE: f64 = 1.0e-3;
/// Default relative tolerance.
pub const DEFAULT_RELATIVE_TOLERANCE: f64 = 1.0e-6;
/// Default angle tolerance in radians (~0.1 degrees).
pub const DEFAULT_ANGLE_TOLERANCE_RAD: f64 = 0.0017453292519943296;
/// Tight numerical epsilon for knot/parameter comparisons.
pub const PARAM_EPSILON: f64 = 1.0e-12;

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
}
