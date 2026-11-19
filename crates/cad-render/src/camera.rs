use cad_math::{Aabb, DMat4, DVec3, DVec4, Mat4, Plane, Ray};
use serde::{Deserialize, Serialize};

use crate::scene::DisplayMode;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViewportKind {
    Perspective,
    Top,
    Front,
    Right,
}

impl ViewportKind {
    pub fn title(self) -> &'static str {
        match self {
            Self::Perspective => "Perspective",
            Self::Top => "Top",
            Self::Front => "Front",
            Self::Right => "Right",
        }
    }
}

/// Z-up CAD viewport camera supporting Perspective and Orthographic projections.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ViewportCamera {
    pub kind: ViewportKind,
    pub target: DVec3,
    pub yaw: f64,
    pub pitch: f64,
    pub distance: f64,
    pub ortho_half_height: f64,
    pub is_perspective: bool,
    pub fov_y_rad: f64,
    pub cplane: Plane,
    pub display_mode: DisplayMode,
}

impl ViewportCamera {
    pub fn preset(kind: ViewportKind) -> Self {
        match kind {
            ViewportKind::Perspective => Self {
                kind,
                target: DVec3::ZERO,
                yaw: -std::f64::consts::FRAC_PI_4,
                pitch: 0.55,
                distance: 18.0,
                ortho_half_height: 10.0,
                is_perspective: true,
                fov_y_rad: 45.0_f64.to_radians(),
                cplane: Plane::world_xy(),
                display_mode: DisplayMode::Shaded,
            },
            ViewportKind::Top => Self {
                kind,
                target: DVec3::ZERO,
                yaw: 0.0,
                pitch: 0.0,
                distance: 50.0,
                ortho_half_height: 12.0,
                is_perspective: false,
                fov_y_rad: 45.0_f64.to_radians(),
                cplane: Plane::world_xy(),
                display_mode: DisplayMode::Wireframe,
            },
            ViewportKind::Front => Self {
                kind,
                target: DVec3::ZERO,
                yaw: 0.0,
                pitch: 0.0,
                distance: 50.0,
                ortho_half_height: 12.0,
                is_perspective: false,
                fov_y_rad: 45.0_f64.to_radians(),
                cplane: Plane::world_zx(),
                display_mode: DisplayMode::Wireframe,
            },
            ViewportKind::Right => Self {
                kind,
                target: DVec3::ZERO,
                yaw: 0.0,
                pitch: 0.0,
                distance: 50.0,
                ortho_half_height: 12.0,
                is_perspective: false,
                fov_y_rad: 45.0_f64.to_radians(),
                cplane: Plane::world_yz(),
                display_mode: DisplayMode::Wireframe,
            },
        }
    }

    pub fn eye_and_up(&self) -> (DVec3, DVec3) {
        match self.kind {
            ViewportKind::Perspective => {
                let cp = self.pitch.cos();
                let dir = DVec3::new(cp * self.yaw.cos(), cp * self.yaw.sin(), self.pitch.sin());
                (self.target + dir * self.distance, DVec3::Z)
            }
            ViewportKind::Top => (self.target + DVec3::Z * self.distance, DVec3::Y),
            ViewportKind::Front => (self.target - DVec3::Y * self.distance, DVec3::Z),
            ViewportKind::Right => (self.target + DVec3::X * self.distance, DVec3::Z),
        }
    }

    pub fn view_matrix(&self) -> DMat4 {
        let (eye, up) = self.eye_and_up();
        DMat4::look_at_rh(eye, self.target, up)
    }

    pub fn proj_matrix(&self, aspect: f64) -> DMat4 {
        let aspect = aspect.max(0.05);
        if self.is_perspective {
            let near = (self.distance * 0.001).max(0.05);
            let far = (self.distance * 25.0).max(50_000.0);
            DMat4::perspective_rh(self.fov_y_rad, aspect, near, far)
        } else {
            let hh = self.ortho_half_height.max(0.1);
            let hw = hh * aspect;
            let z_extent = (hh * 25.0).max(50_000.0);
            DMat4::orthographic_rh(-hw, hw, -hh, hh, -z_extent, z_extent)
        }
    }

    pub fn view_proj_f32(&self, aspect: f64) -> Mat4 {
        (self.proj_matrix(aspect) * self.view_matrix()).as_mat4()
    }

    /// Convert normalized device coordinates `[-1, 1]` into a world-space picking `Ray`.
    pub fn screen_to_ray(&self, ndc_x: f64, ndc_y: f64, aspect: f64) -> Ray {
        let inv = (self.proj_matrix(aspect) * self.view_matrix()).inverse();
        let p_near = inv * DVec4::new(ndc_x, ndc_y, 0.0, 1.0);
        let p_far = inv * DVec4::new(ndc_x, ndc_y, 1.0, 1.0);
        let near = p_near.truncate() / p_near.w;
        let far = p_far.truncate() / p_far.w;
        Ray::new(near, far - near)
    }

    pub fn orbit(&mut self, delta_x: f64, delta_y: f64) {
        if !self.is_perspective {
            return;
        }
        self.yaw -= delta_x * 0.008;
        self.pitch = (self.pitch + delta_y * 0.008).clamp(-1.54, 1.54);
    }

    pub fn world_units_per_pixel(&self, viewport_height_px: f64) -> f64 {
        let h = viewport_height_px.max(1.0);
        if self.is_perspective {
            2.0 * self.distance * (self.fov_y_rad * 0.5).tan() / h
        } else {
            2.0 * self.ortho_half_height / h
        }
    }

    pub fn pan(&mut self, delta_x: f64, delta_y: f64, viewport_height_px: f64) {
        let world_per_px = self.world_units_per_pixel(viewport_height_px);
        let (eye, up) = self.eye_and_up();
        let forward = (self.target - eye).normalize_or_zero();
        let right = forward.cross(up).normalize_or_zero();
        let cam_up = right.cross(forward).normalize_or_zero();
        self.target += (-right * delta_x + cam_up * delta_y) * world_per_px;
    }

    pub fn zoom(&mut self, scroll_delta: f64) {
        let factor = (1.0 - scroll_delta * 0.0015).clamp(0.5, 2.0);
        if self.is_perspective {
            self.distance = (self.distance * factor).clamp(0.1, 10_000_000.0);
        } else {
            self.ortho_half_height = (self.ortho_half_height * factor).clamp(0.1, 10_000_000.0);
        }
    }

    pub fn zoom_extents(&mut self, aabb: &Aabb) {
        if aabb.is_empty() {
            self.target = DVec3::ZERO;
            self.distance = 18.0;
            self.ortho_half_height = 10.0;
            return;
        }
        self.target = aabb.center();
        let radius = (aabb.size().length() * 0.5).max(1.0);
        self.distance = (radius * 2.2).clamp(1.0, 10_000_000.0);
        self.ortho_half_height = (radius * 1.15).clamp(0.5, 10_000_000.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn top_camera_ray_points_down_z() {
        let cam = ViewportCamera::preset(ViewportKind::Top);
        let ray = cam.screen_to_ray(0.0, 0.0, 1.5);
        assert!(ray.direction.z < -0.99);
    }
}
