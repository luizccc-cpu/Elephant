use bytemuck::{Pod, Zeroable};
use cad_doc::Document;
use cad_geom::Geometry;
use cad_math::{DVec3, Plane};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayMode {
    Wireframe,
    Shaded,
    Ghosted,
}

impl DisplayMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Wireframe => "Wireframe",
            Self::Shaded => "Shaded",
            Self::Ghosted => "Ghosted",
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct LineVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

#[derive(Debug, Default, Clone)]
pub struct RenderScene {
    pub version: u64,
    pub mesh_vertices: Vec<MeshVertex>,
    pub mesh_indices: Vec<u32>,
    pub line_vertices: Vec<LineVertex>,
}

const SELECTION_COLOR: [f32; 4] = [1.0, 0.85, 0.12, 1.0];

impl RenderScene {
    pub fn build(doc: &Document, mode: DisplayMode, cplane: &Plane, version: u64) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };
        let mut total_aabb = cad_math::Aabb::EMPTY;
        for (_, obj) in &doc.objects {
            if let Some(layer) = doc.layers.get(obj.layer_id) {
                if layer.visible {
                    total_aabb = total_aabb.union(&obj.aabb);
                }
            }
        }
        let grid_step = if total_aabb.is_empty() {
            1.0
        } else {
            let max_dim = total_aabb.size().max_element().max(1.0);
            10.0_f64.powf((max_dim / 30.0).log10().floor()).max(1.0)
        };
        scene.push_cplane_grid(cplane, 20, grid_step);

        for (_, obj) in &doc.objects {
            let Some(layer) = doc.layers.get(obj.layer_id) else {
                continue;
            };
            if !layer.visible {
                continue;
            }

            let base_color = if obj.selected {
                SELECTION_COLOR
            } else {
                [
                    layer.color[0] as f32 / 255.0,
                    layer.color[1] as f32 / 255.0,
                    layer.color[2] as f32 / 255.0,
                    1.0,
                ]
            };

            let shaded_color = if obj.selected {
                [1.0, 0.88, 0.25, if mode == DisplayMode::Ghosted { 0.45 } else { 1.0 }]
            } else {
                [0.72, 0.76, 0.82, if mode == DisplayMode::Ghosted { 0.45 } else { 1.0 }]
            };

            match &obj.geometry {
                Geometry::Point(p) => {
                    let pf = [p.x as f32, p.y as f32, p.z as f32];
                    let d = 0.15_f32;
                    scene.push_line([pf[0] - d, pf[1], pf[2]], [pf[0] + d, pf[1], pf[2]], base_color);
                    scene.push_line([pf[0], pf[1] - d, pf[2]], [pf[0], pf[1] + d, pf[2]], base_color);
                    scene.push_line([pf[0], pf[1], pf[2] - d], [pf[0], pf[1], pf[2] + d], base_color);
                }
                Geometry::Curve(curve) => {
                    if curve.degree == 1 {
                        for pair in curve.control_points.windows(2) {
                            let a = pair[0].position;
                            let b = pair[1].position;
                            scene.push_line(
                                [a.x as f32, a.y as f32, a.z as f32],
                                [b.x as f32, b.y as f32, b.z as f32],
                                base_color,
                            );
                        }
                    } else {
                        let pts = curve.sample_polyline(32);
                        for pair in pts.windows(2) {
                            scene.push_line(
                                [pair[0].x as f32, pair[0].y as f32, pair[0].z as f32],
                                [pair[1].x as f32, pair[1].y as f32, pair[1].z as f32],
                                base_color,
                            );
                        }
                    }
                }
                Geometry::Surface(surf) => {
                    let mesh = surf.tessellate(24, 8);
                    scene.push_mesh(&mesh.positions, &mesh.normals, &mesh.indices, shaded_color, base_color, mode, obj.selected);
                }
                Geometry::Mesh(mesh) => {
                    scene.push_mesh(&mesh.positions, &mesh.normals, &mesh.indices, shaded_color, base_color, mode, obj.selected);
                }
            }
        }

        scene
    }

    /// Append live rubber-band / parametric preview geometry and optional OSnap crosshair marker.
    pub fn push_preview(&mut self, previews: &[Geometry], snap_point: Option<DVec3>) {
        let preview_color = [0.25, 0.85, 1.0, 1.0];
        let shaded_preview = [0.20, 0.72, 0.90, 0.45];
        for geom in previews {
            match geom {
                Geometry::Curve(curve) => {
                    let pts = curve.sample_polyline(48);
                    for pair in pts.windows(2) {
                        self.push_line(
                            [pair[0].x as f32, pair[0].y as f32, pair[0].z as f32],
                            [pair[1].x as f32, pair[1].y as f32, pair[1].z as f32],
                            preview_color,
                        );
                    }
                }
                Geometry::Surface(surf) => {
                    let mesh = surf.tessellate(24, 8);
                    self.push_mesh(
                        &mesh.positions,
                        &mesh.normals,
                        &mesh.indices,
                        shaded_preview,
                        preview_color,
                        DisplayMode::Ghosted,
                        false,
                    );
                }
                Geometry::Mesh(mesh) => {
                    self.push_mesh(
                        &mesh.positions,
                        &mesh.normals,
                        &mesh.indices,
                        shaded_preview,
                        preview_color,
                        DisplayMode::Ghosted,
                        false,
                    );
                }
                Geometry::Point(p) => {
                    let pf = [p.x as f32, p.y as f32, p.z as f32];
                    let d = 0.2_f32;
                    self.push_line([pf[0] - d, pf[1], pf[2]], [pf[0] + d, pf[1], pf[2]], preview_color);
                    self.push_line([pf[0], pf[1] - d, pf[2]], [pf[0], pf[1] + d, pf[2]], preview_color);
                }
            }
        }
        if let Some(p) = snap_point {
            let snap_color = [1.0, 0.35, 0.95, 1.0];
            let pf = [p.x as f32, p.y as f32, p.z as f32];
            let d = 0.25_f32;
            self.push_line([pf[0] - d, pf[1], pf[2]], [pf[0] + d, pf[1], pf[2]], snap_color);
            self.push_line([pf[0], pf[1] - d, pf[2]], [pf[0], pf[1] + d, pf[2]], snap_color);
            self.push_line([pf[0], pf[1], pf[2] - d], [pf[0], pf[1], pf[2] + d], snap_color);
        }
    }

    fn push_mesh(
        &mut self,
        positions: &[[f64; 3]],
        normals: &[[f64; 3]],
        indices: &[[u32; 3]],
        shaded_color: [f32; 4],
        wire_color: [f32; 4],
        mode: DisplayMode,
        selected: bool,
    ) {
        if mode != DisplayMode::Wireframe {
            let base = self.mesh_vertices.len() as u32;
            for (p, n) in positions.iter().zip(normals.iter()) {
                self.mesh_vertices.push(MeshVertex {
                    position: [p[0] as f32, p[1] as f32, p[2] as f32],
                    normal: [n[0] as f32, n[1] as f32, n[2] as f32],
                    color: shaded_color,
                });
            }
            for &[a, b, c] in indices {
                self.mesh_indices.push(base + a);
                self.mesh_indices.push(base + b);
                self.mesh_indices.push(base + c);
            }
        }

        if mode == DisplayMode::Wireframe || mode == DisplayMode::Ghosted || selected {
            let edge_color = if selected { SELECTION_COLOR } else { wire_color };
            for &[a, b, c] in indices {
                let pa = positions[a as usize];
                let pb = positions[b as usize];
                let pc = positions[c as usize];
                let fa = [pa[0] as f32, pa[1] as f32, pa[2] as f32];
                let fb = [pb[0] as f32, pb[1] as f32, pb[2] as f32];
                let fc = [pc[0] as f32, pc[1] as f32, pc[2] as f32];
                self.push_line(fa, fb, edge_color);
                self.push_line(fb, fc, edge_color);
                self.push_line(fc, fa, edge_color);
            }
        }
    }

    fn push_cplane_grid(&mut self, plane: &Plane, half_count: i32, step: f64) {
        let extent = half_count as f64 * step;
        let minor = [0.32, 0.34, 0.38, 1.0];
        let major = [0.22, 0.24, 0.28, 1.0];
        let x_axis_color = [0.85, 0.22, 0.22, 1.0];
        let y_axis_color = [0.22, 0.78, 0.30, 1.0];

        for i in -half_count..=half_count {
            let d = i as f64 * step;
            let is_major = i % 5 == 0;
            let c_x = if i == 0 { x_axis_color } else if is_major { major } else { minor };
            let c_y = if i == 0 { y_axis_color } else if is_major { major } else { minor };

            let p0 = plane.origin + plane.y_axis * d - plane.x_axis * extent;
            let p1 = plane.origin + plane.y_axis * d + plane.x_axis * extent;
            self.push_line(
                [p0.x as f32, p0.y as f32, p0.z as f32],
                [p1.x as f32, p1.y as f32, p1.z as f32],
                c_x,
            );

            let q0 = plane.origin + plane.x_axis * d - plane.y_axis * extent;
            let q1 = plane.origin + plane.x_axis * d + plane.y_axis * extent;
            self.push_line(
                [q0.x as f32, q0.y as f32, q0.z as f32],
                [q1.x as f32, q1.y as f32, q1.z as f32],
                c_y,
            );
        }
    }

    #[inline]
    fn push_line(&mut self, a: [f32; 3], b: [f32; 3], color: [f32; 4]) {
        self.line_vertices.push(LineVertex { position: a, color });
        self.line_vertices.push(LineVertex { position: b, color });
    }
}
