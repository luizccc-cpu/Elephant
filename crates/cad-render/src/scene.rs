use std::collections::HashMap;

use bytemuck::{Pod, Zeroable};
use cad_doc::{AnnotationKind, Document};
use cad_geom::{Aabb, Geometry};
use cad_math::{DVec3, Plane};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DisplayMode {
    Wireframe,
    Shaded,
    Ghosted,
    Technical,
    Arctic,
}

impl DisplayMode {
    pub const ALL: [Self; 5] = [
        Self::Wireframe,
        Self::Shaded,
        Self::Ghosted,
        Self::Technical,
        Self::Arctic,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Wireframe => "Wireframe",
            Self::Shaded => "Shaded",
            Self::Ghosted => "Ghosted",
            Self::Technical => "Technical",
            Self::Arctic => "Arctic",
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
    /// Combined document + CPlane grid scene (useful for single-viewport rendering and tests).
    pub fn build(doc: &Document, mode: DisplayMode, cplane: &Plane, version: u64) -> Self {
        let doc_aabb = doc.visible_bounding_box();
        let mut scene = Self::build_grid(cplane, &doc_aabb, version);
        let doc_scene = Self::build_document(doc, mode, version);
        scene.append(&doc_scene);
        scene
    }

    /// Build only visible document geometry and annotations (without CPlane grid or transient previews).
    pub fn build_document(doc: &Document, mode: DisplayMode, version: u64) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };

        for (_, obj) in &doc.objects {
            if !obj.visible {
                continue;
            }
            let Some(layer) = doc.layers.get(obj.layer_id) else {
                continue;
            };
            if !layer.visible {
                continue;
            }

            let raw_rgb = obj.color.unwrap_or(layer.color);
            let is_pure_black = raw_rgb[0] == 0 && raw_rgb[1] == 0 && raw_rgb[2] == 0;
            let layer_rgba = if is_pure_black {
                [0.84, 0.86, 0.90, 1.0]
            } else {
                [
                    raw_rgb[0] as f32 / 255.0,
                    raw_rgb[1] as f32 / 255.0,
                    raw_rgb[2] as f32 / 255.0,
                    1.0,
                ]
            };

            let base_color = if obj.selected {
                SELECTION_COLOR
            } else {
                layer_rgba
            };

            let alpha = if mode == DisplayMode::Ghosted { 0.45 } else { 1.0 };
            let shaded_color = if obj.selected {
                [1.0, 0.88, 0.25, alpha]
            } else if mode == DisplayMode::Technical {
                [0.86, 0.88, 0.90, 1.0]
            } else if mode == DisplayMode::Arctic {
                [0.94, 0.95, 0.97, 1.0]
            } else if is_pure_black {
                [0.74, 0.77, 0.82, alpha]
            } else {
                [layer_rgba[0], layer_rgba[1], layer_rgba[2], alpha]
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
                    if mode != DisplayMode::Wireframe {
                        let mesh = surf.tessellate_display();
                        scene.push_mesh_triangles(
                            &mesh.positions,
                            &mesh.normals,
                            &mesh.indices,
                            shaded_color,
                        );
                    }
                    if mode != DisplayMode::Arctic || obj.selected {
                        let iso_color = if obj.selected {
                            SELECTION_COLOR
                        } else if mode == DisplayMode::Wireframe {
                            base_color
                        } else if mode == DisplayMode::Technical {
                            [0.10, 0.12, 0.15, 1.0]
                        } else {
                            [0.16, 0.18, 0.22, 0.85]
                        };
                        let inner = if mode == DisplayMode::Wireframe { 3 } else { 2 };
                        for poly in surf.isocurve_polylines(inner, inner, 36) {
                            for pair in poly.windows(2) {
                                scene.push_line(
                                    [pair[0].x as f32, pair[0].y as f32, pair[0].z as f32],
                                    [pair[1].x as f32, pair[1].y as f32, pair[1].z as f32],
                                    iso_color,
                                );
                            }
                        }
                    }
                }
                Geometry::Mesh(mesh) => {
                    scene.push_mesh(
                        &mesh.positions,
                        &mesh.normals,
                        &mesh.indices,
                        shaded_color,
                        base_color,
                        mode,
                        obj.selected,
                    );
                }
            }
        }

        // Render dimension extension lines & dimension lines for annotations
        for ann in &doc.annotations {
            if let Some(layer) = doc.layers.get(ann.layer_id) {
                if !layer.visible {
                    continue;
                }
            }
            if ann.kind == AnnotationKind::Dimension {
                if let (Some(p1), Some(p2), Some(off)) = (ann.p1, ann.p2, ann.offset_pt) {
                    let dir = (p2 - p1).normalize_or_zero();
                    let perp = off - (p1 + dir * (off - p1).dot(dir));
                    let d1 = p1 + perp;
                    let d2 = p2 + perp;
                    let col = [0.92, 0.94, 0.98, 1.0];
                    let ext_col = [0.68, 0.72, 0.78, 1.0];
                    let f = |v: DVec3| [v.x as f32, v.y as f32, v.z as f32];
                    scene.push_line(f(p1), f(d1), ext_col);
                    scene.push_line(f(p2), f(d2), ext_col);
                    scene.push_line(f(d1), f(d2), col);
                }
            }
        }

        scene
    }

    /// Build a viewport-specific construction plane (CPlane) grid centered near `doc_aabb`.
    pub fn build_grid(cplane: &Plane, doc_aabb: &Aabb, version: u64) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };
        let grid_step = if doc_aabb.is_empty() {
            1.0
        } else {
            let max_dim = doc_aabb.size().max_element().max(1.0);
            10.0_f64.powf((max_dim / 20.0).log10().floor()).max(1.0)
        };
        let (center_u_steps, center_v_steps) = if doc_aabb.is_empty() {
            (0_i32, 0_i32)
        } else {
            let rel = doc_aabb.center() - cplane.origin;
            let snap = grid_step * 5.0;
            let cu = (rel.dot(cplane.x_axis) / snap).round() as i32 * 5;
            let cv = (rel.dot(cplane.y_axis) / snap).round() as i32 * 5;
            (cu, cv)
        };
        scene.push_cplane_grid(cplane, center_u_steps, center_v_steps, 20, grid_step);
        scene
    }

    /// Build a dynamic overlay scene containing active tool previews and snap crosshair markers.
    pub fn build_overlay(previews: &[Geometry], snap_point: Option<DVec3>, version: u64) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };
        scene.push_preview(previews, snap_point);
        scene
    }

    fn append(&mut self, other: &Self) {
        let base = self.mesh_vertices.len() as u32;
        self.mesh_vertices.extend_from_slice(&other.mesh_vertices);
        self.mesh_indices
            .extend(other.mesh_indices.iter().map(|&idx| base + idx));
        self.line_vertices.extend_from_slice(&other.line_vertices);
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
                    let mesh = surf.tessellate_display();
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

    fn push_mesh_triangles(
        &mut self,
        positions: &[[f64; 3]],
        normals: &[[f64; 3]],
        indices: &[[u32; 3]],
        shaded_color: [f32; 4],
    ) {
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
            self.push_mesh_triangles(positions, normals, indices, shaded_color);
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
        } else if mode == DisplayMode::Shaded || mode == DisplayMode::Technical {
            let edge_color = if mode == DisplayMode::Technical {
                [0.10, 0.12, 0.15, 1.0]
            } else {
                [0.14, 0.16, 0.20, 1.0]
            };
            let quant = |p: [f64; 3]| -> [i64; 3] {
                [
                    (p[0] * 1e4).round() as i64,
                    (p[1] * 1e4).round() as i64,
                    (p[2] * 1e4).round() as i64,
                ]
            };
            let mut edge_faces: HashMap<([i64; 3], [i64; 3]), (DVec3, DVec3, Vec<DVec3>)> =
                HashMap::new();
            for &[a, b, c] in indices {
                let va = DVec3::from_array(positions[a as usize]);
                let vb = DVec3::from_array(positions[b as usize]);
                let vc = DVec3::from_array(positions[c as usize]);
                let fn_dir = (vb - va).cross(vc - va).normalize_or_zero();
                if fn_dir.length_squared() < 1e-8 {
                    continue;
                }
                for (u, v) in [(va, vb), (vb, vc), (vc, va)] {
                    let qu = quant(u.to_array());
                    let qv = quant(v.to_array());
                    let key = if qu <= qv { (qu, qv) } else { (qv, qu) };
                    let entry = edge_faces.entry(key).or_insert_with(|| (u, v, Vec::new()));
                    entry.2.push(fn_dir);
                }
            }
            for (_, (u, v, ns)) in edge_faces {
                let is_crease = ns.len() == 1 || ns.windows(2).any(|w| w[0].dot(w[1]) < 0.94);
                if is_crease {
                    self.push_line(
                        [u.x as f32, u.y as f32, u.z as f32],
                        [v.x as f32, v.y as f32, v.z as f32],
                        edge_color,
                    );
                }
            }
        }
    }

    fn axis_color(axis: DVec3) -> [f32; 4] {
        let ax = axis.x.abs();
        let ay = axis.y.abs();
        let az = axis.z.abs();
        if ax >= ay && ax >= az {
            [0.85, 0.22, 0.22, 1.0]
        } else if ay >= az {
            [0.22, 0.78, 0.30, 1.0]
        } else {
            [0.28, 0.54, 0.92, 1.0]
        }
    }

    fn push_cplane_grid(
        &mut self,
        plane: &Plane,
        center_u_steps: i32,
        center_v_steps: i32,
        half_count: i32,
        step: f64,
    ) {
        let minor = [0.32, 0.34, 0.38, 1.0];
        let major = [0.22, 0.24, 0.28, 1.0];
        let u_axis_color = Self::axis_color(plane.x_axis);
        let v_axis_color = Self::axis_color(plane.y_axis);

        let u_min = (center_u_steps - half_count) as f64 * step;
        let u_max = (center_u_steps + half_count) as f64 * step;
        let v_min = (center_v_steps - half_count) as f64 * step;
        let v_max = (center_v_steps + half_count) as f64 * step;

        for iv in (center_v_steps - half_count)..=(center_v_steps + half_count) {
            let d = iv as f64 * step;
            let is_major = iv.rem_euclid(5) == 0;
            let col = if iv == 0 {
                u_axis_color
            } else if is_major {
                major
            } else {
                minor
            };
            let p0 = plane.origin + plane.y_axis * d + plane.x_axis * u_min;
            let p1 = plane.origin + plane.y_axis * d + plane.x_axis * u_max;
            self.push_line(
                [p0.x as f32, p0.y as f32, p0.z as f32],
                [p1.x as f32, p1.y as f32, p1.z as f32],
                col,
            );
        }

        for iu in (center_u_steps - half_count)..=(center_u_steps + half_count) {
            let d = iu as f64 * step;
            let is_major = iu.rem_euclid(5) == 0;
            let col = if iu == 0 {
                v_axis_color
            } else if is_major {
                major
            } else {
                minor
            };
            let q0 = plane.origin + plane.x_axis * d + plane.y_axis * v_min;
            let q1 = plane.origin + plane.x_axis * d + plane.y_axis * v_max;
            self.push_line(
                [q0.x as f32, q0.y as f32, q0.z as f32],
                [q1.x as f32, q1.y as f32, q1.z as f32],
                col,
            );
        }
    }

    #[inline]
    fn push_line(&mut self, a: [f32; 3], b: [f32; 3], color: [f32; 4]) {
        self.line_vertices.push(LineVertex { position: a, color });
        self.line_vertices.push(LineVertex { position: b, color });
    }
}
