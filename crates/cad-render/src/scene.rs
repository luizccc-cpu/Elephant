use std::collections::HashMap;

use bytemuck::{Pod, Zeroable};
use cad_doc::{AnnotationKind, Document};
use cad_geom::{
    Aabb, Geometry, compute_mesh_discrete_curvature, section_geometry_with_plane,
    tessellate_brep_with_curvature, tessellate_surface_with_curvature,
};
use cad_math::{DVec3, Plane};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DisplayMode {
    Wireframe,
    Shaded,
    Ghosted,
    Technical,
    Arctic,
    Rendered,
    Concept,
}

impl DisplayMode {
    pub const ALL: [Self; 7] = [
        Self::Wireframe,
        Self::Shaded,
        Self::Ghosted,
        Self::Technical,
        Self::Arctic,
        Self::Rendered,
        Self::Concept,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Wireframe => "Wireframe",
            Self::Shaded => "Shaded",
            Self::Ghosted => "Ghosted",
            Self::Technical => "Technical",
            Self::Arctic => "Arctic",
            Self::Rendered => "Rendered (PBR)",
            Self::Concept => "Concept",
        }
    }
}

/// Real-time Class-A surface diagnostic shader mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfaceAnalysisMode {
    None,
    Zebra,
    EnvironmentMap,
    GaussianCurvature,
    MeanCurvature,
    DraftAngle,
}

impl Default for SurfaceAnalysisMode {
    fn default() -> Self {
        Self::None
    }
}

impl SurfaceAnalysisMode {
    pub const ALL: [Self; 6] = [
        Self::None,
        Self::Zebra,
        Self::EnvironmentMap,
        Self::GaussianCurvature,
        Self::MeanCurvature,
        Self::DraftAngle,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::None => "Off",
            Self::Zebra => "Zebra",
            Self::EnvironmentMap => "E-Map",
            Self::GaussianCurvature => "Gaussian Curvature",
            Self::MeanCurvature => "Mean Curvature",
            Self::DraftAngle => "Draft Angle",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SurfaceAnalysisConfig {
    pub mode: SurfaceAnalysisMode,
    pub zebra_stripes: f32,
    pub zebra_angle_deg: f32,
    pub emap_preset: u32,
    pub curvature_min: f32,
    pub curvature_max: f32,
    pub draft_pull_dir: DVec3,
    pub draft_angle_deg: f32,
    pub cavity_ao_strength: f32,
}

impl Default for SurfaceAnalysisConfig {
    fn default() -> Self {
        Self {
            mode: SurfaceAnalysisMode::None,
            zebra_stripes: 10.0,
            zebra_angle_deg: 0.0,
            emap_preset: 0,
            curvature_min: -0.5,
            curvature_max: 0.5,
            draft_pull_dir: DVec3::Z,
            draft_angle_deg: 2.0,
            cavity_ao_strength: 0.45,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
    pub curvature: [f32; 2],
    pub material: [f32; 2],
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
        Self::build_document_with_options(doc, mode, version, false)
    }

    /// Build visible document geometry with optional analytical Gaussian/Mean curvature attributes.
    pub fn build_document_with_options(
        doc: &Document,
        mode: DisplayMode,
        version: u64,
        compute_curvature: bool,
    ) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };

        for (_, obj) in &doc.objects {
            if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
                continue;
            }
            let Some(layer) = doc.layers.get(obj.layer_id) else {
                continue;
            };

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

            let mat = obj.material.unwrap_or_default();
            let alpha = if mode == DisplayMode::Ghosted {
                0.45 * mat.opacity
            } else {
                mat.opacity
            };
            let mat_pair = [mat.metallic, mat.roughness];
            let shaded_color = if obj.selected {
                [1.0, 0.88, 0.25, alpha]
            } else if mode == DisplayMode::Technical {
                [0.86, 0.88, 0.90, alpha]
            } else if mode == DisplayMode::Arctic {
                [0.94, 0.95, 0.97, alpha]
            } else if is_pure_black {
                [0.74, 0.77, 0.82, alpha]
            } else {
                [layer_rgba[0], layer_rgba[1], layer_rgba[2], alpha]
            };
            let suppress_edges = matches!(mode, DisplayMode::Arctic | DisplayMode::Rendered);

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
                        if compute_curvature {
                            let cm = tessellate_surface_with_curvature(surf);
                            scene.push_mesh_triangles_with_data(
                                &cm.mesh.positions,
                                &cm.mesh.normals,
                                &cm.mesh.indices,
                                shaded_color,
                                Some((&cm.gaussian, &cm.mean)),
                                mat_pair,
                            );
                        } else {
                            let mesh = surf.tessellate_display();
                            scene.push_mesh_triangles_with_data(
                                &mesh.positions,
                                &mesh.normals,
                                &mesh.indices,
                                shaded_color,
                                None,
                                mat_pair,
                            );
                        }
                    }
                    if (!suppress_edges && mode != DisplayMode::Concept) || obj.selected {
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
                Geometry::BRep(brep) => {
                    if mode != DisplayMode::Wireframe {
                        if compute_curvature {
                            let cm = tessellate_brep_with_curvature(brep);
                            scene.push_mesh_triangles_with_data(
                                &cm.mesh.positions,
                                &cm.mesh.normals,
                                &cm.mesh.indices,
                                shaded_color,
                                Some((&cm.gaussian, &cm.mean)),
                                mat_pair,
                            );
                        } else {
                            let mesh = brep.tessellate_display();
                            scene.push_mesh_triangles_with_data(
                                &mesh.positions,
                                &mesh.normals,
                                &mesh.indices,
                                shaded_color,
                                None,
                                mat_pair,
                            );
                        }
                    }
                    if !suppress_edges || obj.selected {
                        let edge_color = if obj.selected {
                            SELECTION_COLOR
                        } else if mode == DisplayMode::Wireframe {
                            base_color
                        } else if mode == DisplayMode::Technical || mode == DisplayMode::Concept {
                            [0.08, 0.10, 0.14, 1.0]
                        } else {
                            [0.12, 0.14, 0.18, 0.92]
                        };
                        for edge in &brep.edges {
                            if edge.curve.degree == 1 && edge.curve.control_points.len() == 2 {
                                let a = edge.curve.control_points[0].position;
                                let b = edge.curve.control_points[1].position;
                                scene.push_line(
                                    [a.x as f32, a.y as f32, a.z as f32],
                                    [b.x as f32, b.y as f32, b.z as f32],
                                    edge_color,
                                );
                            } else {
                                let pts = edge.curve.sample_polyline(32);
                                for pair in pts.windows(2) {
                                    scene.push_line(
                                        [pair[0].x as f32, pair[0].y as f32, pair[0].z as f32],
                                        [pair[1].x as f32, pair[1].y as f32, pair[1].z as f32],
                                        edge_color,
                                    );
                                }
                            }
                        }
                        if mode != DisplayMode::Concept {
                            let iso_color =
                                [edge_color[0], edge_color[1], edge_color[2], 0.55];
                            for f in &brep.faces {
                                if (f.surface.degree_u > 1 || f.surface.degree_v > 1)
                                    && !f.is_trimmed()
                                {
                                    for poly in f.surface.isocurve_polylines(1, 1, 32) {
                                        for pair in poly.windows(2) {
                                            scene.push_line(
                                                [
                                                    pair[0].x as f32,
                                                    pair[0].y as f32,
                                                    pair[0].z as f32,
                                                ],
                                                [
                                                    pair[1].x as f32,
                                                    pair[1].y as f32,
                                                    pair[1].z as f32,
                                                ],
                                                iso_color,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                Geometry::SubD(sd) => {
                    let mesh = sd.tessellate_display();
                    let curv = if compute_curvature && mode != DisplayMode::Wireframe {
                        Some(compute_mesh_discrete_curvature(&mesh))
                    } else {
                        None
                    };
                    let curv_ref = curv
                        .as_ref()
                        .map(|(g, m)| (g.as_slice(), m.as_slice()));
                    if mode != DisplayMode::Wireframe {
                        scene.push_mesh_triangles_with_data(
                            &mesh.positions,
                            &mesh.normals,
                            &mesh.indices,
                            shaded_color,
                            curv_ref,
                            mat_pair,
                        );
                    }
                    if !suppress_edges && (mode == DisplayMode::Wireframe || mode == DisplayMode::Technical || obj.selected) {
                        for (a, b, is_creased) in sd.control_cage_edges() {
                            let col = if is_creased {
                                [1.0, 0.55, 0.15, 0.95]
                            } else if obj.selected {
                                SELECTION_COLOR
                            } else {
                                base_color
                            };
                            scene.push_line(
                                [a.x as f32, a.y as f32, a.z as f32],
                                [b.x as f32, b.y as f32, b.z as f32],
                                col,
                            );
                        }
                    } else if mode == DisplayMode::Shaded || mode == DisplayMode::Ghosted || mode == DisplayMode::Concept {
                        for (a, b, is_creased) in sd.control_cage_edges() {
                            if is_creased {
                                scene.push_line(
                                    [a.x as f32, a.y as f32, a.z as f32],
                                    [b.x as f32, b.y as f32, b.z as f32],
                                    [0.14, 0.16, 0.20, 0.90],
                                );
                            }
                        }
                    }
                }
                Geometry::Mesh(mesh) => {
                    let curv = if compute_curvature && mode != DisplayMode::Wireframe {
                        Some(compute_mesh_discrete_curvature(mesh))
                    } else {
                        None
                    };
                    let curv_ref = curv
                        .as_ref()
                        .map(|(g, m)| (g.as_slice(), m.as_slice()));
                    scene.push_mesh_full(
                        &mesh.positions,
                        &mesh.normals,
                        &mesh.indices,
                        shaded_color,
                        base_color,
                        mode,
                        obj.selected,
                        curv_ref,
                        mat_pair,
                    );
                }
            }
        }

        // Render dimension extension lines & dimension lines for annotations
        for ann in &doc.annotations {
            if !doc.is_layer_effectively_visible(ann.layer_id) {
                continue;
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

    /// Build unclipped 3D frames, normal handles, and real-time section cut contours for active clipping planes.
    pub fn build_clipping_overlay(doc: &Document, version: u64) -> Self {
        let mut scene = Self {
            version,
            ..Default::default()
        };
        let frame_col = [0.98, 0.68, 0.22, 0.95];
        let cut_col = [1.0, 0.28, 0.22, 1.0];
        let norm_col = [0.35, 0.85, 1.0, 1.0];
        let f = |v: DVec3| [v.x as f32, v.y as f32, v.z as f32];

        for cp in &doc.clipping_planes {
            if !cp.enabled {
                continue;
            }
            let half = (cp.size * 0.5).max(1.0);
            let o = cp.plane.origin;
            let u = cp.plane.x_axis * half;
            let v = cp.plane.y_axis * half;
            let c0 = o - u - v;
            let c1 = o + u - v;
            let c2 = o + u + v;
            let c3 = o - u + v;
            scene.push_line(f(c0), f(c1), frame_col);
            scene.push_line(f(c1), f(c2), frame_col);
            scene.push_line(f(c2), f(c3), frame_col);
            scene.push_line(f(c3), f(c0), frame_col);
            scene.push_line(f(o - u * 0.3), f(o + u * 0.3), frame_col);
            scene.push_line(f(o - v * 0.3), f(o + v * 0.3), frame_col);
            let arrow_tip = o + cp.effective_normal() * (half * 0.35);
            scene.push_line(f(o), f(arrow_tip), norm_col);

            if cp.fill_caps {
                for (_, obj) in &doc.objects {
                    if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
                        continue;
                    }
                    for poly in section_geometry_with_plane(&obj.geometry, &cp.plane) {
                        for pair in poly.windows(2) {
                            scene.push_line(f(pair[0]), f(pair[1]), cut_col);
                        }
                    }
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
                Geometry::BRep(brep) => {
                    let mesh = brep.tessellate_display();
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
                Geometry::SubD(sd) => {
                    let mesh = sd.tessellate_display();
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

    fn push_mesh_triangles_with_data(
        &mut self,
        positions: &[[f64; 3]],
        normals: &[[f64; 3]],
        indices: &[[u32; 3]],
        shaded_color: [f32; 4],
        curvature: Option<(&[f64], &[f64])>,
        material: [f32; 2],
    ) {
        let base = self.mesh_vertices.len() as u32;
        for (i, (p, n)) in positions.iter().zip(normals.iter()).enumerate() {
            let curv = if let Some((g, m)) = curvature {
                [
                    g.get(i).copied().unwrap_or(0.0) as f32,
                    m.get(i).copied().unwrap_or(0.0) as f32,
                ]
            } else {
                [0.0, 0.0]
            };
            self.mesh_vertices.push(MeshVertex {
                position: [p[0] as f32, p[1] as f32, p[2] as f32],
                normal: [n[0] as f32, n[1] as f32, n[2] as f32],
                color: shaded_color,
                curvature: curv,
                material,
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
        self.push_mesh_full(
            positions,
            normals,
            indices,
            shaded_color,
            wire_color,
            mode,
            selected,
            None,
            [0.05, 0.42],
        );
    }

    #[allow(clippy::too_many_arguments)]
    fn push_mesh_full(
        &mut self,
        positions: &[[f64; 3]],
        normals: &[[f64; 3]],
        indices: &[[u32; 3]],
        shaded_color: [f32; 4],
        wire_color: [f32; 4],
        mode: DisplayMode,
        selected: bool,
        curvature: Option<(&[f64], &[f64])>,
        material: [f32; 2],
    ) {
        if mode != DisplayMode::Wireframe {
            self.push_mesh_triangles_with_data(
                positions,
                normals,
                indices,
                shaded_color,
                curvature,
                material,
            );
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
        } else if matches!(
            mode,
            DisplayMode::Shaded | DisplayMode::Technical | DisplayMode::Concept
        ) {
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
