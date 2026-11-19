use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use cad_commands::{CommandSession, SnapConfig, SnapKind};
use cad_doc::{CadObject, Document, ObjectId};
use cad_geom::{DVec3, Geometry, NurbsCurve, Plane, TriangleMesh, UnitSystem};
use cad_io::{
    ProjectMetadata, export_from_document, import_into_document, open_project_with_meta,
    save_project_file_with_meta,
};
use cad_render::{DisplayMode, RenderScene, ViewportCamera, ViewportKind};
use cad_script::{DEFAULT_SAMPLE_SCRIPT, ParametricGraph};
use eframe::egui;
use serde::{Deserialize, Serialize};

pub const KNOWN_COMMANDS: &[&str] = &[
    "Line",
    "Polyline",
    "Rectangle",
    "Arc",
    "Circle",
    "Polygon",
    "Ellipse",
    "Box",
    "Cylinder",
    "Cone",
    "Torus",
    "Plane",
    "Sphere",
    "Extrude",
    "PlanarSrf",
    "Loft",
    "Revolve",
    "BooleanUnion",
    "BooleanDifference",
    "BooleanIntersection",
    "PointsOn",
    "PointsOff",
    "Join",
    "Explode",
    "Offset",
    "Fillet",
    "Move",
    "Copy",
    "Rotate",
    "Scale",
    "Mirror",
    "ArrayLinear",
    "ArrayPolar",
    "Distance",
    "Length",
    "Area",
    "Delete",
    "Hide",
    "Show",
    "Isolate",
    "Group",
    "Ungroup",
    "SelAll",
    "SelNone",
    "Invert",
    "SelLast",
    "SelCrv",
    "SelSrf",
    "SelMesh",
    "SelPt",
    "Undo",
    "Redo",
    "ZoomExtents",
    "Dim",
    "TextDot",
    "Text",
    "Wireframe",
    "Shaded",
    "Ghosted",
    "Technical",
    "Arctic",
    "Units",
    "New",
    "Help",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolPanelTab {
    None,
    Parametric,
    Script,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GumballMode {
    Move,
    Rotate,
}

#[derive(Debug, Clone)]
pub struct GumballDrag {
    pub viewport_idx: usize,
    pub axis: usize,
    pub mode: GumballMode,
    pub origin: DVec3,
    pub last_pos: egui::Pos2,
    pub axis_screen_dir: egui::Vec2,
    pub px_per_world: f64,
    pub total_amount: f64,
    pub moved: bool,
}

#[derive(Debug, Clone)]
pub struct GumballNumericPrompt {
    pub axis: usize,
    pub mode: GumballMode,
    pub text: String,
    pub just_opened: bool,
}

#[derive(Debug, Clone)]
pub struct MarqueeDrag {
    pub viewport_idx: usize,
    pub start: egui::Pos2,
    pub current: egui::Pos2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NamedView {
    pub name: String,
    pub cameras: [ViewportCamera; 4],
}

#[derive(Debug, Clone)]
pub struct GripDrag {
    pub viewport_idx: usize,
    pub object_id: ObjectId,
    pub cp_index: usize,
    pub last_world: DVec3,
    pub before_snapshot: CadObject,
}

pub struct ElephantApp {
    pub doc: Document,
    pub cameras: [ViewportCamera; 4],
    pub named_views: Vec<NamedView>,
    pub new_view_name: String,
    pub active_viewport: usize,
    pub maximized_viewport: Option<usize>,
    pub session: CommandSession,
    pub snap: SnapConfig,
    pub gumball_enabled: bool,
    pub gumball_drag: Option<GumballDrag>,
    pub gumball_prompt: Option<GumballNumericPrompt>,
    pub marquee: Option<MarqueeDrag>,
    pub points_on_objects: HashSet<ObjectId>,
    pub grip_drag: Option<GripDrag>,
    pub command_input: String,
    pub command_log: Vec<String>,
    pub command_history: Vec<String>,
    pub history_cursor: Option<usize>,
    pub last_command: Option<String>,
    pub new_layer_name: String,
    pub cursor_world: Option<DVec3>,
    pub last_snap_kind: Option<SnapKind>,
    pub tool_tab: ToolPanelTab,
    pub parametric_graph: ParametricGraph,
    pub script_source: String,
    pub rescale_on_unit_change: bool,
    pub cached_doc_scenes: HashMap<DisplayMode, Arc<RenderScene>>,
    pub doc_version: u64,
    pub doc_dirty: bool,
    pub cached_grids: [Option<Arc<RenderScene>>; 4],
    pub grid_version: u64,
    pub grids_dirty: bool,
    pub cached_overlay: Option<Arc<RenderScene>>,
    pub overlay_version: u64,
}

impl ElephantApp {
    pub fn new() -> Self {
        let mut app = Self {
            doc: Document::new(),
            cameras: [
                ViewportCamera::preset(ViewportKind::Top),
                ViewportCamera::preset(ViewportKind::Perspective),
                ViewportCamera::preset(ViewportKind::Front),
                ViewportCamera::preset(ViewportKind::Right),
            ],
            named_views: Vec::new(),
            new_view_name: String::new(),
            active_viewport: 1,
            maximized_viewport: None,
            session: CommandSession::default(),
            snap: SnapConfig::default(),
            gumball_enabled: true,
            gumball_drag: None,
            gumball_prompt: None,
            marquee: None,
            points_on_objects: HashSet::new(),
            grip_drag: None,
            command_input: String::new(),
            command_log: vec![
                "Elephant CAD initialized (local clean-room engine). Type 'Help' for commands."
                    .to_string(),
            ],
            command_history: Vec::new(),
            history_cursor: None,
            last_command: None,
            new_layer_name: String::new(),
            cursor_world: None,
            last_snap_kind: None,
            tool_tab: ToolPanelTab::None,
            parametric_graph: ParametricGraph::default(),
            script_source: DEFAULT_SAMPLE_SCRIPT.to_string(),
            rescale_on_unit_change: true,
            cached_doc_scenes: HashMap::new(),
            doc_version: 1,
            doc_dirty: true,
            cached_grids: [None, None, None, None],
            grid_version: 1,
            grids_dirty: true,
            cached_overlay: None,
            overlay_version: 0,
        };
        app.populate_starter_scene();
        app
    }

    pub fn populate_starter_scene(&mut self) {
        let box_id = self.doc.add_object(
            "StarterBox",
            Geometry::Mesh(TriangleMesh::from_box(
                DVec3::new(0.0, 0.0, 1.0),
                DVec3::new(3.0, 3.0, 2.0),
            )),
        );
        if let Some(obj) = self.doc.objects.get_mut(box_id) {
            obj.name = Some("Box_01".to_string());
        }

        let sphere_id = self.doc.add_object(
            "StarterSphere",
            Geometry::Mesh(TriangleMesh::from_sphere(
                DVec3::new(4.5, 0.0, 1.5),
                1.5,
                24,
                16,
            )),
        );
        if let Some(obj) = self.doc.objects.get_mut(sphere_id) {
            obj.name = Some("Sphere_01".to_string());
        }

        if let Ok(circle) = NurbsCurve::circle(&Plane::world_xy(), 4.5) {
            let cid = self.doc.add_object("StarterCircle", Geometry::Curve(circle));
            if let Some(obj) = self.doc.objects.get_mut(cid) {
                obj.name = Some("Circle_01".to_string());
            }
        }
    }

    pub fn log(&mut self, msg: impl Into<String>) {
        self.command_log.push(msg.into());
        if self.command_log.len() > 200 {
            let drain = self.command_log.len() - 200;
            self.command_log.drain(0..drain);
        }
    }

    pub fn mark_doc_dirty(&mut self) {
        self.doc_dirty = true;
    }

    pub fn mark_scene_dirty(&mut self) {
        self.doc_dirty = true;
        self.grids_dirty = true;
    }

    pub fn prune_points_on(&mut self) {
        self.points_on_objects
            .retain(|id| self.doc.objects.get(*id).is_some_and(|o| o.visible));
    }

    pub fn active_display_mode(&self) -> DisplayMode {
        self.cameras[self.active_viewport].display_mode
    }

    pub fn set_active_viewport_display_mode(&mut self, mode: DisplayMode) {
        self.cameras[self.active_viewport].display_mode = mode;
        let title = self.cameras[self.active_viewport].kind.title();
        self.log(format!("{title} display mode: {}.", mode.label()));
    }

    pub fn set_all_viewports_display_mode(&mut self, mode: DisplayMode) {
        for cam in &mut self.cameras {
            cam.display_mode = mode;
        }
        self.log(format!("All viewports display mode: {}.", mode.label()));
    }

    pub fn change_units(&mut self, target: UnitSystem, rescale: bool) {
        let prev = self.doc.units;
        self.doc.set_units(target, rescale);
        if rescale && prev != target {
            let s = prev.scale_to(target);
            for cam in &mut self.cameras {
                cam.target *= s;
                cam.distance = (cam.distance * s).clamp(0.1, 10_000_000.0);
                cam.ortho_half_height = (cam.ortho_half_height * s).clamp(0.1, 10_000_000.0);
            }
        }
        self.mark_scene_dirty();
        self.log(format!(
            "Units set to {} ({}){}, abs tol = {:.6}.",
            target.full_name(),
            target.label(),
            if rescale && prev != target {
                " (geometry rescaled)"
            } else {
                ""
            },
            self.doc.tolerances.absolute
        ));
    }

    pub fn zoom_extents_all(&mut self) {
        let total = self.doc.visible_bounding_box();
        for cam in &mut self.cameras {
            cam.zoom_extents(&total);
        }
    }

    pub fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("CAD Projects (.elephant, .3dm)", &["elephant", "3dm", "json"])
            .pick_file()
        {
            match open_project_with_meta(&path) {
                Ok((doc, meta)) => {
                    self.session.cancel();
                    self.points_on_objects.clear();
                    self.grip_drag = None;
                    self.gumball_drag = None;
                    self.gumball_prompt = None;
                    self.marquee = None;
                    self.doc = doc;
                    self.named_views = meta
                        .named_views
                        .into_iter()
                        .filter_map(|v| serde_json::from_value(v).ok())
                        .collect();
                    self.parametric_graph = meta
                        .parametric_graph
                        .and_then(|v| serde_json::from_value(v).ok())
                        .unwrap_or_default();
                    self.script_source = meta
                        .script_source
                        .unwrap_or_else(|| DEFAULT_SAMPLE_SCRIPT.to_string());
                    self.mark_scene_dirty();
                    self.zoom_extents_all();
                    self.log(format!("Opened {}", path.display()));
                }
                Err(e) => self.log(format!("Open failed: {e}")),
            }
        }
    }

    pub fn save_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Elephant Project", &["elephant"])
            .set_file_name("model.elephant")
            .save_file()
        {
            let meta = ProjectMetadata {
                named_views: self
                    .named_views
                    .iter()
                    .filter_map(|nv| serde_json::to_value(nv).ok())
                    .collect(),
                parametric_graph: serde_json::to_value(&self.parametric_graph).ok(),
                script_source: Some(self.script_source.clone()),
            };
            match save_project_file_with_meta(&path, &self.doc, &meta) {
                Ok(()) => self.log(format!("Saved {}", path.display())),
                Err(e) => self.log(format!("Save failed: {e}")),
            }
        }
    }

    pub fn import_mesh_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter(
                "CAD / 3D Files (.3dm, .dxf, .obj, .stl)",
                &["3dm", "dxf", "obj", "stl"],
            )
            .pick_file()
        {
            match import_into_document(&path, &mut self.doc) {
                Ok(n) => {
                    self.mark_scene_dirty();
                    self.zoom_extents_all();
                    self.log(format!("Imported {n} object(s) from {}", path.display()));
                }
                Err(e) => self.log(format!("Import failed: {e}")),
            }
        }
    }

    pub fn export_mesh_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Wavefront OBJ", &["obj"])
            .add_filter("Binary STL", &["stl"])
            .add_filter("glTF Binary (.glb)", &["glb"])
            .add_filter("AutoCAD DXF (.dxf)", &["dxf"])
            .add_filter("Scalable Vector Graphics (.svg)", &["svg"])
            .set_file_name("export.obj")
            .save_file()
        {
            match export_from_document(&path, &self.doc) {
                Ok(()) => self.log(format!("Exported {}", path.display())),
                Err(e) => self.log(format!("Export failed: {e}")),
            }
        }
    }

    pub fn ensure_render_caches(&mut self) {
        if self.doc_dirty {
            self.doc_version = self.doc_version.wrapping_add(1).max(1);
            self.cached_doc_scenes.clear();
            self.doc_dirty = false;
        }
        for cam in &self.cameras {
            let mode = cam.display_mode;
            if !self.cached_doc_scenes.contains_key(&mode) {
                let built = RenderScene::build_document(&self.doc, mode, self.doc_version);
                self.cached_doc_scenes.insert(mode, Arc::new(built));
            }
        }

        if self.grids_dirty || self.cached_grids.iter().any(|g| g.is_none()) {
            self.grid_version = self.grid_version.wrapping_add(1).max(1);
            let doc_aabb = self.doc.visible_bounding_box();
            for i in 0..4 {
                self.cached_grids[i] = Some(Arc::new(RenderScene::build_grid(
                    &self.cameras[i].cplane,
                    &doc_aabb,
                    self.grid_version,
                )));
            }
            self.grids_dirty = false;
        }

        let has_dynamic_preview = self.session.is_active() || self.parametric_graph.live_preview;
        if has_dynamic_preview {
            let mut previews = Vec::new();
            if self.parametric_graph.live_preview {
                previews.extend(self.parametric_graph.evaluate());
            }
            let mut snap_marker = None;
            if self.session.is_active() {
                let cplane = self.cameras[self.active_viewport].cplane;
                previews.extend(self.session.preview_geometry(self.cursor_world, &cplane));
                if self.last_snap_kind.is_some() {
                    snap_marker = self.cursor_world;
                }
            }
            self.overlay_version = self.overlay_version.wrapping_add(1).max(1);
            self.cached_overlay = Some(Arc::new(RenderScene::build_overlay(
                &previews,
                snap_marker,
                self.overlay_version,
            )));
        } else {
            self.cached_overlay = None;
        }
    }
}
