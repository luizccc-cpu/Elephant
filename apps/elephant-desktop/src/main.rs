#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::sync::Arc;

use cad_commands::{CommandSession, SnapConfig, SnapKind};
use cad_doc::Document;
use cad_geom::{DMat4, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh};
use cad_io::{export_from_document, import_into_document, open_project_file};
use cad_render::{
    CadWgpuState, DisplayMode, RenderScene, ViewportCamera, ViewportKind, ViewportPaintCallback,
};
use cad_script::{DEFAULT_SAMPLE_SCRIPT, NodeKind, ParametricGraph, run_rhai_script};
use eframe::egui;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Elephant CAD")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 540.0]),
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 32,
        ..Default::default()
    };

    eframe::run_native(
        "Elephant CAD",
        options,
        Box::new(|cc| {
            if let Some(wgpu_state) = &cc.wgpu_render_state {
                let cad_state = CadWgpuState::new(
                    &wgpu_state.device,
                    wgpu_state.target_format,
                    Some(eframe::egui_wgpu::wgpu::TextureFormat::Depth32Float),
                );
                wgpu_state
                    .renderer
                    .write()
                    .callback_resources
                    .insert(cad_state);
            }
            Ok(Box::new(ElephantApp::new()))
        }),
    )
}

const KNOWN_COMMANDS: &[&str] = &[
    "Line",
    "Polyline",
    "Rectangle",
    "Arc",
    "Circle",
    "Polygon",
    "Ellipse",
    "Box",
    "Sphere",
    "Extrude",
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
    "SelAll",
    "SelNone",
    "Undo",
    "Redo",
    "ZoomExtents",
    "Wireframe",
    "Shaded",
    "Ghosted",
    "New",
    "Help",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ToolPanelTab {
    None,
    Parametric,
    Script,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GumballMode {
    Move,
    Rotate,
}

#[derive(Debug, Clone)]
struct GumballDrag {
    viewport_idx: usize,
    axis: usize,
    mode: GumballMode,
    origin: DVec3,
    last_pos: egui::Pos2,
    axis_screen_dir: egui::Vec2,
    px_per_world: f64,
    total_amount: f64,
    moved: bool,
}

#[derive(Debug, Clone)]
struct GumballNumericPrompt {
    axis: usize,
    mode: GumballMode,
    text: String,
    just_opened: bool,
}

#[derive(Debug, Clone)]
struct MarqueeDrag {
    viewport_idx: usize,
    start: egui::Pos2,
    current: egui::Pos2,
}

struct ElephantApp {
    doc: Document,
    cameras: [ViewportCamera; 4],
    active_viewport: usize,
    maximized_viewport: Option<usize>,
    display_mode: DisplayMode,
    session: CommandSession,
    snap: SnapConfig,
    gumball_enabled: bool,
    gumball_drag: Option<GumballDrag>,
    gumball_prompt: Option<GumballNumericPrompt>,
    marquee: Option<MarqueeDrag>,
    command_input: String,
    command_log: Vec<String>,
    command_history: Vec<String>,
    history_cursor: Option<usize>,
    last_command: Option<String>,
    new_layer_name: String,
    cursor_world: Option<DVec3>,
    last_snap_kind: Option<SnapKind>,
    tool_tab: ToolPanelTab,
    parametric_graph: ParametricGraph,
    script_source: String,
    cached_scene: Option<Arc<RenderScene>>,
    scene_version: u64,
    scene_dirty: bool,
}

impl ElephantApp {
    fn new() -> Self {
        let mut app = Self {
            doc: Document::new(),
            cameras: [
                ViewportCamera::preset(ViewportKind::Top),
                ViewportCamera::preset(ViewportKind::Perspective),
                ViewportCamera::preset(ViewportKind::Front),
                ViewportCamera::preset(ViewportKind::Right),
            ],
            active_viewport: 1,
            maximized_viewport: None,
            display_mode: DisplayMode::Shaded,
            session: CommandSession::default(),
            snap: SnapConfig::default(),
            gumball_enabled: true,
            gumball_drag: None,
            gumball_prompt: None,
            marquee: None,
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
            cached_scene: None,
            scene_version: 1,
            scene_dirty: true,
        };
        app.populate_starter_scene();
        app
    }

    fn populate_starter_scene(&mut self) {
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

    fn log(&mut self, msg: impl Into<String>) {
        self.command_log.push(msg.into());
        if self.command_log.len() > 200 {
            let drain = self.command_log.len() - 200;
            self.command_log.drain(0..drain);
        }
    }

    fn zoom_extents_all(&mut self) {
        let mut total = cad_math::Aabb::EMPTY;
        for (_, obj) in &self.doc.objects {
            if let Some(layer) = self.doc.layers.get(obj.layer_id) {
                if layer.visible {
                    total = total.union(&obj.aabb);
                }
            }
        }
        for cam in &mut self.cameras {
            cam.zoom_extents(&total);
        }
    }

    fn open_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("CAD Projects (.elephant, .3dm)", &["elephant", "3dm", "json"])
            .pick_file()
        {
            match open_project_file(&path) {
                Ok(doc) => {
                    self.session.cancel();
                    self.doc = doc;
                    self.scene_dirty = true;
                    self.zoom_extents_all();
                    self.log(format!("Opened {}", path.display()));
                }
                Err(e) => self.log(format!("Open failed: {e}")),
            }
        }
    }

    fn save_project_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Elephant Project", &["elephant"])
            .set_file_name("model.elephant")
            .save_file()
        {
            match export_from_document(&path, &self.doc) {
                Ok(()) => self.log(format!("Saved {}", path.display())),
                Err(e) => self.log(format!("Save failed: {e}")),
            }
        }
    }

    fn import_mesh_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("3D Files (.3dm, .obj, .stl)", &["3dm", "obj", "stl"])
            .pick_file()
        {
            match import_into_document(&path, &mut self.doc) {
                Ok(n) => {
                    self.scene_dirty = true;
                    self.zoom_extents_all();
                    self.log(format!("Imported {n} object(s) from {}", path.display()));
                }
                Err(e) => self.log(format!("Import failed: {e}")),
            }
        }
    }

    fn export_mesh_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Wavefront OBJ", &["obj"])
            .add_filter("Binary STL", &["stl"])
            .set_file_name("export.obj")
            .save_file()
        {
            match export_from_document(&path, &self.doc) {
                Ok(()) => self.log(format!("Exported {}", path.display())),
                Err(e) => self.log(format!("Export failed: {e}")),
            }
        }
    }

    fn execute_command(&mut self, raw: &str) {
        self.scene_dirty = true;
        let trimmed = raw.trim();
        let cplane = self.cameras[self.active_viewport].cplane;

        if self.session.is_active() {
            if !trimmed.is_empty() {
                self.log(format!("> {trimmed}"));
            }
            if let Some(outcome) = self.session.on_text_input(trimmed, &mut self.doc, &cplane) {
                for m in outcome.messages {
                    self.log(m);
                }
                return;
            }
        }

        if trimmed.is_empty() {
            if let Some(prev) = self.last_command.clone() {
                self.execute_command(&prev);
            }
            return;
        }
        self.history_cursor = None;
        if self.command_history.last().map(|s| s.as_str()) != Some(trimmed) {
            self.command_history.push(trimmed.to_string());
            if self.command_history.len() > 100 {
                self.command_history.remove(0);
            }
        }
        self.last_command = Some(trimmed.to_string());
        self.log(format!("> {trimmed}"));
        let mut parts = trimmed.split_whitespace();
        let cmd = parts.next().unwrap_or("").to_ascii_lowercase();
        let args: Vec<&str> = parts.collect();

        if args.is_empty() {
            if let Some(prompt) = self.session.start_tool(&cmd, &self.doc) {
                self.log(prompt);
                return;
            }
        }

        match cmd.as_str() {
            "box" => {
                let s = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                let id = self.doc.add_object(
                    "Box",
                    Geometry::Mesh(TriangleMesh::from_box(
                        DVec3::new(0.0, 0.0, s * 0.5),
                        DVec3::splat(s),
                    )),
                );
                if let Some(obj) = self.doc.objects.get_mut(id) {
                    obj.name = Some("Box".to_string());
                }
                self.log(format!("Created Box ({s} x {s} x {s})."));
            }
            "sphere" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                let id = self.doc.add_object(
                    "Sphere",
                    Geometry::Mesh(TriangleMesh::from_sphere(
                        DVec3::new(0.0, 0.0, r),
                        r,
                        24,
                        16,
                    )),
                );
                if let Some(obj) = self.doc.objects.get_mut(id) {
                    obj.name = Some("Sphere".to_string());
                }
                self.log(format!("Created Sphere (r = {r})."));
            }
            "circle" => {
                let r = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                match NurbsCurve::circle(&Plane::world_xy(), r) {
                    Ok(c) => {
                        self.doc.add_object("Circle", Geometry::Curve(c));
                        self.log(format!("Created NURBS Circle (r = {r})."));
                    }
                    Err(e) => self.log(format!("Error: {e}")),
                }
            }
            "line" => {
                let len = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(5.0);
                let c = NurbsCurve::line(DVec3::ZERO, DVec3::new(len, 0.0, 0.0));
                self.doc.add_object("Line", Geometry::Curve(c));
                self.log(format!("Created Line (0,0,0) -> ({len},0,0)."));
            }
            "extrude" => {
                let h = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(3.0);
                let curves: Vec<NurbsCurve> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some(c.clone()),
                        _ => None,
                    })
                    .collect();
                if curves.is_empty() {
                    self.log("Select one or more curves first, then run Extrude.");
                } else {
                    for c in &curves {
                        let surf = NurbsSurface::extrude_curve(c, DVec3::new(0.0, 0.0, h));
                        self.doc.add_object("Extrude", Geometry::Surface(surf));
                    }
                    self.log(format!("Extruded {} curve(s) by Z = {h}.", curves.len()));
                }
            }
            "move" => {
                let dx = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let dy = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                let dz = args.get(2).and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
                let n = self
                    .doc
                    .transform_selected("Move", &DMat4::from_translation(DVec3::new(dx, dy, dz)));
                self.log(format!("Moved {n} selected object(s) by ({dx}, {dy}, {dz})."));
            }
            "scale" => {
                let s = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.5);
                let n = self
                    .doc
                    .transform_selected("Scale", &DMat4::from_scale(DVec3::splat(s)));
                self.log(format!("Scaled {n} selected object(s) by {s}."));
            }
            "delete" => {
                let n = self.doc.remove_selected();
                self.log(format!("Deleted {n} object(s)."));
            }
            "selall" => {
                let n = self.doc.select_all();
                self.log(format!("Selected {n} object(s)."));
            }
            "selnone" => {
                self.doc.clear_selection();
                self.log("Selection cleared.");
            }
            "ellipse" => {
                let rx = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(4.0);
                let ry = args.get(1).and_then(|v| v.parse::<f64>().ok()).unwrap_or(2.0);
                match NurbsCurve::ellipse(&cplane, rx, ry) {
                    Ok(c) => {
                        self.doc.add_object("Ellipse", Geometry::Curve(c));
                        self.log(format!("Created Ellipse (rx={rx}, ry={ry})."));
                    }
                    Err(e) => self.log(format!("Ellipse error: {e}")),
                }
            }
            "join" => {
                let mut curves = Vec::new();
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            curves.push(c.clone());
                        }
                    }
                }
                if curves.len() < 2 {
                    self.log("Select 2 or more curves to join.");
                } else {
                    let joined = NurbsCurve::join_curves(&curves, 1e-3);
                    let removed = self.doc.remove_selected();
                    let count = joined.len();
                    for c in joined {
                        self.doc.add_object("Join", Geometry::Curve(c));
                    }
                    self.log(format!("Joined {removed} curve(s) into {count} curve(s)."));
                }
            }
            "explode" => {
                let mut pieces = Vec::new();
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            let segs = c.explode_segments();
                            if segs.len() > 1 {
                                pieces.extend(segs.into_iter().map(|s| (obj.layer_id, s)));
                            }
                        }
                    }
                }
                if pieces.is_empty() {
                    self.log("Select one or more polylines to explode.");
                } else {
                    let removed = self.doc.remove_selected();
                    let count = pieces.len();
                    for (lid, seg) in pieces {
                        self.doc.add_object_on_layer("Explode", lid, Geometry::Curve(seg));
                    }
                    self.log(format!("Exploded {removed} polyline(s) into {count} segments."));
                }
            }
            "fillet" => {
                let radius = args.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(1.0);
                let lines: Vec<_> = self
                    .doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) if c.degree == 1 && c.control_points.len() == 2 => {
                            Some((o.layer_id, c.control_points[0].position, c.control_points[1].position))
                        }
                        _ => None,
                    })
                    .collect();
                if lines.len() != 2 {
                    self.log("Select exactly 2 line segments to fillet, then run Fillet [radius].");
                } else if let Some((la, lb, arc)) =
                    NurbsCurve::fillet_two_lines(lines[0].1, lines[0].2, lines[1].1, lines[1].2, radius)
                {
                    let lid = lines[0].0;
                    self.doc.remove_selected();
                    self.doc.add_object_on_layer("FilletLine", lid, Geometry::Curve(la));
                    self.doc.add_object_on_layer("FilletLine", lid, Geometry::Curve(lb));
                    self.doc.add_object_on_layer("FilletArc", lid, Geometry::Curve(arc));
                    self.log(format!("Filleted 2 lines with radius {radius:.3}."));
                } else {
                    self.log("Could not fillet the selected lines (parallel, non-coplanar, or radius too large).");
                }
            }
            "length" => {
                let mut total = 0.0;
                let mut count = 0;
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            total += c.length_approx(64);
                            count += 1;
                        }
                    }
                }
                if count == 0 {
                    self.log("Select one or more curves to measure Length.");
                } else {
                    self.log(format!("Total length of {count} curve(s): {total:.4}"));
                }
            }
            "area" => {
                let mut total = 0.0;
                let mut count = 0;
                for obj in self.doc.objects.values() {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            if c.is_closed() {
                                total += c.planar_area_approx(&cplane, 64);
                                count += 1;
                            }
                        }
                    }
                }
                if count == 0 {
                    self.log("Select one or more closed planar curves to measure Area.");
                } else {
                    self.log(format!("Total planar area of {count} closed curve(s): {total:.4}"));
                }
            }
            "undo" => match self.doc.undo() {
                Some(label) => self.log(format!("Undid '{label}'.")),
                None => self.log("Nothing to undo."),
            },
            "redo" => match self.doc.redo() {
                Some(label) => self.log(format!("Redid '{label}'.")),
                None => self.log("Nothing to redo."),
            },
            "zoomextents" | "ze" => {
                self.zoom_extents_all();
                self.log("Zoomed to extents.");
            }
            "wireframe" => {
                self.display_mode = DisplayMode::Wireframe;
                self.log("Display mode: Wireframe.");
            }
            "shaded" | "shade" => {
                self.display_mode = DisplayMode::Shaded;
                self.log("Display mode: Shaded.");
            }
            "ghosted" => {
                self.display_mode = DisplayMode::Ghosted;
                self.log("Display mode: Ghosted.");
            }
            "new" => {
                self.session.cancel();
                self.doc = Document::new();
                self.log("Created new empty document.");
            }
            "open" => {
                self.open_project_dialog();
            }
            "save" => {
                self.save_project_dialog();
            }
            "import" => {
                self.import_mesh_dialog();
            }
            "export" => {
                self.export_mesh_dialog();
            }
            "help" => {
                self.log("Interactive: Line, Polyline, Circle, Box, Move, Copy, Rotate, Scale | Quick: Sphere [r], Extrude [h], Delete, SelAll, SelNone, Undo, Redo, ZoomExtents (ZE), Wireframe, Shaded, Ghosted, New");
            }
            _ => {
                self.log(format!("Unknown command '{cmd}'. Type 'Help' for available commands."));
            }
        }
    }

    fn world_to_screen(pt: DVec3, m: &[f32; 16], rect: egui::Rect) -> Option<egui::Pos2> {
        let (x, y, z) = (pt.x as f32, pt.y as f32, pt.z as f32);
        let cx = m[0] * x + m[4] * y + m[8] * z + m[12];
        let cy = m[1] * x + m[5] * y + m[9] * z + m[13];
        let cw = m[3] * x + m[7] * y + m[11] * z + m[15];
        if cw <= 1e-4 {
            return None;
        }
        let ndc_x = cx / cw;
        let ndc_y = cy / cw;
        Some(egui::pos2(
            rect.left() + (ndc_x * 0.5 + 0.5) * rect.width(),
            rect.top() + (0.5 - ndc_y * 0.5) * rect.height(),
        ))
    }

    fn dist_point_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
        let ab = b - a;
        let len_sq = ab.length_sq();
        if len_sq < 1e-6 {
            return p.distance(a);
        }
        let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
        p.distance(a + ab * t)
    }

    fn apply_raw_selected_transform(&mut self, mat: &DMat4) {
        for (_, obj) in &mut self.doc.objects {
            if obj.selected {
                obj.geometry.transform(mat);
                obj.refresh_aabb();
            }
        }
        self.doc.rebuild_spatial_index();
        self.scene_dirty = true;
    }

    fn gumball_matrix(axis: usize, mode: GumballMode, origin: DVec3, amount: f64) -> DMat4 {
        let dir = [DVec3::X, DVec3::Y, DVec3::Z][axis];
        match mode {
            GumballMode::Move => DMat4::from_translation(dir * amount),
            GumballMode::Rotate => {
                DMat4::from_translation(origin)
                    * DMat4::from_axis_angle(dir, amount)
                    * DMat4::from_translation(-origin)
            }
        }
    }

    fn draw_viewport(&mut self, ui: &mut egui::Ui, idx: usize, scene: &Arc<RenderScene>) {
        let is_active = self.active_viewport == idx;
        let title = self.cameras[idx].kind.title();

        let frame = egui::Frame::none()
            .fill(egui::Color32::from_rgb(38, 41, 46))
            .stroke(egui::Stroke::new(
                if is_active { 1.5_f32 } else { 1.0_f32 },
                if is_active {
                    egui::Color32::from_rgb(80, 150, 235)
                } else {
                    egui::Color32::from_rgb(65, 68, 74)
                },
            ));

        frame.show(ui, |ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let label_res = ui.selectable_label(is_active, title);
                    if label_res.clicked() {
                        self.active_viewport = idx;
                    }
                    if label_res.double_clicked() {
                        self.maximized_viewport = if self.maximized_viewport == Some(idx) {
                            None
                        } else {
                            Some(idx)
                        };
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.weak(self.display_mode.label());
                    });
                });

                let avail = ui.available_size().max(egui::vec2(16.0, 16.0));
                let (rect, response) = ui.allocate_exact_size(avail, egui::Sense::click_and_drag());

                let aspect = (rect.width() / rect.height().max(1.0)) as f64;
                let cplane = self.cameras[idx].cplane;
                let vp_cols = self.cameras[idx].view_proj_f32(aspect).to_cols_array();

                // Compute Gumball handles if enabled and objects are selected
                struct GumballHandle {
                    axis: usize,
                    p0: egui::Pos2,
                    tip: egui::Pos2,
                    rot_pos: egui::Pos2,
                    px_per_world: f64,
                }
                let mut gumball_origin = DVec3::ZERO;
                let mut gumball_handles: Vec<GumballHandle> = Vec::new();
                if self.gumball_enabled && !self.session.is_active() {
                    if let Some(aabb) = self.doc.selection_aabb() {
                        gumball_origin = aabb.center();
                        if let Some(p0) = Self::world_to_screen(gumball_origin, &vp_cols, rect) {
                            let axes = [DVec3::X, DVec3::Y, DVec3::Z];
                            let mut px_per_unit = [0.0_f64; 3];
                            for (a, dir) in axes.iter().enumerate() {
                                if let Some(p1) =
                                    Self::world_to_screen(gumball_origin + *dir, &vp_cols, rect)
                                {
                                    px_per_unit[a] = (p1 - p0).length() as f64;
                                }
                            }
                            let max_px = px_per_unit
                                .iter()
                                .copied()
                                .fold(0.0_f64, f64::max)
                                .max(1e-4);
                            let world_len = 64.0 / max_px;
                            for (a, dir) in axes.iter().enumerate() {
                                if px_per_unit[a] > 1e-4 {
                                    if let Some(tip) = Self::world_to_screen(
                                        gumball_origin + *dir * world_len,
                                        &vp_cols,
                                        rect,
                                    ) {
                                        let len_px = (tip - p0).length();
                                        if len_px >= 10.0 {
                                            let rot_pos = p0 + (tip - p0) * 0.52;
                                            gumball_handles.push(GumballHandle {
                                                axis: a,
                                                p0,
                                                tip,
                                                rot_pos,
                                                px_per_world: px_per_unit[a],
                                            });
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                let hit_gumball = |pos: egui::Pos2| -> Option<(usize, GumballMode, egui::Vec2, f64)> {
                    for h in &gumball_handles {
                        if pos.distance(h.rot_pos) <= 9.0 {
                            let dir = (h.tip - h.p0).normalized();
                            return Some((h.axis, GumballMode::Rotate, dir, h.px_per_world));
                        }
                    }
                    for h in &gumball_handles {
                        if Self::dist_point_to_segment(pos, h.p0, h.tip) <= 8.0
                            || pos.distance(h.tip) <= 10.0
                        {
                            let dir = (h.tip - h.p0).normalized();
                            return Some((h.axis, GumballMode::Move, dir, h.px_per_world));
                        }
                    }
                    None
                };

                if response.hovered() {
                    self.active_viewport = idx;
                    if self.session.is_active() {
                        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
                    }
                    let scroll = ui.input(|i| i.smooth_scroll_delta.y as f64);
                    if scroll.abs() > 0.0 {
                        self.cameras[idx].zoom(scroll);
                    }
                    if let Some(pos) = response.hover_pos() {
                        let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                        let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                        let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);
                        if let Some(raw_pt) = cplane.intersect_ray(&ray) {
                            let res = self.snap.resolve(
                                raw_pt,
                                self.session.base_point(),
                                &self.doc,
                                &cplane,
                            );
                            self.cursor_world = Some(res.point);
                            self.last_snap_kind = res.kind;
                        } else {
                            self.cursor_world = None;
                            self.last_snap_kind = None;
                        }
                    }
                }

                if response.dragged_by(egui::PointerButton::Secondary) {
                    self.active_viewport = idx;
                    let delta = response.drag_delta();
                    let shift = ui.input(|i| i.modifiers.shift);
                    if self.cameras[idx].is_perspective && !shift {
                        self.cameras[idx].orbit(delta.x as f64, delta.y as f64);
                    } else {
                        self.cameras[idx].pan(delta.x as f64, delta.y as f64, rect.height() as f64);
                    }
                } else if response.clicked_by(egui::PointerButton::Secondary) {
                    if self.session.is_active() {
                        let outcome = self.session.finish_or_cancel(&mut self.doc);
                        for m in outcome.messages {
                            self.log(m);
                        }
                    } else if let Some(prev) = self.last_command.clone() {
                        self.execute_command(&prev);
                    }
                }

                // Primary drag: Gumball or Marquee selection (when session is idle)
                if !self.session.is_active() {
                    if response.drag_started_by(egui::PointerButton::Primary) {
                        self.active_viewport = idx;
                        if let Some(pos) = response.interact_pointer_pos() {
                            if let Some((axis, mode, axis_screen_dir, px_per_world)) =
                                hit_gumball(pos)
                            {
                                self.gumball_drag = Some(GumballDrag {
                                    viewport_idx: idx,
                                    axis,
                                    mode,
                                    origin: gumball_origin,
                                    last_pos: pos,
                                    axis_screen_dir,
                                    px_per_world,
                                    total_amount: 0.0,
                                    moved: false,
                                });
                            } else {
                                self.marquee = Some(MarqueeDrag {
                                    viewport_idx: idx,
                                    start: pos,
                                    current: pos,
                                });
                            }
                        }
                    }

                    if let Some(mut drag) = self.gumball_drag.take() {
                        if drag.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    let delta = pos - drag.last_pos;
                                    drag.last_pos = pos;
                                    let d_amt = match drag.mode {
                                        GumballMode::Move => {
                                            let px = delta.dot(drag.axis_screen_dir) as f64;
                                            px / drag.px_per_world.max(1e-4)
                                        }
                                        GumballMode::Rotate => {
                                            (delta.x - delta.y) as f64 * 0.01
                                        }
                                    };
                                    if d_amt.abs() > 1e-9 {
                                        let mat = Self::gumball_matrix(
                                            drag.axis,
                                            drag.mode,
                                            drag.origin,
                                            d_amt,
                                        );
                                        self.apply_raw_selected_transform(&mat);
                                        drag.total_amount += d_amt;
                                        drag.moved = true;
                                    }
                                }
                                self.gumball_drag = Some(drag);
                            } else {
                                if drag.moved && drag.total_amount.abs() > 1e-9 {
                                    let revert = Self::gumball_matrix(
                                        drag.axis,
                                        drag.mode,
                                        drag.origin,
                                        -drag.total_amount,
                                    );
                                    self.apply_raw_selected_transform(&revert);
                                    let final_mat = Self::gumball_matrix(
                                        drag.axis,
                                        drag.mode,
                                        drag.origin,
                                        drag.total_amount,
                                    );
                                    let label = match drag.mode {
                                        GumballMode::Move => "Gumball Move",
                                        GumballMode::Rotate => "Gumball Rotate",
                                    };
                                    self.doc.transform_selected(label, &final_mat);
                                    self.scene_dirty = true;
                                }
                            }
                        } else {
                            self.gumball_drag = Some(drag);
                        }
                    }

                    if let Some(mut m) = self.marquee.take() {
                        if m.viewport_idx == idx {
                            if response.dragged_by(egui::PointerButton::Primary) {
                                if let Some(pos) = response.interact_pointer_pos() {
                                    m.current = pos;
                                }
                                self.marquee = Some(m);
                            } else {
                                let sel_rect = egui::Rect::from_two_pos(m.start, m.current);
                                if sel_rect.width() > 4.0 || sel_rect.height() > 4.0 {
                                    let crossing = m.current.x < m.start.x;
                                    let shift = ui.input(|i| i.modifiers.shift);
                                    let ctrl = ui.input(|i| i.modifiers.ctrl);
                                    if !shift && !ctrl {
                                        self.doc.clear_selection();
                                    }
                                    let ids: Vec<_> = self.doc.objects.keys().collect();
                                    for id in ids {
                                        let Some(obj) = self.doc.objects.get(id) else {
                                            continue;
                                        };
                                        let Some(layer) = self.doc.layers.get(obj.layer_id) else {
                                            continue;
                                        };
                                        if !layer.visible || layer.locked {
                                            continue;
                                        }
                                        let min = obj.aabb.min;
                                        let max = obj.aabb.max;
                                        let corners = [
                                            DVec3::new(min.x, min.y, min.z),
                                            DVec3::new(max.x, min.y, min.z),
                                            DVec3::new(min.x, max.y, min.z),
                                            DVec3::new(max.x, max.y, min.z),
                                            DVec3::new(min.x, min.y, max.z),
                                            DVec3::new(max.x, min.y, max.z),
                                            DVec3::new(min.x, max.y, max.z),
                                            DVec3::new(max.x, max.y, max.z),
                                        ];
                                        let mut obj_rect = egui::Rect::NOTHING;
                                        for c in corners {
                                            if let Some(sp) =
                                                Self::world_to_screen(c, &vp_cols, rect)
                                            {
                                                obj_rect.extend_with(sp);
                                            }
                                        }
                                        if obj_rect.is_positive() {
                                            let hit = if crossing {
                                                sel_rect.intersects(obj_rect)
                                            } else {
                                                sel_rect.contains_rect(obj_rect)
                                            };
                                            if hit {
                                                self.doc.set_selected(id, !ctrl);
                                            }
                                        }
                                    }
                                    self.scene_dirty = true;
                                }
                            }
                        } else {
                            self.marquee = Some(m);
                        }
                    }
                }

                if response.clicked_by(egui::PointerButton::Primary) {
                    self.active_viewport = idx;
                    if let Some(pos) = response.interact_pointer_pos() {
                        let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                        let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                        let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);

                        if self.session.is_active() {
                            if let Some(raw_pt) = cplane.intersect_ray(&ray) {
                                let res = self.snap.resolve(
                                    raw_pt,
                                    self.session.base_point(),
                                    &self.doc,
                                    &cplane,
                                );
                                let outcome = self.session.on_point(res.point, &mut self.doc, &cplane);
                                for m in outcome.messages {
                                    self.log(m);
                                }
                            }
                        } else if let Some((axis, mode, _, _)) = hit_gumball(pos) {
                            self.gumball_prompt = Some(GumballNumericPrompt {
                                axis,
                                mode,
                                text: String::new(),
                                just_opened: true,
                            });
                        } else {
                            let shift = ui.input(|i| i.modifiers.shift);
                            let ctrl = ui.input(|i| i.modifiers.ctrl);
                            if !shift && !ctrl {
                                self.doc.clear_selection();
                            }
                            if let Some(hit_id) = self.doc.pick_ray(&ray) {
                                if ctrl {
                                    let cur = self
                                        .doc
                                        .objects
                                        .get(hit_id)
                                        .map(|o| o.selected)
                                        .unwrap_or(false);
                                    self.doc.set_selected(hit_id, !cur);
                                } else {
                                    self.doc.set_selected(hit_id, true);
                                }
                            }
                            self.scene_dirty = true;
                        }
                    }
                }

                let cb = eframe::egui_wgpu::Callback::new_paint_callback(
                    rect,
                    ViewportPaintCallback {
                        kind: self.cameras[idx].kind,
                        view_proj: self.cameras[idx].view_proj_f32(aspect),
                        scene: Arc::clone(scene),
                    },
                );
                ui.painter().add(cb);

                // Draw Gumball overlay
                let painter = ui.painter_at(rect);
                if !gumball_handles.is_empty() {
                    let axis_colors = [
                        egui::Color32::from_rgb(235, 75, 75),
                        egui::Color32::from_rgb(80, 210, 95),
                        egui::Color32::from_rgb(75, 145, 245),
                    ];
                    for h in &gumball_handles {
                        let col = axis_colors[h.axis];
                        painter.line_segment([h.p0, h.tip], egui::Stroke::new(2.2_f32, col));
                        let dir = (h.tip - h.p0).normalized();
                        let perp = egui::vec2(-dir.y, dir.x);
                        let base = h.tip - dir * 10.0;
                        painter.add(egui::Shape::convex_polygon(
                            vec![h.tip, base + perp * 4.5, base - perp * 4.5],
                            col,
                            egui::Stroke::NONE,
                        ));
                        painter.circle_stroke(h.rot_pos, 5.0, egui::Stroke::new(2.0_f32, col));
                    }
                    painter.circle_filled(
                        gumball_handles[0].p0,
                        3.5,
                        egui::Color32::from_rgb(235, 235, 240),
                    );
                }

                // Draw Marquee selection rectangle
                if let Some(m) = &self.marquee {
                    if m.viewport_idx == idx {
                        let sel_rect = egui::Rect::from_two_pos(m.start, m.current);
                        let crossing = m.current.x < m.start.x;
                        let (fill, stroke_col) = if crossing {
                            (
                                egui::Color32::from_rgba_unmultiplied(80, 200, 110, 35),
                                egui::Color32::from_rgb(90, 220, 120),
                            )
                        } else {
                            (
                                egui::Color32::from_rgba_unmultiplied(70, 145, 240, 35),
                                egui::Color32::from_rgb(90, 165, 250),
                            )
                        };
                        painter.rect(sel_rect, 0.0, fill, egui::Stroke::new(1.5_f32, stroke_col));
                    }
                }
            });
        });
    }
}

impl eframe::App for ElephantApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if self.gumball_prompt.is_some() {
                self.gumball_prompt = None;
            } else if self.session.is_active() {
                let msg = self.session.cancel();
                self.log(msg);
            } else {
                self.doc.clear_selection();
                self.scene_dirty = true;
            }
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z)) {
            self.execute_command("Undo");
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Y)) {
            self.execute_command("Redo");
        }
        if !ctx.wants_keyboard_input() && ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::A)) {
            self.execute_command("SelAll");
        }
        if !ctx.wants_keyboard_input() && ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.execute_command("Delete");
        }

        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New").clicked() {
                        self.execute_command("New");
                        ui.close_menu();
                    }
                    if ui.button("Open (.elephant / .3dm)...").clicked() {
                        self.open_project_dialog();
                        ui.close_menu();
                    }
                    if ui.button("Save As (.elephant)...").clicked() {
                        self.save_project_dialog();
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Import (.3dm / .obj / .stl)...").clicked() {
                        self.import_mesh_dialog();
                        ui.close_menu();
                    }
                    if ui.button("Export (.obj / .stl)...").clicked() {
                        self.export_mesh_dialog();
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
                ui.menu_button("Edit", |ui| {
                    if ui
                        .add_enabled(self.doc.can_undo(), egui::Button::new("Undo (Ctrl+Z)"))
                        .clicked()
                    {
                        self.execute_command("Undo");
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(self.doc.can_redo(), egui::Button::new("Redo (Ctrl+Y)"))
                        .clicked()
                    {
                        self.execute_command("Redo");
                        ui.close_menu();
                    }
                    if ui.button("Delete").clicked() {
                        self.execute_command("Delete");
                        ui.close_menu();
                    }
                });
                ui.menu_button("Create", |ui| {
                    for cmd in [
                        "Line",
                        "Polyline",
                        "Rectangle",
                        "Arc",
                        "Circle",
                        "Polygon",
                        "Ellipse",
                        "Box",
                        "Sphere",
                        "Extrude",
                    ] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Curve", |ui| {
                    for cmd in ["Join", "Explode", "Offset", "Fillet", "Distance", "Length", "Area"] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Transform", |ui| {
                    for cmd in [
                        "Move",
                        "Copy",
                        "Rotate",
                        "Scale",
                        "Mirror",
                        "ArrayLinear",
                        "ArrayPolar",
                    ] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("View", |ui| {
                    if ui.button("Wireframe").clicked() {
                        self.display_mode = DisplayMode::Wireframe;
                        self.scene_dirty = true;
                        ui.close_menu();
                    }
                    if ui.button("Shaded").clicked() {
                        self.display_mode = DisplayMode::Shaded;
                        self.scene_dirty = true;
                        ui.close_menu();
                    }
                    if ui.button("Ghosted").clicked() {
                        self.display_mode = DisplayMode::Ghosted;
                        self.scene_dirty = true;
                        ui.close_menu();
                    }
                    ui.separator();
                    if ui.button("Zoom Extents").clicked() {
                        self.zoom_extents_all();
                        ui.close_menu();
                    }
                    if ui.button("4 Viewports").clicked() {
                        self.maximized_viewport = None;
                        ui.close_menu();
                    }
                });
                ui.menu_button("Tools", |ui| {
                    if ui.button("Parametric Graph (Grasshopper-style)").clicked() {
                        self.tool_tab = if self.tool_tab == ToolPanelTab::Parametric {
                            ToolPanelTab::None
                        } else {
                            ToolPanelTab::Parametric
                        };
                        ui.close_menu();
                    }
                    if ui.button("Rhai Script Editor").clicked() {
                        self.tool_tab = if self.tool_tab == ToolPanelTab::Script {
                            ToolPanelTab::None
                        } else {
                            ToolPanelTab::Script
                        };
                        ui.close_menu();
                    }
                });
            });

            ui.separator();

            let start = self.command_log.len().saturating_sub(2);
            for line in &self.command_log[start..] {
                ui.small(line);
            }
            ui.horizontal(|ui| {
                ui.strong(format!("{}:", self.session.prompt()));
                let edit = ui.add(
                    egui::TextEdit::singleline(&mut self.command_input)
                        .desired_width(380.0)
                        .hint_text("Command, x,y,z, @dx,dy,dz, or distance"),
                );
                if edit.has_focus() && !self.command_history.is_empty() {
                    if ui.input(|i| i.key_pressed(egui::Key::ArrowUp)) {
                        let next = match self.history_cursor {
                            None => self.command_history.len() - 1,
                            Some(c) => c.saturating_sub(1),
                        };
                        self.history_cursor = Some(next);
                        self.command_input = self.command_history[next].clone();
                    } else if ui.input(|i| i.key_pressed(egui::Key::ArrowDown)) {
                        if let Some(c) = self.history_cursor {
                            if c + 1 < self.command_history.len() {
                                self.history_cursor = Some(c + 1);
                                self.command_input = self.command_history[c + 1].clone();
                            } else {
                                self.history_cursor = None;
                                self.command_input.clear();
                            }
                        }
                    } else if ui.input(|i| i.key_pressed(egui::Key::Tab)) && !self.session.is_active() {
                        let typed = self.command_input.trim().to_ascii_lowercase();
                        if !typed.is_empty() {
                            if let Some(&cand) = KNOWN_COMMANDS
                                .iter()
                                .find(|c| c.to_ascii_lowercase().starts_with(&typed))
                            {
                                self.command_input = cand.to_string();
                            }
                        }
                    }
                }
                if edit.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    let cmd = std::mem::take(&mut self.command_input);
                    self.execute_command(&cmd);
                    edit.request_focus();
                }
                if ui.button("Run").clicked() {
                    let cmd = std::mem::take(&mut self.command_input);
                    self.execute_command(&cmd);
                }
                if self.session.is_active() && ui.small_button("Cancel (Esc)").clicked() {
                    let msg = self.session.cancel();
                    self.log(msg);
                }
                ui.separator();
                for quick in ["Line", "Circle", "Box", "Move", "ZoomExtents"] {
                    if ui.small_button(quick).clicked() {
                        self.execute_command(quick);
                    }
                }
            });

            let typed = self.command_input.trim().to_ascii_lowercase();
            if !typed.is_empty() && !self.session.is_active() {
                ui.horizontal(|ui| {
                    ui.weak("Suggestions:");
                    for &cand in KNOWN_COMMANDS {
                        if cand.to_ascii_lowercase().starts_with(&typed)
                            && ui.small_button(cand).clicked()
                        {
                            self.command_input = cand.to_string();
                        }
                    }
                });
            }
        });

        egui::SidePanel::right("inspector")
            .default_width(250.0)
            .show(ctx, |ui| {
                ui.heading("Layers");
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.new_layer_name)
                            .desired_width(140.0)
                            .hint_text("New layer..."),
                    );
                    if ui.button("+").clicked() && !self.new_layer_name.trim().is_empty() {
                        let name = std::mem::take(&mut self.new_layer_name);
                        self.doc.add_layer(name.trim(), [160, 185, 220, 255]);
                    }
                });

                let layer_ids: Vec<_> = self.doc.layers.keys().collect();
                for lid in layer_ids {
                    let is_active = self.doc.active_layer == lid;
                    if let Some(layer) = self.doc.layers.get_mut(lid) {
                        ui.horizontal(|ui| {
                            let (swatch_rect, _) = ui.allocate_exact_size(
                                egui::vec2(12.0, 12.0),
                                egui::Sense::hover(),
                            );
                            ui.painter().rect_filled(
                                swatch_rect,
                                2.0,
                                egui::Color32::from_rgb(
                                    layer.color[0],
                                    layer.color[1],
                                    layer.color[2],
                                ),
                            );
                            if ui.selectable_label(is_active, &layer.name).clicked() {
                                self.doc.active_layer = lid;
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    ui.checkbox(&mut layer.locked, "Lock");
                                    if ui.checkbox(&mut layer.visible, "Vis").changed() {
                                        self.scene_dirty = true;
                                    }
                                },
                            );
                        });
                    }
                }

                ui.separator();
                ui.heading("Properties");
                let selected: Vec<_> = self
                    .doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected)
                    .map(|(_, o)| o)
                    .collect();
                if selected.is_empty() {
                    ui.weak("No object selected.");
                    ui.label(format!("Total objects: {}", self.doc.objects.len()));
                } else if selected.len() == 1 {
                    let obj = selected[0];
                    ui.label(format!(
                        "Name: {}",
                        obj.name.as_deref().unwrap_or("(unnamed)")
                    ));
                    let kind = match &obj.geometry {
                        Geometry::Point(_) => "Point",
                        Geometry::Curve(_) => "NURBS Curve",
                        Geometry::Surface(_) => "NURBS Surface",
                        Geometry::Mesh(_) => "Mesh",
                    };
                    ui.label(format!("Type: {kind}"));
                    let c = obj.aabb.center();
                    let s = obj.aabb.size();
                    ui.label(format!("Center: ({:.2}, {:.2}, {:.2})", c.x, c.y, c.z));
                    ui.label(format!("Size: ({:.2}, {:.2}, {:.2})", s.x, s.y, s.z));
                } else {
                    ui.label(format!("{} objects selected", selected.len()));
                }
            });

        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "Viewport: {}",
                    self.cameras[self.active_viewport].kind.title()
                ));
                ui.separator();
                if let Some(p) = self.cursor_world {
                    let snap_tag = self
                        .last_snap_kind
                        .map(|k| format!(" [{}]", k.label()))
                        .unwrap_or_default();
                    ui.label(format!(
                        "CPlane: {:.2}, {:.2}, {:.2}{}",
                        p.x, p.y, p.z, snap_tag
                    ));
                } else {
                    ui.label("CPlane: --");
                }
                ui.separator();
                ui.checkbox(&mut self.gumball_enabled, "Gumball");
                ui.checkbox(&mut self.snap.grid_snap, "Grid Snap");
                ui.checkbox(&mut self.snap.ortho, "Ortho");
                ui.checkbox(&mut self.snap.osnap_enabled, "OSnap");
                if self.snap.osnap_enabled {
                    ui.checkbox(&mut self.snap.osnap_end, "End");
                    ui.checkbox(&mut self.snap.osnap_mid, "Mid");
                    ui.checkbox(&mut self.snap.osnap_center, "Cen");
                    ui.checkbox(&mut self.snap.osnap_quad, "Quad");
                    ui.checkbox(&mut self.snap.osnap_near, "Near");
                }
                ui.separator();
                let sel_count = self.doc.objects.values().filter(|o| o.selected).count();
                ui.label(format!(
                    "Obj: {} | Sel: {}",
                    self.doc.objects.len(),
                    sel_count
                ));
            });
        });

        if self.tool_tab != ToolPanelTab::None {
            egui::SidePanel::left("tools_panel")
                .default_width(310.0)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(
                            &mut self.tool_tab,
                            ToolPanelTab::Parametric,
                            "Parametric Graph",
                        );
                        ui.selectable_value(
                            &mut self.tool_tab,
                            ToolPanelTab::Script,
                            "Rhai Script",
                        );
                        if ui.small_button("X").clicked() {
                            self.tool_tab = ToolPanelTab::None;
                        }
                    });
                    ui.separator();

                    match self.tool_tab {
                        ToolPanelTab::Parametric => {
                            ui.checkbox(&mut self.parametric_graph.live_preview, "Live 3D Preview");
                            if ui.button("Bake to Document").clicked() {
                                let n = self.parametric_graph.bake_into(&mut self.doc);
                                self.scene_dirty = true;
                                self.log(format!("Baked {n} parametric object(s)."));
                            }
                            ui.separator();
                            egui::ScrollArea::vertical().show(ui, |ui| {
                                for node in &mut self.parametric_graph.nodes {
                                    ui.group(|ui| {
                                        ui.strong(&node.label);
                                        match &mut node.kind {
                                            NodeKind::NumberSlider { value, min, max } => {
                                                ui.add(
                                                    egui::Slider::new(value, *min..=*max)
                                                        .text("Value"),
                                                );
                                            }
                                            NodeKind::LinearArray { step_x, step_y, .. } => {
                                                ui.add(
                                                    egui::Slider::new(step_x, 0.5..=12.0)
                                                        .text("Step X"),
                                                );
                                                ui.add(
                                                    egui::Slider::new(step_y, -12.0..=12.0)
                                                        .text("Step Y"),
                                                );
                                            }
                                            _ => {
                                                ui.weak("Connected node");
                                            }
                                        }
                                    });
                                }
                            });
                        }
                        ToolPanelTab::Script => {
                            if ui.button("Run Script").clicked() {
                                match run_rhai_script(&self.script_source, &mut self.doc) {
                                    Ok(res) => {
                                        self.scene_dirty = true;
                                        for l in res.logs {
                                            self.log(format!("[Script] {l}"));
                                        }
                                        self.log(format!(
                                            "Script created {} object(s).",
                                            res.created_count
                                        ));
                                    }
                                    Err(e) => self.log(format!("Script error: {e}")),
                                }
                            }
                            ui.separator();
                            egui::ScrollArea::vertical().show(ui, |ui| {
                                ui.add(
                                    egui::TextEdit::multiline(&mut self.script_source)
                                        .code_editor()
                                        .desired_rows(18)
                                        .desired_width(f32::INFINITY),
                                );
                            });
                        }
                        ToolPanelTab::None => {}
                    }
                });
        }

        let cplane = self.cameras[self.active_viewport].cplane;
        let has_dynamic_preview = self.session.is_active() || self.parametric_graph.live_preview;
        if self.scene_dirty || has_dynamic_preview || self.cached_scene.is_none() {
            self.scene_version = self.scene_version.wrapping_add(1).max(1);
            let mut built = RenderScene::build(&self.doc, self.display_mode, &cplane, self.scene_version);
            if self.parametric_graph.live_preview {
                let param_geoms = self.parametric_graph.evaluate();
                built.push_preview(&param_geoms, None);
            }
            if self.session.is_active() {
                let previews = self.session.preview_geometry(self.cursor_world, &cplane);
                let snap_marker = if self.last_snap_kind.is_some() {
                    self.cursor_world
                } else {
                    None
                };
                built.push_preview(&previews, snap_marker);
            }
            self.cached_scene = Some(Arc::new(built));
            self.scene_dirty = false;
        }
        let scene = Arc::clone(self.cached_scene.as_ref().unwrap());

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(egui::Color32::from_rgb(28, 30, 34)))
            .show(ctx, |ui| {
                if let Some(max_idx) = self.maximized_viewport {
                    self.draw_viewport(ui, max_idx, &scene);
                } else {
                    ui.spacing_mut().item_spacing = egui::vec2(2.0, 2.0);
                    let avail = ui.available_size();
                    let half_w = (avail.x - 2.0).max(32.0) * 0.5;
                    let half_h = (avail.y - 2.0).max(32.0) * 0.5;

                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.allocate_ui_with_layout(
                                egui::vec2(half_w, half_h),
                                egui::Layout::top_down(egui::Align::Min),
                                |ui| self.draw_viewport(ui, 0, &scene),
                            );
                            ui.allocate_ui_with_layout(
                                egui::vec2(half_w, half_h),
                                egui::Layout::top_down(egui::Align::Min),
                                |ui| self.draw_viewport(ui, 1, &scene),
                            );
                        });
                        ui.horizontal(|ui| {
                            ui.allocate_ui_with_layout(
                                egui::vec2(half_w, half_h),
                                egui::Layout::top_down(egui::Align::Min),
                                |ui| self.draw_viewport(ui, 2, &scene),
                            );
                            ui.allocate_ui_with_layout(
                                egui::vec2(half_w, half_h),
                                egui::Layout::top_down(egui::Align::Min),
                                |ui| self.draw_viewport(ui, 3, &scene),
                            );
                        });
                    });
                }
            });

        if let Some(mut prompt) = self.gumball_prompt.take() {
            let axis_name = ["X", "Y", "Z"][prompt.axis];
            let title = match prompt.mode {
                GumballMode::Move => format!("Gumball Move {axis_name}"),
                GumballMode::Rotate => format!("Gumball Rotate {axis_name} (deg)"),
            };
            let mut keep = true;
            let mut commit_val: Option<f64> = None;
            egui::Window::new(title)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.horizontal(|ui| {
                        let res = ui.add(
                            egui::TextEdit::singleline(&mut prompt.text)
                                .desired_width(120.0)
                                .hint_text("Enter value..."),
                        );
                        if prompt.just_opened {
                            res.request_focus();
                            prompt.just_opened = false;
                        }
                        if (res.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                            || ui.button("Apply").clicked()
                        {
                            if let Ok(v) = prompt.text.trim().parse::<f64>() {
                                commit_val = Some(v);
                            }
                            keep = false;
                        }
                        if ui.button("Cancel").clicked() {
                            keep = false;
                        }
                    });
                });
            if let Some(v) = commit_val {
                if let Some(aabb) = self.doc.selection_aabb() {
                    let origin = aabb.center();
                    let amount = match prompt.mode {
                        GumballMode::Move => v,
                        GumballMode::Rotate => v.to_radians(),
                    };
                    let mat = Self::gumball_matrix(prompt.axis, prompt.mode, origin, amount);
                    let label = match prompt.mode {
                        GumballMode::Move => "Gumball Move",
                        GumballMode::Rotate => "Gumball Rotate",
                    };
                    let n = self.doc.transform_selected(label, &mat);
                    self.scene_dirty = true;
                    self.log(format!("{label} {axis_name} by {v} on {n} object(s)."));
                }
            } else if keep {
                self.gumball_prompt = Some(prompt);
            }
        }
    }
}
