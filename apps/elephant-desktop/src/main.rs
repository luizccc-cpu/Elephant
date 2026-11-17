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
    "Circle",
    "Box",
    "Sphere",
    "Extrude",
    "Move",
    "Copy",
    "Rotate",
    "Scale",
    "Delete",
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

struct ElephantApp {
    doc: Document,
    cameras: [ViewportCamera; 4],
    active_viewport: usize,
    maximized_viewport: Option<usize>,
    display_mode: DisplayMode,
    session: CommandSession,
    snap: SnapConfig,
    command_input: String,
    command_log: Vec<String>,
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
            command_input: String::new(),
            command_log: vec![
                "Elephant CAD initialized (local clean-room engine). Type 'Help' for commands."
                    .to_string(),
            ],
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
            return;
        }
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
                self.log("Interactive: Line, Polyline, Circle, Box, Move, Copy, Rotate, Scale | Quick: Sphere [r], Extrude [h], Delete, Undo, Redo, ZoomExtents (ZE), Wireframe, Shaded, Ghosted, New");
            }
            _ => {
                self.log(format!("Unknown command '{cmd}'. Type 'Help' for available commands."));
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
                } else if response.clicked_by(egui::PointerButton::Secondary)
                    && self.session.is_active()
                {
                    let outcome = self.session.finish_or_cancel(&mut self.doc);
                    for m in outcome.messages {
                        self.log(m);
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
                        } else {
                            let shift = ui.input(|i| i.modifiers.shift);
                            if !shift {
                                self.doc.clear_selection();
                            }
                            if let Some(hit_id) = self.doc.pick_ray(&ray) {
                                self.doc.set_selected(hit_id, true);
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
            });
        });
    }
}

impl eframe::App for ElephantApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) && self.session.is_active() {
            let msg = self.session.cancel();
            self.log(msg);
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z)) {
            self.execute_command("Undo");
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Y)) {
            self.execute_command("Redo");
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
                    for cmd in ["Line", "Polyline", "Circle", "Box", "Sphere", "Extrude"] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Transform", |ui| {
                    for cmd in ["Move", "Copy", "Rotate", "Scale"] {
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
                ui.checkbox(&mut self.snap.grid_snap, "Grid Snap");
                ui.checkbox(&mut self.snap.ortho, "Ortho");
                ui.checkbox(&mut self.snap.osnap_enabled, "OSnap");
                if self.snap.osnap_enabled {
                    ui.checkbox(&mut self.snap.osnap_end, "End");
                    ui.checkbox(&mut self.snap.osnap_mid, "Mid");
                    ui.checkbox(&mut self.snap.osnap_center, "Cen");
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
    }
}
