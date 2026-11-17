use std::sync::Arc;

use cad_doc::Document;
use cad_geom::{DMat4, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh};
use cad_render::{
    CadWgpuState, DisplayMode, RenderScene, ViewportCamera, ViewportKind, ViewportPaintCallback,
};
use eframe::egui;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Elephant CAD")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 540.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };

    eframe::run_native(
        "Elephant CAD",
        options,
        Box::new(|cc| {
            if let Some(wgpu_state) = &cc.wgpu_render_state {
                let cad_state = CadWgpuState::new(&wgpu_state.device, wgpu_state.target_format);
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
    "Box",
    "Sphere",
    "Circle",
    "Line",
    "Extrude",
    "Move",
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

struct ElephantApp {
    doc: Document,
    cameras: [ViewportCamera; 4],
    active_viewport: usize,
    maximized_viewport: Option<usize>,
    display_mode: DisplayMode,
    command_input: String,
    command_log: Vec<String>,
    new_layer_name: String,
    cursor_world: Option<DVec3>,
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
            command_input: String::new(),
            command_log: vec![
                "Elephant CAD initialized (local clean-room engine). Type 'Help' for commands."
                    .to_string(),
            ],
            new_layer_name: String::new(),
            cursor_world: None,
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

    fn execute_command(&mut self, raw: &str) {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return;
        }
        self.log(format!("> {trimmed}"));
        let mut parts = trimmed.split_whitespace();
        let cmd = parts.next().unwrap_or("").to_ascii_lowercase();
        let args: Vec<&str> = parts.collect();

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
                self.doc = Document::new();
                self.log("Created new empty document.");
            }
            "help" => {
                self.log("Commands: Box [s], Sphere [r], Circle [r], Line [len], Extrude [h], Move dx dy dz, Scale s, Delete, Undo, Redo, ZoomExtents (ZE), Wireframe, Shaded, Ghosted, New");
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

            if response.hovered() {
                self.active_viewport = idx;
                let scroll = ui.input(|i| i.smooth_scroll_delta.y as f64);
                if scroll.abs() > 0.0 {
                    self.cameras[idx].zoom(scroll);
                }
                if let Some(pos) = response.hover_pos() {
                    let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                    let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                    let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);
                    self.cursor_world = self.cameras[idx].cplane.intersect_ray(&ray);
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
            }

            if response.clicked_by(egui::PointerButton::Primary) {
                self.active_viewport = idx;
                if let Some(pos) = response.interact_pointer_pos() {
                    let ndc_x = ((pos.x - rect.left()) / rect.width().max(1.0)) * 2.0 - 1.0;
                    let ndc_y = 1.0 - ((pos.y - rect.top()) / rect.height().max(1.0)) * 2.0;
                    let ray = self.cameras[idx].screen_to_ray(ndc_x as f64, ndc_y as f64, aspect);
                    let shift = ui.input(|i| i.modifiers.shift);
                    if !shift {
                        self.doc.clear_selection();
                    }
                    if let Some(hit_id) = self.doc.pick_ray(&ray) {
                        self.doc.set_selected(hit_id, true);
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
    }
}

impl eframe::App for ElephantApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Keyboard shortcuts
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Z)) {
            self.execute_command("Undo");
        }
        if ctx.input(|i| i.modifiers.ctrl && i.key_pressed(egui::Key::Y)) {
            self.execute_command("Redo");
        }
        if !ctx.wants_keyboard_input() && ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.execute_command("Delete");
        }

        // Top Menu + Toolbar + Command Prompt
        egui::TopBottomPanel::top("top_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New").clicked() {
                        self.execute_command("New");
                        ui.close_menu();
                    }
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
                ui.menu_button("Edit", |ui| {
                    if ui.add_enabled(self.doc.can_undo(), egui::Button::new("Undo (Ctrl+Z)")).clicked() {
                        self.execute_command("Undo");
                        ui.close_menu();
                    }
                    if ui.add_enabled(self.doc.can_redo(), egui::Button::new("Redo (Ctrl+Y)")).clicked() {
                        self.execute_command("Redo");
                        ui.close_menu();
                    }
                    if ui.button("Delete").clicked() {
                        self.execute_command("Delete");
                        ui.close_menu();
                    }
                });
                ui.menu_button("Create", |ui| {
                    for cmd in ["Box", "Sphere", "Circle", "Line", "Extrude"] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("View", |ui| {
                    if ui.button("Wireframe").clicked() {
                        self.display_mode = DisplayMode::Wireframe;
                        ui.close_menu();
                    }
                    if ui.button("Shaded").clicked() {
                        self.display_mode = DisplayMode::Shaded;
                        ui.close_menu();
                    }
                    if ui.button("Ghosted").clicked() {
                        self.display_mode = DisplayMode::Ghosted;
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
            });

            ui.separator();

            // Command log preview (last 2 lines) + Command prompt
            let start = self.command_log.len().saturating_sub(2);
            for line in &self.command_log[start..] {
                ui.small(line);
            }
            ui.horizontal(|ui| {
                ui.strong("Command:");
                let edit = ui.add(
                    egui::TextEdit::singleline(&mut self.command_input)
                        .desired_width(420.0)
                        .hint_text("Type a command (Box, Sphere, Circle, Move 1 0 0, Undo...)"),
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
                ui.separator();
                for quick in ["Box", "Sphere", "Circle", "ZoomExtents"] {
                    if ui.small_button(quick).clicked() {
                        self.execute_command(quick);
                    }
                }
            });

            let typed = self.command_input.trim().to_ascii_lowercase();
            if !typed.is_empty() {
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

        // Right Inspector Panel: Layers + Properties
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
                            if ui.selectable_label(is_active, &layer.name).clicked() {
                                self.doc.active_layer = lid;
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    ui.checkbox(&mut layer.locked, "Lock");
                                    ui.checkbox(&mut layer.visible, "Vis");
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

        // Bottom Status Bar
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "Viewport: {}",
                    self.cameras[self.active_viewport].kind.title()
                ));
                ui.separator();
                if let Some(p) = self.cursor_world {
                    ui.label(format!("CPlane: {:.2}, {:.2}, {:.2}", p.x, p.y, p.z));
                } else {
                    ui.label("CPlane: --");
                }
                ui.separator();
                let sel_count = self.doc.objects.values().filter(|o| o.selected).count();
                ui.label(format!(
                    "Objects: {} | Selected: {} | Mode: {}",
                    self.doc.objects.len(),
                    sel_count,
                    self.display_mode.label()
                ));
            });
        });

        // Central 4-Viewport Grid (or Maximized Single Viewport)
        let scene = Arc::new(RenderScene::build(
            &self.doc,
            self.display_mode,
            &self.cameras[self.active_viewport].cplane,
        ));

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(egui::Color32::from_rgb(28, 30, 34)))
            .show(ctx, |ui| {
                if let Some(max_idx) = self.maximized_viewport {
                    self.draw_viewport(ui, max_idx, &scene);
                } else {
                    let avail = ui.available_size();
                    let half_w = (avail.x - 4.0).max(32.0) * 0.5;
                    let half_h = (avail.y - 4.0).max(32.0) * 0.5;

                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.allocate_ui(egui::vec2(half_w, half_h), |ui| {
                                self.draw_viewport(ui, 0, &scene);
                            });
                            ui.allocate_ui(egui::vec2(half_w, half_h), |ui| {
                                self.draw_viewport(ui, 1, &scene);
                            });
                        });
                        ui.horizontal(|ui| {
                            ui.allocate_ui(egui::vec2(half_w, half_h), |ui| {
                                self.draw_viewport(ui, 2, &scene);
                            });
                            ui.allocate_ui(egui::vec2(half_w, half_h), |ui| {
                                self.draw_viewport(ui, 3, &scene);
                            });
                        });
                    });
                }
            });
    }
}
