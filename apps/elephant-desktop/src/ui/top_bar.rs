use cad_commands::{AxisLockMode, CommandOptionKind};
use cad_render::DisplayMode;
use eframe::egui;

use crate::state::{ElephantApp, KNOWN_COMMANDS, NamedView, ToolPanelTab};

impl ElephantApp {
    pub fn draw_top_bar(&mut self, ctx: &egui::Context) {
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
                    if ui.button("Import (.3dm / .dxf / .obj / .stl)...").clicked() {
                        self.import_mesh_dialog();
                        ui.close_menu();
                    }
                    if ui.button("Export (.obj / .stl / .glb / .dxf / .svg)...").clicked() {
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
                    ui.separator();
                    for cmd in ["Hide", "Show", "Isolate", "Group", "Ungroup"] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                    ui.menu_button("Select", |ui| {
                        for cmd in [
                            "SelAll", "SelNone", "Invert", "SelLast", "SelCrv", "SelSrf",
                            "SelMesh", "SelPt",
                        ] {
                            if ui.button(cmd).clicked() {
                                self.execute_command(cmd);
                                ui.close_menu();
                            }
                        }
                    });
                });
                ui.menu_button("Create", |ui| {
                    for cmd in [
                        "Line",
                        "Polyline",
                        "Curve",
                        "InterpCrv",
                        "HandleCurve",
                        "Conic",
                        "Helix",
                        "Rectangle",
                        "Arc",
                        "Circle",
                        "Polygon",
                        "Ellipse",
                        "Plane",
                        "Box",
                        "Cylinder",
                        "Cone",
                        "Torus",
                        "Sphere",
                    ] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Surface", |ui| {
                    for cmd in [
                        "Plane",
                        "Extrude",
                        "PlanarSrf",
                        "Loft",
                        "Revolve",
                        "RailRevolve",
                        "Sweep1",
                        "Sweep2",
                        "EdgeSrf",
                        "NetworkSrf",
                        "BlendSrf",
                        "Patch",
                        "OffsetSrf",
                        "ExtendSrf",
                        "MatchSrf",
                        "FilletSrf",
                        "Intersect",
                        "RebuildSrf",
                        "SplitSrf",
                        "Untrim",
                        "ExtractSrf",
                        "ToBRep",
                        "ShowEdges",
                        "ExtractIsocurve",
                        "DupBorder",
                        "DupEdge",
                        "Flip",
                        "ChangeDegree",
                        "PointsOn",
                        "PointsOff",
                    ] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Solid", |ui| {
                    for cmd in [
                        "Box",
                        "Cylinder",
                        "Cone",
                        "Sphere",
                        "Torus",
                        "Extrude",
                        "PlanarSrf",
                        "Loft",
                        "Revolve",
                        "BooleanUnion",
                        "BooleanDifference",
                        "BooleanIntersection",
                        "BooleanSplit",
                        "Cap",
                        "FilletEdge",
                        "ChamferEdge",
                        "Shell",
                        "PushPull",
                        "Inset",
                        "ShowEdges",
                        "PointsOn",
                        "PointsOff",
                    ] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Curve", |ui| {
                    for cmd in [
                        "Join",
                        "Explode",
                        "Offset",
                        "Split",
                        "Trim",
                        "Fillet",
                        "Chamfer",
                        "BlendCrv",
                        "Match",
                        "Rebuild",
                        "RebuildAdaptive",
                        "Fair",
                        "TweenCurves",
                        "InsertKnot",
                        "ChangeDegree",
                        "CurvatureGraph",
                        "CurvatureGraphOff",
                        "Distance",
                        "Length",
                        "Area",
                    ] {
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
                ui.menu_button("Annotate", |ui| {
                    for cmd in ["Dim", "TextDot", "Text", "GdtFrame", "DatumTag"] {
                        if ui.button(cmd).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("Analyze", |ui| {
                    for (lbl, cmd) in [
                        ("Zebra Analysis", "Zebra"),
                        ("Environment Map (E-Map)", "EMap"),
                        ("Gaussian Curvature", "GaussianCurvature"),
                        ("Mean Curvature", "MeanCurvature"),
                        ("Draft Angle", "DraftAngle"),
                        ("Turn Analysis Off", "AnalysisOff"),
                    ] {
                        if ui.button(lbl).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                    ui.separator();
                    for (lbl, cmd) in [
                        ("Global Edge Continuity", "GlobalEdgeContinuity"),
                        ("Show Naked Edges", "ShowEdges"),
                        ("Mass Properties", "MassProperties"),
                    ] {
                        if ui.button(lbl).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                    ui.separator();
                    for (lbl, cmd) in [
                        ("Add Clipping Plane", "ClippingPlane"),
                        ("Bake Section Cut", "Section"),
                    ] {
                        if ui.button(lbl).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                    ui.separator();
                    for (lbl, cmd) in [
                        ("Make2D (Active View)", "Make2D"),
                        ("New 4-View Layout", "Layout"),
                        ("Export Vector PDF", "ExportPDF"),
                    ] {
                        if ui.button(lbl).clicked() {
                            self.execute_command(cmd);
                            ui.close_menu();
                        }
                    }
                });
                ui.menu_button("View", |ui| {
                    let active_mode = self.active_display_mode();
                    for mode in DisplayMode::ALL {
                        if ui.selectable_label(active_mode == mode, mode.label()).clicked() {
                            self.set_active_viewport_display_mode(mode);
                            ui.close_menu();
                        }
                    }
                    ui.separator();
                    ui.menu_button("Apply Mode to All Viewports", |ui| {
                        for mode in DisplayMode::ALL {
                            if ui.button(mode.label()).clicked() {
                                self.set_all_viewports_display_mode(mode);
                                ui.close_menu();
                            }
                        }
                    });
                    ui.separator();
                    ui.checkbox(&mut self.show_left_palette, "Left Tool Palette");
                    if ui.button("Radial Quick Menu (Q)").clicked() {
                        self.radial_menu_pos = Some(ui.ctx().screen_rect().center());
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
                    ui.separator();
                    ui.menu_button("Named Views", |ui| {
                        ui.horizontal(|ui| {
                            ui.add(
                                egui::TextEdit::singleline(&mut self.new_view_name)
                                    .desired_width(110.0)
                                    .hint_text("View name..."),
                            );
                            if ui.button("Save").clicked() {
                                let name = if self.new_view_name.trim().is_empty() {
                                    format!("View {}", self.named_views.len() + 1)
                                } else {
                                    std::mem::take(&mut self.new_view_name).trim().to_string()
                                };
                                self.named_views.push(NamedView {
                                    name: name.clone(),
                                    cameras: self.cameras.clone(),
                                });
                                self.log(format!("Saved Named View '{name}'."));
                            }
                        });
                        if !self.named_views.is_empty() {
                            ui.separator();
                            let mut restore_idx = None;
                            for (vi, nv) in self.named_views.iter().enumerate() {
                                if ui.button(&nv.name).clicked() {
                                    restore_idx = Some(vi);
                                }
                            }
                            if let Some(vi) = restore_idx {
                                self.cameras = self.named_views[vi].cameras.clone();
                                self.mark_scene_dirty();
                                self.log(format!(
                                    "Restored Named View '{}'.",
                                    self.named_views[vi].name
                                ));
                                ui.close_menu();
                            }
                        }
                    });
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
                    } else if ui.input(|i| i.key_pressed(egui::Key::Tab)) && !self.session.is_active()
                    {
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
                    self.snap.clear_smart_track();
                    self.snap.axis_lock = AxisLockMode::None;
                    self.last_inference_guides.clear();
                    self.smart_track_hover = None;
                    self.log(msg);
                }
                ui.separator();
                for quick in ["Line", "Circle", "Box", "Move", "ZoomExtents"] {
                    if ui.small_button(quick).clicked() {
                        self.execute_command(quick);
                    }
                }
            });

            if self.session.is_active() {
                let options = self.session.active_options();
                let axis_label = match self.snap.axis_lock {
                    AxisLockMode::None => "Off",
                    AxisLockMode::X => "X",
                    AxisLockMode::Y => "Y",
                    AxisLockMode::Z => "Z",
                };
                let st_count = self.snap.smart_track_points.len();
                if !options.is_empty() || self.snap.axis_lock != AxisLockMode::None || st_count > 0 {
                    let mut pending_opt: Option<(String, String)> = None;
                    let mut cycle_axis = false;
                    let mut clear_st = false;
                    ui.horizontal_wrapped(|ui| {
                        ui.label(
                            egui::RichText::new("Options:")
                                .small()
                                .strong()
                                .color(egui::Color32::from_rgb(165, 195, 225)),
                        );
                        for opt in &options {
                            match &opt.kind {
                                CommandOptionKind::Toggle(val) => {
                                    let text = format!("{}: {}", opt.label, if *val { "Yes" } else { "No" });
                                    if ui.selectable_label(*val, text).clicked() {
                                        pending_opt = Some((opt.key.to_string(), (!*val).to_string()));
                                    }
                                }
                                CommandOptionKind::Int { value, min, max } => {
                                    ui.label(egui::RichText::new(format!("{}:", opt.label)).small());
                                    if ui.small_button("-").clicked() {
                                        pending_opt = Some((opt.key.to_string(), (value - 1).max(*min).to_string()));
                                    }
                                    ui.label(egui::RichText::new(value.to_string()).small().strong());
                                    if ui.small_button("+").clicked() {
                                        pending_opt = Some((opt.key.to_string(), (value + 1).min(*max).to_string()));
                                    }
                                }
                                CommandOptionKind::Float { value, step, suffix } => {
                                    ui.label(egui::RichText::new(format!("{}:", opt.label)).small());
                                    if ui.small_button("-").clicked() {
                                        pending_opt = Some((opt.key.to_string(), format!("{:.3}", value - step)));
                                    }
                                    ui.label(
                                        egui::RichText::new(format!("{:.2}{}", value, suffix))
                                            .small()
                                            .strong(),
                                    );
                                    if ui.small_button("+").clicked() {
                                        pending_opt = Some((opt.key.to_string(), format!("{:.3}", value + step)));
                                    }
                                }
                                CommandOptionKind::Choice { current, choices } => {
                                    let idx = choices.iter().position(|c| *c == *current).unwrap_or(0);
                                    let next = choices[(idx + 1) % choices.len().max(1)];
                                    if ui
                                        .small_button(format!("{}: {}", opt.label, current))
                                        .on_hover_text("Click to cycle option")
                                        .clicked()
                                    {
                                        pending_opt = Some((opt.key.to_string(), next.to_string()));
                                    }
                                }
                            }
                        }
                        if ui
                            .selectable_label(
                                self.snap.axis_lock != AxisLockMode::None,
                                format!("Axis: {axis_label}"),
                            )
                            .on_hover_text("Cycle axis lock (or use Arrow Right / Left / Up)")
                            .clicked()
                        {
                            cycle_axis = true;
                        }
                        if st_count > 0
                            && ui
                                .small_button(format!("Clear SmartTrack ({st_count})"))
                                .on_hover_text("Clear temporary SmartTrack reference points (Shift+T)")
                                .clicked()
                        {
                            clear_st = true;
                        }
                    });
                    if let Some((k, v)) = pending_opt {
                        if let Some(msg) = self.session.apply_option(&k, &v) {
                            self.log(msg);
                        }
                    }
                    if cycle_axis {
                        self.snap.axis_lock = match self.snap.axis_lock {
                            AxisLockMode::None => AxisLockMode::X,
                            AxisLockMode::X => AxisLockMode::Y,
                            AxisLockMode::Y => AxisLockMode::Z,
                            AxisLockMode::Z => AxisLockMode::None,
                        };
                    }
                    if clear_st {
                        self.snap.clear_smart_track();
                        self.last_inference_guides.clear();
                        self.smart_track_hover = None;
                    }
                }
            }

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
    }
}
