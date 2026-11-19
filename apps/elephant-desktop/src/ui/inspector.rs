use cad_doc::{HistoryOp, MaterialProps};
use cad_geom::{
    BoundaryLock, Continuity, ContinuityGrade, DVec3, Geometry, LoftStyle, Plane,
    SoftTransformFalloff, UnitSystem, section_geometry_with_plane,
};
use cad_render::SurfaceAnalysisMode;
use eframe::egui;

use crate::state::{ElephantApp, GumballAlignMode, InspectorTab};

impl ElephantApp {
    pub fn draw_inspector(&mut self, ctx: &egui::Context) {
        egui::SidePanel::right("inspector")
            .default_width(275.0)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.selectable_value(
                        &mut self.inspector_tab,
                        InspectorTab::Outliner,
                        "Outliner",
                    );
                    ui.selectable_value(
                        &mut self.inspector_tab,
                        InspectorTab::Properties,
                        "Properties",
                    );
                    ui.selectable_value(
                        &mut self.inspector_tab,
                        InspectorTab::History,
                        format!("History ({})", self.doc.history_records.len()),
                    );
                    ui.selectable_value(
                        &mut self.inspector_tab,
                        InspectorTab::Analyze,
                        "Analyze",
                    );
                });
                ui.separator();

                egui::ScrollArea::vertical().show(ui, |ui| match self.inspector_tab {
                    InspectorTab::Outliner => self.draw_inspector_outliner(ui),
                    InspectorTab::Properties => self.draw_inspector_properties(ui),
                    InspectorTab::History => self.draw_inspector_history(ui),
                    InspectorTab::Analyze => self.draw_inspector_analyze(ui),
                });
            });
    }

    fn draw_inspector_outliner(&mut self, ui: &mut egui::Ui) {
        ui.heading("Layers");
        ui.horizontal(|ui| {
            let hint = if self.new_sublayer_parent.is_some() {
                "New sublayer name..."
            } else {
                "New layer..."
            };
            ui.add(
                egui::TextEdit::singleline(&mut self.new_layer_name)
                    .desired_width(140.0)
                    .hint_text(hint),
            );
            if ui.button("+").clicked() && !self.new_layer_name.trim().is_empty() {
                let name = std::mem::take(&mut self.new_layer_name);
                if let Some(pid) = self.new_sublayer_parent.take() {
                    if self.doc.layers.contains_key(pid) {
                        self.doc.add_sublayer(name.trim(), [170, 200, 235, 255], pid);
                        self.collapsed_layers.remove(&pid);
                    } else {
                        self.doc.add_layer(name.trim(), [160, 185, 220, 255]);
                    }
                } else {
                    self.doc.add_layer(name.trim(), [160, 185, 220, 255]);
                }
            }
            if self.new_sublayer_parent.is_some() && ui.small_button("Root").clicked() {
                self.new_sublayer_parent = None;
            }
        });

        let tree = self.doc.ordered_layer_tree();
        let mut select_layer_req = None;
        let mut move_to_layer_req = None;
        let mut new_sub_req = None;
        let mut toggle_collapse_req = None;
        let mut set_active_req = None;
        let mut layer_changed = false;
        let mut lock_changed = false;
        let mut skip_depth: Option<usize> = None;

        for (lid, depth) in tree {
            if let Some(sd) = skip_depth {
                if depth > sd {
                    continue;
                } else {
                    skip_depth = None;
                }
            }
            let has_children = self
                .doc
                .layers
                .values()
                .any(|l| l.parent == Some(lid));
            let is_collapsed = self.collapsed_layers.contains(&lid);
            if is_collapsed {
                skip_depth = Some(depth);
            }

            let is_active = self.doc.active_layer == lid;
            if let Some(layer) = self.doc.layers.get_mut(lid) {
                ui.horizontal(|ui| {
                    if depth > 0 {
                        ui.add_space(depth as f32 * 14.0);
                    }
                    if has_children {
                        let icon = if is_collapsed { ">" } else { "v" };
                        if ui.small_button(icon).clicked() {
                            toggle_collapse_req = Some(lid);
                        }
                    } else {
                        ui.add_space(16.0);
                    }
                    if ui
                        .color_edit_button_srgba_unmultiplied(&mut layer.color)
                        .changed()
                    {
                        layer_changed = true;
                    }
                    let lbl = ui.selectable_label(is_active, &layer.name);
                    if lbl.clicked() {
                        set_active_req = Some(lid);
                    }
                    lbl.context_menu(|ui| {
                        if ui.button("Set Active").clicked() {
                            set_active_req = Some(lid);
                            ui.close_menu();
                        }
                        if ui.button("+ Add Sublayer").clicked() {
                            new_sub_req = Some(lid);
                            ui.close_menu();
                        }
                        if ui.button("Select Objects on Layer").clicked() {
                            select_layer_req = Some(lid);
                            ui.close_menu();
                        }
                        if ui.button("Move Selected to Layer").clicked() {
                            move_to_layer_req = Some(lid);
                            ui.close_menu();
                        }
                    });
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            if ui.checkbox(&mut layer.locked, "Lock").changed() {
                                lock_changed = true;
                            }
                            if ui.checkbox(&mut layer.visible, "Vis").changed() {
                                layer_changed = true;
                            }
                        },
                    );
                });
            }
        }
        if let Some(cid) = toggle_collapse_req {
            if !self.collapsed_layers.remove(&cid) {
                self.collapsed_layers.insert(cid);
            }
        }
        if let Some(aid) = set_active_req {
            self.doc.active_layer = aid;
        }
        if let Some(pid) = new_sub_req {
            self.new_sublayer_parent = Some(pid);
        }
        if lock_changed {
            self.mark_doc_dirty();
        }
        if layer_changed {
            self.mark_scene_dirty();
        }
        if let Some(lid) = select_layer_req {
            let n = self.doc.select_layer_objects(lid);
            self.mark_doc_dirty();
            self.log(format!("Selected {n} object(s) on layer."));
        }
        if let Some(lid) = move_to_layer_req {
            let n = self.doc.move_selected_to_layer(lid);
            self.mark_scene_dirty();
            self.log(format!("Moved {n} object(s) to layer."));
        }

        ui.separator();
        ui.horizontal(|ui| {
            ui.heading(format!("Objects ({})", self.doc.objects.len()));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.small_button("Show All").clicked() {
                    self.execute_command("Show");
                }
                if ui.small_button("SelNone").clicked() {
                    self.execute_command("SelNone");
                }
            });
        });

        let obj_ids: Vec<_> = self.doc.objects.keys().collect();
        let ctrl_down = ui.input(|i| i.modifiers.ctrl || i.modifiers.shift);
        let mut clicked_oid = None;
        let mut toggled_vis_oid = None;

        for oid in obj_ids {
            let Some(obj) = self.doc.objects.get(oid) else {
                continue;
            };
            let kind_tag = match &obj.geometry {
                Geometry::Point(_) => "[Pt]",
                Geometry::Curve(_) => "[Crv]",
                Geometry::Surface(_) => "[Srf]",
                Geometry::BRep(b) => {
                    if b.is_solid() {
                        "[Solid]"
                    } else {
                        "[BRep]"
                    }
                }
                Geometry::SubD(_) => "[SubD]",
                Geometry::Mesh(_) => "[Mesh]",
            };
            let hist_tag = if self.doc.history_record_for(oid).is_some() {
                " [H]"
            } else {
                ""
            };
            let grp_tag = obj
                .group_id
                .map(|g| format!(" [G{g}]"))
                .unwrap_or_default();
            let display_name = obj
                .name
                .as_deref()
                .unwrap_or("Unnamed")
                .to_string();
            let label_str = format!("{kind_tag} {display_name}{hist_tag}{grp_tag}");
            let is_selected = obj.selected;
            let is_visible = obj.visible;

            ui.horizontal(|ui| {
                let mut vis = is_visible;
                if ui.checkbox(&mut vis, "").on_hover_text("Toggle Visibility").changed() {
                    toggled_vis_oid = Some(oid);
                }
                if ui.selectable_label(is_selected, label_str).clicked() {
                    clicked_oid = Some(oid);
                }
            });
        }

        if let Some(oid) = toggled_vis_oid {
            if let Some(obj) = self.doc.objects.get_mut(oid) {
                obj.visible = !obj.visible;
                if !obj.visible {
                    obj.selected = false;
                }
            }
            self.prune_points_on();
            self.mark_scene_dirty();
        }
        if let Some(oid) = clicked_oid {
            if !ctrl_down {
                self.doc.clear_selection();
                self.selected_subobjects.clear();
            }
            let next_sel = if ctrl_down {
                !self.doc.objects.get(oid).map(|o| o.selected).unwrap_or(false)
            } else {
                true
            };
            self.doc.set_selected(oid, next_sel);
            self.mark_doc_dirty();
        }
    }

    fn draw_inspector_properties(&mut self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        let selected_ids: Vec<_> = self
            .doc
            .objects
            .iter()
            .filter(|(_, o)| o.selected)
            .map(|(id, _)| id)
            .collect();
        let layer_choices: Vec<_> = self
            .doc
            .layers
            .iter()
            .map(|(lid, l)| (lid, l.name.clone()))
            .collect();
        let hidden_count = self.doc.objects.values().filter(|o| !o.visible).count();

        if selected_ids.is_empty() {
            ui.weak("No object selected.");
            ui.label(format!("Total objects: {}", self.doc.objects.len()));
            if hidden_count > 0 {
                ui.horizontal(|ui| {
                    ui.label(format!("Hidden: {hidden_count}"));
                    if ui.small_button("Show All").clicked() {
                        self.execute_command("Show");
                    }
                });
            }
        } else if selected_ids.len() == 1 {
            let oid = selected_ids[0];
            let mut prop_changed = false;
            let mut queued_cmd: Option<&'static str> = None;
            if let Some(obj) = self.doc.objects.get_mut(oid) {
                let mut name_buf = obj.name.clone().unwrap_or_default();
                ui.horizontal(|ui| {
                    ui.label("Name:");
                    if ui
                        .add(
                            egui::TextEdit::singleline(&mut name_buf)
                                .desired_width(150.0)
                                .hint_text("(unnamed)"),
                        )
                        .changed()
                    {
                        obj.name = if name_buf.trim().is_empty() {
                            None
                        } else {
                            Some(name_buf)
                        };
                    }
                });

                let cur_layer_name = layer_choices
                    .iter()
                    .find(|(lid, _)| *lid == obj.layer_id)
                    .map(|(_, n)| n.as_str())
                    .unwrap_or("Unknown");
                let mut new_lid = obj.layer_id;
                egui::ComboBox::from_label("Layer")
                    .selected_text(cur_layer_name)
                    .show_ui(ui, |ui| {
                        for (lid, lname) in &layer_choices {
                            ui.selectable_value(&mut new_lid, *lid, lname);
                        }
                    });
                if new_lid != obj.layer_id {
                    obj.layer_id = new_lid;
                    prop_changed = true;
                }

                let mut is_custom = obj.color.is_some();
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut is_custom, "Custom Color").changed() {
                        obj.color = if is_custom {
                            Some([230, 180, 90, 255])
                        } else {
                            None
                        };
                        prop_changed = true;
                    }
                    if let Some(col) = &mut obj.color {
                        if ui.color_edit_button_srgba_unmultiplied(col).changed() {
                            prop_changed = true;
                        }
                    }
                });

                let kind_detail = match &obj.geometry {
                    Geometry::Point(_) => "Point".to_string(),
                    Geometry::Curve(c) => format!(
                        "NURBS Curve (deg {}, {} CPs, {}, len={:.3})",
                        c.degree,
                        c.control_points.len(),
                        if c.is_closed() { "Closed" } else { "Open" },
                        c.arc_length()
                    ),
                    Geometry::Surface(s) => format!(
                        "NURBS Surface ({}x{}, deg {}x{})",
                        s.count_u, s.count_v, s.degree_u, s.degree_v
                    ),
                    Geometry::BRep(b) => format!(
                        "B-Rep {} ({}F, {}E, {}V)",
                        if b.is_solid() { "Solid" } else { "Polysurface" },
                        b.faces.len(),
                        b.edges.len(),
                        b.vertices.len()
                    ),
                    Geometry::SubD(sd) => format!(
                        "Catmull-Clark SubD ({}V, {}F, {} creases, Lvl {})",
                        sd.vertices.len(),
                        sd.faces.len(),
                        sd.creased_edges.len(),
                        sd.subdivision_level
                    ),
                    Geometry::Mesh(m) => format!(
                        "Mesh ({} verts, {} tris)",
                        m.positions.len(),
                        m.indices.len()
                    ),
                };
                ui.label(format!("Type: {kind_detail}"));
                if let Geometry::Surface(s) = &obj.geometry {
                    ui.small(format!(
                        "Rational: {} | Closed U/V: {}/{}",
                        if s.is_rational() { "Yes" } else { "No" },
                        if s.is_closed_u() { "Yes" } else { "No" },
                        if s.is_closed_v() { "Yes" } else { "No" }
                    ));
                    ui.small(format!("Area: {:.3}", s.area_approx()));
                    ui.horizontal(|ui| {
                        if ui.small_button("PointsOn").clicked() {
                            queued_cmd = Some("PointsOn");
                        }
                        if ui.small_button("Flip").clicked() {
                            queued_cmd = Some("Flip");
                        }
                        if ui.small_button("DupBorder").clicked() {
                            queued_cmd = Some("DupBorder");
                        }
                    });
                } else if let Geometry::BRep(b) = &obj.geometry {
                    let naked_cnt = b.naked_edges().len();
                    if b.is_solid() {
                        ui.small(format!(
                            "Solid | Area: {:.3} | Vol: {:.3}",
                            b.area_approx(),
                            b.volume_approx()
                        ));
                    } else {
                        ui.small(format!(
                            "Open ({naked_cnt} naked edge(s)) | Area: {:.3}",
                            b.area_approx()
                        ));
                    }
                    ui.checkbox(&mut self.show_naked_edges, "ShowEdges (Naked)");
                    let sub_cnt = self
                        .selected_subobjects
                        .iter()
                        .filter(|(id, _)| *id == oid)
                        .count();
                    ui.horizontal(|ui| {
                        ui.small(format!("Sub-sel: {sub_cnt}"));
                        if !self.selected_subobjects.is_empty()
                            && ui.small_button("Clear Sub-Sel").clicked()
                        {
                            self.selected_subobjects.clear();
                        }
                    });
                    ui.small("Tip: Ctrl+Shift+Click to pick Faces/Edges");
                    ui.horizontal(|ui| {
                        if ui.small_button("Cap").clicked() {
                            queued_cmd = Some("Cap");
                        }
                        if ui.small_button("Untrim").clicked() {
                            queued_cmd = Some("Untrim");
                        }
                        if ui.small_button("Flip").clicked() {
                            queued_cmd = Some("Flip");
                        }
                    });
                    ui.horizontal(|ui| {
                        if ui.small_button("FilletEdge").clicked() {
                            queued_cmd = Some("FilletEdge");
                        }
                        if ui.small_button("Shell").clicked() {
                            queued_cmd = Some("Shell");
                        }
                        if ui.small_button("PushPull").clicked() {
                            queued_cmd = Some("PushPull");
                        }
                        if ui.small_button("Explode").clicked() {
                            queued_cmd = Some("Explode");
                        }
                    });
                } else if let Geometry::Curve(c) = &obj.geometry {
                    ui.small(format!(
                        "Rational: {}",
                        if c.is_rational() { "Yes" } else { "No" }
                    ));
                    let mut graph_on = self.curvature_graph_objects.contains(&oid);
                    if ui.checkbox(&mut graph_on, "CurvatureGraph").changed() {
                        if graph_on {
                            self.curvature_graph_objects.insert(oid);
                        } else {
                            self.curvature_graph_objects.remove(&oid);
                        }
                    }
                    if graph_on {
                        ui.add(
                            egui::Slider::new(&mut self.curvature_graph_scale, 0.1..=5.0)
                                .text("Comb Scale"),
                        );
                    }
                } else if let Geometry::SubD(sd) = &mut obj.geometry {
                    let mut lvl = sd.subdivision_level as usize;
                    if ui
                        .add(egui::Slider::new(&mut lvl, 1..=4).text("Subdiv Level"))
                        .changed()
                    {
                        sd.subdivision_level = lvl as u8;
                        obj.refresh_aabb();
                        prop_changed = true;
                    }
                    ui.horizontal(|ui| {
                        if ui.small_button("PointsOn").clicked() {
                            queued_cmd = Some("PointsOn");
                        }
                        if ui.small_button("Subdivide").clicked() {
                            queued_cmd = Some("Subdivide");
                        }
                        if ui.small_button("Crease").clicked() {
                            queued_cmd = Some("Crease");
                        }
                    });
                    ui.horizontal(|ui| {
                        if ui.small_button("ExtrudeF").clicked() {
                            queued_cmd = Some("SubDExtrudeFace");
                        }
                        if ui.small_button("ToNURBS").clicked() {
                            queued_cmd = Some("ToNURBS");
                        }
                        if ui.small_button("MatchSubD").clicked() {
                            queued_cmd = Some("MatchSubD");
                        }
                    });
                }
                if let Some(gid) = obj.group_id {
                    ui.label(format!("Group: #{gid}"));
                }
                let c = obj.aabb.center();
                let s = obj.aabb.size();
                ui.label(format!("Center: ({:.2}, {:.2}, {:.2})", c.x, c.y, c.z));
                ui.label(format!("Size: ({:.2}, {:.2}, {:.2})", s.x, s.y, s.z));

                ui.separator();
                ui.strong("PBR Material");
                let mut mat = obj.material.unwrap_or_default();
                let mut mat_edited = false;
                ui.horizontal(|ui| {
                    if ui.small_button("Clay").clicked() {
                        mat = MaterialProps { metallic: 0.0, roughness: 0.72, opacity: 1.0 };
                        mat_edited = true;
                    }
                    if ui.small_button("Plastic").clicked() {
                        mat = MaterialProps { metallic: 0.02, roughness: 0.28, opacity: 1.0 };
                        mat_edited = true;
                    }
                    if ui.small_button("Metal").clicked() {
                        mat = MaterialProps { metallic: 0.85, roughness: 0.25, opacity: 1.0 };
                        mat_edited = true;
                    }
                    if ui.small_button("Chrome").clicked() {
                        mat = MaterialProps { metallic: 1.0, roughness: 0.08, opacity: 1.0 };
                        mat_edited = true;
                    }
                });
                ui.horizontal(|ui| {
                    mat_edited |= ui
                        .add(
                            egui::DragValue::new(&mut mat.metallic)
                                .speed(0.02)
                                .range(0.0..=1.0)
                                .prefix("Metal: "),
                        )
                        .changed();
                    mat_edited |= ui
                        .add(
                            egui::DragValue::new(&mut mat.roughness)
                                .speed(0.02)
                                .range(0.05..=1.0)
                                .prefix("Rough: "),
                        )
                        .changed();
                });
                if mat_edited {
                    obj.material = Some(mat);
                    prop_changed = true;
                }
            }
            if let Some(cmd) = queued_cmd {
                self.execute_command(cmd);
            }
            if prop_changed {
                self.mark_scene_dirty();
            }
            ui.horizontal(|ui| {
                if ui.small_button("Hide").clicked() {
                    self.execute_command("Hide");
                }
                if ui.small_button("Isolate").clicked() {
                    self.execute_command("Isolate");
                }
                if ui.small_button("Ungroup").clicked() {
                    self.execute_command("Ungroup");
                }
            });
        } else {
            ui.label(format!("{} objects selected", selected_ids.len()));
            let mut target_lid = None;
            egui::ComboBox::from_label("Set Layer")
                .selected_text("Choose layer...")
                .show_ui(ui, |ui| {
                    for (lid, lname) in &layer_choices {
                        if ui.selectable_label(false, lname).clicked() {
                            target_lid = Some(*lid);
                        }
                    }
                });
            if let Some(lid) = target_lid {
                self.doc.move_selected_to_layer(lid);
                self.mark_scene_dirty();
            }
            ui.horizontal(|ui| {
                if ui.small_button("Group").clicked() {
                    self.execute_command("Group");
                }
                if ui.small_button("Ungroup").clicked() {
                    self.execute_command("Ungroup");
                }
                if ui.small_button("Hide").clicked() {
                    self.execute_command("Hide");
                }
                if ui.small_button("Isolate").clicked() {
                    self.execute_command("Isolate");
                }
            });
        }

        ui.separator();
        ui.heading("Gumball & CPlane");
        egui::ComboBox::from_label("Align Mode")
            .selected_text(self.gumball_align.label())
            .show_ui(ui, |ui| {
                for m in GumballAlignMode::ALL {
                    ui.selectable_value(&mut self.gumball_align, m, m.label());
                }
            });
        ui.horizontal(|ui| {
            if self.gumball_custom_origin.is_some() && ui.small_button("Reset Pivot").clicked() {
                self.gumball_custom_origin = None;
            }
            if ui.checkbox(&mut self.auto_cplane, "AutoCPlane").changed() {
                self.update_auto_cplane();
                self.grids_dirty = true;
            }
        });
        ui.small("Tip: Alt-drag to Copy, Ctrl-drag origin to Relocate");

        if !self.points_on_objects.is_empty() {
            ui.separator();
            ui.heading("NURBS Control Points");
            ui.checkbox(&mut self.uvn_gumball_enabled, "UVN Gumball");
            if self.uvn_gumball_enabled {
                ui.add(
                    egui::Slider::new(&mut self.uvn_normal_drag_strength, 0.05..=2.0)
                        .text("N Drag Strength"),
                );
            }
            ui.checkbox(&mut self.soft_transform_enabled, "NURBS Soft Transform");
            if self.soft_transform_enabled {
                ui.add(
                    egui::Slider::new(&mut self.soft_transform_radius, 0.25..=25.0)
                        .text("Soft Radius"),
                );
                egui::ComboBox::from_label("Falloff")
                    .selected_text(format!("{:?}", self.soft_transform_falloff))
                    .show_ui(ui, |ui| {
                        for f in [
                            SoftTransformFalloff::Smooth,
                            SoftTransformFalloff::Linear,
                            SoftTransformFalloff::Sharp,
                        ] {
                            ui.selectable_value(&mut self.soft_transform_falloff, f, format!("{f:?}"));
                        }
                    });
                egui::ComboBox::from_label("Boundary Lock")
                    .selected_text(match self.soft_transform_lock {
                        BoundaryLock::Free => "Free",
                        BoundaryLock::FixG0 => "Fix G0",
                        BoundaryLock::FixG1 => "Fix G1",
                        BoundaryLock::FixG2 => "Fix G2",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.soft_transform_lock, BoundaryLock::Free, "Free");
                        ui.selectable_value(
                            &mut self.soft_transform_lock,
                            BoundaryLock::FixG0,
                            "Fix G0 (1 edge row)",
                        );
                        ui.selectable_value(
                            &mut self.soft_transform_lock,
                            BoundaryLock::FixG1,
                            "Fix G1 (2 edge rows)",
                        );
                        ui.selectable_value(
                            &mut self.soft_transform_lock,
                            BoundaryLock::FixG2,
                            "Fix G2 (3 edge rows)",
                        );
                    });
            }
            ui.horizontal(|ui| {
                ui.small(format!("Selected CPs: {}", self.selected_cps.len()));
                if !self.selected_cps.is_empty() && ui.small_button("Clear CPs").clicked() {
                    self.selected_cps.clear();
                }
            });
        }

        ui.separator();
        ui.heading("Document Settings");
        let mut selected_unit = self.doc.units;
        egui::ComboBox::from_label("Units")
            .selected_text(format!(
                "{} ({})",
                selected_unit.full_name(),
                selected_unit.label()
            ))
            .show_ui(ui, |ui| {
                for u in UnitSystem::ALL {
                    ui.selectable_value(
                        &mut selected_unit,
                        u,
                        format!("{} ({})", u.full_name(), u.label()),
                    );
                }
            });
        if selected_unit != self.doc.units {
            let rescale = self.rescale_on_unit_change;
            self.change_units(selected_unit, rescale);
        }
        ui.checkbox(
            &mut self.rescale_on_unit_change,
            "Rescale geometry on unit change",
        );
        ui.small(format!(
            "Abs Tolerance: {:.6} {}",
            self.doc.tolerances.absolute,
            self.doc.units.label()
        ));
    }

    fn draw_inspector_history(&mut self, ui: &mut egui::Ui) {
        ui.heading("Modeling History");
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.doc.record_history, "Record History");
            if ui.small_button("Purge Stale").clicked() {
                let removed = self.doc.purge_broken_history();
                self.log(format!("Purged {removed} stale history record(s)."));
            }
        });
        ui.weak("Non-blocking: editing a parent updates its children; editing a child directly detaches its link.");
        ui.separator();

        if let Some(sel_id) = self.doc.selected_ids_ordered().into_iter().next() {
            if let Some(rec) = self.doc.history_record_for(sel_id).cloned() {
                ui.strong(format!("Selected: {} ({} parent(s))", rec.op.label(), rec.parent_ids.len()));
                let mut edited_op: Option<HistoryOp> = None;
                match rec.op.clone() {
                    HistoryOp::Extrude { mut direction, mut solid } => {
                        let mut changed = false;
                        ui.horizontal(|ui| {
                            ui.label("Dir:");
                            changed |= ui.add(egui::DragValue::new(&mut direction.x).speed(0.1)).changed();
                            changed |= ui.add(egui::DragValue::new(&mut direction.y).speed(0.1)).changed();
                            changed |= ui.add(egui::DragValue::new(&mut direction.z).speed(0.1)).changed();
                        });
                        changed |= ui.checkbox(&mut solid, "Cap Solid").changed();
                        if changed {
                            edited_op = Some(HistoryOp::Extrude { direction, solid });
                        }
                    }
                    HistoryOp::Revolve {
                        axis_origin,
                        axis_dir,
                        angle_rad,
                    } => {
                        let mut deg = angle_rad.to_degrees();
                        if ui
                            .add(
                                egui::DragValue::new(&mut deg)
                                    .speed(1.0)
                                    .range(-360.0..=360.0)
                                    .prefix("Angle: ")
                                    .suffix(" deg"),
                            )
                            .changed()
                            && deg.abs() > 1e-2
                        {
                            edited_op = Some(HistoryOp::Revolve {
                                axis_origin,
                                axis_dir,
                                angle_rad: deg.to_radians(),
                            });
                        }
                    }
                    HistoryOp::OffsetCurve {
                        mut distance,
                        plane,
                        side_pt,
                        corner,
                    } => {
                        if ui
                            .add(
                                egui::DragValue::new(&mut distance)
                                    .speed(0.05)
                                    .range(0.01..=500.0)
                                    .prefix("Dist: "),
                            )
                            .changed()
                        {
                            edited_op = Some(HistoryOp::OffsetCurve {
                                distance,
                                plane,
                                side_pt,
                                corner,
                            });
                        }
                    }
                    HistoryOp::OffsetSrf { mut distance, solid } => {
                        if ui
                            .add(
                                egui::DragValue::new(&mut distance)
                                    .speed(0.05)
                                    .prefix("Dist: "),
                            )
                            .changed()
                        {
                            edited_op = Some(HistoryOp::OffsetSrf { distance, solid });
                        }
                    }
                    HistoryOp::BlendCrv {
                        mut continuity,
                        mut bulge,
                    } => {
                        let mut changed = false;
                        egui::ComboBox::from_label("Continuity")
                            .selected_text(continuity.label())
                            .show_ui(ui, |ui| {
                                for c in [
                                    Continuity::G0,
                                    Continuity::G1,
                                    Continuity::G2,
                                    Continuity::G3,
                                ] {
                                    if ui.selectable_value(&mut continuity, c, c.label()).changed() {
                                        changed = true;
                                    }
                                }
                            });
                        changed |= ui
                            .add(
                                egui::DragValue::new(&mut bulge)
                                    .speed(0.05)
                                    .range(0.05..=5.0)
                                    .prefix("Bulge: "),
                            )
                            .changed();
                        if changed {
                            edited_op = Some(HistoryOp::BlendCrv { continuity, bulge });
                        }
                    }
                    HistoryOp::Loft { mut style, mut closed } => {
                        let mut changed = false;
                        egui::ComboBox::from_label("Style")
                            .selected_text(format!("{style:?}"))
                            .show_ui(ui, |ui| {
                                for s in [
                                    LoftStyle::Normal,
                                    LoftStyle::Loose,
                                    LoftStyle::Tight,
                                    LoftStyle::Straight,
                                ] {
                                    if ui.selectable_value(&mut style, s, format!("{s:?}")).changed() {
                                        changed = true;
                                    }
                                }
                            });
                        changed |= ui.checkbox(&mut closed, "Closed").changed();
                        if changed {
                            edited_op = Some(HistoryOp::Loft { style, closed });
                        }
                    }
                    _ => {
                        ui.small("Parametric link active (edit parent to regenerate).");
                    }
                }
                if let Some(new_op) = edited_op {
                    if self.doc.update_history_op(sel_id, new_op) {
                        self.mark_scene_dirty();
                    }
                }
                if ui.small_button("Break History Link").clicked() {
                    self.doc.break_history_for(sel_id);
                    self.log("Broke modeling history link for selected object.");
                }
            } else {
                let child_cnt = self.doc.history_children_of(sel_id).len();
                if child_cnt > 0 {
                    ui.label(format!("Parent object driving {child_cnt} child(ren)."));
                } else {
                    ui.weak("Selected object has no history record.");
                }
            }
            ui.separator();
        }

        ui.strong(format!("All History Links ({})", self.doc.history_records.len()));
        let pairs: Vec<(cad_doc::ObjectId, &'static str, usize)> = self
            .doc
            .history_records
            .iter()
            .map(|r| (r.child_id, r.op.label(), r.parent_ids.len()))
            .collect();
        let mut pick_child = None;
        for (cid, lbl, np) in pairs {
            let name = self
                .doc
                .objects
                .get(cid)
                .and_then(|o| o.name.as_deref())
                .unwrap_or("Child");
            let selected = self.doc.objects.get(cid).is_some_and(|o| o.selected);
            if ui
                .selectable_label(selected, format!("{lbl}: {name} ({np}P)"))
                .clicked()
            {
                pick_child = Some(cid);
            }
        }
        if let Some(cid) = pick_child {
            self.doc.clear_selection();
            self.doc.set_selected(cid, true);
            self.mark_doc_dirty();
        }
    }

    fn draw_inspector_analyze(&mut self, ui: &mut egui::Ui) {
        ui.heading("Surface Analysis Shaders");
        let prev_mode = self.surface_analysis.mode;
        ui.horizontal_wrapped(|ui| {
            for mode in SurfaceAnalysisMode::ALL {
                let short = match mode {
                    SurfaceAnalysisMode::None => "Off",
                    SurfaceAnalysisMode::Zebra => "Zebra",
                    SurfaceAnalysisMode::EnvironmentMap => "E-Map",
                    SurfaceAnalysisMode::GaussianCurvature => "Gaussian",
                    SurfaceAnalysisMode::MeanCurvature => "Mean",
                    SurfaceAnalysisMode::DraftAngle => "Draft",
                };
                if ui
                    .selectable_label(self.surface_analysis.mode == mode, short)
                    .on_hover_text(mode.label())
                    .clicked()
                {
                    self.surface_analysis.mode = mode;
                }
            }
        });
        let was_curv = matches!(
            prev_mode,
            SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
        );
        let now_curv = matches!(
            self.surface_analysis.mode,
            SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature
        );
        if prev_mode != self.surface_analysis.mode && (was_curv || now_curv) {
            self.doc_dirty = true;
        }

        match self.surface_analysis.mode {
            SurfaceAnalysisMode::Zebra => {
                ui.add(
                    egui::Slider::new(&mut self.surface_analysis.zebra_stripes, 2.0..=30.0)
                        .text("Stripes"),
                );
                ui.add(
                    egui::Slider::new(&mut self.surface_analysis.zebra_angle_deg, 0.0..=180.0)
                        .text("Angle deg"),
                );
            }
            SurfaceAnalysisMode::EnvironmentMap => {
                let names = [
                    "0: Studio Softbox",
                    "1: Liquid Chrome",
                    "2: Glossy Car Paint",
                    "3: Fluorescent Tubes",
                ];
                let idx = (self.surface_analysis.emap_preset as usize).min(names.len() - 1);
                egui::ComboBox::from_label("E-Map Preset")
                    .selected_text(names[idx])
                    .show_ui(ui, |ui| {
                        for (i, n) in names.iter().enumerate() {
                            ui.selectable_value(&mut self.surface_analysis.emap_preset, i as u32, *n);
                        }
                    });
            }
            SurfaceAnalysisMode::GaussianCurvature | SurfaceAnalysisMode::MeanCurvature => {
                ui.horizontal(|ui| {
                    ui.add(
                        egui::DragValue::new(&mut self.surface_analysis.curvature_min)
                            .speed(0.02)
                            .prefix("Min: "),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.surface_analysis.curvature_max)
                            .speed(0.02)
                            .prefix("Max: "),
                    );
                });
                let (bar_rect, _) =
                    ui.allocate_exact_size(egui::vec2(ui.available_width(), 12.0), egui::Sense::hover());
                let p = ui.painter_at(bar_rect);
                let stops = [
                    egui::Color32::from_rgb(26, 64, 235),
                    egui::Color32::from_rgb(30, 199, 230),
                    egui::Color32::from_rgb(46, 214, 92),
                    egui::Color32::from_rgb(245, 209, 41),
                    egui::Color32::from_rgb(240, 46, 41),
                ];
                for s in 0..stops.len() - 1 {
                    let t0 = s as f32 / (stops.len() - 1) as f32;
                    let t1 = (s + 1) as f32 / (stops.len() - 1) as f32;
                    let seg = egui::Rect::from_x_y_ranges(
                        egui::lerp(bar_rect.left()..=bar_rect.right(), t0)
                            ..=egui::lerp(bar_rect.left()..=bar_rect.right(), t1),
                        bar_rect.top()..=bar_rect.bottom(),
                    );
                    p.rect_filled(seg, 0.0, stops[s]);
                }
            }
            SurfaceAnalysisMode::DraftAngle => {
                ui.horizontal(|ui| {
                    ui.label("Pull:");
                    if ui.small_button("+Z").clicked() {
                        self.surface_analysis.draft_pull_dir = DVec3::Z;
                    }
                    if ui.small_button("-Z").clicked() {
                        self.surface_analysis.draft_pull_dir = DVec3::NEG_Z;
                    }
                    if ui.small_button("+Y").clicked() {
                        self.surface_analysis.draft_pull_dir = DVec3::Y;
                    }
                    if ui.small_button("+X").clicked() {
                        self.surface_analysis.draft_pull_dir = DVec3::X;
                    }
                });
                ui.add(
                    egui::Slider::new(&mut self.surface_analysis.draft_angle_deg, 0.5..=20.0)
                        .text("Parting Band deg"),
                );
                ui.small("Blue = Positive Draft | Yellow = Parting | Red = Undercut");
            }
            SurfaceAnalysisMode::None => {}
        }
        ui.add(
            egui::Slider::new(&mut self.surface_analysis.cavity_ao_strength, 0.0..=1.0)
                .text("Cavity AO"),
        );

        ui.separator();
        ui.heading("Global Edge Continuity");
        ui.horizontal(|ui| {
            if ui
                .checkbox(&mut self.edge_continuity_enabled, "Show GEC Overlay")
                .changed()
                && self.edge_continuity_enabled
            {
                self.recompute_edge_continuity();
            }
            if ui.small_button("Recompute").clicked() {
                self.recompute_edge_continuity();
                self.edge_continuity_enabled = true;
            }
        });
        ui.horizontal(|ui| {
            let mut changed = false;
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.edge_continuity_tol.g0_tol)
                        .speed(0.001)
                        .range(1e-5..=1.0)
                        .prefix("G0:"),
                )
                .changed();
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.edge_continuity_tol.g1_tol_deg)
                        .speed(0.1)
                        .range(0.01..=45.0)
                        .prefix("G1°:"),
                )
                .changed();
            changed |= ui
                .add(
                    egui::DragValue::new(&mut self.edge_continuity_tol.g2_tol_rel)
                        .speed(0.01)
                        .range(0.001..=1.0)
                        .prefix("G2:"),
                )
                .changed();
            if changed && self.edge_continuity_enabled {
                self.recompute_edge_continuity();
            }
        });

        let mut counts = [0usize; 4];
        for r in &self.edge_continuity_reports {
            let idx = match r.grade {
                ContinuityGrade::G2 => 0,
                ContinuityGrade::G1 => 1,
                ContinuityGrade::G0 => 2,
                ContinuityGrade::Broken => 3,
            };
            counts[idx] += 1;
        }
        if !self.edge_continuity_reports.is_empty() {
            ui.horizontal(|ui| {
                ui.colored_label(egui::Color32::from_rgb(75, 225, 115), format!("G2:{}", counts[0]));
                ui.colored_label(egui::Color32::from_rgb(245, 210, 65), format!("G1:{}", counts[1]));
                ui.colored_label(egui::Color32::from_rgb(245, 145, 55), format!("G0:{}", counts[2]));
                ui.colored_label(egui::Color32::from_rgb(245, 70, 70), format!("GAP:{}", counts[3]));
            });
            egui::Frame::group(ui.style()).show(ui, |ui| {
                for r in self.edge_continuity_reports.iter().take(12) {
                    let col = match r.grade {
                        ContinuityGrade::G2 => egui::Color32::from_rgb(75, 225, 115),
                        ContinuityGrade::G1 => egui::Color32::from_rgb(245, 210, 65),
                        ContinuityGrade::G0 => egui::Color32::from_rgb(245, 145, 55),
                        ContinuityGrade::Broken => egui::Color32::from_rgb(245, 70, 70),
                    };
                    ui.horizontal(|ui| {
                        ui.colored_label(col, r.grade.short_label());
                        ui.small(format!(
                            "{}: d={:.4}, {:.1}°, k={:.0}%",
                            r.label,
                            r.max_gap,
                            r.max_angle_deg,
                            r.max_curvature_diff_rel * 100.0
                        ));
                    });
                }
            });
        } else {
            ui.weak("No interior B-Rep or adjacent NURBS seams analyzed yet.");
        }

        ui.separator();
        ui.heading("Edge Diagnostics");
        let mut naked_cnt = 0usize;
        let mut non_man_cnt = 0usize;
        for (_, o) in &self.doc.objects {
            if o.visible && self.doc.is_layer_effectively_visible(o.layer_id) {
                if let Geometry::BRep(b) = &o.geometry {
                    naked_cnt += b.naked_edges().len();
                    non_man_cnt += b.non_manifold_edges().len();
                }
            }
        }
        ui.checkbox(
            &mut self.show_naked_edges,
            format!("Naked Edges ({naked_cnt})"),
        );
        ui.checkbox(
            &mut self.show_non_manifold_edges,
            format!("Non-Manifold Edges ({non_man_cnt})"),
        );

        ui.separator();
        ui.heading("Mass Properties");
        ui.horizontal(|ui| {
            if ui.button("Compute Mass Props").clicked() {
                self.recompute_mass_properties();
            }
            ui.add(
                egui::DragValue::new(&mut self.material_density_g_cm3)
                    .speed(0.05)
                    .range(0.05..=25.0)
                    .suffix(" g/cm3"),
            );
        });
        ui.horizontal(|ui| {
            if ui.small_button("Al 2.70").clicked() {
                self.material_density_g_cm3 = 2.70;
            }
            if ui.small_button("Steel 7.85").clicked() {
                self.material_density_g_cm3 = 7.85;
            }
            if ui.small_button("ABS 1.04").clicked() {
                self.material_density_g_cm3 = 1.04;
            }
            if ui.small_button("Ti 4.43").clicked() {
                self.material_density_g_cm3 = 4.43;
            }
        });
        if let Some(mp) = &self.last_mass_properties {
            let cm3_per_unit3 = (self.doc.units.scale_to(UnitSystem::Centimeters)).powi(3);
            let mass_g = mp.volume * cm3_per_unit3 * self.material_density_g_cm3;
            let u_lbl = self.doc.units.label();
            ui.small(format!(
                "Solid: {} | Area: {:.3} {u_lbl}^2",
                if mp.is_closed_solid { "Yes" } else { "Open" },
                mp.area
            ));
            ui.small(format!("Volume: {:.4} {u_lbl}^3 | Mass: {:.2} g", mp.volume, mass_g));
            ui.small(format!(
                "Centroid: ({:.3}, {:.3}, {:.3})",
                mp.centroid.x, mp.centroid.y, mp.centroid.z
            ));
            ui.small(format!(
                "I1..I3: ({:.2}, {:.2}, {:.2})",
                mp.principal_moments[0], mp.principal_moments[1], mp.principal_moments[2]
            ));
        }

        ui.separator();
        ui.heading("Live Clipping Planes");
        let bbox = self.doc.visible_bounding_box();
        let center = if bbox.is_empty() {
            DVec3::ZERO
        } else {
            bbox.center()
        };
        let span = if bbox.is_empty() {
            16.0
        } else {
            (bbox.size().max_element() * 1.6).max(8.0)
        };
        ui.horizontal(|ui| {
            if ui.small_button("+ Z").clicked() {
                let mut p = Plane::world_xy();
                p.origin = center;
                let id = self.doc.add_clipping_plane("Clip Z", p);
                if let Some(cp) = self.doc.clipping_planes.iter_mut().find(|c| c.id == id) {
                    cp.size = span;
                }
                self.mark_doc_dirty();
            }
            if ui.small_button("+ X").clicked() {
                let mut p = Plane::world_yz();
                p.origin = center;
                let id = self.doc.add_clipping_plane("Clip X", p);
                if let Some(cp) = self.doc.clipping_planes.iter_mut().find(|c| c.id == id) {
                    cp.size = span;
                }
                self.mark_doc_dirty();
            }
            if ui.small_button("+ Y").clicked() {
                let mut p = Plane::world_zx();
                p.origin = center;
                let id = self.doc.add_clipping_plane("Clip Y", p);
                if let Some(cp) = self.doc.clipping_planes.iter_mut().find(|c| c.id == id) {
                    cp.size = span;
                }
                self.mark_doc_dirty();
            }
            if ui.small_button("+ CPlane").clicked() {
                let p = self.cameras[self.active_viewport].cplane;
                let id = self.doc.add_clipping_plane("Clip CP", p);
                if let Some(cp) = self.doc.clipping_planes.iter_mut().find(|c| c.id == id) {
                    cp.size = span;
                }
                self.mark_doc_dirty();
            }
        });

        let mut remove_cp_id = None;
        let mut bake_cp_plane: Option<Plane> = None;
        let mut clip_changed = false;
        for cp in &mut self.doc.clipping_planes {
            ui.group(|ui| {
                ui.horizontal(|ui| {
                    if ui.checkbox(&mut cp.enabled, &cp.name).changed() {
                        clip_changed = true;
                    }
                    if ui.small_button("Flip").clicked() {
                        cp.flip = !cp.flip;
                        clip_changed = true;
                    }
                    if ui.small_button("Bake").clicked() {
                        bake_cp_plane = Some(cp.plane);
                    }
                    if ui.small_button("X").clicked() {
                        remove_cp_id = Some(cp.id);
                    }
                });
                ui.horizontal(|ui| {
                    let mut offset = cp.plane.origin.dot(cp.plane.z_axis);
                    if ui
                        .add(
                            egui::DragValue::new(&mut offset)
                                .speed(0.05)
                                .prefix("Offset: "),
                        )
                        .changed()
                    {
                        let prev = cp.plane.origin.dot(cp.plane.z_axis);
                        cp.plane.origin += cp.plane.z_axis * (offset - prev);
                        clip_changed = true;
                    }
                    if ui.checkbox(&mut cp.fill_caps, "Section Cut").changed() {
                        clip_changed = true;
                    }
                });
            });
        }
        if let Some(id) = remove_cp_id {
            self.doc.remove_clipping_plane(id);
            clip_changed = true;
        }
        if let Some(plane) = bake_cp_plane {
            let mut added = 0usize;
            let objs: Vec<_> = self
                .doc
                .objects
                .values()
                .filter(|o| o.visible && self.doc.is_layer_effectively_visible(o.layer_id))
                .map(|o| (o.layer_id, o.geometry.clone()))
                .collect();
            for (lid, geom) in objs {
                for poly in section_geometry_with_plane(&geom, &plane) {
                    if let Ok(c) = cad_geom::NurbsCurve::polyline(&poly) {
                        self.doc
                            .add_object_on_layer("SectionCut", lid, Geometry::Curve(c));
                        added += 1;
                    }
                }
            }
            if added > 0 {
                self.mark_scene_dirty();
            }
            self.log(format!("Baked {added} section cut curve(s)."));
        }
        if clip_changed {
            self.mark_doc_dirty();
        }
    }
}
