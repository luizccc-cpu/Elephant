use cad_geom::{BoundaryLock, Geometry, SoftTransformFalloff, UnitSystem};
use eframe::egui;

use crate::state::ElephantApp;

impl ElephantApp {
    pub fn draw_inspector(&mut self, ctx: &egui::Context) {
        egui::SidePanel::right("inspector")
            .default_width(250.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
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
                let mut select_layer_req = None;
                let mut move_to_layer_req = None;
                let mut layer_changed = false;
                for lid in &layer_ids {
                    let lid = *lid;
                    let is_active = self.doc.active_layer == lid;
                    if let Some(layer) = self.doc.layers.get_mut(lid) {
                        ui.horizontal(|ui| {
                            if ui
                                .color_edit_button_srgba_unmultiplied(&mut layer.color)
                                .changed()
                            {
                                layer_changed = true;
                            }
                            let lbl = ui.selectable_label(is_active, &layer.name);
                            if lbl.clicked() {
                                self.doc.active_layer = lid;
                            }
                            lbl.context_menu(|ui| {
                                if ui.button("Set Active").clicked() {
                                    self.doc.active_layer = lid;
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
                                    ui.checkbox(&mut layer.locked, "Lock");
                                    if ui.checkbox(&mut layer.visible, "Vis").changed() {
                                        layer_changed = true;
                                    }
                                },
                            );
                        });
                    }
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
                                    egui::Slider::new(
                                        &mut self.curvature_graph_scale,
                                        0.1..=5.0,
                                    )
                                    .text("Comb Scale"),
                                );
                            }
                        }
                        if let Some(gid) = obj.group_id {
                            ui.label(format!("Group: #{gid}"));
                        }
                        let c = obj.aabb.center();
                        let s = obj.aabb.size();
                        ui.label(format!("Center: ({:.2}, {:.2}, {:.2})", c.x, c.y, c.z));
                        ui.label(format!("Size: ({:.2}, {:.2}, {:.2})", s.x, s.y, s.z));
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
                                    ui.selectable_value(
                                        &mut self.soft_transform_falloff,
                                        f,
                                        format!("{f:?}"),
                                    );
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
                                ui.selectable_value(
                                    &mut self.soft_transform_lock,
                                    BoundaryLock::Free,
                                    "Free",
                                );
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
                        if !self.selected_cps.is_empty() && ui.small_button("Clear CPs").clicked()
                        {
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
                });
            });
    }
}
