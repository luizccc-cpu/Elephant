use cad_script::{MathOpKind, NodeBooleanKind, NodeKind, ParametricGraph, PortType};
use eframe::egui;

use crate::state::{ElephantApp, WorkspaceViewMode};

const NODE_WIDTH: f32 = 178.0;
const HEADER_HEIGHT: f32 = 24.0;
const PORT_ROW_H: f32 = 18.0;

impl ElephantApp {
    pub fn draw_node_canvas(&mut self, ui: &mut egui::Ui) {
        self.parametric_graph.ensure_valid_positions();

        let mut add_kind: Option<NodeKind> = None;
        let mut reset_default = false;
        let mut center_view = false;
        let mut bake_req = false;

        ui.horizontal_wrapped(|ui| {
            if ui.button("<- Model View").clicked() {
                self.workspace_view = WorkspaceViewMode::Model;
            }
            ui.separator();

            ui.menu_button("+ Add Node", |ui| {
                ui.menu_button("Input & Math", |ui| {
                    if ui.button("Number Slider").clicked() {
                        add_kind = Some(NodeKind::NumberSlider {
                            value: 2.0,
                            min: 0.1,
                            max: 20.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Point (X,Y,Z)").clicked() {
                        add_kind = Some(NodeKind::Point {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Math Operation").clicked() {
                        add_kind = Some(NodeKind::MathOp {
                            op: MathOpKind::Add,
                            a: 1.0,
                            b: 1.0,
                        });
                        ui.close_menu();
                    }
                });
                ui.menu_button("Curves", |ui| {
                    if ui.button("Circle").clicked() {
                        add_kind = Some(NodeKind::Circle { radius: 2.0 });
                        ui.close_menu();
                    }
                    if ui.button("Rectangle").clicked() {
                        add_kind = Some(NodeKind::Rectangle {
                            width: 4.0,
                            height: 3.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Polygon").clicked() {
                        add_kind = Some(NodeKind::Polygon {
                            radius: 2.0,
                            sides: 6,
                        });
                        ui.close_menu();
                    }
                });
                ui.menu_button("Surface & Solid", |ui| {
                    if ui.button("Extrude").clicked() {
                        add_kind = Some(NodeKind::Extrude { height: 3.0 });
                        ui.close_menu();
                    }
                    if ui.button("Loft").clicked() {
                        add_kind = Some(NodeKind::Loft);
                        ui.close_menu();
                    }
                    if ui.button("Revolve").clicked() {
                        add_kind = Some(NodeKind::Revolve { angle_deg: 360.0 });
                        ui.close_menu();
                    }
                    if ui.button("BRep Box").clicked() {
                        add_kind = Some(NodeKind::Box {
                            sx: 2.0,
                            sy: 2.0,
                            sz: 2.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("NURBS Sphere").clicked() {
                        add_kind = Some(NodeKind::Sphere { radius: 1.5 });
                        ui.close_menu();
                    }
                });
                ui.menu_button("SubD & Mesh", |ui| {
                    if ui.button("SubD Box").clicked() {
                        add_kind = Some(NodeKind::SubDBox {
                            sx: 2.0,
                            sy: 2.0,
                            sz: 2.0,
                            level: 2,
                        });
                        ui.close_menu();
                    }
                    if ui.button("SubD Sphere").clicked() {
                        add_kind = Some(NodeKind::SubDSphere {
                            radius: 1.5,
                            level: 3,
                        });
                        ui.close_menu();
                    }
                    if ui.button("ToNURBS").clicked() {
                        add_kind = Some(NodeKind::ToNurbs);
                        ui.close_menu();
                    }
                    if ui.button("Boolean (BRep / Mesh)").clicked() {
                        add_kind = Some(NodeKind::BooleanOp {
                            op: NodeBooleanKind::Union,
                        });
                        ui.close_menu();
                    }
                    if ui.button("ShrinkWrap").clicked() {
                        add_kind = Some(NodeKind::ShrinkWrap {
                            voxel_size: 0.25,
                            offset: 0.0,
                            smooth: 4,
                        });
                        ui.close_menu();
                    }
                });
                ui.menu_button("Transform & Array", |ui| {
                    if ui.button("Move").clicked() {
                        add_kind = Some(NodeKind::Move {
                            dx: 0.0,
                            dy: 0.0,
                            dz: 0.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Rotate Z").clicked() {
                        add_kind = Some(NodeKind::Rotate { angle_deg: 45.0 });
                        ui.close_menu();
                    }
                    if ui.button("Linear Array").clicked() {
                        add_kind = Some(NodeKind::LinearArray {
                            count: 4,
                            step_x: 3.5,
                            step_y: 0.0,
                            step_z: 0.0,
                        });
                        ui.close_menu();
                    }
                    if ui.button("Polar Array").clicked() {
                        add_kind = Some(NodeKind::PolarArray {
                            count: 6,
                            radius: 4.0,
                        });
                        ui.close_menu();
                    }
                });
                if ui.button("Output Sink").clicked() {
                    add_kind = Some(NodeKind::Output);
                    ui.close_menu();
                }
            });

            ui.checkbox(&mut self.parametric_graph.live_preview, "Live 3D Preview");
            if ui.button("Bake to Model").clicked() {
                bake_req = true;
            }
            if ui.small_button("Center View").clicked() {
                center_view = true;
            }
            if ui.small_button("Reset Default").clicked() {
                reset_default = true;
            }
            ui.separator();
            ui.weak(format!(
                "Nodes: {}  Links: {}  Tip: Drag header=Move, Right socket->Left socket=Wire, R-Click input=Unwire, Mid/Right-drag=Pan",
                self.parametric_graph.nodes.len(),
                self.parametric_graph.links.len(),
            ));
        });

        let avail = ui.available_rect_before_wrap();
        if let Some(kind) = add_kind {
            let zoom = self.node_canvas_zoom.max(0.25);
            let center_world =
                (avail.center() - avail.min - self.node_canvas_pan) / zoom - egui::vec2(85.0, 50.0);
            let nid = self
                .parametric_graph
                .add_node(kind, [center_world.x.max(10.0), center_world.y.max(10.0)]);
            self.node_canvas_selected = Some(nid);
        }
        if reset_default {
            let prev_live = self.parametric_graph.live_preview;
            self.parametric_graph = ParametricGraph::default();
            self.parametric_graph.live_preview = prev_live;
            self.node_canvas_wire_from = None;
            center_view = true;
        }
        if center_view {
            self.node_canvas_pan = egui::vec2(20.0, 20.0);
            self.node_canvas_zoom = 1.0;
        }
        if bake_req {
            let n = self.parametric_graph.bake_into(&mut self.doc);
            self.mark_scene_dirty();
            self.log(format!("Baked {n} parametric object(s) from Node Canvas."));
        }

        let (canvas_rect, canvas_resp) =
            ui.allocate_exact_size(avail.size().max(egui::vec2(100.0, 100.0)), egui::Sense::click_and_drag());
        let painter = ui.painter_at(canvas_rect);

        // Background + grid
        painter.rect_filled(canvas_rect, 0.0, egui::Color32::from_rgb(24, 27, 32));
        let zoom = self.node_canvas_zoom.clamp(0.45, 1.85);
        let grid_step = (32.0 * zoom).max(12.0);
        let ox = (self.node_canvas_pan.x % grid_step + grid_step) % grid_step;
        let oy = (self.node_canvas_pan.y % grid_step + grid_step) % grid_step;
        let grid_col = egui::Color32::from_rgb(35, 39, 46);
        let mut gx = canvas_rect.left() + ox;
        while gx < canvas_rect.right() {
            painter.line_segment(
                [egui::pos2(gx, canvas_rect.top()), egui::pos2(gx, canvas_rect.bottom())],
                egui::Stroke::new(1.0_f32, grid_col),
            );
            gx += grid_step;
        }
        let mut gy = canvas_rect.top() + oy;
        while gy < canvas_rect.bottom() {
            painter.line_segment(
                [egui::pos2(canvas_rect.left(), gy), egui::pos2(canvas_rect.right(), gy)],
                egui::Stroke::new(1.0_f32, grid_col),
            );
            gy += grid_step;
        }

        // Scroll zoom around cursor
        if canvas_resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let factor = (1.0 + scroll * 0.0015).clamp(0.8, 1.25);
                let old_zoom = self.node_canvas_zoom;
                let new_zoom = (old_zoom * factor).clamp(0.45, 1.85);
                if let Some(hp) = ui.input(|i| i.pointer.hover_pos()) {
                    let rel = hp - canvas_rect.min;
                    self.node_canvas_pan =
                        rel - (rel - self.node_canvas_pan) * (new_zoom / old_zoom);
                }
                self.node_canvas_zoom = new_zoom;
            }
        }
        if canvas_resp.dragged_by(egui::PointerButton::Middle)
            || canvas_resp.dragged_by(egui::PointerButton::Secondary)
        {
            self.node_canvas_pan += ui.input(|i| i.pointer.delta());
        }

        let zoom = self.node_canvas_zoom.clamp(0.45, 1.85);
        let to_screen = |world: [f32; 2]| -> egui::Pos2 {
            canvas_rect.min + self.node_canvas_pan + egui::vec2(world[0], world[1]) * zoom
        };

        // Precompute node geometry & sockets for wire rendering and hit-testing
        struct NodeGeom {
            id: usize,
            rect: egui::Rect,
            header_rect: egui::Rect,
            body_rect: egui::Rect,
            in_sockets: Vec<(&'static str, PortType, egui::Pos2)>,
            out_socket: Option<(PortType, egui::Pos2)>,
        }

        let node_geoms: Vec<NodeGeom> = self
            .parametric_graph
            .nodes
            .iter()
            .map(|n| {
                let in_ports = n.kind.input_ports();
                let out_port = n.kind.output_port();
                let param_h = match &n.kind {
                    NodeKind::NumberSlider { .. }
                    | NodeKind::Circle { .. }
                    | NodeKind::Extrude { .. }
                    | NodeKind::Revolve { .. }
                    | NodeKind::Sphere { .. }
                    | NodeKind::Rotate { .. } => 24.0,
                    NodeKind::Point { .. }
                    | NodeKind::Box { .. }
                    | NodeKind::SubDBox { .. }
                    | NodeKind::ShrinkWrap { .. }
                    | NodeKind::LinearArray { .. }
                    | NodeKind::Move { .. } => 44.0,
                    NodeKind::MathOp { .. }
                    | NodeKind::Rectangle { .. }
                    | NodeKind::Polygon { .. }
                    | NodeKind::SubDSphere { .. }
                    | NodeKind::BooleanOp { .. }
                    | NodeKind::PolarArray { .. } => 30.0,
                    NodeKind::Loft | NodeKind::ToNurbs | NodeKind::Output => 12.0,
                };
                let ports_h = (in_ports.len().max(1) as f32) * PORT_ROW_H + 6.0;
                let total_h = (HEADER_HEIGHT + ports_h + param_h) * zoom;
                let tl = to_screen(n.pos);
                let rect = egui::Rect::from_min_size(tl, egui::vec2(NODE_WIDTH * zoom, total_h));
                let header_rect =
                    egui::Rect::from_min_size(tl, egui::vec2(NODE_WIDTH * zoom, HEADER_HEIGHT * zoom));
                let ports_bottom = tl.y + (HEADER_HEIGHT + ports_h) * zoom;
                let body_rect = egui::Rect::from_min_max(
                    egui::pos2(tl.x + 6.0 * zoom, ports_bottom),
                    egui::pos2(rect.right() - 6.0 * zoom, rect.bottom() - 4.0 * zoom),
                );
                let in_sockets = in_ports
                    .iter()
                    .enumerate()
                    .map(|(pi, p)| {
                        let y = tl.y + (HEADER_HEIGHT + 12.0 + pi as f32 * PORT_ROW_H) * zoom;
                        (p.name, p.ty, egui::pos2(rect.left(), y))
                    })
                    .collect();
                let out_socket = out_port.map(|op| {
                    let y = tl.y + (HEADER_HEIGHT + 14.0) * zoom;
                    (op.ty, egui::pos2(rect.right(), y))
                });
                NodeGeom {
                    id: n.id,
                    rect,
                    header_rect,
                    body_rect,
                    in_sockets,
                    out_socket,
                }
            })
            .collect();

        // Draw Bezier wires
        for link in &self.parametric_graph.links {
            let Some(src) = node_geoms.iter().find(|g| g.id == link.from_node) else {
                continue;
            };
            let Some(dst) = node_geoms.iter().find(|g| g.id == link.to_node) else {
                continue;
            };
            let Some((pty, p0)) = src.out_socket else {
                continue;
            };
            let Some(&(_, _, p1)) = dst.in_sockets.iter().find(|(nm, _, _)| *nm == link.to_port)
            else {
                continue;
            };
            draw_bezier_wire(&painter, p0, p1, port_color(pty), zoom);
        }

        // Draw rubber-band wire while dragging from output socket
        let pointer_pos = ui.input(|i| i.pointer.hover_pos().or(i.pointer.interact_pos()));
        let pointer_released = ui.input(|i| i.pointer.primary_released());
        if let (Some(from_id), Some(cur)) = (self.node_canvas_wire_from, pointer_pos) {
            if let Some(src) = node_geoms.iter().find(|g| g.id == from_id) {
                if let Some((pty, p0)) = src.out_socket {
                    draw_bezier_wire(&painter, p0, cur, port_color(pty), zoom);
                }
            }
        }

        // Render & interact with each node card
        let mut delete_node_id: Option<usize> = None;
        let mut connect_req: Option<(usize, usize, &'static str)> = None;
        let mut disconnect_req: Option<(usize, &'static str)> = None;

        for (idx, ng) in node_geoms.iter().enumerate() {
            let node_id = ng.id;
            let selected = self.node_canvas_selected == Some(node_id);
            let cat_col = node_category_color(&self.parametric_graph.nodes[idx].kind);

            // Card background & border
            painter.rect_filled(ng.rect, 6.0 * zoom, egui::Color32::from_rgb(38, 42, 50));
            painter.rect_filled(ng.header_rect, 6.0 * zoom, cat_col);
            let border = if selected {
                egui::Color32::from_rgb(245, 200, 75)
            } else {
                egui::Color32::from_rgb(76, 84, 98)
            };
            painter.rect_stroke(ng.rect, 6.0 * zoom, egui::Stroke::new(1.5_f32, border));

            // Header drag + label
            let header_resp = ui.interact(
                ng.header_rect,
                egui::Id::new(("nc_hdr", node_id)),
                egui::Sense::click_and_drag(),
            );
            if header_resp.clicked() || header_resp.drag_started() {
                self.node_canvas_selected = Some(node_id);
            }
            if header_resp.dragged_by(egui::PointerButton::Primary) {
                let d = header_resp.drag_delta() / zoom;
                self.parametric_graph.nodes[idx].pos[0] += d.x;
                self.parametric_graph.nodes[idx].pos[1] += d.y;
            }
            painter.text(
                ng.header_rect.left_center() + egui::vec2(8.0 * zoom, 0.0),
                egui::Align2::LEFT_CENTER,
                &self.parametric_graph.nodes[idx].label,
                egui::FontId::proportional((12.0 * zoom).clamp(8.5, 16.0)),
                egui::Color32::WHITE,
            );

            // Close/Delete corner button
            let close_rect = egui::Rect::from_center_size(
                ng.header_rect.right_center() - egui::vec2(11.0 * zoom, 0.0),
                egui::vec2(14.0 * zoom, 14.0 * zoom),
            );
            let close_resp = ui.interact(
                close_rect,
                egui::Id::new(("nc_del", node_id)),
                egui::Sense::click(),
            );
            painter.text(
                close_rect.center(),
                egui::Align2::CENTER_CENTER,
                "x",
                egui::FontId::proportional((11.0 * zoom).clamp(8.0, 14.0)),
                if close_resp.hovered() {
                    egui::Color32::from_rgb(255, 110, 110)
                } else {
                    egui::Color32::from_rgb(220, 226, 234)
                },
            );
            if close_resp.clicked() {
                delete_node_id = Some(node_id);
            }

            // Input sockets
            for &(pname, pty, sp) in &ng.in_sockets {
                let sock_r = (6.0 * zoom).clamp(4.0, 9.0);
                let hit_rect = egui::Rect::from_center_size(sp, egui::vec2(sock_r * 2.6, sock_r * 2.6));
                let s_resp = ui.interact(
                    hit_rect,
                    egui::Id::new(("nc_in", node_id, pname)),
                    egui::Sense::click(),
                );
                let col = port_color(pty);
                let hovering_wire = self.node_canvas_wire_from.is_some()
                    && pointer_pos.is_some_and(|p| p.distance(sp) <= sock_r * 2.4);
                painter.circle_filled(sp, if hovering_wire { sock_r * 1.35 } else { sock_r }, col);
                painter.circle_stroke(sp, sock_r, egui::Stroke::new(1.2_f32, egui::Color32::WHITE));
                painter.text(
                    sp + egui::vec2(sock_r + 4.0 * zoom, 0.0),
                    egui::Align2::LEFT_CENTER,
                    pname,
                    egui::FontId::proportional((10.5 * zoom).clamp(7.5, 13.0)),
                    egui::Color32::from_rgb(205, 212, 222),
                );
                if s_resp.secondary_clicked() {
                    disconnect_req = Some((node_id, pname));
                }
                if pointer_released && hovering_wire {
                    if let Some(from_id) = self.node_canvas_wire_from {
                        connect_req = Some((from_id, node_id, pname));
                    }
                }
            }

            // Output socket
            if let Some((pty, sp)) = ng.out_socket {
                let sock_r = (6.0 * zoom).clamp(4.0, 9.0);
                let hit_rect = egui::Rect::from_center_size(sp, egui::vec2(sock_r * 2.6, sock_r * 2.6));
                let o_resp = ui.interact(
                    hit_rect,
                    egui::Id::new(("nc_out", node_id)),
                    egui::Sense::click_and_drag(),
                );
                let col = port_color(pty);
                painter.circle_filled(sp, sock_r, col);
                painter.circle_stroke(sp, sock_r, egui::Stroke::new(1.2_f32, egui::Color32::WHITE));
                if o_resp.drag_started_by(egui::PointerButton::Primary)
                    || o_resp.is_pointer_button_down_on()
                {
                    self.node_canvas_wire_from = Some(node_id);
                }
            }

            // Inline body controls
            if ng.body_rect.height() >= 16.0 {
                let node_mut = &mut self.parametric_graph.nodes[idx];
                ui.allocate_new_ui(egui::UiBuilder::new().max_rect(ng.body_rect), |ui| {
                    ui.set_clip_rect(ng.body_rect.intersect(canvas_rect));
                    draw_node_inline_controls(ui, node_mut.id, &mut node_mut.kind);
                });
            }
        }

        if let Some((from_id, to_id, pname)) = connect_req {
            if !self.parametric_graph.connect(from_id, to_id, pname) {
                self.log("Cannot wire incompatible ports or cycle.");
            }
        }
        if let Some((to_id, pname)) = disconnect_req {
            self.parametric_graph.disconnect(to_id, pname);
        }
        if let Some(did) = delete_node_id {
            self.parametric_graph.remove_node(did);
            if self.node_canvas_selected == Some(did) {
                self.node_canvas_selected = None;
            }
        }
        if pointer_released {
            self.node_canvas_wire_from = None;
        }
    }
}

fn port_color(pty: PortType) -> egui::Color32 {
    match pty {
        PortType::Number => egui::Color32::from_rgb(95, 220, 120),
        PortType::Point => egui::Color32::from_rgb(75, 210, 245),
        PortType::Geometry => egui::Color32::from_rgb(245, 165, 65),
    }
}

fn node_category_color(kind: &NodeKind) -> egui::Color32 {
    match kind {
        NodeKind::NumberSlider { .. } | NodeKind::Point { .. } | NodeKind::MathOp { .. } => {
            egui::Color32::from_rgb(46, 108, 72)
        }
        NodeKind::Circle { .. } | NodeKind::Rectangle { .. } | NodeKind::Polygon { .. } => {
            egui::Color32::from_rgb(44, 96, 146)
        }
        NodeKind::Extrude { .. }
        | NodeKind::Loft
        | NodeKind::Revolve { .. }
        | NodeKind::Box { .. }
        | NodeKind::Sphere { .. } => egui::Color32::from_rgb(112, 72, 142),
        NodeKind::SubDBox { .. }
        | NodeKind::SubDSphere { .. }
        | NodeKind::ToNurbs
        | NodeKind::BooleanOp { .. }
        | NodeKind::ShrinkWrap { .. } => egui::Color32::from_rgb(148, 92, 42),
        NodeKind::Move { .. }
        | NodeKind::Rotate { .. }
        | NodeKind::LinearArray { .. }
        | NodeKind::PolarArray { .. } => egui::Color32::from_rgb(44, 122, 128),
        NodeKind::Output => egui::Color32::from_rgb(156, 60, 68),
    }
}

fn draw_bezier_wire(
    painter: &egui::Painter,
    p0: egui::Pos2,
    p1: egui::Pos2,
    color: egui::Color32,
    zoom: f32,
) {
    let dx = ((p1.x - p0.x).abs() * 0.5).max(38.0 * zoom);
    let c0 = p0 + egui::vec2(dx, 0.0);
    let c1 = p1 - egui::vec2(dx, 0.0);
    painter.add(egui::epaint::CubicBezierShape::from_points_stroke(
        [p0, c0, c1, p1],
        false,
        egui::Color32::TRANSPARENT,
        egui::Stroke::new((2.2 * zoom).clamp(1.2, 4.0), color),
    ));
}

fn draw_node_inline_controls(ui: &mut egui::Ui, id: usize, kind: &mut NodeKind) {
    match kind {
        NodeKind::NumberSlider { value, min, max } => {
            ui.add(egui::Slider::new(value, *min..=*max).show_value(true));
        }
        NodeKind::Point { x, y, z } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(x).speed(0.1).prefix("x:"));
                ui.add(egui::DragValue::new(y).speed(0.1).prefix("y:"));
                ui.add(egui::DragValue::new(z).speed(0.1).prefix("z:"));
            });
        }
        NodeKind::MathOp { op, b, .. } => {
            ui.horizontal(|ui| {
                egui::ComboBox::from_id_salt(("nc_math", id))
                    .selected_text(op.label())
                    .width(72.0)
                    .show_ui(ui, |ui| {
                        for &k in &MathOpKind::ALL {
                            ui.selectable_value(op, k, k.label());
                        }
                    });
                ui.add(egui::DragValue::new(b).speed(0.1).prefix("b:"));
            });
        }
        NodeKind::Circle { radius } | NodeKind::Sphere { radius } => {
            ui.add(
                egui::DragValue::new(radius)
                    .speed(0.1)
                    .range(0.05..=200.0)
                    .prefix("r: "),
            );
        }
        NodeKind::Rectangle { width, height } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(width).speed(0.1).range(0.05..=200.0).prefix("w:"));
                ui.add(egui::DragValue::new(height).speed(0.1).range(0.05..=200.0).prefix("h:"));
            });
        }
        NodeKind::Polygon { radius, sides } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(radius).speed(0.1).range(0.05..=200.0).prefix("r:"));
                ui.add(egui::DragValue::new(sides).range(3..=32).prefix("n:"));
            });
        }
        NodeKind::Extrude { height } => {
            ui.add(egui::DragValue::new(height).speed(0.1).prefix("h: "));
        }
        NodeKind::Revolve { angle_deg } | NodeKind::Rotate { angle_deg } => {
            ui.add(
                egui::DragValue::new(angle_deg)
                    .speed(2.0)
                    .range(-360.0..=360.0)
                    .suffix("°"),
            );
        }
        NodeKind::Box { sx, sy, sz } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(sx).speed(0.1).range(0.05..=200.0));
                ui.add(egui::DragValue::new(sy).speed(0.1).range(0.05..=200.0));
                ui.add(egui::DragValue::new(sz).speed(0.1).range(0.05..=200.0));
            });
        }
        NodeKind::SubDBox { sx, sy, level, .. } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(sx).speed(0.1).range(0.05..=200.0).prefix("s:"));
                ui.add(egui::DragValue::new(sy).speed(0.1).range(0.05..=200.0));
                ui.add(egui::DragValue::new(level).range(1..=4).prefix("lvl:"));
            });
        }
        NodeKind::SubDSphere { radius, level } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(radius).speed(0.1).range(0.05..=200.0).prefix("r:"));
                ui.add(egui::DragValue::new(level).range(1..=4).prefix("lvl:"));
            });
        }
        NodeKind::BooleanOp { op } => {
            egui::ComboBox::from_id_salt(("nc_bool", id))
                .selected_text(op.label())
                .width(110.0)
                .show_ui(ui, |ui| {
                    for &k in &NodeBooleanKind::ALL {
                        ui.selectable_value(op, k, k.label());
                    }
                });
        }
        NodeKind::ShrinkWrap {
            voxel_size,
            offset,
            smooth,
        } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(voxel_size).speed(0.02).range(0.08..=5.0).prefix("v:"));
                ui.add(egui::DragValue::new(offset).speed(0.05).prefix("off:"));
                ui.add(egui::DragValue::new(smooth).range(0..=12).prefix("sm:"));
            });
        }
        NodeKind::Move { dx, dy, dz } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(dx).speed(0.1));
                ui.add(egui::DragValue::new(dy).speed(0.1));
                ui.add(egui::DragValue::new(dz).speed(0.1));
            });
        }
        NodeKind::LinearArray { count, step_x, .. } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(count).range(1..=64).prefix("n:"));
                ui.add(egui::DragValue::new(step_x).speed(0.1).prefix("dx:"));
            });
        }
        NodeKind::PolarArray { count, radius } => {
            ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(count).range(1..=64).prefix("n:"));
                ui.add(egui::DragValue::new(radius).speed(0.1).prefix("r:"));
            });
        }
        NodeKind::Loft | NodeKind::ToNurbs | NodeKind::Output => {}
    }
}
