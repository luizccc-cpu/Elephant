use cad_geom::Geometry;
use eframe::egui;

use crate::state::ElephantApp;

#[derive(Clone, Copy)]
struct RadialSector {
    label: &'static str,
    cmd: &'static str,
    color: egui::Color32,
}

impl ElephantApp {
    pub fn draw_radial_menu(&mut self, ctx: &egui::Context) {
        let Some(center) = self.radial_menu_pos else {
            return;
        };

        let sectors = self.contextual_radial_sectors();
        let outer_r = 114.0_f32;
        let inner_r = 30.0_f32;

        let mut close_menu = false;
        let mut trigger_cmd: Option<&'static str> = None;

        egui::Area::new(egui::Id::new("elephant_radial_menu"))
            .order(egui::Order::Foreground)
            .fixed_pos(center - egui::vec2(outer_r + 12.0, outer_r + 12.0))
            .show(ctx, |ui| {
                let side = (outer_r + 12.0) * 2.0;
                let (rect, response) =
                    ui.allocate_exact_size(egui::vec2(side, side), egui::Sense::click());
                let painter = ui.painter_at(rect);
                let c = rect.center();

                painter.circle_filled(
                    c,
                    outer_r + 4.0,
                    egui::Color32::from_rgba_unmultiplied(20, 22, 27, 228),
                );
                painter.circle_stroke(
                    c,
                    outer_r + 4.0,
                    egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(80, 145, 225)),
                );

                let hover_pos = ui.input(|i| i.pointer.hover_pos());
                let hovered_idx = hover_pos.and_then(|hp| {
                    let v = hp - c;
                    let dist = v.length();
                    if dist >= inner_r && dist <= outer_r + 6.0 {
                        let mut ang = v.y.atan2(v.x) + std::f32::consts::FRAC_PI_2;
                        if ang < 0.0 {
                            ang += std::f32::consts::TAU;
                        }
                        let step = std::f32::consts::TAU / sectors.len() as f32;
                        let shifted = (ang + step * 0.5) % std::f32::consts::TAU;
                        Some((shifted / step) as usize % sectors.len())
                    } else {
                        None
                    }
                });

                let step = std::f32::consts::TAU / sectors.len() as f32;
                for (i, sec) in sectors.iter().enumerate() {
                    let a_center = -std::f32::consts::FRAC_PI_2 + i as f32 * step;
                    let a0 = a_center - step * 0.48;
                    let a1 = a_center + step * 0.48;
                    let is_hot = hovered_idx == Some(i);

                    let mut poly = Vec::with_capacity(14);
                    for k in 0..=5 {
                        let a = a0 + (a1 - a0) * (k as f32 / 5.0);
                        poly.push(c + egui::vec2(a.cos() * outer_r, a.sin() * outer_r));
                    }
                    for k in (0..=5).rev() {
                        let a = a0 + (a1 - a0) * (k as f32 / 5.0);
                        poly.push(c + egui::vec2(a.cos() * inner_r, a.sin() * inner_r));
                    }

                    let fill = if is_hot {
                        sec.color.linear_multiply(1.35)
                    } else {
                        egui::Color32::from_rgba_unmultiplied(44, 49, 58, 235)
                    };
                    let stroke = if is_hot {
                        egui::Stroke::new(1.5_f32, egui::Color32::WHITE)
                    } else {
                        egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(72, 80, 95))
                    };
                    painter.add(egui::Shape::convex_polygon(poly, fill, stroke));

                    let label_pos = c + egui::vec2(a_center.cos(), a_center.sin()) * ((inner_r + outer_r) * 0.52);
                    painter.text(
                        label_pos,
                        egui::Align2::CENTER_CENTER,
                        sec.label,
                        egui::FontId::proportional(11.5),
                        if is_hot {
                            egui::Color32::WHITE
                        } else {
                            egui::Color32::from_rgb(225, 232, 242)
                        },
                    );
                }

                painter.circle_filled(c, inner_r - 2.0, egui::Color32::from_rgb(32, 36, 44));
                painter.circle_stroke(
                    c,
                    inner_r - 2.0,
                    egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(90, 105, 125)),
                );
                painter.text(
                    c,
                    egui::Align2::CENTER_CENTER,
                    "Quick\nMenu",
                    egui::FontId::proportional(10.0),
                    egui::Color32::from_rgb(175, 185, 200),
                );

                if response.clicked() {
                    if let Some(hi) = hovered_idx {
                        trigger_cmd = Some(sectors[hi].cmd);
                    }
                    close_menu = true;
                } else if ui.input(|i| i.pointer.any_pressed())
                    && !rect.contains(ui.input(|i| i.pointer.interact_pos()).unwrap_or(c))
                {
                    close_menu = true;
                }
            });

        if close_menu || trigger_cmd.is_some() {
            self.radial_menu_pos = None;
        }
        if let Some(cmd) = trigger_cmd {
            self.execute_command(cmd);
        }
    }

    fn contextual_radial_sectors(&self) -> [RadialSector; 8] {
        let col_blue = egui::Color32::from_rgb(48, 92, 148);
        let col_teal = egui::Color32::from_rgb(42, 118, 128);
        let col_amber = egui::Color32::from_rgb(140, 96, 42);
        let col_purple = egui::Color32::from_rgb(104, 68, 138);

        let sel_id = self.doc.selected_ids_ordered().into_iter().next();
        match sel_id.and_then(|id| self.doc.objects.get(id)).map(|o| &o.geometry) {
            Some(Geometry::BRep(_)) => [
                RadialSector { label: "FilletEdge", cmd: "FilletEdge", color: col_amber },
                RadialSector { label: "PushPull", cmd: "PushPull", color: col_blue },
                RadialSector { label: "Shell", cmd: "Shell", color: col_teal },
                RadialSector { label: "BoolDiff", cmd: "BooleanDifference", color: col_purple },
                RadialSector { label: "Move", cmd: "Move", color: col_blue },
                RadialSector { label: "Cap", cmd: "Cap", color: col_teal },
                RadialSector { label: "Explode", cmd: "Explode", color: col_amber },
                RadialSector { label: "ShowEdges", cmd: "ShowEdges", color: col_purple },
            ],
            Some(Geometry::Curve(_)) => [
                RadialSector { label: "Extrude", cmd: "Extrude", color: col_blue },
                RadialSector { label: "Loft", cmd: "Loft", color: col_teal },
                RadialSector { label: "Offset", cmd: "Offset", color: col_amber },
                RadialSector { label: "PlanarSrf", cmd: "PlanarSrf", color: col_purple },
                RadialSector { label: "PointsOn", cmd: "PointsOn", color: col_blue },
                RadialSector { label: "Fillet", cmd: "Fillet", color: col_amber },
                RadialSector { label: "Join", cmd: "Join", color: col_teal },
                RadialSector { label: "Mirror", cmd: "Mirror", color: col_purple },
            ],
            Some(Geometry::Surface(_)) => [
                RadialSector { label: "PointsOn", cmd: "PointsOn", color: col_blue },
                RadialSector { label: "OffsetSrf", cmd: "OffsetSrf", color: col_teal },
                RadialSector { label: "FilletSrf", cmd: "FilletSrf", color: col_amber },
                RadialSector { label: "ToBRep", cmd: "ToBRep", color: col_purple },
                RadialSector { label: "Move", cmd: "Move", color: col_blue },
                RadialSector { label: "DupEdge", cmd: "DupEdge", color: col_teal },
                RadialSector { label: "SplitSrf", cmd: "SplitSrf", color: col_amber },
                RadialSector { label: "UVN", cmd: "UVN", color: col_purple },
            ],
            _ => [
                RadialSector { label: "Line", cmd: "Line", color: col_blue },
                RadialSector { label: "Circle", cmd: "Circle", color: col_teal },
                RadialSector { label: "Rectangle", cmd: "Rectangle", color: col_amber },
                RadialSector { label: "InterpCrv", cmd: "InterpCrv", color: col_purple },
                RadialSector { label: "Box", cmd: "Box", color: col_blue },
                RadialSector { label: "Cylinder", cmd: "Cylinder", color: col_teal },
                RadialSector { label: "Dim", cmd: "Dim", color: col_amber },
                RadialSector { label: "ZoomExt", cmd: "ZoomExtents", color: col_purple },
            ],
        }
    }
}
