use cad_doc::{AnnotationKind, LayoutSheet, PageSize};
use cad_geom::{DVec3, DrawingProjection, Make2DLineKind, Make2DOptions, make_2d_projection};
use cad_io::{export_layout_pdf_bytes, export_layout_svg_string};
use eframe::egui;

use crate::state::{ElephantApp, WorkspaceViewMode};

impl ElephantApp {
    pub fn draw_layout_view(&mut self, ui: &mut egui::Ui, layout_id: u64) {
        let Some(layout_idx) = self.doc.layouts.iter().position(|l| l.id == layout_id) else {
            self.workspace_view = WorkspaceViewMode::Model;
            return;
        };

        let mut delete_sheet = false;
        let mut export_pdf_req = false;
        let mut export_svg_req = false;
        let mut add_detail_proj: Option<DrawingProjection> = None;
        let mut reset_4view = false;
        let mut sheet_changed = false;

        ui.horizontal_wrapped(|ui| {
            let layout = &mut self.doc.layouts[layout_idx];
            if ui.button("<- Model View").clicked() {
                self.workspace_view = WorkspaceViewMode::Model;
            }
            ui.separator();
            ui.label("Sheet:");
            ui.add(egui::TextEdit::singleline(&mut layout.name).desired_width(110.0));
            egui::ComboBox::from_id_salt(("layout_page_size", layout.id))
                .selected_text(layout.page_size.label())
                .show_ui(ui, |ui| {
                    for ps in PageSize::ALL {
                        if ui
                            .selectable_value(&mut layout.page_size, ps, ps.label())
                            .changed()
                        {
                            sheet_changed = true;
                        }
                    }
                });
            if ui.checkbox(&mut layout.landscape, "Landscape").changed() {
                sheet_changed = true;
            }
            ui.menu_button("Title Block", |ui| {
                ui.horizontal(|ui| {
                    ui.label("Title:");
                    ui.text_edit_singleline(&mut layout.title_block.title);
                });
                ui.horizontal(|ui| {
                    ui.label("DWG #:");
                    ui.text_edit_singleline(&mut layout.title_block.drawing_number);
                });
                ui.horizontal(|ui| {
                    ui.label("Author:");
                    ui.text_edit_singleline(&mut layout.title_block.author);
                });
                ui.horizontal(|ui| {
                    ui.label("Rev:");
                    ui.text_edit_singleline(&mut layout.title_block.revision);
                });
                ui.horizontal(|ui| {
                    ui.label("Scale:");
                    ui.text_edit_singleline(&mut layout.title_block.scale_text);
                });
                ui.horizontal(|ui| {
                    ui.label("Date:");
                    ui.text_edit_singleline(&mut layout.title_block.date);
                });
            });
            ui.menu_button("+ Detail", |ui| {
                for proj in [
                    DrawingProjection::Top,
                    DrawingProjection::Front,
                    DrawingProjection::Right,
                    DrawingProjection::Isometric,
                ] {
                    if ui.button(proj.label()).clicked() {
                        add_detail_proj = Some(proj);
                        ui.close_menu();
                    }
                }
            });
            if ui.small_button("Reset 4-View").clicked() {
                reset_4view = true;
            }
            if ui.small_button("Fit Sheet").clicked() {
                self.layout_pan = egui::Vec2::ZERO;
                self.layout_zoom = 1.0;
            }
            ui.separator();
            if ui.button("Export PDF").clicked() {
                export_pdf_req = true;
            }
            if ui.small_button("Export SVG").clicked() {
                export_svg_req = true;
            }
            if ui.small_button("Delete").clicked() {
                delete_sheet = true;
            }
        });

        if sheet_changed {
            let layout = &mut self.doc.layouts[layout_idx];
            if layout.details.len() == 4 {
                let rebuilt =
                    LayoutSheet::standard_4_view(layout.id, layout.name.clone(), layout.page_size);
                for (dst, src) in layout.details.iter_mut().zip(rebuilt.details.iter()) {
                    dst.rect_mm = src.rect_mm;
                }
            }
            self.cached_layout_projections.clear();
        }
        if reset_4view {
            let name = self.doc.layouts[layout_idx].name.clone();
            let ps = self.doc.layouts[layout_idx].page_size;
            let tb = self.doc.layouts[layout_idx].title_block.clone();
            let mut rebuilt = LayoutSheet::standard_4_view(layout_id, name, ps);
            rebuilt.title_block = tb;
            self.doc.layouts[layout_idx] = rebuilt;
            self.cached_layout_projections.clear();
        }
        if let Some(proj) = add_detail_proj {
            let layout = &mut self.doc.layouts[layout_idx];
            let next_id = layout.details.iter().map(|d| d.id).max().unwrap_or(0) + 1;
            let (pw, ph) = layout.page_size.dimensions_mm(layout.landscape);
            layout.details.push(cad_doc::DetailView {
                id: next_id,
                title: format!("{} VIEW", proj.label().to_ascii_uppercase()),
                projection: proj,
                rect_mm: [
                    layout.margin_mm + 12.0,
                    layout.margin_mm + 12.0,
                    (pw - 2.0 * layout.margin_mm) * 0.45,
                    (ph - 2.0 * layout.margin_mm) * 0.40,
                ],
                scale: 0.0,
                show_hidden_lines: true,
                show_dimensions: true,
            });
            self.cached_layout_projections.clear();
        }
        if export_pdf_req {
            let sheet = self.doc.layouts[layout_idx].clone();
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Vector PDF (.pdf)", &["pdf"])
                .set_file_name(format!("{}.pdf", sheet.name.replace(' ', "_")))
                .save_file()
            {
                let bytes = export_layout_pdf_bytes(&self.doc, &sheet);
                match std::fs::write(&path, bytes) {
                    Ok(()) => self.log(format!("Exported Vector PDF to {}", path.display())),
                    Err(e) => self.log(format!("PDF export failed: {e}")),
                }
            }
        }
        if export_svg_req {
            let sheet = self.doc.layouts[layout_idx].clone();
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("Layout SVG (.svg)", &["svg"])
                .set_file_name(format!("{}.svg", sheet.name.replace(' ', "_")))
                .save_file()
            {
                let text = export_layout_svg_string(&self.doc, &sheet);
                match std::fs::write(&path, text) {
                    Ok(()) => self.log(format!("Exported Layout SVG to {}", path.display())),
                    Err(e) => self.log(format!("SVG export failed: {e}")),
                }
            }
        }
        if delete_sheet {
            self.doc.remove_layout(layout_id);
            self.workspace_view = self
                .doc
                .layouts
                .first()
                .map(|l| WorkspaceViewMode::Layout(l.id))
                .unwrap_or(WorkspaceViewMode::Model);
            return;
        }

        ui.separator();
        let canvas_rect = ui.available_rect_before_wrap();
        let response = ui.allocate_rect(canvas_rect, egui::Sense::click_and_drag());
        if response.dragged() {
            self.layout_pan += ui.input(|i| i.pointer.delta());
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.raw_scroll_delta.y);
            if scroll.abs() > 0.1 {
                let factor = (1.0 + scroll * 0.0015).clamp(0.75, 1.35);
                self.layout_zoom = (self.layout_zoom * factor).clamp(0.25, 8.0);
            }
        }

        let painter = ui.painter_at(canvas_rect);
        painter.rect_filled(canvas_rect, 0.0, egui::Color32::from_rgb(42, 46, 54));

        let sheet = self.doc.layouts[layout_idx].clone();
        let (pw, ph) = sheet.page_size.dimensions_mm(sheet.landscape);
        let fit_px_per_mm = ((canvas_rect.width() - 48.0).max(40.0) / pw as f32)
            .min((canvas_rect.height() - 48.0).max(40.0) / ph as f32);
        let px_per_mm = fit_px_per_mm * self.layout_zoom;
        let sheet_w_px = pw as f32 * px_per_mm;
        let sheet_h_px = ph as f32 * px_per_mm;
        let sheet_center = canvas_rect.center() + self.layout_pan;
        let sheet_rect =
            egui::Rect::from_center_size(sheet_center, egui::vec2(sheet_w_px, sheet_h_px));

        let mm_to_scr = |x_mm: f64, y_mm: f64| -> egui::Pos2 {
            egui::pos2(
                sheet_rect.left() + x_mm as f32 * px_per_mm,
                sheet_rect.top() + y_mm as f32 * px_per_mm,
            )
        };

        // Sheet shadow + white paper
        painter.rect_filled(
            sheet_rect.translate(egui::vec2(6.0, 6.0)),
            2.0,
            egui::Color32::from_rgba_unmultiplied(0, 0, 0, 95),
        );
        painter.rect_filled(sheet_rect, 1.0, egui::Color32::from_rgb(248, 249, 251));

        // Sheet margin frame
        let m = sheet.margin_mm;
        let inner_tl = mm_to_scr(m, m);
        let inner_br = mm_to_scr(pw - m, ph - m);
        painter.rect_stroke(
            egui::Rect::from_min_max(inner_tl, inner_br),
            0.0,
            egui::Stroke::new(1.3_f32, egui::Color32::from_rgb(35, 40, 48)),
        );

        // Title block
        let tb_w = 88.0_f64.min((pw - 2.0 * m) * 0.48);
        let tb_h = 28.0_f64;
        let tb_x = pw - m - tb_w;
        let tb_y = ph - m - tb_h;
        let tb_rect = egui::Rect::from_min_max(
            mm_to_scr(tb_x, tb_y),
            mm_to_scr(tb_x + tb_w, tb_y + tb_h),
        );
        painter.rect_stroke(
            tb_rect,
            0.0,
            egui::Stroke::new(1.1_f32, egui::Color32::from_rgb(35, 40, 48)),
        );
        painter.line_segment(
            [mm_to_scr(tb_x, tb_y + 10.0), mm_to_scr(tb_x + tb_w, tb_y + 10.0)],
            egui::Stroke::new(0.8_f32, egui::Color32::from_rgb(65, 72, 82)),
        );
        painter.line_segment(
            [mm_to_scr(tb_x, tb_y + 19.0), mm_to_scr(tb_x + tb_w, tb_y + 19.0)],
            egui::Stroke::new(0.8_f32, egui::Color32::from_rgb(65, 72, 82)),
        );
        painter.line_segment(
            [
                mm_to_scr(tb_x + tb_w * 0.52, tb_y + 10.0),
                mm_to_scr(tb_x + tb_w * 0.52, tb_y + tb_h),
            ],
            egui::Stroke::new(0.8_f32, egui::Color32::from_rgb(65, 72, 82)),
        );
        let title_font = egui::FontId::proportional((3.4 * px_per_mm).clamp(7.0, 18.0));
        let meta_font = egui::FontId::proportional((2.4 * px_per_mm).clamp(6.0, 13.0));
        let ink = egui::Color32::from_rgb(24, 28, 36);
        painter.text(
            mm_to_scr(tb_x + 2.5, tb_y + 3.0),
            egui::Align2::LEFT_TOP,
            &sheet.title_block.title,
            title_font,
            ink,
        );
        painter.text(
            mm_to_scr(tb_x + 2.5, tb_y + 12.0),
            egui::Align2::LEFT_TOP,
            format!("DWG: {}", sheet.title_block.drawing_number),
            meta_font.clone(),
            ink,
        );
        painter.text(
            mm_to_scr(tb_x + tb_w * 0.54, tb_y + 12.0),
            egui::Align2::LEFT_TOP,
            format!("REV: {}", sheet.title_block.revision),
            meta_font.clone(),
            ink,
        );
        painter.text(
            mm_to_scr(tb_x + 2.5, tb_y + 21.0),
            egui::Align2::LEFT_TOP,
            format!("BY: {}", sheet.title_block.author),
            meta_font.clone(),
            ink,
        );
        painter.text(
            mm_to_scr(tb_x + tb_w * 0.54, tb_y + 21.0),
            egui::Align2::LEFT_TOP,
            format!("UNITS: {}", self.doc.units.label()),
            meta_font,
            ink,
        );

        let mut cached_items: Option<Vec<_>> = None;
        for detail in &sheet.details {
            let [rx, ry, rw, rh] = detail.rect_mm;
            if rw <= 2.0 || rh <= 2.0 {
                continue;
            }
            let d_rect = egui::Rect::from_min_max(mm_to_scr(rx, ry), mm_to_scr(rx + rw, ry + rh));
            painter.rect_stroke(
                d_rect,
                0.0,
                egui::Stroke::new(0.9_f32, egui::Color32::from_rgb(165, 172, 184)),
            );
            painter.text(
                mm_to_scr(rx + 2.5, ry + 2.0),
                egui::Align2::LEFT_TOP,
                &detail.title,
                egui::FontId::proportional((2.7 * px_per_mm).clamp(7.0, 14.0)),
                ink,
            );

            let key = (sheet.id, detail.id);
            if !self.cached_layout_projections.contains_key(&key) {
                let items = cached_items.get_or_insert_with(|| {
                    self.doc
                        .objects
                        .values()
                        .filter(|o| o.visible && self.doc.is_layer_effectively_visible(o.layer_id))
                        .map(|o| {
                            let col = o
                                .color
                                .or_else(|| self.doc.layers.get(o.layer_id).map(|l| l.color));
                            (o.geometry.clone(), col)
                        })
                        .collect()
                });
                let proj = make_2d_projection(
                    items,
                    detail.projection,
                    &Make2DOptions {
                        include_hidden: detail.show_hidden_lines,
                        ..Default::default()
                    },
                );
                self.cached_layout_projections.insert(key, proj);
            }

            let Some(res) = self.cached_layout_projections.get(&key) else {
                continue;
            };
            let span_x = (res.bounds_max[0] - res.bounds_min[0]).max(1e-6);
            let span_y = (res.bounds_max[1] - res.bounds_min[1]).max(1e-6);
            let inner_w = (rw - 8.0).max(4.0);
            let inner_h = (rh - 11.0).max(4.0);
            let scale = if detail.scale > 1e-6 {
                detail.scale
            } else {
                (inner_w / span_x).min(inner_h / span_y)
            };
            let mid_x = 0.5 * (res.bounds_min[0] + res.bounds_max[0]);
            let mid_y = 0.5 * (res.bounds_min[1] + res.bounds_max[1]);
            let cx_mm = rx + rw * 0.5;
            let cy_mm = ry + 6.0 + inner_h * 0.5;

            let map_pt = |px: f64, py: f64| -> egui::Pos2 {
                mm_to_scr(cx_mm + (px - mid_x) * scale, cy_mm - (py - mid_y) * scale)
            };

            let dp = ui.painter_at(d_rect.shrink(1.0));
            let vis_stroke = egui::Stroke::new(1.1_f32, egui::Color32::from_rgb(24, 28, 36));
            let sil_stroke = egui::Stroke::new(1.8_f32, egui::Color32::from_rgb(10, 12, 16));
            let hid_stroke = egui::Stroke::new(0.8_f32, egui::Color32::from_rgb(140, 148, 160));

            for seg in &res.segments {
                if seg.points_2d.len() < 2 {
                    continue;
                }
                let pts: Vec<egui::Pos2> =
                    seg.points_2d.iter().map(|p| map_pt(p[0], p[1])).collect();
                match seg.kind {
                    Make2DLineKind::Visible => {
                        dp.add(egui::Shape::line(pts, vis_stroke));
                    }
                    Make2DLineKind::Silhouette => {
                        dp.add(egui::Shape::line(pts, sil_stroke));
                    }
                    Make2DLineKind::Hidden => {
                        for win in pts.windows(2) {
                            dp.add(egui::Shape::dashed_line(
                                &[win[0], win[1]],
                                hid_stroke,
                                5.0,
                                4.0,
                            ));
                        }
                    }
                }
            }

            if detail.show_dimensions {
                for ann in &self.doc.annotations {
                    if !self.doc.is_layer_effectively_visible(ann.layer_id) {
                        continue;
                    }
                    let Some(p2) = project_ortho_point(ann.position, detail.projection) else {
                        continue;
                    };
                    let sp = map_pt(p2[0], p2[1]);
                    if let Some(gdt) = &ann.gdt {
                        let txt = format!(
                            "[{}] {}",
                            gdt.symbol.short_code(),
                            if gdt.datums.is_empty() {
                                format!("{:.3}", gdt.tolerance)
                            } else {
                                format!("{:.3} | {}", gdt.tolerance, gdt.datums.join(" | "))
                            }
                        );
                        let gal = dp.layout_no_wrap(
                            txt,
                            egui::FontId::monospace(9.5),
                            egui::Color32::from_rgb(18, 22, 28),
                        );
                        let box_r =
                            egui::Rect::from_min_size(sp, gal.size() + egui::vec2(6.0, 4.0));
                        dp.rect(
                            box_r,
                            0.0,
                            egui::Color32::WHITE,
                            egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(20, 24, 30)),
                        );
                        dp.galley(
                            box_r.min + egui::vec2(3.0, 2.0),
                            gal,
                            egui::Color32::from_rgb(18, 22, 28),
                        );
                    } else if ann.kind == AnnotationKind::Dimension
                        || ann.kind == AnnotationKind::Text
                        || ann.kind == AnnotationKind::DatumTag
                    {
                        dp.text(
                            sp,
                            egui::Align2::CENTER_CENTER,
                            &ann.text,
                            egui::FontId::proportional(9.5),
                            egui::Color32::from_rgb(20, 60, 145),
                        );
                    }
                }
            }
        }
    }
}

fn project_ortho_point(p: DVec3, proj: DrawingProjection) -> Option<[f64; 2]> {
    match proj {
        DrawingProjection::Top => Some([p.x, p.y]),
        DrawingProjection::Bottom => Some([p.x, -p.y]),
        DrawingProjection::Front => Some([p.x, p.z]),
        DrawingProjection::Back => Some([-p.x, p.z]),
        DrawingProjection::Right => Some([p.y, p.z]),
        DrawingProjection::Left => Some([-p.y, p.z]),
        DrawingProjection::Isometric => {
            let forward = DVec3::new(-1.0, 1.0, -1.0).normalize();
            let right = DVec3::new(1.0, 1.0, 0.0).normalize();
            let mut up = right.cross(forward).normalize();
            if up.z < 0.0 {
                up = -up;
            }
            Some([p.dot(right), p.dot(up)])
        }
        DrawingProjection::Perspective { .. } => None,
    }
}
