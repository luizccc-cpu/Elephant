use std::fmt::Write as _;

use cad_doc::{AnnotationItem, Document, LayoutSheet};
use cad_geom::{DVec3, DrawingProjection, Make2DLineKind, Make2DOptions, make_2d_projection};

const MM_TO_PT: f64 = 72.0 / 25.4;

/// Render a paper-space `LayoutSheet` to pure **Vector PDF 1.4** bytes (no rasterization or external crates).
pub fn export_layout_pdf_bytes(doc: &Document, layout: &LayoutSheet) -> Vec<u8> {
    let (pw_mm, ph_mm) = layout.page_size.dimensions_mm(layout.landscape);
    let pw_pt = pw_mm * MM_TO_PT;
    let ph_pt = ph_mm * MM_TO_PT;
    let to_pt_x = |x_mm: f64| x_mm * MM_TO_PT;
    let to_pt_y = |y_mm: f64| (ph_mm - y_mm) * MM_TO_PT;

    let mut s = String::new();
    let _ = writeln!(s, "1 j 1 J");
    // Sheet outer margin frame
    let m = layout.margin_mm;
    let _ = writeln!(
        s,
        "0.75 w 0.15 0.15 0.18 RG {:.3} {:.3} {:.3} {:.3} re S",
        to_pt_x(m),
        to_pt_y(ph_mm - m),
        (pw_mm - 2.0 * m).max(1.0) * MM_TO_PT,
        (ph_mm - 2.0 * m).max(1.0) * MM_TO_PT
    );

    // Title block (bottom-right)
    let tb_w = 88.0_f64.min((pw_mm - 2.0 * m) * 0.48);
    let tb_h = 28.0_f64;
    let tb_x = pw_mm - m - tb_w;
    let tb_y = ph_mm - m - tb_h;
    let _ = writeln!(
        s,
        "0.6 w 0.15 0.15 0.18 RG {:.3} {:.3} {:.3} {:.3} re S",
        to_pt_x(tb_x),
        to_pt_y(tb_y + tb_h),
        tb_w * MM_TO_PT,
        tb_h * MM_TO_PT
    );
    let _ = writeln!(
        s,
        "0.4 w {:.3} {:.3} m {:.3} {:.3} l S {:.3} {:.3} m {:.3} {:.3} l S {:.3} {:.3} m {:.3} {:.3} l S",
        to_pt_x(tb_x),
        to_pt_y(tb_y + 10.0),
        to_pt_x(tb_x + tb_w),
        to_pt_y(tb_y + 10.0),
        to_pt_x(tb_x),
        to_pt_y(tb_y + 19.0),
        to_pt_x(tb_x + tb_w),
        to_pt_y(tb_y + 19.0),
        to_pt_x(tb_x + tb_w * 0.52),
        to_pt_y(tb_y + 10.0),
        to_pt_x(tb_x + tb_w * 0.52),
        to_pt_y(tb_y + tb_h)
    );
    draw_pdf_text(&mut s, "F2", 9.0, to_pt_x(tb_x + 2.5), to_pt_y(tb_y + 6.8), &layout.title_block.title);
    draw_pdf_text(
        &mut s,
        "F1",
        7.0,
        to_pt_x(tb_x + 2.5),
        to_pt_y(tb_y + 15.8),
        &format!("DWG: {}", layout.title_block.drawing_number),
    );
    draw_pdf_text(
        &mut s,
        "F1",
        7.0,
        to_pt_x(tb_x + tb_w * 0.54),
        to_pt_y(tb_y + 15.8),
        &format!("REV: {}  SCALE: {}", layout.title_block.revision, layout.title_block.scale_text),
    );
    draw_pdf_text(
        &mut s,
        "F1",
        6.5,
        to_pt_x(tb_x + 2.5),
        to_pt_y(tb_y + 24.5),
        &format!("AUTHOR: {}", layout.title_block.author),
    );
    draw_pdf_text(
        &mut s,
        "F1",
        6.5,
        to_pt_x(tb_x + tb_w * 0.54),
        to_pt_y(tb_y + 24.5),
        &format!("UNITS: {}  {}", doc.units.label(), layout.title_block.date),
    );

    let items: Vec<_> = doc
        .objects
        .values()
        .filter(|o| o.visible && doc.is_layer_effectively_visible(o.layer_id))
        .map(|o| {
            let col = o
                .color
                .or_else(|| doc.layers.get(o.layer_id).map(|l| l.color));
            (o.geometry.clone(), col)
        })
        .collect();

    for detail in &layout.details {
        let [rx, ry, rw, rh] = detail.rect_mm;
        if rw <= 2.0 || rh <= 2.0 {
            continue;
        }
        // Detail border
        let _ = writeln!(
            s,
            "0.45 w 0.65 0.67 0.72 RG [] 0 d {:.3} {:.3} {:.3} {:.3} re S",
            to_pt_x(rx),
            to_pt_y(ry + rh),
            rw * MM_TO_PT,
            rh * MM_TO_PT
        );
        draw_pdf_text(&mut s, "F2", 7.5, to_pt_x(rx + 2.5), to_pt_y(ry + 4.5), &detail.title);

        let res = make_2d_projection(
            &items,
            detail.projection,
            &Make2DOptions {
                include_hidden: detail.show_hidden_lines,
                ..Default::default()
            },
        );
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

        let map_pt = |px: f64, py: f64| -> (f64, f64) {
            let x_mm = cx_mm + (px - mid_x) * scale;
            let y_mm = cy_mm - (py - mid_y) * scale;
            (to_pt_x(x_mm), to_pt_y(y_mm))
        };

        // Clip drawing strictly to detail frame
        let _ = writeln!(
            s,
            "q {:.3} {:.3} {:.3} {:.3} re W n",
            to_pt_x(rx + 0.5),
            to_pt_y(ry + rh - 0.5),
            (rw - 1.0) * MM_TO_PT,
            (rh - 1.0) * MM_TO_PT
        );

        for seg in &res.segments {
            if seg.points_2d.len() < 2 {
                continue;
            }
            match seg.kind {
                Make2DLineKind::Visible => {
                    let _ = writeln!(s, "0.55 w [] 0 d 0.10 0.12 0.16 RG");
                }
                Make2DLineKind::Silhouette => {
                    let _ = writeln!(s, "0.95 w [] 0 d 0.04 0.05 0.07 RG");
                }
                Make2DLineKind::Hidden => {
                    let _ = writeln!(s, "0.35 w [3.5 2.5] 0 d 0.56 0.60 0.66 RG");
                }
            }
            let (sx, sy) = map_pt(seg.points_2d[0][0], seg.points_2d[0][1]);
            let _ = write!(s, "{sx:.3} {sy:.3} m");
            for pt in &seg.points_2d[1..] {
                let (x, y) = map_pt(pt[0], pt[1]);
                let _ = write!(s, " {x:.3} {y:.3} l");
            }
            let _ = writeln!(s, " S");
        }

        if detail.show_dimensions {
            for ann in &doc.annotations {
                if !doc.is_layer_effectively_visible(ann.layer_id) {
                    continue;
                }
                draw_pdf_annotation(&mut s, ann, detail.projection, &map_pt);
            }
        }

        let _ = writeln!(s, "Q");
    }

    build_pdf_14_document(pw_pt, ph_pt, &s)
}

/// Export the first document layout sheet (or a default A3 4-view sheet) as Vector PDF 1.4.
pub fn export_pdf_bytes(doc: &Document) -> Vec<u8> {
    if let Some(sheet) = doc.layouts.first() {
        export_layout_pdf_bytes(doc, sheet)
    } else {
        let fallback = LayoutSheet::standard_4_view(1, "Sheet 1", cad_doc::PageSize::A3);
        export_layout_pdf_bytes(doc, &fallback)
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

fn draw_pdf_annotation<F>(
    s: &mut String,
    ann: &AnnotationItem,
    proj: DrawingProjection,
    map_pt: &F,
) where
    F: Fn(f64, f64) -> (f64, f64),
{
    let Some(pos2) = project_ortho_point(ann.position, proj) else {
        return;
    };
    let (px, py) = map_pt(pos2[0], pos2[1]);
    if let Some(gdt) = &ann.gdt {
        let cell_h = 12.0;
        let sym_w = 14.0;
        let tol_txt = if gdt.diameter_zone {
            format!("D{:.3}", gdt.tolerance)
        } else {
            format!("{:.3}", gdt.tolerance)
        };
        let tol_w = (tol_txt.len() as f64 * 4.4 + 8.0).max(26.0);
        let datum_ws: Vec<f64> = gdt.datums.iter().map(|d| (d.len() as f64 * 4.5 + 8.0).max(14.0)).collect();
        let total_w = sym_w + tol_w + datum_ws.iter().sum::<f64>();
        let x0 = px;
        let y0 = py - cell_h * 0.5;
        let _ = writeln!(
            s,
            "0.45 w [] 0 d 0.1 0.1 0.15 RG 0.98 0.99 1.0 rg {x0:.2} {y0:.2} {total_w:.2} {cell_h:.2} re B"
        );
        let mut cx = x0 + sym_w;
        let _ = writeln!(s, "{cx:.2} {y0:.2} m {cx:.2} {:.2} l S", y0 + cell_h);
        draw_pdf_text(s, "F1", 6.5, x0 + 2.0, y0 + 3.8, gdt.symbol.short_code());
        draw_pdf_text(s, "F1", 7.0, cx + 3.5, y0 + 3.5, &tol_txt);
        cx += tol_w;
        for (i, d) in gdt.datums.iter().enumerate() {
            let _ = writeln!(s, "{cx:.2} {y0:.2} m {cx:.2} {:.2} l S", y0 + cell_h);
            draw_pdf_text(s, "F2", 7.0, cx + 4.0, y0 + 3.5, d);
            cx += datum_ws[i];
        }
    } else if ann.kind == cad_doc::AnnotationKind::Dimension {
        if let (Some(p1), Some(p2), Some(off)) = (ann.p1, ann.p2, ann.offset_pt) {
            if let (Some(q1), Some(q2), Some(qo)) = (
                project_ortho_point(p1, proj),
                project_ortho_point(p2, proj),
                project_ortho_point(off, proj),
            ) {
                let d12 = [q2[0] - q1[0], q2[1] - q1[1]];
                let len = (d12[0] * d12[0] + d12[1] * d12[1]).sqrt();
                if len > 1e-6 {
                    let dir = [d12[0] / len, d12[1] / len];
                    let rel = [qo[0] - q1[0], qo[1] - q1[1]];
                    let dot = rel[0] * dir[0] + rel[1] * dir[1];
                    let perp = [rel[0] - dir[0] * dot, rel[1] - dir[1] * dot];
                    let d1 = map_pt(q1[0] + perp[0], q1[1] + perp[1]);
                    let d2 = map_pt(q2[0] + perp[0], q2[1] + perp[1]);
                    let s1 = map_pt(q1[0], q1[1]);
                    let s2 = map_pt(q2[0], q2[1]);
                    let _ = writeln!(
                        s,
                        "0.35 w [] 0 d 0.12 0.35 0.65 RG {:.2} {:.2} m {:.2} {:.2} l {:.2} {:.2} l {:.2} {:.2} l S",
                        s1.0, s1.1, d1.0, d1.1, d2.0, d2.1, s2.0, s2.1
                    );
                }
            }
        }
        draw_pdf_text(s, "F1", 7.0, px + 2.0, py + 2.0, &ann.text);
    } else if !ann.text.is_empty() {
        draw_pdf_text(s, "F1", 7.0, px + 2.0, py + 2.0, &ann.text);
    }
}

fn draw_pdf_text(s: &mut String, font: &str, size: f64, x_pt: f64, y_pt: f64, text: &str) {
    let esc = pdf_escape(text);
    let _ = writeln!(
        s,
        "BT /{font} {size:.2} Tf 0.08 0.09 0.12 rg 1 0 0 1 {x_pt:.3} {y_pt:.3} Tm ({esc}) Tj ET"
    );
}

fn pdf_escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '\\' => out.push_str("\\\\"),
            '(' => out.push_str("\\("),
            ')' => out.push_str("\\)"),
            '\x20'..='\x7e' => out.push(ch),
            _ => out.push('?'),
        }
    }
    out
}

fn build_pdf_14_document(pw_pt: f64, ph_pt: f64, content_stream: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(content_stream.len() + 1024);
    out.extend_from_slice(b"%PDF-1.4\n%\xE2\xE3\xCF\xD3\n");

    let mut offsets = [0usize; 6];
    let objects = [
        "1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n".to_string(),
        "2 0 obj\n<< /Type /Pages /Kids [3 0 R] /Count 1 >>\nendobj\n".to_string(),
        format!(
            "3 0 obj\n<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {pw_pt:.3} {ph_pt:.3}] /Resources << /Font << /F1 4 0 R /F2 5 0 R >> >> /Contents 6 0 R >>\nendobj\n"
        ),
        "4 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>\nendobj\n".to_string(),
        "5 0 obj\n<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>\nendobj\n".to_string(),
        format!(
            "6 0 obj\n<< /Length {} >>\nstream\n{content_stream}\nendstream\nendobj\n",
            content_stream.len()
        ),
    ];

    for (idx, body) in objects.iter().enumerate() {
        offsets[idx] = out.len();
        out.extend_from_slice(body.as_bytes());
    }

    let xref_offset = out.len();
    let _ = write_to_vec(&mut out, format!("xref\n0 {}\n", objects.len() + 1));
    out.extend_from_slice(b"0000000000 65535 f \n");
    for off in offsets {
        let _ = write_to_vec(&mut out, format!("{off:010} 00000 n \n"));
    }
    let _ = write_to_vec(
        &mut out,
        format!(
            "trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{xref_offset}\n%%EOF\n",
            objects.len() + 1
        ),
    );
    out
}

fn write_to_vec(buf: &mut Vec<u8>, s: String) -> Result<(), ()> {
    buf.extend_from_slice(s.as_bytes());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_doc::{GdtSpec, GdtSymbol, PageSize};
    use cad_geom::{BRep, Geometry};

    #[test]
    fn vector_pdf_has_valid_header_xref_hidden_dashes_and_gdt() {
        let mut doc = Document::new();
        doc.add_object(
            "Cube",
            Geometry::BRep(BRep::from_box(DVec3::ZERO, DVec3::splat(4.0))),
        );
        doc.add_gdt_annotation(
            GdtSpec {
                symbol: GdtSymbol::Flatness,
                tolerance: 0.025,
                diameter_zone: false,
                modifier: None,
                datums: vec!["A".to_string()],
            },
            DVec3::new(0.0, 0.0, 2.0),
        );
        let sheet = LayoutSheet::standard_4_view(1, "Sheet 1", PageSize::A3);
        let pdf = export_layout_pdf_bytes(&doc, &sheet);
        let text = String::from_utf8_lossy(&pdf);

        assert!(text.starts_with("%PDF-1.4\n"));
        assert!(text.contains("/BaseFont /Helvetica"));
        assert!(text.contains("/MediaBox [0 0 "));
        assert!(text.contains("[3.5 2.5] 0 d"), "expected dashed hidden lines in isometric/ortho");
        assert!(text.contains("FLT"));
        assert!(text.contains("0.025"));
        assert!(text.ends_with("%%EOF\n"));

        // Verify xref table points directly to "1 0 obj" and "6 0 obj"
        let xref_pos = text.find("xref\n").unwrap();
        let entry1_line = text[xref_pos..]
            .lines()
            .find(|l| l.ends_with("00000 n "))
            .unwrap();
        let off1: usize = entry1_line[..10].parse().unwrap();
        assert!(pdf[off1..].starts_with(b"1 0 obj\n"));
    }
}
