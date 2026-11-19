use std::fmt::Write;

use cad_doc::Document;
use cad_geom::Geometry;

/// Export visible 2D/3D curves (projected to XY) and annotations to an SVG document.
pub fn export_svg_string(doc: &Document) -> String {
    let mut polylines: Vec<(String, Vec<[f64; 2]>)> = Vec::new();
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;

    let mut expand = |x: f64, y: f64| {
        if x < min_x {
            min_x = x;
        }
        if y < min_y {
            min_y = y;
        }
        if x > max_x {
            max_x = x;
        }
        if y > max_y {
            max_y = y;
        }
    };

    for (_, obj) in &doc.objects {
        if !obj.visible {
            continue;
        }
        let Some(layer) = doc.layers.get(obj.layer_id) else {
            continue;
        };
        if !layer.visible {
            continue;
        }
        let col = obj.color.unwrap_or(layer.color);
        let hex = format!("#{:02x}{:02x}{:02x}", col[0], col[1], col[2]);

        let curve_to_pts = |curve: &cad_geom::NurbsCurve| -> Vec<[f64; 2]> {
            if curve.degree == 1 {
                curve
                    .control_points
                    .iter()
                    .map(|c| [c.position.x, c.position.y])
                    .collect()
            } else {
                curve
                    .sample_polyline(48)
                    .into_iter()
                    .map(|p| [p.x, p.y])
                    .collect()
            }
        };

        match &obj.geometry {
            Geometry::Curve(curve) => {
                let pts = curve_to_pts(curve);
                if pts.len() >= 2 {
                    for &[x, y] in &pts {
                        expand(x, y);
                    }
                    polylines.push((hex, pts));
                }
            }
            Geometry::BRep(brep) => {
                for edge in &brep.edges {
                    let pts = curve_to_pts(&edge.curve);
                    if pts.len() >= 2 {
                        for &[x, y] in &pts {
                            expand(x, y);
                        }
                        polylines.push((hex.clone(), pts));
                    }
                }
            }
            _ => {}
        }
    }

    if !min_x.is_finite() {
        min_x = -10.0;
        min_y = -10.0;
        max_x = 10.0;
        max_y = 10.0;
    }
    let w = (max_x - min_x).max(1.0);
    let h = (max_y - min_y).max(1.0);
    let pad = (w.max(h)) * 0.05;
    let vx = min_x - pad;
    let vy = -max_y - pad;
    let vw = w + pad * 2.0;
    let vh = h + pad * 2.0;
    let stroke_w = (vw.max(vh) * 0.0025).max(0.02);

    let mut out = String::new();
    let _ = writeln!(
        out,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vx:.4} {vy:.4} {vw:.4} {vh:.4}" width="1000" height="1000">"#
    );
    let _ = writeln!(
        out,
        r##"  <rect x="{vx:.4}" y="{vy:.4}" width="{vw:.4}" height="{vh:.4}" fill="#ffffff"/>"##
    );

    for (hex, pts) in &polylines {
        let stroke = if hex == "#ffffff" { "#222222" } else { hex.as_str() };
        let _ = write!(out, r#"  <polyline fill="none" stroke="{stroke}" stroke-width="{stroke_w:.4}" points=""#);
        for (i, p) in pts.iter().enumerate() {
            if i > 0 {
                out.push(' ');
            }
            let _ = write!(out, "{:.4},{:.4}", p[0], -p[1]);
        }
        let _ = writeln!(out, r#""/>"#);
    }

    let font_size = (vw.max(vh) * 0.018).max(0.2);
    for ann in &doc.annotations {
        let escaped = ann
            .text
            .replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;");
        let _ = writeln!(
            out,
            r##"  <text x="{:.4}" y="{:.4}" font-family="sans-serif" font-size="{font_size:.4}" fill="#222222">{escaped}</text>"##,
            ann.position.x, -ann.position.y
        );
    }

    let _ = writeln!(out, "</svg>");
    out
}

/// Export a paper-space `LayoutSheet` to pure vector SVG including projected visible/hidden lines and title block.
pub fn export_layout_svg_string(doc: &Document, layout: &cad_doc::LayoutSheet) -> String {
    use cad_geom::{Make2DLineKind, Make2DOptions, make_2d_projection};
    let (pw, ph) = layout.page_size.dimensions_mm(layout.landscape);
    let m = layout.margin_mm;
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

    let mut out = String::new();
    let _ = writeln!(
        out,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {pw:.3} {ph:.3}" width="{pw:.1}mm" height="{ph:.1}mm">"#
    );
    let _ = writeln!(out, r##"  <rect x="0" y="0" width="{pw:.3}" height="{ph:.3}" fill="#ffffff"/>"##);
    let _ = writeln!(
        out,
        r##"  <rect x="{m:.3}" y="{m:.3}" width="{:.3}" height="{:.3}" fill="none" stroke="#1e222a" stroke-width="0.45"/>"##,
        (pw - 2.0 * m).max(1.0),
        (ph - 2.0 * m).max(1.0)
    );

    let tb_w = 88.0_f64.min((pw - 2.0 * m) * 0.48);
    let tb_h = 28.0_f64;
    let tb_x = pw - m - tb_w;
    let tb_y = ph - m - tb_h;
    let _ = writeln!(
        out,
        r##"  <rect x="{tb_x:.3}" y="{tb_y:.3}" width="{tb_w:.3}" height="{tb_h:.3}" fill="none" stroke="#1e222a" stroke-width="0.35"/>"##
    );
    let _ = writeln!(
        out,
        r##"  <text x="{:.3}" y="{:.3}" font-family="sans-serif" font-size="3.4" font-weight="bold" fill="#15181e">{}</text>"##,
        tb_x + 2.5,
        tb_y + 6.5,
        escape_xml(&layout.title_block.title)
    );
    let _ = writeln!(
        out,
        r##"  <text x="{:.3}" y="{:.3}" font-family="sans-serif" font-size="2.4" fill="#333842">DWG: {} | REV: {} | UNITS: {}</text>"##,
        tb_x + 2.5,
        tb_y + 15.5,
        escape_xml(&layout.title_block.drawing_number),
        escape_xml(&layout.title_block.revision),
        doc.units.label()
    );

    for detail in &layout.details {
        let [rx, ry, rw, rh] = detail.rect_mm;
        if rw <= 2.0 || rh <= 2.0 {
            continue;
        }
        let _ = writeln!(
            out,
            r##"  <rect x="{rx:.3}" y="{ry:.3}" width="{rw:.3}" height="{rh:.3}" fill="none" stroke="#9aa0aa" stroke-width="0.25"/>"##
        );
        let _ = writeln!(
            out,
            r##"  <text x="{:.3}" y="{:.3}" font-family="sans-serif" font-size="2.6" font-weight="bold" fill="#20242c">{}</text>"##,
            rx + 2.5,
            ry + 4.2,
            escape_xml(&detail.title)
        );

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
        let cx = rx + rw * 0.5;
        let cy = ry + 6.0 + inner_h * 0.5;

        for seg in &res.segments {
            if seg.points_2d.len() < 2 {
                continue;
            }
            let (stroke, width, dash) = match seg.kind {
                Make2DLineKind::Visible => ("#181c22", "0.22", ""),
                Make2DLineKind::Silhouette => ("#090b0f", "0.38", ""),
                Make2DLineKind::Hidden => ("#858d99", "0.14", r#" stroke-dasharray="1.8,1.2""#),
            };
            let _ = write!(
                out,
                r#"  <polyline fill="none" stroke="{stroke}" stroke-width="{width}"{dash} points=""#
            );
            for (i, pt) in seg.points_2d.iter().enumerate() {
                if i > 0 {
                    out.push(' ');
                }
                let x = cx + (pt[0] - mid_x) * scale;
                let y = cy - (pt[1] - mid_y) * scale;
                let _ = write!(out, "{x:.3},{y:.3}");
            }
            let _ = writeln!(out, r#""/>"#);
        }
    }

    let _ = writeln!(out, "</svg>");
    out
}

fn escape_xml(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::{DVec3, NurbsCurve};

    #[test]
    fn svg_export_contains_polyline_and_viewbox() {
        let mut doc = Document::new();
        doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(10.0, 5.0, 0.0))),
        );
        let svg = export_svg_string(&doc);
        assert!(svg.contains("<svg"));
        assert!(svg.contains("<polyline"));
        assert!(svg.contains("10.0000,-5.0000"));
    }
}
