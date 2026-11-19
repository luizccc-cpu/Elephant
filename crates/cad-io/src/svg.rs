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
