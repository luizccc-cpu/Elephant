use std::collections::HashMap;
use std::fmt::Write;

use cad_doc::{Document, LayerId};
use cad_geom::{DVec3, Geometry, NurbsCurve, Plane};

use crate::error::IoError;

/// Export visible points and curves to ASCII DXF (`ENTITIES` section).
pub fn export_dxf_string(doc: &Document) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "0\nSECTION\n2\nENTITIES");

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
        let lname = &layer.name;
        match &obj.geometry {
            Geometry::Point(p) => {
                let _ = writeln!(
                    out,
                    "0\nPOINT\n8\n{lname}\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}",
                    p.x, p.y, p.z
                );
            }
            Geometry::Curve(curve) => {
                let pts = if curve.degree == 1 {
                    curve.control_points.iter().map(|c| c.position).collect()
                } else {
                    curve.sample_polyline(48)
                };
                if pts.len() == 2 {
                    let a = pts[0];
                    let b = pts[1];
                    let _ = writeln!(
                        out,
                        "0\nLINE\n8\n{lname}\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}\n11\n{:.6}\n21\n{:.6}\n31\n{:.6}",
                        a.x, a.y, a.z, b.x, b.y, b.z
                    );
                } else if pts.len() >= 2 {
                    let _ = writeln!(out, "0\nPOLYLINE\n8\n{lname}\n66\n1\n70\n8");
                    for p in pts {
                        let _ = writeln!(
                            out,
                            "0\nVERTEX\n8\n{lname}\n10\n{:.6}\n20\n{:.6}\n30\n{:.6}",
                            p.x, p.y, p.z
                        );
                    }
                    let _ = writeln!(out, "0\nSEQEND");
                }
            }
            _ => {}
        }
    }

    let _ = writeln!(out, "0\nENDSEC\n0\nEOF");
    out
}

/// Import ASCII DXF entities (`LINE`, `LWPOLYLINE`, `CIRCLE`, `ARC`, `POINT`).
pub fn import_dxf_str(input: &str, doc: &mut Document) -> Result<usize, IoError> {
    let lines: Vec<&str> = input.lines().map(|l| l.trim()).collect();
    let mut layer_map: HashMap<String, LayerId> = doc
        .layers
        .iter()
        .map(|(id, l)| (l.name.clone(), id))
        .collect();

    let mut resolve_layer = |name: &str, doc: &mut Document| -> LayerId {
        if let Some(&id) = layer_map.get(name) {
            return id;
        }
        let id = doc.add_layer(name, [180, 200, 230, 255]);
        layer_map.insert(name.to_string(), id);
        id
    };

    let mut i = 0;
    let mut in_entities = false;
    let mut added = 0;

    while i + 1 < lines.len() {
        let code = lines[i];
        let val = lines[i + 1];
        i += 2;

        if code == "2" && val == "ENTITIES" {
            in_entities = true;
            continue;
        }
        if !in_entities {
            continue;
        }
        if code == "0" && val == "ENDSEC" {
            break;
        }
        if code == "0" {
            let etype = val;
            let mut pairs: Vec<(&str, &str)> = Vec::new();
            while i + 1 < lines.len() && lines[i] != "0" {
                pairs.push((lines[i], lines[i + 1]));
                i += 2;
            }
            let mut layer_name = "Default";
            let mut x0 = 0.0_f64;
            let mut y0 = 0.0_f64;
            let mut z0 = 0.0_f64;
            let mut x1 = 0.0_f64;
            let mut y1 = 0.0_f64;
            let mut z1 = 0.0_f64;
            let mut r = 0.0_f64;
            let mut a0 = 0.0_f64;
            let mut a1 = 360.0_f64;
            let mut closed = false;
            let mut poly_pts: Vec<DVec3> = Vec::new();
            let mut cur_x: Option<f64> = None;

            for (c, v) in pairs {
                match c {
                    "8" if !v.is_empty() => layer_name = v,
                    "10" => {
                        if let Ok(n) = v.parse::<f64>() {
                            x0 = n;
                            if etype == "LWPOLYLINE" {
                                cur_x = Some(n);
                            }
                        }
                    }
                    "20" => {
                        if let Ok(n) = v.parse::<f64>() {
                            y0 = n;
                            if etype == "LWPOLYLINE" {
                                if let Some(px) = cur_x.take() {
                                    poly_pts.push(DVec3::new(px, n, 0.0));
                                }
                            }
                        }
                    }
                    "30" => z0 = v.parse().unwrap_or(0.0),
                    "11" => x1 = v.parse().unwrap_or(0.0),
                    "21" => y1 = v.parse().unwrap_or(0.0),
                    "31" => z1 = v.parse().unwrap_or(0.0),
                    "40" => r = v.parse().unwrap_or(0.0),
                    "50" => a0 = v.parse().unwrap_or(0.0),
                    "51" => a1 = v.parse().unwrap_or(360.0),
                    "70" => {
                        if let Ok(flags) = v.parse::<i32>() {
                            closed = (flags & 1) != 0;
                        }
                    }
                    _ => {}
                }
            }

            let lid = resolve_layer(layer_name, doc);
            match etype {
                "POINT" => {
                    doc.add_object_on_layer("DXF_Point", lid, Geometry::Point(DVec3::new(x0, y0, z0)));
                    added += 1;
                }
                "LINE" => {
                    doc.add_object_on_layer(
                        "DXF_Line",
                        lid,
                        Geometry::Curve(NurbsCurve::line(
                            DVec3::new(x0, y0, z0),
                            DVec3::new(x1, y1, z1),
                        )),
                    );
                    added += 1;
                }
                "LWPOLYLINE" => {
                    if closed && poly_pts.len() >= 2 {
                        poly_pts.push(poly_pts[0]);
                    }
                    if let Ok(poly) = NurbsCurve::polyline(&poly_pts) {
                        doc.add_object_on_layer("DXF_Polyline", lid, Geometry::Curve(poly));
                        added += 1;
                    }
                }
                "CIRCLE" => {
                    let mut plane = Plane::world_xy();
                    plane.origin = DVec3::new(x0, y0, z0);
                    if let Ok(c) = NurbsCurve::circle(&plane, r) {
                        doc.add_object_on_layer("DXF_Circle", lid, Geometry::Curve(c));
                        added += 1;
                    }
                }
                "ARC" => {
                    let mut plane = Plane::world_xy();
                    plane.origin = DVec3::new(x0, y0, z0);
                    let mut sweep = a1 - a0;
                    if sweep <= 0.0 {
                        sweep += 360.0;
                    }
                    if let Ok(arc) = NurbsCurve::arc_from_center(
                        &plane,
                        plane.origin,
                        r,
                        a0.to_radians(),
                        sweep.to_radians(),
                    ) {
                        doc.add_object_on_layer("DXF_Arc", lid, Geometry::Curve(arc));
                        added += 1;
                    }
                }
                _ => {}
            }
        }
    }

    Ok(added)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dxf_round_trip_line() {
        let mut doc = Document::new();
        doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(5.0, 2.0, 0.0))),
        );
        let dxf = export_dxf_string(&doc);
        let mut doc2 = Document::new();
        let n = import_dxf_str(&dxf, &mut doc2).unwrap();
        assert_eq!(n, 1);
        assert_eq!(doc2.objects.len(), 1);
    }
}
