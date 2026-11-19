use std::collections::{HashMap, HashSet};
use std::io::Cursor;

use cad_doc::{AnnotationItem, AnnotationKind, Document, Layer, LayerId};
use cad_geom::{ControlPoint, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh};
use cadmpeg_codec_rhino::RhinoCodec;
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use cadmpeg_ir::geometry::nurbs::{NurbsPoleGrid, NurbsPoles3};
use cadmpeg_ir::geometry::{
    CurveGeometry, SolvedCurveGeometry, SolvedSurfaceGeometry, SurfaceGeometry,
};
use cadmpeg_ir::semantic_annotations::SemanticAnnotationKind;

use crate::error::IoError;

fn parse_rhino_hex_rgba(hex: &str) -> Option<[u8; 4]> {
    if hex.len() < 6 {
        return None;
    }
    let r = u8::from_str_radix(&hex[0..2], 16).ok()?;
    let g = u8::from_str_radix(&hex[2..4], 16).ok()?;
    let b = u8::from_str_radix(&hex[4..6], 16).ok()?;
    Some([r, g, b, 255])
}

/// Import layers, colors, meshes, standalone curves, surfaces, and points from Rhino `.3dm` bytes (V5–V8).
pub fn import_3dm_bytes(bytes: &[u8], doc: &mut Document) -> Result<usize, IoError> {
    let decoded = RhinoCodec
        .decode(&mut Cursor::new(bytes), &DecodeOptions::default())
        .map_err(|e| IoError::InvalidFormat(format!("3DM decode failed: {e}")))?;

    let ir = decoded.ir();
    let mut layer_by_uuid: HashMap<String, LayerId> = HashMap::new();
    let mut layer_by_name: HashMap<String, LayerId> = HashMap::new();
    let mut material_colors: HashMap<i64, [u8; 4]> = HashMap::new();
    let mut object_material_colors: HashMap<String, [u8; 4]> = HashMap::new();

    if let Some(ns) = ir.native.namespace("rhino") {
        // Parse materials (archive_index -> base/diffuse color)
        if let Some(mats) = ns.arenas().get("materials") {
            for rec in mats {
                let Ok(fields) = rec.fields() else {
                    continue;
                };
                let idx = fields.get("archive_index").and_then(|v| v.as_i64());
                let pb_color = fields
                    .get("physically_based")
                    .and_then(|pb| pb.get("base_color"))
                    .and_then(|arr| arr.as_array())
                    .and_then(|a| {
                        if a.len() >= 3 {
                            Some([
                                (a[0].as_f64()? * 255.0).round().clamp(0.0, 255.0) as u8,
                                (a[1].as_f64()? * 255.0).round().clamp(0.0, 255.0) as u8,
                                (a[2].as_f64()? * 255.0).round().clamp(0.0, 255.0) as u8,
                                255,
                            ])
                        } else {
                            None
                        }
                    });
                let diff_color = fields
                    .get("diffuse")
                    .and_then(|v| v.as_str())
                    .and_then(parse_rhino_hex_rgba);
                if let (Some(i), Some(col)) = (idx, pb_color.or(diff_color)) {
                    material_colors.insert(i, col);
                }
            }
        }

        // Parse object_presentation (object_id -> material color if assigned)
        if let Some(obj_pres) = ns.arenas().get("object_presentation") {
            for rec in obj_pres {
                let Ok(fields) = rec.fields() else {
                    continue;
                };
                let obj_uuid = fields
                    .get("object_uuid")
                    .or_else(|| fields.get("source_uuid"))
                    .and_then(|v| v.as_str());
                let mat_idx = fields.get("material_index").and_then(|v| v.as_i64());
                if let (Some(uuid), Some(idx)) = (obj_uuid, mat_idx) {
                    if let Some(&col) = material_colors.get(&idx) {
                        object_material_colors.insert(uuid.to_string(), col);
                    }
                }
            }
        }

        // Parse Rhino layers (name, color, visible, locked, source_uuid)
        if let Some(layers) = ns.arenas().get("layers") {
            let mut parsed_layers: Vec<(i64, String, [u8; 4], bool, bool, Option<String>)> =
                Vec::new();
            for rec in layers {
                let Ok(fields) = rec.fields() else {
                    continue;
                };
                let archive_index = fields
                    .get("archive_index")
                    .and_then(|v| v.as_i64())
                    .unwrap_or(0);
                let name = fields
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("Layer")
                    .to_string();
                let mut color = fields
                    .get("color")
                    .and_then(|v| v.as_str())
                    .and_then(parse_rhino_hex_rgba)
                    .unwrap_or([215, 220, 228, 255]);
                if let Some(mat_idx) = fields.get("material_index").and_then(|v| v.as_i64()) {
                    if mat_idx >= 0 {
                        if let Some(&mcol) = material_colors.get(&mat_idx) {
                            color = mcol;
                        }
                    }
                }
                let visible = fields
                    .get("visible")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(true);
                let locked = fields
                    .get("locked")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(false);
                let uuid = fields
                    .get("source_uuid")
                    .and_then(|v| v.as_str())
                    .map(String::from);
                parsed_layers.push((archive_index, name, color, visible, locked, uuid));
            }
            parsed_layers.sort_by_key(|l| l.0);

            if !parsed_layers.is_empty() {
                doc.layers.clear();
                let mut first_id = None;
                for (_, name, color, visible, locked, uuid) in parsed_layers {
                    let lid = doc.layers.insert(Layer {
                        name: name.clone(),
                        color,
                        visible,
                        locked,
                        parent: None,
                    });
                    if first_id.is_none() {
                        first_id = Some(lid);
                    }
                    layer_by_name.insert(name, lid);
                    if let Some(u) = uuid {
                        layer_by_uuid.insert(u, lid);
                    }
                }
                if let Some(fid) = first_id {
                    doc.active_layer = fid;
                }
            }
        }
    }

    let default_layer = doc.active_layer;
    let resolve_layer = |layer_key: Option<&str>| -> LayerId {
        if let Some(l) = layer_key {
            if let Some(&lid) = layer_by_uuid.get(l) {
                return lid;
            }
            if let Some(&lid) = layer_by_name.get(l) {
                return lid;
            }
        }
        default_layer
    };

    let resolve_color = |obj_id: &str, rgb_opt: Option<[u8; 4]>| -> Option<[u8; 4]> {
        if let Some(&mcol) = object_material_colors.get(obj_id) {
            return Some(mcol);
        }
        if let Some(rgb) = rgb_opt {
            if rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0 {
                return Some(rgb);
            }
        }
        None
    };

    let mut added = 0usize;

    // 1. Import tessellated meshes (covers Mesh objects and B-Rep display meshes)
    for tess in &ir.model.tessellations {
        let verts = tess.vertices();
        let tris = tess.triangles();
        if verts.is_empty() || tris.is_empty() {
            continue;
        }
        let positions: Vec<[f64; 3]> = verts.iter().map(|p| p.get().into()).collect();
        let mut normals = vec![[0.0, 0.0, 1.0]; positions.len()];
        let mut accum = vec![DVec3::ZERO; positions.len()];
        for &[i0, i1, i2] in &tris {
            let p0 = DVec3::from_array(positions[i0 as usize]);
            let p1 = DVec3::from_array(positions[i1 as usize]);
            let p2 = DVec3::from_array(positions[i2 as usize]);
            let fn_ = (p1 - p0).cross(p2 - p0);
            accum[i0 as usize] += fn_;
            accum[i1 as usize] += fn_;
            accum[i2 as usize] += fn_;
        }
        for (i, a) in accum.into_iter().enumerate() {
            let n = a.normalize_or_zero();
            normals[i] = if n.length_squared() > 0.0 {
                n.to_array()
            } else {
                [0.0, 0.0, 1.0]
            };
        }
        let mesh = TriangleMesh {
            positions,
            normals,
            indices: tris,
        };
        let lid = resolve_layer(
            tess.source_object
                .as_ref()
                .and_then(|s| s.layer.as_deref()),
        );
        let id = doc.add_object_on_layer("Import3DM_Mesh", lid, Geometry::Mesh(mesh));
        if let Some(obj) = doc.objects.get_mut(id) {
            obj.name = tess
                .source_object
                .as_ref()
                .and_then(|s| s.name.as_ref().map(|n| n.to_string()));
            obj.color = tess.source_object.as_ref().and_then(|s| {
                let rgb = s.color.map(|c| {
                    [
                        (c.r() * 255.0).round().clamp(0.0, 255.0) as u8,
                        (c.g() * 255.0).round().clamp(0.0, 255.0) as u8,
                        (c.b() * 255.0).round().clamp(0.0, 255.0) as u8,
                        255,
                    ]
                });
                resolve_color(s.object_id.as_str(), rgb)
            });
        }
        added += 1;
    }

    // 2. Import standalone curves (exclude curves owned by B-Rep edges when tessellations exist)
    let edge_curve_ids: HashSet<_> = ir.model.edges.iter().filter_map(|e| e.curve()).collect();
    let has_tess = !ir.model.tessellations.is_empty();
    for curve in &ir.model.curves {
        if has_tess && edge_curve_ids.contains(&curve.id) && curve.source_object.is_none() {
            continue;
        }
        let solved = match &curve.geometry {
            CurveGeometry::Solved(s) => Some(s),
            CurveGeometry::Procedural { cache, .. } => cache.as_ref(),
        };
        if let Some(g) = solved.and_then(convert_curve) {
            let lid = resolve_layer(
                curve
                    .source_object
                    .as_ref()
                    .and_then(|s| s.layer.as_deref()),
            );
            let id = doc.add_object_on_layer("Import3DM_Curve", lid, g);
            if let Some(obj) = doc.objects.get_mut(id) {
                obj.name = curve
                    .source_object
                    .as_ref()
                    .and_then(|s| s.name.as_ref().map(|n| n.to_string()));
                obj.color = curve.source_object.as_ref().and_then(|s| {
                    let rgb = s.color.map(|c| {
                        [
                            (c.r() * 255.0).round().clamp(0.0, 255.0) as u8,
                            (c.g() * 255.0).round().clamp(0.0, 255.0) as u8,
                            (c.b() * 255.0).round().clamp(0.0, 255.0) as u8,
                            255,
                        ]
                    });
                    resolve_color(s.object_id.as_str(), rgb)
                });
            }
            added += 1;
        }
    }

    // 3. Import standalone NURBS surfaces when no tessellations were provided
    if !has_tess {
        for surf in &ir.model.surfaces {
            let solved = match &surf.geometry {
                SurfaceGeometry::Solved(s) => Some(s),
                SurfaceGeometry::Procedural { cache, .. } => cache.as_ref(),
            };
            if let Some(g) = solved.and_then(convert_surface) {
                let lid = resolve_layer(
                    surf.source_object
                        .as_ref()
                        .and_then(|s| s.layer.as_deref()),
                );
                let id = doc.add_object_on_layer("Import3DM_Surface", lid, g);
                if let Some(obj) = doc.objects.get_mut(id) {
                    obj.color = surf.source_object.as_ref().and_then(|s| {
                        let rgb = s.color.map(|c| {
                            [
                                (c.r() * 255.0).round().clamp(0.0, 255.0) as u8,
                                (c.g() * 255.0).round().clamp(0.0, 255.0) as u8,
                                (c.b() * 255.0).round().clamp(0.0, 255.0) as u8,
                                255,
                            ]
                        });
                        resolve_color(s.object_id.as_str(), rgb)
                    });
                }
                added += 1;
            }
        }
    }

    // 4. Import standalone points
    for pt in &ir.model.points {
        if pt.source_object.is_some() || (!has_tess && ir.model.bodies.is_empty()) {
            let p: [f64; 3] = pt.position().get().into();
            let lid = resolve_layer(pt.source_object.as_ref().and_then(|s| s.layer.as_deref()));
            let id = doc.add_object_on_layer(
                "Import3DM_Point",
                lid,
                Geometry::Point(DVec3::from_array(p)),
            );
            if let Some(obj) = doc.objects.get_mut(id) {
                obj.color = pt.source_object.as_ref().and_then(|s| {
                    let rgb = s.color.map(|c| {
                        [
                            (c.r() * 255.0).round().clamp(0.0, 255.0) as u8,
                            (c.g() * 255.0).round().clamp(0.0, 255.0) as u8,
                            (c.b() * 255.0).round().clamp(0.0, 255.0) as u8,
                            255,
                        ]
                    });
                    resolve_color(s.object_id.as_str(), rgb)
                });
            }
            added += 1;
        }
    }

    // 5. Import semantic annotations (dimensions, text, leaders)
    for ann in &ir.model.semantic_annotations {
        let text = ann
            .text_override
            .clone()
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| {
                if !ann.text.is_empty() {
                    ann.text.join(" ")
                } else if let Some(v) = ann.value {
                    format!("{:.2}", v.get())
                } else {
                    String::new()
                }
            });
        if text.is_empty() {
            continue;
        }
        if let Some(pos) = ann.position {
            let p: [f64; 3] = pos.get().into();
            let kind = match ann.kind {
                SemanticAnnotationKind::Dimension => AnnotationKind::Dimension,
                SemanticAnnotationKind::Leader => AnnotationKind::Leader,
                _ => AnnotationKind::Text,
            };
            doc.add_annotation(AnnotationItem {
                kind,
                layer_id: default_layer,
                color: None,
                text,
                position: DVec3::from_array(p),
                p1: None,
                p2: None,
                offset_pt: None,
            });
            added += 1;
        }
    }

    Ok(added)
}

fn convert_curve(solved: &SolvedCurveGeometry) -> Option<Geometry> {
    match solved {
        SolvedCurveGeometry::Nurbs(nc) => {
            let (deg, knots, poles, _) = nc.clone().into_parts();
            let cps = match poles {
                NurbsPoles3::Polynomial { points } => points
                    .into_iter()
                    .map(|p| ControlPoint::unweighted(DVec3::from_array(p.get().into())))
                    .collect(),
                NurbsPoles3::Rational { points } => points
                    .into_iter()
                    .map(|p| {
                        ControlPoint::new(DVec3::from_array(p.point.get().into()), p.weight.get())
                    })
                    .collect(),
            };
            NurbsCurve::new(deg as usize, knots.as_slice().to_vec(), cps)
                .ok()
                .map(Geometry::Curve)
        }
        SolvedCurveGeometry::Polyline(pl) => {
            let pts: Vec<DVec3> = pl
                .points()
                .map(|p| DVec3::from_array(p.get().into()))
                .collect();
            NurbsCurve::polyline(&pts).ok().map(Geometry::Curve)
        }
        SolvedCurveGeometry::Circle(c) => {
            let center = DVec3::from_array(c.center().get().into());
            let mut plane = Plane::world_xy();
            plane.origin = center;
            NurbsCurve::circle(&plane, c.radius().get())
                .ok()
                .map(Geometry::Curve)
        }
        SolvedCurveGeometry::Transformed(placed) => {
            let mut g = convert_curve(placed.basis())?;
            let m = placed.transform().rows();
            let mat = cad_geom::DMat4::from_cols_array_2d(&m).transpose();
            g.transform(&mat);
            Some(g)
        }
        _ => None,
    }
}

fn convert_surface(solved: &SolvedSurfaceGeometry) -> Option<Geometry> {
    match solved {
        SolvedSurfaceGeometry::Nurbs(ns) => {
            let cps: Vec<ControlPoint> = match ns.pole_grid() {
                NurbsPoleGrid::Polynomial { rows } => rows
                    .iter()
                    .flat_map(|row| row.iter())
                    .map(|p| ControlPoint::unweighted(DVec3::from_array(p.get().into())))
                    .collect(),
                NurbsPoleGrid::Rational { rows } => rows
                    .iter()
                    .flat_map(|row| row.iter())
                    .map(|p| {
                        ControlPoint::new(DVec3::from_array(p.point.get().into()), p.weight.get())
                    })
                    .collect(),
            };
            NurbsSurface::new(
                ns.u_degree() as usize,
                ns.v_degree() as usize,
                ns.u_knots().as_slice().to_vec(),
                ns.v_knots().as_slice().to_vec(),
                ns.u_count(),
                ns.v_count(),
                cps,
            )
            .ok()
            .map(Geometry::Surface)
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_3dm_bytes() {
        let mut doc = Document::new();
        let err = import_3dm_bytes(b"not a 3dm file", &mut doc).unwrap_err();
        assert!(matches!(err, IoError::InvalidFormat(_)));
    }

    #[test]
    fn parses_rhino_hex_rgba_colors() {
        assert_eq!(parse_rhino_hex_rgba("0000ff00"), Some([0, 0, 255, 255]));
        assert_eq!(parse_rhino_hex_rgba("b4e2de00"), Some([180, 226, 222, 255]));
    }
}
