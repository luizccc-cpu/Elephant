use std::fmt::Write as _;

use cad_doc::Document;
use cad_geom::{DVec3, Geometry, TriangleMesh};

use crate::error::IoError;

/// Export all visible meshes, tessellated surfaces, and curves in `doc` as Wavefront OBJ text.
pub fn export_obj_string(doc: &Document) -> String {
    let mut out = String::from("# Exported by Elephant CAD\n");
    let mut v_offset: u32 = 1;
    let mut vn_offset: u32 = 1;

    for (idx, (_, obj)) in doc.objects.iter().enumerate() {
        if !obj.visible {
            continue;
        }
        if let Some(layer) = doc.layers.get(obj.layer_id) {
            if !layer.visible {
                continue;
            }
        }
        let name = obj
            .name
            .clone()
            .unwrap_or_else(|| format!("object_{}", idx + 1));
        let _ = writeln!(out, "o {name}");

        match &obj.geometry {
            Geometry::Mesh(mesh) => {
                write_obj_mesh(&mut out, mesh, &mut v_offset, &mut vn_offset);
            }
            Geometry::Surface(surf) => {
                let mesh = surf.tessellate_display();
                write_obj_mesh(&mut out, &mesh, &mut v_offset, &mut vn_offset);
            }
            Geometry::Curve(curve) => {
                let pts = curve.sample_polyline(48);
                for p in &pts {
                    let _ = writeln!(out, "v {:.6} {:.6} {:.6}", p.x, p.y, p.z);
                }
                let _ = write!(out, "l");
                for i in 0..pts.len() {
                    let _ = write!(out, " {}", v_offset + i as u32);
                }
                out.push('\n');
                v_offset += pts.len() as u32;
            }
            Geometry::Point(p) => {
                let _ = writeln!(out, "v {:.6} {:.6} {:.6}", p.x, p.y, p.z);
                let _ = writeln!(out, "p {v_offset}");
                v_offset += 1;
            }
        }
    }

    out
}

fn write_obj_mesh(
    out: &mut String,
    mesh: &TriangleMesh,
    v_offset: &mut u32,
    vn_offset: &mut u32,
) {
    for p in &mesh.positions {
        let _ = writeln!(out, "v {:.6} {:.6} {:.6}", p[0], p[1], p[2]);
    }
    for n in &mesh.normals {
        let _ = writeln!(out, "vn {:.6} {:.6} {:.6}", n[0], n[1], n[2]);
    }
    for &[a, b, c] in &mesh.indices {
        let va = *v_offset + a;
        let vb = *v_offset + b;
        let vc = *v_offset + c;
        let na = *vn_offset + a;
        let nb = *vn_offset + b;
        let nc = *vn_offset + c;
        let _ = writeln!(out, "f {va}//{na} {vb}//{nb} {vc}//{nc}");
    }
    *v_offset += mesh.positions.len() as u32;
    *vn_offset += mesh.normals.len() as u32;
}

/// Import Wavefront OBJ text into `doc` as `TriangleMesh` objects.
pub fn import_obj_str(input: &str, doc: &mut Document) -> Result<usize, IoError> {
    let mut global_positions: Vec<[f64; 3]> = Vec::new();
    let mut positions: Vec<[f64; 3]> = Vec::new();
    let mut normals: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut current_name: Option<String> = None;
    let mut added = 0usize;

    let flush = |name: &mut Option<String>,
                 pos: &mut Vec<[f64; 3]>,
                 norm: &mut Vec<[f64; 3]>,
                 idx: &mut Vec<[u32; 3]>,
                 doc: &mut Document,
                 added: &mut usize| {
        if !idx.is_empty() {
            let mesh = TriangleMesh {
                positions: std::mem::take(pos),
                normals: std::mem::take(norm),
                indices: std::mem::take(idx),
            };
            let id = doc.add_object("ImportOBJ", Geometry::Mesh(mesh));
            if let Some(obj) = doc.objects.get_mut(id) {
                obj.name = name.take();
            }
            *added += 1;
        } else {
            pos.clear();
            norm.clear();
            *name = None;
        }
    };

    for raw_line in input.lines() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let Some(tag) = parts.next() else {
            continue;
        };
        match tag {
            "o" | "g" => {
                flush(
                    &mut current_name,
                    &mut positions,
                    &mut normals,
                    &mut indices,
                    doc,
                    &mut added,
                );
                current_name = parts.next().map(|s| s.to_string());
            }
            "v" => {
                let x = parts.next().and_then(|s| s.parse::<f64>().ok());
                let y = parts.next().and_then(|s| s.parse::<f64>().ok());
                let z = parts.next().and_then(|s| s.parse::<f64>().ok());
                if let (Some(x), Some(y), Some(z)) = (x, y, z) {
                    global_positions.push([x, y, z]);
                }
            }
            "f" => {
                let verts: Vec<usize> = parts
                    .filter_map(|tok| {
                        let v_str = tok.split('/').next()?;
                        let idx = v_str.parse::<isize>().ok()?;
                        let resolved = if idx > 0 {
                            (idx - 1) as usize
                        } else if idx < 0 {
                            global_positions.len().checked_sub((-idx) as usize)?
                        } else {
                            return None;
                        };
                        (resolved < global_positions.len()).then_some(resolved)
                    })
                    .collect();

                if verts.len() >= 3 {
                    let p0 = DVec3::from_array(global_positions[verts[0]]);
                    for w in 1..(verts.len() - 1) {
                        let p1 = DVec3::from_array(global_positions[verts[w]]);
                        let p2 = DVec3::from_array(global_positions[verts[w + 1]]);
                        let n = (p1 - p0).cross(p2 - p0).normalize_or_zero().to_array();
                        let base = positions.len() as u32;
                        positions.push(p0.to_array());
                        positions.push(p1.to_array());
                        positions.push(p2.to_array());
                        normals.push(n);
                        normals.push(n);
                        normals.push(n);
                        indices.push([base, base + 1, base + 2]);
                    }
                }
            }
            _ => {}
        }
    }

    flush(
        &mut current_name,
        &mut positions,
        &mut normals,
        &mut indices,
        doc,
        &mut added,
    );
    Ok(added)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::TriangleMesh;

    #[test]
    fn obj_round_trip_preserves_triangles() {
        let mut doc = Document::new();
        let mesh = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        doc.add_object("Box", Geometry::Mesh(mesh));
        let text = export_obj_string(&doc);

        let mut doc2 = Document::new();
        let count = import_obj_str(&text, &mut doc2).unwrap();
        assert_eq!(count, 1);
        let imported = doc2.objects.values().next().unwrap();
        let Geometry::Mesh(m) = &imported.geometry else {
            panic!("expected mesh");
        };
        assert_eq!(m.indices.len(), 12);
    }
}
