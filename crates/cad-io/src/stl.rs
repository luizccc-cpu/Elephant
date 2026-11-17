use cad_doc::Document;
use cad_geom::{DVec3, Geometry, TriangleMesh};

use crate::error::IoError;

/// Export all visible meshes and tessellated surfaces in `doc` as binary STL bytes.
pub fn export_stl_bytes(doc: &Document) -> Vec<u8> {
    let mut tris: Vec<([f32; 3], [[f32; 3]; 3])> = Vec::new();

    for (_, obj) in &doc.objects {
        if let Some(layer) = doc.layers.get(obj.layer_id) {
            if !layer.visible {
                continue;
            }
        }
        match &obj.geometry {
            Geometry::Mesh(m) => collect_mesh_tris(m, &mut tris),
            Geometry::Surface(s) => {
                let m = s.tessellate(24, 8);
                collect_mesh_tris(&m, &mut tris);
            }
            _ => {}
        }
    }

    let mut out = Vec::with_capacity(84 + tris.len() * 50);
    let mut header = [0u8; 80];
    let banner = b"Elephant CAD Binary STL";
    header[..banner.len()].copy_from_slice(banner);
    out.extend_from_slice(&header);
    out.extend_from_slice(&(tris.len() as u32).to_le_bytes());

    for (n, [v0, v1, v2]) in tris {
        for c in n.into_iter().chain(v0).chain(v1).chain(v2) {
            out.extend_from_slice(&c.to_le_bytes());
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

fn collect_mesh_tris(mesh: &TriangleMesh, out: &mut Vec<([f32; 3], [[f32; 3]; 3])>) {
    for &[i0, i1, i2] in &mesh.indices {
        let p0 = DVec3::from_array(mesh.positions[i0 as usize]);
        let p1 = DVec3::from_array(mesh.positions[i1 as usize]);
        let p2 = DVec3::from_array(mesh.positions[i2 as usize]);
        let n = (p1 - p0).cross(p2 - p0).normalize_or_zero();
        out.push((
            [n.x as f32, n.y as f32, n.z as f32],
            [
                [p0.x as f32, p0.y as f32, p0.z as f32],
                [p1.x as f32, p1.y as f32, p1.z as f32],
                [p2.x as f32, p2.y as f32, p2.z as f32],
            ],
        ));
    }
}

/// Import either Binary or ASCII STL bytes into `doc` as a `TriangleMesh`.
pub fn import_stl_bytes(bytes: &[u8], doc: &mut Document, name: Option<String>) -> Result<usize, IoError> {
    if bytes.len() >= 84 {
        let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        if 84 + count * 50 == bytes.len() {
            return import_binary_stl(bytes, count, doc, name);
        }
    }

    if let Ok(text) = std::str::from_utf8(bytes) {
        if text.trim_start().starts_with("solid") {
            return import_ascii_stl(text, doc, name);
        }
    }

    Err(IoError::InvalidFormat("unrecognized STL data".to_string()))
}

fn import_binary_stl(
    bytes: &[u8],
    count: usize,
    doc: &mut Document,
    name: Option<String>,
) -> Result<usize, IoError> {
    let mut positions = Vec::with_capacity(count * 3);
    let mut normals = Vec::with_capacity(count * 3);
    let mut indices = Vec::with_capacity(count);

    for i in 0..count {
        let off = 84 + i * 50;
        let read_f32 = |o: usize| -> f64 {
            f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as f64
        };
        let n = [read_f32(off), read_f32(off + 4), read_f32(off + 8)];
        let v0 = [read_f32(off + 12), read_f32(off + 16), read_f32(off + 20)];
        let v1 = [read_f32(off + 24), read_f32(off + 28), read_f32(off + 32)];
        let v2 = [read_f32(off + 36), read_f32(off + 40), read_f32(off + 44)];

        let base = positions.len() as u32;
        positions.extend_from_slice(&[v0, v1, v2]);
        normals.extend_from_slice(&[n, n, n]);
        indices.push([base, base + 1, base + 2]);
    }

    let id = doc.add_object(
        "ImportSTL",
        Geometry::Mesh(TriangleMesh {
            positions,
            normals,
            indices,
        }),
    );
    if let Some(obj) = doc.objects.get_mut(id) {
        obj.name = name;
    }
    Ok(1)
}

fn import_ascii_stl(
    text: &str,
    doc: &mut Document,
    name: Option<String>,
) -> Result<usize, IoError> {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mut tri_verts: Vec<[f64; 3]> = Vec::with_capacity(3);

    for line in text.lines().map(str::trim) {
        if let Some(rest) = line.strip_prefix("vertex ") {
            let nums: Vec<f64> = rest
                .split_whitespace()
                .filter_map(|s| s.parse::<f64>().ok())
                .collect();
            if nums.len() == 3 {
                tri_verts.push([nums[0], nums[1], nums[2]]);
                if tri_verts.len() == 3 {
                    let p0 = DVec3::from_array(tri_verts[0]);
                    let p1 = DVec3::from_array(tri_verts[1]);
                    let p2 = DVec3::from_array(tri_verts[2]);
                    let n = (p1 - p0).cross(p2 - p0).normalize_or_zero().to_array();
                    let base = positions.len() as u32;
                    positions.extend_from_slice(&tri_verts);
                    normals.extend_from_slice(&[n, n, n]);
                    indices.push([base, base + 1, base + 2]);
                    tri_verts.clear();
                }
            }
        }
    }

    if indices.is_empty() {
        return Err(IoError::InvalidFormat("ASCII STL contained no triangles".to_string()));
    }
    let id = doc.add_object(
        "ImportSTL",
        Geometry::Mesh(TriangleMesh {
            positions,
            normals,
            indices,
        }),
    );
    if let Some(obj) = doc.objects.get_mut(id) {
        obj.name = name;
    }
    Ok(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn binary_stl_round_trip() {
        let mut doc = Document::new();
        doc.add_object("Box", Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0))));
        let bytes = export_stl_bytes(&doc);
        assert_eq!(bytes.len(), 84 + 12 * 50);

        let mut doc2 = Document::new();
        let count = import_stl_bytes(&bytes, &mut doc2, Some("box.stl".to_string())).unwrap();
        assert_eq!(count, 1);
    }
}
