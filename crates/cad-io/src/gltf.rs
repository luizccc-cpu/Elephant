use cad_doc::Document;
use cad_geom::Geometry;

/// Export all visible meshes and surfaces to a binary glTF 2.0 (`.glb`) container.
pub fn export_glb_bytes(doc: &Document) -> Vec<u8> {
    let mut bin: Vec<u8> = Vec::new();
    let mut buffer_views: Vec<String> = Vec::new();
    let mut accessors: Vec<String> = Vec::new();
    let mut meshes_json: Vec<String> = Vec::new();
    let mut nodes_json: Vec<String> = Vec::new();
    let mut materials_json: Vec<String> = Vec::new();

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
        let mesh = match &obj.geometry {
            Geometry::Mesh(m) => m.clone(),
            Geometry::Surface(s) => s.tessellate(24, 8),
            _ => continue,
        };
        if mesh.positions.is_empty() || mesh.indices.is_empty() {
            continue;
        }

        let col = obj.color.unwrap_or(layer.color);
        let r = col[0] as f32 / 255.0;
        let g = col[1] as f32 / 255.0;
        let b = col[2] as f32 / 255.0;
        let mat_idx = materials_json.len();
        materials_json.push(format!(
            r#"{{"pbrMetallicRoughness":{{"baseColorFactor":[{r:.4},{g:.4},{b:.4},1.0],"metallicFactor":0.1,"roughnessFactor":0.6}}}}"#
        ));

        let mut min_p = [f32::INFINITY; 3];
        let mut max_p = [f32::NEG_INFINITY; 3];

        // Positions
        while bin.len() % 4 != 0 {
            bin.push(0);
        }
        let pos_off = bin.len();
        for p in &mesh.positions {
            for k in 0..3 {
                let v = p[k] as f32;
                if v < min_p[k] {
                    min_p[k] = v;
                }
                if v > max_p[k] {
                    max_p[k] = v;
                }
                bin.extend_from_slice(&v.to_le_bytes());
            }
        }
        let pos_len = bin.len() - pos_off;
        let pos_bv = buffer_views.len();
        buffer_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{pos_off},"byteLength":{pos_len},"target":34962}}"#
        ));
        let pos_acc = accessors.len();
        accessors.push(format!(
            r#"{{"bufferView":{pos_bv},"componentType":5126,"count":{},"type":"VEC3","min":[{:.6},{:.6},{:.6}],"max":[{:.6},{:.6},{:.6}]}}"#,
            mesh.positions.len(),
            min_p[0], min_p[1], min_p[2],
            max_p[0], max_p[1], max_p[2]
        ));

        // Normals
        let norm_off = bin.len();
        for n in &mesh.normals {
            for k in 0..3 {
                bin.extend_from_slice(&(n[k] as f32).to_le_bytes());
            }
        }
        let norm_len = bin.len() - norm_off;
        let norm_bv = buffer_views.len();
        buffer_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{norm_off},"byteLength":{norm_len},"target":34962}}"#
        ));
        let norm_acc = accessors.len();
        accessors.push(format!(
            r#"{{"bufferView":{norm_bv},"componentType":5126,"count":{},"type":"VEC3"}}"#,
            mesh.normals.len()
        ));

        // Indices (u32)
        let idx_off = bin.len();
        for tri in &mesh.indices {
            for &idx in tri {
                bin.extend_from_slice(&idx.to_le_bytes());
            }
        }
        let idx_len = bin.len() - idx_off;
        let idx_bv = buffer_views.len();
        buffer_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{idx_off},"byteLength":{idx_len},"target":34963}}"#
        ));
        let idx_acc = accessors.len();
        accessors.push(format!(
            r#"{{"bufferView":{idx_bv},"componentType":5125,"count":{},"type":"SCALAR"}}"#,
            mesh.indices.len() * 3
        ));

        let mesh_idx = meshes_json.len();
        meshes_json.push(format!(
            r#"{{"primitives":[{{"attributes":{{"POSITION":{pos_acc},"NORMAL":{norm_acc}}},"indices":{idx_acc},"material":{mat_idx}}}]}}"#
        ));
        nodes_json.push(format!(r#"{{"mesh":{mesh_idx}}}"#));
    }

    while bin.len() % 4 != 0 {
        bin.push(0);
    }

    let node_indices: Vec<String> = (0..nodes_json.len()).map(|i| i.to_string()).collect();
    let json_str = format!(
        r#"{{"asset":{{"version":"2.0","generator":"Elephant CAD"}},"scene":0,"scenes":[{{"nodes":[{}]}}],"nodes":[{}],"meshes":[{}],"materials":[{}],"accessors":[{}],"bufferViews":[{}],"buffers":[{{"byteLength":{}}}]}}"#,
        node_indices.join(","),
        nodes_json.join(","),
        meshes_json.join(","),
        materials_json.join(","),
        accessors.join(","),
        buffer_views.join(","),
        bin.len()
    );
    let mut json_bytes = json_str.into_bytes();
    while json_bytes.len() % 4 != 0 {
        json_bytes.push(b' ');
    }

    let total_len = 12 + 8 + json_bytes.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total_len);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2_u32.to_le_bytes());
    out.extend_from_slice(&(total_len as u32).to_le_bytes());
    out.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x4E4F534A_u32.to_le_bytes()); // "JSON"
    out.extend_from_slice(&json_bytes);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x004E4942_u32.to_le_bytes()); // "BIN\0"
    out.extend_from_slice(&bin);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::{DVec3, TriangleMesh};

    #[test]
    fn glb_export_writes_valid_header() {
        let mut doc = Document::new();
        doc.add_object("Box", Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0))));
        let glb = export_glb_bytes(&doc);
        assert!(glb.starts_with(b"glTF"));
        assert!(glb.len() > 100);
    }
}
