use std::fmt::Write as _;

use cad_doc::Document;
use cad_geom::{DVec3, Geometry, TriangleMesh};
use cad_math::UnitSystem;

use crate::error::IoError;

const CONTENT_TYPES_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#;

const RELS_XML: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#;

/// Export visible document meshes, surfaces, B-Reps, and SubDs to an uncompressed **3MF** archive.
pub fn export_3mf_bytes(doc: &Document) -> Vec<u8> {
    let unit_str = match doc.units {
        UnitSystem::Millimeters => "millimeter",
        UnitSystem::Centimeters => "centimeter",
        UnitSystem::Meters => "meter",
        UnitSystem::Inches => "inch",
        UnitSystem::Feet => "foot",
    };

    let mut objects_xml = String::new();
    let mut build_xml = String::new();
    let mut next_id = 1usize;

    for (_, obj) in &doc.objects {
        if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
            continue;
        }
        let mesh = match &obj.geometry {
            Geometry::Mesh(m) => m.clone(),
            Geometry::Surface(s) => s.tessellate_display(),
            Geometry::BRep(b) => b.tessellate_display(),
            Geometry::SubD(sd) => sd.tessellate_display(),
            Geometry::Point(_) | Geometry::Curve(_) => continue,
        };
        if mesh.positions.is_empty() || mesh.indices.is_empty() {
            continue;
        }
        let oid = next_id;
        next_id += 1;
        let name = obj
            .name
            .as_deref()
            .unwrap_or("ElephantObject")
            .replace('&', "&amp;")
            .replace('"', "&quot;")
            .replace('<', "&lt;")
            .replace('>', "&gt;");

        let _ = writeln!(objects_xml, r#"    <object id="{oid}" name="{name}" type="model">"#);
        let _ = writeln!(objects_xml, "      <mesh>");
        let _ = writeln!(objects_xml, "        <vertices>");
        for p in &mesh.positions {
            let _ = writeln!(
                objects_xml,
                r#"          <vertex x="{:.6}" y="{:.6}" z="{:.6}"/>"#,
                p[0], p[1], p[2]
            );
        }
        let _ = writeln!(objects_xml, "        </vertices>");
        let _ = writeln!(objects_xml, "        <triangles>");
        for &[a, b, c] in &mesh.indices {
            let _ = writeln!(
                objects_xml,
                r#"          <triangle v1="{a}" v2="{b}" v3="{c}"/>"#
            );
        }
        let _ = writeln!(objects_xml, "        </triangles>");
        let _ = writeln!(objects_xml, "      </mesh>");
        let _ = writeln!(objects_xml, "    </object>");
        let _ = writeln!(build_xml, r#"    <item objectid="{oid}"/>"#);
    }

    let model_xml = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="{unit_str}" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
{objects_xml}  </resources>
  <build>
{build_xml}  </build>
</model>
"#
    );

    write_stored_zip(&[
        ("[Content_Types].xml", CONTENT_TYPES_XML.as_bytes()),
        ("_rels/.rels", RELS_XML.as_bytes()),
        ("3D/3dmodel.model", model_xml.as_bytes()),
    ])
}

/// Import meshes from an uncompressed (Method 0 Stored) `.3mf` archive.
pub fn import_3mf_bytes(bytes: &[u8], doc: &mut Document) -> Result<usize, IoError> {
    let entries = read_zip_entries(bytes)?;
    let model_entry = entries
        .iter()
        .find(|e| e.name.eq_ignore_ascii_case("3d/3dmodel.model"))
        .or_else(|| {
            entries
                .iter()
                .find(|e| e.name.to_ascii_lowercase().ends_with(".model"))
        })
        .ok_or_else(|| IoError::InvalidFormat("3MF archive missing 3D/3dmodel.model".to_string()))?;

    if model_entry.method != 0 {
        return Err(IoError::InvalidFormat(
            "compressed 3MF deflate entries are not supported without zlib; re-export uncompressed 3MF or STL/OBJ"
                .to_string(),
        ));
    }

    let xml = std::str::from_utf8(model_entry.data)
        .map_err(|e| IoError::InvalidFormat(format!("invalid 3MF UTF-8 XML: {e}")))?;

    let mut added = 0usize;
    let mut cursor = xml;
    while let Some(obj_start) = cursor.find("<object") {
        let after_start = &cursor[obj_start..];
        let Some(tag_end) = after_start.find('>') else {
            break;
        };
        let header_tag = &after_start[..tag_end];
        let name = extract_xml_attr(header_tag, "name");
        let Some(obj_close) = after_start.find("</object>") else {
            break;
        };
        let obj_body = &after_start[tag_end + 1..obj_close];

        if let Some(mesh) = parse_3mf_mesh(obj_body) {
            let oid = doc.add_object("Import3MF_Mesh", Geometry::Mesh(mesh));
            if let Some(obj) = doc.objects.get_mut(oid) {
                obj.name = name;
            }
            added += 1;
        }
        cursor = &after_start[obj_close + "</object>".len()..];
    }

    Ok(added)
}

fn parse_3mf_mesh(body: &str) -> Option<TriangleMesh> {
    let mut positions: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();

    let mut cur = body;
    while let Some(vpos) = cur.find("<vertex") {
        let rest = &cur[vpos..];
        let end = rest.find('>')?;
        let tag = &rest[..end];
        if let (Some(x), Some(y), Some(z)) = (
            extract_xml_attr(tag, "x").and_then(|s| s.parse::<f64>().ok()),
            extract_xml_attr(tag, "y").and_then(|s| s.parse::<f64>().ok()),
            extract_xml_attr(tag, "z").and_then(|s| s.parse::<f64>().ok()),
        ) {
            positions.push([x, y, z]);
        }
        cur = &rest[end + 1..];
    }

    let mut tcur = body;
    while let Some(tpos) = tcur.find("<triangle") {
        let rest = &tcur[tpos..];
        let end = rest.find('>')?;
        let tag = &rest[..end];
        if let (Some(v1), Some(v2), Some(v3)) = (
            extract_xml_attr(tag, "v1").and_then(|s| s.parse::<u32>().ok()),
            extract_xml_attr(tag, "v2").and_then(|s| s.parse::<u32>().ok()),
            extract_xml_attr(tag, "v3").and_then(|s| s.parse::<u32>().ok()),
        ) {
            if (v1 as usize) < positions.len()
                && (v2 as usize) < positions.len()
                && (v3 as usize) < positions.len()
            {
                indices.push([v1, v2, v3]);
            }
        }
        tcur = &rest[end + 1..];
    }

    if positions.is_empty() || indices.is_empty() {
        return None;
    }

    let mut accum = vec![DVec3::ZERO; positions.len()];
    for &[i0, i1, i2] in &indices {
        let p0 = DVec3::from_array(positions[i0 as usize]);
        let p1 = DVec3::from_array(positions[i1 as usize]);
        let p2 = DVec3::from_array(positions[i2 as usize]);
        let fn_ = (p1 - p0).cross(p2 - p0);
        accum[i0 as usize] += fn_;
        accum[i1 as usize] += fn_;
        accum[i2 as usize] += fn_;
    }
    let normals = accum
        .into_iter()
        .map(|n| n.normalize_or(DVec3::Z).to_array())
        .collect();

    Some(TriangleMesh {
        positions,
        normals,
        indices,
    })
}

fn extract_xml_attr(tag: &str, attr: &str) -> Option<String> {
    for quote in ['"', '\''] {
        let needle = format!("{attr}={quote}");
        if let Some(pos) = tag.find(&needle) {
            let start = pos + needle.len();
            if let Some(end) = tag[start..].find(quote) {
                return Some(
                    tag[start..start + end]
                        .replace("&quot;", "\"")
                        .replace("&lt;", "<")
                        .replace("&gt;", ">")
                        .replace("&amp;", "&"),
                );
            }
        }
    }
    None
}

fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for &b in data {
        crc ^= u32::from(b);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Write a standard Method-0 (Stored) PKZIP archive with Local Headers, Central Directory, and EOCD.
fn write_stored_zip(files: &[(&str, &[u8])]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut central = Vec::new();

    for &(name, data) in files {
        let name_bytes = name.as_bytes();
        let crc = crc32_ieee(data);
        let size = data.len() as u32;
        let local_offset = out.len() as u32;

        // Local file header (30 bytes + name)
        out.extend_from_slice(&0x0403_4B50_u32.to_le_bytes());
        out.extend_from_slice(&20_u16.to_le_bytes()); // version needed
        out.extend_from_slice(&0_u16.to_le_bytes()); // flags
        out.extend_from_slice(&0_u16.to_le_bytes()); // method 0 = stored
        out.extend_from_slice(&0_u16.to_le_bytes()); // mod time
        out.extend_from_slice(&0_u16.to_le_bytes()); // mod date
        out.extend_from_slice(&crc.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&size.to_le_bytes());
        out.extend_from_slice(&(name_bytes.len() as u16).to_le_bytes());
        out.extend_from_slice(&0_u16.to_le_bytes()); // extra len
        out.extend_from_slice(name_bytes);
        out.extend_from_slice(data);

        // Central directory header (46 bytes + name)
        central.extend_from_slice(&0x0201_4B50_u32.to_le_bytes());
        central.extend_from_slice(&20_u16.to_le_bytes()); // version made by
        central.extend_from_slice(&20_u16.to_le_bytes()); // version needed
        central.extend_from_slice(&0_u16.to_le_bytes()); // flags
        central.extend_from_slice(&0_u16.to_le_bytes()); // method 0
        central.extend_from_slice(&0_u16.to_le_bytes());
        central.extend_from_slice(&0_u16.to_le_bytes());
        central.extend_from_slice(&crc.to_le_bytes());
        central.extend_from_slice(&size.to_le_bytes());
        central.extend_from_slice(&size.to_le_bytes());
        central.extend_from_slice(&(name_bytes.len() as u16).to_le_bytes());
        central.extend_from_slice(&0_u16.to_le_bytes()); // extra
        central.extend_from_slice(&0_u16.to_le_bytes()); // comment
        central.extend_from_slice(&0_u16.to_le_bytes()); // disk
        central.extend_from_slice(&0_u16.to_le_bytes()); // internal attr
        central.extend_from_slice(&0_u32.to_le_bytes()); // external attr
        central.extend_from_slice(&local_offset.to_le_bytes());
        central.extend_from_slice(name_bytes);
    }

    let cd_offset = out.len() as u32;
    let cd_size = central.len() as u32;
    out.extend_from_slice(&central);

    // End of Central Directory (22 bytes)
    out.extend_from_slice(&0x0605_4B50_u32.to_le_bytes());
    out.extend_from_slice(&0_u16.to_le_bytes());
    out.extend_from_slice(&0_u16.to_le_bytes());
    out.extend_from_slice(&(files.len() as u16).to_le_bytes());
    out.extend_from_slice(&(files.len() as u16).to_le_bytes());
    out.extend_from_slice(&cd_size.to_le_bytes());
    out.extend_from_slice(&cd_offset.to_le_bytes());
    out.extend_from_slice(&0_u16.to_le_bytes());
    out
}

struct ZipEntry<'a> {
    name: String,
    method: u16,
    data: &'a [u8],
}

fn read_zip_entries<'a>(bytes: &'a [u8]) -> Result<Vec<ZipEntry<'a>>, IoError> {
    if bytes.len() < 30 || &bytes[0..4] != b"PK\x03\x04" {
        return Err(IoError::InvalidFormat("not a valid 3MF/ZIP file".to_string()));
    }
    let mut entries = Vec::new();
    let mut off = 0usize;
    while off + 30 <= bytes.len() {
        let sig = u32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
        if sig != 0x0403_4B50 {
            break;
        }
        let method = u16::from_le_bytes(bytes[off + 8..off + 10].try_into().unwrap());
        let comp_size = u32::from_le_bytes(bytes[off + 18..off + 22].try_into().unwrap()) as usize;
        let name_len = u16::from_le_bytes(bytes[off + 26..off + 28].try_into().unwrap()) as usize;
        let extra_len = u16::from_le_bytes(bytes[off + 28..off + 30].try_into().unwrap()) as usize;
        let data_start = off + 30 + name_len + extra_len;
        let data_end = data_start.saturating_add(comp_size);
        if data_end > bytes.len() {
            return Err(IoError::InvalidFormat("truncated 3MF ZIP entry".to_string()));
        }
        let name = String::from_utf8_lossy(&bytes[off + 30..off + 30 + name_len]).into_owned();
        entries.push(ZipEntry {
            name,
            method,
            data: &bytes[data_start..data_end],
        });
        off = data_end;
    }
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::{BRep, SubDMesh};

    #[test]
    fn threemf_roundtrip_mesh_and_units() {
        let mut doc = Document::new();
        doc.set_units(UnitSystem::Millimeters, false);
        let box_b = BRep::from_box(DVec3::ZERO, DVec3::splat(10.0));
        let subd = SubDMesh::from_sphere(DVec3::new(20.0, 0.0, 0.0), 4.0);
        let expected_bb = box_b
            .bounding_box()
            .union(&subd.tessellate_display().bounding_box());
        doc.add_object("Box", Geometry::BRep(box_b));
        doc.add_object("SubDSphere", Geometry::SubD(subd));

        let bytes = export_3mf_bytes(&doc);
        assert!(bytes.starts_with(b"PK\x03\x04"));

        let mut doc2 = Document::new();
        let count = import_3mf_bytes(&bytes, &mut doc2).expect("import 3MF");
        assert_eq!(count, 2);

        let bb2 = doc2.visible_bounding_box();
        assert!(expected_bb.min.distance(bb2.min) < 1e-2);
        assert!(expected_bb.max.distance(bb2.max) < 1e-2);
    }
}
