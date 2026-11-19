use cad_doc::{AnnotationItem, AnnotationKind, Document, Layer};
use cad_geom::{DVec3, Geometry, Tolerances, UnitSystem};
use serde::{Deserialize, Serialize};

use crate::error::IoError;

fn default_true() -> bool {
    true
}

#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProjectMetadata {
    #[serde(default)]
    pub named_views: Vec<serde_json::Value>,
    #[serde(default)]
    pub parametric_graph: Option<serde_json::Value>,
    #[serde(default)]
    pub script_source: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
struct SavedLayer {
    name: String,
    color: [u8; 4],
    visible: bool,
    locked: bool,
    active: bool,
}

#[derive(Debug, Serialize, Deserialize)]
struct SavedObject {
    name: Option<String>,
    layer_index: usize,
    #[serde(default)]
    color: Option<[u8; 4]>,
    #[serde(default = "default_true")]
    visible: bool,
    #[serde(default)]
    group_id: Option<u64>,
    geometry: Geometry,
}

#[derive(Debug, Serialize, Deserialize)]
struct SavedAnnotation {
    kind: AnnotationKind,
    layer_index: usize,
    #[serde(default)]
    color: Option<[u8; 4]>,
    text: String,
    position: DVec3,
    #[serde(default)]
    p1: Option<DVec3>,
    #[serde(default)]
    p2: Option<DVec3>,
    #[serde(default)]
    offset_pt: Option<DVec3>,
}

#[derive(Debug, Serialize, Deserialize)]
struct ElephantProject {
    version: u32,
    #[serde(default)]
    units: UnitSystem,
    tolerances: Tolerances,
    layers: Vec<SavedLayer>,
    objects: Vec<SavedObject>,
    #[serde(default)]
    annotations: Vec<SavedAnnotation>,
    #[serde(default)]
    metadata: ProjectMetadata,
}

/// Serialize the full Elephant `Document` and desktop metadata (named views, graph, script) to JSON.
pub fn save_project_with_meta(
    doc: &Document,
    meta: &ProjectMetadata,
) -> Result<String, IoError> {
    let layer_keys: Vec<_> = doc.layers.keys().collect();
    let layers: Vec<SavedLayer> = layer_keys
        .iter()
        .map(|&lid| {
            let l = &doc.layers[lid];
            SavedLayer {
                name: l.name.clone(),
                color: l.color,
                visible: l.visible,
                locked: l.locked,
                active: lid == doc.active_layer,
            }
        })
        .collect();

    let objects = doc
        .objects
        .values()
        .map(|obj| {
            let layer_index = layer_keys
                .iter()
                .position(|&lid| lid == obj.layer_id)
                .unwrap_or(0);
            SavedObject {
                name: obj.name.clone(),
                layer_index,
                color: obj.color,
                visible: obj.visible,
                group_id: obj.group_id,
                geometry: obj.geometry.clone(),
            }
        })
        .collect();

    let annotations = doc
        .annotations
        .iter()
        .map(|ann| {
            let layer_index = layer_keys
                .iter()
                .position(|&lid| lid == ann.layer_id)
                .unwrap_or(0);
            SavedAnnotation {
                kind: ann.kind,
                layer_index,
                color: ann.color,
                text: ann.text.clone(),
                position: ann.position,
                p1: ann.p1,
                p2: ann.p2,
                offset_pt: ann.offset_pt,
            }
        })
        .collect();

    let proj = ElephantProject {
        version: 2,
        units: doc.units,
        tolerances: doc.tolerances,
        layers,
        objects,
        annotations,
        metadata: meta.clone(),
    };
    Ok(serde_json::to_string_pretty(&proj)?)
}

/// Serialize the full Elephant `Document` (exact NURBS, meshes, layers, tolerances) to JSON.
pub fn save_project_json(doc: &Document) -> Result<String, IoError> {
    save_project_with_meta(doc, &ProjectMetadata::default())
}

/// Load an Elephant `.elephant` JSON project (v1 or v2) into a fresh `Document` + metadata.
pub fn load_project_with_meta(input: &str) -> Result<(Document, ProjectMetadata), IoError> {
    let proj: ElephantProject = serde_json::from_str(input)?;
    let mut doc = Document::new();
    doc.units = proj.units;
    doc.tolerances = proj.tolerances;

    if !proj.layers.is_empty() {
        doc.layers.clear();
        let mut new_keys = Vec::with_capacity(proj.layers.len());
        let mut active_idx = 0usize;
        for (i, sl) in proj.layers.into_iter().enumerate() {
            if sl.active {
                active_idx = i;
            }
            let lid = doc.layers.insert(Layer {
                name: sl.name,
                color: sl.color,
                visible: sl.visible,
                locked: sl.locked,
            });
            new_keys.push(lid);
        }
        let restore_active = new_keys[active_idx];
        let mut max_gid = 0_u64;

        for so in proj.objects {
            let lid = *new_keys.get(so.layer_index).unwrap_or(&restore_active);
            let oid = doc.add_object_on_layer("Load", lid, so.geometry);
            if let Some(obj) = doc.objects.get_mut(oid) {
                obj.name = so.name;
                obj.color = so.color;
                obj.visible = so.visible;
                obj.group_id = so.group_id;
                if let Some(gid) = so.group_id {
                    max_gid = max_gid.max(gid);
                }
            }
        }
        doc.ensure_next_group_id_above(max_gid);

        for sa in proj.annotations {
            let lid = *new_keys.get(sa.layer_index).unwrap_or(&restore_active);
            doc.add_annotation(AnnotationItem {
                kind: sa.kind,
                layer_id: lid,
                color: sa.color,
                text: sa.text,
                position: sa.position,
                p1: sa.p1,
                p2: sa.p2,
                offset_pt: sa.offset_pt,
            });
        }
        doc.active_layer = restore_active;
    }

    doc.last_created.clear();
    doc.clear_history();
    Ok((doc, proj.metadata))
}

/// Load an Elephant `.elephant` JSON project into a fresh `Document`.
pub fn load_project_json(input: &str) -> Result<Document, IoError> {
    load_project_with_meta(input).map(|(doc, _)| doc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::{NurbsCurve, Plane};

    #[test]
    fn elephant_project_round_trip_preserves_nurbs() {
        let mut doc = Document::new();
        let c = NurbsCurve::circle(&Plane::world_xy(), 3.5).unwrap();
        let id = doc.add_object("Circle", Geometry::Curve(c));
        doc.objects[id].name = Some("ExactCircle".to_string());

        let json = save_project_json(&doc).unwrap();
        let loaded = load_project_json(&json).unwrap();
        assert_eq!(loaded.objects.len(), 1);
        let obj = loaded.objects.values().next().unwrap();
        assert_eq!(obj.name.as_deref(), Some("ExactCircle"));
        assert!(matches!(obj.geometry, Geometry::Curve(_)));
        assert!(!loaded.can_undo());
    }

    #[test]
    fn elephant_v2_round_trip_preserves_units_visibility_groups_annotations_and_meta() {
        let mut doc = Document::new();
        doc.set_units(UnitSystem::Inches, false);
        let id = doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(10.0, 0.0, 0.0))),
        );
        doc.objects[id].color = Some([255, 120, 40, 255]);
        doc.objects[id].visible = false;
        doc.objects[id].group_id = Some(42);
        doc.add_annotation(AnnotationItem {
            kind: AnnotationKind::TextDot,
            layer_id: doc.active_layer,
            color: None,
            text: "Pin".to_string(),
            position: DVec3::new(1.0, 2.0, 0.0),
            p1: None,
            p2: None,
            offset_pt: None,
        });

        let meta = ProjectMetadata {
            named_views: vec![serde_json::json!({ "name": "FrontDetail" })],
            parametric_graph: Some(serde_json::json!({ "live_preview": true })),
            script_source: Some("log(\"hi\");".to_string()),
        };

        let json = save_project_with_meta(&doc, &meta).unwrap();
        let (loaded, loaded_meta) = load_project_with_meta(&json).unwrap();
        assert_eq!(loaded.units, UnitSystem::Inches);
        let obj = loaded.objects.values().next().unwrap();
        assert_eq!(obj.color, Some([255, 120, 40, 255]));
        assert!(!obj.visible);
        assert_eq!(obj.group_id, Some(42));
        assert_eq!(loaded.annotations.len(), 1);
        assert_eq!(loaded.annotations[0].text, "Pin");
        assert_eq!(loaded_meta, meta);
    }

    #[test]
    fn loads_legacy_v1_project_with_defaults() {
        let v1_json = r#"{
            "version": 1,
            "tolerances": { "absolute": 0.001, "relative": 1e-6, "angle_rad": 0.001745 },
            "layers": [{ "name": "Default", "color": [215, 220, 228, 255], "visible": true, "locked": false, "active": true }],
            "objects": [{ "name": "OriginPt", "layer_index": 0, "geometry": { "Point": [1.0, 2.0, 3.0] } }]
        }"#;
        let (doc, meta) = load_project_with_meta(v1_json).unwrap();
        assert_eq!(doc.units, UnitSystem::Millimeters);
        assert_eq!(doc.objects.len(), 1);
        let obj = doc.objects.values().next().unwrap();
        assert!(obj.visible);
        assert_eq!(obj.color, None);
        assert_eq!(obj.group_id, None);
        assert!(doc.annotations.is_empty());
        assert_eq!(meta, ProjectMetadata::default());
    }
}
