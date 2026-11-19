use cad_doc::{
    AnnotationItem, AnnotationKind, ClippingPlane, Document, GdtSpec, HistoryOp, HistoryRecord,
    Layer, LayoutSheet, MaterialProps,
};
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
    #[serde(default)]
    parent_index: Option<usize>,
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
    #[serde(default)]
    material: Option<MaterialProps>,
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
    #[serde(default)]
    gdt: Option<GdtSpec>,
}

#[derive(Debug, Serialize, Deserialize)]
struct SavedHistoryRecord {
    child_index: usize,
    parent_indices: Vec<usize>,
    op: HistoryOp,
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
    history_records: Vec<SavedHistoryRecord>,
    #[serde(default)]
    clipping_planes: Vec<ClippingPlane>,
    #[serde(default)]
    layouts: Vec<LayoutSheet>,
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
            let parent_index = l
                .parent
                .and_then(|p| layer_keys.iter().position(|&k| k == p));
            SavedLayer {
                name: l.name.clone(),
                color: l.color,
                visible: l.visible,
                locked: l.locked,
                active: lid == doc.active_layer,
                parent_index,
            }
        })
        .collect();

    let obj_keys: Vec<_> = doc.objects.keys().collect();
    let objects = obj_keys
        .iter()
        .map(|&oid| {
            let obj = &doc.objects[oid];
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
                material: obj.material,
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
                gdt: ann.gdt.clone(),
            }
        })
        .collect();

    let history_records = doc
        .history_records
        .iter()
        .filter_map(|rec| {
            let child_index = obj_keys.iter().position(|&id| id == rec.child_id)?;
            let parent_indices: Option<Vec<usize>> = rec
                .parent_ids
                .iter()
                .map(|pid| obj_keys.iter().position(|&id| id == *pid))
                .collect();
            Some(SavedHistoryRecord {
                child_index,
                parent_indices: parent_indices?,
                op: rec.op.clone(),
            })
        })
        .collect();

    let proj = ElephantProject {
        version: 2,
        units: doc.units,
        tolerances: doc.tolerances,
        layers,
        objects,
        annotations,
        history_records,
        clipping_planes: doc.clipping_planes.clone(),
        layouts: doc.layouts.clone(),
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
        let mut parent_indices = Vec::with_capacity(proj.layers.len());
        let mut active_idx = 0usize;
        for (i, sl) in proj.layers.into_iter().enumerate() {
            if sl.active {
                active_idx = i;
            }
            parent_indices.push(sl.parent_index);
            let lid = doc.layers.insert(Layer::new(sl.name, sl.color));
            if let Some(l) = doc.layers.get_mut(lid) {
                l.visible = sl.visible;
                l.locked = sl.locked;
            }
            new_keys.push(lid);
        }
        for (i, pidx) in parent_indices.into_iter().enumerate() {
            if let Some(pi) = pidx {
                if pi != i {
                    if let Some(&parent_lid) = new_keys.get(pi) {
                        if let Some(layer) = doc.layers.get_mut(new_keys[i]) {
                            layer.parent = Some(parent_lid);
                        }
                    }
                }
            }
        }

        let restore_active = new_keys[active_idx];
        let mut max_gid = 0_u64;
        let mut restored_obj_ids = Vec::with_capacity(proj.objects.len());

        for so in proj.objects {
            let lid = *new_keys.get(so.layer_index).unwrap_or(&restore_active);
            let oid = doc.add_object_on_layer("Load", lid, so.geometry);
            if let Some(obj) = doc.objects.get_mut(oid) {
                obj.name = so.name;
                obj.color = so.color;
                obj.visible = so.visible;
                obj.group_id = so.group_id;
                obj.material = so.material;
                if let Some(gid) = so.group_id {
                    max_gid = max_gid.max(gid);
                }
            }
            restored_obj_ids.push(oid);
        }
        doc.ensure_next_group_id_above(max_gid);

        for shr in proj.history_records {
            if let Some(&child_id) = restored_obj_ids.get(shr.child_index) {
                let parent_ids: Option<Vec<_>> = shr
                    .parent_indices
                    .into_iter()
                    .map(|idx| restored_obj_ids.get(idx).copied())
                    .collect();
                if let Some(parent_ids) = parent_ids {
                    doc.history_records.push(HistoryRecord {
                        child_id,
                        parent_ids,
                        op: shr.op,
                    });
                }
            }
        }

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
                gdt: sa.gdt,
            });
        }
        doc.active_layer = restore_active;
    }

    doc.clipping_planes = proj.clipping_planes;
    doc.layouts = proj.layouts;
    doc.ensure_next_aux_ids();
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
            gdt: None,
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

    #[test]
    fn roundtrips_trimmed_brep_in_v2_project() {
        let mut doc = Document::new();
        let cyl = cad_geom::BRep::from_cylinder(DVec3::ZERO, 2.0, 5.0).unwrap();
        let orig_faces = cyl.faces.len();
        let orig_edges = cyl.edges.len();
        let orig_vol = cyl.volume_approx();
        doc.add_object("SolidCyl", Geometry::BRep(cyl));

        let (json, meta) = (save_project_with_meta(&doc, &ProjectMetadata::default()).unwrap(), ProjectMetadata::default());
        let (loaded, loaded_meta) = load_project_with_meta(&json).unwrap();
        assert_eq!(loaded_meta, meta);
        assert_eq!(loaded.objects.len(), 1);
        let obj = loaded.objects.values().next().unwrap();
        match &obj.geometry {
            Geometry::BRep(b) => {
                assert!(b.is_solid());
                assert_eq!(b.faces.len(), orig_faces);
                assert_eq!(b.edges.len(), orig_edges);
                assert!((b.volume_approx() - orig_vol).abs() < 1e-6);
            }
            other => panic!("expected BRep after roundtrip, got {:?}", other),
        }
    }

    #[test]
    fn roundtrips_nested_layers_and_modeling_history() {
        let mut doc = Document::new();
        let parent_layer = doc.add_layer("Assemblies", [120, 180, 240, 255]);
        let child_layer = doc.add_sublayer("Brackets", [240, 160, 120, 255], parent_layer);
        doc.layers[parent_layer].visible = false;
        assert!(!doc.is_layer_effectively_visible(child_layer));

        let line = NurbsCurve::line(DVec3::ZERO, DVec3::new(3.0, 0.0, 0.0));
        let p_id = doc.add_object_on_layer("Line", child_layer, Geometry::Curve(line));
        let ext = cad_geom::NurbsSurface::extrude_curve(
            match &doc.objects[p_id].geometry {
                Geometry::Curve(c) => c,
                _ => unreachable!(),
            },
            DVec3::new(0.0, 0.0, 4.0),
        );
        let c_id = doc.add_object_on_layer("Extrude", child_layer, Geometry::Surface(ext));
        doc.add_history_record(
            c_id,
            vec![p_id],
            HistoryOp::Extrude {
                direction: DVec3::new(0.0, 0.0, 4.0),
                solid: false,
            },
        );

        let json = save_project_json(&doc).unwrap();
        let loaded = load_project_json(&json).unwrap();
        assert_eq!(loaded.layers.len(), 3);
        let loaded_child = loaded
            .layers
            .iter()
            .find(|(_, l)| l.name == "Brackets")
            .map(|(id, _)| id)
            .unwrap();
        assert!(!loaded.is_layer_effectively_visible(loaded_child));
        assert_eq!(loaded.history_records.len(), 1);

        // Moving parent in loaded doc still updates child via preserved history!
        let mut loaded = loaded;
        let restored_parent = loaded
            .objects
            .iter()
            .find(|(_, o)| matches!(o.geometry, Geometry::Curve(_)))
            .map(|(id, _)| id)
            .unwrap();
        loaded.clear_selection();
        loaded.set_selected(restored_parent, true);
        loaded.transform_selected(
            "MoveParent",
            &cad_geom::DMat4::from_translation(DVec3::new(0.0, 5.0, 0.0)),
        );
        let restored_child = loaded.history_records[0].child_id;
        assert!((loaded.objects[restored_child].aabb.center().y - 5.0).abs() < 1e-6);
    }

    #[test]
    fn roundtrips_materials_clipping_layouts_and_gdt() {
        use cad_doc::{GdtSymbol, PageSize};
        let mut doc = Document::new();
        let oid = doc.add_object("Part", Geometry::Point(DVec3::ZERO));
        doc.objects[oid].material = Some(MaterialProps {
            metallic: 0.85,
            roughness: 0.18,
            opacity: 1.0,
        });
        let cp_id = doc.add_clipping_plane("Section Y", Plane::world_zx());
        let lid = doc.add_standard_4view_layout(Some("Main Sheet".to_string()), PageSize::A3);
        doc.add_gdt_annotation(
            GdtSpec {
                symbol: GdtSymbol::Perpendicularity,
                tolerance: 0.04,
                diameter_zone: false,
                modifier: None,
                datums: vec!["A".to_string()],
            },
            DVec3::new(1.0, 2.0, 3.0),
        );

        let json = save_project_json(&doc).unwrap();
        let loaded = load_project_json(&json).unwrap();
        let obj = loaded.objects.values().next().unwrap();
        assert_eq!(
            obj.material,
            Some(MaterialProps {
                metallic: 0.85,
                roughness: 0.18,
                opacity: 1.0,
            })
        );
        assert_eq!(loaded.clipping_planes.len(), 1);
        assert_eq!(loaded.clipping_planes[0].id, cp_id);
        assert_eq!(loaded.layouts.len(), 1);
        assert_eq!(loaded.layouts[0].id, lid);
        assert_eq!(loaded.annotations.len(), 1);
        assert_eq!(
            loaded.annotations[0].gdt.as_ref().unwrap().symbol,
            GdtSymbol::Perpendicularity
        );
    }
}
