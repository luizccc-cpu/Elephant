use cad_doc::{Document, Layer};
use cad_geom::{Geometry, Tolerances};
use serde::{Deserialize, Serialize};

use crate::error::IoError;

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
    geometry: Geometry,
}

#[derive(Debug, Serialize, Deserialize)]
struct ElephantProject {
    version: u32,
    tolerances: Tolerances,
    layers: Vec<SavedLayer>,
    objects: Vec<SavedObject>,
}

/// Serialize the full Elephant `Document` (exact NURBS, meshes, layers, tolerances) to JSON.
pub fn save_project_json(doc: &Document) -> Result<String, IoError> {
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
                geometry: obj.geometry.clone(),
            }
        })
        .collect();

    let proj = ElephantProject {
        version: 1,
        tolerances: doc.tolerances,
        layers,
        objects,
    };
    Ok(serde_json::to_string_pretty(&proj)?)
}

/// Load an Elephant `.elephant` JSON project into a fresh `Document`.
pub fn load_project_json(input: &str) -> Result<Document, IoError> {
    let proj: ElephantProject = serde_json::from_str(input)?;
    let mut doc = Document::new();
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
        doc.active_layer = new_keys[active_idx];

        for so in proj.objects {
            let lid = *new_keys.get(so.layer_index).unwrap_or(&doc.active_layer);
            doc.active_layer = lid;
            let oid = doc.add_object("Load", so.geometry);
            if let Some(obj) = doc.objects.get_mut(oid) {
                obj.name = so.name;
            }
        }
        doc.active_layer = new_keys[active_idx];
    }

    Ok(doc)
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
    }
}
