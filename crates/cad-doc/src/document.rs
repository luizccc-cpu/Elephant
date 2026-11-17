use cad_geom::{Aabb, DMat4, Geometry, Ray, Tolerances};
use slotmap::SlotMap;

use crate::history::{UndoAction, UndoHistory, UndoTransaction};
use crate::layer::{Layer, LayerId};
use crate::object::{CadObject, ObjectId};
use crate::spatial::{SpatialEntry, SpatialIndex};

/// Central Elephant CAD document holding layers, objects, selection, spatial index, and Undo/Redo.
pub struct Document {
    pub tolerances: Tolerances,
    pub layers: SlotMap<LayerId, Layer>,
    pub active_layer: LayerId,
    pub objects: SlotMap<ObjectId, CadObject>,
    spatial: SpatialIndex,
    history: UndoHistory,
}

impl Default for Document {
    fn default() -> Self {
        Self::new()
    }
}

impl Document {
    pub fn new() -> Self {
        let mut layers = SlotMap::with_key();
        let active_layer = layers.insert(Layer::new("Default", [20, 20, 20, 255]));
        Self {
            tolerances: Tolerances::default(),
            layers,
            active_layer,
            objects: SlotMap::with_key(),
            spatial: SpatialIndex::default(),
            history: UndoHistory::default(),
        }
    }

    pub fn add_layer(&mut self, name: impl Into<String>, color: [u8; 4]) -> LayerId {
        self.layers.insert(Layer::new(name, color))
    }

    pub fn add_object(&mut self, label: impl Into<String>, geometry: Geometry) -> ObjectId {
        let obj = CadObject::new(self.active_layer, geometry);
        let id = self.objects.insert(obj);
        self.history.push(UndoTransaction {
            label: label.into(),
            actions: vec![UndoAction::Added(id)],
        });
        self.rebuild_spatial_index();
        id
    }

    pub fn remove_selected(&mut self) -> usize {
        let selected: Vec<ObjectId> = self
            .objects
            .iter()
            .filter(|(_, o)| o.selected)
            .map(|(id, _)| id)
            .collect();
        if selected.is_empty() {
            return 0;
        }
        let mut actions = Vec::with_capacity(selected.len());
        for id in selected {
            if let Some(obj) = self.objects.remove(id) {
                actions.push(UndoAction::Removed(id, obj));
            }
        }
        let count = actions.len();
        self.history.push(UndoTransaction {
            label: "Delete".to_string(),
            actions,
        });
        self.rebuild_spatial_index();
        count
    }

    pub fn transform_selected(&mut self, label: impl Into<String>, mat: &DMat4) -> usize {
        let selected: Vec<ObjectId> = self
            .objects
            .iter()
            .filter(|(_, o)| o.selected)
            .map(|(id, _)| id)
            .collect();
        if selected.is_empty() {
            return 0;
        }
        let mut actions = Vec::with_capacity(selected.len());
        for id in selected {
            if let Some(obj) = self.objects.get_mut(id) {
                let before = obj.clone();
                obj.geometry.transform(mat);
                obj.refresh_aabb();
                let after = obj.clone();
                actions.push(UndoAction::Modified { id, before, after });
            }
        }
        let count = actions.len();
        self.history.push(UndoTransaction {
            label: label.into(),
            actions,
        });
        self.rebuild_spatial_index();
        count
    }

    pub fn set_selected(&mut self, id: ObjectId, selected: bool) {
        if let Some(obj) = self.objects.get_mut(id) {
            obj.selected = selected;
        }
    }

    pub fn clear_selection(&mut self) {
        for (_, obj) in &mut self.objects {
            obj.selected = false;
        }
    }

    /// Pick the closest visible, unlocked object intersected by `ray`.
    pub fn pick_ray(&self, ray: &Ray) -> Option<ObjectId> {
        for (id, _) in self.spatial.query_ray_candidates(ray) {
            let Some(obj) = self.objects.get(id) else {
                continue;
            };
            let Some(layer) = self.layers.get(obj.layer_id) else {
                continue;
            };
            if !layer.visible || layer.locked {
                continue;
            }
            match &obj.geometry {
                Geometry::Mesh(m) => {
                    if m.ray_intersect(ray).is_some() {
                        return Some(id);
                    }
                }
                _ => return Some(id),
            }
        }
        None
    }

    pub fn query_aabb(&self, aabb: &Aabb) -> Vec<ObjectId> {
        self.spatial.query_aabb(aabb)
    }

    pub fn undo(&mut self) -> Option<String> {
        let tx = self.history.pop_undo()?;
        let label = tx.label.clone();
        let mut redo_actions = Vec::with_capacity(tx.actions.len());

        for action in tx.actions.into_iter().rev() {
            match action {
                UndoAction::Added(id) => {
                    if let Some(removed) = self.objects.remove(id) {
                        redo_actions.push(UndoAction::Removed(id, removed));
                    }
                }
                UndoAction::Removed(id, obj) => {
                    let new_id = self.objects.insert(obj);
                    redo_actions.push(UndoAction::Added(new_id));
                    let _ = id;
                }
                UndoAction::Modified { id, before, after } => {
                    if let Some(obj) = self.objects.get_mut(id) {
                        *obj = before.clone();
                        redo_actions.push(UndoAction::Modified { id, before, after });
                    }
                }
            }
        }
        redo_actions.reverse();
        self.history.push_redo(UndoTransaction {
            label: label.clone(),
            actions: redo_actions,
        });
        self.rebuild_spatial_index();
        Some(label)
    }

    pub fn redo(&mut self) -> Option<String> {
        let tx = self.history.pop_redo()?;
        let label = tx.label.clone();
        let mut undo_actions = Vec::with_capacity(tx.actions.len());

        for action in tx.actions.into_iter().rev() {
            match action {
                UndoAction::Added(id) => {
                    if let Some(removed) = self.objects.remove(id) {
                        undo_actions.push(UndoAction::Removed(id, removed));
                    }
                }
                UndoAction::Removed(id, obj) => {
                    let new_id = self.objects.insert(obj);
                    undo_actions.push(UndoAction::Added(new_id));
                    let _ = id;
                }
                UndoAction::Modified { id, before, after } => {
                    if let Some(obj) = self.objects.get_mut(id) {
                        *obj = after.clone();
                        undo_actions.push(UndoAction::Modified { id, before, after });
                    }
                }
            }
        }
        undo_actions.reverse();
        self.history.push_undo_without_clearing_redo(UndoTransaction {
            label: label.clone(),
            actions: undo_actions,
        });
        self.rebuild_spatial_index();
        Some(label)
    }

    pub fn can_undo(&self) -> bool {
        self.history.can_undo()
    }

    pub fn can_redo(&self) -> bool {
        self.history.can_redo()
    }

    pub fn rebuild_spatial_index(&mut self) {
        let entries = self
            .objects
            .iter()
            .map(|(id, obj)| SpatialEntry::new(id, obj.aabb))
            .collect();
        self.spatial.rebuild(entries);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geom::{DVec3, TriangleMesh};

    #[test]
    fn add_transform_undo_redo_cycle() {
        let mut doc = Document::new();
        let mesh = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let id = doc.add_object("Box", Geometry::Mesh(mesh));
        assert_eq!(doc.objects.len(), 1);

        doc.set_selected(id, true);
        let moved = doc.transform_selected("Move", &DMat4::from_translation(DVec3::new(10.0, 0.0, 0.0)));
        assert_eq!(moved, 1);
        assert!((doc.objects[id].aabb.center().x - 10.0).abs() < 1e-9);

        assert_eq!(doc.undo().as_deref(), Some("Move"));
        assert!((doc.objects[id].aabb.center().x - 0.0).abs() < 1e-9);

        assert_eq!(doc.redo().as_deref(), Some("Move"));
        assert!((doc.objects[id].aabb.center().x - 10.0).abs() < 1e-9);

        assert_eq!(doc.undo().as_deref(), Some("Move"));
        assert_eq!(doc.undo().as_deref(), Some("Box"));
        assert_eq!(doc.objects.len(), 0);
    }

    #[test]
    fn ray_pick_hits_visible_box() {
        let mut doc = Document::new();
        let mesh = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let id = doc.add_object("Box", Geometry::Mesh(mesh));
        let ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray(&ray), Some(id));
    }
}
