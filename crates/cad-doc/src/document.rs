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
    pub last_created: Vec<ObjectId>,
    next_group_id: u64,
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
        let active_layer = layers.insert(Layer::new("Default", [215, 220, 228, 255]));
        Self {
            tolerances: Tolerances::default(),
            layers,
            active_layer,
            objects: SlotMap::with_key(),
            last_created: Vec::new(),
            next_group_id: 1,
            spatial: SpatialIndex::default(),
            history: UndoHistory::default(),
        }
    }

    pub fn add_layer(&mut self, name: impl Into<String>, color: [u8; 4]) -> LayerId {
        self.layers.insert(Layer::new(name, color))
    }

    pub fn add_object(&mut self, label: impl Into<String>, geometry: Geometry) -> ObjectId {
        self.add_object_on_layer(label, self.active_layer, geometry)
    }

    pub fn add_object_on_layer(
        &mut self,
        label: impl Into<String>,
        layer_id: LayerId,
        geometry: Geometry,
    ) -> ObjectId {
        let obj = CadObject::new(layer_id, geometry);
        let id = self.objects.insert(obj);
        self.last_created = vec![id];
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
        let gid = self.objects.get(id).and_then(|o| o.group_id);
        if let Some(gid) = gid {
            for (_, obj) in &mut self.objects {
                if obj.group_id == Some(gid) && obj.visible {
                    obj.selected = selected;
                }
            }
        } else if let Some(obj) = self.objects.get_mut(id) {
            obj.selected = selected;
        }
    }

    pub fn clear_selection(&mut self) {
        for (_, obj) in &mut self.objects {
            obj.selected = false;
        }
    }

    pub fn select_all(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if !obj.visible {
                continue;
            }
            let unlocked = self
                .layers
                .get(obj.layer_id)
                .map(|l| l.visible && !l.locked)
                .unwrap_or(true);
            if unlocked {
                obj.selected = true;
                count += 1;
            }
        }
        count
    }

    pub fn invert_selection(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            let selectable = obj.visible
                && self
                    .layers
                    .get(obj.layer_id)
                    .map(|l| l.visible && !l.locked)
                    .unwrap_or(true);
            if selectable {
                obj.selected = !obj.selected;
                if obj.selected {
                    count += 1;
                }
            }
        }
        count
    }

    pub fn select_last(&mut self) -> usize {
        self.clear_selection();
        let ids = self.last_created.clone();
        let mut count = 0;
        for id in ids {
            if let Some(obj) = self.objects.get_mut(id) {
                if obj.visible {
                    obj.selected = true;
                    count += 1;
                }
            }
        }
        count
    }

    pub fn select_by_kind<F>(&mut self, pred: F) -> usize
    where
        F: Fn(&Geometry) -> bool,
    {
        self.clear_selection();
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            let selectable = obj.visible
                && self
                    .layers
                    .get(obj.layer_id)
                    .map(|l| l.visible && !l.locked)
                    .unwrap_or(true);
            if selectable && pred(&obj.geometry) {
                obj.selected = true;
                count += 1;
            }
        }
        count
    }

    pub fn select_layer_objects(&mut self, layer_id: LayerId) -> usize {
        self.clear_selection();
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if obj.layer_id == layer_id && obj.visible {
                obj.selected = true;
                count += 1;
            }
        }
        count
    }

    pub fn move_selected_to_layer(&mut self, layer_id: LayerId) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if obj.selected {
                obj.layer_id = layer_id;
                count += 1;
            }
        }
        count
    }

    pub fn hide_selected(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if obj.selected {
                obj.visible = false;
                obj.selected = false;
                count += 1;
            }
        }
        count
    }

    pub fn show_all(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if !obj.visible {
                obj.visible = true;
                count += 1;
            }
        }
        count
    }

    pub fn isolate_selected(&mut self) -> usize {
        let any_sel = self.objects.values().any(|o| o.selected);
        if !any_sel {
            return 0;
        }
        let mut hidden = 0;
        for (_, obj) in &mut self.objects {
            if !obj.selected && obj.visible {
                obj.visible = false;
                hidden += 1;
            }
        }
        hidden
    }

    pub fn group_selected(&mut self) -> Option<(u64, usize)> {
        let count = self.objects.values().filter(|o| o.selected).count();
        if count < 2 {
            return None;
        }
        let gid = self.next_group_id;
        self.next_group_id += 1;
        for (_, obj) in &mut self.objects {
            if obj.selected {
                obj.group_id = Some(gid);
            }
        }
        Some((gid, count))
    }

    pub fn ungroup_selected(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if obj.selected && obj.group_id.is_some() {
                obj.group_id = None;
                count += 1;
            }
        }
        count
    }

    pub fn selection_aabb(&self) -> Option<Aabb> {
        let mut out = Aabb::EMPTY;
        let mut any = false;
        for (_, obj) in &self.objects {
            if obj.selected && obj.visible {
                out = out.union(&obj.aabb);
                any = true;
            }
        }
        if any && !out.is_empty() {
            Some(out)
        } else {
            None
        }
    }

    /// Pick the closest visible, unlocked object intersected by `ray`.
    pub fn pick_ray(&self, ray: &Ray) -> Option<ObjectId> {
        for (id, _) in self.spatial.query_ray_candidates(ray) {
            let Some(obj) = self.objects.get(id) else {
                continue;
            };
            if !obj.visible {
                continue;
            }
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
