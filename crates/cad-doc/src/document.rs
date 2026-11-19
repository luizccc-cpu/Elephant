use std::collections::{HashSet, VecDeque};

use cad_geom::{Aabb, DMat4, DVec3, Geometry, Ray, Tolerances, UnitSystem};
use slotmap::SlotMap;

use crate::history::{UndoAction, UndoHistory, UndoTransaction};
use crate::layer::{Layer, LayerId};
use crate::layout::{ClippingPlane, LayoutSheet, PageSize};
use crate::modeling_history::{HistoryOp, HistoryRecord};
use crate::object::{AnnotationItem, AnnotationKind, CadObject, GdtSpec, ObjectId, ObjectStore};
use crate::spatial::{SpatialEntry, SpatialIndex};

/// Central Elephant CAD document holding layers, objects, selection, spatial index, and Undo/Redo.
pub struct Document {
    pub units: UnitSystem,
    pub tolerances: Tolerances,
    pub layers: SlotMap<LayerId, Layer>,
    pub active_layer: LayerId,
    pub objects: ObjectStore,
    pub annotations: Vec<AnnotationItem>,
    pub clipping_planes: Vec<ClippingPlane>,
    pub layouts: Vec<LayoutSheet>,
    pub last_created: Vec<ObjectId>,
    pub record_history: bool,
    pub history_records: Vec<HistoryRecord>,
    selection_order: Vec<ObjectId>,
    next_group_id: u64,
    next_clipping_id: u64,
    next_layout_id: u64,
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
        let units = UnitSystem::default();
        Self {
            units,
            tolerances: units.default_tolerances(),
            layers,
            active_layer,
            objects: ObjectStore::new(),
            annotations: Vec::new(),
            clipping_planes: Vec::new(),
            layouts: Vec::new(),
            last_created: Vec::new(),
            record_history: true,
            history_records: Vec::new(),
            selection_order: Vec::new(),
            next_group_id: 1,
            next_clipping_id: 1,
            next_layout_id: 1,
            spatial: SpatialIndex::default(),
            history: UndoHistory::default(),
        }
    }

    pub fn set_units(&mut self, target: UnitSystem, rescale_geometry: bool) {
        let old = self.units;
        self.units = target;
        self.tolerances = target.default_tolerances();
        if rescale_geometry && old != target {
            let s = old.scale_to(target);
            let mat = DMat4::from_scale(DVec3::splat(s));
            for (_, obj) in &mut self.objects {
                obj.geometry.transform(&mat);
                obj.refresh_aabb();
            }
            for ann in &mut self.annotations {
                ann.position *= s;
                if let Some(p1) = &mut ann.p1 {
                    *p1 *= s;
                }
                if let Some(p2) = &mut ann.p2 {
                    *p2 *= s;
                }
                if let Some(off) = &mut ann.offset_pt {
                    *off *= s;
                }
            }
            self.rebuild_spatial_index();
        }
    }

    pub fn ensure_next_group_id_above(&mut self, max_gid: u64) {
        self.next_group_id = self.next_group_id.max(max_gid.saturating_add(1));
    }

    pub fn clear_history(&mut self) {
        self.history.clear();
    }

    pub fn visible_bounding_box(&self) -> Aabb {
        let mut total = Aabb::EMPTY;
        for (_, obj) in &self.objects {
            if !obj.visible || !self.is_layer_effectively_visible(obj.layer_id) {
                continue;
            }
            total = total.union(&obj.aabb);
        }
        total
    }

    pub fn add_layer(&mut self, name: impl Into<String>, color: [u8; 4]) -> LayerId {
        self.layers.insert(Layer::new(name, color))
    }

    pub fn add_sublayer(
        &mut self,
        name: impl Into<String>,
        color: [u8; 4],
        parent: LayerId,
    ) -> LayerId {
        self.layers.insert(Layer::new_child(name, color, parent))
    }

    pub fn is_layer_effectively_visible(&self, lid: LayerId) -> bool {
        let mut cur = Some(lid);
        for _ in 0..32 {
            let Some(id) = cur else {
                break;
            };
            let Some(layer) = self.layers.get(id) else {
                return false;
            };
            if !layer.visible {
                return false;
            }
            cur = layer.parent;
        }
        true
    }

    pub fn is_layer_effectively_locked(&self, lid: LayerId) -> bool {
        let mut cur = Some(lid);
        for _ in 0..32 {
            let Some(id) = cur else {
                break;
            };
            let Some(layer) = self.layers.get(id) else {
                return false;
            };
            if layer.locked {
                return true;
            }
            cur = layer.parent;
        }
        false
    }

    pub fn ordered_layer_tree(&self) -> Vec<(LayerId, usize)> {
        let all_ids: Vec<LayerId> = self.layers.keys().collect();
        let mut out = Vec::with_capacity(all_ids.len());
        let mut visited = HashSet::with_capacity(all_ids.len());
        for &id in &all_ids {
            let is_root = self
                .layers
                .get(id)
                .and_then(|l| l.parent)
                .is_none_or(|p| !self.layers.contains_key(p) || p == id);
            if is_root {
                self.collect_layer_children(id, 0, &all_ids, &mut visited, &mut out);
            }
        }
        // Append any cyclic orphans cleanly
        for &id in &all_ids {
            if !visited.contains(&id) {
                out.push((id, 0));
            }
        }
        out
    }

    fn collect_layer_children(
        &self,
        id: LayerId,
        depth: usize,
        all_ids: &[LayerId],
        visited: &mut HashSet<LayerId>,
        out: &mut Vec<(LayerId, usize)>,
    ) {
        if !visited.insert(id) || depth > 32 {
            return;
        }
        out.push((id, depth));
        for &cand in all_ids {
            if self.layers.get(cand).and_then(|l| l.parent) == Some(id) {
                self.collect_layer_children(cand, depth + 1, all_ids, visited, out);
            }
        }
    }

    pub fn add_annotation(&mut self, item: AnnotationItem) {
        self.annotations.push(item);
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
        let id = self.objects.insert_new(obj);
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
        self.sync_selection_order();
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
        for &id in &selected {
            if let Some(obj) = self.objects.get_mut(id) {
                let before = obj.clone();
                obj.geometry.transform(mat);
                obj.refresh_aabb();
                let after = obj.clone();
                actions.push(UndoAction::Modified { id, before, after });
            }
        }
        let count = selected.len();
        self.apply_history_replay(&selected, &mut actions);
        self.history.push(UndoTransaction {
            label: label.into(),
            actions,
        });
        self.rebuild_spatial_index();
        count
    }

    /// Record an Undo transaction for an already-applied modification on a single object.
    pub fn record_object_modified(
        &mut self,
        label: impl Into<String>,
        id: ObjectId,
        before: CadObject,
    ) {
        if let Some(after) = self.objects.get(id).cloned() {
            let mut actions = vec![UndoAction::Modified { id, before, after }];
            self.apply_history_replay(&[id], &mut actions);
            self.history.push(UndoTransaction {
                label: label.into(),
                actions,
            });
            self.rebuild_spatial_index();
        }
    }

    /// Atomically replace `target_id` with `replacements` in a single Undo transaction.
    pub fn replace_object_with_many(
        &mut self,
        label: impl Into<String>,
        target_id: ObjectId,
        replacements: Vec<(LayerId, Geometry)>,
    ) -> Vec<ObjectId> {
        let mut actions = Vec::with_capacity(1 + replacements.len());
        let mut created_ids = Vec::with_capacity(replacements.len());
        if let Some(removed) = self.objects.remove(target_id) {
            actions.push(UndoAction::Removed(target_id, removed));
        }
        for (lid, geom) in replacements {
            let new_id = self.objects.insert_new(CadObject::new(lid, geom));
            actions.push(UndoAction::Added(new_id));
            created_ids.push(new_id);
        }
        if !created_ids.is_empty() {
            self.last_created = created_ids.clone();
        }
        self.history.push(UndoTransaction {
            label: label.into(),
            actions,
        });
        self.sync_selection_order();
        self.rebuild_spatial_index();
        created_ids
    }

    pub fn set_selected(&mut self, id: ObjectId, selected: bool) {
        let gid = self.objects.get(id).and_then(|o| o.group_id);
        let mut touched = Vec::new();
        if let Some(gid) = gid {
            for (oid, obj) in &mut self.objects {
                if obj.group_id == Some(gid) && obj.visible {
                    obj.selected = selected;
                    touched.push(oid);
                }
            }
        } else if let Some(obj) = self.objects.get_mut(id) {
            obj.selected = selected;
            touched.push(id);
        }
        if selected {
            for oid in touched {
                if !self.selection_order.contains(&oid) {
                    self.selection_order.push(oid);
                }
            }
        } else {
            self.selection_order.retain(|oid| !touched.contains(oid));
        }
    }

    pub fn clear_selection(&mut self) {
        for (_, obj) in &mut self.objects {
            obj.selected = false;
        }
        self.selection_order.clear();
    }

    /// Selected visible object IDs in chronological selection order.
    pub fn selected_ids_ordered(&self) -> Vec<ObjectId> {
        let mut out: Vec<ObjectId> = self
            .selection_order
            .iter()
            .copied()
            .filter(|id| self.objects.get(*id).is_some_and(|o| o.visible && o.selected))
            .collect();
        for (id, obj) in &self.objects {
            if obj.visible && obj.selected && !out.contains(&id) {
                out.push(id);
            }
        }
        out
    }

    fn sync_selection_order(&mut self) {
        self.selection_order
            .retain(|id| self.objects.get(*id).is_some_and(|o| o.visible && o.selected));
        for (id, obj) in &self.objects {
            if obj.visible && obj.selected && !self.selection_order.contains(&id) {
                self.selection_order.push(id);
            }
        }
    }

    pub fn select_all(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            if !obj.visible {
                continue;
            }
            let unlocked = layer_effectively_visible(&self.layers, obj.layer_id)
                && !layer_effectively_locked(&self.layers, obj.layer_id);
            if unlocked {
                obj.selected = true;
                count += 1;
            }
        }
        self.sync_selection_order();
        count
    }

    pub fn invert_selection(&mut self) -> usize {
        let mut count = 0;
        for (_, obj) in &mut self.objects {
            let selectable = obj.visible
                && layer_effectively_visible(&self.layers, obj.layer_id)
                && !layer_effectively_locked(&self.layers, obj.layer_id);
            if selectable {
                obj.selected = !obj.selected;
                if obj.selected {
                    count += 1;
                }
            }
        }
        self.sync_selection_order();
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
        self.sync_selection_order();
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
                && layer_effectively_visible(&self.layers, obj.layer_id)
                && !layer_effectively_locked(&self.layers, obj.layer_id);
            if selectable && pred(&obj.geometry) {
                obj.selected = true;
                count += 1;
            }
        }
        self.sync_selection_order();
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
        self.sync_selection_order();
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
        self.sync_selection_order();
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
        self.pick_ray_with_tolerance(ray, 0.25)
    }

    /// Pick the closest visible, unlocked object intersected by `ray` within `pick_tolerance` for curves/points.
    pub fn pick_ray_with_tolerance(&self, ray: &Ray, pick_tolerance: f64) -> Option<ObjectId> {
        let tol = pick_tolerance.max(1e-6);
        let mut best_curve: Option<(f64, f64, ObjectId)> = None;
        let mut best_surf: Option<(f64, ObjectId)> = None;

        for (id, _) in self.spatial.query_ray_candidates_expanded(ray, tol) {
            let Some(obj) = self.objects.get(id) else {
                continue;
            };
            if !obj.visible
                || !self.is_layer_effectively_visible(obj.layer_id)
                || self.is_layer_effectively_locked(obj.layer_id)
            {
                continue;
            }
            match &obj.geometry {
                Geometry::Point(p) => {
                    let (t, d) = ray.distance_to_point(*p);
                    if d <= tol
                        && best_curve.is_none_or(|b| d < b.0 || ((d - b.0).abs() < 1e-9 && t < b.1))
                    {
                        best_curve = Some((d, t, id));
                    }
                }
                Geometry::Curve(c) => {
                    if let Some((t, d)) = c.ray_distance(ray, 48) {
                        if d <= tol
                            && best_curve
                                .is_none_or(|b| d < b.0 || ((d - b.0).abs() < 1e-9 && t < b.1))
                        {
                            best_curve = Some((d, t, id));
                        }
                    }
                }
                Geometry::Surface(s) => {
                    if let Some(t) = s.ray_intersect(ray) {
                        if best_surf.is_none_or(|b| t < b.0) {
                            best_surf = Some((t, id));
                        }
                    }
                }
                Geometry::BRep(b) => {
                    if let Some(t) = b.ray_intersect(ray) {
                        if best_surf.is_none_or(|b_best| t < b_best.0) {
                            best_surf = Some((t, id));
                        }
                    }
                }
                Geometry::SubD(sd) => {
                    if let Some(t) = sd.ray_intersect(ray) {
                        if best_surf.is_none_or(|b| t < b.0) {
                            best_surf = Some((t, id));
                        }
                    }
                }
                Geometry::Mesh(m) => {
                    if let Some(t) = m.ray_intersect(ray) {
                        if best_surf.is_none_or(|b| t < b.0) {
                            best_surf = Some((t, id));
                        }
                    }
                }
            }
        }

        match (best_curve, best_surf) {
            (Some((_, ct, cid)), Some((st, sid))) => {
                if ct <= st + tol * 2.0 {
                    Some(cid)
                } else {
                    Some(sid)
                }
            }
            (Some((_, _, cid)), None) => Some(cid),
            (None, Some((_, sid))) => Some(sid),
            (None, None) => None,
        }
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
                    self.objects.insert_with_id(id, obj);
                    redo_actions.push(UndoAction::Added(id));
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
        self.sync_selection_order();
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
                    self.objects.insert_with_id(id, obj);
                    undo_actions.push(UndoAction::Added(id));
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
        self.sync_selection_order();
        self.rebuild_spatial_index();
        Some(label)
    }

    pub fn can_undo(&self) -> bool {
        self.history.can_undo()
    }

    pub fn can_redo(&self) -> bool {
        self.history.can_redo()
    }

    pub fn add_history_record(
        &mut self,
        child_id: ObjectId,
        parent_ids: Vec<ObjectId>,
        op: HistoryOp,
    ) {
        if !self.record_history || parent_ids.is_empty() || parent_ids.contains(&child_id) {
            return;
        }
        self.history_records.retain(|r| r.child_id != child_id);
        self.history_records.push(HistoryRecord {
            child_id,
            parent_ids,
            op,
        });
    }

    pub fn break_history_for(&mut self, child_id: ObjectId) -> bool {
        let before = self.history_records.len();
        self.history_records.retain(|r| r.child_id != child_id);
        self.history_records.len() != before
    }

    pub fn purge_broken_history(&mut self) -> usize {
        let before = self.history_records.len();
        self.history_records.retain(|r| {
            self.objects.get(r.child_id).is_some()
                && !r.parent_ids.is_empty()
                && r.parent_ids.iter().all(|&pid| self.objects.get(pid).is_some())
        });
        before - self.history_records.len()
    }

    pub fn history_record_for(&self, child_id: ObjectId) -> Option<&HistoryRecord> {
        self.history_records.iter().find(|r| r.child_id == child_id)
    }

    pub fn history_children_of(&self, parent_id: ObjectId) -> Vec<ObjectId> {
        self.history_records
            .iter()
            .filter(|r| r.parent_ids.contains(&parent_id))
            .map(|r| r.child_id)
            .collect()
    }

    pub fn update_history_op(&mut self, child_id: ObjectId, new_op: HistoryOp) -> bool {
        let Some(pos) = self
            .history_records
            .iter()
            .position(|r| r.child_id == child_id)
        else {
            return false;
        };
        let parent_ids = self.history_records[pos].parent_ids.clone();
        self.history_records[pos].op = new_op.clone();

        let parent_geoms: Option<Vec<Geometry>> = parent_ids
            .iter()
            .map(|&pid| self.objects.get(pid).map(|o| o.geometry.clone()))
            .collect();
        let Some(parent_geoms) = parent_geoms else {
            return false;
        };
        let parent_refs: Vec<&Geometry> = parent_geoms.iter().collect();
        let tol = self.tolerances.absolute.max(1e-4);
        let Some(new_geom) = new_op.evaluate(&parent_refs, tol) else {
            return false;
        };
        let Some(obj) = self.objects.get_mut(child_id) else {
            return false;
        };
        let before = obj.clone();
        obj.geometry = new_geom;
        obj.refresh_aabb();
        let after = obj.clone();

        let mut actions = vec![UndoAction::Modified {
            id: child_id,
            before,
            after,
        }];
        self.apply_history_replay(&[child_id], &mut actions);
        self.history.push(UndoTransaction {
            label: "Edit History".to_string(),
            actions,
        });
        self.rebuild_spatial_index();
        true
    }

    fn apply_history_replay(
        &mut self,
        modified_ids: &[ObjectId],
        undo_actions: &mut Vec<UndoAction>,
    ) -> Vec<ObjectId> {
        if self.history_records.is_empty() || modified_ids.is_empty() {
            return Vec::new();
        }
        // Non-blocking detach: editing a child directly without its parents breaks its link cleanly.
        for &mid in modified_ids {
            let should_break = self
                .history_records
                .iter()
                .find(|r| r.child_id == mid)
                .is_some_and(|rec| !rec.parent_ids.iter().any(|pid| modified_ids.contains(pid)));
            if should_break {
                self.break_history_for(mid);
            }
        }

        let tol = self.tolerances.absolute.max(1e-4);
        let mut queue: VecDeque<ObjectId> = modified_ids.iter().copied().collect();
        let mut updated_set: HashSet<ObjectId> = modified_ids.iter().copied().collect();
        let mut updated = Vec::new();

        while let Some(cur_id) = queue.pop_front() {
            let matching: Vec<(ObjectId, Vec<ObjectId>, HistoryOp)> = self
                .history_records
                .iter()
                .filter(|r| r.parent_ids.contains(&cur_id) && !updated_set.contains(&r.child_id))
                .map(|r| (r.child_id, r.parent_ids.clone(), r.op.clone()))
                .collect();

            for (child_id, parent_ids, op) in matching {
                let parent_geoms: Option<Vec<Geometry>> = parent_ids
                    .iter()
                    .map(|&pid| self.objects.get(pid).map(|o| o.geometry.clone()))
                    .collect();
                let Some(parent_geoms) = parent_geoms else {
                    continue;
                };
                let parent_refs: Vec<&Geometry> = parent_geoms.iter().collect();
                let Some(new_geom) = op.evaluate(&parent_refs, tol) else {
                    continue;
                };
                if let Some(obj) = self.objects.get_mut(child_id) {
                    let before = obj.clone();
                    obj.geometry = new_geom;
                    obj.refresh_aabb();
                    let after = obj.clone();
                    undo_actions.push(UndoAction::Modified {
                        id: child_id,
                        before,
                        after,
                    });
                    updated_set.insert(child_id);
                    updated.push(child_id);
                    queue.push_back(child_id);
                }
            }
        }
        updated
    }

    pub fn rebuild_spatial_index(&mut self) {
        let entries = self
            .objects
            .iter()
            .map(|(id, obj)| SpatialEntry::new(id, obj.aabb))
            .collect();
        self.spatial.rebuild(entries);
    }

    pub fn ensure_next_aux_ids(&mut self) {
        let max_c = self.clipping_planes.iter().map(|c| c.id).max().unwrap_or(0);
        self.next_clipping_id = self.next_clipping_id.max(max_c.saturating_add(1));
        let max_l = self.layouts.iter().map(|l| l.id).max().unwrap_or(0);
        self.next_layout_id = self.next_layout_id.max(max_l.saturating_add(1));
    }

    pub fn add_clipping_plane(&mut self, name: impl Into<String>, plane: cad_geom::Plane) -> u64 {
        let id = self.next_clipping_id;
        self.next_clipping_id = self.next_clipping_id.saturating_add(1);
        self.clipping_planes.push(ClippingPlane::new(id, name, plane));
        id
    }

    pub fn remove_clipping_plane(&mut self, id: u64) -> bool {
        let before = self.clipping_planes.len();
        self.clipping_planes.retain(|c| c.id != id);
        self.clipping_planes.len() != before
    }

    pub fn active_clipping_planes(&self) -> Vec<&ClippingPlane> {
        self.clipping_planes.iter().filter(|c| c.enabled).collect()
    }

    pub fn add_standard_4view_layout(
        &mut self,
        name: Option<String>,
        page_size: PageSize,
    ) -> u64 {
        let id = self.next_layout_id;
        self.next_layout_id = self.next_layout_id.saturating_add(1);
        let sheet_name = name.unwrap_or_else(|| format!("Sheet {id}"));
        self.layouts
            .push(LayoutSheet::standard_4_view(id, sheet_name, page_size));
        id
    }

    pub fn remove_layout(&mut self, id: u64) -> bool {
        let before = self.layouts.len();
        self.layouts.retain(|l| l.id != id);
        self.layouts.len() != before
    }

    pub fn add_gdt_annotation(&mut self, spec: GdtSpec, position: DVec3) {
        let datums_joined = if spec.datums.is_empty() {
            String::new()
        } else {
            format!(" | {}", spec.datums.join(" | "))
        };
        let dia = if spec.diameter_zone { "dia " } else { "" };
        let text = format!(
            "{} | {}{:.3}{}",
            spec.symbol.short_code(),
            dia,
            spec.tolerance,
            datums_joined
        );
        self.add_annotation(AnnotationItem {
            kind: AnnotationKind::GdtFrame,
            layer_id: self.active_layer,
            color: None,
            text,
            position,
            p1: None,
            p2: None,
            offset_pt: None,
            gdt: Some(spec),
        });
    }

    pub fn add_datum_annotation(&mut self, datum_label: impl Into<String>, position: DVec3) {
        let label = datum_label.into();
        self.add_annotation(AnnotationItem {
            kind: AnnotationKind::DatumTag,
            layer_id: self.active_layer,
            color: None,
            text: label,
            position,
            p1: None,
            p2: None,
            offset_pt: None,
            gdt: None,
        });
    }
}

fn layer_effectively_visible(layers: &SlotMap<LayerId, Layer>, lid: LayerId) -> bool {
    let mut cur = Some(lid);
    for _ in 0..32 {
        let Some(id) = cur else {
            break;
        };
        let Some(layer) = layers.get(id) else {
            return false;
        };
        if !layer.visible {
            return false;
        }
        cur = layer.parent;
    }
    true
}

fn layer_effectively_locked(layers: &SlotMap<LayerId, Layer>, lid: LayerId) -> bool {
    let mut cur = Some(lid);
    for _ in 0..32 {
        let Some(id) = cur else {
            break;
        };
        let Some(layer) = layers.get(id) else {
            return false;
        };
        if layer.locked {
            return true;
        }
        cur = layer.parent;
    }
    false
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

    #[test]
    fn undo_delete_then_undo_modify_preserves_object_id() {
        let mut doc = Document::new();
        let mesh = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let id = doc.add_object("Box", Geometry::Mesh(mesh));
        doc.set_selected(id, true);
        doc.transform_selected("Move", &DMat4::from_translation(DVec3::new(6.0, 0.0, 0.0)));
        assert_eq!(doc.remove_selected(), 1);
        assert!(doc.objects.is_empty());

        assert_eq!(doc.undo().as_deref(), Some("Delete"));
        assert!(doc.objects.get(id).is_some());
        assert!((doc.objects[id].aabb.center().x - 6.0).abs() < 1e-9);

        assert_eq!(doc.undo().as_deref(), Some("Move"));
        assert!((doc.objects[id].aabb.center().x - 0.0).abs() < 1e-9);

        assert_eq!(doc.redo().as_deref(), Some("Move"));
        assert!((doc.objects[id].aabb.center().x - 6.0).abs() < 1e-9);

        assert_eq!(doc.redo().as_deref(), Some("Delete"));
        assert!(doc.objects.is_empty());
    }

    #[test]
    fn pick_ray_hits_circle_edge_and_misses_hole() {
        use cad_geom::{NurbsCurve, Plane};
        let mut doc = Document::new();
        let circle = NurbsCurve::circle(&Plane::world_xy(), 5.0).unwrap();
        let id = doc.add_object("Circle", Geometry::Curve(circle));

        let center_ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray_with_tolerance(&center_ray, 0.25), None);

        let edge_ray = Ray::new(DVec3::new(5.05, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray_with_tolerance(&edge_ray, 0.25), Some(id));
    }

    #[test]
    fn pick_ray_hits_nurbs_surface() {
        use cad_geom::{NurbsSurface, Plane};
        let mut doc = Document::new();
        let surf = NurbsSurface::plane_rect(&Plane::world_xy(), 4.0, 4.0);
        let id = doc.add_object("Plane", Geometry::Surface(surf));

        let hit_ray = Ray::new(DVec3::new(1.0, -1.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray(&hit_ray), Some(id));

        let miss_ray = Ray::new(DVec3::new(3.5, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray(&miss_ray), None);
    }

    #[test]
    fn record_object_modified_undo_redo() {
        use cad_geom::NurbsCurve;
        let mut doc = Document::new();
        let id = doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0))),
        );
        let before = doc.objects[id].clone();
        if let Geometry::Curve(c) = &mut doc.objects[id].geometry {
            c.control_points[1].position = DVec3::new(4.0, 5.0, 0.0);
        }
        doc.objects[id].refresh_aabb();
        doc.record_object_modified("Move Control Point", id, before);

        assert_eq!(doc.undo().as_deref(), Some("Move Control Point"));
        let Geometry::Curve(c_after_undo) = &doc.objects[id].geometry else {
            panic!("expected curve");
        };
        assert!((c_after_undo.control_points[1].position - DVec3::new(4.0, 0.0, 0.0)).length() < 1e-9);

        assert_eq!(doc.redo().as_deref(), Some("Move Control Point"));
        let Geometry::Curve(c_after_redo) = &doc.objects[id].geometry else {
            panic!("expected curve");
        };
        assert!((c_after_redo.control_points[1].position - DVec3::new(4.0, 5.0, 0.0)).length() < 1e-9);
    }

    #[test]
    fn set_units_rescales_geometry_and_tolerances() {
        use cad_geom::NurbsCurve;
        let mut doc = Document::new();
        let id = doc.add_object(
            "Line",
            Geometry::Curve(NurbsCurve::line(DVec3::ZERO, DVec3::new(100.0, 0.0, 0.0))),
        );
        doc.set_units(UnitSystem::Centimeters, true);
        assert_eq!(doc.units, UnitSystem::Centimeters);
        assert!((doc.tolerances.absolute - 1e-4).abs() < 1e-12);
        let Geometry::Curve(c) = &doc.objects[id].geometry else {
            panic!("expected curve");
        };
        assert!((c.control_points[1].position.x - 10.0).abs() < 1e-9);
    }

    #[test]
    fn selected_ids_ordered_preserves_click_sequence() {
        let mut doc = Document::new();
        let a = doc.add_object("A", Geometry::Point(DVec3::X));
        let b = doc.add_object("B", Geometry::Point(DVec3::Y));
        let c = doc.add_object("C", Geometry::Point(DVec3::Z));
        doc.set_selected(c, true);
        doc.set_selected(a, true);
        doc.set_selected(b, true);
        assert_eq!(doc.selected_ids_ordered(), vec![c, a, b]);
        doc.set_selected(a, false);
        assert_eq!(doc.selected_ids_ordered(), vec![c, b]);
        doc.undo();
        assert!(doc.objects.get(c).is_none());
        assert_eq!(doc.selected_ids_ordered(), vec![b]);
    }

    #[test]
    fn nested_sublayers_inherit_visibility_and_lock() {
        let mut doc = Document::new();
        let root = doc.add_layer("Root", [200, 200, 200, 255]);
        let sub = doc.add_sublayer("Sub", [100, 200, 255, 255], root);
        let leaf = doc.add_sublayer("Leaf", [255, 180, 100, 255], sub);

        let tree = doc.ordered_layer_tree();
        let sub_pos = tree.iter().position(|&(id, _)| id == sub).unwrap();
        let leaf_pos = tree.iter().position(|&(id, _)| id == leaf).unwrap();
        assert_eq!(tree[sub_pos].1, 1);
        assert_eq!(tree[leaf_pos].1, 2);
        assert!(leaf_pos > sub_pos);

        let oid = doc.add_object_on_layer(
            "Box",
            leaf,
            Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0))),
        );
        let ray = Ray::new(DVec3::new(0.0, 0.0, 10.0), DVec3::new(0.0, 0.0, -1.0));
        assert_eq!(doc.pick_ray(&ray), Some(oid));

        doc.layers[root].visible = false;
        assert!(!doc.is_layer_effectively_visible(leaf));
        assert_eq!(doc.pick_ray(&ray), None);
        doc.layers[root].visible = true;

        doc.layers[sub].locked = true;
        assert!(doc.is_layer_effectively_locked(leaf));
        assert_eq!(doc.pick_ray(&ray), None);
    }

    #[test]
    fn modeling_history_updates_extrude_and_mirror_and_breaks_on_direct_child_edit() {
        use cad_geom::{NurbsCurve, NurbsSurface};
        let mut doc = Document::new();
        let line = NurbsCurve::line(DVec3::ZERO, DVec3::new(4.0, 0.0, 0.0));
        let p_id = doc.add_object("Profile", Geometry::Curve(line.clone()));

        let ext_srf = NurbsSurface::extrude_curve(&line, DVec3::new(0.0, 0.0, 5.0));
        let ext_id = doc.add_object("Extrude", Geometry::Surface(ext_srf));
        doc.add_history_record(
            ext_id,
            vec![p_id],
            HistoryOp::Extrude {
                direction: DVec3::new(0.0, 0.0, 5.0),
                solid: false,
            },
        );

        let mir_op = HistoryOp::Mirror {
            plane_origin: DVec3::ZERO,
            plane_normal: DVec3::X,
        };
        let mir_geom = mir_op.evaluate(&[&doc.objects[ext_id].geometry], 1e-4).unwrap();
        let mir_id = doc.add_object("Mirror", mir_geom);
        doc.add_history_record(mir_id, vec![ext_id], mir_op);

        // Moving parent line in +Y propagates through Extrude AND chained Mirror
        doc.clear_selection();
        doc.set_selected(p_id, true);
        doc.transform_selected("MoveParent", &DMat4::from_translation(DVec3::new(0.0, 6.0, 0.0)));
        assert!((doc.objects[ext_id].aabb.center().y - 6.0).abs() < 1e-6);
        assert!((doc.objects[mir_id].aabb.center().y - 6.0).abs() < 1e-6);

        // Single Undo reverts parent AND children together cleanly!
        assert_eq!(doc.undo().as_deref(), Some("MoveParent"));
        assert!((doc.objects[p_id].aabb.center().y - 0.0).abs() < 1e-6);
        assert!((doc.objects[ext_id].aabb.center().y - 0.0).abs() < 1e-6);
        assert!((doc.objects[mir_id].aabb.center().y - 0.0).abs() < 1e-6);

        // Directly transforming child breaks history link smoothly without blocking edit
        doc.clear_selection();
        doc.set_selected(ext_id, true);
        doc.transform_selected("DirectMoveChild", &DMat4::from_translation(DVec3::new(0.0, 2.0, 0.0)));
        assert!(doc.history_record_for(ext_id).is_none());
        assert!((doc.objects[mir_id].aabb.center().y - 2.0).abs() < 1e-6);

        // Now moving parent no longer alters detached ext_id
        doc.clear_selection();
        doc.set_selected(p_id, true);
        doc.transform_selected("MoveParentAgain", &DMat4::from_translation(DVec3::new(0.0, 10.0, 0.0)));
        assert!((doc.objects[ext_id].aabb.center().y - 2.0).abs() < 1e-6);
    }

    #[test]
    fn clipping_planes_layouts_and_gdt_work_on_document() {
        use crate::GdtSymbol;
        let mut doc = Document::new();
        let cp_id = doc.add_clipping_plane("Section Z", cad_geom::Plane::world_xy());
        assert_eq!(doc.active_clipping_planes().len(), 1);
        let eq = doc.clipping_planes[0].equation_f32();
        assert!((eq[2] - 1.0).abs() < 1e-6);
        doc.clipping_planes[0].flip = true;
        assert!((doc.clipping_planes[0].equation_f32()[2] + 1.0).abs() < 1e-6);
        assert!(doc.remove_clipping_plane(cp_id));
        assert!(doc.clipping_planes.is_empty());

        let lid = doc.add_standard_4view_layout(None, PageSize::A3);
        assert_eq!(doc.layouts.len(), 1);
        assert_eq!(doc.layouts[0].details.len(), 4);
        assert!(doc.remove_layout(lid));

        doc.add_gdt_annotation(
            GdtSpec {
                symbol: GdtSymbol::Position,
                tolerance: 0.05,
                diameter_zone: true,
                modifier: Some("M".to_string()),
                datums: vec!["A".to_string(), "B".to_string()],
            },
            DVec3::new(2.0, 3.0, 0.0),
        );
        doc.add_datum_annotation("A", DVec3::ZERO);
        assert_eq!(doc.annotations.len(), 2);
        assert_eq!(doc.annotations[0].kind, AnnotationKind::GdtFrame);
        assert_eq!(doc.annotations[1].kind, AnnotationKind::DatumTag);
    }
}
