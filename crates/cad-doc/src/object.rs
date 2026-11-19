use std::collections::BTreeMap;
use std::ops::{Index, IndexMut};

use cad_geom::{Aabb, DVec3, Geometry};
use serde::{Deserialize, Serialize};

use crate::layer::LayerId;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AnnotationKind {
    Dimension,
    Text,
    TextDot,
    Leader,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AnnotationItem {
    pub kind: AnnotationKind,
    pub layer_id: LayerId,
    pub color: Option<[u8; 4]>,
    pub text: String,
    pub position: DVec3,
    pub p1: Option<DVec3>,
    pub p2: Option<DVec3>,
    pub offset_pt: Option<DVec3>,
}

/// Stable monotonic object identifier preserved across Undo/Redo delete-and-restore cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ObjectId(pub u64);

fn default_visible() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CadObject {
    pub name: Option<String>,
    pub layer_id: LayerId,
    pub color: Option<[u8; 4]>,
    #[serde(default = "default_visible")]
    pub visible: bool,
    #[serde(default)]
    pub group_id: Option<u64>,
    pub geometry: Geometry,
    pub selected: bool,
    pub aabb: Aabb,
}

impl CadObject {
    pub fn new(layer_id: LayerId, geometry: Geometry) -> Self {
        let aabb = geometry.bounding_box();
        Self {
            name: None,
            layer_id,
            color: None,
            visible: true,
            group_id: None,
            geometry,
            selected: false,
            aabb,
        }
    }

    pub fn refresh_aabb(&mut self) {
        self.aabb = self.geometry.bounding_box();
    }
}

/// Ordered store of `CadObject`s keyed by stable `ObjectId`.
#[derive(Debug, Clone)]
pub struct ObjectStore {
    map: BTreeMap<ObjectId, CadObject>,
    next_id: u64,
}

impl Default for ObjectStore {
    fn default() -> Self {
        Self::new()
    }
}

impl ObjectStore {
    pub fn new() -> Self {
        Self {
            map: BTreeMap::new(),
            next_id: 1,
        }
    }

    pub fn insert_new(&mut self, obj: CadObject) -> ObjectId {
        let id = ObjectId(self.next_id);
        self.next_id = self.next_id.saturating_add(1);
        self.map.insert(id, obj);
        id
    }

    pub fn insert_with_id(&mut self, id: ObjectId, obj: CadObject) {
        self.next_id = self.next_id.max(id.0.saturating_add(1));
        self.map.insert(id, obj);
    }

    pub fn remove(&mut self, id: ObjectId) -> Option<CadObject> {
        self.map.remove(&id)
    }

    pub fn get(&self, id: ObjectId) -> Option<&CadObject> {
        self.map.get(&id)
    }

    pub fn get_mut(&mut self, id: ObjectId) -> Option<&mut CadObject> {
        self.map.get_mut(&id)
    }

    pub fn len(&self) -> usize {
        self.map.len()
    }

    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    pub fn clear(&mut self) {
        self.map.clear();
    }

    pub fn iter(&self) -> impl Iterator<Item = (ObjectId, &CadObject)> {
        self.map.iter().map(|(&id, obj)| (id, obj))
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ObjectId, &mut CadObject)> {
        self.map.iter_mut().map(|(&id, obj)| (id, obj))
    }

    pub fn keys(&self) -> impl Iterator<Item = ObjectId> + '_ {
        self.map.keys().copied()
    }

    pub fn values(&self) -> impl Iterator<Item = &CadObject> {
        self.map.values()
    }

    pub fn values_mut(&mut self) -> impl Iterator<Item = &mut CadObject> {
        self.map.values_mut()
    }
}

impl Index<ObjectId> for ObjectStore {
    type Output = CadObject;

    fn index(&self, index: ObjectId) -> &Self::Output {
        &self.map[&index]
    }
}

impl IndexMut<ObjectId> for ObjectStore {
    fn index_mut(&mut self, index: ObjectId) -> &mut Self::Output {
        self.map.get_mut(&index).expect("ObjectId not in ObjectStore")
    }
}

impl<'a> IntoIterator for &'a ObjectStore {
    type Item = (ObjectId, &'a CadObject);
    type IntoIter = std::iter::Map<
        std::collections::btree_map::Iter<'a, ObjectId, CadObject>,
        fn((&'a ObjectId, &'a CadObject)) -> (ObjectId, &'a CadObject),
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.map.iter().map(|(&id, obj)| (id, obj))
    }
}

impl<'a> IntoIterator for &'a mut ObjectStore {
    type Item = (ObjectId, &'a mut CadObject);
    type IntoIter = std::iter::Map<
        std::collections::btree_map::IterMut<'a, ObjectId, CadObject>,
        fn((&'a ObjectId, &'a mut CadObject)) -> (ObjectId, &'a mut CadObject),
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.map.iter_mut().map(|(&id, obj)| (id, obj))
    }
}
