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
    GdtFrame,
    DatumTag,
}

/// ISO 1101 / ASME Y14.5 Geometric Dimensioning & Tolerancing (GD&T) characteristic symbol.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GdtSymbol {
    Flatness,
    Straightness,
    Circularity,
    Cylindricity,
    ProfileSurface,
    ProfileLine,
    Perpendicularity,
    Parallelism,
    Angularity,
    Position,
    Concentricity,
    Runout,
    TotalRunout,
}

impl GdtSymbol {
    pub const ALL: [Self; 13] = [
        Self::Flatness,
        Self::Straightness,
        Self::Circularity,
        Self::Cylindricity,
        Self::ProfileSurface,
        Self::ProfileLine,
        Self::Perpendicularity,
        Self::Parallelism,
        Self::Angularity,
        Self::Position,
        Self::Concentricity,
        Self::Runout,
        Self::TotalRunout,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Flatness => "Flatness",
            Self::Straightness => "Straightness",
            Self::Circularity => "Circularity",
            Self::Cylindricity => "Cylindricity",
            Self::ProfileSurface => "Profile of a Surface",
            Self::ProfileLine => "Profile of a Line",
            Self::Perpendicularity => "Perpendicularity",
            Self::Parallelism => "Parallelism",
            Self::Angularity => "Angularity",
            Self::Position => "Position",
            Self::Concentricity => "Concentricity",
            Self::Runout => "Circular Runout",
            Self::TotalRunout => "Total Runout",
        }
    }

    pub fn short_code(self) -> &'static str {
        match self {
            Self::Flatness => "FLT",
            Self::Straightness => "STR",
            Self::Circularity => "CIR",
            Self::Cylindricity => "CYL",
            Self::ProfileSurface => "PRS",
            Self::ProfileLine => "PRL",
            Self::Perpendicularity => "PER",
            Self::Parallelism => "PAR",
            Self::Angularity => "ANG",
            Self::Position => "POS",
            Self::Concentricity => "CON",
            Self::Runout => "RUN",
            Self::TotalRunout => "TRU",
        }
    }
}

/// GD&T Feature Control Frame specification attached to a document annotation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GdtSpec {
    pub symbol: GdtSymbol,
    pub tolerance: f64,
    #[serde(default)]
    pub diameter_zone: bool,
    #[serde(default)]
    pub modifier: Option<String>,
    #[serde(default)]
    pub datums: Vec<String>,
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
    #[serde(default)]
    pub gdt: Option<GdtSpec>,
}

/// Per-object physically based studio material properties.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct MaterialProps {
    pub metallic: f32,
    pub roughness: f32,
    pub opacity: f32,
}

impl Default for MaterialProps {
    fn default() -> Self {
        Self {
            metallic: 0.05,
            roughness: 0.42,
            opacity: 1.0,
        }
    }
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
    #[serde(default)]
    pub material: Option<MaterialProps>,
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
            material: None,
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
