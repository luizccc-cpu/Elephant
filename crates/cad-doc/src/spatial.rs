use cad_math::{Aabb, Ray};
use rstar::{AABB, RTree, RTreeObject};

use crate::object::ObjectId;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpatialEntry {
    pub id: ObjectId,
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl SpatialEntry {
    pub fn new(id: ObjectId, aabb: Aabb) -> Self {
        Self {
            id,
            min: aabb.min.to_array(),
            max: aabb.max.to_array(),
        }
    }

    pub fn to_aabb(&self) -> Aabb {
        Aabb {
            min: self.min.into(),
            max: self.max.into(),
        }
    }
}

impl RTreeObject for SpatialEntry {
    type Envelope = AABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.min, self.max)
    }
}

/// R-tree spatial index over document objects for fast viewport picking and box queries.
#[derive(Debug, Default)]
pub struct SpatialIndex {
    tree: RTree<SpatialEntry>,
}

impl SpatialIndex {
    pub fn rebuild(&mut self, entries: Vec<SpatialEntry>) {
        self.tree = RTree::bulk_load(entries);
    }

    pub fn query_aabb(&self, aabb: &Aabb) -> Vec<ObjectId> {
        if aabb.is_empty() {
            return Vec::new();
        }
        let env = AABB::from_corners(aabb.min.to_array(), aabb.max.to_array());
        self.tree
            .locate_in_envelope_intersecting(&env)
            .map(|e| e.id)
            .collect()
    }

    /// Return candidate `ObjectId`s whose AABB is intersected by `ray`, sorted by entry distance.
    pub fn query_ray_candidates(&self, ray: &Ray) -> Vec<(ObjectId, f64)> {
        self.query_ray_candidates_expanded(ray, 0.0)
    }

    /// Return candidate `ObjectId`s whose expanded AABB is intersected by `ray`, sorted by entry distance.
    pub fn query_ray_candidates_expanded(&self, ray: &Ray, margin: f64) -> Vec<(ObjectId, f64)> {
        let mut hits: Vec<(ObjectId, f64)> = self
            .tree
            .iter()
            .filter_map(|entry| {
                ray.intersect_aabb(&entry.to_aabb().expanded(margin))
                    .map(|(t_near, _)| (entry.id, t_near))
            })
            .collect();
        hits.sort_by(|a, b| a.1.total_cmp(&b.1));
        hits
    }
}
