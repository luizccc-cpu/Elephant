use cad_geom::{Aabb, Geometry};
use serde::{Deserialize, Serialize};
use slotmap::new_key_type;

use crate::layer::LayerId;

new_key_type! {
    pub struct ObjectId;
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CadObject {
    pub name: Option<String>,
    pub layer_id: LayerId,
    pub color: Option<[u8; 4]>,
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
            geometry,
            selected: false,
            aabb,
        }
    }

    pub fn refresh_aabb(&mut self) {
        self.aabb = self.geometry.bounding_box();
    }
}
