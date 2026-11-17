use cad_geom::{Aabb, Geometry};
use serde::{Deserialize, Serialize};
use slotmap::new_key_type;

use crate::layer::LayerId;

new_key_type! {
    pub struct ObjectId;
}

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
