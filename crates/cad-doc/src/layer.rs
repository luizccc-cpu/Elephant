use serde::{Deserialize, Serialize};
use slotmap::new_key_type;

new_key_type! {
    pub struct LayerId;
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Layer {
    pub name: String,
    pub color: [u8; 4],
    pub visible: bool,
    pub locked: bool,
    #[serde(default)]
    pub parent: Option<LayerId>,
}

impl Layer {
    pub fn new(name: impl Into<String>, color: [u8; 4]) -> Self {
        Self {
            name: name.into(),
            color,
            visible: true,
            locked: false,
            parent: None,
        }
    }

    pub fn new_child(name: impl Into<String>, color: [u8; 4], parent: LayerId) -> Self {
        Self {
            name: name.into(),
            color,
            visible: true,
            locked: false,
            parent: Some(parent),
        }
    }
}
