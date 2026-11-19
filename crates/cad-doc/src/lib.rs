//! Document model, generational object store, layers, spatial index, and Undo/Redo for Elephant.

pub mod document;
pub mod history;
pub mod layer;
pub mod modeling_history;
pub mod object;
pub mod spatial;

pub use document::Document;
pub use layer::{Layer, LayerId};
pub use modeling_history::{HistoryOp, HistoryRecord};
pub use object::{AnnotationItem, AnnotationKind, CadObject, ObjectId, ObjectStore};
