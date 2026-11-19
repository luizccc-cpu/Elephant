//! Document model, generational object store, layers, spatial index, and Undo/Redo for Elephant.

pub mod document;
pub mod history;
pub mod layer;
pub mod layout;
pub mod modeling_history;
pub mod object;
pub mod spatial;

pub use document::Document;
pub use layer::{Layer, LayerId};
pub use layout::{ClippingPlane, DetailView, LayoutSheet, PageSize, TitleBlock};
pub use modeling_history::{HistoryOp, HistoryRecord};
pub use object::{
    AnnotationItem, AnnotationKind, CadObject, GdtSpec, GdtSymbol, MaterialProps, ObjectId,
    ObjectStore,
};
