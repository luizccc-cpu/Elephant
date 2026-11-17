//! File format importers and exporters (.elephant, .obj, .stl) for Elephant.

pub mod error;
pub mod obj;
pub mod project;
pub mod stl;
pub mod threedom;

use std::path::Path;

use cad_doc::Document;
pub use error::IoError;

pub fn import_into_document(path: &Path, doc: &mut Document) -> Result<usize, IoError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    match ext.as_str() {
        "obj" => {
            let text = std::fs::read_to_string(path)?;
            obj::import_obj_str(&text, doc)
        }
        "stl" => {
            let bytes = std::fs::read(path)?;
            let name = path.file_stem().and_then(|s| s.to_str()).map(String::from);
            stl::import_stl_bytes(&bytes, doc, name)
        }
        "3dm" => {
            let bytes = std::fs::read(path)?;
            threedom::import_3dm_bytes(&bytes, doc)
        }
        other => Err(IoError::UnsupportedExtension(other.to_string())),
    }
}

pub fn export_from_document(path: &Path, doc: &Document) -> Result<(), IoError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    match ext.as_str() {
        "obj" => {
            let text = obj::export_obj_string(doc);
            std::fs::write(path, text)?;
            Ok(())
        }
        "stl" => {
            let bytes = stl::export_stl_bytes(doc);
            std::fs::write(path, bytes)?;
            Ok(())
        }
        "elephant" | "json" => {
            let text = project::save_project_json(doc)?;
            std::fs::write(path, text)?;
            Ok(())
        }
        other => Err(IoError::UnsupportedExtension(other.to_string())),
    }
}

pub fn open_project_file(path: &Path) -> Result<Document, IoError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if ext == "3dm" {
        let mut doc = Document::new();
        let bytes = std::fs::read(path)?;
        threedom::import_3dm_bytes(&bytes, &mut doc)?;
        return Ok(doc);
    }
    let text = std::fs::read_to_string(path)?;
    project::load_project_json(&text)
}
