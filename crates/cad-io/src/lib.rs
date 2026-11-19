//! File format importers and exporters (.elephant, .3dm, .step, .iges, .3mf, .dxf, .obj, .stl, .svg, .pdf, .glb) for Elephant.

pub mod dxf;
pub mod error;
pub mod gltf;
pub mod iges;
pub mod obj;
pub mod pdf;
pub mod project;
pub mod step;
pub mod stl;
pub mod svg;
pub mod threedom;
pub mod threemf;

use std::path::Path;

use cad_doc::Document;
pub use error::IoError;
pub use iges::{export_iges_string, import_iges_str};
pub use pdf::{export_layout_pdf_bytes, export_pdf_bytes};
pub use project::ProjectMetadata;
pub use step::{export_step_string, import_step_str};
pub use svg::export_layout_svg_string;
pub use threedom::{export_3dm_bytes, import_3dm_bytes};
pub use threemf::{export_3mf_bytes, import_3mf_bytes};

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
        "step" | "stp" => {
            let text = std::fs::read_to_string(path)?;
            step::import_step_str(&text, doc)
        }
        "iges" | "igs" => {
            let text = std::fs::read_to_string(path)?;
            iges::import_iges_str(&text, doc)
        }
        "3mf" => {
            let bytes = std::fs::read(path)?;
            threemf::import_3mf_bytes(&bytes, doc)
        }
        "dxf" => {
            let text = std::fs::read_to_string(path)?;
            dxf::import_dxf_str(&text, doc)
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
        "dxf" => {
            let text = dxf::export_dxf_string(doc);
            std::fs::write(path, text)?;
            Ok(())
        }
        "svg" => {
            let text = svg::export_svg_string(doc);
            std::fs::write(path, text)?;
            Ok(())
        }
        "pdf" => {
            let bytes = pdf::export_pdf_bytes(doc);
            std::fs::write(path, bytes)?;
            Ok(())
        }
        "glb" => {
            let bytes = gltf::export_glb_bytes(doc);
            std::fs::write(path, bytes)?;
            Ok(())
        }
        "step" | "stp" => {
            let text = step::export_step_string(doc);
            std::fs::write(path, text)?;
            Ok(())
        }
        "iges" | "igs" => {
            let text = iges::export_iges_string(doc);
            std::fs::write(path, text)?;
            Ok(())
        }
        "3mf" => {
            let bytes = threemf::export_3mf_bytes(doc);
            std::fs::write(path, bytes)?;
            Ok(())
        }
        "3dm" => {
            let bytes = threedom::export_3dm_bytes(doc)?;
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
    open_project_with_meta(path).map(|(doc, _)| doc)
}

pub fn open_project_with_meta(path: &Path) -> Result<(Document, ProjectMetadata), IoError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if ext == "3dm" {
        let mut doc = Document::new();
        let bytes = std::fs::read(path)?;
        threedom::import_3dm_bytes(&bytes, &mut doc)?;
        doc.clear_history();
        return Ok((doc, ProjectMetadata::default()));
    }
    let text = std::fs::read_to_string(path)?;
    project::load_project_with_meta(&text)
}

pub fn save_project_file_with_meta(
    path: &Path,
    doc: &Document,
    meta: &ProjectMetadata,
) -> Result<(), IoError> {
    let text = project::save_project_with_meta(doc, meta)?;
    std::fs::write(path, text)?;
    Ok(())
}
