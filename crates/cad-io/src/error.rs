use thiserror::Error;

#[derive(Debug, Error)]
pub enum IoError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("serialization error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("invalid file format: {0}")]
    InvalidFormat(String),
    #[error("unsupported file extension: {0}")]
    UnsupportedExtension(String),
}
