use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq)]
pub enum GeomError {
    #[error("invalid NURBS degree {0}: must be >= 1")]
    InvalidDegree(usize),
    #[error("insufficient control points ({found}) for degree {degree}: need at least {needed}")]
    InsufficientControlPoints {
        degree: usize,
        found: usize,
        needed: usize,
    },
    #[error("invalid knot vector length {found}: expected {expected}")]
    InvalidKnotCount { found: usize, expected: usize },
    #[error("knot vector must be non-decreasing")]
    NonMonotonicKnots,
    #[error("control point weight must be positive, found {0}")]
    NonPositiveWeight(f64),
    #[error("invalid grid dimensions {count_u}x{count_v} for {total} control points")]
    InvalidGridDimensions {
        count_u: usize,
        count_v: usize,
        total: usize,
    },
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    #[error("{0}")]
    InvalidInput(String),
}
