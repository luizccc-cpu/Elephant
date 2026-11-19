//! wgpu 4-viewport renderer, CPlane grids, Wireframe/Shaded modes, and camera controls for Elephant.

pub mod camera;
pub mod scene;
pub mod wgpu_pass;

pub use camera::{ViewportCamera, ViewportKind};
pub use scene::{
    DisplayMode, LineVertex, MeshVertex, RenderScene, SurfaceAnalysisConfig, SurfaceAnalysisMode,
};
pub use wgpu_pass::{CadWgpuState, ViewportPaintCallback};
