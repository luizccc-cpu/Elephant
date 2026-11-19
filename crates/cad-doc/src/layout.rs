use cad_geom::{DVec3, DrawingProjection, Plane};
use serde::{Deserialize, Serialize};

fn default_true() -> bool {
    true
}

fn default_clip_size() -> f64 {
    24.0
}

fn default_margin() -> f64 {
    12.0
}

/// Live viewport section/clipping plane (Rhino ClippingPlane / SolidWorks Section View).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ClippingPlane {
    pub id: u64,
    pub name: String,
    pub plane: Plane,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub flip: bool,
    #[serde(default = "default_true")]
    pub fill_caps: bool,
    #[serde(default = "default_clip_size")]
    pub size: f64,
}

impl ClippingPlane {
    pub fn new(id: u64, name: impl Into<String>, plane: Plane) -> Self {
        Self {
            id,
            name: name.into(),
            plane,
            enabled: true,
            flip: false,
            fill_caps: true,
            size: 24.0,
        }
    }

    pub fn effective_normal(&self) -> DVec3 {
        let n = self.plane.z_axis.normalize_or(DVec3::Z);
        if self.flip { -n } else { n }
    }

    /// Clip plane half-space equation `(nx, ny, nz, d)` where fragments with `dot(p, n) + d < 0` are clipped.
    pub fn equation_f32(&self) -> [f32; 4] {
        let n = self.effective_normal();
        let d = -n.dot(self.plane.origin);
        [n.x as f32, n.y as f32, n.z as f32, d as f32]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum PageSize {
    A4,
    A3,
    A2,
    A1,
    Letter,
    Tabloid,
}

impl Default for PageSize {
    fn default() -> Self {
        Self::A3
    }
}

impl PageSize {
    pub const ALL: [Self; 6] = [
        Self::A4,
        Self::A3,
        Self::A2,
        Self::A1,
        Self::Letter,
        Self::Tabloid,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::A4 => "A4 (297x210)",
            Self::A3 => "A3 (420x297)",
            Self::A2 => "A2 (594x420)",
            Self::A1 => "A1 (841x594)",
            Self::Letter => "Letter (11x8.5\")",
            Self::Tabloid => "Tabloid (17x11\")",
        }
    }

    pub fn dimensions_mm(self, landscape: bool) -> (f64, f64) {
        let (short, long) = match self {
            Self::A4 => (210.0, 297.0),
            Self::A3 => (297.0, 420.0),
            Self::A2 => (420.0, 594.0),
            Self::A1 => (594.0, 841.0),
            Self::Letter => (215.9, 279.4),
            Self::Tabloid => (279.4, 431.8),
        };
        if landscape { (long, short) } else { (short, long) }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TitleBlock {
    pub title: String,
    pub drawing_number: String,
    pub author: String,
    pub scale_text: String,
    pub revision: String,
    pub date: String,
    pub notes: String,
}

impl Default for TitleBlock {
    fn default() -> Self {
        Self {
            title: "ELEPHANT CAD ASSEMBLY".to_string(),
            drawing_number: "DWG-001".to_string(),
            author: "Designer".to_string(),
            scale_text: "AUTO".to_string(),
            revision: "A".to_string(),
            date: "2026-11".to_string(),
            notes: "ALL DIMENSIONS IN MODEL UNITS UNLESS NOTED".to_string(),
        }
    }
}

/// Paper-space detail viewport displaying a projected 2D drawing inside `rect_mm = [x, y, w, h]`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DetailView {
    pub id: u64,
    pub title: String,
    pub projection: DrawingProjection,
    pub rect_mm: [f64; 4],
    /// Drawing scale factor (`0.0` = auto-fit inside detail frame).
    pub scale: f64,
    #[serde(default = "default_true")]
    pub show_hidden_lines: bool,
    #[serde(default = "default_true")]
    pub show_dimensions: bool,
}

/// Technical Paper-Space layout sheet containing a title block and detail views.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LayoutSheet {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub page_size: PageSize,
    #[serde(default = "default_true")]
    pub landscape: bool,
    #[serde(default = "default_margin")]
    pub margin_mm: f64,
    #[serde(default)]
    pub title_block: TitleBlock,
    #[serde(default)]
    pub details: Vec<DetailView>,
}

impl LayoutSheet {
    pub fn standard_4_view(id: u64, name: impl Into<String>, page_size: PageSize) -> Self {
        let landscape = true;
        let margin_mm = 12.0;
        let (pw, ph) = page_size.dimensions_mm(landscape);
        let tb_h = 28.0;
        let gap = 8.0;
        let x0 = margin_mm + 4.0;
        let y0 = margin_mm + 4.0;
        let usable_w = (pw - 2.0 * (margin_mm + 4.0) - gap).max(40.0);
        let usable_h = (ph - 2.0 * (margin_mm + 4.0) - tb_h - gap - 4.0).max(40.0);
        let cell_w = usable_w * 0.5;
        let cell_h = usable_h * 0.5;

        let details = vec![
            DetailView {
                id: 1,
                title: "TOP VIEW".to_string(),
                projection: DrawingProjection::Top,
                rect_mm: [x0, y0, cell_w, cell_h],
                scale: 0.0,
                show_hidden_lines: true,
                show_dimensions: true,
            },
            DetailView {
                id: 2,
                title: "ISOMETRIC VIEW".to_string(),
                projection: DrawingProjection::Isometric,
                rect_mm: [x0 + cell_w + gap, y0, cell_w, cell_h],
                scale: 0.0,
                show_hidden_lines: false,
                show_dimensions: false,
            },
            DetailView {
                id: 3,
                title: "FRONT VIEW".to_string(),
                projection: DrawingProjection::Front,
                rect_mm: [x0, y0 + cell_h + gap, cell_w, cell_h],
                scale: 0.0,
                show_hidden_lines: true,
                show_dimensions: true,
            },
            DetailView {
                id: 4,
                title: "RIGHT VIEW".to_string(),
                projection: DrawingProjection::Right,
                rect_mm: [x0 + cell_w + gap, y0 + cell_h + gap, cell_w, cell_h],
                scale: 0.0,
                show_hidden_lines: true,
                show_dimensions: true,
            },
        ];

        Self {
            id,
            name: name.into(),
            page_size,
            landscape,
            margin_mm,
            title_block: TitleBlock::default(),
            details,
        }
    }
}
