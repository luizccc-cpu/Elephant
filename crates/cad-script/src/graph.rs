use std::collections::{HashMap, HashSet};

use cad_doc::Document;
use cad_geom::{
    BRep, BooleanOp, DMat4, DVec3, Geometry, LoftStyle, NurbsCurve, NurbsSurface, Plane,
    ShrinkWrapOptions, SubDMesh, TriangleMesh, brep_boolean, loft_nurbs, mesh_boolean,
    revolve_nurbs, shrinkwrap_geometries, subd_to_nurbs_brep,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PortType {
    Number,
    Point,
    Geometry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PortSpec {
    pub name: &'static str,
    pub ty: PortType,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MathOpKind {
    Add,
    Sub,
    Mul,
    Div,
    Sin,
    Cos,
    Pow,
}

impl MathOpKind {
    pub const ALL: [Self; 7] = [
        Self::Add,
        Self::Sub,
        Self::Mul,
        Self::Div,
        Self::Sin,
        Self::Cos,
        Self::Pow,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Add => "A + B",
            Self::Sub => "A - B",
            Self::Mul => "A * B",
            Self::Div => "A / B",
            Self::Sin => "sin(A)",
            Self::Cos => "cos(A)",
            Self::Pow => "pow(A,B)",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NodeBooleanKind {
    Union,
    Difference,
    Intersection,
}

impl NodeBooleanKind {
    pub const ALL: [Self; 3] = [Self::Union, Self::Difference, Self::Intersection];

    pub fn label(self) -> &'static str {
        match self {
            Self::Union => "Union",
            Self::Difference => "Difference",
            Self::Intersection => "Intersection",
        }
    }

    pub fn to_op(self) -> BooleanOp {
        match self {
            Self::Union => BooleanOp::Union,
            Self::Difference => BooleanOp::Difference,
            Self::Intersection => BooleanOp::Intersection,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeKind {
    NumberSlider {
        value: f64,
        min: f64,
        max: f64,
    },
    Point {
        x: f64,
        y: f64,
        z: f64,
    },
    MathOp {
        op: MathOpKind,
        a: f64,
        b: f64,
    },
    Circle {
        radius: f64,
    },
    Rectangle {
        width: f64,
        height: f64,
    },
    Polygon {
        radius: f64,
        sides: usize,
    },
    Extrude {
        height: f64,
    },
    Loft,
    Revolve {
        angle_deg: f64,
    },
    Box {
        sx: f64,
        sy: f64,
        sz: f64,
    },
    Sphere {
        radius: f64,
    },
    SubDBox {
        sx: f64,
        sy: f64,
        sz: f64,
        level: u8,
    },
    SubDSphere {
        radius: f64,
        level: u8,
    },
    ToNurbs,
    BooleanOp {
        op: NodeBooleanKind,
    },
    ShrinkWrap {
        voxel_size: f64,
        offset: f64,
        smooth: usize,
    },
    Move {
        dx: f64,
        dy: f64,
        dz: f64,
    },
    Rotate {
        angle_deg: f64,
    },
    LinearArray {
        count: usize,
        step_x: f64,
        step_y: f64,
        step_z: f64,
    },
    PolarArray {
        count: usize,
        radius: f64,
    },
    Output,
}

impl NodeKind {
    pub fn default_label(&self) -> &'static str {
        match self {
            Self::NumberSlider { .. } => "Number Slider",
            Self::Point { .. } => "Point",
            Self::MathOp { .. } => "Math",
            Self::Circle { .. } => "Circle",
            Self::Rectangle { .. } => "Rectangle",
            Self::Polygon { .. } => "Polygon",
            Self::Extrude { .. } => "Extrude",
            Self::Loft => "Loft",
            Self::Revolve { .. } => "Revolve",
            Self::Box { .. } => "BRep Box",
            Self::Sphere { .. } => "NURBS Sphere",
            Self::SubDBox { .. } => "SubD Box",
            Self::SubDSphere { .. } => "SubD Sphere",
            Self::ToNurbs => "ToNURBS",
            Self::BooleanOp { .. } => "Boolean",
            Self::ShrinkWrap { .. } => "ShrinkWrap",
            Self::Move { .. } => "Move",
            Self::Rotate { .. } => "Rotate Z",
            Self::LinearArray { .. } => "Linear Array",
            Self::PolarArray { .. } => "Polar Array",
            Self::Output => "Output",
        }
    }

    pub fn category(&self) -> &'static str {
        match self {
            Self::NumberSlider { .. } | Self::Point { .. } | Self::MathOp { .. } => "Input/Math",
            Self::Circle { .. } | Self::Rectangle { .. } | Self::Polygon { .. } => "Curve",
            Self::Extrude { .. }
            | Self::Loft
            | Self::Revolve { .. }
            | Self::Box { .. }
            | Self::Sphere { .. } => "Surface/Solid",
            Self::SubDBox { .. }
            | Self::SubDSphere { .. }
            | Self::ToNurbs
            | Self::BooleanOp { .. }
            | Self::ShrinkWrap { .. } => "SubD/Mesh",
            Self::Move { .. }
            | Self::Rotate { .. }
            | Self::LinearArray { .. }
            | Self::PolarArray { .. } => "Transform",
            Self::Output => "Output",
        }
    }

    pub fn input_ports(&self) -> &'static [PortSpec] {
        use PortType::{Geometry as G, Number as N, Point as P};
        match self {
            Self::NumberSlider { .. } => &[],
            Self::Point { .. } => &[
                PortSpec { name: "x", ty: N },
                PortSpec { name: "y", ty: N },
                PortSpec { name: "z", ty: N },
            ],
            Self::MathOp { .. } => &[
                PortSpec { name: "a", ty: N },
                PortSpec { name: "b", ty: N },
            ],
            Self::Circle { .. } => &[
                PortSpec { name: "radius", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::Rectangle { .. } => &[
                PortSpec { name: "width", ty: N },
                PortSpec { name: "height", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::Polygon { .. } => &[
                PortSpec { name: "radius", ty: N },
                PortSpec { name: "sides", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::Extrude { .. } => &[
                PortSpec { name: "curve", ty: G },
                PortSpec { name: "height", ty: N },
            ],
            Self::Loft => &[
                PortSpec { name: "curve_a", ty: G },
                PortSpec { name: "curve_b", ty: G },
            ],
            Self::Revolve { .. } => &[
                PortSpec { name: "curve", ty: G },
                PortSpec { name: "angle_deg", ty: N },
            ],
            Self::Box { .. } => &[
                PortSpec { name: "size", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::Sphere { .. } | Self::SubDSphere { .. } => &[
                PortSpec { name: "radius", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::SubDBox { .. } => &[
                PortSpec { name: "size", ty: N },
                PortSpec { name: "center", ty: P },
            ],
            Self::ToNurbs => &[PortSpec { name: "subd", ty: G }],
            Self::BooleanOp { .. } => &[
                PortSpec { name: "a", ty: G },
                PortSpec { name: "b", ty: G },
            ],
            Self::ShrinkWrap { .. } => &[
                PortSpec { name: "geom", ty: G },
                PortSpec { name: "offset", ty: N },
            ],
            Self::Move { .. } => &[
                PortSpec { name: "geom", ty: G },
                PortSpec { name: "dx", ty: N },
                PortSpec { name: "dy", ty: N },
                PortSpec { name: "dz", ty: N },
            ],
            Self::Rotate { .. } => &[
                PortSpec { name: "geom", ty: G },
                PortSpec { name: "angle_deg", ty: N },
            ],
            Self::LinearArray { .. } => &[
                PortSpec { name: "geom", ty: G },
                PortSpec { name: "count", ty: N },
                PortSpec { name: "step_x", ty: N },
            ],
            Self::PolarArray { .. } => &[
                PortSpec { name: "geom", ty: G },
                PortSpec { name: "count", ty: N },
                PortSpec { name: "radius", ty: N },
            ],
            Self::Output => &[PortSpec { name: "geom", ty: G }],
        }
    }

    pub fn output_port(&self) -> Option<PortSpec> {
        match self {
            Self::NumberSlider { .. } | Self::MathOp { .. } => Some(PortSpec {
                name: "val",
                ty: PortType::Number,
            }),
            Self::Point { .. } => Some(PortSpec {
                name: "pt",
                ty: PortType::Point,
            }),
            Self::Output => None,
            _ => Some(PortSpec {
                name: "geom",
                ty: PortType::Geometry,
            }),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParametricNode {
    pub id: usize,
    pub label: String,
    #[serde(default)]
    pub pos: [f32; 2],
    pub kind: NodeKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeLink {
    pub from_node: usize,
    pub to_node: usize,
    pub to_port: String,
}

#[derive(Debug, Clone)]
pub enum NodeValue {
    Number(f64),
    Point(DVec3),
    Geometries(Vec<Geometry>),
}

/// Grasshopper-style visual 2D parametric node graph.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParametricGraph {
    pub nodes: Vec<ParametricNode>,
    pub links: Vec<NodeLink>,
    pub live_preview: bool,
}

impl Default for ParametricGraph {
    fn default() -> Self {
        Self {
            live_preview: false,
            nodes: vec![
                ParametricNode {
                    id: 1,
                    label: "Radius".to_string(),
                    pos: [40.0, 50.0],
                    kind: NodeKind::NumberSlider {
                        value: 1.5,
                        min: 0.2,
                        max: 8.0,
                    },
                },
                ParametricNode {
                    id: 2,
                    label: "Height".to_string(),
                    pos: [40.0, 175.0],
                    kind: NodeKind::NumberSlider {
                        value: 3.0,
                        min: 0.2,
                        max: 12.0,
                    },
                },
                ParametricNode {
                    id: 3,
                    label: "Count".to_string(),
                    pos: [40.0, 300.0],
                    kind: NodeKind::NumberSlider {
                        value: 4.0,
                        min: 1.0,
                        max: 12.0,
                    },
                },
                ParametricNode {
                    id: 4,
                    label: "Circle".to_string(),
                    pos: [290.0, 50.0],
                    kind: NodeKind::Circle { radius: 1.5 },
                },
                ParametricNode {
                    id: 5,
                    label: "Extrude".to_string(),
                    pos: [520.0, 120.0],
                    kind: NodeKind::Extrude { height: 3.0 },
                },
                ParametricNode {
                    id: 6,
                    label: "Linear Array".to_string(),
                    pos: [750.0, 200.0],
                    kind: NodeKind::LinearArray {
                        count: 4,
                        step_x: 4.0,
                        step_y: 0.0,
                        step_z: 0.0,
                    },
                },
                ParametricNode {
                    id: 7,
                    label: "Output".to_string(),
                    pos: [1000.0, 200.0],
                    kind: NodeKind::Output,
                },
            ],
            links: vec![
                NodeLink {
                    from_node: 1,
                    to_node: 4,
                    to_port: "radius".to_string(),
                },
                NodeLink {
                    from_node: 4,
                    to_node: 5,
                    to_port: "curve".to_string(),
                },
                NodeLink {
                    from_node: 2,
                    to_node: 5,
                    to_port: "height".to_string(),
                },
                NodeLink {
                    from_node: 5,
                    to_node: 6,
                    to_port: "geom".to_string(),
                },
                NodeLink {
                    from_node: 3,
                    to_node: 6,
                    to_port: "count".to_string(),
                },
                NodeLink {
                    from_node: 6,
                    to_node: 7,
                    to_port: "geom".to_string(),
                },
            ],
        }
    }
}

impl ParametricGraph {
    pub fn next_node_id(&self) -> usize {
        self.nodes.iter().map(|n| n.id).max().unwrap_or(0) + 1
    }

    pub fn add_node(&mut self, kind: NodeKind, pos: [f32; 2]) -> usize {
        let id = self.next_node_id();
        let label = kind.default_label().to_string();
        self.nodes.push(ParametricNode {
            id,
            label,
            pos,
            kind,
        });
        id
    }

    pub fn remove_node(&mut self, id: usize) {
        self.nodes.retain(|n| n.id != id);
        self.links.retain(|l| l.from_node != id && l.to_node != id);
    }

    pub fn would_create_cycle(&self, from_node: usize, to_node: usize) -> bool {
        if from_node == to_node {
            return true;
        }
        // Check if `from_node` is reachable upstream from `to_node`? No: if `to_node` can reach `from_node` via existing links.
        let mut visited = HashSet::new();
        let mut stack = vec![to_node];
        while let Some(cur) = stack.pop() {
            if cur == from_node {
                return true;
            }
            if !visited.insert(cur) {
                continue;
            }
            for link in &self.links {
                if link.from_node == cur {
                    stack.push(link.to_node);
                }
            }
        }
        false
    }

    pub fn connect(&mut self, from_node: usize, to_node: usize, to_port: impl Into<String>) -> bool {
        let to_port = to_port.into();
        if self.would_create_cycle(from_node, to_node) {
            return false;
        }
        let Some(src) = self.nodes.iter().find(|n| n.id == from_node) else {
            return false;
        };
        let Some(dst) = self.nodes.iter().find(|n| n.id == to_node) else {
            return false;
        };
        let Some(out_port) = src.kind.output_port() else {
            return false;
        };
        let Some(in_port) = dst.kind.input_ports().iter().find(|p| p.name == to_port) else {
            return false;
        };
        if out_port.ty != in_port.ty {
            return false;
        }
        self.links
            .retain(|l| !(l.to_node == to_node && l.to_port == to_port));
        self.links.push(NodeLink {
            from_node,
            to_node,
            to_port,
        });
        true
    }

    pub fn disconnect(&mut self, to_node: usize, to_port: &str) {
        self.links
            .retain(|l| !(l.to_node == to_node && l.to_port == to_port));
    }

    pub fn ensure_valid_positions(&mut self) {
        if self.nodes.is_empty() {
            return;
        }
        let all_origin = self
            .nodes
            .iter()
            .all(|n| n.pos[0].abs() < 1e-3 && n.pos[1].abs() < 1e-3);
        if all_origin {
            for (idx, n) in self.nodes.iter_mut().enumerate() {
                let col = idx / 3;
                let row = idx % 3;
                n.pos = [40.0 + col as f32 * 240.0, 50.0 + row as f32 * 140.0];
            }
        }
    }

    fn eval_node(
        &self,
        id: usize,
        depth: usize,
        memo: &mut HashMap<usize, Option<NodeValue>>,
    ) -> Option<NodeValue> {
        if let Some(cached) = memo.get(&id) {
            return cached.clone();
        }
        if depth > 48 {
            return None;
        }
        let node = self.nodes.iter().find(|n| n.id == id)?;
        let links_for_node: Vec<&NodeLink> = self.links.iter().filter(|l| l.to_node == id).collect();
        let mut get_in = |port: &str| -> Option<NodeValue> {
            let link = links_for_node.iter().find(|l| l.to_port == port)?;
            self.eval_node(link.from_node, depth + 1, memo)
        };

        let val = match &node.kind {
            NodeKind::NumberSlider { value, .. } => Some(NodeValue::Number(*value)),
            NodeKind::Point { x, y, z } => {
                let vx = match get_in("x") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *x,
                };
                let vy = match get_in("y") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *y,
                };
                let vz = match get_in("z") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *z,
                };
                Some(NodeValue::Point(DVec3::new(vx, vy, vz)))
            }
            NodeKind::MathOp { op, a, b } => {
                let va = match get_in("a") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *a,
                };
                let vb = match get_in("b") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *b,
                };
                let out = match op {
                    MathOpKind::Add => va + vb,
                    MathOpKind::Sub => va - vb,
                    MathOpKind::Mul => va * vb,
                    MathOpKind::Div => {
                        if vb.abs() > 1e-12 {
                            va / vb
                        } else {
                            0.0
                        }
                    }
                    MathOpKind::Sin => va.sin(),
                    MathOpKind::Cos => va.cos(),
                    MathOpKind::Pow => va.powf(vb),
                };
                Some(NodeValue::Number(out))
            }
            NodeKind::Circle { radius } => {
                let r = match get_in("radius") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *radius,
                };
                let origin = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                let mut plane = Plane::world_xy();
                plane.origin = origin;
                NurbsCurve::circle(&plane, r)
                    .ok()
                    .map(|c| NodeValue::Geometries(vec![Geometry::Curve(c)]))
            }
            NodeKind::Rectangle { width, height } => {
                let w = match get_in("width") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *width,
                };
                let h = match get_in("height") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *height,
                };
                let origin = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                let mut plane = Plane::world_xy();
                plane.origin = origin;
                let half = DVec3::new(w * 0.5, h * 0.5, 0.0);
                NurbsCurve::rectangle(&plane, origin - half, origin + half)
                    .ok()
                    .map(|c| NodeValue::Geometries(vec![Geometry::Curve(c)]))
            }
            NodeKind::Polygon { radius, sides } => {
                let r = match get_in("radius") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *radius,
                };
                let n = match get_in("sides") {
                    Some(NodeValue::Number(v)) => (v.round() as isize).clamp(3, 32) as usize,
                    _ => (*sides).clamp(3, 32),
                };
                let origin = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                let mut plane = Plane::world_xy();
                plane.origin = origin;
                NurbsCurve::polygon(&plane, origin, origin + DVec3::X * r, n)
                    .ok()
                    .map(|c| NodeValue::Geometries(vec![Geometry::Curve(c)]))
            }
            NodeKind::Extrude { height } => {
                let h = match get_in("height") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *height,
                };
                let geoms = match get_in("curve") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let out = geoms
                    .into_iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(Geometry::Surface(NurbsSurface::extrude_curve(
                            &c,
                            DVec3::new(0.0, 0.0, h),
                        ))),
                        _ => None,
                    })
                    .collect();
                Some(NodeValue::Geometries(out))
            }
            NodeKind::Loft => {
                let ga = match get_in("curve_a") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let gb = match get_in("curve_b") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let mut curves: Vec<NurbsCurve> = ga
                    .into_iter()
                    .chain(gb)
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(c),
                        _ => None,
                    })
                    .collect();
                if curves.len() >= 2 {
                    loft_nurbs(&curves.drain(..2).collect::<Vec<_>>(), LoftStyle::Normal, false)
                        .ok()
                        .map(|s| NodeValue::Geometries(vec![Geometry::Surface(s)]))
                } else {
                    Some(NodeValue::Geometries(Vec::new()))
                }
            }
            NodeKind::Revolve { angle_deg } => {
                let ang = match get_in("angle_deg") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *angle_deg,
                };
                let geoms = match get_in("curve") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let out = geoms
                    .into_iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => revolve_nurbs(
                            &c,
                            DVec3::ZERO,
                            DVec3::Z,
                            ang.to_radians(),
                        )
                        .ok()
                        .map(Geometry::Surface),
                        _ => None,
                    })
                    .collect();
                Some(NodeValue::Geometries(out))
            }
            NodeKind::Box { sx, sy, sz } => {
                let size = match get_in("size") {
                    Some(NodeValue::Number(v)) => DVec3::splat(v.max(0.05)),
                    _ => DVec3::new(*sx, *sy, *sz),
                };
                let center = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::new(0.0, 0.0, size.z * 0.5),
                };
                Some(NodeValue::Geometries(vec![Geometry::BRep(BRep::from_box(
                    center, size,
                ))]))
            }
            NodeKind::Sphere { radius } => {
                let r = match get_in("radius") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *radius,
                };
                let center = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                NurbsSurface::sphere(center, r)
                    .ok()
                    .map(|s| NodeValue::Geometries(vec![Geometry::Surface(s)]))
            }
            NodeKind::SubDBox { sx, sy, sz, level } => {
                let size = match get_in("size") {
                    Some(NodeValue::Number(v)) => DVec3::splat(v.max(0.05)),
                    _ => DVec3::new(*sx, *sy, *sz),
                };
                let center = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::new(0.0, 0.0, size.z * 0.5),
                };
                let mut sd = SubDMesh::from_box(center, size);
                sd.subdivision_level = (*level).clamp(1, 4);
                Some(NodeValue::Geometries(vec![Geometry::SubD(sd)]))
            }
            NodeKind::SubDSphere { radius, level } => {
                let r = match get_in("radius") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *radius,
                };
                let center = match get_in("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                let mut sd = SubDMesh::from_sphere(center, r);
                sd.subdivision_level = (*level).clamp(1, 4);
                Some(NodeValue::Geometries(vec![Geometry::SubD(sd)]))
            }
            NodeKind::ToNurbs => {
                let geoms = match get_in("subd") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let out = geoms
                    .into_iter()
                    .filter_map(|g| match g {
                        Geometry::SubD(sd) => subd_to_nurbs_brep(&sd).ok().map(Geometry::BRep),
                        other => Some(other),
                    })
                    .collect();
                Some(NodeValue::Geometries(out))
            }
            NodeKind::BooleanOp { op } => {
                let ga = match get_in("a") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let gb = match get_in("b") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                if let (Some(a), Some(b)) = (ga.first(), gb.first()) {
                    let bop = op.to_op();
                    let res = match (a, b) {
                        (Geometry::BRep(ba), Geometry::BRep(bb)) => {
                            brep_boolean(ba, bb, bop, 1e-4).ok().map(Geometry::BRep)
                        }
                        _ => {
                            let ma = geom_to_mesh(a);
                            let mb = geom_to_mesh(b);
                            if let (Some(ma), Some(mb)) = (ma, mb) {
                                Some(Geometry::Mesh(mesh_boolean(&ma, &mb, bop)))
                            } else {
                                None
                            }
                        }
                    };
                    Some(NodeValue::Geometries(res.into_iter().collect()))
                } else {
                    Some(NodeValue::Geometries(Vec::new()))
                }
            }
            NodeKind::ShrinkWrap {
                voxel_size,
                offset,
                smooth,
            } => {
                let off = match get_in("offset") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *offset,
                };
                let geoms = match get_in("geom") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let refs: Vec<&Geometry> = geoms.iter().collect();
                if refs.is_empty() {
                    Some(NodeValue::Geometries(Vec::new()))
                } else {
                    let wrapped = shrinkwrap_geometries(
                        &refs,
                        &ShrinkWrapOptions {
                            voxel_size: voxel_size.max(0.05),
                            offset: off,
                            smoothing_iterations: (*smooth).min(12),
                            fill_holes: true,
                            max_grid_dim: 44,
                        },
                    )
                    .ok()
                    .map(Geometry::Mesh);
                    Some(NodeValue::Geometries(wrapped.into_iter().collect()))
                }
            }
            NodeKind::Move { dx, dy, dz } => {
                let x = match get_in("dx") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *dx,
                };
                let y = match get_in("dy") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *dy,
                };
                let z = match get_in("dz") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *dz,
                };
                let mut geoms = match get_in("geom") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let mat = DMat4::from_translation(DVec3::new(x, y, z));
                for g in &mut geoms {
                    g.transform(&mat);
                }
                Some(NodeValue::Geometries(geoms))
            }
            NodeKind::Rotate { angle_deg } => {
                let ang = match get_in("angle_deg") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *angle_deg,
                };
                let mut geoms = match get_in("geom") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let mat = DMat4::from_rotation_z(ang.to_radians());
                for g in &mut geoms {
                    g.transform(&mat);
                }
                Some(NodeValue::Geometries(geoms))
            }
            NodeKind::LinearArray {
                count,
                step_x,
                step_y,
                step_z,
            } => {
                let n = match get_in("count") {
                    Some(NodeValue::Number(v)) => (v.round() as isize).clamp(1, 64) as usize,
                    _ => (*count).clamp(1, 64),
                };
                let sx = match get_in("step_x") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *step_x,
                };
                let geoms = match get_in("geom") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let step = DVec3::new(sx, *step_y, *step_z);
                let mut out = Vec::with_capacity(geoms.len() * n);
                for i in 0..n {
                    let mat = DMat4::from_translation(step * (i as f64));
                    for g in &geoms {
                        let mut cloned = g.clone();
                        cloned.transform(&mat);
                        out.push(cloned);
                    }
                }
                Some(NodeValue::Geometries(out))
            }
            NodeKind::PolarArray { count, radius } => {
                let n = match get_in("count") {
                    Some(NodeValue::Number(v)) => (v.round() as isize).clamp(1, 64) as usize,
                    _ => (*count).clamp(1, 64),
                };
                let rad = match get_in("radius") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *radius,
                };
                let geoms = match get_in("geom") {
                    Some(NodeValue::Geometries(g)) => g,
                    _ => Vec::new(),
                };
                let mut out = Vec::with_capacity(geoms.len() * n);
                for i in 0..n {
                    let ang = std::f64::consts::TAU * (i as f64 / n as f64);
                    let mat = DMat4::from_rotation_z(ang)
                        * DMat4::from_translation(DVec3::new(rad, 0.0, 0.0));
                    for g in &geoms {
                        let mut cloned = g.clone();
                        cloned.transform(&mat);
                        out.push(cloned);
                    }
                }
                Some(NodeValue::Geometries(out))
            }
            NodeKind::Output => get_in("geom"),
        };

        memo.insert(id, val.clone());
        val
    }

    /// Evaluate Output nodes (or terminal geometry-producing nodes if none exist) into preview/bakeable `Geometry`.
    pub fn evaluate(&self) -> Vec<Geometry> {
        let mut memo = HashMap::new();
        let output_ids: Vec<usize> = self
            .nodes
            .iter()
            .filter(|n| matches!(n.kind, NodeKind::Output))
            .map(|n| n.id)
            .collect();

        if !output_ids.is_empty() {
            let mut out = Vec::new();
            for oid in output_ids {
                if let Some(NodeValue::Geometries(gs)) = self.eval_node(oid, 0, &mut memo) {
                    out.extend(gs);
                }
            }
            return out;
        }

        let has_outgoing: HashSet<usize> = self.links.iter().map(|l| l.from_node).collect();
        let mut out = Vec::new();
        for n in &self.nodes {
            if !has_outgoing.contains(&n.id) {
                if let Some(NodeValue::Geometries(gs)) = self.eval_node(n.id, 0, &mut memo) {
                    out.extend(gs);
                }
            }
        }
        if !out.is_empty() {
            return out;
        }
        if let Some(last) = self.nodes.last() {
            if let Some(NodeValue::Geometries(gs)) = self.eval_node(last.id, 0, &mut memo) {
                return gs;
            }
        }
        Vec::new()
    }

    /// Bake the evaluated parametric geometry into `doc` as real Undo-able objects.
    pub fn bake_into(&self, doc: &mut Document) -> usize {
        let geoms = self.evaluate();
        let count = geoms.len();
        for g in geoms {
            doc.add_object("BakeParametric", g);
        }
        count
    }
}

fn geom_to_mesh(g: &Geometry) -> Option<TriangleMesh> {
    match g {
        Geometry::Mesh(m) => Some(m.clone()),
        Geometry::Surface(s) => Some(s.tessellate_display()),
        Geometry::BRep(b) => Some(b.tessellate_display()),
        Geometry::SubD(sd) => Some(sd.tessellate_display()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_graph_evaluates_and_bakes_array() {
        let graph = ParametricGraph::default();
        let geoms = graph.evaluate();
        assert_eq!(geoms.len(), 4);
        assert!(geoms.iter().all(|g| matches!(g, Geometry::Surface(_))));

        let mut doc = Document::new();
        assert_eq!(graph.bake_into(&mut doc), 4);
        assert_eq!(doc.objects.len(), 4);
    }

    #[test]
    fn node_graph_evaluates_math_subd_array_and_rejects_cycles() {
        let mut graph = ParametricGraph {
            nodes: Vec::new(),
            links: Vec::new(),
            live_preview: false,
        };
        let n_num = graph.add_node(
            NodeKind::NumberSlider {
                value: 2.0,
                min: 0.5,
                max: 5.0,
            },
            [20.0, 20.0],
        );
        let n_math = graph.add_node(
            NodeKind::MathOp {
                op: MathOpKind::Mul,
                a: 1.0,
                b: 1.5,
            },
            [180.0, 20.0],
        );
        let n_subd = graph.add_node(
            NodeKind::SubDBox {
                sx: 2.0,
                sy: 2.0,
                sz: 2.0,
                level: 1,
            },
            [340.0, 20.0],
        );
        let n_brep = graph.add_node(NodeKind::ToNurbs, [520.0, 20.0]);
        let n_polar = graph.add_node(
            NodeKind::PolarArray {
                count: 3,
                radius: 4.0,
            },
            [700.0, 20.0],
        );
        let n_out = graph.add_node(NodeKind::Output, [880.0, 20.0]);

        assert!(graph.connect(n_num, n_math, "a"));
        assert!(graph.connect(n_math, n_subd, "size"));
        assert!(graph.connect(n_subd, n_brep, "subd"));
        assert!(graph.connect(n_brep, n_polar, "geom"));
        assert!(graph.connect(n_polar, n_out, "geom"));
        // Cycle & incompatible port rejections
        assert!(!graph.connect(n_out, n_num, "x"));
        assert!(!graph.connect(n_polar, n_math, "a"));

        let geoms = graph.evaluate();
        assert_eq!(geoms.len(), 3);
        assert!(geoms.iter().all(|g| matches!(g, Geometry::BRep(_))));
    }
}
