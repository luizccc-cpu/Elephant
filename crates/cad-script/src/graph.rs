use cad_doc::Document;
use cad_geom::{DMat4, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh};
use serde::{Deserialize, Serialize};

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
    Circle {
        radius: f64,
    },
    Extrude {
        height: f64,
    },
    Box {
        sx: f64,
        sy: f64,
        sz: f64,
    },
    LinearArray {
        count: usize,
        step_x: f64,
        step_y: f64,
        step_z: f64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParametricNode {
    pub id: usize,
    pub label: String,
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

/// Grasshopper-style local parametric node graph.
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
                    kind: NodeKind::NumberSlider {
                        value: 1.5,
                        min: 0.2,
                        max: 8.0,
                    },
                },
                ParametricNode {
                    id: 2,
                    label: "Height".to_string(),
                    kind: NodeKind::NumberSlider {
                        value: 3.0,
                        min: 0.2,
                        max: 12.0,
                    },
                },
                ParametricNode {
                    id: 3,
                    label: "Count".to_string(),
                    kind: NodeKind::NumberSlider {
                        value: 4.0,
                        min: 1.0,
                        max: 12.0,
                    },
                },
                ParametricNode {
                    id: 4,
                    label: "Circle".to_string(),
                    kind: NodeKind::Circle { radius: 1.5 },
                },
                ParametricNode {
                    id: 5,
                    label: "Extrude".to_string(),
                    kind: NodeKind::Extrude { height: 3.0 },
                },
                ParametricNode {
                    id: 6,
                    label: "Linear Array".to_string(),
                    kind: NodeKind::LinearArray {
                        count: 4,
                        step_x: 4.0,
                        step_y: 0.0,
                        step_z: 0.0,
                    },
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
            ],
        }
    }
}

impl ParametricGraph {
    fn eval_node(&self, id: usize, depth: usize) -> Option<NodeValue> {
        if depth > 32 {
            return None;
        }
        let node = self.nodes.iter().find(|n| n.id == id)?;
        let input = |port: &str| -> Option<NodeValue> {
            let link = self
                .links
                .iter()
                .find(|l| l.to_node == id && l.to_port == port)?;
            self.eval_node(link.from_node, depth + 1)
        };

        match &node.kind {
            NodeKind::NumberSlider { value, .. } => Some(NodeValue::Number(*value)),
            NodeKind::Point { x, y, z } => Some(NodeValue::Point(DVec3::new(*x, *y, *z))),
            NodeKind::Circle { radius } => {
                let r = match input("radius") {
                    Some(NodeValue::Number(v)) => v.max(0.05),
                    _ => *radius,
                };
                let origin = match input("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::ZERO,
                };
                let mut plane = Plane::world_xy();
                plane.origin = origin;
                let c = NurbsCurve::circle(&plane, r).ok()?;
                Some(NodeValue::Geometries(vec![Geometry::Curve(c)]))
            }
            NodeKind::Extrude { height } => {
                let h = match input("height") {
                    Some(NodeValue::Number(v)) => v,
                    _ => *height,
                };
                let Some(NodeValue::Geometries(geoms)) = input("curve") else {
                    return None;
                };
                let out = geoms
                    .into_iter()
                    .filter_map(|g| match g {
                        Geometry::Curve(c) => Some(Geometry::Surface(
                            NurbsSurface::extrude_curve(&c, DVec3::new(0.0, 0.0, h)),
                        )),
                        _ => None,
                    })
                    .collect();
                Some(NodeValue::Geometries(out))
            }
            NodeKind::Box { sx, sy, sz } => {
                let size = match input("size") {
                    Some(NodeValue::Number(v)) => DVec3::splat(v.max(0.05)),
                    _ => DVec3::new(*sx, *sy, *sz),
                };
                let center = match input("center") {
                    Some(NodeValue::Point(p)) => p,
                    _ => DVec3::new(0.0, 0.0, size.z * 0.5),
                };
                Some(NodeValue::Geometries(vec![Geometry::Mesh(
                    TriangleMesh::from_box(center, size),
                )]))
            }
            NodeKind::LinearArray {
                count,
                step_x,
                step_y,
                step_z,
            } => {
                let n = match input("count") {
                    Some(NodeValue::Number(v)) => (v.round() as isize).clamp(1, 64) as usize,
                    _ => (*count).clamp(1, 64),
                };
                let Some(NodeValue::Geometries(geoms)) = input("geom") else {
                    return None;
                };
                let step = DVec3::new(*step_x, *step_y, *step_z);
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
        }
    }

    /// Evaluate the terminal node(s) of the graph into preview/bakeable `Geometry`.
    pub fn evaluate(&self) -> Vec<Geometry> {
        let Some(last) = self.nodes.last() else {
            return Vec::new();
        };
        match self.eval_node(last.id, 0) {
            Some(NodeValue::Geometries(g)) => g,
            _ => Vec::new(),
        }
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
}
