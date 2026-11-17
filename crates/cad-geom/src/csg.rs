use cad_math::DVec3;

use crate::TriangleMesh;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    Union,
    Difference,
    Intersection,
}

const BSP_EPS: f64 = 1e-5;

#[derive(Debug, Clone)]
struct BspPlane {
    normal: DVec3,
    w: f64,
}

impl BspPlane {
    fn from_points(a: DVec3, b: DVec3, c: DVec3) -> Option<Self> {
        let n = (b - a).cross(c - a);
        let len = n.length();
        if len < 1e-10 {
            return None;
        }
        let normal = n / len;
        Some(Self {
            normal,
            w: normal.dot(a),
        })
    }

    fn flip(&mut self) {
        self.normal = -self.normal;
        self.w = -self.w;
    }
}

#[derive(Debug, Clone)]
struct Polygon {
    vertices: Vec<DVec3>,
    plane: BspPlane,
}

impl Polygon {
    fn new(vertices: Vec<DVec3>) -> Option<Self> {
        if vertices.len() < 3 {
            return None;
        }
        let plane = BspPlane::from_points(vertices[0], vertices[1], vertices[2])?;
        Some(Self { vertices, plane })
    }

    fn flip(&mut self) {
        self.vertices.reverse();
        self.plane.flip();
    }
}

#[derive(Debug, Clone)]
struct BspNode {
    plane: Option<BspPlane>,
    front: Option<Box<BspNode>>,
    back: Option<Box<BspNode>>,
    polygons: Vec<Polygon>,
}

impl BspNode {
    fn new(polygons: Vec<Polygon>) -> Self {
        let mut node = Self {
            plane: None,
            front: None,
            back: None,
            polygons: Vec::new(),
        };
        if !polygons.is_empty() {
            node.build(polygons);
        }
        node
    }

    fn invert(&mut self) {
        for p in &mut self.polygons {
            p.flip();
        }
        if let Some(pl) = &mut self.plane {
            pl.flip();
        }
        if let Some(f) = &mut self.front {
            f.invert();
        }
        if let Some(b) = &mut self.back {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    fn clip_polygons(&self, polygons: Vec<Polygon>) -> Vec<Polygon> {
        let Some(plane) = &self.plane else {
            return polygons;
        };
        let mut front = Vec::new();
        let mut back = Vec::new();
        let mut cf = Vec::new();
        let mut cb = Vec::new();
        for poly in polygons {
            split_polygon(plane, poly, &mut cf, &mut cb, &mut front, &mut back);
        }
        front.extend(cf);
        back.extend(cb);
        if let Some(f) = &self.front {
            front = f.clip_polygons(front);
        }
        if let Some(b) = &self.back {
            back = b.clip_polygons(back);
        } else {
            back.clear();
        }
        front.extend(back);
        front
    }

    fn clip_to(&mut self, bsp: &BspNode) {
        self.polygons = bsp.clip_polygons(std::mem::take(&mut self.polygons));
        if let Some(f) = &mut self.front {
            f.clip_to(bsp);
        }
        if let Some(b) = &mut self.back {
            b.clip_to(bsp);
        }
    }

    fn all_polygons(&self) -> Vec<Polygon> {
        let mut out = self.polygons.clone();
        if let Some(f) = &self.front {
            out.extend(f.all_polygons());
        }
        if let Some(b) = &self.back {
            out.extend(b.all_polygons());
        }
        out
    }

    fn build(&mut self, polygons: Vec<Polygon>) {
        if polygons.is_empty() {
            return;
        }
        if self.plane.is_none() {
            self.plane = Some(polygons[0].plane.clone());
        }
        let plane = self.plane.clone().unwrap();
        let mut front = Vec::new();
        let mut back = Vec::new();
        let mut cf = Vec::new();
        let mut cb = Vec::new();
        for poly in polygons {
            split_polygon(
                &plane,
                poly,
                &mut cf,
                &mut cb,
                &mut front,
                &mut back,
            );
        }
        self.polygons.extend(cf);
        self.polygons.extend(cb);
        if !front.is_empty() {
            if let Some(f) = &mut self.front {
                f.build(front);
            } else {
                self.front = Some(Box::new(BspNode::new(front)));
            }
        }
        if !back.is_empty() {
            if let Some(b) = &mut self.back {
                b.build(back);
            } else {
                self.back = Some(Box::new(BspNode::new(back)));
            }
        }
    }
}

fn split_polygon(
    plane: &BspPlane,
    poly: Polygon,
    coplanar_front: &mut Vec<Polygon>,
    coplanar_back: &mut Vec<Polygon>,
    front: &mut Vec<Polygon>,
    back: &mut Vec<Polygon>,
) {
    const COPLANAR: u8 = 0;
    const FRONT: u8 = 1;
    const BACK: u8 = 2;
    const SPANNING: u8 = 3;

    let mut poly_type = 0_u8;
    let mut types = Vec::with_capacity(poly.vertices.len());
    for v in &poly.vertices {
        let t = plane.normal.dot(*v) - plane.w;
        let vt = if t < -BSP_EPS {
            BACK
        } else if t > BSP_EPS {
            FRONT
        } else {
            COPLANAR
        };
        poly_type |= vt;
        types.push(vt);
    }

    match poly_type {
        COPLANAR => {
            if plane.normal.dot(poly.plane.normal) > 0.0 {
                coplanar_front.push(poly);
            } else {
                coplanar_back.push(poly);
            }
        }
        FRONT => front.push(poly),
        BACK => back.push(poly),
        SPANNING => {
            let mut f = Vec::new();
            let mut b = Vec::new();
            let n = poly.vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let ti = types[i];
                let tj = types[j];
                let vi = poly.vertices[i];
                let vj = poly.vertices[j];
                if ti != BACK {
                    f.push(vi);
                }
                if ti != FRONT {
                    b.push(vi);
                }
                if (ti | tj) == SPANNING {
                    let denom = plane.normal.dot(vj - vi);
                    if denom.abs() > 1e-12 {
                        let t = ((plane.w - plane.normal.dot(vi)) / denom).clamp(0.0, 1.0);
                        let v = vi.lerp(vj, t);
                        f.push(v);
                        b.push(v);
                    }
                }
            }
            if f.len() >= 3 {
                if let Some(p) = Polygon::new(f) {
                    front.push(p);
                }
            }
            if b.len() >= 3 {
                if let Some(p) = Polygon::new(b) {
                    back.push(p);
                }
            }
        }
        _ => {}
    }
}

fn mesh_to_polygons(mesh: &TriangleMesh) -> Vec<Polygon> {
    mesh.indices
        .iter()
        .filter_map(|&[i0, i1, i2]| {
            let v0 = DVec3::from_array(mesh.positions[i0 as usize]);
            let v1 = DVec3::from_array(mesh.positions[i1 as usize]);
            let v2 = DVec3::from_array(mesh.positions[i2 as usize]);
            Polygon::new(vec![v0, v1, v2])
        })
        .collect()
}

fn polygons_to_mesh(polygons: &[Polygon]) -> TriangleMesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();

    for poly in polygons {
        if poly.vertices.len() < 3 {
            continue;
        }
        let n = poly.plane.normal.to_array();
        let base = positions.len() as u32;
        for v in &poly.vertices {
            positions.push(v.to_array());
            normals.push(n);
        }
        for i in 1..(poly.vertices.len() - 1) {
            indices.push([base, base + i as u32, base + (i + 1) as u32]);
        }
    }

    TriangleMesh {
        positions,
        normals,
        indices,
    }
}

/// Perform a clean-room BSP CSG Boolean operation on two closed manifold meshes.
pub fn mesh_boolean(a: &TriangleMesh, b: &TriangleMesh, op: BooleanOp) -> TriangleMesh {
    let mut bsp_a = BspNode::new(mesh_to_polygons(a));
    let mut bsp_b = BspNode::new(mesh_to_polygons(b));

    match op {
        BooleanOp::Union => {
            bsp_a.clip_to(&bsp_b);
            bsp_b.clip_to(&bsp_a);
            bsp_b.invert();
            bsp_b.clip_to(&bsp_a);
            bsp_b.invert();
            let mut polys = bsp_a.all_polygons();
            polys.extend(bsp_b.all_polygons());
            polygons_to_mesh(&polys)
        }
        BooleanOp::Difference => {
            bsp_a.invert();
            bsp_a.clip_to(&bsp_b);
            bsp_b.clip_to(&bsp_a);
            bsp_b.invert();
            bsp_b.clip_to(&bsp_a);
            bsp_b.invert();
            let mut polys = bsp_a.all_polygons();
            polys.extend(bsp_b.all_polygons());
            let mut combined = BspNode::new(polys);
            combined.invert();
            polygons_to_mesh(&combined.all_polygons())
        }
        BooleanOp::Intersection => {
            bsp_a.invert();
            bsp_b.clip_to(&bsp_a);
            bsp_b.invert();
            bsp_a.clip_to(&bsp_b);
            bsp_b.clip_to(&bsp_a);
            let mut polys = bsp_a.all_polygons();
            polys.extend(bsp_b.all_polygons());
            let mut combined = BspNode::new(polys);
            combined.invert();
            polygons_to_mesh(&combined.all_polygons())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boolean_union_and_difference_boxes_produce_valid_geometry() {
        let a = TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let b = TriangleMesh::from_box(DVec3::new(1.0, 0.0, 0.0), DVec3::splat(2.0));

        let uni = mesh_boolean(&a, &b, BooleanOp::Union);
        assert!(!uni.indices.is_empty());
        let uni_bb = uni.bounding_box();
        assert!((uni_bb.min.x - (-1.0)).abs() < 1e-4);
        assert!((uni_bb.max.x - 2.0).abs() < 1e-4);

        let diff = mesh_boolean(&a, &b, BooleanOp::Difference);
        assert!(!diff.indices.is_empty());
        let diff_bb = diff.bounding_box();
        assert!((diff_bb.min.x - (-1.0)).abs() < 1e-4);
        assert!((diff_bb.max.x - 0.0).abs() < 1e-4);
    }
}
