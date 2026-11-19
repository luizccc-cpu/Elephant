use std::collections::{HashMap, HashSet};

use cad_math::{Aabb, DMat4, DVec3, Ray};
use serde::{Deserialize, Serialize};

use crate::brep::{BRep, BRepFace};
use crate::error::GeomError;
use crate::mesh::TriangleMesh;
use crate::nurbs_curve::{ControlPoint, NurbsCurve};
use crate::nurbs_surface::NurbsSurface;

/// Normalize an undirected edge vertex pair to `[min, max]`.
#[inline]
pub fn normalize_edge(a: usize, b: usize) -> [usize; 2] {
    if a < b { [a, b] } else { [b, a] }
}

/// Catmull-Clark Subdivision Surface with sharp edge creases and control-cage editing.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubDMesh {
    pub vertices: Vec<DVec3>,
    pub faces: Vec<Vec<usize>>,
    #[serde(default)]
    pub creased_edges: Vec<[usize; 2]>,
    #[serde(default = "default_subdiv_level")]
    pub subdivision_level: u8,
}

fn default_subdiv_level() -> u8 {
    2
}

impl SubDMesh {
    pub fn new(
        vertices: Vec<DVec3>,
        faces: Vec<Vec<usize>>,
        creased_edges: Vec<[usize; 2]>,
        subdivision_level: u8,
    ) -> Self {
        let mut normalized: Vec<[usize; 2]> = creased_edges
            .into_iter()
            .filter(|&[a, b]| a != b)
            .map(|[a, b]| normalize_edge(a, b))
            .collect();
        normalized.sort_unstable();
        normalized.dedup();
        Self {
            vertices,
            faces,
            creased_edges: normalized,
            subdivision_level: subdivision_level.clamp(1, 4),
        }
    }

    /// 6-quad Catmull-Clark box control cage centered at `center` with extents `size`.
    pub fn from_box(center: DVec3, size: DVec3) -> Self {
        let hx = size.x.abs().max(1e-4) * 0.5;
        let hy = size.y.abs().max(1e-4) * 0.5;
        let hz = size.z.abs().max(1e-4) * 0.5;
        let vertices = vec![
            center + DVec3::new(-hx, -hy, -hz), // 0
            center + DVec3::new(hx, -hy, -hz),  // 1
            center + DVec3::new(hx, hy, -hz),   // 2
            center + DVec3::new(-hx, hy, -hz),  // 3
            center + DVec3::new(-hx, -hy, hz),  // 4
            center + DVec3::new(hx, -hy, hz),   // 5
            center + DVec3::new(hx, hy, hz),    // 6
            center + DVec3::new(-hx, hy, hz),   // 7
        ];
        let faces = vec![
            vec![0, 3, 2, 1], // -Z
            vec![4, 5, 6, 7], // +Z
            vec![0, 1, 5, 4], // -Y
            vec![1, 2, 6, 5], // +X
            vec![2, 3, 7, 6], // +Y
            vec![3, 0, 4, 7], // -X
        ];
        Self {
            vertices,
            faces,
            creased_edges: Vec::new(),
            subdivision_level: 2,
        }
    }

    /// Closed cube cage calibrated so its Catmull-Clark limit surface approximates a sphere of `radius`.
    pub fn from_sphere(center: DVec3, radius: f64) -> Self {
        let r = radius.abs().max(1e-4);
        let mut cage = Self::from_box(DVec3::ZERO, DVec3::splat(2.0));
        cage.subdivision_level = 3;
        let refined = cage.subdivided(3);
        let avg_r = if refined.vertices.is_empty() {
            1.0
        } else {
            refined.vertices.iter().map(|v| v.length()).sum::<f64>()
                / refined.vertices.len() as f64
        };
        let scale = if avg_r > 1e-9 { r / avg_r } else { r };
        for v in &mut cage.vertices {
            *v = *v * scale + center;
        }
        cage
    }

    /// Closed SubD cylinder cage with creased top and bottom cap rims.
    pub fn from_cylinder(base_center: DVec3, radius: f64, height: f64, sides: usize) -> Self {
        let n = sides.clamp(6, 32);
        let r = radius.abs().max(1e-4);
        let h = if height.abs() < 1e-4 { 1.0 } else { height };
        let z0 = if h >= 0.0 { base_center.z } else { base_center.z + h };
        let z1 = if h >= 0.0 { base_center.z + h } else { base_center.z };

        let mut vertices = Vec::with_capacity(n * 2);
        for i in 0..n {
            let a = std::f64::consts::TAU * (i as f64 / n as f64);
            vertices.push(DVec3::new(base_center.x + r * a.cos(), base_center.y + r * a.sin(), z0));
        }
        for i in 0..n {
            let a = std::f64::consts::TAU * (i as f64 / n as f64);
            vertices.push(DVec3::new(base_center.x + r * a.cos(), base_center.y + r * a.sin(), z1));
        }

        let mut faces = Vec::with_capacity(n + 2);
        let mut creased = Vec::with_capacity(n * 2);
        for i in 0..n {
            let next = (i + 1) % n;
            faces.push(vec![i, next, n + next, n + i]);
            creased.push(normalize_edge(i, next));
            creased.push(normalize_edge(n + i, n + next));
        }
        faces.push((0..n).rev().collect());
        faces.push((n..2 * n).collect());
        Self::new(vertices, faces, creased, 2)
    }

    /// Closed quad SubD torus cage centered at `center`.
    pub fn from_torus(
        center: DVec3,
        major_radius: f64,
        minor_radius: f64,
        major_seg: usize,
        minor_seg: usize,
    ) -> Self {
        let maj_n = major_seg.clamp(6, 32);
        let min_n = minor_seg.clamp(4, 24);
        let maj_r = major_radius.abs().max(1e-3);
        let min_r = minor_radius.abs().max(1e-4);

        let mut vertices = Vec::with_capacity(maj_n * min_n);
        for i in 0..maj_n {
            let u = std::f64::consts::TAU * (i as f64 / maj_n as f64);
            let cu = u.cos();
            let su = u.sin();
            for j in 0..min_n {
                let v = std::f64::consts::TAU * (j as f64 / min_n as f64);
                let ring = maj_r + min_r * v.cos();
                vertices.push(center + DVec3::new(ring * cu, ring * su, min_r * v.sin()));
            }
        }

        let mut faces = Vec::with_capacity(maj_n * min_n);
        for i in 0..maj_n {
            let ni = (i + 1) % maj_n;
            for j in 0..min_n {
                let nj = (j + 1) % min_n;
                faces.push(vec![
                    i * min_n + j,
                    ni * min_n + j,
                    ni * min_n + nj,
                    i * min_n + nj,
                ]);
            }
        }
        Self::new(vertices, faces, Vec::new(), 2)
    }

    /// Convert a `TriangleMesh` into a welded `SubDMesh` cage (merging coplanar adjacent triangle pairs into quads).
    pub fn from_mesh(mesh: &TriangleMesh) -> Self {
        let quant = |p: [f64; 3]| {
            (
                (p[0] * 1e5).round() as i64,
                (p[1] * 1e5).round() as i64,
                (p[2] * 1e5).round() as i64,
            )
        };
        let mut weld_map: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut vertices = Vec::new();
        let mut tri_faces: Vec<[usize; 3]> = Vec::new();

        for &[a, b, c] in &mesh.indices {
            let mut idx = [0usize; 3];
            for (k, &orig) in [a, b, c].iter().enumerate() {
                let p = mesh.positions[orig as usize];
                let key = quant(p);
                let vi = *weld_map.entry(key).or_insert_with(|| {
                    let id = vertices.len();
                    vertices.push(DVec3::from_array(p));
                    id
                });
                idx[k] = vi;
            }
            if idx[0] != idx[1] && idx[1] != idx[2] && idx[2] != idx[0] {
                tri_faces.push(idx);
            }
        }

        // Attempt to merge adjacent coplanar, convex triangle pairs into quads
        let mut edge_to_tris: HashMap<[usize; 2], Vec<usize>> = HashMap::new();
        for (ti, &[a, b, c]) in tri_faces.iter().enumerate() {
            for (u, v) in [(a, b), (b, c), (c, a)] {
                edge_to_tris.entry(normalize_edge(u, v)).or_default().push(ti);
            }
        }

        let tri_normal = |t: [usize; 3]| -> DVec3 {
            let p0 = vertices[t[0]];
            let p1 = vertices[t[1]];
            let p2 = vertices[t[2]];
            (p1 - p0).cross(p2 - p0).normalize_or_zero()
        };

        let mut merged = vec![false; tri_faces.len()];
        let mut faces: Vec<Vec<usize>> = Vec::new();

        for ti in 0..tri_faces.len() {
            if merged[ti] {
                continue;
            }
            let t0 = tri_faces[ti];
            let n0 = tri_normal(t0);
            let mut quad_found: Option<(usize, Vec<usize>)> = None;

            for (u, v, opp0) in [(t0[0], t0[1], t0[2]), (t0[1], t0[2], t0[0]), (t0[2], t0[0], t0[1])] {
                let ek = normalize_edge(u, v);
                let Some(adj) = edge_to_tris.get(&ek) else {
                    continue;
                };
                if adj.len() != 2 {
                    continue;
                }
                let tj = if adj[0] == ti { adj[1] } else { adj[0] };
                if merged[tj] {
                    continue;
                }
                let t1 = tri_faces[tj];
                if n0.dot(tri_normal(t1)) < 0.998 {
                    continue;
                }
                let opp1 = t1.into_iter().find(|&x| x != u && x != v);
                let Some(opp1) = opp1 else {
                    continue;
                };
                // t0 has directed edge u -> v with third vertex opp0, so CCW quad is [opp0, u, opp1, v]
                let quad = [opp0, u, opp1, v];
                // Verify convexity and same normal sign
                let mut ok = true;
                for k in 0..4 {
                    let a = vertices[quad[k]];
                    let b = vertices[quad[(k + 1) % 4]];
                    let c = vertices[quad[(k + 2) % 4]];
                    if (b - a).cross(c - b).dot(n0) <= 1e-10 {
                        ok = false;
                        break;
                    }
                }
                if ok {
                    quad_found = Some((tj, quad.to_vec()));
                    break;
                }
            }

            if let Some((tj, quad)) = quad_found {
                merged[ti] = true;
                merged[tj] = true;
                faces.push(quad);
            } else {
                merged[ti] = true;
                faces.push(t0.to_vec());
            }
        }

        Self::new(vertices, faces, Vec::new(), 2)
    }

    /// Toggle crease state on an undirected cage edge `[a, b]`. Returns `true` if now creased.
    pub fn toggle_crease_edge(&mut self, a: usize, b: usize) -> bool {
        if a == b || a >= self.vertices.len() || b >= self.vertices.len() {
            return false;
        }
        let ek = normalize_edge(a, b);
        if let Some(pos) = self.creased_edges.iter().position(|&e| e == ek) {
            self.creased_edges.remove(pos);
            false
        } else {
            self.creased_edges.push(ek);
            self.creased_edges.sort_unstable();
            true
        }
    }

    /// Toggle sharp creases on open boundary edges, or if closed, toggle the top-most face's perimeter ring.
    pub fn toggle_boundary_creases(&mut self) -> usize {
        let mut edge_adj: HashMap<[usize; 2], usize> = HashMap::new();
        for f in &self.faces {
            if f.len() < 3 {
                continue;
            }
            for k in 0..f.len() {
                let ek = normalize_edge(f[k], f[(k + 1) % f.len()]);
                *edge_adj.entry(ek).or_insert(0) += 1;
            }
        }
        let mut targets: Vec<[usize; 2]> = edge_adj
            .iter()
            .filter_map(|(&ek, &c)| (c == 1).then_some(ek))
            .collect();

        if targets.is_empty() {
            if let Some((best_fi, _)) = self
                .faces
                .iter()
                .enumerate()
                .filter(|(_, f)| f.len() >= 3)
                .max_by(|(_, a), (_, b)| {
                    let za = a.iter().map(|&i| self.vertices[i].z).sum::<f64>() / a.len() as f64;
                    let zb = b.iter().map(|&i| self.vertices[i].z).sum::<f64>() / b.len() as f64;
                    za.total_cmp(&zb)
                })
            {
                let f = &self.faces[best_fi];
                for k in 0..f.len() {
                    targets.push(normalize_edge(f[k], f[(k + 1) % f.len()]));
                }
            }
        }

        for ek in &targets {
            self.toggle_crease_edge(ek[0], ek[1]);
        }
        targets.len()
    }

    /// Extrude cage face `face_idx` along its unit normal by `distance`.
    pub fn extrude_face(&mut self, face_idx: usize, distance: f64) -> Result<usize, GeomError> {
        let Some(face) = self.faces.get(face_idx).cloned() else {
            return Err(GeomError::InvalidInput("invalid SubD face index".to_string()));
        };
        if face.len() < 3 {
            return Err(GeomError::Degenerate("cannot extrude degenerate face"));
        }
        let mut n = DVec3::ZERO;
        for k in 0..face.len() {
            let p0 = self.vertices[face[k]];
            let p1 = self.vertices[face[(k + 1) % face.len()]];
            let p2 = self.vertices[face[(k + 2) % face.len()]];
            n += (p1 - p0).cross(p2 - p0);
        }
        let normal = n.normalize_or(DVec3::Z);
        let offset = normal * distance;

        let mut new_cap = Vec::with_capacity(face.len());
        for &vi in &face {
            let new_vi = self.vertices.len();
            self.vertices.push(self.vertices[vi] + offset);
            new_cap.push(new_vi);
        }

        for k in 0..face.len() {
            let next = (k + 1) % face.len();
            self.faces.push(vec![
                face[k],
                face[next],
                new_cap[next],
                new_cap[k],
            ]);
        }
        self.faces[face_idx] = new_cap;
        Ok(face_idx)
    }

    /// Perform one full step of Catmull-Clark subdivision, honoring sharp/creased edges and open boundaries.
    pub fn subdivide_once(&self) -> Self {
        let nv = self.vertices.len();
        let valid_faces: Vec<&Vec<usize>> = self.faces.iter().filter(|f| f.len() >= 3).collect();
        let nf = valid_faces.len();
        if nv == 0 || nf == 0 {
            return self.clone();
        }

        let crease_set: HashSet<[usize; 2]> = self.creased_edges.iter().copied().collect();
        let mut edge_to_faces: HashMap<[usize; 2], Vec<usize>> = HashMap::new();
        let mut edge_list: Vec<[usize; 2]> = Vec::new();
        let mut edge_idx_map: HashMap<[usize; 2], usize> = HashMap::new();
        let mut vert_faces: Vec<Vec<usize>> = vec![Vec::new(); nv];
        let mut vert_edges: Vec<Vec<[usize; 2]>> = vec![Vec::new(); nv];

        for (fi, f) in valid_faces.iter().enumerate() {
            for &vi in *f {
                if vi < nv && !vert_faces[vi].contains(&fi) {
                    vert_faces[vi].push(fi);
                }
            }
            for k in 0..f.len() {
                let a = f[k];
                let b = f[(k + 1) % f.len()];
                if a == b || a >= nv || b >= nv {
                    continue;
                }
                let ek = normalize_edge(a, b);
                if !edge_idx_map.contains_key(&ek) {
                    let ei = edge_list.len();
                    edge_list.push(ek);
                    edge_idx_map.insert(ek, ei);
                    vert_edges[a].push(ek);
                    vert_edges[b].push(ek);
                }
                edge_to_faces.entry(ek).or_default().push(fi);
            }
        }

        // 1. Face points
        let face_pts: Vec<DVec3> = valid_faces
            .iter()
            .map(|f| {
                let sum: DVec3 = f.iter().map(|&i| self.vertices[i]).sum();
                sum / (f.len() as f64)
            })
            .collect();

        // 2. Edge points
        let edge_pts: Vec<DVec3> = edge_list
            .iter()
            .map(|&ek| {
                let mid = (self.vertices[ek[0]] + self.vertices[ek[1]]) * 0.5;
                let adj = &edge_to_faces[&ek];
                if crease_set.contains(&ek) || adj.len() != 2 {
                    mid
                } else {
                    (mid * 2.0 + face_pts[adj[0]] + face_pts[adj[1]]) * 0.25
                }
            })
            .collect();

        // 3. Updated vertex points
        let mut out_vertices = Vec::with_capacity(nv + nf + edge_list.len());
        for vi in 0..nv {
            let v = self.vertices[vi];
            let adj_faces = &vert_faces[vi];
            let adj_edges = &vert_edges[vi];
            if adj_faces.is_empty() || adj_edges.is_empty() {
                out_vertices.push(v);
                continue;
            }

            let mut sharp_neighbors: Vec<usize> = Vec::new();
            for &ek in adj_edges {
                let is_sharp = crease_set.contains(&ek) || edge_to_faces[&ek].len() < 2;
                if is_sharp {
                    let other = if ek[0] == vi { ek[1] } else { ek[0] };
                    if !sharp_neighbors.contains(&other) {
                        sharp_neighbors.push(other);
                    }
                }
            }

            if sharp_neighbors.len() >= 3 {
                out_vertices.push(v);
            } else if sharp_neighbors.len() == 2 {
                let p0 = self.vertices[sharp_neighbors[0]];
                let p1 = self.vertices[sharp_neighbors[1]];
                out_vertices.push(v * 0.75 + (p0 + p1) * 0.125);
            } else {
                let f_avg = adj_faces.iter().map(|&fi| face_pts[fi]).sum::<DVec3>()
                    / (adj_faces.len() as f64);
                let r_avg = adj_edges
                    .iter()
                    .map(|&ek| (self.vertices[ek[0]] + self.vertices[ek[1]]) * 0.5)
                    .sum::<DVec3>()
                    / (adj_edges.len() as f64);
                let n = adj_faces.len() as f64;
                if adj_edges.len() == adj_faces.len() && n >= 3.0 {
                    out_vertices.push((f_avg + 2.0 * r_avg + (n - 3.0) * v) / n);
                } else {
                    out_vertices.push((f_avg + 2.0 * r_avg + v) / 4.0);
                }
            }
        }

        let face_base = out_vertices.len();
        out_vertices.extend_from_slice(&face_pts);
        let edge_base = out_vertices.len();
        out_vertices.extend_from_slice(&edge_pts);

        // 4. Emit pure quad faces
        let mut out_faces = Vec::with_capacity(nf * 4);
        for (fi, f) in valid_faces.iter().enumerate() {
            let k = f.len();
            let f_pt = face_base + fi;
            for corner in 0..k {
                let curr = f[corner];
                let next = f[(corner + 1) % k];
                let prev = f[(corner + k - 1) % k];
                let Some(&e_next_idx) = edge_idx_map.get(&normalize_edge(curr, next)) else {
                    continue;
                };
                let Some(&e_prev_idx) = edge_idx_map.get(&normalize_edge(prev, curr)) else {
                    continue;
                };
                out_faces.push(vec![
                    curr,
                    edge_base + e_next_idx,
                    f_pt,
                    edge_base + e_prev_idx,
                ]);
            }
        }

        // 5. Propagate crease tags to subdivided half-edges
        let mut out_creases = Vec::new();
        for &ek in &self.creased_edges {
            if let Some(&ei) = edge_idx_map.get(&ek) {
                let ep = edge_base + ei;
                out_creases.push(normalize_edge(ek[0], ep));
                out_creases.push(normalize_edge(ek[1], ep));
            }
        }

        Self::new(out_vertices, out_faces, out_creases, self.subdivision_level)
    }

    pub fn subdivided(&self, levels: u8) -> Self {
        let mut cur = self.clone();
        for _ in 0..levels.min(4) {
            cur = cur.subdivide_once();
        }
        cur
    }

    pub fn subdivide_cage_in_place(&mut self) {
        let next = self.subdivide_once();
        self.vertices = next.vertices;
        self.faces = next.faces;
        self.creased_edges = next.creased_edges;
    }

    /// Tessellate the limit surface into a triangle mesh (respecting `subdivision_level`).
    pub fn tessellate_display(&self) -> TriangleMesh {
        let level = self.subdivision_level.clamp(1, 4);
        let sub = self.subdivided(level);
        if sub.vertices.is_empty() || sub.faces.is_empty() {
            return TriangleMesh {
                positions: Vec::new(),
                normals: Vec::new(),
                indices: Vec::new(),
            };
        }

        let mut indices = Vec::with_capacity(sub.faces.len() * 2);
        let mut normal_accum = vec![DVec3::ZERO; sub.vertices.len()];

        for f in &sub.faces {
            if f.len() < 3 {
                continue;
            }
            for k in 1..(f.len() - 1) {
                let i0 = f[0];
                let i1 = f[k];
                let i2 = f[k + 1];
                let p0 = sub.vertices[i0];
                let p1 = sub.vertices[i1];
                let p2 = sub.vertices[i2];
                let fn_ = (p1 - p0).cross(p2 - p0);
                normal_accum[i0] += fn_;
                normal_accum[i1] += fn_;
                normal_accum[i2] += fn_;
                indices.push([i0 as u32, i1 as u32, i2 as u32]);
            }
        }

        let positions = sub.vertices.iter().map(|v| v.to_array()).collect();
        let normals = normal_accum
            .into_iter()
            .map(|n| n.normalize_or(DVec3::Z).to_array())
            .collect();
        TriangleMesh {
            positions,
            normals,
            indices,
        }
    }

    /// Unique undirected control-cage edges `(a, b, is_creased)` for display and cage wireframe rendering.
    pub fn control_cage_edges(&self) -> Vec<(DVec3, DVec3, bool)> {
        let crease_set: HashSet<[usize; 2]> = self.creased_edges.iter().copied().collect();
        let mut seen: HashSet<[usize; 2]> = HashSet::new();
        let mut out = Vec::new();
        for f in &self.faces {
            if f.len() < 2 {
                continue;
            }
            for k in 0..f.len() {
                let a = f[k];
                let b = f[(k + 1) % f.len()];
                if a >= self.vertices.len() || b >= self.vertices.len() || a == b {
                    continue;
                }
                let ek = normalize_edge(a, b);
                if seen.insert(ek) {
                    out.push((self.vertices[a], self.vertices[b], crease_set.contains(&ek)));
                }
            }
        }
        out
    }

    /// Open-boundary vertex indices (vertices incident to any edge with `< 2` adjacent faces).
    pub fn boundary_vertices(&self) -> Vec<usize> {
        let mut edge_counts: HashMap<[usize; 2], usize> = HashMap::new();
        for f in &self.faces {
            if f.len() < 3 {
                continue;
            }
            for k in 0..f.len() {
                *edge_counts
                    .entry(normalize_edge(f[k], f[(k + 1) % f.len()]))
                    .or_insert(0) += 1;
            }
        }
        let mut bset = HashSet::new();
        for (ek, cnt) in edge_counts {
            if cnt < 2 {
                bset.insert(ek[0]);
                bset.insert(ek[1]);
            }
        }
        let mut out: Vec<usize> = bset.into_iter().collect();
        out.sort_unstable();
        out
    }

    pub fn bounding_box(&self) -> Aabb {
        let mesh = self.tessellate_display();
        let mut bb = Aabb::from_points(self.vertices.iter().copied());
        if !mesh.positions.is_empty() {
            bb = bb.union(&mesh.bounding_box());
        }
        bb
    }

    pub fn transform(&mut self, mat: &DMat4) {
        for v in &mut self.vertices {
            *v = mat.transform_point3(*v);
        }
    }

    pub fn snap_points(&self) -> Vec<DVec3> {
        self.vertices.clone()
    }

    pub fn ray_intersect(&self, ray: &Ray) -> Option<f64> {
        self.tessellate_display().ray_intersect(ray)
    }
}

/// Rhino 9 `ToNURBS`: convert a `SubDMesh` into a watertight stitched `BRep` of bicubic G1/G0 NURBS patches.
pub fn subd_to_nurbs_brep(subd: &SubDMesh) -> Result<BRep, GeomError> {
    if subd.vertices.is_empty() || subd.faces.is_empty() {
        return Err(GeomError::Degenerate("empty SubD mesh cannot convert to NURBS"));
    }
    // At least 1 subdivision ensures every face is strictly a quad
    let level = subd.subdivision_level.clamp(1, 2);
    let quad_mesh = subd.subdivided(level);
    let nv = quad_mesh.vertices.len();

    let mut vert_normals = vec![DVec3::ZERO; nv];
    for f in &quad_mesh.faces {
        if f.len() != 4 {
            continue;
        }
        let p0 = quad_mesh.vertices[f[0]];
        let p1 = quad_mesh.vertices[f[1]];
        let p2 = quad_mesh.vertices[f[2]];
        let p3 = quad_mesh.vertices[f[3]];
        let fn_ = (p1 - p0).cross(p2 - p0) + (p2 - p0).cross(p3 - p0);
        for &vi in f {
            vert_normals[vi] += fn_;
        }
    }
    for n in &mut vert_normals {
        *n = n.normalize_or(DVec3::Z);
    }

    let crease_set: HashSet<[usize; 2]> = quad_mesh.creased_edges.iter().copied().collect();

    // Tangent leaving `from` toward `to`, projected onto `from`'s tangent plane unless creased.
    // Crucially depends ONLY on undirected edge endpoints and shared vertex normals, so adjacent
    // patches produce identical boundary curves to machine precision.
    let edge_tangent = |from: usize, to: usize| -> DVec3 {
        let p0 = quad_mesh.vertices[from];
        let p1 = quad_mesh.vertices[to];
        let chord = p1 - p0;
        let len = chord.length();
        if len < 1e-12 {
            return DVec3::ZERO;
        }
        let ek = normalize_edge(from, to);
        if crease_set.contains(&ek) {
            return chord;
        }
        let n = vert_normals[from];
        let proj = chord - n * chord.dot(n);
        let plen = proj.length();
        if plen > len * 0.25 {
            proj * (len / plen)
        } else {
            chord
        }
    };

    let cubic_edge = |a: usize, b: usize| -> [DVec3; 4] {
        let pa = quad_mesh.vertices[a];
        let pb = quad_mesh.vertices[b];
        let ta = edge_tangent(a, b);
        let tb = edge_tangent(b, a);
        [pa, pa + ta / 3.0, pb + tb / 3.0, pb]
    };

    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let mut faces = Vec::with_capacity(quad_mesh.faces.len());

    for f in &quad_mesh.faces {
        if f.len() != 4 {
            continue;
        }
        let [i0, i1, i2, i3] = [f[0], f[1], f[2], f[3]];
        let e_bot = cubic_edge(i0, i1);  // j = 0, i = 0..=3
        let e_top = cubic_edge(i3, i2);  // j = 3, i = 0..=3
        let e_left = cubic_edge(i0, i3); // i = 0, j = 0..=3
        let e_right = cubic_edge(i1, i2);// i = 3, j = 0..=3

        let mut grid = [[DVec3::ZERO; 4]; 4];
        for i in 0..4 {
            grid[i][0] = e_bot[i];
            grid[i][3] = e_top[i];
        }
        for j in 0..4 {
            grid[0][j] = e_left[j];
            grid[3][j] = e_right[j];
        }

        let c00 = grid[0][0];
        let c10 = grid[3][0];
        let c01 = grid[0][3];
        let c11 = grid[3][3];
        for i in 1..3 {
            let u = (i as f64) / 3.0;
            for j in 1..3 {
                let v = (j as f64) / 3.0;
                let bilinear = (1.0 - u) * (1.0 - v) * c00
                    + u * (1.0 - v) * c10
                    + (1.0 - u) * v * c01
                    + u * v * c11;
                grid[i][j] = (1.0 - u) * grid[0][j]
                    + u * grid[3][j]
                    + (1.0 - v) * grid[i][0]
                    + v * grid[i][3]
                    - bilinear;
            }
        }

        let mut cps = Vec::with_capacity(16);
        for i in 0..4 {
            for j in 0..4 {
                cps.push(ControlPoint::unweighted(grid[i][j]));
            }
        }
        let srf = NurbsSurface::new(3, 3, knots.clone(), knots.clone(), 4, 4, cps)?;
        faces.push(BRepFace::untrimmed(srf));
    }

    if faces.is_empty() {
        return Err(GeomError::Degenerate("SubD produced no valid quad patches"));
    }
    Ok(BRep::from_faces(faces, 1e-4))
}

/// Rhino `MatchSubD`: snap open SubD boundary vertices to `target` surface (plus optional G1 1st interior ring alignment).
pub fn match_subd_to_surface(
    subd: &mut SubDMesh,
    target: &NurbsSurface,
    g1_tangent: bool,
) -> Result<usize, GeomError> {
    let bverts = subd.boundary_vertices();
    if bverts.is_empty() {
        return Err(GeomError::InvalidInput(
            "SubD has no open boundary vertices to match".to_string(),
        ));
    }

    let bset: HashSet<usize> = bverts.iter().copied().collect();
    let mut normals: HashMap<usize, DVec3> = HashMap::with_capacity(bverts.len());
    for &vi in &bverts {
        let p = subd.vertices[vi];
        let (u, v, pt, _) = target.closest_point(p);
        let (_, n) = target.evaluate_point_and_normal(u, v);
        subd.vertices[vi] = pt;
        normals.insert(vi, n);
    }

    if g1_tangent {
        // Adjust interior 1-ring neighbors connected to boundary vertices onto target's tangent plane
        let mut adj_boundary: HashMap<usize, Vec<usize>> = HashMap::new();
        for f in &subd.faces {
            for k in 0..f.len() {
                let a = f[k];
                let b = f[(k + 1) % f.len()];
                if bset.contains(&a) && !bset.contains(&b) {
                    adj_boundary.entry(b).or_default().push(a);
                }
                if bset.contains(&b) && !bset.contains(&a) {
                    adj_boundary.entry(a).or_default().push(b);
                }
            }
        }
        for (vi, b_neighbors) in adj_boundary {
            let q = subd.vertices[vi];
            let mut accum = DVec3::ZERO;
            let mut count = 0usize;
            for &bi in &b_neighbors {
                if let Some(&n) = normals.get(&bi) {
                    let bp = subd.vertices[bi];
                    let proj = q - n * (q - bp).dot(n);
                    accum += proj;
                    count += 1;
                }
            }
            if count > 0 {
                subd.vertices[vi] = accum / (count as f64);
            }
        }
    }

    Ok(bverts.len())
}

/// Snap open SubD boundary vertices to `target` NURBS curve.
pub fn match_subd_to_curve(subd: &mut SubDMesh, target: &NurbsCurve) -> Result<usize, GeomError> {
    let bverts = subd.boundary_vertices();
    if bverts.is_empty() {
        return Err(GeomError::InvalidInput(
            "SubD has no open boundary vertices to match".to_string(),
        ));
    }
    for &vi in &bverts {
        let (_, pt, _) = target.closest_point(subd.vertices[vi]);
        subd.vertices[vi] = pt;
    }
    Ok(bverts.len())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::compute_geometry_mass_properties;
    use crate::Geometry;
    use cad_math::Plane;

    #[test]
    fn catmull_clark_box_converges_and_closes() {
        let subd = SubDMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        let level2 = subd.subdivided(2);
        assert_eq!(level2.faces.len(), 6 * 16);
        assert!(level2.faces.iter().all(|f| f.len() == 4));

        let mp = compute_geometry_mass_properties(&Geometry::SubD(subd)).expect("mass props");
        assert!(mp.is_closed_solid);
        assert!(mp.volume > 1.5 && mp.volume < 8.0);
        assert!(mp.area > 6.0 && mp.area < 24.0);
    }

    #[test]
    fn subd_crease_preserves_sharp_edge() {
        let mut subd = SubDMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        assert!(subd.toggle_crease_edge(4, 5));
        let sub1 = subd.subdivided(1);
        // Creased midpoint between (-1,-1,1) and (1,-1,1) must remain strictly at y=-1, z=1
        assert!(sub1.vertices.iter().any(|p| p.distance(DVec3::new(0.0, -1.0, 1.0)) < 1e-9));
        // Uncreased opposite bottom edge midpoint shrinks inward smoothly
        assert!(!sub1.vertices.iter().any(|p| p.distance(DVec3::new(0.0, -1.0, -1.0)) < 1e-4));
    }

    #[test]
    fn subd_to_nurbs_produces_stitched_brep() {
        let mut subd = SubDMesh::from_box(DVec3::ZERO, DVec3::splat(2.0));
        subd.subdivision_level = 1;
        let brep = subd_to_nurbs_brep(&subd).expect("ToNURBS succeeds");
        assert_eq!(brep.faces.len(), 24);
        assert!(brep.is_solid(), "stitched ToNURBS B-Rep must be closed solid");
        let mp = compute_geometry_mass_properties(&Geometry::BRep(brep)).expect("brep mass");
        assert!(mp.is_closed_solid);
        assert!(mp.volume > 1.0);
    }

    #[test]
    fn match_subd_aligns_boundary_to_surface() {
        // Flat open 2x2 quad patch above a curved saddle/plane surface
        let mut patch = SubDMesh::new(
            vec![
                DVec3::new(0.0, 0.0, 3.0),
                DVec3::new(4.0, 0.0, 3.0),
                DVec3::new(4.0, 4.0, 3.0),
                DVec3::new(0.0, 4.0, 3.0),
                DVec3::new(2.0, 0.0, 3.0),
                DVec3::new(4.0, 2.0, 3.0),
                DVec3::new(2.0, 4.0, 3.0),
                DVec3::new(0.0, 2.0, 3.0),
                DVec3::new(2.0, 2.0, 3.5),
            ],
            vec![
                vec![0, 4, 8, 7],
                vec![4, 1, 5, 8],
                vec![8, 5, 2, 6],
                vec![7, 8, 6, 3],
            ],
            Vec::new(),
            2,
        );
        let plane = Plane {
            origin: DVec3::new(2.0, 2.0, 0.5),
            x_axis: DVec3::X,
            y_axis: DVec3::Y,
            z_axis: DVec3::Z,
        };
        let target = NurbsSurface::plane_rect(&plane, 6.0, 6.0);
        let matched = match_subd_to_surface(&mut patch, &target, true).expect("MatchSubD");
        assert_eq!(matched, 8);
        for vi in patch.boundary_vertices() {
            let (_, _, closest, d) = target.closest_point(patch.vertices[vi]);
            assert!(d < 1e-4 && patch.vertices[vi].distance(closest) < 1e-4);
        }
        // Interior center vertex (index 8) should also have projected to z=0.5 by G1 alignment
        assert!((patch.vertices[8].z - 0.5).abs() < 1e-4);
    }
}
