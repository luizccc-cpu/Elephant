use std::collections::{HashMap, VecDeque};

use cad_math::{Aabb, DVec3};
use serde::{Deserialize, Serialize};

use crate::error::GeomError;
use crate::mesh::TriangleMesh;
use crate::Geometry;

/// Parameters controlling Rhino 8/9-style `ShrinkWrap` watertight voxel remeshing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShrinkWrapOptions {
    pub voxel_size: f64,
    pub offset: f64,
    pub smoothing_iterations: usize,
    pub fill_holes: bool,
    pub max_grid_dim: usize,
}

impl Default for ShrinkWrapOptions {
    fn default() -> Self {
        Self {
            voxel_size: 0.25,
            offset: 0.0,
            smoothing_iterations: 4,
            fill_holes: true,
            max_grid_dim: 68,
        }
    }
}

struct TriData {
    a: DVec3,
    b: DVec3,
    c: DVec3,
    min: DVec3,
    max: DVec3,
}

/// Produce a closed, watertight, manifold `TriangleMesh` wrapping one or more geometries.
pub fn shrinkwrap_geometries(
    geoms: &[&Geometry],
    opts: &ShrinkWrapOptions,
) -> Result<TriangleMesh, GeomError> {
    let mut tris: Vec<TriData> = Vec::new();
    let mut bbox = Aabb::EMPTY;

    for g in geoms {
        collect_geometry_tris(g, &mut tris, &mut bbox);
    }
    if tris.is_empty() || bbox.is_empty() {
        return Err(GeomError::Degenerate(
            "shrinkwrap requires at least one non-empty geometry",
        ));
    }

    let diag = bbox.size().length().max(1e-3);
    let mut h = opts.voxel_size.max(diag / 250.0).max(1e-4);
    let pad = opts.offset.abs() + h * 3.5;
    let bmin = bbox.min - DVec3::splat(pad);
    let bmax = bbox.max + DVec3::splat(pad);
    let span = bmax - bmin;

    let max_dim = opts.max_grid_dim.clamp(12, 128) as f64;
    let longest = span.max_element();
    if longest / h > max_dim {
        h = longest / max_dim;
    }

    let nx = ((span.x / h).ceil() as usize).max(6);
    let ny = ((span.y / h).ceil() as usize).max(6);
    let nz = ((span.z / h).ceil() as usize).max(6);

    let vx_cnt = nx + 1;
    let vy_cnt = ny + 1;
    let vz_cnt = nz + 1;
    let total_verts = vx_cnt * vy_cnt * vz_cnt;

    let idx = |ix: usize, iy: usize, iz: usize| -> usize { (ix * vy_cnt + iy) * vz_cnt + iz };

    let shell_band = h * 0.92;
    let splat_radius = opts.offset.abs() + shell_band + h * 1.75;
    let mut udf = vec![f64::INFINITY; total_verts];

    // Narrow-band splatting around each triangle for fast exact distance evaluation
    for t in &tris {
        let min_gx = (((t.min.x - splat_radius - bmin.x) / h).floor() as isize).clamp(0, nx as isize)
            as usize;
        let max_gx = (((t.max.x + splat_radius - bmin.x) / h).ceil() as isize).clamp(0, nx as isize)
            as usize;
        let min_gy = (((t.min.y - splat_radius - bmin.y) / h).floor() as isize).clamp(0, ny as isize)
            as usize;
        let max_gy = (((t.max.y + splat_radius - bmin.y) / h).ceil() as isize).clamp(0, ny as isize)
            as usize;
        let min_gz = (((t.min.z - splat_radius - bmin.z) / h).floor() as isize).clamp(0, nz as isize)
            as usize;
        let max_gz = (((t.max.z + splat_radius - bmin.z) / h).ceil() as isize).clamp(0, nz as isize)
            as usize;

        for ix in min_gx..=max_gx {
            let px = bmin.x + (ix as f64) * h;
            for iy in min_gy..=max_gy {
                let py = bmin.y + (iy as f64) * h;
                for iz in min_gz..=max_gz {
                    let pz = bmin.z + (iz as f64) * h;
                    let d = point_triangle_distance_sq(DVec3::new(px, py, pz), t.a, t.b, t.c).sqrt();
                    let vid = idx(ix, iy, iz);
                    if d < udf[vid] {
                        udf[vid] = d;
                    }
                }
            }
        }
    }

    // Flood-fill exterior starting from (0,0,0) through vertices with udf > barrier
    let barrier = if opts.fill_holes {
        shell_band
    } else {
        h * 0.55
    };
    let mut is_exterior = vec![false; total_verts];
    let mut queue = VecDeque::new();
    is_exterior[0] = true;
    queue.push_back((0usize, 0usize, 0usize));

    while let Some((ix, iy, iz)) = queue.pop_front() {
        let neighbors = [
            (ix.wrapping_sub(1), iy, iz, ix > 0),
            (ix + 1, iy, iz, ix + 1 < vx_cnt),
            (ix, iy.wrapping_sub(1), iz, iy > 0),
            (ix, iy + 1, iz, iy + 1 < vy_cnt),
            (ix, iy, iz.wrapping_sub(1), iz > 0),
            (ix, iy, iz + 1, iz + 1 < vz_cnt),
        ];
        for (nx_i, ny_i, nz_i, valid) in neighbors {
            if !valid {
                continue;
            }
            let nvid = idx(nx_i, ny_i, nz_i);
            if !is_exterior[nvid] && udf[nvid] > barrier {
                is_exterior[nvid] = true;
                queue.push_back((nx_i, ny_i, nz_i));
            }
        }
    }

    let has_interior = is_exterior.iter().any(|&ext| !ext);
    let mut sdf = vec![0.0_f64; total_verts];
    for i in 0..total_verts {
        let d = if udf[i].is_finite() { udf[i] } else { h * 8.0 };
        if has_interior {
            let signed = if is_exterior[i] { d } else { -d };
            sdf[i] = signed - opts.offset;
        } else {
            // Open surface or curve: thicken into closed watertight wrapper shell
            let thick = opts.offset.max(h * 0.65);
            sdf[i] = d - thick;
        }
    }

    // Guarantee outer grid boundary remains strictly positive so the extracted mesh is closed
    for ix in 0..vx_cnt {
        for iy in 0..vy_cnt {
            for iz in 0..vz_cnt {
                if ix == 0 || ix == nx || iy == 0 || iy == ny || iz == 0 || iz == nz {
                    let vid = idx(ix, iy, iz);
                    if sdf[vid] <= 0.0 {
                        sdf[vid] = 1e-3 * h;
                    }
                }
            }
        }
    }

    // Marching Tetrahedra (6-tet symmetric cube decomposition -> guaranteed 2-manifold)
    let mut edge_vert_map: HashMap<(usize, usize), u32> = HashMap::new();
    let mut vertices: Vec<DVec3> = Vec::new();
    let mut pending_tris: Vec<([u32; 3], DVec3)> = Vec::new();

    let grid_pos = |vid: usize| -> DVec3 {
        let ix = vid / (vy_cnt * vz_cnt);
        let rem = vid % (vy_cnt * vz_cnt);
        let iy = rem / vz_cnt;
        let iz = rem % vz_cnt;
        bmin + DVec3::new(ix as f64 * h, iy as f64 * h, iz as f64 * h)
    };

    let mut interp_edge = |v0: usize, v1: usize| -> u32 {
        let key = if v0 < v1 { (v0, v1) } else { (v1, v0) };
        if let Some(&existing) = edge_vert_map.get(&key) {
            return existing;
        }
        let s0 = sdf[key.0];
        let s1 = sdf[key.1];
        let denom = s0 - s1;
        let t = if denom.abs() < 1e-12 {
            0.5
        } else {
            (s0 / denom).clamp(0.02, 0.98)
        };
        let p = grid_pos(key.0).lerp(grid_pos(key.1), t);
        let vid = vertices.len() as u32;
        vertices.push(p);
        edge_vert_map.insert(key, vid);
        vid
    };

    // 6-tet positive-volume permutations splitting each grid cell along (0,0,0)->(1,1,1)
    const TET_CORNERS: [[usize; 4]; 6] = [
        [0, 1, 2, 6],
        [0, 5, 1, 6],
        [0, 3, 7, 6],
        [0, 2, 3, 6],
        [0, 4, 5, 6],
        [0, 7, 4, 6],
    ];

    for ix in 0..nx {
        for iy in 0..ny {
            for iz in 0..nz {
                let c = [
                    idx(ix, iy, iz),             // 0: (0,0,0)
                    idx(ix + 1, iy, iz),         // 1: (1,0,0)
                    idx(ix + 1, iy + 1, iz),     // 2: (1,1,0)
                    idx(ix, iy + 1, iz),         // 3: (0,1,0)
                    idx(ix, iy, iz + 1),         // 4: (0,0,1)
                    idx(ix + 1, iy, iz + 1),     // 5: (1,0,1)
                    idx(ix + 1, iy + 1, iz + 1), // 6: (1,1,1)
                    idx(ix, iy + 1, iz + 1),     // 7: (0,1,1)
                ];

                for tc in &TET_CORNERS {
                    let t = [c[tc[0]], c[tc[1]], c[tc[2]], c[tc[3]]];
                    let mut inside = Vec::with_capacity(4);
                    let mut outside = Vec::with_capacity(4);
                    for &tv in &t {
                        if sdf[tv] <= 0.0 {
                            inside.push(tv);
                        } else {
                            outside.push(tv);
                        }
                    }
                    if inside.is_empty() || outside.is_empty() {
                        continue;
                    }

                    let in_avg =
                        inside.iter().map(|&v| grid_pos(v)).sum::<DVec3>() / inside.len() as f64;
                    let out_avg =
                        outside.iter().map(|&v| grid_pos(v)).sum::<DVec3>() / outside.len() as f64;
                    let outward_hint = out_avg - in_avg;

                    match (inside.len(), outside.len()) {
                        (1, 3) => {
                            let i0 = inside[0];
                            let v0 = interp_edge(i0, outside[0]);
                            let v1 = interp_edge(i0, outside[1]);
                            let v2 = interp_edge(i0, outside[2]);
                            pending_tris.push(([v0, v1, v2], outward_hint));
                        }
                        (3, 1) => {
                            let o0 = outside[0];
                            let v0 = interp_edge(inside[0], o0);
                            let v1 = interp_edge(inside[1], o0);
                            let v2 = interp_edge(inside[2], o0);
                            pending_tris.push(([v0, v1, v2], outward_hint));
                        }
                        (2, 2) => {
                            let v0 = interp_edge(inside[0], outside[0]);
                            let v1 = interp_edge(inside[0], outside[1]);
                            let v2 = interp_edge(inside[1], outside[1]);
                            let v3 = interp_edge(inside[1], outside[0]);
                            pending_tris.push(([v0, v1, v2], outward_hint));
                            pending_tris.push(([v0, v2, v3], outward_hint));
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    let mut indices = Vec::with_capacity(pending_tris.len());
    for ([a, b, c], outward_hint) in pending_tris {
        if a == b || b == c || c == a {
            continue;
        }
        let pa = vertices[a as usize];
        let pb = vertices[b as usize];
        let pc = vertices[c as usize];
        let n = (pb - pa).cross(pc - pa);
        if n.dot(outward_hint) >= 0.0 {
            indices.push([a, b, c]);
        } else {
            indices.push([a, c, b]);
        }
    }

    if vertices.is_empty() || indices.is_empty() {
        return Err(GeomError::Degenerate("shrinkwrap produced an empty mesh"));
    }

    // Taubin (lambda / mu) smoothing preserves volume while eliminating voxel faceting
    let smooth_iters = opts.smoothing_iterations.min(20);
    if smooth_iters > 0 {
        taubin_smooth(&mut vertices, &indices, smooth_iters, 0.5, -0.53);
    }

    // Area-weighted vertex normals
    let mut norm_acc = vec![DVec3::ZERO; vertices.len()];
    for &[a, b, c] in &indices {
        let pa = vertices[a as usize];
        let pb = vertices[b as usize];
        let pc = vertices[c as usize];
        let fn_ = (pb - pa).cross(pc - pa);
        norm_acc[a as usize] += fn_;
        norm_acc[b as usize] += fn_;
        norm_acc[c as usize] += fn_;
    }

    Ok(TriangleMesh {
        positions: vertices.into_iter().map(|v| v.to_array()).collect(),
        normals: norm_acc
            .into_iter()
            .map(|n| n.normalize_or(DVec3::Z).to_array())
            .collect(),
        indices,
    })
}

fn taubin_smooth(
    vertices: &mut [DVec3],
    indices: &[[u32; 3]],
    iterations: usize,
    lambda: f64,
    mu: f64,
) {
    let nv = vertices.len();
    let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); nv];
    for &[a, b, c] in indices {
        let [a, b, c] = [a as usize, b as usize, c as usize];
        for (u, v) in [(a, b), (b, c), (c, a)] {
            if !neighbors[u].contains(&v) {
                neighbors[u].push(v);
            }
            if !neighbors[v].contains(&u) {
                neighbors[v].push(u);
            }
        }
    }

    let mut next_pos = vertices.to_vec();
    for _ in 0..iterations {
        for &factor in &[lambda, mu] {
            for (i, nbrs) in neighbors.iter().enumerate() {
                if nbrs.is_empty() {
                    next_pos[i] = vertices[i];
                    continue;
                }
                let avg =
                    nbrs.iter().map(|&j| vertices[j]).sum::<DVec3>() / (nbrs.len() as f64);
                next_pos[i] = vertices[i] + (avg - vertices[i]) * factor;
            }
            vertices.copy_from_slice(&next_pos);
        }
    }
}

fn collect_geometry_tris(geom: &Geometry, out: &mut Vec<TriData>, bbox: &mut Aabb) {
    let push_mesh = |m: &TriangleMesh, out: &mut Vec<TriData>, bbox: &mut Aabb| {
        for &[i0, i1, i2] in &m.indices {
            let a = DVec3::from_array(m.positions[i0 as usize]);
            let b = DVec3::from_array(m.positions[i1 as usize]);
            let c = DVec3::from_array(m.positions[i2 as usize]);
            if (b - a).cross(c - a).length_squared() < 1e-18 {
                continue;
            }
            let min = a.min(b).min(c);
            let max = a.max(b).max(c);
            *bbox = bbox.union(&Aabb { min, max });
            out.push(TriData { a, b, c, min, max });
        }
    };

    match geom {
        Geometry::Mesh(m) => push_mesh(m, out, bbox),
        Geometry::Surface(s) => {
            let m = s.tessellate_display();
            push_mesh(&m, out, bbox);
        }
        Geometry::BRep(b) => {
            let m = b.tessellate_display();
            push_mesh(&m, out, bbox);
        }
        Geometry::SubD(sd) => {
            let m = sd.tessellate_display();
            push_mesh(&m, out, bbox);
        }
        Geometry::Curve(c) => {
            let pts = c.sample_polyline(32);
            let r = (c.bounding_box().size().length() * 0.02).max(0.08);
            for w in pts.windows(2) {
                let p0 = w[0];
                let p1 = w[1];
                let dir = (p1 - p0).normalize_or(DVec3::X);
                let up = if dir.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
                let u = dir.cross(up).normalize() * r;
                let v = dir.cross(u).normalize() * r;
                let ring0 = [p0 + u, p0 + v, p0 - u, p0 - v];
                let ring1 = [p1 + u, p1 + v, p1 - u, p1 - v];
                for k in 0..4 {
                    let nk = (k + 1) % 4;
                    let tris = [[ring0[k], ring0[nk], ring1[nk]], [ring0[k], ring1[nk], ring1[k]]];
                    for [a, b, c] in tris {
                        let min = a.min(b).min(c);
                        let max = a.max(b).max(c);
                        *bbox = bbox.union(&Aabb { min, max });
                        out.push(TriData { a, b, c, min, max });
                    }
                }
            }
        }
        Geometry::Point(p) => {
            let m = TriangleMesh::from_box(*p, DVec3::splat(0.25));
            push_mesh(&m, out, bbox);
        }
    }
}

/// Exact squared Euclidean distance from point `p` to triangle `(a, b, c)`.
fn point_triangle_distance_sq(p: DVec3, a: DVec3, b: DVec3, c: DVec3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return p.distance_squared(a);
    }

    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return p.distance_squared(b);
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return p.distance_squared(a + ab * v);
    }

    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return p.distance_squared(c);
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return p.distance_squared(a + ac * w);
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return p.distance_squared(b + (c - b) * w);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    p.distance_squared(a + ab * v + ac * w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analysis::compute_mesh_mass_properties;

    #[test]
    fn shrinkwrap_unions_two_boxes_watertight() {
        let m1 = Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0)));
        let m2 = Geometry::Mesh(TriangleMesh::from_box(
            DVec3::new(1.2, 0.0, 0.0),
            DVec3::splat(2.0),
        ));
        let wrapped = shrinkwrap_geometries(
            &[&m1, &m2],
            &ShrinkWrapOptions {
                voxel_size: 0.2,
                offset: 0.05,
                smoothing_iterations: 3,
                fill_holes: true,
                max_grid_dim: 48,
            },
        )
        .expect("shrinkwrap succeeds");

        assert!(!wrapped.positions.is_empty());
        assert!(!wrapped.indices.is_empty());

        // Strictly closed 2-manifold edge check: every undirected edge shared by exact 2 triangles
        let mut edge_counts: HashMap<[u32; 2], usize> = HashMap::new();
        for &[i0, i1, i2] in &wrapped.indices {
            for (u, v) in [(i0, i1), (i1, i2), (i2, i0)] {
                let k = if u < v { [u, v] } else { [v, u] };
                *edge_counts.entry(k).or_insert(0) += 1;
            }
        }
        assert!(
            edge_counts.values().all(|&c| c == 2),
            "shrinkwrap mesh must be closed 2-manifold"
        );

        let mp = compute_mesh_mass_properties(&wrapped, true);
        assert!(mp.is_closed_solid);
        assert!(mp.volume > 8.0 && mp.volume < 20.0, "got vol {}", mp.volume);
    }
}
