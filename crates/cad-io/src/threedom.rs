use std::io::Cursor;

use cad_doc::Document;
use cad_geom::{ControlPoint, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, TriangleMesh};
use cadmpeg_codec_rhino::RhinoCodec;
use cadmpeg_ir::codec::{Codec, DecodeOptions};
use cadmpeg_ir::geometry::nurbs::{NurbsPoleGrid, NurbsPoles3};
use cadmpeg_ir::geometry::{CurveGeometry, SolvedCurveGeometry, SolvedSurfaceGeometry, SurfaceGeometry};

use crate::error::IoError;

/// Import geometry from Rhino `.3dm` bytes (V5–V8) into `doc`.
pub fn import_3dm_bytes(bytes: &[u8], doc: &mut Document) -> Result<usize, IoError> {
    let decoded = RhinoCodec
        .decode(&mut Cursor::new(bytes), &DecodeOptions::default())
        .map_err(|e| IoError::InvalidFormat(format!("3DM decode failed: {e}")))?;

    let ir = decoded.ir();
    let mut added = 0usize;

    // 1. Import tessellated meshes (covers Mesh objects and B-Rep display meshes)
    for tess in &ir.model.tessellations {
        let verts = tess.vertices();
        let tris = tess.triangles();
        if verts.is_empty() || tris.is_empty() {
            continue;
        }
        let positions: Vec<[f64; 3]> = verts.iter().map(|p| p.get().into()).collect();
        let mut normals = vec![[0.0, 0.0, 1.0]; positions.len()];
        let mut accum = vec![DVec3::ZERO; positions.len()];
        for &[i0, i1, i2] in &tris {
            let p0 = DVec3::from_array(positions[i0 as usize]);
            let p1 = DVec3::from_array(positions[i1 as usize]);
            let p2 = DVec3::from_array(positions[i2 as usize]);
            let fn_ = (p1 - p0).cross(p2 - p0);
            accum[i0 as usize] += fn_;
            accum[i1 as usize] += fn_;
            accum[i2 as usize] += fn_;
        }
        for (i, a) in accum.into_iter().enumerate() {
            let n = a.normalize_or_zero();
            normals[i] = if n.length_squared() > 0.0 {
                n.to_array()
            } else {
                [0.0, 0.0, 1.0]
            };
        }
        let mesh = TriangleMesh {
            positions,
            normals,
            indices: tris,
        };
        let id = doc.add_object("Import3DM_Mesh", Geometry::Mesh(mesh));
        if let Some(obj) = doc.objects.get_mut(id) {
            obj.name = tess
                .source_object
                .as_ref()
                .and_then(|s| s.name.as_ref().map(|n| n.to_string()));
        }
        added += 1;
    }

    // 2. Import standalone NURBS/polyline/circle curves when not purely internal B-rep edges
    let has_tess = !ir.model.tessellations.is_empty();
    if !has_tess || ir.model.bodies.is_empty() {
        for curve in &ir.model.curves {
            let solved = match &curve.geometry {
                CurveGeometry::Solved(s) => Some(s),
                CurveGeometry::Procedural { cache, .. } => cache.as_ref(),
            };
            if let Some(g) = solved.and_then(convert_curve) {
                doc.add_object("Import3DM_Curve", g);
                added += 1;
            }
        }
    }

    // 3. Import standalone NURBS surfaces when no tessellations were provided
    if !has_tess {
        for surf in &ir.model.surfaces {
            let solved = match &surf.geometry {
                SurfaceGeometry::Solved(s) => Some(s),
                SurfaceGeometry::Procedural { cache, .. } => cache.as_ref(),
            };
            if let Some(g) = solved.and_then(convert_surface) {
                doc.add_object("Import3DM_Surface", g);
                added += 1;
            }
        }
    }

    // 4. Import standalone points when no bodies/tessellations exist
    if ir.model.bodies.is_empty() && !has_tess {
        for pt in &ir.model.points {
            let p: [f64; 3] = pt.position().get().into();
            doc.add_object("Import3DM_Point", Geometry::Point(DVec3::from_array(p)));
            added += 1;
        }
    }

    Ok(added)
}

fn convert_curve(solved: &SolvedCurveGeometry) -> Option<Geometry> {
    match solved {
        SolvedCurveGeometry::Nurbs(nc) => {
            let (deg, knots, poles, _) = nc.clone().into_parts();
            let cps = match poles {
                NurbsPoles3::Polynomial { points } => points
                    .into_iter()
                    .map(|p| ControlPoint::unweighted(DVec3::from_array(p.get().into())))
                    .collect(),
                NurbsPoles3::Rational { points } => points
                    .into_iter()
                    .map(|p| {
                        ControlPoint::new(DVec3::from_array(p.point.get().into()), p.weight.get())
                    })
                    .collect(),
            };
            NurbsCurve::new(deg as usize, knots.as_slice().to_vec(), cps)
                .ok()
                .map(Geometry::Curve)
        }
        SolvedCurveGeometry::Polyline(pl) => {
            let pts: Vec<DVec3> = pl
                .points()
                .map(|p| DVec3::from_array(p.get().into()))
                .collect();
            NurbsCurve::polyline(&pts).ok().map(Geometry::Curve)
        }
        SolvedCurveGeometry::Circle(c) => {
            let center = DVec3::from_array(c.center().get().into());
            let mut plane = Plane::world_xy();
            plane.origin = center;
            NurbsCurve::circle(&plane, c.radius().get())
                .ok()
                .map(Geometry::Curve)
        }
        SolvedCurveGeometry::Transformed(placed) => {
            let mut g = convert_curve(placed.basis())?;
            let m = placed.transform().rows();
            let mat = cad_geom::DMat4::from_cols_array_2d(&m).transpose();
            g.transform(&mat);
            Some(g)
        }
        _ => None,
    }
}

fn convert_surface(solved: &SolvedSurfaceGeometry) -> Option<Geometry> {
    match solved {
        SolvedSurfaceGeometry::Nurbs(ns) => {
            let cps: Vec<ControlPoint> = match ns.pole_grid() {
                NurbsPoleGrid::Polynomial { rows } => rows
                    .iter()
                    .flat_map(|row| row.iter())
                    .map(|p| ControlPoint::unweighted(DVec3::from_array(p.get().into())))
                    .collect(),
                NurbsPoleGrid::Rational { rows } => rows
                    .iter()
                    .flat_map(|row| row.iter())
                    .map(|p| {
                        ControlPoint::new(DVec3::from_array(p.point.get().into()), p.weight.get())
                    })
                    .collect(),
            };
            NurbsSurface::new(
                ns.u_degree() as usize,
                ns.v_degree() as usize,
                ns.u_knots().as_slice().to_vec(),
                ns.v_knots().as_slice().to_vec(),
                ns.u_count(),
                ns.v_count(),
                cps,
            )
            .ok()
            .map(Geometry::Surface)
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_3dm_bytes() {
        let mut doc = Document::new();
        let err = import_3dm_bytes(b"not a 3dm file", &mut doc).unwrap_err();
        assert!(matches!(err, IoError::InvalidFormat(_)));
    }
}
