use std::cell::RefCell;
use std::rc::Rc;

use cad_doc::Document;
use cad_geom::{
    BRep, BooleanOp, DVec3, Geometry, LoftStyle, NurbsCurve, NurbsSurface, Plane,
    ShrinkWrapOptions, SubDMesh, TriangleMesh, brep_boolean, loft_nurbs, mesh_boolean,
    revolve_nurbs, shrinkwrap_geometries, subd_to_nurbs_brep,
};
use rhai::{Array, Engine};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ScriptError {
    #[error("script error: {0}")]
    Runtime(String),
}

#[derive(Debug, Default)]
pub struct ScriptResult {
    pub created_count: usize,
    pub cleared: bool,
    pub logs: Vec<String>,
}

/// Execute a local Rhai CAD script against `doc` in a bounded sandbox (max 250k ops).
pub fn run_rhai_script(source: &str, doc: &mut Document) -> Result<ScriptResult, ScriptError> {
    let pending: Rc<RefCell<Vec<(String, Geometry)>>> = Rc::new(RefCell::new(Vec::new()));
    let logs: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    let clear_flag: Rc<RefCell<bool>> = Rc::new(RefCell::new(false));

    let mut engine = Engine::new();
    engine.set_max_operations(250_000);

    // Primitives & curves
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_box", move |cx: f64, cy: f64, cz: f64, sx: f64, sy: f64, sz: f64| {
            let mesh = TriangleMesh::from_box(DVec3::new(cx, cy, cz), DVec3::new(sx, sy, sz));
            p.borrow_mut().push(("ScriptBox".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_nurbs_box",
            move |cx: f64, cy: f64, cz: f64, sx: f64, sy: f64, sz: f64| {
                let brep = BRep::from_box(DVec3::new(cx, cy, cz), DVec3::new(sx, sy, sz));
                p.borrow_mut()
                    .push(("ScriptNurbsBox".to_string(), Geometry::BRep(brep)));
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_sphere", move |cx: f64, cy: f64, cz: f64, r: f64| {
            let mesh = TriangleMesh::from_sphere(DVec3::new(cx, cy, cz), r.max(0.01), 24, 16);
            p.borrow_mut()
                .push(("ScriptSphere".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_nurbs_sphere", move |cx: f64, cy: f64, cz: f64, r: f64| {
            if let Ok(s) = NurbsSurface::sphere(DVec3::new(cx, cy, cz), r.max(0.01)) {
                p.borrow_mut()
                    .push(("ScriptNurbsSphere".to_string(), Geometry::Surface(s)));
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_circle", move |cx: f64, cy: f64, cz: f64, r: f64| {
            let mut plane = Plane::world_xy();
            plane.origin = DVec3::new(cx, cy, cz);
            if let Ok(c) = NurbsCurve::circle(&plane, r.max(0.01)) {
                p.borrow_mut()
                    .push(("ScriptCircle".to_string(), Geometry::Curve(c)));
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_line",
            move |x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64| {
                let c = NurbsCurve::line(DVec3::new(x0, y0, z0), DVec3::new(x1, y1, z1));
                p.borrow_mut()
                    .push(("ScriptLine".to_string(), Geometry::Curve(c)));
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_helix",
            move |cx: f64, cy: f64, cz: f64, r: f64, pitch: f64, turns: f64| {
                let base = DVec3::new(cx, cy, cz);
                let top = base + DVec3::Z * (pitch * turns.max(0.25));
                let start_pt = base + DVec3::X * r.max(0.05);
                if let Ok(h) = NurbsCurve::helix(base, top, start_pt, turns.max(0.25), None) {
                    p.borrow_mut()
                        .push(("ScriptHelix".to_string(), Geometry::Curve(h)));
                }
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_interp_curve", move |pts: Array| {
            let parsed: Vec<DVec3> = pts
                .into_iter()
                .filter_map(|item| {
                    let arr = item.try_cast::<Array>()?;
                    if arr.len() >= 3 {
                        let x = arr[0].as_float().ok()?;
                        let y = arr[1].as_float().ok()?;
                        let z = arr[2].as_float().ok()?;
                        Some(DVec3::new(x, y, z))
                    } else {
                        None
                    }
                })
                .collect();
            if parsed.len() >= 2 {
                let deg = parsed.len().saturating_sub(1).min(3).max(1);
                if let Ok(c) = NurbsCurve::interpolate(&parsed, deg, false) {
                    p.borrow_mut()
                        .push(("ScriptInterpCrv".to_string(), Geometry::Curve(c)));
                }
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_cylinder", move |cx: f64, cy: f64, cz: f64, r: f64, h: f64| {
            let mesh = TriangleMesh::from_cylinder(DVec3::new(cx, cy, cz), r.max(0.01), h, 32);
            p.borrow_mut()
                .push(("ScriptCylinder".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_nurbs_cylinder",
            move |cx: f64, cy: f64, cz: f64, r: f64, h: f64| {
                if let Ok(b) = BRep::from_cylinder(DVec3::new(cx, cy, cz), r.max(0.01), h) {
                    p.borrow_mut()
                        .push(("ScriptNurbsCyl".to_string(), Geometry::BRep(b)));
                }
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_cone", move |cx: f64, cy: f64, cz: f64, r: f64, h: f64| {
            let mesh = TriangleMesh::from_cone(DVec3::new(cx, cy, cz), r.max(0.01), h, 32);
            p.borrow_mut()
                .push(("ScriptCone".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_torus",
            move |cx: f64, cy: f64, cz: f64, maj: f64, min: f64| {
                let mesh = TriangleMesh::from_torus(
                    DVec3::new(cx, cy, cz),
                    maj.max(0.02),
                    min.max(0.01),
                    32,
                    16,
                );
                p.borrow_mut()
                    .push(("ScriptTorus".to_string(), Geometry::Mesh(mesh)));
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_rectangle",
            move |x0: f64, y0: f64, z0: f64, x1: f64, y1: f64| {
                let mut plane = Plane::world_xy();
                plane.origin = DVec3::new(x0, y0, z0);
                if let Ok(rect) =
                    NurbsCurve::rectangle(&plane, DVec3::new(x0, y0, z0), DVec3::new(x1, y1, z0))
                {
                    p.borrow_mut()
                        .push(("ScriptRect".to_string(), Geometry::Curve(rect)));
                }
            },
        );
    }

    // SubD & Hybrid Organic CAD
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_subd_box",
            move |cx: f64, cy: f64, cz: f64, sx: f64, sy: f64, sz: f64, level: i64| {
                let mut sd = SubDMesh::from_box(DVec3::new(cx, cy, cz), DVec3::new(sx, sy, sz));
                sd.subdivision_level = level.clamp(1, 4) as u8;
                p.borrow_mut()
                    .push(("ScriptSubDBox".to_string(), Geometry::SubD(sd)));
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "add_subd_sphere",
            move |cx: f64, cy: f64, cz: f64, r: f64, level: i64| {
                let mut sd = SubDMesh::from_sphere(DVec3::new(cx, cy, cz), r.max(0.01));
                sd.subdivision_level = level.clamp(1, 4) as u8;
                p.borrow_mut()
                    .push(("ScriptSubDSphere".to_string(), Geometry::SubD(sd)));
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("subd_to_nurbs_last", move || {
            let mut list = p.borrow_mut();
            if let Some((_, Geometry::SubD(sd))) = list.last() {
                if let Ok(brep) = subd_to_nurbs_brep(sd) {
                    list.pop();
                    list.push(("ScriptToNURBS".to_string(), Geometry::BRep(brep)));
                }
            }
        });
    }

    // Surfacing & Solid operations on pending stack
    {
        let p = Rc::clone(&pending);
        engine.register_fn("extrude_last", move |dx: f64, dy: f64, dz: f64| {
            let mut list = p.borrow_mut();
            if let Some((_, Geometry::Curve(c))) = list.last().cloned() {
                list.pop();
                let dir = DVec3::new(dx, dy, dz);
                if c.is_closed() {
                    if let Ok(b) = BRep::extrude_curve_solid(&c, dir) {
                        list.push(("ScriptExtrude".to_string(), Geometry::BRep(b)));
                        return;
                    }
                }
                list.push((
                    "ScriptExtrude".to_string(),
                    Geometry::Surface(NurbsSurface::extrude_curve(&c, dir)),
                ));
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("loft_last_n", move |n: i64| {
            let mut list = p.borrow_mut();
            let count = (n.max(2) as usize).min(list.len());
            if count < 2 {
                return;
            }
            let start = list.len() - count;
            let curves: Vec<NurbsCurve> = list[start..]
                .iter()
                .filter_map(|(_, g)| match g {
                    Geometry::Curve(c) => Some(c.clone()),
                    _ => None,
                })
                .collect();
            if curves.len() >= 2 {
                if let Ok(s) = loft_nurbs(&curves, LoftStyle::Normal, false) {
                    list.truncate(start);
                    list.push(("ScriptLoft".to_string(), Geometry::Surface(s)));
                }
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("revolve_last", move |angle_deg: f64| {
            let mut list = p.borrow_mut();
            if let Some((_, Geometry::Curve(c))) = list.last().cloned() {
                if let Ok(s) =
                    revolve_nurbs(&c, DVec3::ZERO, DVec3::Z, angle_deg.to_radians())
                {
                    list.pop();
                    list.push(("ScriptRevolve".to_string(), Geometry::Surface(s)));
                }
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("boolean_union_last", move || {
            apply_boolean_last(&mut p.borrow_mut(), BooleanOp::Union);
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("boolean_diff_last", move || {
            apply_boolean_last(&mut p.borrow_mut(), BooleanOp::Difference);
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn(
            "shrinkwrap_all",
            move |voxel_size: f64, offset: f64, smooth: i64| {
                let mut list = p.borrow_mut();
                if list.is_empty() {
                    return;
                }
                let refs: Vec<&Geometry> = list.iter().map(|(_, g)| g).collect();
                if let Ok(mesh) = shrinkwrap_geometries(
                    &refs,
                    &ShrinkWrapOptions {
                        voxel_size: voxel_size.max(0.05),
                        offset,
                        smoothing_iterations: smooth.clamp(0, 12) as usize,
                        fill_holes: true,
                        max_grid_dim: 48,
                    },
                ) {
                    list.clear();
                    list.push(("ScriptShrinkWrap".to_string(), Geometry::Mesh(mesh)));
                }
            },
        );
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("object_count", move || -> i64 { p.borrow().len() as i64 });
    }
    {
        let l = Rc::clone(&logs);
        engine.register_fn("log", move |msg: &str| {
            l.borrow_mut().push(msg.to_string());
        });
    }
    {
        let cf = Rc::clone(&clear_flag);
        let p = Rc::clone(&pending);
        engine.register_fn("clear_doc", move || {
            *cf.borrow_mut() = true;
            p.borrow_mut().clear();
        });
    }

    engine
        .run(source)
        .map_err(|e| ScriptError::Runtime(e.to_string()))?;

    let cleared = *clear_flag.borrow();
    if cleared {
        *doc = Document::new();
    }

    let items = pending.borrow();
    let created_count = items.len();
    for (label, geom) in items.iter() {
        doc.add_object(label.clone(), geom.clone());
    }

    Ok(ScriptResult {
        created_count,
        cleared,
        logs: logs.borrow().clone(),
    })
}

fn apply_boolean_last(list: &mut Vec<(String, Geometry)>, op: BooleanOp) {
    if list.len() < 2 {
        return;
    }
    let (_, b) = list.pop().unwrap();
    let (lbl, a) = list.pop().unwrap();
    let res = match (&a, &b) {
        (Geometry::BRep(ba), Geometry::BRep(bb)) => brep_boolean(ba, bb, op, 1e-4)
            .ok()
            .map(Geometry::BRep),
        _ => {
            let to_m = |g: &Geometry| match g {
                Geometry::Mesh(m) => Some(m.clone()),
                Geometry::Surface(s) => Some(s.tessellate_display()),
                Geometry::BRep(b) => Some(b.tessellate_display()),
                Geometry::SubD(sd) => Some(sd.tessellate_display()),
                _ => None,
            };
            if let (Some(ma), Some(mb)) = (to_m(&a), to_m(&b)) {
                Some(Geometry::Mesh(mesh_boolean(&ma, &mb, op)))
            } else {
                None
            }
        }
    };
    if let Some(geom) = res {
        list.push(("ScriptBoolean".to_string(), geom));
    } else {
        list.push((lbl, a));
        list.push(("Item".to_string(), b));
    }
}

pub const DEFAULT_SAMPLE_SCRIPT: &str = r#"// Elephant Rhai Script (100% local)
// Creates a parametric spiral of spheres
let count = 12;
for i in 0..count {
    let angle = i * 0.52;
    let r = 2.0 + i * 0.35;
    let x = r * cos(angle);
    let y = r * sin(angle);
    let z = 0.5 + i * 0.25;
    add_sphere(x, y, z, 0.35);
}
log("Generated spiral of 12 spheres.");
"#;

pub const SCRIPT_PRESETS: &[(&str, &str)] = &[
    ("Spiral Spheres", DEFAULT_SAMPLE_SCRIPT),
    (
        "Lofted Organic Vessel",
        r#"// Multi-section NURBS Loft vessel
for i in 0..4 {
    let z = i * 1.4;
    let r = 2.2 - 0.6 * sin(i * 0.9);
    add_circle(0.0, 0.0, z, r);
}
loft_last_n(4);
log("Lofted 4 circular sections into NURBS vessel.");
"#,
    ),
    (
        "SubD Cushion -> ToNURBS",
        r#"// Create Catmull-Clark SubD body and convert directly to B-Rep NURBS
add_subd_box(0.0, 0.0, 1.5, 4.0, 3.0, 2.0, 2);
subd_to_nurbs_last();
log("Converted SubD cage into stitched bicubic B-Rep.");
"#,
    ),
    (
        "ShrinkWrap Solid Cluster",
        r#"// Build overlapping primitives and ShrinkWrap into one closed 3D-printable mesh
add_box(0.0, 0.0, 1.0, 2.5, 2.5, 2.0);
add_sphere(1.2, 0.0, 1.2, 1.4);
add_cylinder(-1.0, 0.5, 0.0, 0.9, 2.8);
shrinkwrap_all(0.22, 0.05, 4);
log("ShrinkWrapped cluster into watertight printable mesh.");
"#,
    ),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn script_creates_spiral_spheres() {
        let mut doc = Document::new();
        let res = run_rhai_script(DEFAULT_SAMPLE_SCRIPT, &mut doc).unwrap();
        assert_eq!(res.created_count, 12);
        assert_eq!(doc.objects.len(), 12);
        assert_eq!(res.logs, vec!["Generated spiral of 12 spheres."]);
    }

    #[test]
    fn rhai_script_nurbs_subd_and_shrinkwrap() {
        let mut doc = Document::new();
        let src = r#"
            add_nurbs_box(-4.0, 0.0, 1.0, 2.0, 2.0, 2.0);
            add_subd_box(0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 1);
            subd_to_nurbs_last();
            add_subd_sphere(4.0, 0.0, 1.0, 1.2, 2);
            log("Built mixed NURBS + SubD scene");
        "#;
        let res = run_rhai_script(src, &mut doc).unwrap();
        assert_eq!(res.created_count, 3);
        assert_eq!(doc.objects.len(), 3);

        let mut doc_wrap = Document::new();
        let wrap_res = run_rhai_script(SCRIPT_PRESETS[3].1, &mut doc_wrap).unwrap();
        assert_eq!(wrap_res.created_count, 1);
        assert!(matches!(
            doc_wrap.objects.values().next().unwrap().geometry,
            Geometry::Mesh(_)
        ));
    }
}
