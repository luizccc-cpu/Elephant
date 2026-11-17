use std::cell::RefCell;
use std::rc::Rc;

use cad_doc::Document;
use cad_geom::{DVec3, Geometry, NurbsCurve, Plane, TriangleMesh};
use rhai::Engine;
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

/// Execute a local Rhai CAD script against `doc` in a bounded sandbox (max 100k ops).
pub fn run_rhai_script(source: &str, doc: &mut Document) -> Result<ScriptResult, ScriptError> {
    let pending: Rc<RefCell<Vec<(String, Geometry)>>> = Rc::new(RefCell::new(Vec::new()));
    let logs: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    let clear_flag: Rc<RefCell<bool>> = Rc::new(RefCell::new(false));

    let mut engine = Engine::new();
    engine.set_max_operations(100_000);

    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_box", move |cx: f64, cy: f64, cz: f64, sx: f64, sy: f64, sz: f64| {
            let mesh = TriangleMesh::from_box(DVec3::new(cx, cy, cz), DVec3::new(sx, sy, sz));
            p.borrow_mut().push(("ScriptBox".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_sphere", move |cx: f64, cy: f64, cz: f64, r: f64| {
            let mesh = TriangleMesh::from_sphere(DVec3::new(cx, cy, cz), r.max(0.01), 24, 16);
            p.borrow_mut().push(("ScriptSphere".to_string(), Geometry::Mesh(mesh)));
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_circle", move |cx: f64, cy: f64, cz: f64, r: f64| {
            let mut plane = Plane::world_xy();
            plane.origin = DVec3::new(cx, cy, cz);
            if let Ok(c) = NurbsCurve::circle(&plane, r.max(0.01)) {
                p.borrow_mut().push(("ScriptCircle".to_string(), Geometry::Curve(c)));
            }
        });
    }
    {
        let p = Rc::clone(&pending);
        engine.register_fn("add_line", move |x0: f64, y0: f64, z0: f64, x1: f64, y1: f64, z1: f64| {
            let c = NurbsCurve::line(DVec3::new(x0, y0, z0), DVec3::new(x1, y1, z1));
            p.borrow_mut().push(("ScriptLine".to_string(), Geometry::Curve(c)));
        });
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
}
