use cad_doc::Document;
use cad_geom::{
    BRep, BooleanOp, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, ShrinkWrapOptions, SubDMesh,
    brep_boolean, combine_mass_properties, compute_geometry_mass_properties, mesh_boolean,
    shrinkwrap_geometries, subd_to_nurbs_brep,
};
use cad_io::export_step_string;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::engine::run_rhai_script;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct McpToolDescriptor {
    pub name: String,
    pub description: String,
    pub input_schema: Value,
}

#[derive(Debug, Clone)]
pub struct McpDispatchOutput {
    pub response_json: String,
    pub mutated_document: bool,
}

/// Enumerate all CAD tools exposed via Elephant's local JSON-RPC 2.0 / MCP bridge.
pub fn mcp_tools_manifest() -> Vec<McpToolDescriptor> {
    vec![
        McpToolDescriptor {
            name: "inspect_document".to_string(),
            description: "List objects, layers, bounding box, and units in the active Elephant document.".to_string(),
            input_schema: json!({"type": "object", "properties": {}}),
        },
        McpToolDescriptor {
            name: "create_box".to_string(),
            description: "Create a solid NURBS B-Rep box centered at [cx, cy, cz] with extents [sx, sy, sz].".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "cx": {"type": "number"}, "cy": {"type": "number"}, "cz": {"type": "number"},
                    "sx": {"type": "number"}, "sy": {"type": "number"}, "sz": {"type": "number"}
                },
                "required": ["sx", "sy", "sz"]
            }),
        },
        McpToolDescriptor {
            name: "create_sphere".to_string(),
            description: "Create a NURBS sphere surface at [cx, cy, cz] with radius r.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "cx": {"type": "number"}, "cy": {"type": "number"}, "cz": {"type": "number"},
                    "r": {"type": "number"}
                },
                "required": ["r"]
            }),
        },
        McpToolDescriptor {
            name: "create_circle".to_string(),
            description: "Create an exact rational NURBS circle on XY at [cx, cy, cz] with radius r.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "cx": {"type": "number"}, "cy": {"type": "number"}, "cz": {"type": "number"},
                    "r": {"type": "number"}
                },
                "required": ["r"]
            }),
        },
        McpToolDescriptor {
            name: "create_subd_box".to_string(),
            description: "Create a Catmull-Clark SubD box cage at [cx, cy, cz] with extents [sx, sy, sz].".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "cx": {"type": "number"}, "cy": {"type": "number"}, "cz": {"type": "number"},
                    "sx": {"type": "number"}, "sy": {"type": "number"}, "sz": {"type": "number"},
                    "level": {"type": "integer"}
                },
                "required": ["sx", "sy", "sz"]
            }),
        },
        McpToolDescriptor {
            name: "create_subd_sphere".to_string(),
            description: "Create a smooth Catmull-Clark SubD sphere cage at [cx, cy, cz] with radius r.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "cx": {"type": "number"}, "cy": {"type": "number"}, "cz": {"type": "number"},
                    "r": {"type": "number"},
                    "level": {"type": "integer"}
                },
                "required": ["r"]
            }),
        },
        McpToolDescriptor {
            name: "subd_to_nurbs".to_string(),
            description: "Convert SubD objects (selected, or latest if none selected) into watertight stitched NURBS B-Reps (ToNURBS).".to_string(),
            input_schema: json!({"type": "object", "properties": {}}),
        },
        McpToolDescriptor {
            name: "shrinkwrap".to_string(),
            description: "Voxel-remesh visible (or selected) geometry into a closed 3D-printable watertight mesh (ShrinkWrap).".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "voxel_size": {"type": "number"},
                    "offset": {"type": "number"},
                    "smoothing_iterations": {"type": "integer"}
                }
            }),
        },
        McpToolDescriptor {
            name: "boolean_op".to_string(),
            description: "Run Boolean union, difference, or intersection on the two most recent solids/meshes.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "op": {"type": "string", "enum": ["union", "difference", "intersection"]}
                },
                "required": ["op"]
            }),
        },
        McpToolDescriptor {
            name: "compute_mass_properties".to_string(),
            description: "Compute surface area, volume, centroid, and inertia tensor of visible or selected geometry.".to_string(),
            input_schema: json!({"type": "object", "properties": {}}),
        },
        McpToolDescriptor {
            name: "run_rhai".to_string(),
            description: "Execute a bounded local Rhai CAD script against the document.".to_string(),
            input_schema: json!({
                "type": "object",
                "properties": {
                    "source": {"type": "string"}
                },
                "required": ["source"]
            }),
        },
        McpToolDescriptor {
            name: "export_step".to_string(),
            description: "Export current visible document geometry as an ISO-10303-21 STEP AP214 ASCII string.".to_string(),
            input_schema: json!({"type": "object", "properties": {}}),
        },
        McpToolDescriptor {
            name: "clear_document".to_string(),
            description: "Reset the active document to an empty scene.".to_string(),
            input_schema: json!({"type": "object", "properties": {}}),
        },
    ]
}

pub fn mcp_tools_manifest_json() -> String {
    serde_json::to_string_pretty(&mcp_tools_manifest()).unwrap_or_else(|_| "[]".to_string())
}

/// Dispatch a single local MCP / JSON-RPC 2.0 request (or direct `{tool, arguments}` call) against `doc`.
pub fn dispatch_mcp_json(request_json: &str, doc: &mut Document) -> McpDispatchOutput {
    let parsed: Value = match serde_json::from_str(request_json) {
        Ok(v) => v,
        Err(e) => {
            return McpDispatchOutput {
                response_json: rpc_err(Value::Null, -32700, &format!("Parse error: {e}")),
                mutated_document: false,
            };
        }
    };

    let id = parsed.get("id").cloned().unwrap_or(Value::from(1));

    // Support direct shorthand {"tool": "...", "arguments": {...}}
    if let Some(tool_name) = parsed.get("tool").and_then(|v| v.as_str()) {
        let args = parsed.get("arguments").cloned().unwrap_or_else(|| json!({}));
        let (res, mutated) = execute_tool(tool_name, &args, doc);
        let response_json = match res {
            Ok(payload) => serde_json::to_string_pretty(&json!({
                "jsonrpc": "2.0",
                "id": id,
                "result": {
                    "content": [{"type": "text", "text": payload.to_string()}],
                    "structuredContent": payload
                }
            }))
            .unwrap_or_default(),
            Err(msg) => rpc_err(id, -32602, &msg),
        };
        return McpDispatchOutput {
            response_json,
            mutated_document: mutated,
        };
    }

    let method = parsed.get("method").and_then(|v| v.as_str()).unwrap_or("");
    let params = parsed.get("params").cloned().unwrap_or_else(|| json!({}));

    match method {
        "initialize" => McpDispatchOutput {
            response_json: serde_json::to_string_pretty(&json!({
                "jsonrpc": "2.0",
                "id": id,
                "result": {
                    "protocolVersion": "2024-11-05",
                    "serverInfo": {"name": "elephant-cad", "version": "0.1.0"},
                    "capabilities": {"tools": {}}
                }
            }))
            .unwrap_or_default(),
            mutated_document: false,
        },
        "tools/list" => McpDispatchOutput {
            response_json: serde_json::to_string_pretty(&json!({
                "jsonrpc": "2.0",
                "id": id,
                "result": { "tools": mcp_tools_manifest() }
            }))
            .unwrap_or_default(),
            mutated_document: false,
        },
        "tools/call" => {
            let name = params.get("name").and_then(|v| v.as_str()).unwrap_or("");
            let args = params
                .get("arguments")
                .cloned()
                .unwrap_or_else(|| json!({}));
            let (res, mutated) = execute_tool(name, &args, doc);
            let response_json = match res {
                Ok(payload) => serde_json::to_string_pretty(&json!({
                    "jsonrpc": "2.0",
                    "id": id,
                    "result": {
                        "content": [{"type": "text", "text": payload.to_string()}],
                        "structuredContent": payload
                    }
                }))
                .unwrap_or_default(),
                Err(msg) => rpc_err(id, -32602, &msg),
            };
            McpDispatchOutput {
                response_json,
                mutated_document: mutated,
            }
        }
        other => McpDispatchOutput {
            response_json: rpc_err(id, -32601, &format!("Unknown method: {other}")),
            mutated_document: false,
        },
    }
}

fn execute_tool(name: &str, args: &Value, doc: &mut Document) -> (Result<Value, String>, bool) {
    let f = |k: &str, def: f64| args.get(k).and_then(|v| v.as_f64()).unwrap_or(def);
    let u = |k: &str, def: u64| args.get(k).and_then(|v| v.as_u64()).unwrap_or(def);

    match name {
        "inspect_document" => {
            let objs: Vec<Value> = doc
                .objects
                .iter()
                .map(|(oid, obj)| {
                    let kind = match &obj.geometry {
                        Geometry::Point(_) => "Point",
                        Geometry::Curve(_) => "Curve",
                        Geometry::Surface(_) => "Surface",
                        Geometry::BRep(_) => "BRep",
                        Geometry::SubD(_) => "SubD",
                        Geometry::Mesh(_) => "Mesh",
                    };
                    json!({
                        "id": oid.0,
                        "name": obj.name,
                        "kind": kind,
                        "visible": obj.visible,
                        "selected": obj.selected,
                        "aabb_min": obj.aabb.min.to_array(),
                        "aabb_max": obj.aabb.max.to_array(),
                    })
                })
                .collect();
            let bb = doc.visible_bounding_box();
            (
                Ok(json!({
                    "units": doc.units.label(),
                    "object_count": doc.objects.len(),
                    "layer_count": doc.layers.len(),
                    "bbox_min": if bb.is_empty() { Value::Null } else { json!(bb.min.to_array()) },
                    "bbox_max": if bb.is_empty() { Value::Null } else { json!(bb.max.to_array()) },
                    "objects": objs,
                })),
                false,
            )
        }
        "create_box" => {
            let c = DVec3::new(f("cx", 0.0), f("cy", 0.0), f("cz", 0.0));
            let s = DVec3::new(f("sx", 2.0), f("sy", 2.0), f("sz", 2.0));
            let oid = doc.add_object("MCP_Box", Geometry::BRep(BRep::from_box(c, s)));
            (Ok(json!({"created_id": oid.0, "kind": "BRep"})), true)
        }
        "create_sphere" => {
            let c = DVec3::new(f("cx", 0.0), f("cy", 0.0), f("cz", 0.0));
            let r = f("r", 1.5).max(0.01);
            match NurbsSurface::sphere(c, r) {
                Ok(s) => {
                    let oid = doc.add_object("MCP_Sphere", Geometry::Surface(s));
                    (Ok(json!({"created_id": oid.0, "kind": "Surface"})), true)
                }
                Err(e) => (Err(e.to_string()), false),
            }
        }
        "create_circle" => {
            let mut pl = Plane::world_xy();
            pl.origin = DVec3::new(f("cx", 0.0), f("cy", 0.0), f("cz", 0.0));
            let r = f("r", 1.5).max(0.01);
            match NurbsCurve::circle(&pl, r) {
                Ok(crv) => {
                    let oid = doc.add_object("MCP_Circle", Geometry::Curve(crv));
                    (Ok(json!({"created_id": oid.0, "kind": "Curve"})), true)
                }
                Err(e) => (Err(e.to_string()), false),
            }
        }
        "create_subd_box" => {
            let c = DVec3::new(f("cx", 0.0), f("cy", 0.0), f("cz", 0.0));
            let s = DVec3::new(f("sx", 2.0), f("sy", 2.0), f("sz", 2.0));
            let mut sd = SubDMesh::from_box(c, s);
            sd.subdivision_level = u("level", 2).clamp(1, 4) as u8;
            let oid = doc.add_object("MCP_SubDBox", Geometry::SubD(sd));
            (Ok(json!({"created_id": oid.0, "kind": "SubD"})), true)
        }
        "create_subd_sphere" => {
            let c = DVec3::new(f("cx", 0.0), f("cy", 0.0), f("cz", 0.0));
            let r = f("r", 1.5).max(0.01);
            let mut sd = SubDMesh::from_sphere(c, r);
            sd.subdivision_level = u("level", 3).clamp(1, 4) as u8;
            let oid = doc.add_object("MCP_SubDSphere", Geometry::SubD(sd));
            (Ok(json!({"created_id": oid.0, "kind": "SubD"})), true)
        }
        "subd_to_nurbs" => {
            let mut targets: Vec<_> = doc
                .objects
                .iter()
                .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::SubD(_)))
                .map(|(id, _)| id)
                .collect();
            if targets.is_empty() {
                if let Some((last_id, _)) = doc
                    .objects
                    .iter()
                    .filter(|(_, o)| matches!(o.geometry, Geometry::SubD(_)))
                    .last()
                {
                    targets.push(last_id);
                }
            }
            if targets.is_empty() {
                return (Err("No SubD object found in document".to_string()), false);
            }
            let mut converted = Vec::new();
            for tid in targets {
                let Some(obj) = doc.objects.get(tid) else { continue };
                let lid = obj.layer_id;
                let Geometry::SubD(sd) = &obj.geometry else { continue };
                if let Ok(brep) = subd_to_nurbs_brep(sd) {
                    let faces = brep.faces.len();
                    let created = doc.replace_object_with_many(
                        "MCP ToNURBS",
                        tid,
                        vec![(lid, Geometry::BRep(brep))],
                    );
                    if let Some(&new_id) = created.first() {
                        if let Some(o) = doc.objects.get_mut(new_id) {
                            o.name = Some("MCP_ToNURBS".to_string());
                        }
                        converted.push(json!({"id": new_id.0, "faces": faces}));
                    }
                }
            }
            let mutated = !converted.is_empty();
            (Ok(json!({"converted": converted})), mutated)
        }
        "shrinkwrap" => {
            let selected: Vec<_> = doc
                .objects
                .iter()
                .filter(|(_, o)| o.selected && o.visible)
                .map(|(id, _)| id)
                .collect();
            let src_ids = if selected.is_empty() {
                doc.objects
                    .iter()
                    .filter(|(_, o)| o.visible && doc.is_layer_effectively_visible(o.layer_id))
                    .map(|(id, _)| id)
                    .collect::<Vec<_>>()
            } else {
                selected
            };
            let geoms: Vec<&Geometry> = src_ids
                .iter()
                .filter_map(|id| doc.objects.get(*id).map(|o| &o.geometry))
                .collect();
            if geoms.is_empty() {
                return (Err("No visible geometry to ShrinkWrap".to_string()), false);
            }
            let opts = ShrinkWrapOptions {
                voxel_size: f("voxel_size", 0.25).max(0.05),
                offset: f("offset", 0.0),
                smoothing_iterations: u("smoothing_iterations", 4).min(12) as usize,
                fill_holes: true,
                max_grid_dim: 56,
            };
            match shrinkwrap_geometries(&geoms, &opts) {
                Ok(mesh) => {
                    let v = mesh.positions.len();
                    let t = mesh.indices.len();
                    let oid = doc.add_object("MCP_ShrinkWrap", Geometry::Mesh(mesh));
                    (Ok(json!({"created_id": oid.0, "vertices": v, "triangles": t})), true)
                }
                Err(e) => (Err(e.to_string()), false),
            }
        }
        "boolean_op" => {
            let op = match args.get("op").and_then(|v| v.as_str()).unwrap_or("union") {
                "difference" | "diff" => BooleanOp::Difference,
                "intersection" | "intersect" => BooleanOp::Intersection,
                _ => BooleanOp::Union,
            };
            let mut sel: Vec<_> = doc.selected_ids_ordered();
            if sel.len() < 2 {
                sel = doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.visible && !matches!(o.geometry, Geometry::Point(_) | Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
            }
            if sel.len() < 2 {
                return (Err("Need at least 2 solids/meshes for Boolean op".to_string()), false);
            }
            let id_a = sel[sel.len() - 2];
            let id_b = sel[sel.len() - 1];
            let (Some(oa), Some(ob)) = (doc.objects.get(id_a), doc.objects.get(id_b)) else {
                return (Err("Invalid operands".to_string()), false);
            };
            let res = match (&oa.geometry, &ob.geometry) {
                (Geometry::BRep(ba), Geometry::BRep(bb)) => {
                    brep_boolean(ba, bb, op, doc.tolerances.absolute)
                        .ok()
                        .map(Geometry::BRep)
                }
                _ => {
                    let ma = to_mesh(&oa.geometry);
                    let mb = to_mesh(&ob.geometry);
                    ma.zip(mb).map(|(a, b)| Geometry::Mesh(mesh_boolean(&a, &b, op)))
                }
            };
            let Some(geom) = res else {
                return (Err("Boolean operation failed on operands".to_string()), false);
            };
            let new_id = doc.add_object("MCP_Boolean", geom);
            (Ok(json!({"created_id": new_id.0})), true)
        }
        "compute_mass_properties" => {
            let sel: Vec<_> = doc
                .objects
                .values()
                .filter(|o| o.selected && o.visible)
                .filter_map(|o| compute_geometry_mass_properties(&o.geometry))
                .collect();
            let list = if !sel.is_empty() {
                sel
            } else {
                doc.objects
                    .values()
                    .filter(|o| o.visible && doc.is_layer_effectively_visible(o.layer_id))
                    .filter_map(|o| compute_geometry_mass_properties(&o.geometry))
                    .collect()
            };
            if list.is_empty() {
                return (Err("No surface/solid/SubD/mesh objects in document".to_string()), false);
            }
            let mp = combine_mass_properties(&list);
            (
                Ok(json!({
                    "area": mp.area,
                    "volume": mp.volume,
                    "centroid": mp.centroid.to_array(),
                    "is_closed_solid": mp.is_closed_solid,
                    "principal_moments": mp.principal_moments,
                })),
                false,
            )
        }
        "run_rhai" => {
            let Some(src) = args.get("source").and_then(|v| v.as_str()) else {
                return (Err("Missing 'source' Rhai script string".to_string()), false);
            };
            match run_rhai_script(src, doc) {
                Ok(res) => (
                    Ok(json!({
                        "created_count": res.created_count,
                        "cleared": res.cleared,
                        "logs": res.logs,
                    })),
                    res.created_count > 0 || res.cleared,
                ),
                Err(e) => (Err(e.to_string()), false),
            }
        }
        "export_step" => {
            let step = export_step_string(doc);
            (Ok(json!({"step_length": step.len(), "step": step})), false)
        }
        "clear_document" => {
            *doc = Document::new();
            (Ok(json!({"cleared": true})), true)
        }
        other => (Err(format!("Unknown MCP tool '{other}'")), false),
    }
}

fn to_mesh(g: &Geometry) -> Option<cad_geom::TriangleMesh> {
    match g {
        Geometry::Mesh(m) => Some(m.clone()),
        Geometry::Surface(s) => Some(s.tessellate_display()),
        Geometry::BRep(b) => Some(b.tessellate_display()),
        Geometry::SubD(sd) => Some(sd.tessellate_display()),
        _ => None,
    }
}

fn rpc_err(id: Value, code: i64, message: &str) -> String {
    serde_json::to_string_pretty(&json!({
        "jsonrpc": "2.0",
        "id": id,
        "error": { "code": code, "message": message }
    }))
    .unwrap_or_default()
}

pub const DEFAULT_MCP_REQUEST: &str = r#"{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/call",
  "params": {
    "name": "create_subd_box",
    "arguments": {
      "cx": 0.0,
      "cy": 0.0,
      "cz": 1.0,
      "sx": 3.0,
      "sy": 2.0,
      "sz": 2.0,
      "level": 2
    }
  }
}"#;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mcp_jsonrpc_tools_list_and_call_roundtrip() {
        let mut doc = Document::new();

        let init = dispatch_mcp_json(
            r#"{"jsonrpc":"2.0","id":1,"method":"initialize","params":{}}"#,
            &mut doc,
        );
        assert!(!init.mutated_document);
        assert!(init.response_json.contains("elephant-cad"));

        let list = dispatch_mcp_json(
            r#"{"jsonrpc":"2.0","id":2,"method":"tools/list"}"#,
            &mut doc,
        );
        assert!(list.response_json.contains("shrinkwrap"));
        assert!(list.response_json.contains("subd_to_nurbs"));

        let mk = dispatch_mcp_json(DEFAULT_MCP_REQUEST, &mut doc);
        assert!(mk.mutated_document);
        assert_eq!(doc.objects.len(), 1);

        let tonurbs = dispatch_mcp_json(
            r#"{"tool":"subd_to_nurbs","arguments":{}}"#,
            &mut doc,
        );
        assert!(tonurbs.mutated_document);
        assert!(matches!(
            doc.objects.values().next().unwrap().geometry,
            Geometry::BRep(_)
        ));

        let mass = dispatch_mcp_json(
            r#"{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"compute_mass_properties","arguments":{}}}"#,
            &mut doc,
        );
        assert!(!mass.mutated_document);
        let v: Value = serde_json::from_str(&mass.response_json).unwrap();
        let vol = v["result"]["structuredContent"]["volume"].as_f64().unwrap();
        assert!(vol > 1.0);
    }
}
