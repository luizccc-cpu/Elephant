use cad_doc::Document;
use cad_geom::{DMat4, DVec3, Geometry, NurbsCurve, Plane, TriangleMesh};

#[derive(Debug, Clone, PartialEq)]
pub enum ActiveTool {
    Idle,
    Line { start: Option<DVec3> },
    Polyline { points: Vec<DVec3> },
    Rectangle { corner1: Option<DVec3> },
    Arc { p0: Option<DVec3>, p1: Option<DVec3> },
    Circle { center: Option<DVec3> },
    Polygon { center: Option<DVec3>, sides: usize },
    Box { base: Option<DVec3>, corner: Option<DVec3> },
    Cylinder { center: Option<DVec3>, radius: Option<f64> },
    Cone { center: Option<DVec3>, radius: Option<f64> },
    Revolve { axis_start: Option<DVec3> },
    Move { base: Option<DVec3> },
    Copy { base: Option<DVec3> },
    Rotate { center: Option<DVec3>, ref_vec: Option<DVec3> },
    Scale { origin: Option<DVec3>, ref_dist: Option<f64> },
    Mirror { start: Option<DVec3> },
    Offset { distance: f64 },
    ArrayLinear { count: usize, base: Option<DVec3> },
    ArrayPolar { count: usize },
    Distance { first: Option<DVec3> },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CommandOutcome {
    pub messages: Vec<String>,
    pub finished: bool,
}

impl CommandOutcome {
    fn msg(m: impl Into<String>) -> Self {
        Self {
            messages: vec![m.into()],
            finished: false,
        }
    }

    fn done(m: impl Into<String>) -> Self {
        Self {
            messages: vec![m.into()],
            finished: true,
        }
    }
}

/// Interactive multi-step CAD command state machine.
#[derive(Debug, Clone)]
pub struct CommandSession {
    pub tool: ActiveTool,
}

impl Default for CommandSession {
    fn default() -> Self {
        Self {
            tool: ActiveTool::Idle,
        }
    }
}

impl CommandSession {
    pub fn is_active(&self) -> bool {
        !matches!(self.tool, ActiveTool::Idle)
    }

    pub fn base_point(&self) -> Option<DVec3> {
        match &self.tool {
            ActiveTool::Line { start } => *start,
            ActiveTool::Polyline { points } => points.last().copied(),
            ActiveTool::Rectangle { corner1 } => *corner1,
            ActiveTool::Arc { p0, p1 } => p1.or(*p0),
            ActiveTool::Circle { center } => *center,
            ActiveTool::Polygon { center, .. } => *center,
            ActiveTool::Box { base, .. } => *base,
            ActiveTool::Cylinder { center, .. } | ActiveTool::Cone { center, .. } => *center,
            ActiveTool::Revolve { axis_start } => *axis_start,
            ActiveTool::Move { base } | ActiveTool::Copy { base } => *base,
            ActiveTool::Rotate { center, .. } => *center,
            ActiveTool::Scale { origin, .. } => *origin,
            ActiveTool::Mirror { start } => *start,
            ActiveTool::ArrayLinear { base, .. } => *base,
            ActiveTool::Distance { first } => *first,
            ActiveTool::Offset { .. } | ActiveTool::ArrayPolar { .. } | ActiveTool::Idle => None,
        }
    }

    pub fn prompt(&self) -> &'static str {
        match &self.tool {
            ActiveTool::Idle => "Command",
            ActiveTool::Line { start: None } => "Start of line (click or x,y,z)",
            ActiveTool::Line { start: Some(_) } => "End of line (click, x,y,z, or distance)",
            ActiveTool::Polyline { points } if points.is_empty() => "Start of polyline",
            ActiveTool::Polyline { .. } => "Next point of polyline (Right-click or Enter to finish)",
            ActiveTool::Rectangle { corner1: None } => "First corner of rectangle",
            ActiveTool::Rectangle { corner1: Some(_) } => "Other corner of rectangle",
            ActiveTool::Arc { p0: None, .. } => "Start of 3-point arc",
            ActiveTool::Arc { p1: None, .. } => "Point on arc",
            ActiveTool::Arc { .. } => "End of arc",
            ActiveTool::Circle { center: None } => "Center of circle",
            ActiveTool::Circle { center: Some(_) } => "Radius (click or enter number)",
            ActiveTool::Polygon { center: None, .. } => "Center of polygon (or enter NumSides)",
            ActiveTool::Polygon { center: Some(_), .. } => "Corner point / radius of polygon",
            ActiveTool::Box { base: None, .. } => "First corner of box base",
            ActiveTool::Box { corner: None, .. } => "Other corner of box base",
            ActiveTool::Box { .. } => "Height of box (click or enter number)",
            ActiveTool::Cylinder { center: None, .. } => "Base center of cylinder",
            ActiveTool::Cylinder { radius: None, .. } => "Radius of cylinder (click or enter number)",
            ActiveTool::Cylinder { .. } => "Height of cylinder (click or enter number)",
            ActiveTool::Cone { center: None, .. } => "Base center of cone",
            ActiveTool::Cone { radius: None, .. } => "Radius of cone (click or enter number)",
            ActiveTool::Cone { .. } => "Height of cone (click or enter number)",
            ActiveTool::Revolve { axis_start: None } => "Start of revolve axis",
            ActiveTool::Revolve { axis_start: Some(_) } => "End of revolve axis",
            ActiveTool::Move { base: None } => "Point to move from",
            ActiveTool::Move { base: Some(_) } => "Point to move to",
            ActiveTool::Copy { base: None } => "Point to copy from",
            ActiveTool::Copy { base: Some(_) } => "Point to copy to",
            ActiveTool::Rotate { center: None, .. } => "Center of rotation",
            ActiveTool::Rotate { ref_vec: None, .. } => "Angle or first reference point",
            ActiveTool::Rotate { .. } => "Second reference point",
            ActiveTool::Scale { origin: None, .. } => "Origin point for scale",
            ActiveTool::Scale { ref_dist: None, .. } => "Scale factor or first reference point",
            ActiveTool::Scale { .. } => "Second reference point",
            ActiveTool::Mirror { start: None } => "Start of mirror plane",
            ActiveTool::Mirror { start: Some(_) } => "End of mirror plane",
            ActiveTool::Offset { .. } => "Side to offset (click side, or enter distance)",
            ActiveTool::ArrayLinear { base: None, .. } => "Base point for linear array (or enter count)",
            ActiveTool::ArrayLinear { base: Some(_), .. } => "Second reference point (step spacing)",
            ActiveTool::ArrayPolar { .. } => "Center of polar array (or enter count)",
            ActiveTool::Distance { first: None } => "First point for distance",
            ActiveTool::Distance { first: Some(_) } => "Second point for distance",
        }
    }

    pub fn start_tool(&mut self, name: &str, doc: &Document) -> Option<String> {
        let has_selection = doc.objects.values().any(|o| o.selected);
        match name.to_ascii_lowercase().as_str() {
            "line" => {
                self.tool = ActiveTool::Line { start: None };
                Some(self.prompt().to_string())
            }
            "polyline" => {
                self.tool = ActiveTool::Polyline { points: Vec::new() };
                Some(self.prompt().to_string())
            }
            "rectangle" | "rect" => {
                self.tool = ActiveTool::Rectangle { corner1: None };
                Some(self.prompt().to_string())
            }
            "arc" => {
                self.tool = ActiveTool::Arc { p0: None, p1: None };
                Some(self.prompt().to_string())
            }
            "circle" => {
                self.tool = ActiveTool::Circle { center: None };
                Some(self.prompt().to_string())
            }
            "polygon" => {
                self.tool = ActiveTool::Polygon {
                    center: None,
                    sides: 6,
                };
                Some(self.prompt().to_string())
            }
            "box" => {
                self.tool = ActiveTool::Box {
                    base: None,
                    corner: None,
                };
                Some(self.prompt().to_string())
            }
            "cylinder" => {
                self.tool = ActiveTool::Cylinder {
                    center: None,
                    radius: None,
                };
                Some(self.prompt().to_string())
            }
            "cone" => {
                self.tool = ActiveTool::Cone {
                    center: None,
                    radius: None,
                };
                Some(self.prompt().to_string())
            }
            "revolve" => {
                let has_curve = doc
                    .objects
                    .values()
                    .any(|o| o.selected && matches!(o.geometry, Geometry::Curve(_)));
                if !has_curve {
                    return Some("Select a profile curve to revolve first.".to_string());
                }
                self.tool = ActiveTool::Revolve { axis_start: None };
                Some(self.prompt().to_string())
            }
            "move" => {
                if !has_selection {
                    return Some("Select objects to move first.".to_string());
                }
                self.tool = ActiveTool::Move { base: None };
                Some(self.prompt().to_string())
            }
            "copy" => {
                if !has_selection {
                    return Some("Select objects to copy first.".to_string());
                }
                self.tool = ActiveTool::Copy { base: None };
                Some(self.prompt().to_string())
            }
            "rotate" => {
                if !has_selection {
                    return Some("Select objects to rotate first.".to_string());
                }
                self.tool = ActiveTool::Rotate {
                    center: None,
                    ref_vec: None,
                };
                Some(self.prompt().to_string())
            }
            "scale" => {
                if !has_selection {
                    return Some("Select objects to scale first.".to_string());
                }
                self.tool = ActiveTool::Scale {
                    origin: None,
                    ref_dist: None,
                };
                Some(self.prompt().to_string())
            }
            "mirror" => {
                if !has_selection {
                    return Some("Select objects to mirror first.".to_string());
                }
                self.tool = ActiveTool::Mirror { start: None };
                Some(self.prompt().to_string())
            }
            "offset" => {
                let has_curve = doc
                    .objects
                    .values()
                    .any(|o| o.selected && matches!(o.geometry, Geometry::Curve(_)));
                if !has_curve {
                    return Some("Select a curve to offset first.".to_string());
                }
                self.tool = ActiveTool::Offset { distance: 1.0 };
                Some(self.prompt().to_string())
            }
            "arraylinear" | "array" => {
                if !has_selection {
                    return Some("Select objects to array first.".to_string());
                }
                self.tool = ActiveTool::ArrayLinear {
                    count: 4,
                    base: None,
                };
                Some(self.prompt().to_string())
            }
            "arraypolar" => {
                if !has_selection {
                    return Some("Select objects to polar-array first.".to_string());
                }
                self.tool = ActiveTool::ArrayPolar { count: 6 };
                Some(self.prompt().to_string())
            }
            "distance" | "dist" => {
                self.tool = ActiveTool::Distance { first: None };
                Some(self.prompt().to_string())
            }
            _ => None,
        }
    }

    pub fn cancel(&mut self) -> String {
        self.tool = ActiveTool::Idle;
        "Command cancelled.".to_string()
    }

    pub fn finish_or_cancel(&mut self, doc: &mut Document) -> CommandOutcome {
        if let ActiveTool::Polyline { points } = &self.tool {
            if points.len() >= 2 {
                if let Ok(poly) = NurbsCurve::polyline(points) {
                    doc.add_object("Polyline", Geometry::Curve(poly));
                    let count = points.len();
                    self.tool = ActiveTool::Idle;
                    return CommandOutcome::done(format!("Created Polyline with {count} points."));
                }
            }
        }
        self.tool = ActiveTool::Idle;
        CommandOutcome::done("Command cancelled.")
    }

    pub fn on_point(&mut self, pt: DVec3, doc: &mut Document, cplane: &Plane) -> CommandOutcome {
        match &mut self.tool {
            ActiveTool::Idle => CommandOutcome {
                messages: Vec::new(),
                finished: false,
            },
            ActiveTool::Line { start } => {
                if let Some(s) = *start {
                    doc.add_object("Line", Geometry::Curve(NurbsCurve::line(s, pt)));
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!(
                        "Created Line ({:.2}, {:.2}, {:.2}) -> ({:.2}, {:.2}, {:.2}).",
                        s.x, s.y, s.z, pt.x, pt.y, pt.z
                    ))
                } else {
                    *start = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Polyline { points } => {
                points.push(pt);
                CommandOutcome::msg(self.prompt())
            }
            ActiveTool::Circle { center } => {
                if let Some(c) = *center {
                    let r = c.distance(pt);
                    let mut plane = *cplane;
                    plane.origin = c;
                    match NurbsCurve::circle(&plane, r) {
                        Ok(circle) => {
                            doc.add_object("Circle", Geometry::Curve(circle));
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Created Circle (r = {r:.3})."))
                        }
                        Err(e) => {
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Circle error: {e}"))
                        }
                    }
                } else {
                    *center = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Box { base, corner } => {
                if base.is_none() {
                    *base = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if corner.is_none() {
                    *corner = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else {
                    let b = base.unwrap();
                    let c = corner.unwrap();
                    let h = (pt - c).dot(cplane.z_axis);
                    let h = if h.abs() < 1e-3 { 2.0 } else { h };
                    let size = DVec3::new((c.x - b.x).abs().max(0.1), (c.y - b.y).abs().max(0.1), h.abs());
                    let center = DVec3::new((b.x + c.x) * 0.5, (b.y + c.y) * 0.5, b.z + h * 0.5);
                    doc.add_object("Box", Geometry::Mesh(TriangleMesh::from_box(center, size)));
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!(
                        "Created Box ({:.2} x {:.2} x {:.2}).",
                        size.x, size.y, size.z
                    ))
                }
            }
            ActiveTool::Cylinder { center, radius } => {
                if center.is_none() {
                    *center = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if radius.is_none() {
                    let c = center.unwrap();
                    *radius = Some(c.distance(pt).max(0.05));
                    CommandOutcome::msg(self.prompt())
                } else {
                    let c = center.unwrap();
                    let r = radius.unwrap();
                    let h = (pt - c).dot(cplane.z_axis);
                    let h = if h.abs() < 1e-3 { 3.0 } else { h };
                    doc.add_object(
                        "Cylinder",
                        Geometry::Mesh(TriangleMesh::from_cylinder(c, r, h, 32)),
                    );
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Created Cylinder (r={r:.2}, h={h:.2})."))
                }
            }
            ActiveTool::Cone { center, radius } => {
                if center.is_none() {
                    *center = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if radius.is_none() {
                    let c = center.unwrap();
                    *radius = Some(c.distance(pt).max(0.05));
                    CommandOutcome::msg(self.prompt())
                } else {
                    let c = center.unwrap();
                    let r = radius.unwrap();
                    let h = (pt - c).dot(cplane.z_axis);
                    let h = if h.abs() < 1e-3 { 3.0 } else { h };
                    doc.add_object(
                        "Cone",
                        Geometry::Mesh(TriangleMesh::from_cone(c, r, h, 32)),
                    );
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Created Cone (r={r:.2}, h={h:.2})."))
                }
            }
            ActiveTool::Revolve { axis_start } => {
                if let Some(a) = *axis_start {
                    let dir = (pt - a).normalize_or_zero();
                    if dir.length_squared() < 1e-6 {
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done("Revolve axis is degenerate.");
                    }
                    let curves: Vec<_> = doc
                        .objects
                        .values()
                        .filter(|o| o.selected)
                        .filter_map(|o| match &o.geometry {
                            Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                            _ => None,
                        })
                        .collect();
                    let count = curves.len();
                    for (lid, c) in curves {
                        let mesh = TriangleMesh::revolve_curve(
                            &c,
                            a,
                            dir,
                            std::f64::consts::TAU,
                            24,
                            32,
                        );
                        doc.add_object_on_layer("Revolve", lid, Geometry::Mesh(mesh));
                    }
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Revolved {count} curve(s) 360 deg."))
                } else {
                    *axis_start = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Move { base } => {
                if let Some(b) = *base {
                    let delta = pt - b;
                    let n = doc.transform_selected("Move", &DMat4::from_translation(delta));
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!(
                        "Moved {n} object(s) by ({:.2}, {:.2}, {:.2}).",
                        delta.x, delta.y, delta.z
                    ))
                } else {
                    *base = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Copy { base } => {
                if let Some(b) = *base {
                    let delta = pt - b;
                    let mat = DMat4::from_translation(delta);
                    let copies: Vec<_> = doc
                        .objects
                        .values()
                        .filter(|o| o.selected)
                        .map(|o| {
                            let mut g = o.geometry.clone();
                            g.transform(&mat);
                            g
                        })
                        .collect();
                    let count = copies.len();
                    for g in copies {
                        doc.add_object("Copy", g);
                    }
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Copied {count} object(s)."))
                } else {
                    *base = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Rotate { center, ref_vec } => {
                if center.is_none() {
                    *center = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if ref_vec.is_none() {
                    let c = center.unwrap();
                    let v = (pt - c).normalize_or_zero();
                    *ref_vec = Some(if v.length_squared() < 1e-6 { cplane.x_axis } else { v });
                    CommandOutcome::msg(self.prompt())
                } else {
                    let c = center.unwrap();
                    let v0 = ref_vec.unwrap();
                    let v1 = (pt - c).normalize_or_zero();
                    let angle = v0.angle_between(v1)
                        * if v0.cross(v1).dot(cplane.z_axis) >= 0.0 {
                            1.0
                        } else {
                            -1.0
                        };
                    let mat = DMat4::from_translation(c)
                        * DMat4::from_axis_angle(cplane.z_axis, angle)
                        * DMat4::from_translation(-c);
                    let n = doc.transform_selected("Rotate", &mat);
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!(
                        "Rotated {n} object(s) by {:.1} deg.",
                        angle.to_degrees()
                    ))
                }
            }
            ActiveTool::Scale { origin, ref_dist } => {
                if origin.is_none() {
                    *origin = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if ref_dist.is_none() {
                    let o = origin.unwrap();
                    *ref_dist = Some(o.distance(pt).max(1e-4));
                    CommandOutcome::msg(self.prompt())
                } else {
                    let o = origin.unwrap();
                    let d0 = ref_dist.unwrap();
                    let d1 = o.distance(pt).max(1e-4);
                    let factor = d1 / d0;
                    let mat = DMat4::from_translation(o)
                        * DMat4::from_scale(DVec3::splat(factor))
                        * DMat4::from_translation(-o);
                    let n = doc.transform_selected("Scale", &mat);
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Scaled {n} object(s) by {factor:.3}."))
                }
            }
            ActiveTool::Rectangle { corner1 } => {
                if let Some(c1) = *corner1 {
                    match NurbsCurve::rectangle(cplane, c1, pt) {
                        Ok(rect) => {
                            doc.add_object("Rectangle", Geometry::Curve(rect));
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done("Created Rectangle.")
                        }
                        Err(e) => {
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Rectangle error: {e}"))
                        }
                    }
                } else {
                    *corner1 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Arc { p0, p1 } => {
                if p0.is_none() {
                    *p0 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if p1.is_none() {
                    *p1 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else {
                    let a = p0.unwrap();
                    let b = p1.unwrap();
                    match NurbsCurve::arc_3pt(a, b, pt) {
                        Ok(arc) => {
                            doc.add_object("Arc", Geometry::Curve(arc));
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done("Created 3-point Arc.")
                        }
                        Err(e) => {
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Arc error: {e}"))
                        }
                    }
                }
            }
            ActiveTool::Polygon { center, sides } => {
                if let Some(c) = *center {
                    let n_sides = *sides;
                    match NurbsCurve::polygon(cplane, c, pt, n_sides) {
                        Ok(poly) => {
                            doc.add_object("Polygon", Geometry::Curve(poly));
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Created {n_sides}-sided Polygon."))
                        }
                        Err(e) => {
                            self.tool = ActiveTool::Idle;
                            CommandOutcome::done(format!("Polygon error: {e}"))
                        }
                    }
                } else {
                    *center = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Mirror { start } => {
                if let Some(a) = *start {
                    let dir = (pt - a).normalize_or_zero();
                    let n = cplane.z_axis.cross(dir).normalize_or_zero();
                    if n.length_squared() < 1e-6 {
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done("Mirror line is degenerate.");
                    }
                    let refl = DMat4::from_cols_array(&[
                        1.0 - 2.0 * n.x * n.x, -2.0 * n.y * n.x, -2.0 * n.z * n.x, 0.0,
                        -2.0 * n.x * n.y, 1.0 - 2.0 * n.y * n.y, -2.0 * n.z * n.y, 0.0,
                        -2.0 * n.x * n.z, -2.0 * n.y * n.z, 1.0 - 2.0 * n.z * n.z, 0.0,
                        0.0, 0.0, 0.0, 1.0,
                    ]);
                    let mat = DMat4::from_translation(a) * refl * DMat4::from_translation(-a);
                    let copies: Vec<_> = doc
                        .objects
                        .values()
                        .filter(|o| o.selected)
                        .map(|o| {
                            let mut g = o.geometry.clone();
                            g.transform(&mat);
                            (o.layer_id, g)
                        })
                        .collect();
                    let count = copies.len();
                    for (lid, g) in copies {
                        doc.add_object_on_layer("Mirror", lid, g);
                    }
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Mirrored {count} object(s)."))
                } else {
                    *start = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::Offset { distance } => {
                let d = *distance;
                let curves: Vec<_> = doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .filter_map(|o| match &o.geometry {
                        Geometry::Curve(c) => Some((o.layer_id, c.clone())),
                        _ => None,
                    })
                    .collect();
                let mut created = 0;
                for (lid, c) in &curves {
                    if let Ok(off) = c.offset_in_plane(cplane, d, pt) {
                        doc.add_object_on_layer("Offset", *lid, Geometry::Curve(off));
                        created += 1;
                    }
                }
                self.tool = ActiveTool::Idle;
                CommandOutcome::done(format!("Offset {created} curve(s) by {d:.3}."))
            }
            ActiveTool::ArrayLinear { count, base } => {
                if let Some(b) = *base {
                    let step = pt - b;
                    let total_copies = *count;
                    let selected: Vec<_> = doc
                        .objects
                        .values()
                        .filter(|o| o.selected)
                        .map(|o| (o.layer_id, o.geometry.clone()))
                        .collect();
                    let mut created = 0;
                    for i in 1..total_copies {
                        let mat = DMat4::from_translation(step * i as f64);
                        for (lid, g) in &selected {
                            let mut ng = g.clone();
                            ng.transform(&mat);
                            doc.add_object_on_layer("ArrayLinear", *lid, ng);
                            created += 1;
                        }
                    }
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Linear array created {created} copy/copies."))
                } else {
                    *base = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
            ActiveTool::ArrayPolar { count } => {
                let total = *count;
                let selected: Vec<_> = doc
                    .objects
                    .values()
                    .filter(|o| o.selected)
                    .map(|o| (o.layer_id, o.geometry.clone()))
                    .collect();
                let mut created = 0;
                for i in 1..total {
                    let angle = std::f64::consts::TAU * (i as f64 / total as f64);
                    let mat = DMat4::from_translation(pt)
                        * DMat4::from_axis_angle(cplane.z_axis, angle)
                        * DMat4::from_translation(-pt);
                    for (lid, g) in &selected {
                        let mut ng = g.clone();
                        ng.transform(&mat);
                        doc.add_object_on_layer("ArrayPolar", *lid, ng);
                        created += 1;
                    }
                }
                self.tool = ActiveTool::Idle;
                CommandOutcome::done(format!("Polar array created {created} copy/copies."))
            }
            ActiveTool::Distance { first } => {
                if let Some(a) = *first {
                    let d = pt - a;
                    let dist = d.length();
                    let dx = d.dot(cplane.x_axis);
                    let dy = d.dot(cplane.y_axis);
                    let ang = dy.atan2(dx).to_degrees();
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!(
                        "Distance = {dist:.4} (dx={dx:.3}, dy={dy:.3}, dz={:.3}, angle={ang:.2} deg)",
                        d.z
                    ))
                } else {
                    *first = Some(pt);
                    CommandOutcome::msg(self.prompt())
                }
            }
        }
    }

    /// Parse coordinate (`x,y,z`, `@dx,dy,dz`) or scalar input while a command is active.
    pub fn on_text_input(
        &mut self,
        raw: &str,
        doc: &mut Document,
        cplane: &Plane,
    ) -> Option<CommandOutcome> {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return Some(self.finish_or_cancel(doc));
        }

        if let Some(pt) = parse_point_input(trimmed, self.base_point()) {
            return Some(self.on_point(pt, doc, cplane));
        }

        if let Ok(val) = trimmed.parse::<f64>() {
            match &self.tool {
                ActiveTool::Circle { center: Some(c) }
            | ActiveTool::Cylinder {
                center: Some(c),
                radius: None,
            }
            | ActiveTool::Cone {
                center: Some(c),
                radius: None,
            } => {
                    let pt = *c + cplane.x_axis * val;
                    return Some(self.on_point(pt, doc, cplane));
                }
                ActiveTool::Box {
                    base: Some(_),
                    corner: Some(c),
                } => {
                    let pt = *c + cplane.z_axis * val;
                    return Some(self.on_point(pt, doc, cplane));
                }
                ActiveTool::Cylinder {
                    center: Some(c),
                    radius: Some(_),
                }
                | ActiveTool::Cone {
                    center: Some(c),
                    radius: Some(_),
                } => {
                    let pt = *c + cplane.z_axis * val;
                    return Some(self.on_point(pt, doc, cplane));
                }
                ActiveTool::Rotate {
                    center: Some(c),
                    ref_vec: None,
                } => {
                    let mat = DMat4::from_translation(*c)
                        * DMat4::from_axis_angle(cplane.z_axis, val.to_radians())
                        * DMat4::from_translation(-*c);
                    let n = doc.transform_selected("Rotate", &mat);
                    self.tool = ActiveTool::Idle;
                    return Some(CommandOutcome::done(format!(
                        "Rotated {n} object(s) by {val:.1} deg."
                    )));
                }
                ActiveTool::Scale {
                    origin: Some(o),
                    ref_dist: None,
                } => {
                    let mat = DMat4::from_translation(*o)
                        * DMat4::from_scale(DVec3::splat(val))
                        * DMat4::from_translation(-*o);
                    let n = doc.transform_selected("Scale", &mat);
                    self.tool = ActiveTool::Idle;
                    return Some(CommandOutcome::done(format!(
                        "Scaled {n} object(s) by {val:.3}."
                    )));
                }
                ActiveTool::Polygon { center: None, .. } if val >= 3.0 => {
                    let n_sides = (val.round() as usize).max(3);
                    self.tool = ActiveTool::Polygon {
                        center: None,
                        sides: n_sides,
                    };
                    return Some(CommandOutcome::msg(format!(
                        "Polygon sides = {n_sides}. Center of polygon"
                    )));
                }
                ActiveTool::Offset { .. } if val.abs() > 1e-6 => {
                    self.tool = ActiveTool::Offset { distance: val.abs() };
                    return Some(CommandOutcome::msg(format!(
                        "Offset distance = {:.3}. Click side to offset",
                        val.abs()
                    )));
                }
                ActiveTool::ArrayLinear { base: None, .. } if val >= 2.0 => {
                    let n = (val.round() as usize).max(2);
                    self.tool = ActiveTool::ArrayLinear {
                        count: n,
                        base: None,
                    };
                    return Some(CommandOutcome::msg(format!(
                        "Linear array count = {n}. Base point"
                    )));
                }
                ActiveTool::ArrayPolar { .. } if val >= 2.0 => {
                    let n = (val.round() as usize).max(2);
                    self.tool = ActiveTool::ArrayPolar { count: n };
                    return Some(CommandOutcome::msg(format!(
                        "Polar array count = {n}. Center of polar array"
                    )));
                }
                _ => {}
            }
        }
        None
    }

    pub fn preview_geometry(&self, cursor: Option<DVec3>, cplane: &Plane) -> Vec<Geometry> {
        let Some(cur) = cursor else {
            return Vec::new();
        };
        match &self.tool {
            ActiveTool::Line { start: Some(s) }
            | ActiveTool::Mirror { start: Some(s) }
            | ActiveTool::Revolve { axis_start: Some(s) }
            | ActiveTool::Distance { first: Some(s) } => {
                vec![Geometry::Curve(NurbsCurve::line(*s, cur))]
            }
            ActiveTool::Polyline { points } if !points.is_empty() => {
                let mut pts = points.clone();
                pts.push(cur);
                NurbsCurve::polyline(&pts)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Rectangle { corner1: Some(c1) } => {
                NurbsCurve::rectangle(cplane, *c1, cur)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Arc {
                p0: Some(a),
                p1: None,
            } => vec![Geometry::Curve(NurbsCurve::line(*a, cur))],
            ActiveTool::Arc {
                p0: Some(a),
                p1: Some(b),
            } => NurbsCurve::arc_3pt(*a, *b, cur)
                .map(|c| vec![Geometry::Curve(c)])
                .unwrap_or_default(),
            ActiveTool::Circle { center: Some(c) }
            | ActiveTool::Cylinder {
                center: Some(c),
                radius: None,
            }
            | ActiveTool::Cone {
                center: Some(c),
                radius: None,
            } => {
                let r = c.distance(cur);
                if r > 1e-4 {
                    let mut plane = *cplane;
                    plane.origin = *c;
                    NurbsCurve::circle(&plane, r)
                        .map(|circle| vec![Geometry::Curve(circle)])
                        .unwrap_or_default()
                } else {
                    Vec::new()
                }
            }
            ActiveTool::Polygon {
                center: Some(c),
                sides,
            } => NurbsCurve::polygon(cplane, *c, cur, *sides)
                .map(|p| vec![Geometry::Curve(p)])
                .unwrap_or_default(),
            ActiveTool::Box {
                base: Some(b),
                corner: None,
            } => {
                NurbsCurve::rectangle(cplane, *b, cur)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Move { base: Some(b) }
            | ActiveTool::Copy { base: Some(b) }
            | ActiveTool::ArrayLinear { base: Some(b), .. } => {
                vec![Geometry::Curve(NurbsCurve::line(*b, cur))]
            }
            _ => Vec::new(),
        }
    }
}

pub fn parse_point_input(raw: &str, base: Option<DVec3>) -> Option<DVec3> {
    let (relative, rest) = if let Some(s) = raw.strip_prefix('@') {
        (true, s)
    } else {
        (false, raw)
    };
    let coords: Vec<f64> = rest
        .split(',')
        .map(|s| s.trim().parse::<f64>())
        .collect::<Result<Vec<_>, _>>()
        .ok()?;
    let pt = match coords.as_slice() {
        [x, y] => DVec3::new(*x, *y, 0.0),
        [x, y, z] => DVec3::new(*x, *y, *z),
        _ => return None,
    };
    if relative {
        Some(base.unwrap_or(DVec3::ZERO) + pt)
    } else {
        Some(pt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interactive_line_creates_curve() {
        let mut doc = Document::new();
        let mut session = CommandSession::default();
        session.start_tool("line", &doc);
        session.on_point(DVec3::ZERO, &mut doc, &Plane::world_xy());
        let out = session.on_point(DVec3::new(5.0, 0.0, 0.0), &mut doc, &Plane::world_xy());
        assert!(out.finished);
        assert_eq!(doc.objects.len(), 1);
    }

    #[test]
    fn parse_relative_and_absolute_coordinates() {
        assert_eq!(parse_point_input("1, 2, 3", None), Some(DVec3::new(1.0, 2.0, 3.0)));
        assert_eq!(
            parse_point_input("@4, 0", Some(DVec3::new(1.0, 2.0, 0.0))),
            Some(DVec3::new(5.0, 2.0, 0.0))
        );
    }
}
