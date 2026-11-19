use cad_doc::{AnnotationItem, AnnotationKind, Document, ObjectId};
use cad_geom::{
    DMat4, DVec3, Geometry, NurbsCurve, OffsetCornerStyle, Plane, TriangleMesh,
    trim_curve_with_cutters,
};

#[derive(Debug, Clone, PartialEq)]
pub enum ActiveTool {
    Idle,
    Line { start: Option<DVec3> },
    Polyline { points: Vec<DVec3> },
    Curve { points: Vec<DVec3>, degree: usize, closed: bool },
    InterpCrv { points: Vec<DVec3>, degree: usize, closed: bool },
    HandleCurve { anchors: Vec<(DVec3, DVec3, DVec3)>, pending_anchor: Option<DVec3> },
    Conic { p0: Option<DVec3>, p2: Option<DVec3>, apex: Option<DVec3> },
    Helix { axis_start: Option<DVec3>, axis_end: Option<DVec3>, turns: f64 },
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
    SplitCurve,
    Trim { cutter_ids: Vec<ObjectId> },
    InsertKnot,
    ArrayLinear { count: usize, base: Option<DVec3> },
    ArrayPolar { count: usize },
    Distance { first: Option<DVec3> },
    Dim { p1: Option<DVec3>, p2: Option<DVec3> },
    TextDot { label: String },
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
            ActiveTool::Polyline { points }
            | ActiveTool::Curve { points, .. }
            | ActiveTool::InterpCrv { points, .. } => points.last().copied(),
            ActiveTool::HandleCurve {
                anchors,
                pending_anchor,
            } => pending_anchor.or_else(|| anchors.last().map(|a| a.0)),
            ActiveTool::Conic { p0, p2, apex } => apex.or(*p2).or(*p0),
            ActiveTool::Helix {
                axis_start,
                axis_end,
                ..
            } => axis_end.or(*axis_start),
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
            ActiveTool::Dim { p1, p2 } => p2.or(*p1),
            ActiveTool::Offset { .. }
            | ActiveTool::SplitCurve
            | ActiveTool::Trim { .. }
            | ActiveTool::InsertKnot
            | ActiveTool::ArrayPolar { .. }
            | ActiveTool::TextDot { .. }
            | ActiveTool::Idle => None,
        }
    }

    pub fn prompt(&self) -> &'static str {
        match &self.tool {
            ActiveTool::Idle => "Command",
            ActiveTool::Line { start: None } => "Start of line (click or x,y,z)",
            ActiveTool::Line { start: Some(_) } => "End of line (click, x,y,z, or distance)",
            ActiveTool::Polyline { points } if points.is_empty() => "Start of polyline",
            ActiveTool::Polyline { .. } => "Next point of polyline (Right-click or Enter to finish)",
            ActiveTool::Curve { points, .. } if points.is_empty() => {
                "Start of NURBS curve (or enter d=3, close)"
            }
            ActiveTool::Curve { .. } => {
                "Next control point (d=degree, close, or Enter/Right-click to finish)"
            }
            ActiveTool::InterpCrv { points, .. } if points.is_empty() => {
                "Start of interpolated curve (or enter d=3, close)"
            }
            ActiveTool::InterpCrv { .. } => {
                "Next curve point (d=degree, close, or Enter/Right-click to finish)"
            }
            ActiveTool::HandleCurve {
                pending_anchor: Some(_),
                ..
            } => "Outgoing handle point for anchor",
            ActiveTool::HandleCurve { anchors, .. } if anchors.is_empty() => {
                "First anchor point of handle curve"
            }
            ActiveTool::HandleCurve { .. } => {
                "Next anchor point of handle curve (Enter or Right-click to finish)"
            }
            ActiveTool::Conic { p0: None, .. } => "Start of conic curve",
            ActiveTool::Conic { p2: None, .. } => "End of conic curve",
            ActiveTool::Conic { apex: None, .. } => "Apex of conic curve",
            ActiveTool::Conic { .. } => "Curvature point or rho (0.05..0.95)",
            ActiveTool::Helix {
                axis_start: None, ..
            } => "Start of helix axis (or turns=3)",
            ActiveTool::Helix { axis_end: None, .. } => "End of helix axis",
            ActiveTool::Helix { .. } => "Radius / start point of helix (click or enter radius)",
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
            ActiveTool::SplitCurve => "Click point on selected curve to split",
            ActiveTool::Trim { .. } => "Click curve segment to trim away (Enter/Esc to finish)",
            ActiveTool::InsertKnot => "Click point on selected curve to insert knot",
            ActiveTool::ArrayLinear { base: None, .. } => "Base point for linear array (or enter count)",
            ActiveTool::ArrayLinear { base: Some(_), .. } => "Second reference point (step spacing)",
            ActiveTool::ArrayPolar { .. } => "Center of polar array (or enter count)",
            ActiveTool::Distance { first: None } => "First point for distance",
            ActiveTool::Distance { first: Some(_) } => "Second point for distance",
            ActiveTool::Dim { p1: None, .. } => "First dimension point",
            ActiveTool::Dim { p2: None, .. } => "Second dimension point",
            ActiveTool::Dim { .. } => "Dimension line location",
            ActiveTool::TextDot { .. } => "Location of text dot",
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
            "curve" => {
                self.tool = ActiveTool::Curve {
                    points: Vec::new(),
                    degree: 3,
                    closed: false,
                };
                Some(self.prompt().to_string())
            }
            "interpcrv" => {
                self.tool = ActiveTool::InterpCrv {
                    points: Vec::new(),
                    degree: 3,
                    closed: false,
                };
                Some(self.prompt().to_string())
            }
            "handlecurve" => {
                self.tool = ActiveTool::HandleCurve {
                    anchors: Vec::new(),
                    pending_anchor: None,
                };
                Some(self.prompt().to_string())
            }
            "conic" => {
                self.tool = ActiveTool::Conic {
                    p0: None,
                    p2: None,
                    apex: None,
                };
                Some(self.prompt().to_string())
            }
            "helix" => {
                self.tool = ActiveTool::Helix {
                    axis_start: None,
                    axis_end: None,
                    turns: 3.0,
                };
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
            "splitcurve" => {
                let has_curve = doc
                    .objects
                    .values()
                    .any(|o| o.selected && matches!(o.geometry, Geometry::Curve(_)));
                if !has_curve {
                    return Some("Select a curve to split first.".to_string());
                }
                self.tool = ActiveTool::SplitCurve;
                Some(self.prompt().to_string())
            }
            "trim" => {
                let cutter_ids: Vec<ObjectId> = doc
                    .objects
                    .iter()
                    .filter(|(_, o)| o.selected && matches!(o.geometry, Geometry::Curve(_)))
                    .map(|(id, _)| id)
                    .collect();
                if cutter_ids.is_empty() {
                    return Some("Select one or more cutting curves first.".to_string());
                }
                self.tool = ActiveTool::Trim { cutter_ids };
                Some(self.prompt().to_string())
            }
            "insertknot" => {
                let has_curve = doc
                    .objects
                    .values()
                    .any(|o| o.selected && matches!(o.geometry, Geometry::Curve(_)));
                if !has_curve {
                    return Some("Select a curve to insert a knot into first.".to_string());
                }
                self.tool = ActiveTool::InsertKnot;
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
            "dim" | "dimaligned" => {
                self.tool = ActiveTool::Dim { p1: None, p2: None };
                Some(self.prompt().to_string())
            }
            "textdot" | "dot" => {
                self.tool = ActiveTool::TextDot {
                    label: "Note".to_string(),
                };
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
        match &self.tool {
            ActiveTool::Polyline { points } => {
                if points.len() >= 2 {
                    if let Ok(poly) = NurbsCurve::polyline(points) {
                        doc.add_object("Polyline", Geometry::Curve(poly));
                        let count = points.len();
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done(format!(
                            "Created Polyline with {count} points."
                        ));
                    }
                }
            }
            ActiveTool::Curve {
                points,
                degree,
                closed,
            } => {
                if points.len() >= 2 {
                    if let Ok(curve) = NurbsCurve::from_control_points(points, *degree, *closed) {
                        let d = curve.degree;
                        let n = curve.control_points.len();
                        doc.add_object("Curve", Geometry::Curve(curve));
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done(format!(
                            "Created NURBS Curve (deg {d}, {n} CPs)."
                        ));
                    }
                }
            }
            ActiveTool::InterpCrv {
                points,
                degree,
                closed,
            } => {
                if points.len() >= 2 {
                    if let Ok(curve) = NurbsCurve::interpolate(points, *degree, *closed) {
                        let d = curve.degree;
                        let n = points.len();
                        doc.add_object("InterpCrv", Geometry::Curve(curve));
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done(format!(
                            "Created Interpolated Curve (deg {d}, {n} pts)."
                        ));
                    }
                }
            }
            ActiveTool::HandleCurve {
                anchors,
                pending_anchor,
            } => {
                let mut all = anchors.clone();
                if let Some(pa) = *pending_anchor {
                    all.push((pa, pa, pa));
                }
                if all.len() >= 2 {
                    let segs: Vec<_> = all
                        .windows(2)
                        .map(|w| (w[0].0, w[0].2, w[1].1, w[1].0))
                        .collect();
                    if let Ok(curve) = NurbsCurve::handle_curve(&segs) {
                        let seg_count = segs.len();
                        doc.add_object("HandleCurve", Geometry::Curve(curve));
                        self.tool = ActiveTool::Idle;
                        return CommandOutcome::done(format!(
                            "Created HandleCurve ({seg_count} cubic span(s))."
                        ));
                    }
                }
            }
            ActiveTool::Trim { .. } => {
                self.tool = ActiveTool::Idle;
                return CommandOutcome::done("Finished Trim.");
            }
            _ => {}
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
            ActiveTool::Curve { points, .. } | ActiveTool::InterpCrv { points, .. } => {
                points.push(pt);
                CommandOutcome::msg(self.prompt())
            }
            ActiveTool::HandleCurve {
                anchors,
                pending_anchor,
            } => {
                if let Some(a) = pending_anchor.take() {
                    let h_out = pt;
                    let h_in = a * 2.0 - h_out;
                    anchors.push((a, h_in, h_out));
                } else {
                    *pending_anchor = Some(pt);
                }
                CommandOutcome::msg(self.prompt())
            }
            ActiveTool::Conic { p0, p2, apex } => {
                if p0.is_none() {
                    *p0 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if p2.is_none() {
                    *p2 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if apex.is_none() {
                    *apex = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else {
                    let a = p0.unwrap();
                    let b = p2.unwrap();
                    let ap = apex.unwrap();
                    let mid = (a + b) * 0.5;
                    let leg = ap - mid;
                    let rho = if leg.length_squared() > 1e-10 {
                        ((pt - mid).dot(leg) / leg.length_squared()).clamp(0.05, 0.95)
                    } else {
                        0.5
                    };
                    self.tool = ActiveTool::Idle;
                    match NurbsCurve::conic(a, ap, b, rho) {
                        Ok(conic) => {
                            doc.add_object("Conic", Geometry::Curve(conic));
                            CommandOutcome::done(format!("Created Conic (rho = {rho:.3})."))
                        }
                        Err(e) => CommandOutcome::done(format!("Conic error: {e}")),
                    }
                }
            }
            ActiveTool::Helix {
                axis_start,
                axis_end,
                turns,
            } => {
                if axis_start.is_none() {
                    *axis_start = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if axis_end.is_none() {
                    *axis_end = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else {
                    let a0 = axis_start.unwrap();
                    let a1 = axis_end.unwrap();
                    let t = *turns;
                    self.tool = ActiveTool::Idle;
                    match NurbsCurve::helix(a0, a1, pt, t, None) {
                        Ok(helix) => {
                            doc.add_object("Helix", Geometry::Curve(helix));
                            CommandOutcome::done(format!("Created Helix ({t:.2} turns)."))
                        }
                        Err(e) => CommandOutcome::done(format!("Helix error: {e}")),
                    }
                }
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
                    if let Ok(off) =
                        c.offset_nurbs_in_plane(cplane, d, pt, OffsetCornerStyle::Sharp)
                    {
                        doc.add_object_on_layer("Offset", *lid, Geometry::Curve(off));
                        created += 1;
                    }
                }
                self.tool = ActiveTool::Idle;
                CommandOutcome::done(format!("Offset {created} curve(s) by {d:.3}."))
            }
            ActiveTool::SplitCurve => {
                let mut best: Option<(ObjectId, cad_doc::LayerId, NurbsCurve, f64, f64)> = None;
                for (id, obj) in &doc.objects {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            let (u, _, d) = c.closest_point(pt);
                            if best.as_ref().is_none_or(|b| d < b.4) {
                                best = Some((id, obj.layer_id, c.clone(), u, d));
                            }
                        }
                    }
                }
                self.tool = ActiveTool::Idle;
                if let Some((id, lid, curve, u, _)) = best {
                    if let Some((left, right)) = curve.split_at(u) {
                        doc.replace_object_with_many(
                            "SplitCurve",
                            id,
                            vec![(lid, Geometry::Curve(left)), (lid, Geometry::Curve(right))],
                        );
                        CommandOutcome::done("Split curve into 2 segments.")
                    } else {
                        CommandOutcome::done("Click an interior point on the curve to split.")
                    }
                } else {
                    CommandOutcome::done("No selected curve to split.")
                }
            }
            ActiveTool::Trim { cutter_ids } => {
                let mut best_target: Option<(ObjectId, cad_doc::LayerId, NurbsCurve, f64)> = None;
                for (id, obj) in &doc.objects {
                    if !obj.visible {
                        continue;
                    }
                    let unlocked = doc
                        .layers
                        .get(obj.layer_id)
                        .map(|l| l.visible && !l.locked)
                        .unwrap_or(true);
                    if !unlocked {
                        continue;
                    }
                    if let Geometry::Curve(c) = &obj.geometry {
                        let (_, _, d) = c.closest_point(pt);
                        if best_target.as_ref().is_none_or(|b| d < b.3) {
                            best_target = Some((id, obj.layer_id, c.clone(), d));
                        }
                    }
                }
                let Some((target_id, lid, target_curve, _)) = best_target else {
                    return CommandOutcome::msg("No curve found near click point.");
                };
                let cutters: Vec<NurbsCurve> = cutter_ids
                    .iter()
                    .filter(|&&cid| cid != target_id)
                    .filter_map(|&cid| {
                        doc.objects.get(cid).and_then(|o| match &o.geometry {
                            Geometry::Curve(c) => Some(c.clone()),
                            _ => None,
                        })
                    })
                    .collect();
                if cutters.is_empty() {
                    return CommandOutcome::msg(
                        "Clicked the only cutting curve; click another intersecting curve.",
                    );
                }
                let tol = doc.tolerances.absolute.max(1e-3);
                if let Some(kept) = trim_curve_with_cutters(&target_curve, &cutters, pt, tol) {
                    let replacements: Vec<_> =
                        kept.into_iter().map(|c| (lid, Geometry::Curve(c))).collect();
                    let kept_count = replacements.len();
                    let new_ids = doc.replace_object_with_many("Trim", target_id, replacements);
                    if let Some(pos) = cutter_ids.iter().position(|&cid| cid == target_id) {
                        cutter_ids.remove(pos);
                        cutter_ids.extend(new_ids);
                    }
                    CommandOutcome::msg(format!(
                        "Trimmed curve ({kept_count} segment(s) kept). Click another segment or press Enter."
                    ))
                } else {
                    CommandOutcome::msg("No intersection with cutting curves on that curve.")
                }
            }
            ActiveTool::InsertKnot => {
                let mut best: Option<(ObjectId, f64, f64)> = None;
                for (id, obj) in &doc.objects {
                    if obj.selected {
                        if let Geometry::Curve(c) = &obj.geometry {
                            let (u, _, d) = c.closest_point(pt);
                            if best.as_ref().is_none_or(|b| d < b.2) {
                                best = Some((id, u, d));
                            }
                        }
                    }
                }
                self.tool = ActiveTool::Idle;
                if let Some((id, u, _)) = best {
                    let before = doc.objects[id].clone();
                    if let Geometry::Curve(c) = &before.geometry {
                        if let Ok(updated) = c.insert_knot(u, 1) {
                            let count = updated.control_points.len();
                            doc.objects[id].geometry = Geometry::Curve(updated);
                            doc.objects[id].refresh_aabb();
                            doc.record_object_modified("InsertKnot", id, before);
                            return CommandOutcome::done(format!(
                                "Inserted knot at u={u:.4} ({count} CPs)."
                            ));
                        }
                    }
                }
                CommandOutcome::done("No selected curve for knot insertion.")
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
            ActiveTool::Dim { p1, p2 } => {
                if p1.is_none() {
                    *p1 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else if p2.is_none() {
                    *p2 = Some(pt);
                    CommandOutcome::msg(self.prompt())
                } else {
                    let a = p1.unwrap();
                    let b = p2.unwrap();
                    let dist = a.distance(b);
                    let dir = (b - a).normalize_or_zero();
                    let perp = pt - (a + dir * (pt - a).dot(dir));
                    let mid = (a + b) * 0.5 + perp;
                    let text = format!("{dist:.2}");
                    doc.add_annotation(AnnotationItem {
                        kind: AnnotationKind::Dimension,
                        layer_id: doc.active_layer,
                        color: None,
                        text: text.clone(),
                        position: mid,
                        p1: Some(a),
                        p2: Some(b),
                        offset_pt: Some(pt),
                    });
                    self.tool = ActiveTool::Idle;
                    CommandOutcome::done(format!("Created Dimension: {text}"))
                }
            }
            ActiveTool::TextDot { label } => {
                let text = label.clone();
                doc.add_annotation(AnnotationItem {
                    kind: AnnotationKind::TextDot,
                    layer_id: doc.active_layer,
                    color: None,
                    text: text.clone(),
                    position: pt,
                    p1: None,
                    p2: None,
                    offset_pt: None,
                });
                self.tool = ActiveTool::Idle;
                CommandOutcome::done(format!("Created TextDot '{text}'."))
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

        let lower = trimmed.to_ascii_lowercase();
        match &mut self.tool {
            ActiveTool::Curve { degree, closed, .. }
            | ActiveTool::InterpCrv { degree, closed, .. } => {
                if lower == "close" || lower == "closed" {
                    *closed = !*closed;
                    return Some(CommandOutcome::msg(format!(
                        "Curve closed = {closed}. Next point (or Enter to finish)"
                    )));
                }
                if let Some(d_str) = lower
                    .strip_prefix("d=")
                    .or_else(|| lower.strip_prefix("degree="))
                {
                    if let Ok(d) = d_str.trim().parse::<usize>() {
                        *degree = d.clamp(1, 7);
                        return Some(CommandOutcome::msg(format!(
                            "Curve degree = {degree}. Next point"
                        )));
                    }
                }
            }
            ActiveTool::Helix { turns, .. } => {
                if let Some(t_str) = lower
                    .strip_prefix("t=")
                    .or_else(|| lower.strip_prefix("turns="))
                {
                    if let Ok(t) = t_str.trim().parse::<f64>() {
                        if t.abs() >= 0.05 {
                            let new_turns = t.clamp(-100.0, 100.0);
                            *turns = new_turns;
                            return Some(CommandOutcome::msg(format!(
                                "Helix turns = {new_turns}. {}",
                                self.prompt()
                            )));
                        }
                    }
                }
            }
            _ => {}
        }

        if let Ok(val) = trimmed.parse::<f64>() {
            match &self.tool {
                ActiveTool::Conic {
                    p0: Some(a),
                    p2: Some(b),
                    apex: Some(ap),
                } if val > 0.01 && val < 0.99 => {
                    let a = *a;
                    let b = *b;
                    let ap = *ap;
                    self.tool = ActiveTool::Idle;
                    return Some(match NurbsCurve::conic(a, ap, b, val) {
                        Ok(conic) => {
                            doc.add_object("Conic", Geometry::Curve(conic));
                            CommandOutcome::done(format!("Created Conic (rho = {val:.3})."))
                        }
                        Err(e) => CommandOutcome::done(format!("Conic error: {e}")),
                    });
                }
                ActiveTool::Helix {
                    axis_start: _,
                    axis_end: None,
                    ..
                } if val.abs() >= 0.05 => {
                    if let ActiveTool::Helix { turns, .. } = &mut self.tool {
                        *turns = val.clamp(-100.0, 100.0);
                    }
                    return Some(CommandOutcome::msg(format!(
                        "Helix turns = {val}. {}",
                        self.prompt()
                    )));
                }
                ActiveTool::Helix {
                    axis_start: Some(a0),
                    axis_end: Some(a1),
                    turns,
                } if val > 1e-4 => {
                    let axis = (*a1 - *a0).normalize_or_zero();
                    let ref_v = if axis.z.abs() < 0.9 {
                        DVec3::Z
                    } else {
                        DVec3::X
                    };
                    let radial = ref_v.cross(axis).normalize_or_zero();
                    let pt = *a0 + radial * val;
                    let _ = turns;
                    return Some(self.on_point(pt, doc, cplane));
                }
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
            | ActiveTool::Distance { first: Some(s) }
            | ActiveTool::Dim { p1: Some(s), p2: None } => {
                vec![Geometry::Curve(NurbsCurve::line(*s, cur))]
            }
            ActiveTool::Dim {
                p1: Some(a),
                p2: Some(b),
            } => {
                let dir = (*b - *a).normalize_or_zero();
                let perp = cur - (*a + dir * (cur - *a).dot(dir));
                vec![
                    Geometry::Curve(NurbsCurve::line(*a, *a + perp)),
                    Geometry::Curve(NurbsCurve::line(*b, *b + perp)),
                    Geometry::Curve(NurbsCurve::line(*a + perp, *b + perp)),
                ]
            }
            ActiveTool::Polyline { points } if !points.is_empty() => {
                let mut pts = points.clone();
                pts.push(cur);
                NurbsCurve::polyline(&pts)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Curve {
                points,
                degree,
                closed,
            } if !points.is_empty() => {
                let mut pts = points.clone();
                pts.push(cur);
                let mut out = Vec::new();
                if let Ok(poly) = NurbsCurve::polyline(&pts) {
                    out.push(Geometry::Curve(poly));
                }
                if let Ok(c) = NurbsCurve::from_control_points(&pts, *degree, *closed) {
                    out.push(Geometry::Curve(c));
                }
                out
            }
            ActiveTool::InterpCrv {
                points,
                degree,
                closed,
            } if !points.is_empty() => {
                let mut pts = points.clone();
                pts.push(cur);
                NurbsCurve::interpolate(&pts, *degree, *closed)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::HandleCurve {
                anchors,
                pending_anchor,
            } => {
                let mut preview_anchors = anchors.clone();
                let mut guides = Vec::new();
                if let Some(a) = *pending_anchor {
                    let h_out = cur;
                    let h_in = a * 2.0 - h_out;
                    guides.push(Geometry::Curve(NurbsCurve::line(h_in, h_out)));
                    preview_anchors.push((a, h_in, h_out));
                } else if !anchors.is_empty() {
                    preview_anchors.push((cur, cur, cur));
                }
                if preview_anchors.len() >= 2 {
                    let segs: Vec<_> = preview_anchors
                        .windows(2)
                        .map(|w| (w[0].0, w[0].2, w[1].1, w[1].0))
                        .collect();
                    if let Ok(c) = NurbsCurve::handle_curve(&segs) {
                        guides.push(Geometry::Curve(c));
                    }
                }
                guides
            }
            ActiveTool::Conic { p0, p2, apex } => match (p0, p2, apex) {
                (Some(a), None, None) => vec![Geometry::Curve(NurbsCurve::line(*a, cur))],
                (Some(a), Some(b), None) => vec![
                    Geometry::Curve(NurbsCurve::line(*a, cur)),
                    Geometry::Curve(NurbsCurve::line(cur, *b)),
                ],
                (Some(a), Some(b), Some(ap)) => {
                    let mid = (*a + *b) * 0.5;
                    let leg = *ap - mid;
                    let rho = if leg.length_squared() > 1e-10 {
                        ((cur - mid).dot(leg) / leg.length_squared()).clamp(0.05, 0.95)
                    } else {
                        0.5
                    };
                    NurbsCurve::conic(*a, *ap, *b, rho)
                        .map(|c| vec![Geometry::Curve(c)])
                        .unwrap_or_default()
                }
                _ => Vec::new(),
            },
            ActiveTool::Helix {
                axis_start,
                axis_end,
                turns,
            } => match (axis_start, axis_end) {
                (Some(a0), None) => vec![Geometry::Curve(NurbsCurve::line(*a0, cur))],
                (Some(a0), Some(a1)) => {
                    let mut out = vec![Geometry::Curve(NurbsCurve::line(*a0, *a1))];
                    if let Ok(h) = NurbsCurve::helix(*a0, *a1, cur, *turns, None) {
                        out.push(Geometry::Curve(h));
                    }
                    out
                }
                _ => Vec::new(),
            },
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

    #[test]
    fn interactive_freeform_curve_interp_conic_helix_split_and_trim() {
        let mut doc = Document::new();
        let mut session = CommandSession::default();
        let plane = Plane::world_xy();

        // Curve with degree option
        session.start_tool("curve", &doc);
        session.on_text_input("d=2", &mut doc, &plane);
        for pt in [
            DVec3::ZERO,
            DVec3::new(2.0, 3.0, 0.0),
            DVec3::new(5.0, 0.0, 0.0),
        ] {
            session.on_point(pt, &mut doc, &plane);
        }
        let out = session.finish_or_cancel(&mut doc);
        assert!(out.finished);
        assert_eq!(doc.objects.len(), 1);
        let Geometry::Curve(c) = &doc.objects.values().next().unwrap().geometry else {
            panic!("expected curve");
        };
        assert_eq!(c.degree, 2);

        // InterpCrv
        session.start_tool("interpcrv", &doc);
        for pt in [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(3.0, 2.0, 0.0),
            DVec3::new(6.0, 0.0, 0.0),
        ] {
            session.on_point(pt, &mut doc, &plane);
        }
        session.finish_or_cancel(&mut doc);
        assert_eq!(doc.objects.len(), 2);

        // Conic
        session.start_tool("conic", &doc);
        session.on_point(DVec3::new(-2.0, 0.0, 0.0), &mut doc, &plane);
        session.on_point(DVec3::new(2.0, 0.0, 0.0), &mut doc, &plane);
        session.on_point(DVec3::new(0.0, 4.0, 0.0), &mut doc, &plane);
        let conic_out = session.on_text_input("0.5", &mut doc, &plane).unwrap();
        assert!(conic_out.finished);
        assert_eq!(doc.objects.len(), 3);

        // Helix
        session.start_tool("helix", &doc);
        session.on_point(DVec3::ZERO, &mut doc, &plane);
        session.on_point(DVec3::new(0.0, 0.0, 5.0), &mut doc, &plane);
        let helix_out = session.on_point(DVec3::new(2.0, 0.0, 0.0), &mut doc, &plane);
        assert!(helix_out.finished);
        assert_eq!(doc.objects.len(), 4);

        // SplitCurve & Trim
        let mut doc2 = Document::new();
        let line_a = doc2.add_object(
            "LineA",
            Geometry::Curve(NurbsCurve::line(
                DVec3::new(-5.0, 0.0, 0.0),
                DVec3::new(5.0, 0.0, 0.0),
            )),
        );
        let line_b = doc2.add_object(
            "LineB",
            Geometry::Curve(NurbsCurve::line(
                DVec3::new(0.0, -5.0, 0.0),
                DVec3::new(0.0, 5.0, 0.0),
            )),
        );
        doc2.set_selected(line_b, true);
        session.start_tool("trim", &doc2);
        let trim_out = session.on_point(DVec3::new(3.0, 0.0, 0.0), &mut doc2, &plane);
        assert!(!trim_out.finished);
        session.finish_or_cancel(&mut doc2);
        assert!(doc2.objects.get(line_a).is_none());
        assert_eq!(doc2.objects.len(), 2);

        doc2.select_all();
        session.start_tool("splitcurve", &doc2);
        let split_out = session.on_point(DVec3::new(-2.5, 0.0, 0.0), &mut doc2, &plane);
        assert!(split_out.finished);
        assert_eq!(doc2.objects.len(), 3);
    }
}
