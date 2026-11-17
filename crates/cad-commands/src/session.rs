use cad_doc::Document;
use cad_geom::{DMat4, DVec3, Geometry, NurbsCurve, Plane, TriangleMesh};

#[derive(Debug, Clone, PartialEq)]
pub enum ActiveTool {
    Idle,
    Line { start: Option<DVec3> },
    Polyline { points: Vec<DVec3> },
    Circle { center: Option<DVec3> },
    Box { base: Option<DVec3>, corner: Option<DVec3> },
    Move { base: Option<DVec3> },
    Copy { base: Option<DVec3> },
    Rotate { center: Option<DVec3>, ref_vec: Option<DVec3> },
    Scale { origin: Option<DVec3>, ref_dist: Option<f64> },
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
            ActiveTool::Circle { center } => *center,
            ActiveTool::Box { base, .. } => *base,
            ActiveTool::Move { base } | ActiveTool::Copy { base } => *base,
            ActiveTool::Rotate { center, .. } => *center,
            ActiveTool::Scale { origin, .. } => *origin,
            ActiveTool::Idle => None,
        }
    }

    pub fn prompt(&self) -> &'static str {
        match &self.tool {
            ActiveTool::Idle => "Command",
            ActiveTool::Line { start: None } => "Start of line (click or x,y,z)",
            ActiveTool::Line { start: Some(_) } => "End of line (click, x,y,z, or distance)",
            ActiveTool::Polyline { points } if points.is_empty() => "Start of polyline",
            ActiveTool::Polyline { .. } => "Next point of polyline (Right-click or Enter to finish)",
            ActiveTool::Circle { center: None } => "Center of circle",
            ActiveTool::Circle { center: Some(_) } => "Radius (click or enter number)",
            ActiveTool::Box { base: None, .. } => "First corner of box base",
            ActiveTool::Box { corner: None, .. } => "Other corner of box base",
            ActiveTool::Box { .. } => "Height of box (click or enter number)",
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
            "circle" => {
                self.tool = ActiveTool::Circle { center: None };
                Some(self.prompt().to_string())
            }
            "box" => {
                self.tool = ActiveTool::Box {
                    base: None,
                    corner: None,
                };
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
                ActiveTool::Circle { center: Some(c) } => {
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
            ActiveTool::Line { start: Some(s) } => vec![Geometry::Curve(NurbsCurve::line(*s, cur))],
            ActiveTool::Polyline { points } if !points.is_empty() => {
                let mut pts = points.clone();
                pts.push(cur);
                NurbsCurve::polyline(&pts)
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Circle { center: Some(c) } => {
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
            ActiveTool::Box {
                base: Some(b),
                corner: None,
            } => {
                let p1 = DVec3::new(cur.x, b.y, b.z);
                let p2 = DVec3::new(cur.x, cur.y, b.z);
                let p3 = DVec3::new(b.x, cur.y, b.z);
                NurbsCurve::polyline(&[*b, p1, p2, p3, *b])
                    .map(|c| vec![Geometry::Curve(c)])
                    .unwrap_or_default()
            }
            ActiveTool::Move { base: Some(b) } | ActiveTool::Copy { base: Some(b) } => {
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
