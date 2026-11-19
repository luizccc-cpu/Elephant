use eframe::egui;

use crate::state::{ElephantApp, ToolPaletteTab};

#[derive(Clone, Copy)]
struct PaletteTool {
    label: &'static str,
    cmd: &'static str,
    icon: &'static str,
    tip: &'static str,
}

impl ElephantApp {
    pub fn draw_tool_palette(&mut self, ctx: &egui::Context) {
        if !self.show_left_palette {
            return;
        }
        let mut trigger_cmd: Option<&'static str> = None;

        egui::SidePanel::left("tool_palette")
            .default_width(142.0)
            .min_width(128.0)
            .max_width(185.0)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.spacing_mut().item_spacing = egui::vec2(2.0, 2.0);
                    for tab in ToolPaletteTab::ALL {
                        let active = self.palette_tab == tab;
                        if ui
                            .selectable_label(active, tab.short_label())
                            .on_hover_text(tab.title())
                            .clicked()
                        {
                            self.palette_tab = tab;
                        }
                    }
                });
                ui.separator();
                ui.strong(self.palette_tab.title());
                ui.add_space(4.0);

                let tools = tools_for_tab(self.palette_tab);
                egui::ScrollArea::vertical().show(ui, |ui| {
                    egui::Grid::new(("palette_grid", self.palette_tab as usize))
                        .num_columns(2)
                        .spacing([4.0, 4.0])
                        .show(ui, |ui| {
                            for (i, t) in tools.iter().enumerate() {
                                if draw_icon_tile(ui, t) {
                                    trigger_cmd = Some(t.cmd);
                                }
                                if i % 2 == 1 {
                                    ui.end_row();
                                }
                            }
                        });
                });
            });

        if let Some(cmd) = trigger_cmd {
            self.execute_command(cmd);
        }
    }
}

fn draw_icon_tile(ui: &mut egui::Ui, tool: &PaletteTool) -> bool {
    let size = egui::vec2(58.0, 46.0);
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
    let hovered = response.hovered();
    let fill = if response.is_pointer_button_down_on() {
        egui::Color32::from_rgb(56, 88, 132)
    } else if hovered {
        egui::Color32::from_rgb(52, 58, 68)
    } else {
        egui::Color32::from_rgb(40, 44, 51)
    };
    let stroke_col = if hovered {
        egui::Color32::from_rgb(100, 165, 240)
    } else {
        egui::Color32::from_rgb(66, 72, 82)
    };
    let painter = ui.painter_at(rect);
    painter.rect(rect, 4.0, fill, egui::Stroke::new(1.0_f32, stroke_col));

    let glyph_center = egui::pos2(rect.center().x, rect.top() + 16.0);
    draw_vector_glyph(&painter, glyph_center, tool.icon, hovered);

    let text_col = if hovered {
        egui::Color32::WHITE
    } else {
        egui::Color32::from_rgb(218, 224, 232)
    };
    painter.text(
        egui::pos2(rect.center().x, rect.bottom() - 8.0),
        egui::Align2::CENTER_CENTER,
        tool.label,
        egui::FontId::proportional(10.5),
        text_col,
    );

    response.on_hover_text(format!("{} ({})\n{}", tool.label, tool.cmd, tool.tip)).clicked()
}

fn draw_vector_glyph(painter: &egui::Painter, c: egui::Pos2, kind: &str, hot: bool) {
    let fg = if hot {
        egui::Color32::from_rgb(130, 210, 255)
    } else {
        egui::Color32::from_rgb(195, 212, 232)
    };
    let accent = egui::Color32::from_rgb(245, 190, 85);
    let s = egui::Stroke::new(1.5_f32, fg);
    let sa = egui::Stroke::new(1.5_f32, accent);

    match kind {
        "line" => {
            painter.line_segment([c + egui::vec2(-11.0, 6.0), c + egui::vec2(11.0, -6.0)], s);
            painter.circle_filled(c + egui::vec2(-11.0, 6.0), 2.0, accent);
            painter.circle_filled(c + egui::vec2(11.0, -6.0), 2.0, accent);
        }
        "poly" => {
            let pts = vec![
                c + egui::vec2(-11.0, 6.0),
                c + egui::vec2(-3.0, -7.0),
                c + egui::vec2(5.0, 4.0),
                c + egui::vec2(11.0, -5.0),
            ];
            painter.add(egui::Shape::line(pts, s));
        }
        "curve" => {
            let pts = vec![
                c + egui::vec2(-11.0, 5.0),
                c + egui::vec2(-4.0, -8.0),
                c + egui::vec2(4.0, 8.0),
                c + egui::vec2(11.0, -5.0),
            ];
            painter.add(egui::Shape::line(pts, s));
            painter.circle_filled(c + egui::vec2(-4.0, -8.0), 1.8, accent);
            painter.circle_filled(c + egui::vec2(4.0, 8.0), 1.8, accent);
        }
        "rect" => {
            painter.rect_stroke(egui::Rect::from_center_size(c, egui::vec2(20.0, 14.0)), 1.0, s);
        }
        "circle" => {
            painter.circle_stroke(c, 8.5, s);
        }
        "arc" => {
            let pts: Vec<egui::Pos2> = (0..=8)
                .map(|i| {
                    let a = -0.2 - (i as f32 / 8.0) * 2.5;
                    c + egui::vec2(a.cos() * 9.0, a.sin() * 9.0 + 3.0)
                })
                .collect();
            painter.add(egui::Shape::line(pts, s));
        }
        "box" => {
            let r = egui::Rect::from_center_size(c + egui::vec2(-2.0, 2.0), egui::vec2(14.0, 12.0));
            painter.rect_stroke(r, 0.0, s);
            let off = egui::vec2(5.0, -5.0);
            painter.line_segment([r.left_top(), r.left_top() + off], s);
            painter.line_segment([r.right_top(), r.right_top() + off], s);
            painter.line_segment([r.right_bottom(), r.right_bottom() + off], s);
            painter.line_segment([r.left_top() + off, r.right_top() + off], s);
            painter.line_segment([r.right_top() + off, r.right_bottom() + off], s);
        }
        "cyl" => {
            let top = c + egui::vec2(0.0, -6.0);
            let bot = c + egui::vec2(0.0, 6.0);
            painter.circle_stroke(top, 6.0, s);
            painter.line_segment([top + egui::vec2(-6.0, 0.0), bot + egui::vec2(-6.0, 0.0)], s);
            painter.line_segment([top + egui::vec2(6.0, 0.0), bot + egui::vec2(6.0, 0.0)], s);
            painter.line_segment([bot + egui::vec2(-6.0, 0.0), bot + egui::vec2(6.0, 0.0)], s);
        }
        "loft" => {
            painter.line_segment([c + egui::vec2(-10.0, -6.0), c + egui::vec2(10.0, -6.0)], sa);
            painter.line_segment([c + egui::vec2(-10.0, 6.0), c + egui::vec2(10.0, 6.0)], sa);
            painter.line_segment([c + egui::vec2(-8.0, -6.0), c + egui::vec2(-8.0, 6.0)], s);
            painter.line_segment([c + egui::vec2(0.0, -6.0), c + egui::vec2(0.0, 6.0)], s);
            painter.line_segment([c + egui::vec2(8.0, -6.0), c + egui::vec2(8.0, 6.0)], s);
        }
        "bool" => {
            painter.circle_stroke(c + egui::vec2(-3.5, 0.0), 6.5, s);
            painter.circle_stroke(c + egui::vec2(3.5, 0.0), 6.5, sa);
        }
        "move" => {
            painter.line_segment([c + egui::vec2(-10.0, 0.0), c + egui::vec2(10.0, 0.0)], s);
            painter.line_segment([c + egui::vec2(0.0, -9.0), c + egui::vec2(0.0, 9.0)], s);
        }
        "rot" => {
            painter.circle_stroke(c, 7.5, s);
            painter.circle_filled(c + egui::vec2(7.5, 0.0), 2.5, accent);
        }
        "fillet" => {
            painter.line_segment([c + egui::vec2(-10.0, 6.0), c + egui::vec2(0.0, 6.0)], s);
            painter.line_segment([c + egui::vec2(10.0, -4.0), c + egui::vec2(10.0, 6.0)], s);
            painter.circle_stroke(c + egui::vec2(4.0, 1.0), 5.0, sa);
        }
        "dim" => {
            painter.line_segment([c + egui::vec2(-9.0, 6.0), c + egui::vec2(-9.0, -6.0)], s);
            painter.line_segment([c + egui::vec2(9.0, 6.0), c + egui::vec2(9.0, -6.0)], s);
            painter.line_segment([c + egui::vec2(-9.0, 0.0), c + egui::vec2(9.0, 0.0)], sa);
        }
        _ => {
            painter.rect_stroke(
                egui::Rect::from_center_size(c, egui::vec2(14.0, 14.0)),
                3.0,
                sa,
            );
        }
    }
}

fn tools_for_tab(tab: ToolPaletteTab) -> &'static [PaletteTool] {
    match tab {
        ToolPaletteTab::Standard => &[
            PaletteTool { label: "Line", cmd: "Line", icon: "line", tip: "Draw a line segment" },
            PaletteTool { label: "Polyline", cmd: "Polyline", icon: "poly", tip: "Multi-segment polyline" },
            PaletteTool { label: "InterpCrv", cmd: "InterpCrv", icon: "curve", tip: "Interpolated smooth NURBS curve" },
            PaletteTool { label: "Circle", cmd: "Circle", icon: "circle", tip: "Center-radius NURBS circle" },
            PaletteTool { label: "Rectangle", cmd: "Rectangle", icon: "rect", tip: "Closed NURBS rectangle" },
            PaletteTool { label: "Box", cmd: "Box", icon: "box", tip: "Solid B-Rep box" },
            PaletteTool { label: "Cylinder", cmd: "Cylinder", icon: "cyl", tip: "Solid B-Rep cylinder" },
            PaletteTool { label: "Extrude", cmd: "Extrude", icon: "loft", tip: "Extrude curves into solids/surfaces" },
            PaletteTool { label: "Join", cmd: "Join", icon: "bool", tip: "Join curves or surfaces into B-Rep" },
            PaletteTool { label: "Explode", cmd: "Explode", icon: "poly", tip: "Explode curves or B-Reps" },
            PaletteTool { label: "Trim", cmd: "Trim", icon: "fillet", tip: "Trim curves or surfaces" },
            PaletteTool { label: "Split", cmd: "Split", icon: "fillet", tip: "Split curves or faces" },
            PaletteTool { label: "Move", cmd: "Move", icon: "move", tip: "Move selected objects" },
            PaletteTool { label: "Rotate", cmd: "Rotate", icon: "rot", tip: "Rotate selected objects" },
            PaletteTool { label: "BoolDiff", cmd: "BooleanDifference", icon: "bool", tip: "B-Rep Boolean Difference" },
            PaletteTool { label: "ZoomExt", cmd: "ZoomExtents", icon: "rect", tip: "Fit all visible geometry in view" },
        ],
        ToolPaletteTab::Curves => &[
            PaletteTool { label: "Line", cmd: "Line", icon: "line", tip: "Two-point line" },
            PaletteTool { label: "Polyline", cmd: "Polyline", icon: "poly", tip: "Open/closed polyline" },
            PaletteTool { label: "Curve", cmd: "Curve", icon: "curve", tip: "Control-point NURBS curve" },
            PaletteTool { label: "InterpCrv", cmd: "InterpCrv", icon: "curve", tip: "Curve through fit points" },
            PaletteTool { label: "HandleCrv", cmd: "HandleCurve", icon: "curve", tip: "Bezier handle curve" },
            PaletteTool { label: "Rectangle", cmd: "Rectangle", icon: "rect", tip: "2-corner rectangle" },
            PaletteTool { label: "Circle", cmd: "Circle", icon: "circle", tip: "Exact rational circle" },
            PaletteTool { label: "Arc", cmd: "Arc", icon: "arc", tip: "3-point rational arc" },
            PaletteTool { label: "Ellipse", cmd: "Ellipse", icon: "circle", tip: "Planar NURBS ellipse" },
            PaletteTool { label: "Polygon", cmd: "Polygon", icon: "rect", tip: "Regular N-sided polygon" },
            PaletteTool { label: "Helix", cmd: "Helix", icon: "arc", tip: "3D helical curve" },
            PaletteTool { label: "Offset", cmd: "Offset", icon: "curve", tip: "Offset curve in CPlane" },
            PaletteTool { label: "Fillet", cmd: "Fillet", icon: "fillet", tip: "Fillet 2 curves" },
            PaletteTool { label: "Chamfer", cmd: "Chamfer", icon: "fillet", tip: "Chamfer 2 curves" },
            PaletteTool { label: "BlendCrv", cmd: "BlendCrv", icon: "curve", tip: "G0-G3 blend between curves" },
            PaletteTool { label: "Elmo", cmd: "RebuildAdaptive", icon: "curve", tip: "Adaptive Elmo rebuild" },
        ],
        ToolPaletteTab::Surfaces => &[
            PaletteTool { label: "Plane", cmd: "Plane", icon: "rect", tip: "Planar NURBS surface" },
            PaletteTool { label: "PlanarSrf", cmd: "PlanarSrf", icon: "rect", tip: "Trimmed surface from planar loops" },
            PaletteTool { label: "Extrude", cmd: "Extrude", icon: "loft", tip: "Extrude curve normal to CPlane" },
            PaletteTool { label: "Loft", cmd: "Loft", icon: "loft", tip: "Loft across section curves" },
            PaletteTool { label: "Revolve", cmd: "Revolve", icon: "rot", tip: "Revolve profile around axis" },
            PaletteTool { label: "RailRev", cmd: "RailRevolve", icon: "rot", tip: "Revolve along rail curve" },
            PaletteTool { label: "Sweep1", cmd: "Sweep1", icon: "loft", tip: "1-rail sweep surface" },
            PaletteTool { label: "Sweep2", cmd: "Sweep2", icon: "loft", tip: "2-rail sweep surface" },
            PaletteTool { label: "EdgeSrf", cmd: "EdgeSrf", icon: "loft", tip: "Coons surface from 2-4 edges" },
            PaletteTool { label: "NetworkSrf", cmd: "NetworkSrf", icon: "loft", tip: "Gordon network surface" },
            PaletteTool { label: "BlendSrf", cmd: "BlendSrf", icon: "fillet", tip: "Smooth blend between surfaces" },
            PaletteTool { label: "Patch", cmd: "Patch", icon: "loft", tip: "Constrained patch fit" },
            PaletteTool { label: "FilletSrf", cmd: "FilletSrf", icon: "fillet", tip: "Rolling-ball surface fillet" },
            PaletteTool { label: "MatchSrf", cmd: "MatchSrf", icon: "fillet", tip: "Match surface edge continuity" },
            PaletteTool { label: "OffsetSrf", cmd: "OffsetSrf", icon: "loft", tip: "Offset NURBS surface" },
            PaletteTool { label: "ExtendSrf", cmd: "ExtendSrf", icon: "loft", tip: "Extend surface edge" },
        ],
        ToolPaletteTab::Solids => &[
            PaletteTool { label: "Box", cmd: "Box", icon: "box", tip: "Interactive B-Rep solid box" },
            PaletteTool { label: "Cylinder", cmd: "Cylinder", icon: "cyl", tip: "Interactive B-Rep cylinder" },
            PaletteTool { label: "Cone", cmd: "Cone", icon: "cyl", tip: "Interactive B-Rep cone" },
            PaletteTool { label: "Sphere", cmd: "Sphere", icon: "circle", tip: "Exact NURBS sphere" },
            PaletteTool { label: "Torus", cmd: "Torus", icon: "circle", tip: "Exact NURBS torus" },
            PaletteTool { label: "Union", cmd: "BooleanUnion", icon: "bool", tip: "B-Rep Boolean Union" },
            PaletteTool { label: "Diff", cmd: "BooleanDifference", icon: "bool", tip: "B-Rep Boolean Difference" },
            PaletteTool { label: "Intersect", cmd: "BooleanIntersection", icon: "bool", tip: "B-Rep Boolean Intersection" },
            PaletteTool { label: "BoolSplit", cmd: "BooleanSplit", icon: "bool", tip: "Split target solid by cutter" },
            PaletteTool { label: "Cap", cmd: "Cap", icon: "box", tip: "Cap planar holes to close solid" },
            PaletteTool { label: "FilletEdge", cmd: "FilletEdge", icon: "fillet", tip: "Fillet solid edges (Ctrl+Shift+Click)" },
            PaletteTool { label: "ChamferEd", cmd: "ChamferEdge", icon: "fillet", tip: "Chamfer solid edges" },
            PaletteTool { label: "Shell", cmd: "Shell", icon: "box", tip: "Hollow solid removing selected faces" },
            PaletteTool { label: "PushPull", cmd: "PushPull", icon: "move", tip: "Push/pull planar B-Rep face" },
            PaletteTool { label: "Inset", cmd: "Inset", icon: "rect", tip: "Inset face border inward" },
            PaletteTool { label: "Untrim", cmd: "Untrim", icon: "rect", tip: "Remove trim loops from B-Rep" },
        ],
        ToolPaletteTab::SubD => &[
            PaletteTool { label: "SubDBox", cmd: "SubDBox", icon: "box", tip: "Catmull-Clark SubD box cage" },
            PaletteTool { label: "SubDSph", cmd: "SubDSphere", icon: "circle", tip: "Catmull-Clark SubD sphere" },
            PaletteTool { label: "SubDCyl", cmd: "SubDCylinder", icon: "cyl", tip: "SubD cylinder with creased rims" },
            PaletteTool { label: "SubDTorus", cmd: "SubDTorus", icon: "circle", tip: "Smooth SubD torus" },
            PaletteTool { label: "ToSubD", cmd: "ToSubD", icon: "bool", tip: "Convert Mesh/BRep/Surface to SubD" },
            PaletteTool { label: "Subdivide", cmd: "Subdivide", icon: "loft", tip: "Subdivide SubD control cage" },
            PaletteTool { label: "Crease", cmd: "Crease", icon: "line", tip: "Toggle sharp crease on boundary/top ring" },
            PaletteTool { label: "ExtrudeF", cmd: "SubDExtrudeFace", icon: "move", tip: "Extrude face of SubD cage" },
            PaletteTool { label: "ToNURBS", cmd: "ToNURBS", icon: "loft", tip: "Convert SubD into watertight B-Rep NURBS" },
            PaletteTool { label: "MatchSubD", cmd: "MatchSubD", icon: "fillet", tip: "Match open SubD edge to Surface/Curve" },
            PaletteTool { label: "ShrinkWrp", cmd: "ShrinkWrap", icon: "bool", tip: "Voxel remesh objects into watertight solid mesh" },
            PaletteTool { label: "PointsOn", cmd: "PointsOn", icon: "curve", tip: "Edit SubD control cage vertices" },
            PaletteTool { label: "NodeEdit", cmd: "NodeEditor", icon: "loft", tip: "Open 2D Grasshopper Node Canvas" },
            PaletteTool { label: "AI / MCP", cmd: "McpBridge", icon: "dim", tip: "Open local AI / MCP JSON-RPC bridge" },
        ],
        ToolPaletteTab::Transform => &[
            PaletteTool { label: "Move", cmd: "Move", icon: "move", tip: "Move selected objects" },
            PaletteTool { label: "Copy", cmd: "Copy", icon: "move", tip: "Copy selected objects" },
            PaletteTool { label: "Rotate", cmd: "Rotate", icon: "rot", tip: "Rotate around CPlane axis" },
            PaletteTool { label: "Scale", cmd: "Scale", icon: "rect", tip: "Scale 3D from origin" },
            PaletteTool { label: "Mirror", cmd: "Mirror", icon: "line", tip: "Mirror across CPlane line (History)" },
            PaletteTool { label: "ArrayLin", cmd: "ArrayLinear", icon: "box", tip: "Linear pattern array" },
            PaletteTool { label: "ArrayPol", cmd: "ArrayPolar", icon: "rot", tip: "Circular polar array" },
            PaletteTool { label: "PointsOn", cmd: "PointsOn", icon: "curve", tip: "Show NURBS control points (F10)" },
            PaletteTool { label: "PointsOff", cmd: "PointsOff", icon: "curve", tip: "Hide control points (F11)" },
            PaletteTool { label: "UVN", cmd: "UVN", icon: "move", tip: "Toggle UVN Gumball on CPs" },
            PaletteTool { label: "SoftXform", cmd: "SoftTransform", icon: "curve", tip: "Toggle soft NURBS falloff" },
            PaletteTool { label: "AutoCPln", cmd: "AutoCPlane", icon: "rect", tip: "Align CPlane to selected face/curve" },
            PaletteTool { label: "ResetGmb", cmd: "GumballReset", icon: "move", tip: "Reset relocated Gumball pivot" },
            PaletteTool { label: "History", cmd: "History", icon: "rot", tip: "Toggle modeling history recording" },
        ],
        ToolPaletteTab::Analysis => &[
            PaletteTool { label: "Zebra", cmd: "Zebra", icon: "curve", tip: "Zebra reflection stripes (Class-A smoothness)" },
            PaletteTool { label: "E-Map", cmd: "EMap", icon: "circle", tip: "Environment map reflection shader" },
            PaletteTool { label: "Curvature", cmd: "GaussianCurvature", icon: "loft", tip: "False-color Gaussian curvature" },
            PaletteTool { label: "Draft", cmd: "DraftAngle", icon: "cyl", tip: "Mold pull draft angle & undercuts" },
            PaletteTool { label: "GEC", cmd: "GlobalEdgeContinuity", icon: "bool", tip: "Rhino 9 Global Edge Continuity (G0/G1/G2)" },
            PaletteTool { label: "Mass", cmd: "MassProperties", icon: "box", tip: "Volume, area, centroid & inertia tensor" },
            PaletteTool { label: "ShowEdges", cmd: "ShowEdges", icon: "box", tip: "Highlight naked B-Rep edges" },
            PaletteTool { label: "ClipPlane", cmd: "ClippingPlane", icon: "rect", tip: "Add live GPU clipping plane" },
            PaletteTool { label: "Section", cmd: "Section", icon: "line", tip: "Bake section contours on clipping/CPlane" },
            PaletteTool { label: "CurvGraph", cmd: "CurvatureGraph", icon: "curve", tip: "Show curvature comb on curves" },
            PaletteTool { label: "Distance", cmd: "Distance", icon: "dim", tip: "Measure distance and delta" },
            PaletteTool { label: "Area", cmd: "Area", icon: "rect", tip: "Measure area & solid volume" },
        ],
        ToolPaletteTab::Drafting => &[
            PaletteTool { label: "Make2D", cmd: "Make2D", icon: "box", tip: "Hidden-line 2D projection on XY" },
            PaletteTool { label: "Layout", cmd: "Layout", icon: "rect", tip: "Open 4-view paper-space sheet" },
            PaletteTool { label: "GDT", cmd: "GdtFrame", icon: "dim", tip: "Place GD&T feature control frame" },
            PaletteTool { label: "Datum", cmd: "DatumTag", icon: "rect", tip: "Place datum target tag (A/B/C)" },
            PaletteTool { label: "PDF", cmd: "ExportPDF", icon: "rect", tip: "Export technical sheet to Vector PDF" },
            PaletteTool { label: "Dim", cmd: "Dim", icon: "dim", tip: "Linear dimension annotation" },
            PaletteTool { label: "TextDot", cmd: "TextDot", icon: "rect", tip: "Always-readable screen pin note" },
            PaletteTool { label: "Text", cmd: "Text", icon: "dim", tip: "Place text label at CPlane origin" },
            PaletteTool { label: "DupBorder", cmd: "DupBorder", icon: "rect", tip: "Duplicate naked border curves" },
            PaletteTool { label: "Export", cmd: "Export", icon: "box", tip: "Export STEP / IGES / 3DM / 3MF / OBJ / STL / glTF / DXF / SVG / PDF" },
            PaletteTool { label: "STEP", cmd: "ExportSTEP", icon: "box", tip: "Export ISO-10303-21 STEP AP214" },
            PaletteTool { label: "IGES", cmd: "ExportIGES", icon: "rect", tip: "Export IGES 5.3 NURBS" },
            PaletteTool { label: "3DM", cmd: "Export3DM", icon: "box", tip: "Export Rhino .3dm archive" },
            PaletteTool { label: "3MF", cmd: "Export3MF", icon: "box", tip: "Export 3MF 3D-print package" },
        ],
    }
}
