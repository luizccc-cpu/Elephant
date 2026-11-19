use std::collections::BTreeMap;

use cad_doc::Document;
use cad_geom::{ControlPoint, DVec3, Geometry, NurbsCurve, NurbsSurface, Plane, subd_to_nurbs_brep};
use cad_math::UnitSystem;

use crate::error::IoError;

struct IgesWriter {
    d_lines: Vec<String>,
    p_lines: Vec<String>,
}

impl IgesWriter {
    fn new() -> Self {
        Self {
            d_lines: Vec::new(),
            p_lines: Vec::new(),
        }
    }

    fn push_entity(&mut self, entity_type: usize, label: &str, params: &[String]) {
        let p_start = self.p_lines.len() + 1;
        let d_ptr = self.d_lines.len() + 1;

        let payload = format!("{entity_type},{};", params.join(","));
        let chunks = wrap_iges_param_payload(&payload, 64);
        let p_count = chunks.len();

        for chunk in chunks {
            let seq = self.p_lines.len() + 1;
            let line = format!("{chunk:<64} {d_ptr:>7}P{seq:>7}");
            self.p_lines.push(line);
        }

        let short_lbl: String = label.chars().filter(|c| c.is_ascii()).take(8).collect();
        let d_seq1 = self.d_lines.len() + 1;
        let d1 = format!(
            "{entity_type:>8}{p_start:>8}{:>8}{:>8}{:>8}{:>8}{:>8}{:>8}00000000D{d_seq1:>7}",
            0, 0, 0, 0, 0, 0
        );
        self.d_lines.push(d1);

        let d_seq2 = self.d_lines.len() + 1;
        let d2 = format!(
            "{entity_type:>8}{:>8}{:>8}{p_count:>8}{:>8}{:>8}{:>8}{short_lbl:>8}{:>8}D{d_seq2:>7}",
            0, 0, 0, 0, 0, 0
        );
        self.d_lines.push(d2);
    }

    fn emit_curve(&mut self, label: &str, curve: &NurbsCurve) {
        if curve.control_points.len() < 2 {
            return;
        }
        let k = curve.control_points.len() - 1;
        let m = curve.degree;
        let rational_flag = if curve
            .control_points
            .iter()
            .any(|c| (c.weight - 1.0).abs() > 1e-7)
        {
            0 // 0 = rational
        } else {
            1 // 1 = polynomial
        };
        let closed_flag = if curve.is_closed() { 1 } else { 0 };
        let mut params = vec![
            k.to_string(),
            m.to_string(),
            "0".to_string(), // planar unknown
            closed_flag.to_string(),
            rational_flag.to_string(),
            "0".to_string(), // non-periodic
        ];
        for &knot in &curve.knots {
            params.push(fmt_num(knot));
        }
        for cp in &curve.control_points {
            params.push(fmt_num(cp.weight));
        }
        for cp in &curve.control_points {
            params.push(fmt_num(cp.position.x));
            params.push(fmt_num(cp.position.y));
            params.push(fmt_num(cp.position.z));
        }
        let (u0, u1) = curve.domain();
        params.push(fmt_num(u0));
        params.push(fmt_num(u1));
        params.push("0.0".to_string());
        params.push("0.0".to_string());
        params.push("1.0".to_string());
        self.push_entity(126, label, &params);
    }

    fn emit_surface(&mut self, label: &str, srf: &NurbsSurface) {
        if srf.count_u < 2 || srf.count_v < 2 {
            return;
        }
        let k1 = srf.count_u - 1;
        let k2 = srf.count_v - 1;
        let m1 = srf.degree_u;
        let m2 = srf.degree_v;
        let rational_flag = if srf
            .control_points
            .iter()
            .any(|c| (c.weight - 1.0).abs() > 1e-7)
        {
            0
        } else {
            1
        };
        let mut params = vec![
            k1.to_string(),
            k2.to_string(),
            m1.to_string(),
            m2.to_string(),
            "0".to_string(),
            "0".to_string(),
            rational_flag.to_string(),
            "0".to_string(),
            "0".to_string(),
        ];
        for &ku in &srf.knots_u {
            params.push(fmt_num(ku));
        }
        for &kv in &srf.knots_v {
            params.push(fmt_num(kv));
        }
        // IGES 128 stores Fortran column-major (u fastest inner loop)
        for v in 0..srf.count_v {
            for u in 0..srf.count_u {
                params.push(fmt_num(srf.control_points[u * srf.count_v + v].weight));
            }
        }
        for v in 0..srf.count_v {
            for u in 0..srf.count_u {
                let p = srf.control_points[u * srf.count_v + v].position;
                params.push(fmt_num(p.x));
                params.push(fmt_num(p.y));
                params.push(fmt_num(p.z));
            }
        }
        let (u0, u1) = srf.domain_u();
        let (v0, v1) = srf.domain_v();
        params.push(fmt_num(u0));
        params.push(fmt_num(u1));
        params.push(fmt_num(v0));
        params.push(fmt_num(v1));
        self.push_entity(128, label, &params);
    }

    fn emit_point(&mut self, label: &str, p: DVec3) {
        let params = vec![
            fmt_num(p.x),
            fmt_num(p.y),
            fmt_num(p.z),
            "0".to_string(),
        ];
        self.push_entity(116, label, &params);
    }
}

/// Export visible CAD geometry in `doc` to strict 80-column fixed-width **IGES 5.3** ASCII.
pub fn export_iges_string(doc: &Document) -> String {
    let mut w = IgesWriter::new();
    for (idx, (_, obj)) in doc.objects.iter().enumerate() {
        if !obj.visible || !doc.is_layer_effectively_visible(obj.layer_id) {
            continue;
        }
        let label = obj
            .name
            .clone()
            .unwrap_or_else(|| format!("OBJ{}", idx + 1));
        match &obj.geometry {
            Geometry::Point(p) => w.emit_point(&label, *p),
            Geometry::Curve(c) => w.emit_curve(&label, c),
            Geometry::Surface(s) => w.emit_surface(&label, s),
            Geometry::BRep(b) => {
                for (fi, f) in b.faces.iter().enumerate() {
                    w.emit_surface(&format!("{label}_F{fi}"), &f.surface);
                }
            }
            Geometry::SubD(sd) => {
                if let Ok(brep) = subd_to_nurbs_brep(sd) {
                    for (fi, f) in brep.faces.iter().enumerate() {
                        w.emit_surface(&format!("{label}_S{fi}"), &f.surface);
                    }
                }
            }
            Geometry::Mesh(_) => {}
        }
    }

    let (unit_flag, unit_str) = match doc.units {
        UnitSystem::Inches => (1, "IN"),
        UnitSystem::Millimeters => (2, "MM"),
        UnitSystem::Feet => (4, "FT"),
        UnitSystem::Meters => (6, "M"),
        UnitSystem::Centimeters => (10, "CM"),
    };

    let s_lines = [format!(
        "{:<72}S{:>7}",
        "Elephant CAD IGES 5.3 Export", 1
    )];
    let g_payload = format!(
        "1H,,1H;,8HElephant,12Helephant.igs,8HElephant,3HELP,32,38,6,308,15,\
         8HElephant,1.0,{unit_flag},{unit_str},1,0.0,15H20260101.000000,\
         0.0001,10000.0,8HElephant,8HElephant,11,0,15H20260101.000000;"
    );
    let g_chunks = wrap_iges_param_payload(&g_payload, 72);
    let mut g_lines = Vec::with_capacity(g_chunks.len());
    for (idx, chunk) in g_chunks.into_iter().enumerate() {
        g_lines.push(format!("{chunk:<72}G{:>7}", idx + 1));
    }

    let s_cnt = s_lines.len();
    let g_cnt = g_lines.len();
    let d_cnt = w.d_lines.len();
    let p_cnt = w.p_lines.len();
    let t_body = format!("S{s_cnt:>7}G{g_cnt:>7}D{d_cnt:>7}P{p_cnt:>7}");
    let t_line = format!("{t_body:<72}T{:>7}", 1);

    let mut out = String::new();
    for l in s_lines {
        out.push_str(&l);
        out.push('\n');
    }
    for l in g_lines {
        out.push_str(&l);
        out.push('\n');
    }
    for l in w.d_lines {
        out.push_str(&l);
        out.push('\n');
    }
    for l in w.p_lines {
        out.push_str(&l);
        out.push('\n');
    }
    out.push_str(&t_line);
    out.push('\n');
    out
}

/// Import NURBS surfaces (Entity 128), NURBS curves (Entity 126), Lines (110), Arcs (100), and Points (116) from IGES.
pub fn import_iges_str(text: &str, doc: &mut Document) -> Result<usize, IoError> {
    let mut d_records: Vec<(usize, usize)> = Vec::new(); // (entity_type, de_ptr)
    let mut p_by_de: BTreeMap<usize, String> = BTreeMap::new();
    let mut has_iges = false;

    // Collect D-lines sequentially to recover 1-based DE indices (paired by 2 lines per DE)
    let mut d_line_idx = 0usize;
    for raw_line in text.lines() {
        if raw_line.len() < 73 {
            continue;
        }
        let sec = raw_line.as_bytes()[72] as char;
        match sec {
            'S' | 'G' | 'T' => {
                has_iges = true;
            }
            'D' => {
                has_iges = true;
                d_line_idx += 1;
                if d_line_idx % 2 == 1 {
                    let etype = raw_line[0..8].trim().parse::<usize>().unwrap_or(0);
                    d_records.push((etype, d_line_idx));
                }
            }
            'P' => {
                has_iges = true;
                let data = &raw_line[..64.min(raw_line.len())];
                let de_ptr = raw_line.get(65..72).and_then(|s| s.trim().parse::<usize>().ok());
                if let Some(ptr) = de_ptr {
                    p_by_de.entry(ptr).or_default().push_str(data.trim());
                }
            }
            _ => {}
        }
    }

    if !has_iges {
        return Err(IoError::InvalidFormat("unrecognized IGES file format".to_string()));
    }

    let mut added = 0usize;
    let mut points_buf: Vec<DVec3> = Vec::new();

    for (etype, de_ptr) in d_records {
        let Some(raw_params) = p_by_de.get(&de_ptr) else {
            continue;
        };
        let clean = raw_params.split(';').next().unwrap_or(raw_params);
        let tokens: Vec<f64> = clean
            .split(',')
            .filter_map(|t| {
                let s = t.trim().replace('D', "E").replace('d', "e");
                s.parse::<f64>().ok()
            })
            .collect();

        match etype {
            110 if tokens.len() >= 7 => {
                let p0 = DVec3::new(tokens[1], tokens[2], tokens[3]);
                let p1 = DVec3::new(tokens[4], tokens[5], tokens[6]);
                doc.add_object("ImportIGES_Line", Geometry::Curve(NurbsCurve::line(p0, p1)));
                added += 1;
            }
            100 if tokens.len() >= 8 => {
                let z = tokens[1];
                let cen = DVec3::new(tokens[2], tokens[3], z);
                let s = DVec3::new(tokens[4], tokens[5], z);
                let e = DVec3::new(tokens[6], tokens[7], z);
                let r = cen.distance(s).max(1e-4);
                let mut a0 = (s.y - cen.y).atan2(s.x - cen.x);
                let mut a1 = (e.y - cen.y).atan2(e.x - cen.x);
                if a1 <= a0 {
                    a1 += std::f64::consts::TAU;
                }
                if a0 < 0.0 {
                    a0 += std::f64::consts::TAU;
                    a1 += std::f64::consts::TAU;
                }
                let pl = Plane {
                    origin: cen,
                    x_axis: DVec3::X,
                    y_axis: DVec3::Y,
                    z_axis: DVec3::Z,
                };
                if let Ok(c) = NurbsCurve::arc_from_center(&pl, cen, r, a0, a1 - a0) {
                    doc.add_object("ImportIGES_Arc", Geometry::Curve(c));
                    added += 1;
                }
            }
            116 if tokens.len() >= 4 => {
                points_buf.push(DVec3::new(tokens[1], tokens[2], tokens[3]));
            }
            126 if tokens.len() >= 10 => {
                let k = tokens[1].round() as usize;
                let m = tokens[2].round() as usize;
                let n_cps = k + 1;
                let n_knots = k + m + 2;
                let idx_knots = 7;
                let idx_weights = idx_knots + n_knots;
                let idx_pts = idx_weights + n_cps;
                if tokens.len() >= idx_pts + n_cps * 3 {
                    let knots = tokens[idx_knots..idx_knots + n_knots].to_vec();
                    let mut cps = Vec::with_capacity(n_cps);
                    for i in 0..n_cps {
                        let w = tokens[idx_weights + i].max(1e-6);
                        let p = DVec3::new(
                            tokens[idx_pts + i * 3],
                            tokens[idx_pts + i * 3 + 1],
                            tokens[idx_pts + i * 3 + 2],
                        );
                        cps.push(ControlPoint::new(p, w));
                    }
                    if let Ok(c) = NurbsCurve::new(m, knots, cps) {
                        doc.add_object("ImportIGES_Curve", Geometry::Curve(c));
                        added += 1;
                    }
                }
            }
            128 if tokens.len() >= 14 => {
                let k1 = tokens[1].round() as usize;
                let k2 = tokens[2].round() as usize;
                let m1 = tokens[3].round() as usize;
                let m2 = tokens[4].round() as usize;
                let count_u = k1 + 1;
                let count_v = k2 + 1;
                let n_ku = k1 + m1 + 2;
                let n_kv = k2 + m2 + 2;
                let n_w = count_u * count_v;
                let idx_ku = 10;
                let idx_kv = idx_ku + n_ku;
                let idx_w = idx_kv + n_kv;
                let idx_pts = idx_w + n_w;
                if tokens.len() >= idx_pts + n_w * 3 {
                    let ku = tokens[idx_ku..idx_ku + n_ku].to_vec();
                    let kv = tokens[idx_kv..idx_kv + n_kv].to_vec();
                    let mut cps = vec![ControlPoint::unweighted(DVec3::ZERO); n_w];
                    for v in 0..count_v {
                        for u in 0..count_u {
                            let col_idx = v * count_u + u;
                            let w = tokens[idx_w + col_idx].max(1e-6);
                            let p = DVec3::new(
                                tokens[idx_pts + col_idx * 3],
                                tokens[idx_pts + col_idx * 3 + 1],
                                tokens[idx_pts + col_idx * 3 + 2],
                            );
                            cps[u * count_v + v] = ControlPoint::new(p, w);
                        }
                    }
                    if let Ok(srf) = NurbsSurface::new(m1, m2, ku, kv, count_u, count_v, cps) {
                        doc.add_object("ImportIGES_Surface", Geometry::Surface(srf));
                        added += 1;
                    }
                }
            }
            _ => {}
        }
    }

    for pt in points_buf {
        doc.add_object("ImportIGES_Point", Geometry::Point(pt));
        added += 1;
    }

    Ok(added)
}

fn fmt_num(v: f64) -> String {
    let s = format!("{v:.6}");
    let trimmed = s.trim_end_matches('0').trim_end_matches('.');
    if trimmed == "-0" || trimmed.is_empty() {
        "0".to_string()
    } else {
        trimmed.to_string()
    }
}

fn wrap_iges_param_payload(payload: &str, max_width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut cur = String::new();
    for token in payload.split_inclusive(',') {
        if cur.len() + token.len() <= max_width {
            cur.push_str(token);
        } else if token.len() <= max_width {
            if !cur.is_empty() {
                lines.push(std::mem::take(&mut cur));
            }
            cur.push_str(token);
        } else {
            for chunk in token.as_bytes().chunks(max_width) {
                let s = std::str::from_utf8(chunk).unwrap_or("");
                if cur.len() + s.len() > max_width && !cur.is_empty() {
                    lines.push(std::mem::take(&mut cur));
                }
                cur.push_str(s);
            }
        }
    }
    if !cur.is_empty() {
        lines.push(cur);
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iges_roundtrip_curve_and_surface() {
        let mut doc = Document::new();
        let circle = NurbsCurve::circle(&Plane::world_xy(), 2.5).unwrap();
        doc.add_object("C1", Geometry::Curve(circle.clone()));

        let sph = NurbsSurface::sphere(DVec3::new(5.0, 0.0, 0.0), 1.5).unwrap();
        doc.add_object("S1", Geometry::Surface(sph.clone()));

        let iges = export_iges_string(&doc);
        for line in iges.lines() {
            assert_eq!(line.len(), 80, "IGES line must be strictly 80 chars: {line}");
        }
        assert!(iges.contains("S      1"));
        assert!(iges.contains("T      1"));

        let mut doc2 = Document::new();
        let count = import_iges_str(&iges, &mut doc2).expect("import IGES");
        assert_eq!(count, 2);

        let bb1 = doc.visible_bounding_box();
        let bb2 = doc2.visible_bounding_box();
        assert!(bb1.min.distance(bb2.min) < 1e-4);
        assert!(bb1.max.distance(bb2.max) < 1e-4);
    }
}
