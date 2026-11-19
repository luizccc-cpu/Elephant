struct Uniforms {
    view_proj: mat4x4<f32>,
    camera_pos: vec4<f32>,    // xyz = eye pos, w = cavity_ao_strength
    camera_dir: vec4<f32>,    // xyz = view forward dir, w = is_perspective (1/0)
    render_params: vec4<f32>, // x = shader_mode, y = p1, z = p2, w = extra
    draft_dir: vec4<f32>,     // xyz = pull dir, w = active_clip_count (0..4)
    clip_planes: array<vec4<f32>, 4>,
};

@group(0) @binding(0)
var<uniform> u: Uniforms;

struct MeshIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) curvature: vec2<f32>,
    @location(4) material: vec2<f32>,
};

struct MeshOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) curvature: vec2<f32>,
    @location(4) material: vec2<f32>,
};

@vertex
fn vs_mesh(in: MeshIn) -> MeshOut {
    var out: MeshOut;
    out.clip_pos = u.view_proj * vec4<f32>(in.position, 1.0);
    out.world_pos = in.position;
    out.normal = in.normal;
    out.color = in.color;
    out.curvature = in.curvature;
    out.material = in.material;
    return out;
}

fn should_clip(world_pos: vec3<f32>) -> bool {
    let count = i32(round(u.draft_dir.w));
    for (var i = 0; i < 4; i = i + 1) {
        if (i < count) {
            let p = u.clip_planes[i];
            if (dot(world_pos, p.xyz) + p.w < 0.0) {
                return true;
            }
        }
    }
    return false;
}

fn view_dir_to_cam(world_pos: vec3<f32>) -> vec3<f32> {
    if (u.camera_dir.w > 0.5) {
        return normalize(u.camera_pos.xyz - world_pos);
    }
    return normalize(-u.camera_dir.xyz);
}

fn aces_tonemap(x: vec3<f32>) -> vec3<f32> {
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), vec3<f32>(0.0), vec3<f32>(1.0));
}

fn curvature_ramp(t_raw: f32) -> vec3<f32> {
    let t = clamp(t_raw, 0.0, 1.0);
    let c0 = vec3<f32>(0.10, 0.25, 0.92); // blue (concave / min)
    let c1 = vec3<f32>(0.12, 0.78, 0.90); // cyan
    let c2 = vec3<f32>(0.18, 0.84, 0.36); // green (zero / flat)
    let c3 = vec3<f32>(0.96, 0.82, 0.16); // yellow
    let c4 = vec3<f32>(0.94, 0.18, 0.16); // red (convex / max)
    if (t < 0.25) {
        return mix(c0, c1, t / 0.25);
    } else if (t < 0.50) {
        return mix(c1, c2, (t - 0.25) / 0.25);
    } else if (t < 0.75) {
        return mix(c2, c3, (t - 0.50) / 0.25);
    }
    return mix(c3, c4, (t - 0.75) / 0.25);
}

fn ggx_eval(n: vec3<f32>, v: vec3<f32>, l: vec3<f32>, albedo: vec3<f32>, metallic: f32, roughness: f32) -> vec3<f32> {
    let n_dot_l = max(dot(n, l), 0.0);
    let n_dot_v = max(dot(n, v), 1e-4);
    if (n_dot_l <= 0.0) {
        return vec3<f32>(0.0);
    }
    let h = normalize(v + l);
    let n_dot_h = max(dot(n, h), 0.0);
    let v_dot_h = max(dot(v, h), 0.0);

    let r = clamp(roughness, 0.05, 1.0);
    let a = r * r;
    let a2 = a * a;
    let denom_d = (n_dot_h * n_dot_h * (a2 - 1.0) + 1.0);
    let ndf = a2 / max(3.14159265 * denom_d * denom_d, 1e-5);

    let k = ((r + 1.0) * (r + 1.0)) * 0.125;
    let g_v = n_dot_v / (n_dot_v * (1.0 - k) + k);
    let g_l = n_dot_l / (n_dot_l * (1.0 - k) + k);
    let g = g_v * g_l;

    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let fresnel = f0 + (vec3<f32>(1.0) - f0) * pow(clamp(1.0 - v_dot_h, 0.0, 1.0), 5.0);

    let spec = (ndf * g * fresnel) / max(4.0 * n_dot_v * n_dot_l, 1e-4);
    let kd = (vec3<f32>(1.0) - fresnel) * (1.0 - metallic);
    let diff = kd * albedo / 3.14159265;
    return (diff + spec) * n_dot_l;
}

@fragment
fn fs_mesh(in: MeshOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    let n_len = length(in.normal);
    var n = vec3<f32>(0.0, 0.0, 1.0);
    if (n_len > 1e-6) {
        n = in.normal / n_len;
    }

    // Screen-space curvature/cavity proxy computed in uniform control flow before discard.
    let fw_pos = max(length(fwidth(in.world_pos)), 1e-5);
    let fw_nrm = length(fwidth(n));
    let cavity_proxy = clamp(fw_nrm / (fw_pos * 1.6 + 0.06), 0.0, 1.0);
    let ao_strength = clamp(u.camera_pos.w, 0.0, 1.0);
    let cavity_ao = 1.0 - ao_strength * 0.42 * cavity_proxy;

    let v = view_dir_to_cam(in.world_pos);
    let r_raw = reflect(-v, select(-n, n, front));
    let zebra_freq = max(u.render_params.y, 1.0);
    let angle = u.render_params.z;
    let ca = cos(angle);
    let sa = sin(angle);
    let r_rot = vec3<f32>(r_raw.x * ca - r_raw.y * sa, r_raw.x * sa + r_raw.y * ca, r_raw.z);
    let zebra_phase = (r_rot.z * ca + r_rot.x * sa) * zebra_freq;
    let zebra_fw = max(fwidth(zebra_phase), 0.015);

    if (should_clip(in.world_pos)) {
        discard;
    }

    if (!front) {
        n = -n;
    }

    let r = reflect(-v, n);
    let mode = i32(round(u.render_params.x));
    let base_col = in.color.rgb;

    // 4: Class-A Zebra Stripes (reflection evaluation)
    if (mode == 4) {
        let wave = sin(zebra_phase * 3.14159265);
        let aa = smoothstep(-zebra_fw * 3.14159265, zebra_fw * 3.14159265, wave);
        let band = mix(0.06, 0.96, aa);
        let shade = 0.85 + 0.15 * max(dot(n, v), 0.0);
        return vec4<f32>(vec3<f32>(band * shade), in.color.a);
    }

    // 5: Procedural Class-A Environment Map (Studio Softbox / Chrome / Car Paint / Tubes)
    if (mode == 5) {
        let preset = i32(round(u.render_params.y));
        let elev = r.z * 0.5 + 0.5;
        let az = atan2(r.y, r.x);
        var env_col = vec3<f32>(0.15, 0.18, 0.22);
        if (preset == 0) {
            // Studio Softbox
            let band1 = smoothstep(0.55, 0.85, r.z);
            let band2 = smoothstep(0.75, 0.95, abs(sin(az * 2.0))) * smoothstep(0.1, 0.5, elev);
            let floor_g = mix(vec3<f32>(0.16, 0.18, 0.22), vec3<f32>(0.55, 0.60, 0.68), smoothstep(-0.2, 0.2, r.z));
            env_col = floor_g + vec3<f32>(0.95, 0.96, 0.98) * (band1 * 0.75 + band2 * 0.55);
        } else if (preset == 1) {
            // Liquid Chrome
            let horizon = exp(-abs(r.z) * 14.0);
            let sky = mix(vec3<f32>(0.22, 0.28, 0.38), vec3<f32>(0.92, 0.96, 1.0), smoothstep(0.0, 0.85, max(r.z, 0.0)));
            let ground = mix(vec3<f32>(0.08, 0.09, 0.11), vec3<f32>(0.25, 0.26, 0.28), smoothstep(-1.0, 0.0, r.z));
            let base_chrome = select(ground, sky, r.z >= 0.0);
            let glint = pow(max(sin(az * 3.0 + r.z * 2.5), 0.0), 8.0) * 0.65;
            env_col = base_chrome + vec3<f32>(1.0, 0.98, 0.94) * (horizon * 0.7 + glint);
        } else if (preset == 2) {
            // Glossy Car Paint
            let ndotv = clamp(dot(n, v), 0.0, 1.0);
            let clearcoat_f = pow(1.0 - ndotv, 4.0) * 0.75 + 0.08;
            let base_paint = mix(vec3<f32>(0.05, 0.22, 0.68), vec3<f32>(0.18, 0.56, 0.96), ndotv);
            let key_spec = pow(max(dot(r, normalize(vec3<f32>(0.45, 0.55, 0.70))), 0.0), 48.0);
            let rim_strip = smoothstep(0.6, 0.92, r.z) * 0.85;
            env_col = mix(base_paint * 0.9, vec3<f32>(0.98, 0.98, 1.0), clearcoat_f * (0.45 + rim_strip))
                + vec3<f32>(1.0) * key_spec;
        } else {
            // Fluorescent Tubes
            let tubes = pow(0.5 + 0.5 * cos(r.x * 16.0 + r.y * 8.0), 6.0);
            let warm_cool = mix(vec3<f32>(0.14, 0.16, 0.20), vec3<f32>(0.92, 0.97, 1.0), tubes);
            env_col = warm_cool * (0.45 + 0.55 * elev);
        }
        return vec4<f32>(aces_tonemap(env_col), in.color.a);
    }

    // 6: Gaussian Curvature, 7: Mean Curvature
    if (mode == 6 || mode == 7) {
        let k_val = select(in.curvature.x, in.curvature.y, mode == 7);
        let cmin = u.render_params.y;
        let cmax = max(u.render_params.z, cmin + 1e-5);
        let t = (k_val - cmin) / (cmax - cmin);
        let ramp = curvature_ramp(t);
        let shade = 0.52 + 0.48 * max(dot(n, normalize(vec3<f32>(0.4, 0.5, 0.75))), 0.0);
        return vec4<f32>(ramp * shade, in.color.a);
    }

    // 8: Draft Angle Analysis
    if (mode == 8) {
        let pull = normalize(u.draft_dir.xyz);
        let angle_deg = degrees(asin(clamp(dot(n, pull), -1.0, 1.0)));
        let thresh = max(u.render_params.y, 0.1);
        var dcol = vec3<f32>(0.96, 0.84, 0.16);
        if (angle_deg > thresh) {
            let k = clamp((angle_deg - thresh) / 45.0, 0.0, 1.0);
            dcol = mix(vec3<f32>(0.18, 0.76, 0.88), vec3<f32>(0.10, 0.40, 0.86), k);
        } else if (angle_deg < -thresh) {
            let k = clamp((-angle_deg - thresh) / 45.0, 0.0, 1.0);
            dcol = mix(vec3<f32>(0.95, 0.36, 0.16), vec3<f32>(0.85, 0.10, 0.14), k);
        }
        let shade = 0.55 + 0.45 * max(dot(n, v), 0.0);
        return vec4<f32>(dcol * shade, in.color.a);
    }

    // 1: Rendered (Cook-Torrance GGX PBR + Studio IBL + Cavity AO)
    if (mode == 1) {
        let metallic = clamp(in.material.x, 0.0, 1.0);
        let roughness = clamp(in.material.y, 0.05, 1.0);
        let l_key = normalize(vec3<f32>(0.45, 0.55, 0.70));
        let l_fill = normalize(vec3<f32>(-0.65, -0.25, 0.45));
        let l_rim = normalize(vec3<f32>(-0.20, 0.70, -0.35));

        var lo = ggx_eval(n, v, l_key, base_col, metallic, roughness) * vec3<f32>(2.4, 2.35, 2.25);
        lo += ggx_eval(n, v, l_fill, base_col, metallic, roughness) * vec3<f32>(0.75, 0.84, 1.0);
        lo += ggx_eval(n, v, l_rim, base_col, metallic, roughness) * vec3<f32>(0.62, 0.68, 0.78);

        let hemi = clamp(n.z * 0.5 + 0.5, 0.0, 1.0);
        let env_hemi = mix(vec3<f32>(0.18, 0.20, 0.24), vec3<f32>(0.78, 0.83, 0.90), hemi);
        let env_refl = mix(vec3<f32>(0.15, 0.17, 0.20), vec3<f32>(0.88, 0.92, 0.98), clamp(r.z * 0.5 + 0.5, 0.0, 1.0));
        let fresnel_amb = pow(1.0 - max(dot(n, v), 0.0), 5.0);
        let f0 = mix(vec3<f32>(0.04), base_col, metallic);
        let ambient = ((1.0 - metallic) * base_col * env_hemi * 0.42
            + mix(f0, vec3<f32>(1.0), fresnel_amb) * env_refl * (1.0 - roughness * 0.72) * 0.55)
            * cavity_ao;

        let final_rgb = aces_tonemap(ambient + lo);
        return vec4<f32>(final_rgb, in.color.a);
    }

    // 2: Concept (Gooch Studio Clay + Silhouette Rim + Cavity AO)
    if (mode == 2) {
        let l_key = normalize(vec3<f32>(0.42, 0.58, 0.68));
        let t = 0.5 + 0.5 * dot(n, l_key);
        let cool = vec3<f32>(0.16, 0.24, 0.42) + base_col * 0.24;
        let warm = vec3<f32>(0.88, 0.76, 0.58) * 0.38 + base_col * 0.62;
        let gooch = mix(cool, warm, t) * cavity_ao;
        let ndotv = clamp(dot(n, v), 0.0, 1.0);
        let spec = pow(max(dot(r, l_key), 0.0), 28.0) * 0.28;
        let sil = smoothstep(0.10, 0.35, ndotv);
        let col = mix(vec3<f32>(0.08, 0.09, 0.12), gooch + vec3<f32>(spec), sil);
        return vec4<f32>(col, in.color.a);
    }

    // 3: Arctic (Matte White Ambient Occlusion Studio)
    if (mode == 3) {
        let hemi = clamp(n.z * 0.5 + 0.5, 0.0, 1.0);
        let l1 = max(dot(n, normalize(vec3<f32>(0.3, 0.4, 0.85))), 0.0) * 0.32;
        let l2 = max(dot(n, normalize(vec3<f32>(-0.5, -0.4, 0.65))), 0.0) * 0.18;
        let occ = (0.48 + 0.26 * hemi + l1 + l2) * cavity_ao;
        let clay = mix(vec3<f32>(0.78, 0.80, 0.84), vec3<f32>(0.97, 0.98, 1.0), occ);
        return vec4<f32>(clay * base_col, in.color.a);
    }

    // 0: Standard Shaded / Ghosted / Technical
    let key_dir = normalize(vec3<f32>(0.45, 0.55, 0.70));
    let fill_dir = normalize(vec3<f32>(-0.6, -0.3, 0.4));
    let key = max(dot(n, key_dir), 0.0) * 0.65;
    let fill = max(dot(n, fill_dir), 0.0) * 0.25;
    let ambient = 0.28;
    var lit = base_col * (ambient + key + fill);
    if (!front) {
        lit = mix(lit, vec3<f32>(0.55, 0.32, 0.26), 0.35);
    }
    return vec4<f32>(lit, in.color.a);
}

struct LineIn {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct LineOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_line(in: LineIn) -> LineOut {
    var out: LineOut;
    out.clip_pos = u.view_proj * vec4<f32>(in.position, 1.0);
    out.world_pos = in.position;
    out.color = in.color;
    return out;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    if (should_clip(in.world_pos)) {
        discard;
    }
    return in.color;
}
