use std::collections::HashMap;
use std::sync::Arc;

use cad_math::Mat4;
use eframe::egui;
use eframe::egui_wgpu::{self, wgpu};
use wgpu::util::DeviceExt;

use crate::camera::ViewportKind;
use crate::scene::{
    DisplayMode, LineVertex, MeshVertex, RenderScene, SurfaceAnalysisConfig, SurfaceAnalysisMode,
};

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct UniformData {
    view_proj: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    camera_dir: [f32; 4],
    render_params: [f32; 4],
    draft_dir: [f32; 4],
    clip_planes: [[f32; 4]; 4],
}

struct ViewportUniform {
    doc_buffer: wgpu::Buffer,
    doc_bind_group: wgpu::BindGroup,
    aux_buffer: wgpu::Buffer,
    aux_bind_group: wgpu::BindGroup,
}

#[derive(Default)]
struct GpuBatch {
    initialized: bool,
    version: u64,
    mesh_vb: Option<wgpu::Buffer>,
    mesh_ib: Option<wgpu::Buffer>,
    mesh_index_count: u32,
    line_vb: Option<wgpu::Buffer>,
    line_vertex_count: u32,
}

impl GpuBatch {
    fn upload_if_needed(&mut self, device: &wgpu::Device, scene: &RenderScene) {
        if self.initialized && self.version == scene.version {
            return;
        }
        self.initialized = true;
        self.version = scene.version;

        if !scene.mesh_vertices.is_empty() && !scene.mesh_indices.is_empty() {
            self.mesh_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_mesh_vb"),
                contents: bytemuck::cast_slice(&scene.mesh_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            self.mesh_ib = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_mesh_ib"),
                contents: bytemuck::cast_slice(&scene.mesh_indices),
                usage: wgpu::BufferUsages::INDEX,
            }));
            self.mesh_index_count = scene.mesh_indices.len() as u32;
        } else {
            self.mesh_vb = None;
            self.mesh_ib = None;
            self.mesh_index_count = 0;
        }

        if !scene.line_vertices.is_empty() {
            self.line_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_line_vb"),
                contents: bytemuck::cast_slice(&scene.line_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            self.line_vertex_count = scene.line_vertices.len() as u32;
        } else {
            self.line_vb = None;
            self.line_vertex_count = 0;
        }
    }

    fn paint(
        &self,
        render_pass: &mut wgpu::RenderPass<'static>,
        mesh_pipeline: &wgpu::RenderPipeline,
        line_pipeline: &wgpu::RenderPipeline,
        vp_bind_group: &wgpu::BindGroup,
    ) {
        if self.mesh_index_count > 0 {
            if let (Some(vb), Some(ib)) = (&self.mesh_vb, &self.mesh_ib) {
                render_pass.set_pipeline(mesh_pipeline);
                render_pass.set_bind_group(0, vp_bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..self.mesh_index_count, 0, 0..1);
            }
        }
        if self.line_vertex_count > 0 {
            if let Some(vb) = &self.line_vb {
                render_pass.set_pipeline(line_pipeline);
                render_pass.set_bind_group(0, vp_bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..self.line_vertex_count, 0..1);
            }
        }
    }
}

/// Shared `wgpu` pipelines (MSAA-ready), per-mode document GPU buffers, per-viewport grids, and dynamic overlays.
pub struct CadWgpuState {
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    viewports: HashMap<u8, ViewportUniform>,
    mode_buffers: HashMap<DisplayMode, GpuBatch>,
    grid_buffers: HashMap<u8, GpuBatch>,
    overlay_buffers: GpuBatch,
    clipping_overlay_buffers: GpuBatch,
}

impl CadWgpuState {
    pub fn new(
        device: &wgpu::Device,
        target_format: wgpu::TextureFormat,
        depth_format: Option<wgpu::TextureFormat>,
        sample_count: u32,
    ) -> Self {
        let samples = sample_count.max(1);
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("elephant_cad_shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("cad_shader.wgsl").into()),
        });

        let depth_stencil = depth_format.map(|format| wgpu::DepthStencilState {
            format,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::LessEqual,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("cad_uniform_bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("cad_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let multisample = wgpu::MultisampleState {
            count: samples,
            mask: !0,
            alpha_to_coverage_enabled: false,
        };

        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cad_mesh_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_mesh",
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<MeshVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x3,
                        2 => Float32x4,
                        3 => Float32x2,
                        4 => Float32x2
                    ],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_mesh",
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: depth_stencil.clone(),
            multisample,
            multiview: None,
            cache: None,
        });

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cad_line_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_line",
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<LineVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x3,
                        1 => Float32x4
                    ],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_line",
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil,
            multisample,
            multiview: None,
            cache: None,
        });

        Self {
            mesh_pipeline,
            line_pipeline,
            bind_group_layout,
            viewports: HashMap::new(),
            mode_buffers: HashMap::new(),
            grid_buffers: HashMap::new(),
            overlay_buffers: GpuBatch::default(),
            clipping_overlay_buffers: GpuBatch::default(),
        }
    }
}

fn viewport_key(kind: ViewportKind) -> u8 {
    match kind {
        ViewportKind::Perspective => 0,
        ViewportKind::Top => 1,
        ViewportKind::Front => 2,
        ViewportKind::Right => 3,
    }
}

/// Per-viewport `egui_wgpu` paint callback.
pub struct ViewportPaintCallback {
    pub kind: ViewportKind,
    pub display_mode: DisplayMode,
    pub view_proj: Mat4,
    pub camera_pos: [f32; 3],
    pub camera_dir: [f32; 3],
    pub is_perspective: bool,
    pub analysis: SurfaceAnalysisConfig,
    pub clip_planes: [[f32; 4]; 4],
    pub clip_plane_count: u32,
    pub doc_scene: Arc<RenderScene>,
    pub grid_scene: Arc<RenderScene>,
    pub overlay_scene: Option<Arc<RenderScene>>,
    pub clipping_overlay: Option<Arc<RenderScene>>,
}

impl ViewportPaintCallback {
    fn build_uniforms(&self) -> (UniformData, UniformData) {
        let (shader_mode, p1, p2) = match self.analysis.mode {
            SurfaceAnalysisMode::Zebra => (
                4.0,
                self.analysis.zebra_stripes,
                self.analysis.zebra_angle_deg.to_radians(),
            ),
            SurfaceAnalysisMode::EnvironmentMap => (5.0, self.analysis.emap_preset as f32, 0.0),
            SurfaceAnalysisMode::GaussianCurvature => (
                6.0,
                self.analysis.curvature_min,
                self.analysis.curvature_max,
            ),
            SurfaceAnalysisMode::MeanCurvature => (
                7.0,
                self.analysis.curvature_min,
                self.analysis.curvature_max,
            ),
            SurfaceAnalysisMode::DraftAngle => (8.0, self.analysis.draft_angle_deg, 0.0),
            SurfaceAnalysisMode::None => match self.display_mode {
                DisplayMode::Rendered => (1.0, 0.0, 0.0),
                DisplayMode::Concept => (2.0, 0.0, 0.0),
                DisplayMode::Arctic => (3.0, 0.0, 0.0),
                DisplayMode::Wireframe
                | DisplayMode::Shaded
                | DisplayMode::Ghosted
                | DisplayMode::Technical => (0.0, 0.0, 0.0),
            },
        };
        let pull = self.analysis.draft_pull_dir.normalize_or(cad_math::DVec3::Z);
        let persp_flag = if self.is_perspective { 1.0 } else { 0.0 };
        let clip_cnt = self.clip_plane_count.min(4) as f32;
        let view_proj = self.view_proj.to_cols_array_2d();

        let doc_u = UniformData {
            view_proj,
            camera_pos: [
                self.camera_pos[0],
                self.camera_pos[1],
                self.camera_pos[2],
                self.analysis.cavity_ao_strength,
            ],
            camera_dir: [
                self.camera_dir[0],
                self.camera_dir[1],
                self.camera_dir[2],
                persp_flag,
            ],
            render_params: [shader_mode, p1, p2, 0.0],
            draft_dir: [pull.x as f32, pull.y as f32, pull.z as f32, clip_cnt],
            clip_planes: self.clip_planes,
        };

        let aux_u = UniformData {
            view_proj,
            camera_pos: [self.camera_pos[0], self.camera_pos[1], self.camera_pos[2], 0.0],
            camera_dir: [
                self.camera_dir[0],
                self.camera_dir[1],
                self.camera_dir[2],
                persp_flag,
            ],
            render_params: [0.0; 4],
            draft_dir: [0.0, 0.0, 1.0, 0.0],
            clip_planes: [[0.0; 4]; 4],
        };

        (doc_u, aux_u)
    }
}

impl egui_wgpu::CallbackTrait for ViewportPaintCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(state) = resources.get_mut::<CadWgpuState>() else {
            return Vec::new();
        };
        let key = viewport_key(self.kind);
        let (doc_u, aux_u) = self.build_uniforms();

        let entry = state.viewports.entry(key).or_insert_with(|| {
            let make_buf = |label, data: &UniformData| {
                let buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(label),
                    contents: bytemuck::bytes_of(data),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some(label),
                    layout: &state.bind_group_layout,
                    entries: &[wgpu::BindGroupEntry {
                        binding: 0,
                        resource: buf.as_entire_binding(),
                    }],
                });
                (buf, bg)
            };
            let (doc_buffer, doc_bind_group) = make_buf("cad_vp_doc_uniform", &doc_u);
            let (aux_buffer, aux_bind_group) = make_buf("cad_vp_aux_uniform", &aux_u);
            ViewportUniform {
                doc_buffer,
                doc_bind_group,
                aux_buffer,
                aux_bind_group,
            }
        });

        queue.write_buffer(&entry.doc_buffer, 0, bytemuck::bytes_of(&doc_u));
        queue.write_buffer(&entry.aux_buffer, 0, bytemuck::bytes_of(&aux_u));

        state
            .grid_buffers
            .entry(key)
            .or_default()
            .upload_if_needed(device, &self.grid_scene);
        state
            .mode_buffers
            .entry(self.display_mode)
            .or_default()
            .upload_if_needed(device, &self.doc_scene);
        if let Some(overlay) = &self.overlay_scene {
            state.overlay_buffers.upload_if_needed(device, overlay);
        }
        if let Some(clip_ov) = &self.clipping_overlay {
            state.clipping_overlay_buffers.upload_if_needed(device, clip_ov);
        }

        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        let Some(state) = resources.get::<CadWgpuState>() else {
            return;
        };
        let key = viewport_key(self.kind);
        let Some(vp) = state.viewports.get(&key) else {
            return;
        };

        if let Some(grid) = state.grid_buffers.get(&key) {
            grid.paint(
                render_pass,
                &state.mesh_pipeline,
                &state.line_pipeline,
                &vp.aux_bind_group,
            );
        }
        if let Some(doc_batch) = state.mode_buffers.get(&self.display_mode) {
            doc_batch.paint(
                render_pass,
                &state.mesh_pipeline,
                &state.line_pipeline,
                &vp.doc_bind_group,
            );
        }
        if self.clipping_overlay.is_some() {
            state.clipping_overlay_buffers.paint(
                render_pass,
                &state.mesh_pipeline,
                &state.line_pipeline,
                &vp.aux_bind_group,
            );
        }
        if self.overlay_scene.is_some() {
            state.overlay_buffers.paint(
                render_pass,
                &state.mesh_pipeline,
                &state.line_pipeline,
                &vp.aux_bind_group,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::ViewportCamera;
    use crate::scene::DisplayMode;
    use cad_doc::Document;
    use cad_geom::{DVec3, Geometry, TriangleMesh};
    use eframe::egui_wgpu::CallbackTrait;

    fn render_offscreen(
        mode: DisplayMode,
        analysis: SurfaceAnalysisConfig,
        clip_planes: [[f32; 4]; 4],
        clip_count: u32,
        sample_count: u32,
    ) -> usize {
        let instance = wgpu::Instance::default();
        let adapter =
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
                .expect("adapter");
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None))
                .expect("device");

        let format = wgpu::TextureFormat::Bgra8UnormSrgb;
        let state = CadWgpuState::new(&device, format, None, sample_count);
        let mut resources = egui_wgpu::CallbackResources::new();
        resources.insert(state);

        let mut doc = Document::new();
        doc.add_object(
            "Box",
            Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(3.0))),
        );
        let cam = ViewportCamera::preset(ViewportKind::Perspective);
        let (eye, _) = cam.eye_and_up();
        let fwd = (cam.target - eye).normalize_or_zero();
        let doc_scene = Arc::new(RenderScene::build_document(&doc, mode, 1));
        let grid_scene = Arc::new(RenderScene::build_grid(
            &cam.cplane,
            &doc.visible_bounding_box(),
            1,
        ));
        let cb = ViewportPaintCallback {
            kind: ViewportKind::Perspective,
            display_mode: mode,
            view_proj: cam.view_proj_f32(1.0),
            camera_pos: [eye.x as f32, eye.y as f32, eye.z as f32],
            camera_dir: [fwd.x as f32, fwd.y as f32, fwd.z as f32],
            is_perspective: cam.is_perspective,
            analysis,
            clip_planes,
            clip_plane_count: clip_count,
            doc_scene,
            grid_scene,
            overlay_scene: None,
            clipping_overlay: None,
        };

        let resolve_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("offscreen_resolve"),
            size: wgpu::Extent3d {
                width: 128,
                height: 128,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let resolve_view = resolve_tex.create_view(&wgpu::TextureViewDescriptor::default());
        let msaa_tex = (sample_count > 1).then(|| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some("offscreen_msaa"),
                size: wgpu::Extent3d {
                    width: 128,
                    height: 128,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
        });
        let msaa_view = msaa_tex
            .as_ref()
            .map(|t| t.create_view(&wgpu::TextureViewDescriptor::default()));

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        let sd = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [128, 128],
            pixels_per_point: 1.0,
        };

        let extra_cmds = cb.prepare(&device, &queue, &sd, &mut encoder, &mut resources);
        {
            let rp = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("test_rp"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: msaa_view.as_ref().unwrap_or(&resolve_view),
                    resolve_target: msaa_view.as_ref().map(|_| &resolve_view),
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            let mut rp = rp.forget_lifetime();
            let info = egui::PaintCallbackInfo {
                viewport: egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(128.0, 128.0)),
                clip_rect: egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(128.0, 128.0)),
                pixels_per_point: 1.0,
                screen_size_px: [128, 128],
            };
            cb.paint(info, &mut rp, &resources);
        }

        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size: (128 * 4 * 128) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            resolve_tex.as_image_copy(),
            wgpu::ImageCopyBuffer {
                buffer: &buf,
                layout: wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: Some(128 * 4),
                    rows_per_image: Some(128),
                },
            },
            wgpu::Extent3d {
                width: 128,
                height: 128,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(extra_cmds.into_iter().chain(std::iter::once(encoder.finish())));

        let slice = buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::Maintain::Wait);
        let data = slice.get_mapped_range();
        data.chunks_exact(4)
            .filter(|px| px[0] > 0 || px[1] > 0 || px[2] > 0)
            .count()
    }

    #[test]
    fn wgpu_renders_starter_scene_pixels() {
        let non_black = render_offscreen(
            DisplayMode::Shaded,
            SurfaceAnalysisConfig::default(),
            [[0.0; 4]; 4],
            0,
            1,
        );
        assert!(non_black > 50, "expected rendered pixels, got {non_black}");
    }

    #[test]
    fn wgpu_renders_msaa_pbr_zebra_and_gpu_clipping() {
        let msaa_pbr = render_offscreen(
            DisplayMode::Rendered,
            SurfaceAnalysisConfig::default(),
            [[0.0; 4]; 4],
            0,
            4,
        );
        assert!(msaa_pbr > 50, "expected 4x MSAA PBR pixels, got {msaa_pbr}");

        let zebra = render_offscreen(
            DisplayMode::Shaded,
            SurfaceAnalysisConfig {
                mode: SurfaceAnalysisMode::Zebra,
                ..Default::default()
            },
            [[0.0; 4]; 4],
            0,
            1,
        );
        assert!(zebra > 50, "expected Zebra pixels, got {zebra}");

        let draft = render_offscreen(
            DisplayMode::Shaded,
            SurfaceAnalysisConfig {
                mode: SurfaceAnalysisMode::DraftAngle,
                ..Default::default()
            },
            [[0.0; 4]; 4],
            0,
            1,
        );
        assert!(draft > 50, "expected DraftAngle pixels, got {draft}");

        // Half-space clipping plane z >= 10 clips the 3x3x3 centered box completely vs unclipped.
        let unclipped_concept = render_offscreen(
            DisplayMode::Concept,
            SurfaceAnalysisConfig::default(),
            [[0.0; 4]; 4],
            0,
            1,
        );
        let clipped_away = render_offscreen(
            DisplayMode::Concept,
            SurfaceAnalysisConfig::default(),
            [[0.0, 0.0, 1.0, -10.0], [0.0; 4], [0.0; 4], [0.0; 4]],
            1,
            1,
        );
        assert!(clipped_away < unclipped_concept, "expected clipping plane to reduce shaded pixels ({clipped_away} vs {unclipped_concept})");
    }

    #[test]
    fn wgpu_caches_separate_shaded_and_wireframe_mode_batches() {
        let instance = wgpu::Instance::default();
        let adapter =
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
                .expect("adapter");
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None))
                .expect("device");

        let format = wgpu::TextureFormat::Bgra8UnormSrgb;
        let state = CadWgpuState::new(&device, format, None, 1);
        let mut resources = egui_wgpu::CallbackResources::new();
        resources.insert(state);

        let mut doc = Document::new();
        doc.add_object(
            "Box",
            Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(2.0))),
        );
        let cam_persp = ViewportCamera::preset(ViewportKind::Perspective);
        let cam_top = ViewportCamera::preset(ViewportKind::Top);

        let doc_shaded = Arc::new(RenderScene::build_document(&doc, DisplayMode::Shaded, 1));
        let doc_wire = Arc::new(RenderScene::build_document(&doc, DisplayMode::Wireframe, 1));
        let grid_persp = Arc::new(RenderScene::build_grid(
            &cam_persp.cplane,
            &doc.visible_bounding_box(),
            1,
        ));
        let grid_top = Arc::new(RenderScene::build_grid(
            &cam_top.cplane,
            &doc.visible_bounding_box(),
            1,
        ));

        let cb1 = ViewportPaintCallback {
            kind: ViewportKind::Perspective,
            display_mode: DisplayMode::Shaded,
            view_proj: cam_persp.view_proj_f32(1.0),
            camera_pos: [10.0, -10.0, 10.0],
            camera_dir: [-0.5, 0.5, -0.5],
            is_perspective: true,
            analysis: SurfaceAnalysisConfig::default(),
            clip_planes: [[0.0; 4]; 4],
            clip_plane_count: 0,
            doc_scene: doc_shaded,
            grid_scene: grid_persp,
            overlay_scene: None,
            clipping_overlay: None,
        };
        let cb2 = ViewportPaintCallback {
            kind: ViewportKind::Top,
            display_mode: DisplayMode::Wireframe,
            view_proj: cam_top.view_proj_f32(1.0),
            camera_pos: [0.0, 0.0, 50.0],
            camera_dir: [0.0, 0.0, -1.0],
            is_perspective: false,
            analysis: SurfaceAnalysisConfig::default(),
            clip_planes: [[0.0; 4]; 4],
            clip_plane_count: 0,
            doc_scene: doc_wire,
            grid_scene: grid_top,
            overlay_scene: None,
            clipping_overlay: None,
        };

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        let sd = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [64, 64],
            pixels_per_point: 1.0,
        };
        cb1.prepare(&device, &queue, &sd, &mut encoder, &mut resources);
        cb2.prepare(&device, &queue, &sd, &mut encoder, &mut resources);

        let st = resources.get::<CadWgpuState>().unwrap();
        assert!(st.mode_buffers.contains_key(&DisplayMode::Shaded));
        assert!(st.mode_buffers.contains_key(&DisplayMode::Wireframe));
        assert!(st.mode_buffers[&DisplayMode::Shaded].mesh_index_count > 0);
        assert_eq!(st.mode_buffers[&DisplayMode::Wireframe].mesh_index_count, 0);
        assert!(st.mode_buffers[&DisplayMode::Wireframe].line_vertex_count > 0);
    }
}
