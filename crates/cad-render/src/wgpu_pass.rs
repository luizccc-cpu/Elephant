use std::collections::HashMap;
use std::sync::Arc;

use cad_math::Mat4;
use eframe::egui;
use eframe::egui_wgpu::{self, wgpu};
use wgpu::util::DeviceExt;

use crate::camera::ViewportKind;
use crate::scene::{LineVertex, MeshVertex, RenderScene};

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct UniformData {
    view_proj: [[f32; 4]; 4],
}

struct ViewportUniform {
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// Shared `wgpu` pipelines, shared scene GPU buffers, and per-viewport uniforms.
pub struct CadWgpuState {
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    viewports: HashMap<u8, ViewportUniform>,
    uploaded_version: u64,
    mesh_vb: Option<wgpu::Buffer>,
    mesh_ib: Option<wgpu::Buffer>,
    mesh_index_count: u32,
    line_vb: Option<wgpu::Buffer>,
    line_vertex_count: u32,
}

impl CadWgpuState {
    pub fn new(
        device: &wgpu::Device,
        target_format: wgpu::TextureFormat,
        depth_format: Option<wgpu::TextureFormat>,
    ) -> Self {
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
                visibility: wgpu::ShaderStages::VERTEX,
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
                        2 => Float32x4
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
            multisample: wgpu::MultisampleState::default(),
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
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            mesh_pipeline,
            line_pipeline,
            bind_group_layout,
            viewports: HashMap::new(),
            uploaded_version: 0,
            mesh_vb: None,
            mesh_ib: None,
            mesh_index_count: 0,
            line_vb: None,
            line_vertex_count: 0,
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
    pub view_proj: Mat4,
    pub scene: Arc<RenderScene>,
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
        let entry = state.viewports.entry(key).or_insert_with(|| {
            let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_vp_uniform"),
                contents: bytemuck::bytes_of(&UniformData {
                    view_proj: self.view_proj.to_cols_array_2d(),
                }),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("cad_vp_bg"),
                layout: &state.bind_group_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: uniform_buffer.as_entire_binding(),
                }],
            });
            ViewportUniform {
                uniform_buffer,
                bind_group,
            }
        });

        queue.write_buffer(
            &entry.uniform_buffer,
            0,
            bytemuck::bytes_of(&UniformData {
                view_proj: self.view_proj.to_cols_array_2d(),
            }),
        );

        if state.uploaded_version != self.scene.version {
            state.uploaded_version = self.scene.version;
            if !self.scene.mesh_vertices.is_empty() && !self.scene.mesh_indices.is_empty() {
                state.mesh_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("cad_mesh_vb"),
                    contents: bytemuck::cast_slice(&self.scene.mesh_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                }));
                state.mesh_ib = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("cad_mesh_ib"),
                    contents: bytemuck::cast_slice(&self.scene.mesh_indices),
                    usage: wgpu::BufferUsages::INDEX,
                }));
                state.mesh_index_count = self.scene.mesh_indices.len() as u32;
            } else {
                state.mesh_index_count = 0;
            }

            if !self.scene.line_vertices.is_empty() {
                state.line_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("cad_line_vb"),
                    contents: bytemuck::cast_slice(&self.scene.line_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                }));
                state.line_vertex_count = self.scene.line_vertices.len() as u32;
            } else {
                state.line_vertex_count = 0;
            }
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
        let Some(vp) = state.viewports.get(&viewport_key(self.kind)) else {
            return;
        };

        if state.mesh_index_count > 0 {
            if let (Some(vb), Some(ib)) = (&state.mesh_vb, &state.mesh_ib) {
                render_pass.set_pipeline(&state.mesh_pipeline);
                render_pass.set_bind_group(0, &vp.bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..state.mesh_index_count, 0, 0..1);
            }
        }

        if state.line_vertex_count > 0 {
            if let Some(vb) = &state.line_vb {
                render_pass.set_pipeline(&state.line_pipeline);
                render_pass.set_bind_group(0, &vp.bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..state.line_vertex_count, 0..1);
            }
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

    #[test]
    fn wgpu_renders_starter_scene_pixels() {
        let instance = wgpu::Instance::default();
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
            .expect("adapter");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None))
            .expect("device");

        let format = wgpu::TextureFormat::Bgra8UnormSrgb;
        let state = CadWgpuState::new(&device, format, None);
        let mut resources = egui_wgpu::CallbackResources::new();
        resources.insert(state);

        let mut doc = Document::new();
        doc.add_object("Box", Geometry::Mesh(TriangleMesh::from_box(DVec3::ZERO, DVec3::splat(3.0))));
        let cam = ViewportCamera::preset(ViewportKind::Perspective);
        let scene = Arc::new(RenderScene::build(&doc, DisplayMode::Shaded, &cam.cplane, 1));
        let cb = ViewportPaintCallback {
            kind: ViewportKind::Perspective,
            view_proj: cam.view_proj_f32(1.0),
            scene,
        };

        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("offscreen"),
            size: wgpu::Extent3d { width: 128, height: 128, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
        let sd = egui_wgpu::ScreenDescriptor { size_in_pixels: [128, 128], pixels_per_point: 1.0 };

        let extra_cmds = cb.prepare(&device, &queue, &sd, &mut encoder, &mut resources);
        {
            let rp = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("test_rp"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
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
            tex.as_image_copy(),
            wgpu::ImageCopyBuffer {
                buffer: &buf,
                layout: wgpu::ImageDataLayout { offset: 0, bytes_per_row: Some(128 * 4), rows_per_image: Some(128) },
            },
            wgpu::Extent3d { width: 128, height: 128, depth_or_array_layers: 1 },
        );
        queue.submit(extra_cmds.into_iter().chain(std::iter::once(encoder.finish())));

        let slice = buf.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device.poll(wgpu::Maintain::Wait);
        let data = slice.get_mapped_range();
        let non_black = data.chunks_exact(4).filter(|px| px[0] > 0 || px[1] > 0 || px[2] > 0).count();
        eprintln!("offscreen non_black_pixels={non_black}");
        assert!(non_black > 50, "expected rendered pixels, got {non_black}");
    }
}
