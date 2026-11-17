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

struct ViewportBuffers {
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    mesh_vb: Option<wgpu::Buffer>,
    mesh_ib: Option<wgpu::Buffer>,
    mesh_index_count: u32,
    line_vb: Option<wgpu::Buffer>,
    line_vertex_count: u32,
}

/// Shared `wgpu` pipelines and per-viewport buffers stored in `egui_wgpu::CallbackResources`.
pub struct CadWgpuState {
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    viewports: HashMap<u8, ViewportBuffers>,
}

impl CadWgpuState {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("elephant_cad_shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("cad_shader.wgsl").into()),
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
            depth_stencil: None,
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
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            mesh_pipeline,
            line_pipeline,
            bind_group_layout,
            viewports: HashMap::new(),
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
            ViewportBuffers {
                uniform_buffer,
                bind_group,
                mesh_vb: None,
                mesh_ib: None,
                mesh_index_count: 0,
                line_vb: None,
                line_vertex_count: 0,
            }
        });

        queue.write_buffer(
            &entry.uniform_buffer,
            0,
            bytemuck::bytes_of(&UniformData {
                view_proj: self.view_proj.to_cols_array_2d(),
            }),
        );

        if !self.scene.mesh_vertices.is_empty() && !self.scene.mesh_indices.is_empty() {
            entry.mesh_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_mesh_vb"),
                contents: bytemuck::cast_slice(&self.scene.mesh_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            entry.mesh_ib = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_mesh_ib"),
                contents: bytemuck::cast_slice(&self.scene.mesh_indices),
                usage: wgpu::BufferUsages::INDEX,
            }));
            entry.mesh_index_count = self.scene.mesh_indices.len() as u32;
        } else {
            entry.mesh_index_count = 0;
        }

        if !self.scene.line_vertices.is_empty() {
            entry.line_vb = Some(device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("cad_line_vb"),
                contents: bytemuck::cast_slice(&self.scene.line_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            entry.line_vertex_count = self.scene.line_vertices.len() as u32;
        } else {
            entry.line_vertex_count = 0;
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

        if vp.line_vertex_count > 0 {
            if let Some(vb) = &vp.line_vb {
                render_pass.set_pipeline(&state.line_pipeline);
                render_pass.set_bind_group(0, &vp.bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.draw(0..vp.line_vertex_count, 0..1);
            }
        }

        if vp.mesh_index_count > 0 {
            if let (Some(vb), Some(ib)) = (&vp.mesh_vb, &vp.mesh_ib) {
                render_pass.set_pipeline(&state.mesh_pipeline);
                render_pass.set_bind_group(0, &vp.bind_group, &[]);
                render_pass.set_vertex_buffer(0, vb.slice(..));
                render_pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..vp.mesh_index_count, 0, 0..1);
            }
        }
    }
}
