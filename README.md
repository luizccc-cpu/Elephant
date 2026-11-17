# Elephant

A clean-room, local-first, cross-platform 3D NURBS & mesh CAD application inspired by Rhinoceros 9's modeling workflows, built in Rust + `wgpu` + `egui`.

## Design Principles
- **Clean-room implementation:** Replicates CAD workflows and geometric behavior from first principles without copying proprietary code.
- **100% local:** Zero proprietary server, licensing, or telemetry connections.
- **Modern architecture:** Single native Rust binary with `wgpu` (DX12/Vulkan/Metal) rendering and `egui` immediate-mode CAD UI.

## Workspace Layout
- `crates/cad-math` — Double-precision math, transforms, rays, bounding boxes, and geometric tolerances
- `crates/cad-geom` — NURBS curves/surfaces, B-Rep topology, meshes, and adaptive tessellation
- `crates/cad-doc` — Generational object store, layers, selection, and transaction-based Undo/Redo
- `crates/cad-io` — `.obj`, `.stl`, and `.3dm` file I/O
- `crates/cad-commands` — Command parser, autocomplete, interactive state machine, OSnap & Ortho
- `crates/cad-render` — `wgpu` 4-viewport renderer, CPlane grids, Wireframe/Shaded modes, gizmos
- `apps/elephant-desktop` — Cross-platform desktop application
