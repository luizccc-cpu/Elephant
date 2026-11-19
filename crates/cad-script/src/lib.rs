//! Embedded local Rhai scripting, 2D Grasshopper-style node graph, and local MCP JSON-RPC bridge for Elephant.

pub mod engine;
pub mod graph;
pub mod mcp;

pub use engine::{DEFAULT_SAMPLE_SCRIPT, SCRIPT_PRESETS, ScriptError, ScriptResult, run_rhai_script};
pub use graph::{
    MathOpKind, NodeBooleanKind, NodeKind, NodeLink, ParametricGraph, ParametricNode, PortSpec,
    PortType,
};
pub use mcp::{
    DEFAULT_MCP_REQUEST, McpDispatchOutput, McpToolDescriptor, dispatch_mcp_json,
    mcp_tools_manifest, mcp_tools_manifest_json,
};
