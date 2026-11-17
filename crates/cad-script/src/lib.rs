//! Embedded local Rhai scripting and Grasshopper-style parametric node graph for Elephant.

pub mod engine;
pub mod graph;

pub use engine::{DEFAULT_SAMPLE_SCRIPT, ScriptError, ScriptResult, run_rhai_script};
pub use graph::{NodeKind, NodeLink, ParametricGraph, ParametricNode};
