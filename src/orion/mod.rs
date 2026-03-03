// Generated Orion protocol types (from OrionPublicProtocol.xml via build.rs).
// Rust 2024 edition has TryFrom/TryInto in the prelude; no explicit import needed.
#[allow(
    unused,
    non_snake_case,
    non_camel_case_types,
    dead_code,
    clippy::all,
    unused_variables,
    unused_mut,
    unreachable_patterns,
    irrefutable_let_patterns
)]
mod generated {
    include!(concat!(env!("OUT_DIR"), "/orion_generated.rs"));
}
pub use generated::*;

pub mod wire;
