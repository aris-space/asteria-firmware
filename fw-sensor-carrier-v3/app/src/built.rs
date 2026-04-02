#![allow(clippy::all)]
#![allow(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![allow(clippy::needless_raw_string_hashes)]
include!(concat!(env!("OUT_DIR"), "/built.rs"));
pub const ASTERIA_ARTIFACT_TIMESTAMP_MS: Option<&str> =
    option_env!("ASTERIA_ARTIFACT_TIMESTAMP_MS");
