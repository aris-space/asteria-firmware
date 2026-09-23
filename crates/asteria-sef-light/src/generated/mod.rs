//! Development-time generated numerical kernels.

#[allow(
    non_snake_case,
    clippy::doc_markdown,
    clippy::empty_line_after_outer_attr,
    clippy::trivially_copy_pass_by_ref,
    clippy::used_underscore_binding
)]
mod barometer_0_model;
#[allow(
    non_snake_case,
    clippy::doc_markdown,
    clippy::empty_line_after_outer_attr,
    clippy::trivially_copy_pass_by_ref,
    clippy::used_underscore_binding
)]
mod barometer_1_model;
#[allow(
    non_snake_case,
    clippy::doc_markdown,
    clippy::empty_line_after_outer_attr,
    clippy::trivially_copy_pass_by_ref,
    clippy::used_underscore_binding
)]
mod gnss_model;
#[allow(
    non_snake_case,
    clippy::doc_markdown,
    clippy::empty_line_after_outer_attr,
    clippy::trivially_copy_pass_by_ref,
    clippy::used_underscore_binding
)]
mod vertical_process_model;

pub(crate) use barometer_0_model::generated::barometer_0_model;
pub(crate) use barometer_1_model::generated::barometer_1_model;
pub(crate) use gnss_model::generated::gnss_model;
pub(crate) use vertical_process_model::generated::vertical_process_model;
