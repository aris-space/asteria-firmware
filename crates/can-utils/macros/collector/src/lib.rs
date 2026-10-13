// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

//! This `Collector` derive macro can be used to set fields of a struct based on an enum.
//!
//! This reduces boilerplate when implementing message dispatching patterns for structs
//! containing the message's variants. It generates trait implementations that pattern-match on
//! enum message types and update the corresponding fields.
//!
//! This is particularly useful for driver code that receives CAN messages, telemetry updates,
//! or other enum-based message types and needs to dispatch them to the appropriate field.
//!
//! ## Basic Usage
//!
//! ```rust,ignore
//! enum TelemetryMessage {
//!     Temperature(f32),
//!     Pressure(f32),
//!     Humidity(f32),
//! }
//!
//! #[derive(Collector)]
//! #[collector(
//!     message_type = "TelemetryMessage",
//!     update_expr = "let _ = #field.update(#value);"
//! )]
//! struct HumiditySensor {
//!     #[collector(pattern = "TelemetryMessage::Humidity(#value)")]
//!     humidity: Watchable<f32>,
//! }
//!
//! #[derive(Collector)]
//! #[collector(
//!     message_type = "TelemetryMessage",
//!     update_expr = "let _ = #field.update(#value);"
//! )]
//! struct Sensors {
//!     #[collector(pattern = "TelemetryMessage::Temperature(#value)")]
//!     temperature: Watchable<f32>,
//!
//!     #[collector(pattern = "TelemetryMessage::Pressure(#value)")]
//!     pressure: Watchable<f32>,
//!
//!     // Also match on the collectors in `HumiditySensor`.
//!     #[collector(flatten)]
//!     humidity_sensor: HumiditySensor,
//! }
//!
//! // Usage:
//! let msg = TelemetryMessage::Humidity(65.0);
//! sensors.update_from(msg).expect("matched");
//! ```
//!
//! This generates something like:
//!
//! ```rust,ignore
//! impl Collector<TelemetryMessage> for Sensors {
//!     fn update_from(& self, msg: TelemetryMessage) -> Result<(), TelemetryMessage> {
//!         let msg = match msg {
//!             TelemetryMessage::Temperature(__value) => {
//!                 let _ = self.temperature.update(__value);
//!                 return Ok(());
//!             }
//!             TelemetryMessage::Pressure(__value) => {
//!                 let _ = self.pressure.update(__value);
//!                 return Ok(());
//!             }
//!             msg => msg,
//!         };
//!         let msg = match <HumiditySensor as Collector<TelemetryMessage>>::update_from(
//!             & self.humidity_sensor, msg,
//!         ) {
//!             Ok(()) => return Ok(()),
//!             Err(msg) => msg,
//!         };
//!         Err(msg)
//!     }
//! }
//! ```
//!
//! ## Explanation
//!
//! Each field with `#[collector(pattern = ...)]` generates a match arm. The `#value` placeholder
//! is replaced with `__value` which captures the matched value and makes it available in
//! `update_expr`. Match arms are generated in the order fields are declared in the struct.
//! Fields with `#[collector(flatten)]` are tried after all direct patterns, in declaration order.
//! Unmatched messages fall through to the final `Err(msg)` for composability.
//!
//! ## Struct Attributes
//!
//! `#[collector(message_type = ..., update_expr = "...")]`
//!
//! - `message_type` (required): The enum type to match against. Must be a valid type path.
//! - `update_expr` (required when direct `#[collector(pattern = ...)]` fields are present):
//!   A template expression executed when a match occurs.
//!   - `#field`: Replaced with `self.<field_name>`
//!   - `#value`: Replaced with the extracted value from the collector pattern
//!
//! ### Examples:
//!
//! ```rust,ignore
//! // Log updates
//! #[collector(
//!     message_type = "LoggedMessage",
//!     update_expr = r#"{
//!         tracing::debug!("Updating {} with {:?}", stringify!(#field), &#value);
//!         #field.update(#value);
//!     }"#
//! )]
//!
//! // Conditional updates
//! #[collector(
//!     message_type = "ValidatedMessage",
//!     update_expr = "if validate(&#value) { #field = #value; }"
//! )]
//! ```
//!
//! ## Field Attributes
//!
//! ### `#[collector(pattern = "...")]`
//!
//! Maps an enum variant to a field update. The pattern must contain `#value`,
//! whose captured value becomes available in `update_expr`.
//!
//! #### Examples:
//!
//! ```rust,ignore
//! // Simple extraction
//! #[collector(pattern = "Msg::Value(#value)")]
//!
//! // Nested enum
//! #[collector(pattern = "Msg::SystemA(system_a::Msg::Foo(#value))")]
//! ```
//!
//! ### `#[collector(flatten)]`
//!
//! Delegates unmatched messages to a nested field that itself implements `Collector<T>`.
//! After all direct patterns fail to match, flatten delegates are tried in field declaration
//! order. Must not be combined with `#[collector(pattern = ...)]` on the same field.
//!
//! ## Hygiene
//!
//! The macro generates a single impl block for `can_utils::collector::Collector<{message_type}>`.
//!
// The [`darling::FromVariant`] macro seems to have needless continues
#![allow(clippy::needless_continue)]

use darling::{FromDeriveInput, FromField, ast, util::Flag};
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, spanned::Spanned};

/// Struct-level attributes parsed from `#[collector(...)]`.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(collector), supports(struct_named))]
struct CollectorInput {
    ident: syn::Ident,
    data: ast::Data<(), CollectorField>,

    /// The enum type to match against (e.g., `"TelemetryMessage"`).
    message_type: syn::Type,

    /// Template expression executed when a match occurs.
    /// Use `#field` for `self.<field_name>` and `#value` for the captured value.
    /// Required when any direct `#[collector(pattern = ...)]` fields are present.
    #[darling(default)]
    update_expr: Option<String>,
}

/// Field-level attributes parsed from `#[collector(...)]` and `#[collector(...)]`.
#[derive(Debug, FromField)]
#[darling(attributes(collector))]
struct CollectorField {
    ident: Option<syn::Ident>,
    ty: syn::Type,

    /// The match pattern containing `#value` placeholder, from `#[collector(pattern = "...")]`.
    #[darling(default)]
    pattern: Option<String>,

    /// If true, delegate unmatched messages to this field's own `Collector` impl,
    /// from `#[collector(flatten)]`.
    flatten: Flag,
}

/// Derive macro for implementing the `Collector` trait.
///
/// See the crate-level documentation for usage examples.
#[proc_macro_derive(Collector, attributes(collector))]
pub fn derive_collector(input: TokenStream) -> TokenStream {
    let raw_input = parse_macro_input!(input as syn::DeriveInput);
    derive_collector_impl(&raw_input).into()
}

/// Inner implementation using proc_macro2 types (testable).
#[allow(clippy::too_many_lines)] // meep
fn derive_collector_impl(raw_input: &syn::DeriveInput) -> proc_macro2::TokenStream {
    let collector_input = match CollectorInput::from_derive_input(raw_input) {
        Ok(v) => v,
        Err(e) => return e.write_errors(),
    };

    let struct_name = &collector_input.ident;
    let message_type = &collector_input.message_type;

    // Extract fields from the parsed data
    let fields = collector_input
        .data
        .as_ref()
        .take_struct()
        .expect("darling restricted to struct_named")
        .fields;

    let has_direct = fields
        .iter()
        .any(|f| !f.flatten.is_present() && f.pattern.is_some());
    let has_flatten = fields.iter().any(|f| f.flatten.is_present());

    if !has_direct && !has_flatten {
        return darling::Error::custom(
            "at least one field must have a #[collector(pattern = ...)] or #[collector(flatten)] attribute",
        )
        .write_errors();
    }

    let mut error = darling::Error::accumulator();

    // Validate: flatten must not also carry a pattern.
    for field in &fields {
        if field.flatten.is_present() && field.pattern.is_some() {
            let field_name = field
                .ident
                .as_ref()
                .expect("darling restricted to struct_named");
            error.push(
                darling::Error::custom(format!(
                    "field `{field_name}` has #[collector(flatten)] and must not also have a #[collector(pattern = ...)]"
                ))
                .with_span(&field.ident.span()),
            );
        }
    }

    // Validate: update_expr required when direct pattern fields exist.
    if has_direct && collector_input.update_expr.is_none() {
        error.push(darling::Error::custom(
            "struct has fields with #[collector(pattern = ...)] but is missing \
             `update_expr` in #[collector(...)]",
        ));
    }

    if let Err(e) = error.finish() {
        return e.write_errors();
    }

    // --- Build direct match arms ---
    let mut match_arms: Vec<proc_macro2::TokenStream> = Vec::new();
    for field in &fields {
        if field.flatten.is_present() {
            continue;
        }
        let Some(collector_pattern) = &field.pattern else {
            continue;
        };
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");

        let pattern: proc_macro2::TokenStream =
            match collector_pattern.replace("#value", "__value").parse() {
                Ok(t) => t,
                Err(e) => {
                    return darling::Error::custom(format!("invalid pattern: {e}"))
                        .with_span(&field.ident.span())
                        .write_errors();
                }
            };

        let expr_template = collector_input
            .update_expr
            .as_ref()
            .expect("validated above to be Some");
        let update_code = expr_template
            .replace("#field", &format!("self.{field_name}"))
            .replace("#value", "__value");
        let update_tokens: proc_macro2::TokenStream = match update_code.parse() {
            Ok(t) => t,
            Err(e) => {
                return darling::Error::custom(format!("invalid update_expr: {e}")).write_errors();
            }
        };

        match_arms.push(quote! {
            #pattern => {
                #update_tokens
                return Ok(());
            }
        });
    }

    // --- Build body statements ---
    let mut stmts: Vec<proc_macro2::TokenStream> = Vec::new();

    if !match_arms.is_empty() {
        stmts.push(quote! {
            let msg = match msg {
                #(#match_arms)*
                msg => msg,
            };
        });
    }

    for field in fields.iter().filter(|f| f.flatten.is_present()) {
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");
        let field_ty = &field.ty;
        stmts.push(quote! {
            let msg = match <#field_ty as can_utils::collector::Collector<#message_type>>::update_from(
                & self.#field_name,
                msg,
            ) {
                Ok(()) => return Ok(()),
                Err(msg) => msg,
            };
        });
    }

    quote! {
        impl can_utils::collector::Collector<#message_type> for #struct_name {
            fn update_from(& self, msg: #message_type) -> Result<(), #message_type> {
                #(#stmts)*
                Err(msg)
            }
        }
    }
}

/// Tests the error cases in the outputted source.
/// Functional tests live in an integration test.
#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;

    #[test]
    fn test_error_on_no_collectors() {
        let input = quote! {
            #[collector(
                message_type = "Msg",
                update_expr = "#field = #value;"
            )]
            struct Empty {
                field: i32,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_collector_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("at least one field must have"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_flatten_with_pattern() {
        let input = quote! {
            #[collector(message_type = "Msg", update_expr = "#field = #value;")]
            struct Data {
                #[collector(pattern = "Msg::A(#value)")]
                #[collector(flatten)]
                inner: Inner,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_collector_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("must not also have a #[collector(pattern"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_update_expr() {
        let input = quote! {
            #[collector(message_type = "Msg")]
            struct Data {
                #[collector(pattern = "Msg::A(#value)")]
                a: i32,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_collector_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("missing `update_expr`"),
            "expected error in: {output_str}"
        );
    }
}
