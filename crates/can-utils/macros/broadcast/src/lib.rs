//! The `Broadcast` derive macro spawns an embassy CAN broadcast task for each annotated field.
//!
//! For each field annotated with `#[broadcast(map = ..., min_freq_hz = ..., max_freq_hz = ...)]`
//! this generates an `#[embassy_executor::task]` and a `can_utils::broadcast::Broadcast` impl
//! that spawns it.
//!
//! ## Basic Usage
//!
//! ```rust,ignore
//! #[derive(Broadcast)]
//! #[broadcast(loop_type = "MyLoop")]
//! struct Outputs {
//!     #[broadcast(filter_map = "#value.map(Msg::Positions)", min_freq_hz = 1., max_freq_hz = 10.)]
//!     positions: Watch<ThreadModeRawMutex, Option<Positions>, 2>,
//!
//!     // Fields without #[broadcast] are ignored.
//!     other: u32,
//!
//!     #[broadcast(flatten)]
//!     nested: NestedOutputs,
//! }
//! ```
//!
//! This generates something like:
//!
//! ```rust,ignore
//! impl can_utils::broadcast::Broadcast for Outputs {
//!     fn start_broadcasting(
//!         &'static self,
//!         spawner: Spawner,
//!         transmit: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
//!     ) -> Result<(), SpawnError> {
//!         #[embassy_executor::task]
//!         async fn positions(
//!             transmit: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
//!             field: Receiver<'static, ThreadModeRawMutex, Option<Positions>, 2>,
//!         ) {
//!             MyLoop::broadcast_loop(
//!                 field,
//!                 |__value| __value.map(Msg::Positions),
//!                 transmit,
//!                 1_f32,
//!                 10_f32,
//!             ).await;
//!         }
//!         spawner.spawn(positions(
//!             transmit,
//!             self.positions.receiver().ok_or(SpawnError::Busy)?,
//!         ))?;
//!         self.nested.start_broadcasting(spawner, transmit)?;
//!         Ok(())
//!     }
//! }
//! ```
//!
//! ## Struct Attributes
//!
//! `#[broadcast(loop_type = "TypePath")]`
//!
//! - `loop_type` (required): The `BroadcastLoop` implementor used for all generated tasks.
//!
//! ## Field Attributes
//!
//! - `#[broadcast(filter_map = "<expr>"]`: Expression `T -> Option<M>`. `#value` is replaced with the field
//!   value. Use this when the expression already produces an `Option`.
//! - `#[broadcast(map = "<expr>"]`: Shorthand for `filter_map` that wraps the result in `Some(...)`.
//!   `map = "Msg::Variant(#value)"` is equivalent to `filter_map = "Some(Msg::Variant(#value))"`.
//! - `#[broadcast(min_freq_hz = <float>]`: Hint for minimum broadcast frequency in Hz.
//! - `#[broadcast(max_freq_hz = <float>]`: Hint for maximum broadcast frequency in Hz.
//! - `#[broadcast(flatten)]`: Delegates to the field's own `Broadcast` impl.
//!   Must not be combined with any other `#[broadcast]` attribute.
//!
//! ## Field Type
//!
//! Each broadcast field must have type `Watch<MTX, T, N>`. The macro extracts `MTX`, `T`, and `N`
//! to generate correctly typed embassy tasks.
// The [`darling::FromVariant`] macro seems to have needless continues
#![allow(clippy::needless_continue)]

use darling::{FromDeriveInput, FromField, ast, util::Flag};
use proc_macro::TokenStream;
use proc_macro2::Literal;
use quote::quote;
use syn::{GenericArgument, PathArguments, Type, parse_macro_input, spanned::Spanned};

/// Struct-level attributes parsed from `#[broadcast(...)]`.
#[derive(Debug, FromDeriveInput)]
#[darling(attributes(broadcast), supports(struct_named))]
struct BroadcastInput {
    ident: syn::Ident,
    data: ast::Data<(), BroadcastField>,

    /// The `BroadcastLoop` implementor used in all generated tasks (e.g., `"MyLoop"`).
    loop_type: syn::Type,
}

/// Field-level attributes parsed from `#[broadcast(...)]`.
#[derive(Debug, FromField)]
#[darling(attributes(broadcast))]
struct BroadcastField {
    ident: Option<syn::Ident>,
    ty: syn::Type,

    /// Map expression `T -> M`. `#value` is replaced with the closure argument.
    /// Generates `|__value| Some(expr)`.
    map: Option<String>,

    /// Filter-map expression `T -> Option<M>`. `#value` is replaced with the closure argument.
    /// Generates `|__value| expr` directly.
    filter_map: Option<String>,

    /// Minimum broadcast frequency in Hz.
    min_freq_hz: Option<f32>,

    /// Maximum broadcast frequency in Hz.
    max_freq_hz: Option<f32>,

    /// If present, delegate to the field's own `Broadcast` impl.
    flatten: Flag,
}

/// Extracts `[MTX, T, N]` token streams from `Watch<MTX, T, N>`.
/// This is necessary, because embassy tasks cannot be generic.
fn extract_watch_type_args(ty: &Type) -> Option<[proc_macro2::TokenStream; 3]> {
    if let Type::Path(tp) = ty {
        let seg = tp.path.segments.last()?;
        if let PathArguments::AngleBracketed(ab) = &seg.arguments {
            let args: Vec<_> = ab.args.iter().collect();
            if args.len() == 3 {
                let mtx = match args[0] {
                    GenericArgument::Type(t) => quote!(#t),
                    _ => return None,
                };
                let t = match args[1] {
                    GenericArgument::Type(t) => quote!(#t),
                    _ => return None,
                };
                let n = match args[2] {
                    GenericArgument::Const(e) => quote!(#e),
                    _ => return None,
                };
                return Some([mtx, t, n]);
            }
        }
    }
    None
}

/// Derive macro for implementing the `Broadcast` trait.
///
/// See the crate-level documentation for usage examples.
#[proc_macro_derive(Broadcast, attributes(broadcast))]
pub fn derive_broadcast(input: TokenStream) -> TokenStream {
    let raw_input = parse_macro_input!(input as syn::DeriveInput);
    derive_broadcast_impl(&raw_input).into()
}

/// Inner implementation using proc_macro2 types (testable).
#[allow(clippy::too_many_lines)]
fn derive_broadcast_impl(raw_input: &syn::DeriveInput) -> proc_macro2::TokenStream {
    let broadcast_input = match BroadcastInput::from_derive_input(raw_input) {
        Ok(v) => v,
        Err(e) => return e.write_errors(),
    };

    let struct_name = &broadcast_input.ident;
    let loop_type = &broadcast_input.loop_type;

    let fields = broadcast_input
        .data
        .as_ref()
        .take_struct()
        .expect("darling restricted to struct_named")
        .fields;

    let mut error = darling::Error::accumulator();
    // Each element is one or two statements inside start_broadcasting: an inner task fn
    // definition followed by a spawn call, or a flatten delegation.
    let mut body_stmts: Vec<proc_macro2::TokenStream> = Vec::new();

    for field in &fields {
        let field_name = field
            .ident
            .as_ref()
            .expect("darling restricted to struct_named");

        if field.flatten.is_present() {
            if field.map.is_some()
                || field.filter_map.is_some()
                || field.min_freq_hz.is_some()
                || field.max_freq_hz.is_some()
            {
                error.push(
                    darling::Error::custom(format!(
                        "field `{field_name}` has #[broadcast(flatten)] \
                         and must not have map, filter_map, min_freq_hz, or max_freq_hz"
                    ))
                    .with_span(&field.ident.span()),
                );
                continue;
            }
            body_stmts.push(quote! {
                self.#field_name.start_broadcasting(__spawner, __transmit)?;
            });
            continue;
        }

        let (Some(min_freq_hz), Some(max_freq_hz)) = (field.min_freq_hz, field.max_freq_hz) else {
            if field.min_freq_hz.is_some() || field.max_freq_hz.is_some() {
                error.push(
                    darling::Error::custom(format!(
                        "field `{field_name}` has min_freq_hz or max_freq_hz but not both"
                    ))
                    .with_span(&field.ident.span()),
                );
            }
            // No broadcast annotation — skip field.
            continue;
        };

        let closure_body: proc_macro2::TokenStream = match (&field.map, &field.filter_map) {
            (Some(_), Some(_)) => {
                error.push(
                    darling::Error::custom(format!(
                        "field `{field_name}` has both map and filter_map — use only one"
                    ))
                    .with_span(&field.ident.span()),
                );
                continue;
            }
            (Some(map_expr), None) => {
                let body = map_expr.replace("#value", "__value");
                match body.parse::<proc_macro2::TokenStream>() {
                    Ok(ts) => quote! { ::core::option::Option::Some(#ts) },
                    Err(e) => {
                        error.push(
                            darling::Error::custom(format!("invalid map expression: {e}"))
                                .with_span(&field.ident.span()),
                        );
                        continue;
                    }
                }
            }
            (None, Some(fm_expr)) => {
                let body = fm_expr.replace("#value", "__value");
                match body.parse::<proc_macro2::TokenStream>() {
                    Ok(ts) => ts,
                    Err(e) => {
                        error.push(
                            darling::Error::custom(format!("invalid filter_map expression: {e}"))
                                .with_span(&field.ident.span()),
                        );
                        continue;
                    }
                }
            }
            (None, None) => {
                error.push(
                    darling::Error::custom(format!(
                        "field `{field_name}` has min_freq_hz/max_freq_hz but no map or filter_map"
                    ))
                    .with_span(&field.ident.span()),
                );
                continue;
            }
        };

        let Some(watch_args) = extract_watch_type_args(&field.ty) else {
            error.push(
                darling::Error::custom(format!(
                    "field `{field_name}` must have type Watch<MTX, T, N>"
                ))
                .with_span(&field.ident.span()),
            );
            continue;
        };
        let [mtx, t_ty, n_val] = watch_args;

        let min_lit = Literal::f32_suffixed(min_freq_hz);
        let max_lit = Literal::f32_suffixed(max_freq_hz);

        body_stmts.push(quote! {
            #[::embassy_executor::task]
            async fn #field_name(
                transmit: &'static ::embassy_sync::mutex::Mutex<#mtx, ::embassy_stm32::can::CanTx<'static>>,
                field: ::embassy_sync::watch::Receiver<'static, #mtx, #t_ty, #n_val>,
            ) {
                #loop_type::broadcast_loop(
                    field,
                    |__value| #closure_body,
                    transmit,
                    #min_lit,
                    #max_lit,
                )
                .await;
            }
            __spawner.spawn(#field_name(
                __transmit,
                self.#field_name.receiver().ok_or(::embassy_executor::SpawnError::Busy)?,
            )?);
        });
    }

    if let Err(e) = error.finish() {
        return e.write_errors();
    }

    if body_stmts.is_empty() {
        return darling::Error::custom(
            "at least one field must have a #[broadcast(...)] or #[broadcast(flatten)] attribute",
        )
        .write_errors();
    }

    quote! {
        impl ::can_utils::broadcast::Broadcast for #struct_name {
            fn start_broadcasting(
                &'static self,
                __spawner: ::embassy_executor::Spawner,
                __transmit: &'static ::embassy_sync::mutex::Mutex<
                    ::embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
                    ::embassy_stm32::can::CanTx<'static>,
                >,
            ) -> ::core::result::Result<(), ::embassy_executor::SpawnError> {
                use can_utils::broadcast::BroadcastLoop as _;
                #(#body_stmts)*
                ::core::result::Result::Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;

    #[test]
    fn test_error_on_flatten_with_freq() {
        let input = quote! {
            #[broadcast(loop_type = "MyLoop")]
            struct Data {
                #[broadcast(flatten, min_freq_hz = 1., max_freq_hz = 2.)]
                inner: Inner,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains(
                "field `inner` has #[broadcast(flatten)] and must not have map, filter_map, min_freq_hz, or max_freq_hz"
            ),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_no_broadcast_fields() {
        let input = quote! {
            #[broadcast(loop_type = "MyLoop")]
            struct Empty {
                field: i32,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains(
                "at least one field must have a #[broadcast(...)] or #[broadcast(flatten)] attribute"
            ),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_freq() {
        let input = quote! {
            #[broadcast(loop_type = "MyLoop")]
            struct Data {
                #[broadcast(map = "Msg::A(#value)", min_freq_hz = 1.)]
                a: Watch<ThreadModeRawMutex, i32, 1>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("has min_freq_hz or max_freq_hz but not both"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_missing_map() {
        let input = quote! {
            #[broadcast(loop_type = "MyLoop")]
            struct Data {
                #[broadcast(min_freq_hz = 1., max_freq_hz = 1.)]
                a: Watch<ThreadModeRawMutex, i32, 1>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("no map or filter_map"),
            "expected error in: {output_str}"
        );
    }

    #[test]
    fn test_error_on_both_map_and_filter_map() {
        let input = quote! {
            #[broadcast(loop_type = "MyLoop")]
            struct Data {
                #[broadcast(map = "Msg::A(#value)", filter_map = "#value.map(Msg::A)", min_freq_hz = 1., max_freq_hz = 1.)]
                a: Watch<ThreadModeRawMutex, i32, 1>,
            }
        };

        let parsed: syn::DeriveInput = syn::parse2(input).expect("failed to parse input");
        let output = derive_broadcast_impl(&parsed);
        let output_str = output.to_string();

        assert!(
            output_str.contains("both map and filter_map"),
            "expected error in: {output_str}"
        );
    }
}
