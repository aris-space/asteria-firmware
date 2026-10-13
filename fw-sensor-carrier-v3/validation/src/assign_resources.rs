//! Vendored from the `assign-resources` crate, patched to propagate attributes
//! (e.g. `#[cfg]`) to every generated site so a single resource group can be
//! feature-gated. Upstream applies them only to the group struct definition.

#[macro_export]
macro_rules! assign_resources {
    {
        $(
            $(#[$outer:meta])*
            $group_name:ident : $group_struct:ident {
                $(
                    $(#[$inner:meta])*
                    $resource_name:ident : $resource_field:ident $(=$resource_alias:ident)?),*
                $(,)?
            }
            $(,)?
        )+
    } => {
        #[allow(dead_code,non_snake_case,missing_docs)]
        pub struct AssignedResources {
            $(
                $(#[$outer])*
                pub $group_name : $group_struct
            ),*
        }
        $(
            #[allow(dead_code,non_snake_case)]
            $(#[$outer])*
            pub struct $group_struct {
                $(
                    $(#[$inner])*
                    pub $resource_name: Peri<'static, peripherals::$resource_field>
                ),*
            }
        )+

        $($($(
            #[allow(missing_docs)]
            pub type $resource_alias = Peri<'static, peripherals::$resource_field>;
        )?)*)*

        #[macro_export]
        /// `split_resources!` macro
        macro_rules! split_resources (
            ($p:ident) => {
                AssignedResources {
                    $(
                        $(#[$outer])*
                        $group_name: $group_struct {
                            $(
                                $(#[$inner])*
                                $resource_name: $p.$resource_field
                            ),*
                        }
                    ),*
                }
            }
        );
    }
}
