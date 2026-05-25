//! Vendored from the `assign-resources` crate, patched so group-level attributes
//! (`$outer`, e.g. `#[cfg(...)]`) and field-level attributes (`$inner`) propagate
//! to *every* generated site: the `AssignedResources` field and the
//! `split_resources!` assignment, not just the group struct definition. That lets
//! a single resource group be `#[cfg]`-gated (e.g. swap bus2 between I2C4 and I2C2
//! by feature). The upstream macro only applied them to the struct definition,
//! which left the field/split references dangling under `#[cfg]`.

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
