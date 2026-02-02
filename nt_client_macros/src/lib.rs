//! Derive macros for the `nt_client` crate.
//!
//! Currently not designed to be used publicly.

#[cfg(any(feature = "struct", feature = "protobuf"))]
use proc_macro::TokenStream;

#[cfg(feature = "struct")]
mod r#struct;

#[cfg(feature = "protobuf")]
mod proto;

#[cfg(feature = "struct")]
#[proc_macro_derive(StructData, attributes(structdata))]
pub fn derive_struct_data(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    r#struct::expand_derive_struct_data(input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

#[cfg(feature = "protobuf")]
#[proc_macro_derive(ProtobufData, attributes(protobuf))]
pub fn derive_protobuf_data(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    proto::expand_derive_protobuf_data(input)
        .unwrap_or_else(syn::Error::into_compile_error)
        .into()
}

