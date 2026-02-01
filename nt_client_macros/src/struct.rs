use proc_macro2::{Span, TokenStream};
use quote::{quote, quote_spanned};
use syn::{Attribute, Data, DeriveInput, Expr, Lit, LitStr, Meta, Token, punctuated::Punctuated, spanned::Spanned};

pub fn expand_derive_struct_data(input: DeriveInput) -> syn::Result<TokenStream> {
    let StructContainerAttributes { type_name, schema } = StructContainerAttributes::from_attrs(&input.attrs)?;
    let DeriveInput { ident, data, .. } = input;
    let data = match data {
        Data::Struct(data) => data,
        Data::Enum(data) => return Err(syn::Error::new_spanned(data.enum_token, "enums are not supported")),
        Data::Union(data) => return Err(syn::Error::new_spanned(data.union_token, "unions are not supported")),
    };

    let fields = data.fields.into_iter()
        .map(|field| {
            let span = field.span();
            let attrs = StructFieldAttributes::from_attrs(&field.attrs)?;
            let ident = field.ident
                .ok_or_else(|| syn::Error::new(span, "struct fields must be namd"))?;
            Ok((ident, field.ty, attrs))
        })
        .collect::<syn::Result<Vec<_>>>()?;

    let field_idents = fields.iter().map(|(ident, _, _)| quote! { #ident });
    let pack = fields.iter()
        .map(|(ident, ty, attrs)| {
            if attrs.nested {
                quote_spanned!(ident.span()=> <#ty as ::nt_client::r#struct::StructData>::pack(self.#ident, buf);)
            } else {
                quote_spanned!(ident.span()=> <#ty as ::nt_client::r#struct::byte::BytePrimitive>::write(self.#ident, buf);)
            }
        });
    let unpack = fields.iter()
        .map(|(ident, ty, attrs)| {
            if attrs.nested {
                quote_spanned! {ident.span()=>
                    let #ident = <#ty as ::nt_client::r#struct::StructData>::unpack(read)?;
                }
            } else {
                quote_spanned! {ident.span()=>
                    let #ident = <#ty as ::nt_client::r#struct::byte::BytePrimitive>::read(read)?;
                }
            }
        });

    Ok(quote! {
        impl ::nt_client::r#struct::StructData for #ident {
            fn struct_type_name() -> ::std::string::String {
                <str as ::std::string::ToString>::to_string(#type_name)
            }

            fn schema() -> ::nt_client::r#struct::StructSchema {
                ::nt_client::r#struct::StructSchema(
                    <str as ::std::string::ToString>::to_string(#schema)
                )
            }

            fn pack(self, buf: &mut ::nt_client::r#struct::byte::ByteBuffer) {
                #(#pack)*
            }

            fn unpack(read: &mut ::nt_client::r#struct::byte::ByteReader) -> ::std::option::Option<Self> {
                #(#unpack)*
                Some(Self { #(#field_idents),* })
            }
        }
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct StructContainerAttributes {
    type_name: LitStr,
    schema: LitStr,
}

impl StructContainerAttributes {
    fn from_attrs(attrs: &[Attribute]) -> syn::Result<Self> {
        let mut type_name = None;
        let mut schema = None;

        for attr in attrs {
            if !attr.path().is_ident("structdata") {
                continue;
            }

            let nested = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
            for meta in nested {
                match meta {
                    Meta::NameValue(meta) if meta.path.is_ident("type_name") => {
                        let Expr::Lit(value) = meta.value else {
                            return Err(syn::Error::new_spanned(meta.value, "expected `type_name` to be a string"));
                        };
                        let Lit::Str(value) = value.lit else {
                            return Err(syn::Error::new_spanned(value, "expected `type_name` to be a string"));
                        };

                        type_name = Some(value);
                    },
                    Meta::NameValue(meta) if meta.path.is_ident("schema") => {
                        let Expr::Lit(value) = meta.value else {
                            return Err(syn::Error::new_spanned(meta.value, "expected `schema` to be a string"));
                        };
                        let Lit::Str(value) = value.lit else {
                            return Err(syn::Error::new_spanned(value, "expected `schema` to be a string"));
                        };

                        schema = Some(value);
                    },
                    _ => return Err(syn::Error::new_spanned(meta, "unknown meta attribute")),
                }
            }
        }

        Ok(Self {
            type_name: type_name.ok_or_else(|| syn::Error::new(Span::call_site(), "missing structdata attribute `type_name`"))?,
            schema: schema.ok_or_else(|| syn::Error::new(Span::call_site(), "missing structdata attribute `schema`"))?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StructFieldAttributes {
    nested: bool,
}

impl StructFieldAttributes {
    fn from_attrs(attrs: &[Attribute]) -> syn::Result<Self> {
        let mut this = Self {
            nested: false,
        };

        for attr in attrs {
            if !attr.path().is_ident("structdata") {
                continue;
            }

            let nested = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
            for meta in nested {
                match meta {
                    Meta::Path(path) if path.is_ident("nested") => {
                        this.nested = true;
                    },
                    _ => return Err(syn::Error::new_spanned(meta, "unknown meta attribute")),
                }
            }
        }

        Ok(this)
    }
}

