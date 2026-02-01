use proc_macro2::{Span, TokenStream};
use quote::{quote, quote_spanned};
use syn::{Attribute, Data, DeriveInput, Ident, Meta, Token, Type, punctuated::Punctuated, spanned::Spanned};

pub fn expand_derive_protobuf_data(input: DeriveInput) -> syn::Result<TokenStream> {
    let ProtobufContainerAttributes { from } = ProtobufContainerAttributes::from_attrs(&input.attrs)?;
    let DeriveInput { ident, data, .. } = input;
    let data = match data {
        Data::Struct(data) => data,
        Data::Enum(data) => return Err(syn::Error::new_spanned(data.enum_token, "enums are not supported")),
        Data::Union(data) => return Err(syn::Error::new_spanned(data.union_token, "unions are not supported")),
    };

    let fields = data.fields.into_iter()
        .map(|field| {
            let span = field.span();
            let attrs = ProtobufFieldAttributes::from_attrs(&field.attrs)?;
            let ident = field.ident
                .ok_or_else(|| syn::Error::new(span, "struct fields must be namd"))?;
            Ok((ident, field.ty, attrs))
        })
        .collect::<syn::Result<Vec<_>>>()?;

    let from_proto = fields.iter()
        .map(|(ident, ty, attrs)| {
            let proto_field = attrs.field.as_ref().unwrap_or(ident);
            if attrs.nested {
                quote_spanned! {ident.span()=>
                    #ident: <#ty as ::nt_client::protobuf::ProtobufData>::from_proto(proto.#proto_field.into_option()?)?
                }
            } else {
                quote_spanned!(ident.span()=> #ident: proto.#proto_field)
            }
        });

    let into_proto = fields.iter()
        .map(|(ident, ty, attrs)| {
            let proto_field = attrs.field.as_ref().unwrap_or(ident);
            if attrs.nested {
                quote_spanned! {ident.span()=>
                    proto.#proto_field = ::protobuf::MessageField::some(<#ty as ::nt_client::protobuf::ProtobufData>::into_proto(self.#ident));
                }
            } else {
                quote_spanned!(ident.span()=> proto.#proto_field = self.#ident;)
            }
        });

    Ok(quote! {
        impl ::nt_client::protobuf::ProtobufData for #ident {
            type Proto = #from;

            fn from_proto(proto: Self::Proto) -> ::std::option::Option<Self> {
                ::std::option::Option::Some(Self { #(#from_proto),* })
            }

            fn into_proto(self) -> Self::Proto {
                let mut proto = Self::Proto::new();
                #(#into_proto)*
                proto
            }
        }
    })
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ProtobufContainerAttributes {
    from: Type,
}

impl ProtobufContainerAttributes {
    fn from_attrs(attrs: &[Attribute]) -> syn::Result<Self> {
        let mut from = None;

        for attr in attrs {
            if !attr.path().is_ident("protobuf") {
                continue;
            }

            let nested = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
            for meta in nested {
                match meta {
                    Meta::List(meta) if meta.path.is_ident("from") => {
                        from = Some(meta.parse_args()?);
                    },
                    _ => return Err(syn::Error::new_spanned(meta, "unknown meta attribute")),
                }
            }
        }

        Ok(Self {
            from: from.ok_or_else(|| syn::Error::new(Span::call_site(), "missing protobuf attribute `from`"))?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ProtobufFieldAttributes {
    field: Option<Ident>,
    nested: bool,
}

impl ProtobufFieldAttributes {
    fn from_attrs(attrs: &[Attribute]) -> syn::Result<Self> {
        let mut this = Self {
            field: None,
            nested: false,
        };

        for attr in attrs {
            if !attr.path().is_ident("protobuf") {
                continue;
            }

            let nested = attr.parse_args_with(Punctuated::<Meta, Token![,]>::parse_terminated)?;
            for meta in nested {
                match meta {
                    Meta::List(meta) if meta.path.is_ident("field") => {
                        this.field = Some(meta.parse_args()?);
                    },
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

